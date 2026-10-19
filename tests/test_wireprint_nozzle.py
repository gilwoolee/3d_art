"""Nozzle-aware slope control uses actual chords and preserves the source shape."""


from collections import defaultdict

import numpy as np
import pytest
import trimesh
import yaml

from pen_art.abstraction.wireprint import eulerize_frame, mesh_framework
from pen_art.graph import topology
from pen_art.planning.process_clock import source_vertex_schedule
from pen_art.planning.sections import section_actions
from pen_art.structure import analyze_structure


def cube():
    mesh = trimesh.creation.box(extents=[0.08, 0.08, 0.08])
    mesh.apply_translation([0, 0, 0.04])
    return mesh


def descending_angles(graph):
    nodes = {node["id"]: np.asarray(node["position"]) for node in graph["nodes"]}
    groups = defaultdict(list)
    angles = []
    for edge in graph["edges"]:
        if not edge["parent_curve"].startswith("zigzag-"):
            continue
        groups[edge["parent_curve"]].append(edge)
        delta = nodes[edge["end"]] - nodes[edge["start"]]
        if delta[2] < -1e-12:
            angles.append(np.degrees(np.arctan2(-delta[2], np.linalg.norm(delta[:2]))))
    return np.asarray(angles), groups, nodes


def test_slice_height_follows_the_nozzle_on_a_cube_and_a_solidified_fork():
    mesh = cube()
    graph = mesh_framework(mesh, maximum_descent_angle_deg=27)
    angles, groups, nodes = descending_angles(graph)
    assert angles.size and angles.max() <= 27 + 1e-8
    metadata = graph["generation"]
    assert metadata["requested_layer_height_m"] == 0.01
    # Each zigzag is a vertical riser and one slanted stroke down. The stroke
    # across a square corner would be too steep, so its foot stands further
    # round the loop and the layer keeps its requested height.
    assert metadata["effective_layer_height_m"] == pytest.approx(0.01)
    assert graph["stretched_diagonals"] == 29 and not metadata["descent_angle_refinements"]
    rising = [
        nodes[edge["end"]] - nodes[edge["start"]]
        for edges in groups.values()
        for edge in edges
        if nodes[edge["end"]][2] > nodes[edge["start"]][2]
    ]
    assert len(rising) == 13 + 12 * (len(groups) - 1)
    # The end slices are cut a micron inside the mesh, hence the allowance.
    assert max(np.linalg.norm(delta[:2]) for delta in rising) < 2e-6
    assert metadata["actual_maximum_descent_angle_deg"] == pytest.approx(angles.max())
    assert metadata["descending_member_count"] == len(angles)
    assert metadata["closing_diagonal_omitted"]
    assert metadata["omitted_closing_diagonals"] == len(groups)
    # 13 or 12 pairs survive; each final downward leg is omitted.
    assert {len(edges) for edges in groups.values()} == {25, 23}
    for edges in groups.values():
        final = max(edges, key=lambda edge: edge["source_interval"][-1])
        assert nodes[final["end"]][2] > nodes[final["start"]][2]
    # Each zigzag starts where the one below ended, so the pen can run from a
    # zigzag round its contour into the next.
    ordered = sorted(groups, key=lambda name: int(name.split("-")[1]))
    ends = {}
    for name in ordered:
        legs = sorted(groups[name], key=lambda edge: edge["source_interval"][0])
        ends[name] = (legs[0]["start"], legs[-1]["end"])
    assert all(ends[a][1] == ends[b][0] for a, b in zip(ordered[:-1], ordered[1:], strict=True))
    # Where equal spacing would make a diagonal too steep, its foot stands
    # further round the loop, rather than every layer being thinned.
    tower = trimesh.creation.cylinder(radius=0.0065, height=0.03, sections=48)
    tower.apply_translation([0, 0, 0.015])
    slim = mesh_framework(tower, layer_height=0.006, maximum_descent_angle_deg=27)
    assert slim["stretched_diagonals"] == 10
    assert slim["generation"]["effective_layer_height_m"] == pytest.approx(0.006)
    assert slim["generation"]["actual_maximum_descent_angle_deg"] <= 27
    # The base is a grid of lines on the bed that meet at shared points, drawn
    # before the outline they join.
    gridded = mesh_framework(mesh, maximum_descent_angle_deg=27, base_line_spacing=0.02)
    lines = [edge for edge in gridded["edges"] if edge["parent_curve"].startswith("base-")]
    assert gridded["base_lines"] == 8 and len(lines) == 40
    assert sum(edge["length_m"] for edge in lines) == pytest.approx(8 * 0.08, abs=1e-5)
    assert max(edge["order_hint"] for edge in lines) < min(
        edge["order_hint"] for edge in gridded["edges"] if edge not in lines
    )
    # A post too narrow for loops and zigzags is wound as a coil tower: one
    # strand climbing its own thickness per turn, drawn a layer at a time, with
    # touching turns recorded as fused.
    block = trimesh.creation.cylinder(radius=0.009, height=0.025, sections=32)
    block.apply_translation([0, 0, 0.0125])
    post = trimesh.creation.cylinder(radius=0.003, height=0.03, sections=24)
    post.apply_translation([0.002, 0, 0.035])
    wound = mesh_framework(
        trimesh.util.concatenate([block, post]),
        maximum_descent_angle_deg=27,
        coil_maximum_width=0.01,
    )
    coil = [edge for edge in wound["edges"] if edge["parent_curve"].startswith("coil-")]
    assert wound["coil_towers"] == {"count": 1, "turns": 30}
    assert {edge["parent_curve"] for edge in coil} == {"coil-0-3", "coil-0-4", "coil-0-5"}
    spots = {node["id"]: np.asarray(node["position"]) for node in wound["nodes"]}
    rise = [
        (spots[edge["end"]] - spots[edge["start"]])[2] / edge["length_m"] for edge in coil
    ]
    assert 0 < min(rise) and np.median(rise) < 0.1 and len(wound["fused_turns"]) > 300
    # A strand resting on the coil, touching it only where it is fused, is held up.
    top = spots[coil[-1]["end"]]
    rest = {
        **wound,
        "nodes": wound["nodes"] + [
            {"id": "rest-a", "position": (top + [0, 0, 0.001]).tolist(), "ground": False},
            {"id": "rest-b", "position": (top + [0.004, 0, 0.001]).tolist(), "ground": False},
        ],
        "edges": wound["edges"] + [{
            "id": "rest", "start": "rest-a", "end": "rest-b", "diameter_m": 0.001,
            "segments": [np.linspace(top + [0, 0, 0.001], top + [0.004, 0, 0.001], 4).tolist()],
        }],
    }
    material = yaml.safe_load(open("configs/fabrication.yaml"))["structure"]
    assert analyze_structure(rest, material).get("failure") == "UNSUPPORTED_COMPONENT"
    rest["fused_turns"] = wound["fused_turns"] + [["rest-a", coil[-1]["end"]]]
    assert analyze_structure(rest, material).get("failure") != "UNSUPPORTED_COMPONENT"
    # A 13 mm pillar at 8 mm layers fits only two diagonals per loop within the
    # descent limit; it is sliced thinner so every loop carries four risers.
    pillar = mesh_framework(
        trimesh.creation.cylinder(radius=0.0065, height=0.04, sections=48),
        layer_height=0.008,
        maximum_descent_angle_deg=27,
        minimum_connectors=4,
    )
    assert max(pillar["generation"]["layer_heights_m"]) < 0.006
    at = {node["id"]: node["position"][2] for node in pillar["nodes"]}
    risers = defaultdict(int)
    for edge in pillar["edges"]:
        if edge["parent_curve"].startswith("zigzag-"):
            risers[round(min(at[edge["start"]], at[edge["end"]]), 4)] += 1
    assert len(risers) >= 6 and min(risers.values()) >= 5
    assert not mesh_framework(mesh, maximum_descent_angle_deg=27, coil_maximum_width=0.01)[
        "coil_towers"
    ]["count"]
    hints = {edge["parent_curve"]: edge["order_hint"] for edge in graph["edges"]}
    drawn = sorted(hints, key=hints.get)
    assert [name.split("-")[0] for name in drawn[:5]] == [
        "contour", "zigzag", "contour", "zigzag", "contour"
    ]
    points = np.asarray(list(nodes.values()))
    np.testing.assert_allclose([points.min(axis=0), points.max(axis=0)], mesh.bounds, atol=1e-9)
    contour_lengths = defaultdict(float)
    for edge in graph["edges"]:
        if edge["parent_curve"].startswith("contour-"):
            contour_lengths[edge["parent_curve"]] += edge["length_m"]
    assert all(length == pytest.approx(0.32, abs=5e-6) for length in contour_lengths.values())
    solidified_fork_drops_the_fin_and_keeps_printable_layers()

    # Connector wire pairs the odd nodes so the whole frame is one trail.
    graph = mesh_framework(cube(), maximum_descent_angle_deg=27, base_line_spacing=0.02)
    structure = yaml.safe_load(open("configs/fabrication.yaml"))["structure"]
    graph = eulerize_frame(graph, budget_m=10.0, structure=structure)
    assert sum(degree % 2 for _, degree in topology(graph).degree()) <= 2
    info = graph["generation"]["eulerization"]
    assert info["odd_nodes_before"] == 18 and info["minimum_trails_before"] == 9
    assert info["minimum_trails_after"] == 1
    assert 0 < info["added_m"] < 0.5
    connectors = [edge for edge in graph["edges"] if edge.get("origin") == "euler_connector"]
    assert connectors and all(clear_of_other_wire(edge, graph) for edge in connectors)
    # The bed is drawn as one stroke that ends where the walls begin.
    actions = section_actions(graph)["actions"]
    bed = [row for row in actions if row["parent_curve"] == "base-trail"]
    assert len(bed) == 1 and bed[0]["closed"]
    walls = [row for row in actions if row["parent_curve"].startswith("zigzag-")]
    assert any(row["start"] == bed[0]["end"] for row in walls)
    # The bed trail passes base-grid crossings twice; every corner is still
    # found in drawing order, not at a later pass through the same point,
    # even when the robot reaches each point only to within microns.
    path = np.asarray([bed[0]["segments"][0][0]] + [piece[-1] for piece in bed[0]["segments"]])
    path += np.random.default_rng(0).normal(0, 2e-6, path.shape)
    breaks, _, _ = source_vertex_schedule(bed[0], {}, path, np.eye(4), 0.0)
    assert breaks == sorted(breaks) and len(breaks) > 8


def clear_of_other_wire(connector, graph, margin=0.003):
    """No point of the connector away from its ends lies on another wire."""
    nodes = {node["id"]: np.asarray(node["position"]) for node in graph["nodes"]}
    ends = nodes[connector["start"]], nodes[connector["end"]]
    points = [np.asarray(point) for segment in connector["segments"] for point in segment]
    points = [p for p in points if min(np.linalg.norm(p - end) for end in ends) > margin]
    for edge in graph["edges"]:
        if edge is connector:
            continue
        for segment in edge["segments"]:
            a, b = np.asarray(segment[0]), np.asarray(segment[-1])
            for p in points:
                t = np.clip(np.dot(p - a, b - a) / max(np.dot(b - a, b - a), 1e-18), 0, 1)
                if np.linalg.norm(p - (a + t * (b - a))) < connector["diameter_m"]:
                    return False
    return True


def solidified_fork_drops_the_fin_and_keeps_printable_layers():
    from pen_art.abstraction.solidify import solidify

    def part(shape, offset):
        shape.apply_translation(offset)
        return shape

    # Overlapping closed parts: a base, two posts that fork from it, a 1 mm fin.
    source = trimesh.util.concatenate(
        [
            part(trimesh.creation.box(extents=[0.05, 0.02, 0.01]), [0, 0, 0.005]),
            part(trimesh.creation.cylinder(radius=0.007, height=0.03), [-0.015, 0, 0.022]),
            part(trimesh.creation.cylinder(radius=0.007, height=0.03), [0.015, 0, 0.022]),
            part(trimesh.creation.box(extents=[0.001, 0.02, 0.02]), [0, 0, 0.018]),
        ]
    )
    solid, report = solidify(source, 0.006)
    assert solid.is_watertight
    assert report["surface_shift_m"]["p95"] < 0.0015
    # The fin is gone: nothing remains above the base between the posts.
    assert not solid.contains([[0.0, 0.0, 0.02]])[0] and solid.contains([[0.015, 0, 0.03]])[0]
    assert solid.bounds[0, 2] == pytest.approx(0.0, abs=1e-9)

    graph = mesh_framework(
        solid,
        layer_height=0.003,
        maximum_descent_angle_deg=27,
        slice_simplification=0.00025,
        minimum_contour=0.0045,
    )
    angles, _, _ = descending_angles(graph)
    assert angles.max() <= 27 + 1e-8
    events = [event["event"] for event in graph["contour_events"]]
    assert "split" in events and "unsupported_birth" not in events
    # Neither the fork nor the rounded tips may thin every layer of the object.
    assert graph["generation"]["effective_layer_height_m"] > 0.001
    assert graph["omitted_small_contours"] > 0
