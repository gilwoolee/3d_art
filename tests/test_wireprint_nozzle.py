"""Nozzle-aware slope control uses actual chords and preserves the source shape."""


from collections import defaultdict

import numpy as np
import pytest
import trimesh

from pen_art.abstraction.wireprint import mesh_framework


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
    # across a square corner is the shortest, and it sets the layer height.
    assert metadata["effective_layer_height_m"] == pytest.approx(0.008)
    rising = [
        nodes[edge["end"]] - nodes[edge["start"]]
        for edges in groups.values()
        for edge in edges
        if nodes[edge["end"]][2] > nodes[edge["start"]][2]
    ]
    assert len(rising) == 13 * len(groups)
    # The end slices are cut a micron inside the mesh, hence the allowance.
    assert max(np.linalg.norm(delta[:2]) for delta in rising) < 2e-6
    assert metadata["actual_maximum_descent_angle_deg"] == pytest.approx(angles.max())
    assert metadata["descending_member_count"] == len(angles)
    assert metadata["closing_diagonal_omitted"]
    assert metadata["omitted_closing_diagonals"] == len(groups)
    # 13 pairs at 24.6mm spacing survive; each final downward leg is omitted.
    assert {len(edges) for edges in groups.values()} == {25}
    for edges in groups.values():
        final = max(edges, key=lambda edge: edge["source_interval"][-1])
        assert nodes[final["end"]][2] > nodes[final["start"]][2]
    points = np.asarray(list(nodes.values()))
    np.testing.assert_allclose([points.min(axis=0), points.max(axis=0)], mesh.bounds, atol=1e-9)
    contour_lengths = defaultdict(float)
    for edge in graph["edges"]:
        if edge["parent_curve"].startswith("contour-"):
            contour_lengths[edge["parent_curve"]] += edge["length_m"]
    assert all(length == pytest.approx(0.32, abs=5e-6) for length in contour_lengths.values())
    solidified_fork_drops_the_fin_and_keeps_printable_layers()


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
