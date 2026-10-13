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


def test_cube_reduces_slice_height_keeps_density_and_full_outline():
    mesh = cube()
    graph = mesh_framework(mesh, maximum_descent_angle_deg=27)
    angles, groups, nodes = descending_angles(graph)
    assert angles.size and angles.max() <= 27 + 1e-8
    metadata = graph["generation"]
    assert metadata["requested_layer_height_m"] == 0.01
    # Even the straight-leg formula is too tall around a square corner.
    assert metadata["effective_layer_height_m"] < 0.025 / 2 * np.tan(np.radians(27))
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
