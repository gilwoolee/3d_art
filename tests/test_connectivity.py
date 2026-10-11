import networkx as nx
import numpy as np
import pytest

from pen_art.curves import curve_coordinate_bounds
from pen_art.graph import build_graph, topology


@pytest.fixture
def settings():
    return {
        "junction_tolerance_m": 0.0001,
        "flatten_tolerance_m": 0.00001,
        "sampling_chord_m": 0.002,
        "minimum_crossing_angle_deg": 15,
        "strand_diameter_m": 0.001,
        "ground_tolerance_m": 0.0001,
        "maximum_connector_m": 0.1,
    }


def line(identity, a, b):
    a, b = np.array(a), np.array(b)
    return {
        "id": identity,
        "origin": "test",
        "segments": [
            [a.tolist(), (a + (b - a) / 3).tolist(), (a + 2 * (b - a) / 3).tolist(), b.tolist()]
        ],
    }


def graph_of(curves, settings):
    return build_graph({"method": "test", "curves": curves}, settings)


def test_true_t_junction_splits_curve_without_losing_geometry(settings):
    graph = graph_of(
        [line("a", [-0.01, 0, 0.02], [0.01, 0, 0.02]), line("b", [0, 0, 0.02], [0, 0.01, 0.02])],
        settings,
    )
    artistic = [e for e in graph["edges"] if e["role"] == "artistic"]
    assert len(artistic) == 3
    assert sum(e["length_m"] for e in artistic) == pytest.approx(0.03)
    endpoints = [node for edge in artistic for node in (edge["start"], edge["end"])]
    assert max(endpoints.count(node) for node in endpoints) == 3


def test_projected_crossing_is_not_a_physical_junction(settings):
    graph = graph_of(
        [
            line("a", [-0.01, 0, 0.02], [0.01, 0, 0.02]),
            line("b", [0, -0.01, 0.03], [0, 0.01, 0.03]),
        ],
        settings,
    )
    artistic = [e for e in graph["edges"] if e["role"] == "artistic"]
    assert graph["metrics"]["input_components"] == 2
    assert {artistic[0]["start"], artistic[0]["end"]}.isdisjoint(
        {artistic[1]["start"], artistic[1]["end"]}
    )
    assert graph["checks"]["material_connected"]
    assert any(e["role"] != "artistic" for e in graph["edges"])


def test_connection_excludes_the_bed_from_material_graph(settings):
    graph = graph_of(
        [line("a", [0, 0, 0.01], [0.01, 0, 0.01]), line("b", [0.03, 0, 0.01], [0.04, 0, 0.01])],
        settings,
    )
    assert nx.is_connected(topology(graph))
    assert any(edge["role"] == "base" for edge in graph["edges"])
    assert not any(node["id"] == "bed" for node in graph["nodes"])
    nodes = {node["id"]: np.array(node["position"]) for node in graph["nodes"]}
    for edge in graph["edges"]:
        np.testing.assert_allclose(edge["segments"][0][0], nodes[edge["start"]], atol=1e-10)
        np.testing.assert_allclose(edge["segments"][-1][-1], nodes[edge["end"]], atol=1e-10)


def test_bed_placement_uses_interior_cubic_extremum(settings):
    curve = {
        "id": "dip",
        "origin": "test",
        "segments": [[[0, 0, 0], [0.01, 0, -0.004], [0.02, 0, -0.004], [0.03, 0, 0]]],
    }
    graph = graph_of([curve], settings)
    assert graph["candidate_to_build"][2][3] == pytest.approx(0.003)
    assert min(curve_coordinate_bounds(e)[0, 2] for e in graph["edges"]) >= -1e-12
    assert curve["segments"][0][1][2] == -0.004  # preserve candidate artifact
    assert graph["checks"]["material_connected"]


def test_curve_fit_does_not_overshoot_between_uneven_samples():
    from pen_art.curves import fit_polyline

    points = np.array([[0, 0, 0], [0.0001, 0, 0.0001], [0.0002, 0, 0], [0.1, 0, 0]])
    tolerance = 0.00002
    curve = {"segments": fit_polyline(points, tolerance)}
    bounds = curve_coordinate_bounds(curve)
    assert bounds[0, 2] >= -tolerance
    assert bounds[1, 2] <= 0.0001 + tolerance
    assert len(curve["segments"]) > 1
