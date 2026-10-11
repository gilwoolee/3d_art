import numpy as np
import pytest

from pen_art.graph import straight_edge
from pen_art.process import evaluate_action
from pen_art.strokes import orient_stroke, pen_strokes


def square():
    nodes = {
        str(i): {"id": str(i), "position": p, "ground": False}
        for i, p in enumerate([[0, 0, 0.01], [0.01, 0, 0.01], [0.01, 0.01, 0.01], [0, 0.01, 0.01]])
    }
    edges = []
    for i in range(4):
        edge = straight_edge(str(i), str((i + 1) % 4), nodes, str(i), "artistic", 0.001)
        edge.update(parent_curve="square", source_interval=[i, i + 1])
        edges.append(edge)
    return {"nodes": list(nodes.values()), "edges": edges}


def test_junctions_do_not_force_feed_stops():
    graph = square()
    result = pen_strokes(graph)
    assert result["material_members"] == 4
    assert result["proposed_starts"] == 1
    assert result["total_length_m"] == pytest.approx(0.04)
    curve = result["curves"][0]
    assert curve["closed"]
    assert len(curve["junction_visits"]) == 5
    assert curve["segments"] == [s for e in graph["edges"] for s in e["segments"]]


def test_closed_contour_can_start_at_a_supported_junction_in_either_direction():
    stroke = pen_strokes(square())["curves"][0]
    for reverse in (False, True):
        action = orient_stroke(stroke, reverse=reverse, start_visit=2)
        assert action["start"] == action["end"] == "2"
        assert set(action["members"]) == set(stroke["members"])
        np.testing.assert_allclose(action["segments"][0][0], [0.01, 0.01, 0.01])
        assert np.all(np.diff([v["distance_m"] for v in action["junction_visits"]]) >= 0)
        assert action["junction_visits"][-1]["distance_m"] == pytest.approx(0.04)


def test_free_span_resets_only_at_preexisting_real_junctions():
    graph = square()
    stroke = pen_strokes(graph)["curves"][0]
    nodes = {n["id"]: n for n in graph["nodes"]}
    profile = {"calibrated": True, "strand_diameter_m": 0.001, "maximum_free_tip_length_m": 0.015}
    unsupported = evaluate_action(stroke, nodes, profile, {"0"})
    assert "UNSUPPORTED_LENGTH" in unsupported["failures"]
    supported = evaluate_action(stroke, nodes, profile, set(nodes))
    assert supported["features"]["maximum_unanchored_length_m"] == pytest.approx(0.01)
    assert supported["modeled_pass"]
