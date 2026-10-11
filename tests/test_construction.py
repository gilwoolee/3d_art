import pytest

from pen_art.construction import construction_preview, split_stroke
from pen_art.graph import straight_edge
from pen_art.strokes import pen_strokes


def test_open_contour_starts_from_an_interior_anchor_without_manual_connection():
    nodes = {
        name: {"id": name, "position": point, "ground": name == "foot"}
        for name, point in {
            "foot": [0, 0, 0],
            "joint": [0, 0, 0.01],
            "left": [-0.01, 0, 0.02],
            "right": [0.01, 0, 0.02],
        }.items()
    }
    post = straight_edge("foot", "joint", nodes, "post", "support", 0.001)
    left = straight_edge("left", "joint", nodes, "left-half", "artistic", 0.001)
    right = straight_edge("joint", "right", nodes, "right-half", "artistic", 0.001)
    left.update(parent_curve="arch", source_interval=[0, 1])
    right.update(parent_curve="arch", source_interval=[1, 2])
    graph = {
        "nodes": list(nodes.values()),
        "edges": [post, left, right],
        "checks": {"material_connected": True},
    }
    strokes = pen_strokes(graph)
    arch = next(s for s in strokes["curves"] if s["parent_curve"] == "arch")
    pieces = split_stroke(arch, 1)
    assert sum(p["length_m"] for p in pieces) == pytest.approx(arch["length_m"])
    plan = construction_preview(graph, strokes)
    assert plan["status"] == "GEOMETRY_PREVIEW"
    assert plan["all_members_covered_once"]
    assert len(plan["actions"]) == 3
    assert plan["actions"][0]["start"] == "foot"
    assert plan["actions"][1]["start"] == plan["actions"][2]["start"] == "joint"
    assert plan["actions"][1]["depends_on"] == [0]
    assert plan["robot_feasible"] is None
    assert plan["pen_feasible"] is None
