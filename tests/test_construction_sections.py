"""Small regression checks for temporal/source errors that visual replay can hide."""


from copy import deepcopy

import pytest

from pen_art.planning.sections import coverage_audit, replay_sections


def test_interleaved_coverage_cooling_and_stage_boundaries():
    graph = {"nodes": [{"id": "bed", "ground": True}, {"id": "join", "ground": False}]}
    rows = [
        {
            "id": "a0",
            "members": ["a0"],
            "parent_curve": "a",
            "source_interval": [0, 0.5],
            "stage": "framework",
            "start": "bed",
            "end": "join",
            "start_anchor": "bed",
            "start_s": 0,
            "end_s": 1,
        },
        {
            "id": "b",
            "members": ["b"],
            "parent_curve": "b",
            "source_interval": [0, 1],
            "stage": "framework",
            "start": "join",
            "end": "other",
            "start_anchor": "join",
            "start_s": 3,
            "end_s": 4,
        },
        {
            "id": "a1",
            "members": ["a1"],
            "parent_curve": "a",
            "source_interval": [0.5, 1],
            "stage": "framework",
            "start": "join",
            "end": "end",
            "start_anchor": "join",
            "start_s": 6,
            "end_s": 7,
        },
    ]
    assert coverage_audit(rows, rows)["sources"] == 2
    assert replay_sections(rows, graph)["completed_sections"] == 3
    for interval, message in (([0.6, 1], "gap"), ([0.4, 1], "Overlapping")):
        broken = deepcopy(rows)
        broken[2]["source_interval"] = interval
        with pytest.raises(ValueError, match=message):
            coverage_audit(broken, rows)
    broken = deepcopy(rows)
    broken[1].update(start_s=1.5, end_s=2.5)
    with pytest.raises(ValueError, match="cooling"):
        replay_sections(broken, graph)
    broken = deepcopy(rows)
    broken[1]["stage"] = "artistic"
    with pytest.raises(ValueError, match="framework completion"):
        replay_sections(broken, graph)
    broken = deepcopy(rows)
    broken[2].update(stage="artistic", start_s=4, end_s=5)
    with pytest.raises(ValueError, match="framework cooling"):
        replay_sections(broken, graph)

    # A grouped closed contour deposits its internal physical junctions too.
    closed = deepcopy(rows[:2])
    closed[0].update(
        end="bed",
        closed=True,
        junction_visits=[
            {"node": "bed"},
            {"node": "join"},
            {"node": "bed"},
        ],
    )
    assert replay_sections(closed, graph)["completed_sections"] == 2


def test_connected_zigzag_keeps_integer_vertices_and_exact_source_gaps():
    from pen_art.graph import straight_edge
    from pen_art.planning.sections import section_actions
    from pen_art.provenance import audit_source_provenance

    nodes = {
        str(index): {"id": str(index), "position": point, "ground": point[2] == 0}
        for index, point in enumerate(([0, 0, 0], [0.01, 0, 0.01], [0.02, 0, 0], [0.03, 0, 0.01]))
    }
    edges = []
    for index in range(3):
        edge = straight_edge(str(index), str(index + 1), nodes, f"leg-{index}", "framework", 0.001)
        edge.update(
            parent_curve="zigzag-1",
            source_interval=[index, index + 1],
            origin="wireprint",
            stage="framework",
        )
        edges.append(edge)
    # A seam correction changes provenance without breaking the actual path.
    edges[0]["source_interval"][1] -= 0.000003
    graph = {"nodes": list(nodes.values()), "edges": edges}
    actions = section_actions(graph)["actions"]
    assert len(actions) == 1
    assert actions[0]["members"] == [edge["id"] for edge in edges]
    assert actions[0]["segments"] == [s for edge in edges for s in edge["segments"]]
    assert len(actions[0]["junction_visits"]) == 4
    assert audit_source_provenance(actions, graph)["physical_members"] == 3
    broken = deepcopy(actions)
    broken[0].pop("source_intervals")
    with pytest.raises(ValueError, match="gap or extension"):
        audit_source_provenance(broken, graph)
    disconnected = deepcopy(graph)
    disconnected["nodes"].append({**nodes["1"], "id": "elsewhere"})
    disconnected["edges"][1]["start"] = "elsewhere"
    assert len(section_actions(disconnected)["actions"]) == 2
    restarted = deepcopy(graph)
    restarted["edges"][0]["stop_after"] = True
    assert len(section_actions(restarted)["actions"]) == 2


def test_progressive_structure_checks_actual_member_order_without_tip_support(monkeypatch):
    from types import SimpleNamespace

    from pen_art.graph import straight_edge
    from pen_art.planning.section_build import SectionBuild
    from pen_art.planning.sections import section_actions

    nodes = {
        name: {"id": name, "position": point, "ground": name == "a"}
        for name, point in (("a", [0, 0, 0]), ("b", [0.01, 0, 0]), ("c", [0.01, 0, 0.01]))
    }
    edges = []
    for index, (start, end) in enumerate((("a", "b"), ("b", "c"), ("c", "a"))):
        edge = straight_edge(start, end, nodes, str(index), "framework", 0.001)
        edge.update(parent_curve="loop", source_interval=[index, index + 1], stage="framework")
        edges.append(edge)
    graph = {"nodes": list(nodes.values()), "edges": edges}
    action = section_actions(graph)["actions"][0]
    build = object.__new__(SectionBuild)
    build.planner = SimpleNamespace(graph=graph)
    build.actions, build.material = {action["id"]: action}, {}
    calls = []

    def screen(graph, material, *, edge_ids):
        calls.append(set(edge_ids))
        # Final closure is stable; the forward intermediate prefix is not.
        return {"modeled_pass": edge_ids != {"0", "1"}}

    monkeypatch.setattr("pen_art.structure.analyze_structure", screen)
    rejected = build.progressive_structure(action, {"reverse": False}, set())
    assert not rejected["modeled_pass"] and rejected["rejected_member"] == "1"
    assert calls == [{"0"}, {"0", "1"}]
    assert not rejected["growing_tip_constraint"]
    calls.clear()
    accepted = build.progressive_structure(action, {"reverse": True}, set())
    assert accepted["modeled_pass"]
    assert calls == [{"2"}, {"2", "1"}, {"0", "1", "2"}]
    calls.clear()
    assert build.progressive_structure(action, {"reverse": True}, set()) is accepted
    assert calls == []  # Repeated robot postures reuse only matching material prefixes.
