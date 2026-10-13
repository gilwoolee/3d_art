"""Try depositing a witnessed artistic occluder later, without changing geometry."""

from pathlib import Path

from pen_art.cell import prepare_cell
from pen_art.io import read_json, sha256, write_json
from pen_art.planning.section_build import prepare_candidates, write_section_artifacts
from pen_art.provenance import audit_source_provenance
from pen_art.structure import analyze_structure

source = Path("runs/palace_hybrid_art_restarts_source")
run = Path("runs/palace_hybrid_art_order_v1")
evidence = Path("research/blondie/palace_art_blockers.json")
assert not run.exists()
report = read_json(evidence)
graph = read_json(source / "stroke_graph.json")
construction = read_json(source / "construction_plan.json")
material = read_json(source / "fabrication_config.json")["structure"]
assert (
    sha256(source / "stroke_graph.json")
    == report["inputs"]["runs/palace_restart_dense_axis_v1/stroke_graph.json"]
)
blocker = report["blockers"][0]["section"]
assert report["blockers"][0]["count"] == report["framework_clear_orientations"] > 0
row = next(r for r in construction["actions"] if r["id"] == blocker)
assert row["stage"] == "artistic"
original_priority = row["priority"]
row["priority"] = (
    max(r["priority"] for r in construction["actions"] if r["stage"] == "artistic") + 1
)
members = {m for r in construction["actions"] if r["id"] != blocker for m in r["members"]}
screen = analyze_structure(graph, material, edge_ids=members)
assert screen["modeled_pass"], "Removing this blocker would fail the predecessor structural screen"
audit_source_provenance(construction["actions"], graph)
prepare_cell(Path("research/planning_cell/cell.yaml"), run, source)
write_json(run / "construction_plan.json", construction)
write_section_artifacts(run, graph, construction, preserve_graph=True)
prepare_candidates(run)
assert sha256(run / "stroke_graph.json") == sha256(source / "stroke_graph.json")
original = {r["id"]: r for r in read_json(source / "construction_plan.json")["actions"]}
for candidate in construction["actions"]:
    assert {k: v for k, v in candidate.items() if k != "priority"} == {
        k: v for k, v in original[candidate["id"]].items() if k != "priority"
    }
    if candidate["id"] != blocker:
        assert candidate["priority"] == original[candidate["id"]]["priority"]
receipt = {
    "status": "ORDER_ONLY_TRIAL_PREPARED",
    "source": str(source),
    "run": str(run),
    "promoted_last_section": blocker,
    "target_section": report["target"],
    "old_priority": original_priority,
    "new_priority": row["priority"],
    "predecessor_displacement_m": screen["maximum_displacement_m"],
    "sections": len(construction["actions"]),
    "geometry_unchanged": True,
    "simulation_only": True,
    "scope": (
        "A finite geometric occlusion witness motivates this priority proposal; "
        "full robot validation remains required."
    ),
    "inputs": {
        str(p): sha256(p)
        for p in [
            source / "stroke_graph.json",
            source / "construction_plan.json",
            run / "stroke_graph.json",
            run / "construction_plan.json",
            evidence,
        ]
    },
}
write_json(Path("research/blondie/palace_art_order.json"), receipt)
write_json(run / "trial_reason.json", receipt)
write_json(
    run / "section_build.json",
    {
        "status": "PREPARED_PENDING_ROBOT_VALIDATION",
        "completed_sections": 0,
        "required_sections": len(construction["actions"]),
        "best_prefix": [],
        "simulation_only": True,
    },
)
print(
    blocker,
    "last-priority",
    row["priority"],
    "predecessor screen passes",
    screen["maximum_displacement_m"],
    flush=True,
)
