"""Try an adjacent-leg order change with the exact original Palace graph."""

import shutil
from copy import deepcopy
from pathlib import Path

from pen_art.io import now, read_json, sha256, write_json
from pen_art.planning.section_build import SectionBuild, prepare_candidates, write_section_artifacts
from pen_art.provenance import audit_source_provenance

source = Path("runs/palace_hybrid_spatial_v1")
run = Path("runs/palace_hybrid_neighbor_order_v1")
assert not run.exists(), "Use a fresh run to preserve evidence"
construction = read_json(source / "construction_plan.json")
original = deepcopy(construction)
graph = read_json(source / "stroke_graph.json")
nodes = {row["id"]: row for row in graph["nodes"]}
order = sorted(
    construction["actions"],
    key=lambda row: (
        row["stage"] != "framework",
        row["priority"],
        min(nodes[row[end]]["position"][2] for end in ("start", "end")),
        row["id"],
    ),
)
index = next(i for i, row in enumerate(order) if row["id"] == "section-00004")
assert order[index + 1]["id"] == "section-00005"
order[index], order[index + 1] = order[index + 1], order[index]
for rank, row in enumerate(order):
    row["priority"] = rank
for old, new in zip(original["actions"], construction["actions"], strict=True):
    assert {k: v for k, v in old.items() if k != "priority"} == {
        k: v for k, v in new.items() if k != "priority"
    }
audit = audit_source_provenance(construction["actions"], graph)
shutil.copytree(
    source,
    run,
    ignore=shutil.ignore_patterns("axis_repairs", "section_build.json", ".*"),
)
metadata = read_json(run / "run.json")
metadata.update(id=run.name, updated_at=now())
write_json(run / "run.json", metadata)
checkpoint = read_json(run / "forward_checkpoint.json")
(run / "forward_checkpoint.json").rename(run / "imported_forward_checkpoint.json")
construction["proposed_order_note"] = (
    "Try section-00005 before section-00004; all physical curves and stages unchanged."
)
write_json(run / "construction_plan.json", construction)
write_section_artifacts(run, graph, construction, preserve_graph=True)
prepare_candidates(run)
build = SectionBuild(run, Path("research/planning_cell/cell.yaml"), 21600)
write_json(
    run / "forward_checkpoint.json",
    {
        "status": "PROPOSALS_REQUIRE_REVALIDATION",
        "binding": build.checkpoint_binding,
        "witnesses": checkpoint["witnesses"][:index],
        "imported_from": str(source),
        "note": "Only the unchanged preceding motions; each needs fresh validation.",
    },
)
write_json(
    run / "section_build.json",
    {
        "status": "PREPARED_PROPOSALS_PENDING_REVALIDATION",
        "completed_sections": 0,
        "required_sections": len(order),
        "best_prefix": [],
        "simulation_only": True,
    },
)
assert sha256(run / "stroke_graph.json") == sha256(source / "stroke_graph.json")
report = {
    "at": now(),
    "run": str(run),
    "source": str(source),
    "status": "UNCHANGED_GEOMETRY_ORDER_CANDIDATE",
    "changed_order": ["section-00005", "section-00004"],
    "imported_proposal_count": index,
    "order": [row["id"] for row in order],
    "source_provenance": audit,
    "simulation_only": True,
    "inputs": {
        str(path): sha256(path)
        for path in (
            source / "stroke_graph.json",
            source / "construction_plan.json",
            run / "stroke_graph.json",
            run / "construction_plan.json",
            run / "imported_forward_checkpoint.json",
        )
    },
}
write_json(run / "trial_reason.json", report)
write_json(Path("research/blondie/palace_neighbor_order.json"), report)
print("Prepared unchanged-geometry neighbor-order trial", len(order), "sections", flush=True)
