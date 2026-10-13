"""Revalidate a real Palace prefix and reject its invariant structural failure."""

from pathlib import Path

from pen_art.io import read_json, sha256, write_json
from pen_art.planning.section_build import SectionBuild

run = Path("runs/palace_hybrid_spaced_1200um_v1")
build = SectionBuild(run, Path("research/planning_cell/cell.yaml"), 600)
nodes = {row["id"]: row for row in build.planner.graph["nodes"]}
order = sorted(
    build.actions.values(),
    key=lambda row: (
        row["stage"] != "framework",
        row["priority"],
        min(nodes[row[end]]["position"][2] for end in ("start", "end")),
        row["id"],
    ),
)
assert build.forward(order) is None
state = read_json(run / "section_build.json")
assert state["status"] == "PROPOSED_ORDER_STRUCTURE_REJECTED"
assert state["rejected_section"] == "section-00524"
assert state["completed_sections"] == 6 and build.resumed_motions == 6
assert not build.gpu and not build.planner.attempts
ids = set(state["best_prefix"]) | {state["rejected_section"]}
screen = build.structure[frozenset(ids)]
assert not screen["modeled_pass"]
write_json(
    Path("research/blondie/fixed_order_rejection.json"),
    {
        "status": "PASSED",
        "run": str(run),
        "revalidated_motions": build.resumed_motions,
        "rejected_section": state["rejected_section"],
        "motion_retries_after_structural_rejection": 0,
        "structure": {
            key: screen[key]
            for key in ("modeled_pass", "maximum_displacement_m", "linear_model_valid")
            if key in screen
        },
        "scope": "Reject this fixed order; no claim of infeasibility for other orders.",
        "simulation_only": True,
        "inputs": {
            str(path): sha256(path)
            for path in (
                Path(__file__),
                Path("src/pen_art/planning/section_build.py"),
                run / "stroke_graph.json",
                run / "construction_plan.json",
                run / "section_build.json",
                run / "forward_checkpoint.json",
            )
        },
    },
)
print("Real prefix revalidated; structural order rejected without motion retries", flush=True)
