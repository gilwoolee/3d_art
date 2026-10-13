"""Test additional corrections on a saved Palace stroke without changing geometry."""

from pathlib import Path
from time import monotonic

import numpy as np

from pen_art.cell import prepare_cell
from pen_art.io import read_json, sha256, write_json
from pen_art.planning.axis_repair import repair_collision_witness
from pen_art.planning.dual_arm import ScheduledMotion
from pen_art.planning.section_build import SectionBuild, prepare_candidates

source = Path("runs/palace_hybrid_art_order_v1")
run = Path("runs/palace_collision_continuation_v1")
profile = Path("research/planning_cell/cell.yaml")
assert not run.exists(), "Preserve an existing diagnostic run"
snapshot_bytes = (source / "axis_repairs/arm_1_section-00602-part-5.json").read_bytes()
prepare_cell(profile, run, source)
prepare_candidates(run)
snapshot = run / "source_repair_snapshot.json"
snapshot.write_bytes(snapshot_bytes)
saved = read_json(snapshot)
build = SectionBuild(run, profile, 1800, gpu=False)
removed = {
    "section-00602-part-2",
    "section-00601-part-9",
    "section-00602-part-1",
    "section-00602-part-3",
    "section-00601-part-8",
    "section-00602-part-4",
}
row = build.actions["section-00602-part-5"]
remaining = set(build.actions) - removed - {row["id"]}
scene = build.scene(remaining)
assert build.planner.material_key(scene) == saved["material_sha256"]
state = build.state(remaining, build.initial, scene)
targets = np.asarray(saved["tcp_targets_m"])
for drawing in build.drawings(row, state.ready_nodes(), build.initial):
    proposed = np.asarray(drawing["tcp_targets_m"])
    if np.max(np.linalg.norm(proposed[[0, -1]] - targets[[0, -1]], axis=1)) < 1e-7:
        break
else:
    raise AssertionError("No source drawing matches the saved direction")
drawing.update(
    positions=saved["positions"],
    tcp_targets_m=saved["tcp_targets_m"],
    free_tool_orientation=True,
)
report = {
    "status": "RUNNING",
    "section": row["id"],
    "removed_suffix_sections": sorted(removed),
    "material_sha256": saved["material_sha256"],
    "saved_corrections": len(saved.get("collision_refinements", [])),
    "additional_correction_limit": 8,
    "attempts": [],
    "simulation_only": True,
    "scope": "One local stroke in its exact predecessor scene; not a complete build.",
    "inputs": {
        str(path): sha256(path)
        for path in (
            snapshot,
            run / "stroke_graph.json",
            run / "construction_plan.json",
            run / "cell_manifest.json",
            Path(__file__),
            Path("src/pen_art/planning/axis_repair.py"),
            Path("src/pen_art/planning/cell_planner.py"),
        )
    },
}
output = run / "continuation.json"
for correction in range(9):
    write_json(output, report)
    move = build.planner._prepare_drawing(state, build.arm.identity, drawing, None)
    if move is not None:
        actual_state = build.state(remaining, move.payload.edges[0].q0, scene)
        _, failure = build.planner.validate_interval(
            actual_state, (ScheduledMotion(move, 0),), move.duration_s
        )
        report.update(
            status="LOCAL_SECTION_MOTION_VALIDATED" if failure is None else "REPLAY_REJECTED",
            validation_failure=failure,
        )
        if failure is None:
            report["witness"] = build.checkpoint_witness(row["id"], move, drawing)
        break
    failure = build.planner.attempts[-1]
    report["attempts"].append({"additional_corrections": correction, "failure": failure})
    print("Continuation attempt", correction, failure["phase"], failure["status"], flush=True)
    if correction == 8 or monotonic() >= build.planner.deadline:
        report["status"] = "CONTINUATION_BUDGET"
        break
    if failure["phase"] != "DRAW" or "positions" not in failure:
        report["status"] = "NON_DRAW_REJECTION"
        break
    corrected = repair_collision_witness(build.planner, drawing, scene, failure)
    report["attempts"][-1]["correction"] = {
        key: value for key, value in corrected.items() if key not in {"positions", "tcp_targets_m"}
    }
    if corrected["status"] != "SAMPLED_FREE_AXIS_PROPOSAL":
        report["status"] = "CORRECTION_REJECTED"
        break
    drawing.update(positions=corrected["positions"], tcp_targets_m=corrected["tcp_targets_m"])
    write_json(run / "continued_drawing.json", drawing)
write_json(output, report)
print(report["status"], flush=True)
