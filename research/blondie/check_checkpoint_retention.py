"""Interrupt real checkpoint revalidation, then complete independent replay."""

import shutil
from pathlib import Path

from pen_art.cell import prepare_cell
from pen_art.io import read_json, sha256, write_json
from pen_art.planning.section_build import SectionBuild, prepare_candidates

source = Path("runs/staged_checkpoint_partial_v1")
run = Path("runs/staged_checkpoint_retention_v1")
profile = Path("research/planning_cell/cell.yaml")
assert not run.exists(), "Choose a fresh run to preserve evidence"
prepare_cell(profile, run, source)
prepare_candidates(run)
index = read_json(source / "forward_checkpoint.json")
assert len(index["witnesses"]) == 3
for reference in index["witnesses"]:
    path = Path(reference["file"])
    (run / path).parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source / path, run / path)
    assert sha256(run / path) == reference["sha256"]
first = SectionBuild(run, profile, 600)
index["binding"] = first.checkpoint_binding
write_json(run / "forward_checkpoint.json", index)
first = SectionBuild(run, profile, 600)
original_checkpoint = (run / "forward_checkpoint.json").read_bytes()
report = first.report


class Interrupted(Exception):
    pass


def interrupt_after_first_revalidated_motion(status, **extra):
    report(status, **extra)
    if status == "PROPOSED_ORDER_RUNNING":
        assert first.resumed_motions == 1
        raise Interrupted


first.report = interrupt_after_first_revalidated_motion
try:
    first.run_build()
except Interrupted:
    pass
else:
    raise AssertionError("Revalidation interruption did not occur")
assert (run / "forward_checkpoint.json").read_bytes() == original_checkpoint
second = SectionBuild(run, profile, 600)
second.run_build()
state = read_json(run / "section_build.json")
validation = read_json(run / "offline_dual_arm_validation.json")
assert state["status"] == "FULL_BUILD_VERIFIED", state["status"]
assert validation["status"] == "CERTIFIED_GEOMETRIC_REPLAY"
assert second.resumed_motions == 3
write_json(
    Path("research/blondie/checkpoint_retention.json"),
    {
        "status": "PASSED",
        "run": str(run),
        "preserved_checkpoint_after_revalidating": 1,
        "retained_proposals": 3,
        "revalidated_on_second_resume": second.resumed_motions,
        "complete_build": state["status"],
        "independent_replay": validation["status"],
        "checked_intervals": validation["checked_intervals"],
        "simulation_only": True,
        "inputs": {
            str(path): sha256(path)
            for path in (
                Path(__file__),
                Path("src/pen_art/planning/section_build.py"),
                run / "forward_checkpoint.json",
                run / "offline_dual_arm_plan.json",
                run / "offline_dual_arm_validation.json",
            )
        },
    },
)
print("Checkpoint survived interrupted revalidation; full replay passed", flush=True)
