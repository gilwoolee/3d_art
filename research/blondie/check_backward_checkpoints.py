"""Interrupt, recover, and replay a real backward section build."""

import subprocess
from copy import deepcopy
from pathlib import Path

from pen_art.cell import prepare_cell
from pen_art.io import read_json, sha256, write_json
from pen_art.planning.section_build import SectionBuild, prepare_candidates

source = Path("runs/staged_beam_cache_v1")
run = Path("runs/staged_backward_checkpoint_v1")
profile = Path("research/planning_cell/cell.yaml")
assert not run.exists()
prepare_cell(profile, run, source)
prepare_candidates(run)
first = SectionBuild(run, profile, 600)
original_report = first.report


class Interrupted(Exception):
    pass


def interrupt_after_first_suffix(status, **extra):
    original_report(status, **extra)
    if status == "REGRESSION_RUNNING" and extra.get("depth") == 0:
        raise Interrupted


first.report = interrupt_after_first_suffix
try:
    first.run_build(regression_only=True)
except Interrupted:
    pass
else:
    raise AssertionError("The real backward build did not save its first suffix")
checkpoint_path = run / "backward_checkpoint.json"
original_index = checkpoint_path.read_bytes()
index = read_json(checkpoint_path)
assert len(index["witnesses"]) == 1
reference = next(iter(index["witnesses"].values()))[0]
path = run / reference["file"]
saved_bytes = path.read_bytes()
saved = read_json(path)
row = first.actions[saved["section"]]
context = saved["context"]
state = first.state(context["predecessor"], first.initial)
try:
    path.write_bytes(saved_bytes + b"\n")
    assert first.backward_checkpoint_proposal(reference, row, context, state) is None
finally:
    path.write_bytes(saved_bytes)
wrong_context = deepcopy(context)
wrong_context["terminal_configuration"][0] += 0.01
assert first.backward_checkpoint_proposal(reference, row, wrong_context, state) is None
try:
    changed = deepcopy(index)
    changed["binding"]["initial_positions"][0] += 0.01
    write_json(checkpoint_path, changed)
    assert not SectionBuild(run, profile, 600).backward_checkpoint_records
finally:
    checkpoint_path.write_bytes(original_index)

second = SectionBuild(run, profile, 600)
second.run_build(regression_only=True)
assert read_json(run / "section_build.json")["status"] == "FULL_BUILD_VERIFIED"
assert second.revalidated_backward_motions == 1
index = read_json(checkpoint_path)
assert all(key in index["witnesses"] for key in first.backward_checkpoint_records)
print("Interrupted suffix resumed; full backward build passed", flush=True)

# Recover the complete saved suffix without requesting any new drawing modes.
third = SectionBuild(run, profile, 600)


def unexpected_fresh_drawing(*args, **kwargs):
    raise AssertionError("Every suffix motion should be recovered and revalidated")


third.drawings = unexpected_fresh_drawing
(run / "offline_dual_arm_replay_checkpoint.json").unlink(missing_ok=True)
third.run_build(regression_only=True)
state = read_json(run / "section_build.json")
validation = read_json(run / "offline_dual_arm_validation.json")
assert state["status"] == "FULL_BUILD_VERIFIED"
assert third.revalidated_backward_motions == state["required_sections"] == 3
assert validation["status"] == "CERTIFIED_GEOMETRIC_REPLAY"
assert not validation.get("resumed_from_s")
result = {
    "status": "REPLAY_PASSED_BROWSER_PENDING",
    "run": str(run),
    "interrupted_suffix_sections": 1,
    "partial_resume_revalidated": second.revalidated_backward_motions,
    "complete_resume_revalidated": third.revalidated_backward_motions,
    "corrupt_witness_rejected": True,
    "different_terminal_context_rejected": True,
    "changed_input_binding_rejected": True,
    "checked_intervals": validation["checked_intervals"],
    "simulation_only": True,
    "scope": (
        "Actual three-section backward robot build with fresh independent replay; "
        "no large-build completion claim."
    ),
    "inputs": {
        str(p): sha256(p)
        for p in [
            Path(__file__),
            Path("src/pen_art/planning/section_build.py"),
            checkpoint_path,
            run / "offline_dual_arm_plan.json",
            run / "offline_dual_arm_validation.json",
        ]
    },
}
receipt = Path("research/blondie/backward_checkpoint.json")
write_json(receipt, result)
subprocess.run(
    [
        "tools/node/bin/node",
        "web/scripts/check-replay.mjs",
        run.name,
        "research/blondie/backward_checkpoint_browser",
    ],
    check=True,
)
result["status"] = "PASS"
result["browser_receipt"] = "research/blondie/backward_checkpoint_browser_browser_replay.json"
write_json(receipt, result)
print("Complete suffix recovered, independently replayed, and shown at 10x", flush=True)
