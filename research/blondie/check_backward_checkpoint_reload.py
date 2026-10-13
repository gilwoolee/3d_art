"""Revalidate a complete saved suffix from a fresh interpreter."""

import shutil
import subprocess
from pathlib import Path

from pen_art.io import read_json, sha256, write_json
from pen_art.planning.section_build import SectionBuild

source = Path("runs/staged_backward_checkpoint_v1")
run = Path("runs/staged_backward_checkpoint_reload_v1")
profile = Path("research/planning_cell/cell.yaml")
assert read_json(Path("research/blondie/backward_checkpoint.json"))["status"] == "PASS"
assert not run.exists()
shutil.copytree(source, run)
metadata = read_json(run / "run.json")
metadata["id"] = run.name
metadata["name"] = "Fresh-process backward checkpoint recovery"
write_json(run / "run.json", metadata)
write_json(
    run / "section_build.json",
    {"status": "RECOVERY_PENDING_REVALIDATION", "completed_sections": 0, "required_sections": 3},
)
checkpoint = run / "backward_checkpoint.json"
original = checkpoint.read_bytes()
try:
    malformed = read_json(checkpoint)
    malformed["witnesses"][next(iter(malformed["witnesses"]))] = 42
    write_json(checkpoint, malformed)
    assert not SectionBuild(run, profile, 600).backward_checkpoint_records
finally:
    checkpoint.write_bytes(original)
build = SectionBuild(run, profile, 600)


def unexpected_drawing(*args, **kwargs):
    raise AssertionError("Fresh interpreter should recover every saved suffix motion")


build.drawings = unexpected_drawing
(run / "offline_dual_arm_replay_checkpoint.json").unlink(missing_ok=True)
build.run_build(regression_only=True)
state = read_json(run / "section_build.json")
validation = read_json(run / "offline_dual_arm_validation.json")
assert state["status"] == "FULL_BUILD_VERIFIED"
assert build.revalidated_backward_motions == state["required_sections"] == 3
assert validation["status"] == "CERTIFIED_GEOMETRIC_REPLAY"
assert not validation.get("resumed_from_s")
receipt = Path("research/blondie/backward_checkpoint_reload.json")
result = {
    "status": "REPLAY_PASSED_BROWSER_PENDING",
    "run": str(run),
    "source": str(source),
    "revalidated_backward_motions": build.revalidated_backward_motions,
    "checked_intervals": validation["checked_intervals"],
    "malformed_index_rejected": True,
    "simulation_only": True,
    "inputs": {
        str(path): sha256(path)
        for path in [
            Path(__file__),
            Path("src/pen_art/planning/section_build.py"),
            checkpoint,
            run / "offline_dual_arm_plan.json",
            run / "offline_dual_arm_validation.json",
        ]
    },
}
write_json(receipt, result)
subprocess.run(
    [
        "tools/node/bin/node",
        "web/scripts/check-replay.mjs",
        run.name,
        "research/blondie/backward_checkpoint_reload_browser",
    ],
    check=True,
)
result["status"] = "PASS"
result["browser_receipt"] = (
    "research/blondie/backward_checkpoint_reload_browser_browser_replay.json"
)
write_json(receipt, result)
print(
    "Fresh process recovered the full suffix and passed independent replay plus browser", flush=True
)
