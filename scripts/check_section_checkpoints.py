"""Resume a real staged build from a saved prefix and reject stale witnesses.

Prepare the small source with check_shared_source.py first. Use a fresh --run.
"""

import argparse
from pathlib import Path

from pen_art.cell import prepare_cell
from pen_art.io import read_json, sha256, write_json
from pen_art.planning.section_build import SectionBuild, prepare_candidates

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--source-run", type=Path, default=Path("runs/staged_shared_regression_v1"))
parser.add_argument("--run", type=Path, default=Path("runs/staged_checkpoint_v1"))
args = parser.parse_args()
source, run = args.source_run, args.run
profile = Path("research/planning_cell/cell.yaml")
assert not run.exists(), "Choose a fresh --run; existing evidence is preserved"
prepare_cell(profile, run, source)
prepare_candidates(run)
first = SectionBuild(run, profile, 600, gpu=True)
first.run_build()
assert read_json(run / "section_build.json")["status"] == "FULL_BUILD_VERIFIED"
index = read_json(run / "forward_checkpoint.json")
assert len(index["witnesses"]) == 3
# Model an interrupted search that saved only its first two accepted motions.
# The third witness remains an unused file, just as it could after a crash
# before the atomic index write. It must not be assumed completed on resume.
index["witnesses"] = index["witnesses"][:2]
write_json(run / "forward_checkpoint.json", index)
second = SectionBuild(run, profile, 600, gpu=True)
second.run_build()
state = read_json(run / "section_build.json")
assert state["status"] == "FULL_BUILD_VERIFIED"
assert state["revalidated_saved_motions"] == 2, state
assert second.gpu.batches, "The missing section must receive new motion planning"
reference = index["witnesses"][0]
path = run / reference["file"]
original = path.read_bytes()
try:
    path.write_bytes(original + b"\n")
    third = SectionBuild(run, profile, 600, gpu=True)
    row = third.actions["section-00000"]
    check = third.checkpoint_proposal(0, row, third.state(set(), third.initial), third.initial)
    assert check is None
finally:
    path.write_bytes(original)
checkpoint_path = run / "forward_checkpoint.json"
original = checkpoint_path.read_bytes()
try:
    broken = read_json(checkpoint_path)
    broken["binding"]["initial_positions"][0] += 0.1
    write_json(checkpoint_path, broken)
    fourth = SectionBuild(run, profile, 600, gpu=True)
    assert not fourth.checkpoint_records
finally:
    checkpoint_path.write_bytes(original)
validation = read_json(run / "offline_dual_arm_validation.json")
write_json(
    Path("research/blondie/section_checkpoint.json"),
    {
        "status": "PASSED",
        "run": str(run),
        "revalidated_saved_motions": 2,
        "freshly_planned_remaining_sections": 1,
        "fresh_ik_batches_on_resume": len(second.gpu.batches),
        "corrupt_witness_rejected": True,
        "changed_input_binding_rejected": True,
        "independent_replay": validation["status"],
        "checked_intervals": validation["checked_intervals"],
        "simulation_only": True,
        "inputs": {
            str(run / name): sha256(run / name)
            for name in [
                "forward_checkpoint.json",
                "offline_dual_arm_plan.json",
                "offline_dual_arm_validation.json",
            ]
        },
    },
)
print("Saved section motions revalidated; full replay and both rejection checks PASSED", flush=True)
