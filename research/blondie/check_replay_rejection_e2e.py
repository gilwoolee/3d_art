from pathlib import Path

from pen_art.cell import prepare_cell
from pen_art.io import read_json, sha256, write_json
from pen_art.planning.section_build import SectionBuild, prepare_candidates

source = Path("runs/staged_shared_regression_v1")
run = Path("runs/staged_replay_rejection_v1")
assert not run.exists()
profile = Path("research/planning_cell/cell.yaml")
prepare_cell(profile, run, source)
prepare_candidates(run)
build = SectionBuild(run, profile, 600, gpu=True)
validate = build.planner.validate_interval
forced = []


def reject_one_verified_candidate(state, motions, end):
    after, failure = validate(state, motions, end)
    if not forced and not failure:
        rejection = {
            "status": "CONTACT_PATH_REJECTED",
            "arm": build.arm.identity,
            "time_s": 0.0,
            "test_only_forced_rejection": True,
        }
        forced.append(rejection)
        return None, rejection
    return after, failure


build.planner.validate_interval = reject_one_verified_candidate
build.run_build()
assert forced
state = read_json(run / "section_build.json")
assert state["status"] == "FULL_BUILD_VERIFIED", state["status"]
validation = read_json(run / "offline_dual_arm_validation.json")
assert validation["status"] == "CERTIFIED_GEOMETRIC_REPLAY"
assert any(r.get("validation", {}).get("test_only_forced_rejection") for r in state["failures"])
write_json(
    Path("research/blondie/replay_rejection_e2e.json"),
    {
        "status": "PASSED",
        "run": str(run),
        "test_only_forced_rejection": forced[0],
        "complete_build": state["status"],
        "independent_replay": validation["status"],
        "checked_intervals": validation["checked_intervals"],
        "simulation_only": True,
        "inputs": {
            str(p): sha256(p)
            for p in [
                run / "section_build.json",
                run / "offline_dual_arm_plan.json",
                run / "offline_dual_arm_validation.json",
                Path("src/pen_art/planning/section_build.py"),
            ]
        },
    },
)
print(
    "Forced rejection retained; alternate real motion completed full independent replay", flush=True
)
