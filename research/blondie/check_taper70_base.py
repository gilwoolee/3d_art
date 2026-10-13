"""Validate the complete base contour from home on the tapered-pen cell."""

import argparse
from pathlib import Path
from time import monotonic

from pen_art.cell import prepare_cell
from pen_art.io import read_json, sha256, write_json
from pen_art.planning.clock_path import ClockPath
from pen_art.planning.dual_arm import MotionProposal, ScheduledMotion
from pen_art.planning.indexed_build import IndexedMotions
from pen_art.planning.section_build import SectionBuild, prepare_candidates

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("object", choices=["cube", "bottle"])
parser.add_argument("--seconds", type=float, default=120)
parser.add_argument("--output", type=Path)
parser.add_argument("--seed-witness", type=Path)
parser.add_argument("--source-run", type=Path)
parser.add_argument("--library", type=Path, default=Path("runs/motion_library_phase1_upright27_v1"))
args = parser.parse_args()
source_run = args.source_run or Path(f"runs/{args.object}_taper70_phase1_v1")
run = args.output or Path(f"runs/{args.object}_taper70_base_check_v1")
profile = source_run / "profile.yaml"
prepare_cell(profile, run, Path(read_json(source_run / "run.json")["cell_source_run"]))
prepare_candidates(run)
build = SectionBuild(
    run,
    profile,
    args.seconds,
    gpu=True,
    motion_library=args.library,
)
model = IndexedMotions(build)
state = build.state(set(), build.initial)
state.payload.table_index = model.table.home_index
started = monotonic()
if args.seed_witness:
    old_run = args.seed_witness.parent
    assert read_json(old_run / "cell_config.json") == build.planner.profile
    assert sha256(old_run / "construction_plan.json") == sha256(run / "construction_plan.json")
    saved = read_json(args.seed_witness)
    drawing = dict(saved["drawing"])
    drawing.pop("motion_request", None)
    drawing["proposal_source"] = "revalidated_previous_policy"
    path = ClockPath.from_record(saved["trajectory"])
    move = MotionProposal(
        "section-00000",
        build.arm.identity,
        "revalidated-previous-policy",
        path.duration_s,
        drawing["start_anchor"],
        tuple(path.edges[-1].q1),
        path,
    )
    accepted = (move, drawing, None)
else:
    accepted = next(iter(model.forward_motions(build.actions["section-00000"], state)), None)
report = {
    "seconds": monotonic() - started,
    "failures": build.planner.attempts,
    "status": "NO_ACCEPTED_BASE",
    "source_run": str(source_run),
}
if accepted is not None:
    move, drawing, after = accepted
    arm = build.arm
    arm.collision = getattr(arm.collision, "model", arm.collision)
    tip = build.planner.tip_models[arm.identity]
    build.planner.tip_models[arm.identity] = getattr(tip, "model", tip)
    _, failure = build.planner.validate_interval(
        state, (ScheduledMotion(move, 0),), move.duration_s
    )
    report.update(
        status="BASE_INDEPENDENTLY_VERIFIED" if failure is None else "REPLAY_REJECTED",
        replay_failure=failure,
    )
    if args.seed_witness and failure is None:
        build.motion_runtime.observe(drawing, state.payload, move=move)
    write_json(
        run / "accepted_witness.json", {"drawing": drawing, "trajectory": move.payload.record()}
    )
build.motion_runtime.flush()
report["reuse"] = build.motion_runtime.report()
write_json(run / "result.json", report)
print(report["status"], report["seconds"], flush=True)
