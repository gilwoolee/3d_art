"""Revalidate a saved complete base candidate with current policy and raw replay."""

import argparse
from dataclasses import replace
from pathlib import Path
from time import monotonic

from pen_art.io import read_json, write_json
from pen_art.planning.clock_path import ClockPath
from pen_art.planning.dual_arm import MotionProposal, ScheduledMotion
from pen_art.planning.indexed_build import IndexedMotions, at_index
from pen_art.planning.section_build import SectionBuild

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--run", type=Path, required=True)
parser.add_argument("--profile", type=Path, required=True)
parser.add_argument("--maximum-seconds", type=float, default=100)
parser.add_argument("--replay-only", action="store_true")
args = parser.parse_args()
build = SectionBuild(args.run, args.profile, args.maximum_seconds, gpu=False,
                     motion_library=Path("runs/motion_library_phase1_taper70_v1"))
model = IndexedMotions(build)
state = build.state(set(), build.initial)
state.payload.table_index = model.table.home_index
drawing = read_json(args.run / "candidate.json")
drawing.setdefault("variant", "guided-base-cache")
write_json(args.run / "candidate.json", drawing)
started = monotonic()
if args.replay_only:
    saved = read_json(args.run / "prepared_witness.json")
    clock = ClockPath.from_record(saved["trajectory"])
    drawing = saved["drawing"]
    move = MotionProposal(
        drawing["stroke_id"], build.arm.identity, drawing["variant"], clock.duration_s,
        drawing["start_anchor"], tuple(clock.edges[-1].q1), clock,
    )
else:
    move = build.planner.prepare_drawing(state, build.arm.identity, drawing, build.initial)
if move is not None:
    move = replace(move, payload=at_index(move.payload, model.table.home_index))
report = {"status": "NO_ACCEPTED_BASE", "failures": build.planner.attempts}
if move is not None:
    write_json(args.run / "prepared_witness.json", {
        "drawing": drawing, "trajectory": move.payload.record(),
    })
    build.arm.collision = getattr(build.arm.collision, "model", build.arm.collision)
    tip = build.planner.tip_models[build.arm.identity]
    build.planner.tip_models[build.arm.identity] = getattr(tip, "model", tip)
    _, failure = build.planner.validate_interval(
        state, (ScheduledMotion(move, 0),), move.duration_s
    )
    report["replay_failure"] = failure
    if failure is None:
        build.motion_runtime.observe(drawing, state.payload, move=move)
        write_json(args.run / "accepted_witness.json", {
            "drawing": drawing, "trajectory": move.payload.record(),
        })
        report["status"] = "BASE_INDEPENDENTLY_VERIFIED"
build.motion_runtime.flush()
report["seconds"] = monotonic() - started
report["reuse"] = build.motion_runtime.report()
write_json(args.run / "resume_result.json", report)
print(report["status"], report["seconds"], flush=True)
