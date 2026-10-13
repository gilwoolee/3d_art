"""Bounded free-axis base-contour solve using existing cache postures as hints."""

import argparse
from dataclasses import replace
from pathlib import Path
from time import monotonic

import numpy as np

from pen_art.cell import prepare_cell
from pen_art.curves import flatten_curve, transform_points
from pen_art.io import read_json, write_json
from pen_art.planning.axis_repair import repair_drawing_axes
from pen_art.planning.cell_motion import (
    anchored_strokes,
    inverse_axis_candidates,
    radial_pen_rotations,
)
from pen_art.planning.dual_arm import ScheduledMotion
from pen_art.planning.indexed_build import IndexedMotions, at_index
from pen_art.planning.section_build import SectionBuild, prepare_candidates


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, default=Path("runs/bottle_taper70_phase1_v1"))
    parser.add_argument("--output", type=Path, default=Path("runs/bottle_taper70_base_guided_v1"))
    parser.add_argument("--maximum-seconds", type=float, default=180)
    args = parser.parse_args()
    profile = args.run / "profile.yaml"
    prepare_cell(profile, args.output, Path(read_json(args.run / "run.json")["cell_source_run"]))
    prepare_candidates(args.output)
    build = SectionBuild(args.output, profile, args.maximum_seconds, gpu=False,
                         motion_library=Path("runs/motion_library_phase1_taper70_v1"))
    model = IndexedMotions(build)
    arm, planner = build.arm, build.planner
    state = build.state(set(), build.initial)
    state.payload.table_index = model.table.home_index
    started = monotonic()
    report = {"status": "RUNNING", "run": str(args.run), "attempts": []}
    actions = anchored_strokes(build.actions["section-00000"], state.ready_nodes())
    for oriented in actions:
        points, _ = flatten_curve(oriented, .00005, .003)
        points = transform_points(points, build.transform)
        rotations = radial_pen_rotations(arm, points, build.transform, 70)
        target = np.eye(4)
        target[:3, :3], target[:3, 3] = rotations[0], points[0]
        records = build.motion_runtime.library.records
        hint = next((r for r in records if r["provenance"].get("section") == "section-00000"), None)
        seeds = [build.initial, (arm.robot.lower + arm.robot.upper) / 2]
        hints = None
        if hint is not None:
            old_q = np.asarray(hint["trajectory"]["q_path"])
            old_points = np.asarray(hint["trajectory"]["source_points_m"])
            arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(old_points, axis=0), axis=1))]
            grid = np.linspace(0, arc[-1], len(points))
            hints = np.column_stack([np.interp(grid, arc, axis) for axis in old_q.T])
            seeds.insert(0, hints[0])
        starts = inverse_axis_candidates(arm, target, seeds)
        for initial in starts[:2]:
            if arm.collision.check(initial, planner.obstacles, planner.margin):
                continue
            positions = np.tile(initial, (len(points), 1)) if hints is None else hints.copy()
            positions[0] = initial
            drawing = {
                "arm": arm.identity, "stroke_id": "section-00000",
                "start_anchor": oriented["start"], "end_anchor": oriented["end"],
                "start_visit": oriented["start_visit"], "reverse": oriented["reverse"],
                "positions": positions.tolist(), "tcp_targets_m": points.tolist(),
                "tcp_rotation": rotations[0].tolist(), "tcp_rotations": rotations.tolist(),
                "orientation_policy": "outward_radial", "pen_tilt_deg": 70., "roll_rad": 0.,
                "table_index": 0, "proposal_source": "guided_cache_posture",
                "variant": "guided-base-cache",
                "release_cooling_s": build.cooling,
            }
            repair = repair_drawing_axes(
                planner, drawing, state.payload,
                progress=lambda done, total, detail: print("axis", done, total, detail, flush=True),
            )
            row = {"repair_status": repair["status"], "seconds": monotonic() - started}
            report["attempts"].append(row)
            if repair["status"] != "SAMPLED_FREE_AXIS_PROPOSAL":
                row["failed_sample"] = repair.get("sample")
                write_json(args.output / "result.json", report)
                continue
            drawing.update(repair, status="SAMPLED_DRAWING_PATH", free_tool_orientation=True)
            write_json(args.output / "candidate.json", drawing)
            move = planner.prepare_drawing(state, arm.identity, drawing, build.initial)
            row["prepared"] = move is not None
            row["failures"] = planner.attempts
            if move is not None:
                move = replace(move, payload=at_index(move.payload, model.table.home_index))
                arm.collision = getattr(arm.collision, "model", arm.collision)
                tip = planner.tip_models[arm.identity]
                planner.tip_models[arm.identity] = getattr(tip, "model", tip)
                _, failure = planner.validate_interval(
                    state, (ScheduledMotion(move, 0),), move.duration_s
                )
                row["replay_failure"] = failure
                if failure is None:
                    build.motion_runtime.observe(drawing, state.payload, move=move)
                    write_json(args.output / "accepted_witness.json", {
                        "drawing": drawing, "trajectory": move.payload.record(),
                    })
                    report["status"] = "BASE_INDEPENDENTLY_VERIFIED"
            write_json(args.output / "result.json", report)
            if report["status"] == "BASE_INDEPENDENTLY_VERIFIED" or monotonic() >= planner.deadline:
                break
        if report["status"] == "BASE_INDEPENDENTLY_VERIFIED" or monotonic() >= planner.deadline:
            break
    if report["status"] == "RUNNING":
        report["status"] = "UNRESOLVED_WITHIN_BUDGET"
    build.motion_runtime.flush()
    report["seconds"] = monotonic() - started
    report["reuse"] = build.motion_runtime.report()
    write_json(args.output / "result.json", report)
    print(report["status"], report["seconds"], flush=True)


if __name__ == "__main__":
    main()
