"""Bounded fixed-upright whole-action IK, tracking, and growing-prefix check."""

import argparse
from pathlib import Path
from time import monotonic

import numpy as np

from pen_art.cell import load_cell
from pen_art.curves import flatten_curve, transform_points
from pen_art.io import read_json, sha256, write_json
from pen_art.planning.cell_motion import drawing_alternatives, pen_rotation
from pen_art.planning.cell_planner import check_growing_drawing_prefix, tracked_drawing_path
from pen_art.planning.scene import MaterialScene


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--maximum-seconds", type=float, default=120)
    parser.add_argument("--gpu", action="store_true")
    parser.add_argument("--section", default="section-00001")
    parser.add_argument("--rolls", type=float, nargs="+", default=[0, 180])
    parser.add_argument("--maximum-chord", type=float, default=0.003)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    profile_path = args.run / "profile.yaml"
    profile, arms, obstacles = load_cell(profile_path)
    arm = arms["arm_1"]
    actions = read_json(args.run / "construction_plan.json")["actions"]
    action = next(a for a in actions if a["id"] == args.section)
    base = next(a for a in actions if a.get("parent_curve", "").startswith("contour-"))
    transform = np.asarray(profile["world_from_build"])
    scene = MaterialScene()
    points, _ = flatten_curve(base, 0.00005, 0.0015)
    points = transform_points(points, transform)
    for index, (a, b) in enumerate(zip(points[:-1], points[1:], strict=True)):
        scene.add(a, b, base["diameter_m"] / 2, f"base-source/{index}")
    started, deadline = monotonic(), monotonic() + args.maximum_seconds
    report = {
        "status": "RUNNING",
        "stage": "setup",
        "run": str(args.run),
        "scope": "fixed upright full-arm whole drawing; conditional nominal completed base",
        "section": args.section,
        "profile_sha256": sha256(profile_path),
        "source_sha256": sha256(args.run / "construction_plan.json"),
        "margin_m": 0.001,
        "attempts": [],
        "full_clock_verified": False,
    }

    def save(stage):
        report.update(stage=stage, seconds=monotonic() - started)
        write_json(args.output / "result.json", report)
        print(stage, round(report["seconds"], 3), flush=True)

    try:
        backend = None
        if args.gpu:
            save("gpu_setup")
            from pen_art.planning.curobo_adapter import CuroboAdapter

            backend = CuroboAdapter(arm)
        rotations = [
            {
                "rotations": pen_rotation(arm, transform, np.deg2rad(roll), 0),
                "orientation_policy": "fixed",
                "pen_tilt_deg": 0.0,
                "roll_rad": float(np.deg2rad(roll)),
            }
            for roll in args.rolls
        ]
        save("whole_path_ik")
        for index, drawing in enumerate(
            drawing_alternatives(
                arm,
                action,
                {action["start"]},
                transform,
                obstacles,
                deadline=deadline,
                seeds=[arm.initial_positions, (arm.robot.lower + arm.robot.upper) / 2],
                settings={
                    "maximum_subdivision_depth": 20,
                    "beam_width": 4,
                    "maximum_chord_m": args.maximum_chord,
                },
                clearance_m=0.001,
                rotations=rotations,
                orientation_policy="fixed",
                ik_backend=backend,
            )
        ):
            attempt = {
                "status": drawing["status"],
                "roll_rad": drawing.get("roll_rad"),
                "failed_sample": drawing.get("failed_sample"),
                "sampling_diagnostics": drawing.get("sampling_diagnostics"),
            }
            report["attempts"].append(attempt)
            save("ik_candidate")
            if drawing["status"] != "SAMPLED_DRAWING_PATH":
                continue
            write_json(args.output / f"candidate-{index:03}.json", drawing)
            save("continuous_fixed_orientation_tracking")
            tracked = tracked_drawing_path(
                arm, drawing, True, deadline, require_fixed_orientation=True
            )
            attempt["tracking_status"] = tracked["status"]
            if tracked["status"] != "BOUNDED":
                attempt["tracking_failure"] = tracked
                save("tracking_rejected")
                continue
            attempt.update(
                orientation_error_bound_rad=tracked["orientation_error_bound_rad"],
                position_error_bound_m=tracked["position_error_bound_m"],
                checked_intervals=len(tracked["positions"]) - 1,
            )
            save("growing_prefix")
            _, failure = check_growing_drawing_prefix(
                arm,
                tracked,
                scene,
                obstacles,
                action["diameter_m"] / 2,
                0.001,
                20,
                deadline,
                progress=lambda done, total: print("prefix", done, total, flush=True),
            )
            attempt["growing_prefix_failure"] = failure
            if failure is None:
                report["status"] = "FIXED_WHOLE_PREFIX_VERIFIED"
                write_json(args.output / "checked_drawing.json", drawing)
                break
            save("prefix_rejected")
    except TimeoutError as error:
        report.update(status="TIME_BUDGET", reason=str(error))
    except Exception as error:
        report.update(status="ERROR", reason=f"{type(error).__name__}: {error}")
        save("error")
        raise
    finally:
        if report["status"] == "RUNNING":
            report["status"] = "UNRESOLVED_WITHIN_BUDGET"
        save("complete")


if __name__ == "__main__":
    main()
