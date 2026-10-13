"""Bounded full-arm check of a tool-clear fixed-rotation sweep candidate."""

import argparse
from pathlib import Path
from time import monotonic

import numpy as np
from evaluate_fixed_pen_orientation import rotation

from pen_art.cell import load_cell
from pen_art.curves import flatten_curve, transform_points
from pen_art.io import read_json, sha256, write_json
from pen_art.planning.cell_motion import drawing_alternatives
from pen_art.planning.cell_planner import (
    check_growing_drawing_prefix,
    tracked_drawing_path,
)
from pen_art.planning.scene import MaterialScene


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--object", choices=["cube", "bottle"], required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--maximum-seconds", type=float, default=60)
    parser.add_argument("--gpu", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    data = read_json(args.summary)["results"][args.object]
    passing = [c for c in data["candidates"] if c["sampled_clearance_pass"]]
    if not passing:
        write_json(
            args.output / "result.json", {"status": "NO_TOOL_CLEAR_CANDIDATE", "run": data["run"]}
        )
        return
    run = Path(data["run"])
    profile_path = run / "profile.yaml"
    if sha256(profile_path) != data["profile_sha256"]:
        raise ValueError("Profile changed since the tool-only sweep")
    if sha256(run / "construction_plan.json") != data["source_sha256"]:
        raise ValueError("Source changed since the tool-only sweep")
    profile, arms, obstacles = load_cell(profile_path)
    arm = arms["arm_1"]
    actions = read_json(run / "construction_plan.json")["actions"]
    row = next(a for a in actions if a["id"] == data["section"])
    base = next(a for a in actions if a["id"] == data["base_section"])
    transform = np.asarray(profile["world_from_build"])
    base_points, _ = flatten_curve(base, 0.00005, 0.0015)
    base_points = transform_points(base_points, transform)
    scene = MaterialScene()
    for i, (a, b) in enumerate(zip(base_points[:-1], base_points[1:], strict=True)):
        scene.add(a, b, base["diameter_m"] / 2, f"base-source/{i}")
    started = monotonic()
    deadline = started + args.maximum_seconds
    backend = None
    if args.gpu:
        from pen_art.planning.curobo_adapter import CuroboAdapter

        backend = CuroboAdapter(arm)
    report = {
        "status": "RUNNING",
        "run": str(run),
        "scope": "fixed-R full-arm whole-zigzag plus conditional ideal-base prefix",
        "profile_sha256": sha256(profile_path),
        "sweep_sha256": sha256(args.summary),
        "attempts": [],
    }
    seeds = [arm.initial_positions, (arm.robot.lower + arm.robot.upper) / 2]
    for candidate_index, candidate in enumerate(passing):
        if monotonic() >= deadline:
            break
        descriptor = {
            "rotations": rotation(
                candidate["tilt_deg"], candidate["azimuth_deg"], candidate["roll_deg"]
            ),
            "orientation_policy": "fixed",
            "pen_tilt_deg": candidate["tilt_deg"],
            "roll_rad": np.deg2rad(candidate["roll_deg"]),
        }
        for drawing in drawing_alternatives(
            arm,
            row,
            {row["start"]},
            transform,
            obstacles,
            deadline=deadline,
            seeds=seeds,
            settings={"maximum_subdivision_depth": 20, "beam_width": 4},
            rotations=[descriptor],
            orientation_policy="fixed",
            ik_backend=backend,
        ):
            attempt = {
                "candidate": {
                    k: v for k, v in candidate.items() if k not in {"group_witnesses", "worst"}
                },
                "status": drawing["status"],
                "failed_sample": drawing.get("failed_sample"),
                "sampling_diagnostics": drawing.get("sampling_diagnostics"),
                "seconds": monotonic() - started,
            }
            report["attempts"].append(attempt)
            if drawing["status"] == "SAMPLED_DRAWING_PATH":
                write_json(args.output / f"candidate-{candidate_index:03}.json", drawing)
                tracked = tracked_drawing_path(
                    arm, drawing, True, deadline, require_fixed_orientation=True
                )
                attempt["tracking_status"] = tracked["status"]
                if tracked["status"] == "BOUNDED":
                    _, failure = check_growing_drawing_prefix(
                        arm,
                        tracked,
                        scene,
                        obstacles,
                        row["diameter_m"] / 2,
                        0.001,
                        20,
                        deadline,
                    )
                    attempt["growing_prefix_failure"] = failure
                    if failure is None:
                        report["status"] = "FIXED_WHOLE_PREFIX_VERIFIED"
                        attempt["orientation_error_bound_rad"] = tracked[
                            "orientation_error_bound_rad"
                        ]
                        write_json(args.output / "checked_drawing.json", drawing)
            report["seconds"] = monotonic() - started
            write_json(args.output / "result.json", report)
            print(attempt, flush=True)
            if report["status"] == "FIXED_WHOLE_PREFIX_VERIFIED" or monotonic() >= deadline:
                break
        if report["status"] == "FIXED_WHOLE_PREFIX_VERIFIED":
            break
    if report["status"] == "RUNNING":
        report["status"] = "UNRESOLVED_WITHIN_BUDGET"
    report["seconds"] = monotonic() - started
    write_json(args.output / "result.json", report)


if __name__ == "__main__":
    main()
