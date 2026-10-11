"""Fixed three-option base-mount assessment; sampled poses are not motion plans."""

import argparse
from dataclasses import replace
from itertools import product
from pathlib import Path
from time import monotonic

import numpy as np
from scipy.spatial.transform import Rotation

from ..cell import WorldArm, load_cell
from ..curves import sample_curve, transform_points
from ..io import artifact, read_json, sha256, write_json
from ..kinematics import inverse_candidates
from .cell_motion import pen_rotation
from .collision import RobotCollisionModel
from .coordinated_motion import check_pair_linear


def assess_mounting(run, profile_path, maximum_seconds=180):
    run, profile_path = Path(run), Path(profile_path)
    deadline = monotonic() + maximum_seconds
    profile, original, obstacles = load_cell(profile_path)
    if profile != read_json(run / "cell_config.json"):
        raise ValueError("Mount assessment must match the saved cell configuration")
    if len(original) != 2 or maximum_seconds <= 0:
        raise ValueError("Mount assessment requires two arms and a positive budget")
    source = read_json(run / "pen_strokes.json")
    cloud = np.unique(
        np.round(np.concatenate([sample_curve(c, 4) for c in source["curves"]]), 7), axis=0
    )
    # Fixed farthest-point coverage of actual strokes, including ground and crown.
    selected = [int(np.argmin(cloud[:, 2]))]
    for _ in range(min(12, len(cloud)) - 1):
        distances = np.min(np.linalg.norm(cloud[:, None] - cloud[selected], axis=2), axis=1)
        selected.append(int(np.argmax(distances)))
    points = transform_points(cloud[selected], np.asarray(profile["world_from_build"]))
    order = np.argsort(points[:, 0])
    pairs = [(int(a), int(b)) for a, b in zip(order[: len(order) // 2], order[::-1], strict=False)]
    candidates = []
    for key, label in (
        ("table", "Upright table mount"),
        ("yaw45", "45° turn on table"),
        ("tilt45", "45° base tilt toward sculpture"),
    ):
        arms, rows, poses = {}, {}, {}
        for identity, original_arm in original.items():
            base = original_arm.robot.world_from_base.copy()
            if key == "yaw45":
                base[:3, :3] = Rotation.from_euler("z", 45, degrees=True).as_matrix() @ base[:3, :3]
            elif key == "tilt45":
                inward = np.asarray(profile["world_from_build"])[:2, 3] - base[:2, 3]
                inward /= np.linalg.norm(inward)
                axis = np.array([-inward[1], inward[0], 0.0])
                base[:3, :3] = Rotation.from_rotvec(axis * np.pi / 4).as_matrix() @ base[:3, :3]
            robot = WorldArm(original_arm.robot.local, base)
            arm = replace(
                original_arm,
                robot=robot,
                collision=RobotCollisionModel(robot, original_arm.tool.bodies),
            )
            arms[identity] = arm
            rotation = pen_rotation(
                original_arm,
                profile["world_from_build"],
                tilt_deg=profile.get("drawing_pen_tilt_deg", 0.0),
            )
            initial = original_arm.initial_positions.copy()
            if key == "yaw45":
                initial[0] -= np.pi / 4
            seeds = [initial.copy(), initial.copy(), initial.copy()]
            seeds[1][2], seeds[1][4] = 0.8, -0.8
            seeds[2][2], seeds[2][4] = -0.8, 0.8
            records, solutions, previous = [], {}, initial
            for index, point in enumerate(points):
                if monotonic() >= deadline:
                    break
                target = np.eye(4)
                target[:3, :3], target[:3, 3] = rotation, point
                found = inverse_candidates(
                    robot,
                    target,
                    arm.flange_to_tcp,
                    [previous, *seeds],
                    position_tolerance=0.00005,
                    orientation_tolerance=0.001,
                    maximum_evaluations=60,
                )
                clear = []
                for solution in found:
                    q = np.asarray(solution["positions"])
                    contacts = arm.collision.check(
                        q, obstacles, profile["environment"]["clearance_m"]
                    )
                    if not contacts:
                        margin = float(
                            np.min(
                                np.minimum(q - robot.lower, robot.upper - q)
                                / (robot.upper - robot.lower)
                            )
                        )
                        clear.append({"positions": q.tolist(), "joint_margin_fraction": margin})
                clear.sort(key=lambda row: row["joint_margin_fraction"], reverse=True)
                if clear:
                    previous = np.asarray(clear[0]["positions"])
                solutions[index] = clear
                records.append(
                    {
                        "index": index,
                        "tcp_m": point.tolist(),
                        "ik_found": bool(found),
                        "clear_solutions": clear,
                    }
                )
            rows[identity] = records
            poses[identity] = solutions
        first, second = list(arms)
        pair_results = []
        for a, b in pairs:
            accepted = None
            checked = 0
            for left, right in product(poses[first].get(a, []), poses[second].get(b, [])):
                if monotonic() >= deadline:
                    break
                checked += 1
                result = check_pair_linear(
                    arms[first].collision,
                    arms[second].collision,
                    left["positions"],
                    left["positions"],
                    right["positions"],
                    right["positions"],
                    margin_m=profile["environment"]["clearance_m"],
                    deadline=deadline,
                )
                if result["status"] == "CERTIFIED_INTER_ARM_INTERVAL":
                    accepted = {first: left["positions"], second: right["positions"]}
                    break
            pair_results.append(
                {"samples": [a, b], "configurations_checked": checked, "positions": accepted}
            )
        clear_rows = [r for arm_rows in rows.values() for r in arm_rows if r["clear_solutions"]]
        candidates.append(
            {
                "id": key,
                "label": label,
                "world_from_base": {
                    identity: arm.robot.world_from_base.tolist() for identity, arm in arms.items()
                },
                "arms": rows,
                "pose_count": sum(len(r) for r in rows.values()),
                "requested_pose_count": len(points) * len(arms),
                "clear_pose_count": len(clear_rows),
                "minimum_best_joint_margin": min(
                    (r["clear_solutions"][0]["joint_margin_fraction"] for r in clear_rows),
                    default=0,
                ),
                "pose_pairs": pair_results,
                "clear_pair_count": sum(r["positions"] is not None for r in pair_results),
            }
        )
    result = artifact(
        "cell_mounting_assessment",
        status="TIME_BUDGET" if monotonic() >= deadline else "SAMPLED_MOUNT_ASSESSMENT",
        simulation_only=True,
        executable=False,
        drawing_pen_tilt_deg=profile.get("drawing_pen_tilt_deg", 0.0),
        inputs={
            name: sha256(run / name)
            for name in ("pen_strokes.json", "cell_config.json", "cell_manifest.json")
        },
        candidates=candidates,
        tcp_samples_m=points.tolist(),
        limitations=[
            "Fixed three-option study; not a global placement optimum",
            "Static samples and pose pairs do not certify paths or the full build",
            "Tilted-base wedge, fasteners, cables, payload dynamics and gravity "
            "configuration remain unmodeled",
            "Pen and bracket dimensions are provisional",
        ],
    )
    write_json(run / "cell_mounting_assessment.json", result)
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--maximum-seconds", type=float, default=180)
    args = parser.parse_args()
    assessment = assess_mounting(args.run, args.config, args.maximum_seconds)
    for row in assessment["candidates"]:
        print(
            row["label"],
            row["clear_pose_count"],
            "/",
            row["pose_count"],
            "clear poses;",
            row["clear_pair_count"],
            "clear pairs",
        )
