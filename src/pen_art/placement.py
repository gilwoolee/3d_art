"""Provisional table/platform placement search; sampled IK is not a motion plan."""

import argparse
from itertools import product
from pathlib import Path

import numpy as np
import yaml

from .curves import curve_coordinate_bounds, sample_curve, transform_points
from .io import artifact, read_json, sha256, write_json
from .kinematics import geometric_jacobian, inverse_candidates, rigid_transform
from .robot import SerialArm


def build_transform(bounds, center, height):
    """Center the footprint at XY, keeping the build plane at platform height."""
    transform = np.eye(4)
    transform[:2, 3] = np.asarray(center) - np.mean(bounds, axis=0)[:2]
    transform[2, 3] = height
    return transform


def probe_poses(robot, points, transform, offset, rotation, initial):
    seed = np.asarray(initial)
    records = []
    for point in transform_points(points, transform):
        target = np.eye(4)
        target[:3, :3], target[:3, 3] = rotation, point
        solutions = inverse_candidates(robot, target, offset, [seed], maximum_evaluations=60)
        if not solutions:
            solutions = inverse_candidates(
                robot, target, offset, [initial], maximum_evaluations=100
            )
        if not solutions:
            records.append({"tcp_m": point.tolist(), "ik_found": False})
            continue
        seed = np.asarray(solutions[0]["positions"])
        margin = np.min(
            np.minimum(seed - robot.lower, robot.upper - seed) / (robot.upper - robot.lower)
        )
        singular = np.linalg.svd(geometric_jacobian(robot, seed, offset), compute_uv=False)[-1]
        records.append(
            {
                "tcp_m": point.tolist(),
                "ik_found": True,
                "positions": seed.tolist(),
                "joint_limit_margin_fraction": float(margin),
                "jacobian_min_singular_value_mixed_units": float(singular),
            }
        )
    success = [r for r in records if r["ik_found"]]
    return {
        "samples": records,
        "coverage": len(success) / len(records),
        "minimum_joint_margin": min(
            (r["joint_limit_margin_fraction"] for r in success), default=0.0
        ),
    }


def search(run, config):
    settings = yaml.safe_load(config.read_text())
    curves = read_json(run / "pen_strokes.json")["curves"]
    robot = SerialArm(run / "robot.urdf")
    manifest = read_json(run / "robot_manifest.json")
    initial = manifest["initial_positions"]
    offset = rigid_transform(settings["flange_to_tcp"])
    rotation = np.asarray(settings["tcp_rotation"])
    pose = np.eye(4)
    pose[:3, :3] = rotation
    rigid_transform(pose)
    curve_bounds = np.array([curve_coordinate_bounds(c) for c in curves])
    bounds = np.array([curve_bounds[:, 0].min(axis=0), curve_bounds[:, 1].max(axis=0)])
    # Corners, edges, face centers and interior cover the full bounding volume.
    probes = np.array(list(product(*[np.linspace(a, b, 3) for a, b in zip(*bounds, strict=True)])))
    candidates = []
    for x, y, height in product(
        settings["platform_centers_x_m"],
        settings["platform_centers_y_m"],
        settings["platform_heights_above_table_m"],
    ):
        if height < 0:
            raise ValueError("Platform height cannot be negative")
        transform = build_transform(bounds, [x, y], settings["table_z_m"] + height)
        result = probe_poses(robot, probes, transform, offset, rotation, initial)
        candidates.append(
            {
                "center_xy_m": [x, y],
                "height_above_table_m": height,
                "build_to_robot_base": transform.tolist(),
                **result,
            }
        )
    # Coverage first; keep away from joint limits second. This is a finite-grid ranking.
    candidates.sort(key=lambda c: (c["coverage"], c["minimum_joint_margin"]), reverse=True)
    best = candidates[0]
    samples = np.concatenate([sample_curve(c, 8) for c in curves])
    voxel = float(settings["curve_sample_voxel_m"])
    if voxel <= 0:
        raise ValueError("Curve sample voxel must be positive")
    _, indices = np.unique(np.floor(samples / voxel).astype(int), axis=0, return_index=True)
    verification = probe_poses(
        robot,
        samples[np.sort(indices)],
        np.array(best["build_to_robot_base"]),
        offset,
        rotation,
        initial,
    )
    result = artifact(
        "placement_search",
        status="PROVISIONAL_SAMPLED_IK",
        simulation_only=True,
        robot_variant=manifest["variant"],
        assumptions=settings,
        inputs={name: sha256(run / name) for name in ["pen_strokes.json", "robot.urdf"]},
        config_sha256=sha256(config),
        selected=best,
        candidates=candidates,
        curve_sample_verification=verification,
        collision_checked=False,
        continuous_motion_verified=False,
        limitations=[
            "Finite placement grid; not a global optimum.",
            "Independent sampled IK does not establish a continuous path.",
            "Table, riser, pen, mount and printed-material collisions pending.",
            "Arm variant and tool transform are unconfirmed.",
        ],
    )
    write_json(run / "placement_search.json", result)
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, default=Path("runs/palace_15cm"))
    parser.add_argument("--config", type=Path, default=Path("configs/placement.yaml"))
    args = parser.parse_args()
    result = search(args.run, args.config)
    print(
        {
            key: result["selected"][key]
            for key in ["center_xy_m", "height_above_table_m", "coverage"]
        }
    )
    print("Curve sample coverage:", result["curve_sample_verification"]["coverage"])
