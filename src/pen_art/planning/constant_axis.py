"""Finite whole-contour pen-orientation screening before free-axis repair."""

from itertools import product

import numpy as np
from scipy.spatial.transform import Rotation

from ..kinematics import inverse_candidates
from .axis_repair import optimization_distance
from .collision import placed


def constant_axis_candidates(planner, drawing, material):
    """Keep rigid pen orientations clear over the complete growing contour.

    These are geometric proposals, not robot motion certificates. The caller
    still checks IK, continuous tracking, robot collision and actual deposition.
    """
    arm = planner.arms[drawing["arm"]]
    targets = np.asarray(drawing["tcp_targets_m"])
    original = np.asarray(drawing["tcp_rotation"])
    rotations = [original]
    for direction in product((-1, 0, 1), repeat=3):
        z = np.asarray(direction, dtype=float)
        if np.linalg.norm(z) == 0:
            continue
        z /= np.linalg.norm(z)
        x = original[:, 0] - z * np.dot(original[:, 0], z)
        if np.linalg.norm(x) < 1e-6:
            x = original[:, 1] - z * np.dot(original[:, 1], z)
        x /= np.linalg.norm(x)
        frame = np.column_stack([x, np.cross(z, x), z])
        rotations.extend(
            frame @ Rotation.from_rotvec([0, 0, roll]).as_matrix() for roll in (0, np.pi)
        )
    inverse = np.linalg.inv(arm.flange_to_tcp)
    scene = material.copy()
    clearance = planner.margin + 0.0001
    radius = planner.actions[drawing["stroke_id"]]["diameter_m"] / 2 + 0.00002
    for index, point in enumerate(targets):
        scene.add(targets[max(0, index - 1)], point, radius, f"constant-axis/{index}")
        clear = []
        for rotation in rotations:
            target = np.eye(4)
            target[:3, :3], target[:3, 3] = rotation, point
            flange = target @ inverse
            feasible = True
            for body in arm.tool.bodies:
                transform = flange @ body.origin
                obj = placed(body.geometry, transform)
                if any(
                    optimization_distance(obj, other) < clearance
                    for other in planner.obstacles.values()
                ):
                    feasible = False
                    break
                if scene.possibly_near_body(obj, clearance):
                    nearest = scene.nearest(obj, bounding_sphere=(transform[:3, 3], body.radius))
                    if nearest and nearest["distance_m"] < clearance:
                        feasible = False
                        break
            if feasible:
                clear.append(rotation)
        rotations = clear
        if not rotations:
            break
    return sorted(
        rotations,
        key=lambda rotation: np.linalg.norm(
            Rotation.from_matrix(rotation @ original.T).as_rotvec()
        ),
    )


def constant_axis_path(planner, drawing, material, rotation):
    """Follow a screened fixed orientation with continuous seeded IK."""
    arm = planner.arms[drawing["arm"]]
    scene = material.copy()
    radius = planner.actions[drawing["stroke_id"]]["diameter_m"] / 2 + 0.00002
    points = np.asarray(drawing["tcp_targets_m"])
    positions, previous = [], np.asarray(drawing["positions"][0])
    for index, point in enumerate(points):
        scene.add(points[max(0, index - 1)], point, radius, f"constant-axis/{index}")
        target = np.eye(4)
        target[:3, :3], target[:3, 3] = rotation, point
        seeds = [previous, drawing["positions"][index]]
        if index == 0:
            seeds.append(arm.initial_positions)
        solutions = inverse_candidates(
            arm.robot,
            target,
            arm.flange_to_tcp,
            seeds,
            maximum_evaluations=100,
            position_tolerance=0.00005,
            orientation_tolerance=0.001,
        )
        good = [
            np.asarray(row["positions"])
            for row in solutions
            if not arm.collision.check(
                row["positions"], planner.obstacles, planner.margin + 0.00002, scene
            )
        ]
        if not good:
            return None
        chosen = min(good, key=lambda q: np.linalg.norm(q - previous))
        if index and np.max(abs(chosen - previous)) > 0.2:
            return None
        positions.append(chosen.tolist())
        previous = chosen
    return {
        "status": "SAMPLED_FREE_AXIS_PROPOSAL",
        "positions": positions,
        "tcp_targets_m": points.tolist(),
        "free_tool_orientation": True,
        "constant_tcp_rotation": np.asarray(rotation).tolist(),
        "scope": "Whole-contour orientation and seeded IK; full continuous validation required",
    }
