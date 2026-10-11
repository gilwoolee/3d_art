"""Bounded redundant-arm IK; solutions are candidates, not collision-free motions."""

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def rigid_transform(value, name="transform"):
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError(f"{name} must be a finite 4x4 matrix")
    if (
        not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-9)
        or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-8)
        or not np.isclose(np.linalg.det(matrix[:3, :3]), 1.0, atol=1e-8)
    ):
        raise ValueError(f"{name} must be a rigid, right-handed transform")
    return matrix


def pose_error(actual, target):
    return np.r_[
        actual[:3, 3] - target[:3, 3],
        Rotation.from_matrix(actual[:3, :3] @ target[:3, :3].T).as_rotvec(),
    ]


def geometric_jacobian(robot, positions, flange_to_tcp):
    """World-frame translational/angular Jacobian evaluated at the pen tip."""
    frames = robot.frames(positions)
    tcp = (frames[robot.tip] @ rigid_transform(flange_to_tcp))[:3, 3]
    columns = []
    for joint in robot.joint_data:
        if joint["index"] is None:
            continue
        frame = frames[joint["parent"]] @ joint["origin"]
        axis = frame[:3, :3] @ joint["axis"]
        columns.append(np.r_[np.cross(axis, tcp - frame[:3, 3]), axis])
    return np.array(columns).T


def pose_jacobian(robot, positions, offset, target):
    """Derivative of world translation and SO(3) log residuals."""
    jacobian = geometric_jacobian(robot, positions, offset)
    vector = pose_error(robot.forward(positions, offset), target)[3:]
    angle = np.linalg.norm(vector)
    x, y, z = vector
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    coefficient = (
        (1 / 12 + angle**2 / 720)
        if angle < 1e-4
        else (1 - 0.5 * angle / np.tan(0.5 * angle)) / angle**2
    )
    inverse_left = np.eye(3) - 0.5 * skew + coefficient * (skew @ skew)
    jacobian[3:] = inverse_left @ jacobian[3:]
    return jacobian


def inverse_candidates(
    robot,
    target,
    flange_to_tcp,
    seeds,
    position_tolerance=0.0005,
    orientation_tolerance=0.005,
    maximum_evaluations=150,
):
    """Retain distinct solutions from several seeds within the URDF joint limits.

    No collision or singularity acceptance is implied by pose convergence. Full
    paths must still validate interpolation, tools, and growing printed material.
    """
    target = rigid_transform(target, "target TCP pose")
    offset = rigid_transform(flange_to_tcp, "flange-to-TCP")
    if position_tolerance <= 0 or orientation_tolerance <= 0:
        raise ValueError("IK tolerances must be positive")
    solutions = []
    weights = np.array([1 / position_tolerance] * 3 + [1 / orientation_tolerance] * 3)
    for seed in seeds:
        seed = np.asarray(seed, dtype=float)
        if seed.shape != robot.lower.shape or not np.isfinite(seed).all():
            raise ValueError("IK seed must match the robot's finite joint vector")
        seed = np.clip(seed, robot.lower + 1e-9, robot.upper - 1e-9)

        def residual(q, reference=seed):
            # A weak redundant-posture preference preserves alternatives without
            # replacing pose accuracy by a joint-distance objective.
            return np.r_[
                pose_error(robot.forward(q, offset), target) * weights, 1e-4 * (q - reference)
            ]

        def jacobian(q):
            return np.vstack(
                [pose_jacobian(robot, q, offset, target) * weights[:, None], 1e-4 * np.eye(len(q))]
            )

        result = least_squares(
            residual,
            seed,
            jac=jacobian,
            bounds=(robot.lower, robot.upper),
            max_nfev=maximum_evaluations,
            xtol=1e-9,
            ftol=1e-9,
            gtol=1e-8,
        )
        error = pose_error(robot.forward(result.x, offset), target)
        if (
            np.linalg.norm(error[:3]) > position_tolerance
            or np.linalg.norm(error[3:]) > orientation_tolerance
        ):
            continue
        if any(np.linalg.norm(result.x - np.array(s["positions"])) < 1e-3 for s in solutions):
            continue
        jacobian = geometric_jacobian(robot, result.x, offset)
        solutions.append(
            {
                "positions": result.x.tolist(),
                "position_error_m": float(np.linalg.norm(error[:3])),
                "orientation_error_rad": float(np.linalg.norm(error[3:])),
                "jacobian_rank": int(np.linalg.matrix_rank(jacobian)),
                "collision_checked": False,
            }
        )
    return solutions
