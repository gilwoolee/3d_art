"""Conservative TCP error bounds for linear joint segments and fixed orientation."""

import numpy as np

from ..kinematics import pose_error, rigid_transform


def tracking_bounds(robot, offset, q0, q1, target0, target1):
    """Bound TCP error for every point of one linearly interpolated joint edge.

    A transformed link translation has second-derivative norm bounded by its
    length times the square of the sum of upstream joint angle changes. Sum
    those contributions and the final TCP offset. Linear interpolation error
    is at most max(endpoint error) + max(second derivative)/8 on [0, 1].

    The same matrix-derivative bound applies to fixed target orientation in
    operator norm. For rotations, ||R-S||_2 = 2 sin(angle(R S^T)/2). This is a
    geometric bound, not a controller tracking or calibration guarantee.
    """
    offset = rigid_transform(offset)
    target0, target1 = rigid_transform(target0), rigid_transform(target1)
    if not np.allclose(target0[:3, :3], target1[:3, :3], atol=1e-12, rtol=0):
        raise ValueError("Tracking bound requires constant target orientation")
    q0, q1 = np.asarray(q0), np.asarray(q1)
    actual0, actual1 = robot.forward(q0, offset), robot.forward(q1, offset)
    endpoint_errors = [pose_error(a, t) for a, t in [(actual0, target0), (actual1, target1)]]
    omega, acceleration = 0.0, 0.0
    for joint in robot.joint_data:
        acceleration += float(np.linalg.norm(joint["origin"][:3, 3])) * omega**2
        if joint["index"] is not None:
            omega += abs(float(q1[joint["index"]] - q0[joint["index"]]))
    acceleration += float(np.linalg.norm(offset[:3, 3])) * omega**2
    position = max(float(np.linalg.norm(error[:3])) for error in endpoint_errors)
    orientation = max(float(np.linalg.norm(error[3:])) for error in endpoint_errors)
    matrix_error = 2 * np.sin(orientation / 2) + omega**2 / 8
    return {
        "position_error_bound_m": position + acceleration / 8 + 1e-12,
        "orientation_error_bound_rad": float(2 * np.arcsin(min(1.0, matrix_error / 2))) + 1e-12,
    }
