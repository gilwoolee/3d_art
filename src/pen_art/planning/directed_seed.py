"""One geometry-derived pen posture seed, without pose grids or acceptance claims."""

import numpy as np

from ..kinematics import inverse_candidates


def material_away_seed(arm, point, previous, material, upward_component=0.4):
    """Tilt away from the nearest deposited strand and solve one fixed-pose IK.

    The build's gravity is world Z. The shared upward component preserves room
    above the bed while the horizontal direction comes from actual material.
    ``material`` should exclude the prospective guide when available, so a
    centerline segment ending at the tip does not choose the direction itself.
    This only supplies a seed; collision-constrained correction must follow.
    """
    if not 0 < upward_component < 1:
        raise ValueError("Upward component must lie strictly between zero and one")
    if not material.segments:
        return {"status": "NO_MATERIAL_FOR_DIRECTED_SEED"}
    point, previous = np.asarray(point), np.asarray(previous)
    starts = np.asarray([row["start"] for row in material.segments])
    ends = np.asarray([row["end"] for row in material.segments])
    delta = ends - starts
    fractions = np.clip(
        np.sum((point - starts) * delta, axis=1)
        / np.maximum(np.sum(delta * delta, axis=1), 1e-20),
        0, 1,
    )
    nearest_points = starts + fractions[:, None] * delta
    offsets = point - nearest_points
    horizontal = offsets.copy()
    horizontal[:, 2] = 0
    lengths = np.linalg.norm(horizontal, axis=1)
    valid = np.flatnonzero(lengths > 1e-6)
    if not len(valid):
        return {"status": "NO_HORIZONTAL_MATERIAL_DIRECTION"}
    index = int(valid[np.argmin(np.linalg.norm(offsets[valid], axis=1))])
    away = horizontal[index] / lengths[index]
    axis = np.sqrt(1 - upward_component**2) * away + [0, 0, upward_component]
    z = -axis
    prior = arm.robot.forward(previous, arm.flange_to_tcp)[:3, :3]
    x = prior[:, 0] - np.dot(prior[:, 0], z) * z
    if np.linalg.norm(x) < 1e-8:
        x = prior[:, 1] - np.dot(prior[:, 1], z) * z
    x /= np.linalg.norm(x)
    target = np.eye(4)
    target[:3, :3] = np.column_stack([x, np.cross(z, x), z])
    target[:3, 3] = point
    report = {
        "status": "NO_IK_FOR_DIRECTED_SEED",
        "body_axis": axis.tolist(),
        "nearest_material": material.segments[index]["name"],
        "nearest_centerline_m": float(np.linalg.norm(offsets[index])),
        "upward_component": upward_component,
        "method": "nearest_material_away_axis_with_previous_roll_projection",
    }
    rows = inverse_candidates(
        arm.robot, target, arm.flange_to_tcp, [previous],
        position_tolerance=1e-6, orientation_tolerance=0.001, maximum_evaluations=150,
    )
    if rows:
        report.update(status="IK_SEED_ONLY", positions=rows[0]["positions"])
    return report
