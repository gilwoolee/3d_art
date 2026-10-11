"""Continuous inter-arm clearance for paths evaluated on one shared clock.

Callers supply valid joint paths and per-joint midpoint displacement bounds.
This checks moving bodies against moving bodies, including a stationary arm.
Single-arm self, environment, process and material validation remain required.
"""

from time import monotonic

import fcl
import numpy as np

from .collision import placed


def arm_objects(model, q):
    frames = model.robot.frames(q)
    return [(body, placed(body.geometry, frames[body.link] @ body.origin)) for body in model.bodies]


def check_pair_paths(
    first,
    second,
    first_position,
    second_position,
    first_deviation,
    second_deviation,
    *,
    margin_m=0.001,
    maximum_depth=12,
    deadline=float("inf"),
):
    """Certify all inter-arm body pairs on a normalized shared interval [0,1].

    A deviation callback bounds absolute joint displacement from that interval's
    midpoint configuration, including any interior turning points. The sum of
    each body's reach-weighted joint displacement bounds its relative motion.
    A colliding midpoint is evidence; insufficient clearance at maximum depth
    is unresolved, not evidence that no alternate schedule can exist.
    """
    if not np.isfinite(margin_m) or margin_m < 0 or maximum_depth < 0:
        raise ValueError("Invalid coordinated clearance settings")
    evaluations, deepest = 0, 0

    def inspect(t, bounds):
        nonlocal evaluations
        evaluations += 1
        configurations = (first_position(t), second_position(t))
        objects = (arm_objects(first, configurations[0]), arm_objects(second, configurations[1]))
        uncertain = []
        for left, left_object in objects[0]:
            for right, right_object in objects[1]:
                pair = {"first": left.name, "second": right.name}
                if fcl.collide(
                    left_object, right_object, fcl.CollisionRequest(), fcl.CollisionResult()
                ):
                    return {
                        "status": "COLLISION",
                        "parameter": t,
                        "positions": [np.asarray(q).tolist() for q in configurations],
                        "pairs": [pair],
                    }
                distance = fcl.distance(
                    left_object, right_object, fcl.DistanceRequest(), fcl.DistanceResult()
                )
                if not np.isfinite(distance) or distance < 0:
                    raise ValueError("Invalid inter-arm distance")
                displacement = float(
                    first.motion_radii[left.name] @ bounds[0]
                    + second.motion_radii[right.name] @ bounds[1]
                )
                if distance <= displacement + margin_m + 1e-6:
                    uncertain.append(
                        {**pair, "distance_m": float(distance), "motion_bound_m": displacement}
                    )
        return {
            "status": "CLEARANCE_UNRESOLVED" if uncertain else "CERTIFIED_INTER_ARM_INTERVAL",
            "parameter": t,
            "positions": [np.asarray(q).tolist() for q in configurations],
            "pairs": uncertain,
        }

    zero = [np.zeros(len(model.robot.names)) for model in (first, second)]
    for t in (0, 1):
        result = inspect(t, zero)
        if result["status"] != "CERTIFIED_INTER_ARM_INTERVAL":
            return {**result, "evaluations": evaluations, "maximum_depth": 0}

    def visit(a, b, depth):
        nonlocal deepest
        if monotonic() >= deadline:
            return {"status": "TIME_BUDGET"}
        deepest = max(deepest, depth)
        bounds = [np.asarray(fn(a, b), float) for fn in (first_deviation, second_deviation)]
        for value, model in zip(bounds, (first, second), strict=True):
            if (
                value.shape != (len(model.robot.names),)
                or not np.isfinite(value).all()
                or np.any(value < 0)
            ):
                raise ValueError(
                    "Joint displacement bounds must be finite, nonnegative and match DOF"
                )
        midpoint = (a + b) / 2
        result = inspect(midpoint, bounds)
        if result["status"] != "CLEARANCE_UNRESOLVED" or depth >= maximum_depth:
            return result
        left = visit(a, midpoint, depth + 1)
        return (
            visit(midpoint, b, depth + 1)
            if left["status"] == "CERTIFIED_INTER_ARM_INTERVAL"
            else left
        )

    result = visit(0, 1, 0)
    return {**result, "evaluations": evaluations, "maximum_depth": deepest}


def check_pair_linear(first, second, first_start, first_end, second_start, second_end, **kwargs):
    """Convenience wrapper for synchronized joint-linear moves and waits."""
    a, b, c, d = [np.asarray(q, float) for q in (first_start, first_end, second_start, second_end)]
    return check_pair_paths(
        first,
        second,
        lambda t: a * (1 - t) + b * t,
        lambda t: c * (1 - t) + d * t,
        lambda x, y: np.abs(b - a) * (y - x) / 2,
        lambda x, y: np.abs(d - c) * (y - x) / 2,
        **kwargs,
    )
