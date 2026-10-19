"""Sequence drawing paths with grounded growth, lifts and bounded joint-space travel.

This is a simulation preview. Approach/withdrawal and free travel check the arm
and an idealized tip sphere against deposited material. Only the tip receives a
local, explicit contact allowance at a stroke endpoint.
Physical pen/mount geometry and contact/process calibration remain required.
"""

from time import monotonic, perf_counter

import numpy as np

from ..kinematics import inverse_candidates, pose_error, rigid_transform
from .cell_motion import search_layers
from .collision import QUERY_STATS


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


def cartesian_lift(
    robot,
    offset,
    start,
    distance,
    settings,
    edge_valid,
    deadline=np.inf,
    direction=(0, 0, 1),
):
    """Depart along a Cartesian direction, retaining the contact configuration."""
    direction = np.asarray(direction, dtype=float)
    if (
        direction.shape != (3,)
        or not np.isfinite(direction).all()
        or np.linalg.norm(direction) == 0
    ):
        raise ValueError("Departure direction must be a finite nonzero 3D vector")
    direction = direction / np.linalg.norm(direction)
    start = np.asarray(start)
    initial = robot.forward(start, offset)
    count = max(1, int(np.ceil(distance / settings["lift_sample_m"])))
    targets = np.repeat(initial[None], count + 1, axis=0)
    targets[:, :3, 3] += np.linspace(0, distance, count + 1)[:, None] * direction

    def solve(target, seeds):
        if np.array_equal(target, initial):
            return [start]
        return [
            row["positions"]
            for row in inverse_candidates(
                robot,
                target,
                offset,
                seeds,
                position_tolerance=0.00005,
                orientation_tolerance=0.001,
                maximum_evaluations=60,
            )
        ]

    tracking_rejections = []

    def check(a, b, p0, p1):
        bound = tracking_bounds(robot, offset, a, b, p0, p1)
        if (
            bound["position_error_bound_m"] > settings["lift_position_tolerance_m"]
            or bound["orientation_error_bound_rad"] > settings["lift_orientation_tolerance_rad"]
        ):
            if len(tracking_rejections) < 20:
                tracking_rejections.append(bound)
            return None
        if not edge_valid(a, b):
            return None
        return bound["position_error_bound_m"]

    result = search_layers(targets, solve, check, [start], beam_width=2, deadline=deadline)
    return {**result, "tracking_rejections": tracking_rejections}


def find_departure(robot, offset, start, settings, edge_valid, deadline=np.inf):
    """Try upward and oblique exits without weakening any collision constraints."""
    attempts = []
    for direction in settings["departure_directions"]:
        unit = np.asarray(direction, dtype=float)
        unit /= np.linalg.norm(unit)
        # Refine the same Cartesian ray only after a tracking-bound rejection.
        # Every new joint edge still needs the original tracking and collision
        # checks; neither a collision nor missing IK alone triggers refinement.
        for subdivisions in (1, 2, 4, 8, 16, 32, 64):
            if hasattr(edge_valid, "failures"):
                edge_valid.failures.clear()
            local_settings = {
                **settings,
                "lift_sample_m": settings["lift_sample_m"] / subdivisions,
            }
            result = cartesian_lift(
                robot,
                offset,
                start,
                settings["withdraw_distance_m"],
                local_settings,
                edge_valid,
                deadline,
                direction=direction,
            )
            attempts.append(
                {
                    "direction": unit.tolist(),
                    "status": result["status"],
                    "failed_sample": result.get("failed_sample"),
                    "lift_sample_m": local_settings["lift_sample_m"],
                    "tracking_rejections": result["tracking_rejections"],
                    "collision_evidence": list(getattr(edge_valid, "failures", [])),
                }
            )
            if result["status"] == "SAMPLED_DRAWING_PATH":
                return {**result, "direction": unit.tolist(), "attempts": attempts}
            if result["status"] == "TIME_BUDGET":
                return {**result, "attempts": attempts}
            if not result["tracking_rejections"]:
                break
    return {"status": "NO_DEPARTURE_FOUND", "attempts": attempts}


def local_contact_surfaces(contact_z, plate_z, table_z, radius):
    """Surfaces intersecting the permitted local contact region, in robot Z."""
    surfaces = [("plate", plate_z), ("table", table_z)]
    return [name for name, height in surfaces if abs(contact_z - height) <= radius]


def contact_validator(
    robot,
    arm,
    tip_model,
    offset,
    material,
    obstacles,
    contact,
    contact_radius,
    tip_radius,
    margin,
    maximum_depth,
    contact_surfaces=(),
    deadline=float("inf"),
    optimize_contact_geometry=None,
):
    """Keep full arm checks; permit tip contact only inside a local join ball."""
    clipped = material.contact_view(contact, contact_radius, local=optimize_contact_geometry)
    failures = []

    def rejected(scope, result):
        if len(failures) < 20:
            failures.append({"scope": scope, **result})
        return False

    def valid(a, b, depth=0):
        if depth:
            return check(a, b, depth)
        # Nested time: the arm and tip checks inside are also counted as
        # self/static and material seconds.
        started = perf_counter()
        try:
            return check(a, b, 0)
        finally:
            QUERY_STATS["contact_seconds"] += perf_counter() - started

    def check(a, b, depth):
        if monotonic() >= deadline:
            return rejected("contact", {"status": "TIME_BUDGET"})
        arm_result = arm.check_edge(
            a, b, obstacles, margin, maximum_depth, material, deadline=deadline
        )
        if arm_result["status"] != "CERTIFIED_MODELED_EDGE":
            return rejected("arm", arm_result)
        a, b = np.asarray(a), np.asarray(b)
        middle = (a + b) / 2
        point = robot.forward(middle, offset)[:3, 3]
        travel_bound = float(tip_model.motion_radii["virtual_tip"] @ (np.abs(b - a) / 2))
        entirely_local = np.linalg.norm(point - contact) + travel_bound + tip_radius + margin <= (
            contact_radius
        )
        scene = (
            {name: body for name, body in obstacles.items() if name not in contact_surfaces}
            if entirely_local
            else obstacles
        )
        result = tip_model.check_edge(
            a, b, scene, margin, maximum_depth, clipped, deadline=deadline
        )
        if result["status"] == "CERTIFIED_MODELED_EDGE":
            return True
        if result["status"] == "TIME_BUDGET":
            return rejected("tip", result)
        # A witnessed collision outside any potentially exempt work surface
        # cannot disappear by subdividing the same joint path.
        if result["status"] == "COLLISION" and not any(
            pair["second"] in contact_surfaces for pair in result["pairs"]
        ):
            return rejected("tip", result)
        if depth >= 8:
            return rejected("tip", result)
        return check(a, middle, depth + 1) and check(middle, b, depth + 1)

    valid.failures = failures
    return valid
