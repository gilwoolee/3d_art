"""Sequence drawing paths with grounded growth, lifts and bounded joint-space travel.

This is a simulation preview. Approach/withdrawal and free travel check the arm
and an idealized tip sphere against deposited material. Only the tip receives a
local, explicit contact allowance at a stroke endpoint.
Physical pen/mount geometry and contact/process calibration remain required.
"""

from time import monotonic, perf_counter

import numpy as np

from ..kinematics import inverse_candidates
from .collision import QUERY_STATS
from .curve_motion import search_layers
from .tracking import tracking_bounds


def cartesian_lift(
    robot,
    offset,
    start,
    distance,
    settings,
    edge_valid,
    deadline=np.inf,
    solve_candidates=None,
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
        if solve_candidates:
            return solve_candidates(target, seeds)
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


def sampled_operation(kind, positions, robot, offset, step, **metadata):
    """Subdivide joint-linear edges for playback without changing their geometry."""
    positions = np.asarray(positions)
    expanded = [positions[0]]
    for a, b in zip(positions[:-1], positions[1:], strict=True):
        count = max(1, int(np.ceil(np.max(np.abs(b - a)) / step)))
        expanded.extend(a + (b - a) * t for t in np.linspace(0, 1, count + 1)[1:])
    points = [robot.forward(q, offset)[:3, 3].tolist() for q in expanded]
    return {
        "kind": kind,
        "positions": [q.tolist() for q in expanded],
        "tcp_path_m": points,
        "extrusion_enabled": kind == "DRAW",
        "tcp_length_m": float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()),
        **metadata,
    }


def local_contact_surfaces(
    contact_z, table_z, platform_z, radius, *, riser_z=None, riser_name="build_riser"
):
    """Surfaces intersecting the permitted local contact region, in robot Z."""
    surfaces = [("platform", platform_z), ("table", table_z)]
    if riser_z is not None:
        surfaces.append((riser_name, riser_z))
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


def completed_prefix(operations, stroke_ids, allow_reordered=False):
    """Resume only through a completed withdrawal; discard an unfinished action."""
    last = max((i for i, op in enumerate(operations) if op["kind"] == "WITHDRAW"), default=-1)
    prefix = operations[: last + 1]
    if len(prefix) % 4 or len(prefix) // 4 > len(stroke_ids):
        raise ValueError("Malformed saved sequence prefix")
    visited = set()
    for i in range(0, len(prefix), 4):
        group = prefix[i : i + 4]
        identity = group[0]["stroke_id"]
        expected = identity if allow_reordered else stroke_ids[i // 4]
        if (
            identity not in stroke_ids
            or identity in visited
            or [op["kind"] for op in group] != ["TRAVEL", "APPROACH", "DRAW", "WITHDRAW"]
            or any(op["stroke_id"] != expected for op in group)
        ):
            raise ValueError("Saved sequence order does not match the drawing paths")
        visited.add(identity)
    for a, b in zip(prefix[:-1], prefix[1:], strict=True):
        if not np.allclose(a["positions"][-1], b["positions"][0], atol=1e-9, rtol=0):
            raise ValueError("Saved sequence has a discontinuity")
    return prefix, len(prefix) // 4
