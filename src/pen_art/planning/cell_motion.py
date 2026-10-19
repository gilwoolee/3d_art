"""Drawing paths for one arm: IK along a stroke at a fixed pen orientation.

These are drawing-path candidates, not schedules: transits and checks against
the growing material belong to the planner and the final replay.
"""

from time import monotonic

import fcl
import numpy as np
from scipy.spatial.transform import Rotation

from ..curves import flatten_curve, orient_stroke, transform_points
from ..kinematics import inverse_candidates, pose_error
from .collision import placed

DEFAULT_SETTINGS = {
    "maximum_chord_m": 0.003,
    "flatten_tolerance_m": 0.0001,
    "ik_position_tolerance_m": 0.00005,
    "ik_orientation_tolerance_rad": 0.001,
    "interpolation_tolerance_m": 0.0003,
    "interpolation_tolerance_rad": 0.003,
    "maximum_joint_step_rad": 0.2,
    "beam_width": 2,
    "maximum_subdivision_depth": 12,
}


def pen_rotation(arm, world_from_build, roll=0.0, tilt_deg=0.0, lean_azimuth_deg=None):
    """Fixed inward tip axis, keeping the pen body on its robot's side.

    With ``lean_azimuth_deg`` the pen body leans toward that world azimuth
    instead, so the arm can draw the side of the object facing away from it.

    Zero tilt keeps the nozzle vertical in the shared world, even on a tilted base.
    Horizontal bases preserve their existing downward orientation exactly.
    The policy is a pose prescription, not a clearance or process certificate.
    """
    if not np.isfinite(tilt_deg) or not 0 <= tilt_deg < 90:
        raise ValueError("Drawing pen tilt must be finite and in [0, 90) degrees")
    base = arm.robot.world_from_base
    down = base[:3, :3] @ Rotation.from_rotvec([0, 0, roll]).as_matrix() @ np.diag([-1, 1, -1])
    if tilt_deg == 0 and np.allclose(base[:3, 2], [0, 0, 1], atol=1e-12, rtol=0):
        return down
    inward = np.asarray(world_from_build)[:2, 3] - base[:2, 3]
    length = np.linalg.norm(inward)
    if length < 1e-9 and tilt_deg != 0:
        raise ValueError("Inward pen tilt requires a base offset from the build center")
    angle = np.deg2rad(tilt_deg)
    if lean_azimuth_deg is not None:
        heading = np.deg2rad(lean_azimuth_deg)
        inward, length = -np.array([np.cos(heading), np.sin(heading)]), 1.0
    z = (
        np.array([0.0, 0.0, -1.0])
        if tilt_deg == 0
        else np.r_[np.sin(angle) * inward / length, -np.cos(angle)]
    )
    reference = base[:3, :3] @ np.diag([-1, 1, -1])
    x = reference[:, 0] - np.dot(reference[:, 0], z) * z
    if np.linalg.norm(x) < 1e-9:
        x = reference[:, 1] - np.dot(reference[:, 1], z) * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z]) @ Rotation.from_rotvec([0, 0, -roll]).as_matrix()


DEFAULT_PEN_TILTS_DEG = (70.0, 80.0)


def fixed_rotation_families(
    arm,
    world_from_build,
    tilt_degrees=DEFAULT_PEN_TILTS_DEG,
    roll_angles=(0.0, np.pi),
    *,
    lean_azimuth_deg=None,
):
    """Choose a complete TCP frame before drawing, then keep it unchanged."""
    for tilt in dict.fromkeys(float(value) for value in tilt_degrees):
        for roll in roll_angles:
            yield {
                "orientation_policy": "fixed",
                "fixed_orientation": True,
                "pen_tilt_deg": tilt,
                "roll_rad": float(roll),
                "rotations": pen_rotation(arm, world_from_build, roll, tilt, lean_azimuth_deg),
            }


def anchored_strokes(stroke, available):
    """Re-root closed contours, or reverse open contours, at existing anchors."""
    if stroke["closed"]:
        for index, visit in enumerate(stroke["junction_visits"][:-1]):
            if visit["node"] in available:
                for reverse in (False, True):
                    yield orient_stroke(stroke, reverse=reverse, start_visit=index)
    else:
        upward_first = stroke["segments"][0][0][2] > stroke["segments"][-1][-1][2]
        for reverse in (upward_first, not upward_first):
            start = stroke["end"] if reverse else stroke["start"]
            if start in available:
                yield orient_stroke(stroke, reverse=reverse)


def drawing_alternatives(
    arm,
    stroke,
    available,
    world_from_build,
    obstacles,
    *,
    deadline,
    rotations,
    seeds=None,
    settings=None,
    clearance_m=0.001,
    orientation_policy="fixed",
):
    """Yield actual IK paths, including every retained endpoint backtrace.

    ``rotations`` are pen orientations as ``fixed_rotation_families`` yields
    them; each keeps one TCP frame (tilt, azimuth and roll) for the whole path.
    Collision checks here cover self and static environment only. Every path is
    rechecked against the growing material when scheduled. Feasibility checks
    never split the source into separate pen actions.
    """
    settings = {**DEFAULT_SETTINGS, **(settings or {})}
    seeds = [arm.initial_positions] if seeds is None else seeds
    for oriented in anchored_strokes(stroke, available):
        points, _ = flatten_curve(
            oriented, settings["flatten_tolerance_m"], settings["maximum_chord_m"]
        )
        points = transform_points(points, world_from_build)
        for roll_index, descriptor in enumerate(rotations):
            if monotonic() >= deadline:
                return
            rotation = np.asarray(descriptor["rotations"])
            if orientation_policy not in {"fixed", "fixed_axis"} or rotation.ndim != 2:
                raise ValueError("Fixed drawing orientation requires one constant TCP rotation")
            diagnostics = {
                "ik_calls": 0,
                "ik_solutions": 0,
                "static_rejections": 0,
                "tracking_rejections": 0,
                "clearance_rejections": 0,
            }
            targets = np.repeat(np.eye(4)[None], len(points), axis=0)
            targets[:, :3, :3], targets[:, :3, 3] = rotation, points

            def solve(target, previous, diagnostics=diagnostics):
                if monotonic() >= deadline:
                    return []
                diagnostics["ik_calls"] += 1
                positions = [
                    s["positions"]
                    for s in inverse_candidates(
                        arm.robot,
                        target,
                        arm.flange_to_tcp,
                        previous,
                        position_tolerance=settings["ik_position_tolerance_m"],
                        orientation_tolerance=settings["ik_orientation_tolerance_rad"],
                        maximum_evaluations=60,
                    )
                ]
                diagnostics["ik_solutions"] += len(positions)
                diagnostics["last_target_ik_solutions"] = len(positions)
                accepted = []
                for position in positions:
                    contacts = arm.collision.check(position, obstacles, clearance_m)
                    if contacts:
                        diagnostics["static_rejections"] += 1
                        diagnostics["last_static_contacts"] = contacts
                    else:
                        accepted.append(position)
                diagnostics["last_target_collision_free_solutions"] = len(accepted)
                return accepted

            def edge_check(q0, q1, p0, p1, diagnostics=diagnostics):
                error = tracking_edge(
                    arm.robot,
                    arm.flange_to_tcp,
                    q0,
                    q1,
                    p0,
                    p1,
                    settings["interpolation_tolerance_m"],
                    settings["interpolation_tolerance_rad"],
                    settings["maximum_joint_step_rad"],
                )
                if error is None:
                    diagnostics["tracking_rejections"] += 1
                    diagnostics["last_tracking_failure"] = {
                        "maximum_joint_step_rad": float(np.max(np.abs(np.asarray(q1) - q0))),
                        "from_q": np.asarray(q0).tolist(),
                        "to_q": np.asarray(q1).tolist(),
                        "from_tcp_m": p0[:3, 3].tolist(),
                        "to_tcp_m": p1[:3, 3].tolist(),
                    }
                    return None
                check = arm.collision.check_edge(
                    q0,
                    q1,
                    obstacles,
                    clearance_m,
                    settings["maximum_subdivision_depth"],
                    deadline=deadline,
                )
                if check["status"] != "CERTIFIED_MODELED_EDGE":
                    diagnostics["clearance_rejections"] += 1
                    diagnostics["last_clearance_failure"] = check
                    return None
                return error

            result = search_layers(
                targets,
                solve,
                edge_check,
                seeds,
                settings["beam_width"],
                deadline,
            )
            base = {
                "arm": arm.identity,
                "stroke_id": stroke["id"],
                "start_anchor": oriented["start"],
                "end_anchor": oriented["end"],
                "reverse": oriented["reverse"],
                "start_visit": oriented["start_visit"],
                "roll_rad": float(descriptor.get("roll_rad", 0.0)),
                "pen_tilt_deg": float(descriptor["pen_tilt_deg"]),
                "orientation_policy": descriptor.get("orientation_policy", "fixed_axis"),
                "fixed_orientation": True,
                "tcp_rotation": rotation.tolist(),
                "tcp_targets_m": points.tolist(),
                "sampling_diagnostics": diagnostics,
            }
            if result["status"] != "SAMPLED_DRAWING_PATH":
                yield {**base, **result}
                continue
            for index, path in enumerate(result["path_alternatives"]):
                yield {
                    **base,
                    **path,
                    "status": "SAMPLED_DRAWING_PATH",
                    "variant": (
                        f"anchor-{oriented['start_visit']}-reverse-{int(oriented['reverse'])}"
                        f"-roll-{roll_index}-branch-{index}"
                    ),
                    "static_collision_status": "CERTIFIED_MODELED_PATH",
                    "tracking_status": "SAMPLED_ONLY",
                }


def search_layers(targets, solve, edge_check, initial_seeds, beam_width=3, deadline=np.inf):
    """Retain alternate configurations, then backtrace the lowest-cost full path.

    Each target is solved using every retained previous configuration. Endpoint
    reachability alone is insufficient: edge_check must accept each connection.
    """
    if beam_width < 1:
        raise ValueError("beam_width must be positive")
    layers = []
    seeds = [np.asarray(seed) for seed in initial_seeds]
    for index, target in enumerate(targets):
        if monotonic() >= deadline:
            return {"status": "TIME_BUDGET", "failed_sample": index}
        candidates = solve(target, seeds)
        nodes = []
        for q in candidates:
            q = np.asarray(q)
            if index == 0:
                cost = min(float(np.linalg.norm(q - seed)) for seed in seeds)
                nodes.append({"q": q, "cost": cost, "parent": None, "edge_error": 0.0})
                continue
            parents = []
            for parent_index, previous in enumerate(layers[-1]):
                error = edge_check(previous["q"], q, targets[index - 1], target)
                if error is not None:
                    cost = previous["cost"] + float(np.linalg.norm(q - previous["q"]))
                    parents.append((cost, parent_index, error))
            if parents:
                cost, parent, error = min(parents)
                nodes.append({"q": q, "cost": cost, "parent": parent, "edge_error": error})
        nodes.sort(key=lambda node: node["cost"])
        distinct = []
        for node in nodes:
            if not any(np.linalg.norm(node["q"] - other["q"]) < 1e-3 for other in distinct):
                distinct.append(node)
        if not distinct:
            return {"status": "NO_CONNECTED_IK_PATH", "failed_sample": index}
        layers.append(distinct[:beam_width])
        seeds = [node["q"] for node in layers[-1]]
    if not layers:
        raise ValueError("At least one target pose is required")
    def backtrace(index):
        selected = []
        for layer in reversed(layers):
            node = layer[index]
            selected.append(node)
            index = node["parent"]
        selected.reverse()
        return {
            "positions": [node["q"].tolist() for node in selected],
            "maximum_sampled_interpolation_error_m": max(node["edge_error"] for node in selected),
        }

    alternatives = [backtrace(index) for index in range(len(layers[-1]))]
    return {
        "status": "SAMPLED_DRAWING_PATH",
        **alternatives[0],
        "path_alternatives": alternatives,
        "endpoint_alternatives": [node["q"].tolist() for node in layers[-1]],
        "retained_states": sum(len(layer) for layer in layers),
    }


def tracking_edge(robot, offset, q0, q1, p0, p1, tolerance_m, tolerance_rad, maximum_step):
    """Sample joint interpolation; reject branch jumps and excessive TCP drift."""
    if np.max(np.abs(q1 - q0)) > maximum_step:
        return None
    worst = 0.0
    for alpha in (0.25, 0.5, 0.75):
        target = p0.copy()
        target[:3, 3] = (1 - alpha) * p0[:3, 3] + alpha * p1[:3, 3]
        actual = robot.forward((1 - alpha) * q0 + alpha * q1, offset)
        error = pose_error(actual, target)
        distance = float(np.linalg.norm(error[:3]))
        if distance > tolerance_m or np.linalg.norm(error[3:]) > tolerance_rad:
            return None
        worst = max(worst, distance)
    return worst


def optimization_distance(first, second):
    """Use contact depth where FCL's unsigned distance is a flat -1 sentinel."""
    distance = fcl.distance(first, second, fcl.DistanceRequest(), fcl.DistanceResult())
    if distance >= 0:
        return distance
    contact = fcl.CollisionResult()
    fcl.collide(first, second, fcl.CollisionRequest(enable_contact=True), contact)
    return -max((c.penetration_depth for c in contact.contacts), default=1e-9)


def pen_clear(planner, arm, point, rotation, scene, clearance):
    """Whether the pen bodies at this tip pose clear the obstacles and the wire."""
    if monotonic() >= planner.deadline:
        return False
    target = np.eye(4)
    target[:3, :3], target[:3, 3] = rotation, point
    flange = target @ np.linalg.inv(arm.flange_to_tcp)
    for body in arm.tool.bodies:
        transform = flange @ body.origin
        obj = placed(body.geometry, transform)
        if any(
            optimization_distance(obj, other) < clearance for other in planner.obstacles.values()
        ):
            return False
        # A body may keep its own clearance from wire (see pen_wire_failure).
        own = planner.tool_margins.get(body.name)
        wire_clearance = clearance if own is None else clearance - planner.margin + own
        if scene.possibly_near_body(obj, wire_clearance):
            try:
                nearest = scene.nearest(
                    obj,
                    deadline=planner.deadline,
                    bounding_sphere=(transform[:3, 3], body.radius),
                )
            except TimeoutError:
                return False  # A sample timeout is not an infeasibility certificate.
            if nearest and nearest["distance_m"] < wire_clearance:
                return False
    return True
