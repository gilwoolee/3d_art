"""Position-constrained, collision-aware IK for a pen with a free tool axis.

This produces a proposal. The ordinary spline, timing and continuous material
checks must still accept it before it can be used in a complete motion.
"""

import hashlib
import json
from functools import wraps
from itertools import product
from time import monotonic
from types import SimpleNamespace

import fcl
import numpy as np
from scipy.optimize import least_squares, minimize, minimize_scalar
from scipy.spatial.transform import Rotation

from ..io import read_json, sha256, write_json
from ..kinematics import geometric_jacobian, inverse_candidates
from .collision import placed
from .directed_seed import material_away_seed

AXIS_REPAIR_REVISION = 9
# Checkpoint only the matching target-solver implementation.
AXIS_WORK_REVISION = 6
WARM_SEED_PREFERRED_CHECK_REVISION = 1


def _fixed_drawing_policy(planner, drawing):
    return (
        getattr(planner, "profile", {}).get("drawing_orientation_policy") in {"fixed", "fixed_axis"}
        or drawing.get("fixed_orientation", False)
        or drawing.get("orientation_policy") in {"fixed", "fixed_axis"}
    )


def requires_outward_body(planner, drawing):
    """Side-wall zigzags use out-in poses; base contours may need other axes."""
    action = planner.actions[drawing["stroke_id"]]
    return getattr(planner, "profile", {}).get(
        "drawing_orientation_policy"
    ) == "outward_radial" and str(action.get("parent_curve") or "").startswith("zigzag-")


def outward_pen_value(planner, arm, positions, point, *, required=True):
    """Positive means the pen body stays on the object's outside hemisphere."""
    profile = getattr(planner, "profile", {})
    if not required or profile.get("drawing_orientation_policy") != "outward_radial":
        return 1.0
    radial = np.asarray(point, float)[:2] - np.asarray(profile["world_from_build"])[:2, 3]
    if np.linalg.norm(radial) < 1e-9:
        return 1.0
    body = -arm.robot.forward(positions, arm.flange_to_tcp)[:2, 2]
    return float(body @ (radial / np.linalg.norm(radial)) - 0.05)


class _PoseBudgetExceeded(TimeoutError):
    pass


def _check_pose_budget(planner):
    if monotonic() >= planner.deadline:
        raise _PoseBudgetExceeded


def _budget_status(function):
    @wraps(function)
    def checked(*args, **kwargs):
        try:
            return function(*args, **kwargs)
        except TimeoutError:
            return {"status": "TIME_BUDGET"}

    return checked


def optimization_distance(first, second):
    """Use contact depth where FCL's unsigned distance is a flat -1 sentinel."""
    distance = fcl.distance(first, second, fcl.DistanceRequest(), fcl.DistanceResult())
    if distance >= 0:
        return distance
    contact = fcl.CollisionResult()
    fcl.collide(first, second, fcl.CollisionRequest(enable_contact=True), contact)
    return -max((c.penetration_depth for c in contact.contacts), default=1e-9)


def transported_pen_axes(points, initial_body_axis):
    """Carry a transverse pen direction smoothly along the spatial curve."""
    tangents = np.gradient(np.asarray(points), axis=0)
    previous = np.asarray(initial_body_axis)
    axes = []
    for tangent in tangents:
        length = np.linalg.norm(tangent)
        if length > 1e-10:
            tangent = tangent / length
            transverse = previous - tangent * np.dot(previous, tangent)
            if np.linalg.norm(transverse) < 1e-6:
                basis = np.eye(3)[np.argmin(abs(tangent))]
                transverse = basis - tangent * np.dot(basis, tangent)
            previous = transverse / np.linalg.norm(transverse)
        axes.append(previous.copy())
    return axes


def solve_pen_pose(planner, arm, point, axis, previous, preferred, scene, clearance):
    """Optimize the rigid pen pose first, then solve the redundant arm posture."""
    if not arm.tool.bodies:
        return None
    base = arm.robot.forward(previous, arm.flange_to_tcp)[:3, :3]
    inverse = np.linalg.inv(arm.flange_to_tcp)
    objects = {row["name"]: obj for row, obj in zip(scene.segments, scene.objects, strict=True)}
    desired_z = -np.asarray(axis)
    desired_x = base[:, 0] - desired_z * np.dot(base[:, 0], desired_z)
    if np.linalg.norm(desired_x) < 1e-6:
        desired_x = base[:, 1] - desired_z * np.dot(base[:, 1], desired_z)
    desired_x /= np.linalg.norm(desired_x)
    desired = np.column_stack([desired_x, np.cross(desired_z, desired_x), desired_z])

    def rotation(x):
        return Rotation.from_rotvec(x).as_matrix() @ base

    gap_cache = {}

    def gaps(x):
        _check_pose_budget(planner)
        key = np.asarray(x).tobytes()
        if gap_cache.get("key") == key:
            return gap_cache["value"]
        target = np.eye(4)
        target[:3, :3], target[:3, 3] = rotation(x), point
        flange = target @ inverse
        distances = []
        for body in arm.tool.bodies:
            transform = flange @ body.origin
            obj = placed(body.geometry, transform)
            distances.extend(
                optimization_distance(obj, other) for other in planner.obstacles.values()
            )
            nearest = (
                scene.nearest(
                    obj,
                    deadline=planner.deadline,
                    bounding_sphere=(transform[:3, 3], body.radius),
                )
                if scene.possibly_near_body(obj, clearance + 0.005)
                else None
            )
            if nearest:
                distance = nearest["distance_m"]
                if distance < 0:
                    distance = optimization_distance(obj, objects[nearest["name"]])
                distances.append(distance)
            else:
                distances.append(clearance + 0.005)
        result = np.asarray(distances)
        gap_cache.update(key=key, value=result)
        return result

    def objective(x):
        room = np.minimum(gaps(x) - (planner.margin + 0.0005), 0) * 1000
        return float(
            0.01 * (x @ x) + 0.2 * np.sum((-rotation(x)[:, 2] - axis) ** 2) + 5 * np.sum(room**2)
        )

    for seed in (np.zeros(3), Rotation.from_matrix(desired @ base.T).as_rotvec()):
        solved = minimize(
            objective,
            seed,
            method="SLSQP",
            constraints=[{"type": "ineq", "fun": lambda x: (gaps(x) - clearance) * 1000}],
            options={"maxiter": 150, "ftol": 1e-8, "eps": 1e-5},
        )
        if gaps(solved.x).min() < clearance - 0.000002:
            continue
        target = np.eye(4)
        target[:3, :3], target[:3, 3] = rotation(solved.x), point
        solutions = inverse_candidates(
            arm.robot,
            target,
            arm.flange_to_tcp,
            [previous, preferred],
            position_tolerance=0.00005,
            orientation_tolerance=0.001,
            maximum_evaluations=100,
        )
        candidates = [
            np.asarray(row["positions"])
            for row in solutions
            if not arm.collision.check(
                row["positions"],
                planner.obstacles,
                clearance - 0.000002,
                scene,
                deadline=planner.deadline,
            )
        ]
        if candidates:
            q = min(candidates, key=lambda value: np.linalg.norm(value - previous))
            return SimpleNamespace(x=q, success=True, nit=solved.nit, message="Pen pose plus IK")
    # A fixed orientation lattice escapes disconnected local angle basins.
    # Geometrically cull first; attempt at most eight reachable pen poses.
    clear_poses = []
    for direction in product((-1, 0, 1), repeat=3):
        _check_pose_budget(planner)
        z = np.asarray(direction, dtype=float)
        if np.linalg.norm(z) == 0:
            continue
        z /= np.linalg.norm(z)
        x = base[:, 0] - z * np.dot(base[:, 0], z)
        if np.linalg.norm(x) < 1e-6:
            x = base[:, 1] - z * np.dot(base[:, 1], z)
        x /= np.linalg.norm(x)
        for roll in (0, np.pi / 2, np.pi, -np.pi / 2):
            candidate = (
                np.column_stack([x, np.cross(z, x), z])
                @ Rotation.from_rotvec([0, 0, roll]).as_matrix()
            )
            angle = Rotation.from_matrix(candidate @ base.T).as_rotvec()
            gap = float(gaps(angle).min())
            if gap >= clearance:
                clear_poses.append((float(angle @ angle), gap, candidate))
    # Spend the same eight IK attempts on both nearby poses and poses with room.
    # Quarter-turn rolls matter for a right-angle bracket even when the nozzle
    # axis is unchanged; a geometrically clear pose can otherwise be unreachable.
    ranked = sorted(clear_poses, key=lambda item: item[0])[:4]
    for item in sorted(clear_poses, key=lambda item: -item[1]):
        if not any(np.allclose(item[2], prior[2]) for prior in ranked):
            ranked.append(item)
        if len(ranked) == 8:
            break
    for _, _, candidate in ranked:
        target = np.eye(4)
        target[:3, :3], target[:3, 3] = candidate, point
        for row in inverse_candidates(
            arm.robot,
            target,
            arm.flange_to_tcp,
            [previous, preferred],
            position_tolerance=0.00005,
            orientation_tolerance=0.001,
            maximum_evaluations=100,
        ):
            if not arm.collision.check(
                row["positions"],
                planner.obstacles,
                clearance - 0.000002,
                scene,
                deadline=planner.deadline,
            ):
                return SimpleNamespace(
                    x=np.asarray(row["positions"]),
                    success=True,
                    nit=0,
                    message="Fixed orientation lattice plus IK",
                )
    return None


def solve_joint_pose(
    planner,
    arm,
    point,
    axis,
    previous,
    preferred,
    scene,
    clearance,
    *,
    allow_directed_retry=True,
    allow_pose_lattice=True,
    reference_material=None,
    check_warm_preferred=False,
    outward_required=False,
):
    """Bound a pose solve even when its numerical optimizer has not converged."""
    try:
        return _solve_joint_pose(
            planner,
            arm,
            point,
            axis,
            previous,
            preferred,
            scene,
            clearance,
            allow_directed_retry=allow_directed_retry,
            allow_pose_lattice=allow_pose_lattice,
            reference_material=reference_material,
            check_warm_preferred=check_warm_preferred,
            outward_required=outward_required,
        )
    except TimeoutError:
        return (
            SimpleNamespace(
                x=np.asarray(previous),
                success=False,
                nit=0,
                message="Pose time budget",
                deadline_exceeded=True,
            ),
            np.full(3, np.inf),
            np.array([-np.inf]),
        )


def _solve_joint_pose(
    planner,
    arm,
    point,
    axis,
    previous,
    preferred,
    scene,
    clearance,
    *,
    allow_directed_retry=True,
    allow_pose_lattice=True,
    reference_material=None,
    check_warm_preferred=False,
    outward_required=False,
):
    """Repair the exact tip position with all arm, pen and scene constraints."""
    _check_pose_budget(planner)
    robot = arm.robot

    def outward(q):
        return outward_pen_value(planner, arm, q, point, required=outward_required)

    def aligned(q):
        return not outward_required or (
            -robot.forward(q, arm.flange_to_tcp)[:3, 2] @ axis >= np.cos(np.deg2rad(25))
        )

    ranges = robot.upper - robot.lower
    scene_objects = {
        row["name"]: obj for row, obj in zip(scene.segments, scene.objects, strict=True)
    }
    bodies_by_name = {body.name: body for body in arm.collision.bodies}
    material_bounds = dict.fromkeys((body.name for body in arm.collision.bodies), clearance + 0.005)
    anchor = previous.copy()
    seed = previous.copy()
    for _ in range(20):
        delta = point - robot.forward(seed, arm.flange_to_tcp)[:3, 3]
        if np.linalg.norm(delta) < 1e-9:
            break
        jacobian = geometric_jacobian(robot, seed, arm.flange_to_tcp)[:3]
        seed = np.clip(seed + np.linalg.pinv(jacobian) @ delta, robot.lower, robot.upper)
    cache = {}

    def evaluate(q, cache=cache, point=point):
        _check_pose_budget(planner)
        key = np.asarray(q).tobytes()
        if key in cache:
            return cache[key]
        tcp = robot.forward(q, arm.flange_to_tcp)
        distances = [
            gap
            for _, _, gap, _ in arm.collision.distances(
                q, planner.obstacles, deadline=planner.deadline
            )
        ]
        nearby = {}
        frames = None
        for body, name, distance in arm.collision.material_distances(
            q, scene, material_bounds, deadline=planner.deadline
        ):
            if distance < 0:
                frames = robot.frames(q) if frames is None else frames
                geometry = bodies_by_name[body]
                obj = placed(geometry.geometry, frames[geometry.link] @ geometry.origin)
                distance = optimization_distance(obj, scene_objects[name])
            nearby[body] = distance
        # Preserve a fixed constraint vector. Broadphase exclusions prove
        # these bodies farther than the clearance plus a 5 mm buffer.
        distances.extend(nearby.get(body.name, clearance + 0.005) for body in arm.collision.bodies)
        value = (tcp[:3, 3] - point, np.asarray(distances))
        if len(cache) >= 128:
            cache.pop(next(iter(cache)))
        cache[key] = value
        return value

    def objective(q):
        body_axis = -robot.forward(q, arm.flange_to_tcp)[:3, 2]
        room = np.minimum(evaluate(q)[1] - (clearance + 0.0002), 0) * 1000
        return float(
            np.sum(((q - anchor) / ranges) ** 2)
            + 0.002 * np.sum(((q - preferred) / ranges) ** 2)
            + (0.03 if outward_required else 0.0002) * np.sum((body_axis - axis) ** 2)
            + 0.1 * np.sum(room**2)
        )

    error, distances = evaluate(seed)
    if (
        np.linalg.norm(error) < 1e-8
        and distances.min() >= clearance + 0.0001
        and outward(seed) >= 0
        and aligned(seed)
    ):
        return (
            SimpleNamespace(x=seed, success=True, nit=0, message="Continuous position step"),
            error,
            distances,
        )
    # Reuse only a posture, never prior collision acceptance. Keep the local
    # continuity projection above first; all connecting motion is still
    # independently certified by the drawing preparation stage.
    if check_warm_preferred:
        candidate = np.asarray(preferred, dtype=float)
        if (
            candidate.shape == robot.lower.shape
            and np.isfinite(candidate).all()
            and np.all(candidate >= robot.lower)
            and np.all(candidate <= robot.upper)
        ):
            delta, gaps = evaluate(candidate)
            if (
                np.isfinite(delta).all()
                and np.isfinite(gaps).all()
                and np.linalg.norm(delta) <= 0.000001
                and gaps.min() >= clearance - 0.000002
                and outward(candidate) >= 0
                and aligned(candidate)
                and not arm.collision.check(
                    candidate,
                    planner.obstacles,
                    clearance - 0.000002,
                    scene,
                    deadline=planner.deadline,
                )
            ):
                return (
                    SimpleNamespace(
                        x=candidate.copy(),
                        success=True,
                        nit=0,
                        message="Revalidated warm proposal pose",
                        warm_preferred_revalidated=True,
                    ),
                    delta,
                    gaps,
                )

    class FeasiblePose(Exception):
        def __init__(self, q):
            self.q = np.asarray(q).copy()

    iterations = 0

    def stop_when_feasible(q):
        nonlocal iterations
        iterations += 1
        delta, gaps = evaluate(q)
        if (
            np.linalg.norm(delta) < 1e-7
            and gaps.min() >= clearance + 0.00001
            and outward(q) >= 0
            and aligned(q)
        ):
            raise FeasiblePose(q)

    def constrained_minimize(*args, **kwargs):
        try:
            return minimize(*args, callback=stop_when_feasible, **kwargs)
        except FeasiblePose as feasible:
            return SimpleNamespace(
                x=feasible.q,
                success=True,
                nit=iterations,
                message="Position and clearance constraints satisfied",
            )

    solved = constrained_minimize(
        objective,
        seed,
        method="SLSQP",
        bounds=list(zip(robot.lower, robot.upper, strict=True)),
        constraints=[
            {
                "type": "eq",
                "fun": lambda q: evaluate(q)[0] * 1000,
                "jac": lambda q: geometric_jacobian(robot, q, arm.flange_to_tcp)[:3] * 1000,
            },
            {"type": "ineq", "fun": lambda q: (evaluate(q)[1] - clearance) * 1000},
            {"type": "ineq", "fun": outward},
        ],
        options={"maxiter": 150, "ftol": 1e-9},
    )
    error, distances = evaluate(solved.x)
    if allow_pose_lattice and (
        np.linalg.norm(error) > 0.00001
        or distances.min() < clearance - 0.000002
        or outward(solved.x) < 0
    ):
        pose_solution = solve_pen_pose(
            planner, arm, point, axis, previous, preferred, scene, clearance
        )
        if pose_solution is not None and outward(pose_solution.x) >= 0:
            solved = pose_solution
            error, distances = evaluate(solved.x)
    if (
        np.linalg.norm(error) > 0.00001
        or distances.min() < clearance - 0.000002
        or outward(solved.x) < 0
    ):
        alternate = constrained_minimize(
            objective,
            preferred,
            method="SLSQP",
            bounds=list(zip(robot.lower, robot.upper, strict=True)),
            constraints=[
                {
                    "type": "eq",
                    "fun": lambda q: evaluate(q)[0] * 1000,
                    "jac": lambda q: geometric_jacobian(robot, q, arm.flange_to_tcp)[:3] * 1000,
                },
                {"type": "ineq", "fun": outward},
                {
                    "type": "ineq",
                    "fun": lambda q: (evaluate(q)[1] - clearance) * 1000,
                },
            ],
            options={"maxiter": 300, "ftol": 1e-7},
        )
        alternate_error, alternate_gaps = evaluate(alternate.x)
        if (
            np.linalg.norm(alternate_error) <= 0.00001
            and alternate_gaps.min() >= clearance - 0.000002
            and outward(alternate.x) >= 0
        ):
            solved = alternate
            error, distances = alternate_error, alternate_gaps
    if (
        np.linalg.norm(error) > 0.00001
        or distances.min() < clearance - 0.000002
        or outward(solved.x) < 0
    ):
        # Trust-region least squares handles nonsmooth nearest-body changes
        # when the equality-constrained local step stalls at an active wall.
        # Acceptance still checks the actual position and clearance below.
        def residual(q, anchor=anchor, axis=axis):
            delta, gaps = evaluate(q)
            return np.r_[
                delta * 10000,
                np.minimum(gaps - clearance, 0) * 10000,
                0.01 * (q - anchor) / ranges,
                0.01 * (-robot.forward(q, arm.flange_to_tcp)[:3, 2] - axis),
                min(outward(q), 0) * 10,
            ]

        recovered = least_squares(
            residual,
            np.clip(preferred, robot.lower + 1e-9, robot.upper - 1e-9),
            bounds=(robot.lower, robot.upper),
            max_nfev=200,
            ftol=1e-10,
            xtol=1e-10,
            gtol=1e-10,
        )
        solved.x = recovered.x
        solved.success = recovered.success
        solved.message = recovered.message
        error, distances = evaluate(solved.x)
    if allow_directed_retry and (
        not np.isfinite(solved.x).all()
        or not np.isfinite(error).all()
        or np.linalg.norm(error) > 0.00001
        or distances.min() < clearance - 0.000002
        or outward(solved.x) < 0
    ):
        directed = material_away_seed(
            arm, point, previous, scene if reference_material is None else reference_material
        )
        if directed["status"] == "IK_SEED_ONLY":
            seed = np.asarray(directed["positions"])
            recovered, delta, gaps = _solve_joint_pose(
                planner,
                arm,
                point,
                np.asarray(directed["body_axis"]),
                seed,
                seed,
                scene,
                clearance,
                allow_directed_retry=False,
                allow_pose_lattice=False,
                outward_required=outward_required,
            )
            # A new seed is never acceptance: verify the exact tip and actual
            # arm/tool/material collision query with the original margin.
            if (
                np.isfinite(recovered.x).all()
                and np.linalg.norm(delta) <= 1e-6
                and gaps.min() >= clearance - 0.000002
                and outward(recovered.x) >= 0
                and not arm.collision.check(
                    recovered.x,
                    planner.obstacles,
                    clearance - 0.000002,
                    scene,
                    deadline=planner.deadline,
                )
            ):
                recovered.directed_seed = directed
                recovered.message = "Material-away seed: " + str(recovered.message)
                solved, error, distances = recovered, delta, gaps
    solved.outward_valid = outward(solved.x) >= -1e-8
    solved.outward_value = outward(solved.x)
    if not solved.outward_valid:
        solved.success = False
        solved.message = "No outward pen pose: " + str(solved.message)
    return solved, error, distances


def refine_axis_path(arm, positions, targets, joint_step=0.025):
    """Project inserted joint samples onto the unchanged Cartesian polyline."""
    output_q, output_p = [positions[0]], [targets[0]]
    for qa, qb, pa, pb in zip(
        positions[:-1], positions[1:], targets[:-1], targets[1:], strict=True
    ):
        qa, qb, pa, pb = map(np.asarray, (qa, qb, pa, pb))
        count = max(1, int(np.ceil(np.max(abs(qb - qa)) / joint_step)))
        for index in range(1, count):
            fraction = index / count
            point = pa + fraction * (pb - pa)
            q = qa + fraction * (qb - qa)
            for _ in range(20):
                error = point - arm.robot.forward(q, arm.flange_to_tcp)[:3, 3]
                if np.linalg.norm(error) < 1e-8:
                    break
                jacobian = geometric_jacobian(arm.robot, q, arm.flange_to_tcp)[:3]
                q = np.clip(q + np.linalg.pinv(jacobian) @ error, arm.robot.lower, arm.robot.upper)
            output_q.append(q.tolist())
            output_p.append(point.tolist())
        output_q.append(qb.tolist())
        output_p.append(pb.tolist())
    return output_q, output_p


@_budget_status
def clear_intermediate_poses(planner, drawing, positions, targets, material):
    """Repair interpolated poses against the same growing-material context."""
    if _fixed_drawing_policy(planner, drawing):
        return {"status": "FIXED_ORIENTATION_REQUIRED"}
    arm = planner.arms[drawing["arm"]]
    radius = planner.actions[drawing["stroke_id"]]["diameter_m"] / 2 + 0.00002
    scene = material.copy()
    result, changed = [], 0
    checkpoint = (
        planner.run / "axis_repairs" / f"refine_{drawing['arm']}_{drawing['stroke_id']}.json"
    )
    binding = {
        "revision": AXIS_REPAIR_REVISION,
        "position_projection_revision": 2,
        "cell_sha256": sha256(planner.run / "cell_manifest.json"),
        "material_sha256": planner.material_key(material),
        "path_sha256": hashlib.sha256(
            json.dumps([positions, targets], sort_keys=True).encode()
        ).hexdigest(),
    }
    saved = read_json(checkpoint) if checkpoint.exists() else {}
    if saved.get("binding") == binding:
        result = saved["positions"]
        changed = saved["changed"]
        cached = np.asarray(result)
        if (
            cached.shape != (len(result), len(arm.robot.lower))
            or len(result) > len(positions)
            or not np.isfinite(cached).all()
            or np.any(cached < arm.robot.lower)
            or np.any(cached > arm.robot.upper)
        ):
            raise ValueError("Invalid interpolation repair checkpoint")
        print(f"Restored {len(result)}/{len(positions)} refined poses", flush=True)
    restored = len(result)
    for index, (q, point) in enumerate(zip(positions, targets, strict=True)):
        if monotonic() >= planner.deadline:
            return {"status": "TIME_BUDGET", "sample": index}
        q, point = np.asarray(q), np.asarray(point)
        scene.add(targets[max(0, index - 1)], point, radius, f"refined-guide/{index}")
        if index < restored:
            continue
        if index % 250 == 0:
            print(f"Refining {drawing['stroke_id']}: pose {index + 1}/{len(positions)}", flush=True)
        position_error = np.linalg.norm(arm.robot.forward(q, arm.flange_to_tcp)[:3, 3] - point)
        if (
            position_error > 0.000001
            or outward_pen_value(
                planner, arm, q, point, required=requires_outward_body(planner, drawing)
            )
            < 0
            or arm.collision.check(
                q, planner.obstacles, planner.margin + 0.000018, scene, deadline=planner.deadline
            )
        ):
            print(f"Repairing interpolated pose {index + 1}/{len(positions)}", flush=True)
            axis = -arm.robot.forward(q, arm.flange_to_tcp)[:3, 2]
            previous = q if not result else np.asarray(result[-1])
            solved, error, distances = solve_joint_pose(
                planner,
                arm,
                point,
                axis,
                previous,
                q,
                scene,
                planner.margin + 0.00002,
                reference_material=material,
                outward_required=requires_outward_body(planner, drawing),
            )
            if getattr(solved, "deadline_exceeded", False):
                return {"status": "TIME_BUDGET", "sample": index}
            if (
                not np.isfinite(solved.x).all()
                or not getattr(solved, "outward_valid", True)
                or np.linalg.norm(error) > 0.00005
                or distances.min() < planner.margin + 0.000018
            ):
                return {
                    "status": "NO_CLEAR_INTERPOLATED_POSE",
                    "sample": index,
                    "position_error_m": float(np.linalg.norm(error)),
                    "minimum_clearance_m": float(distances.min()),
                    "positions": result,
                    "failed_position": solved.x.tolist(),
                    "target_m": point.tolist(),
                }
            q, changed = solved.x, changed + 1
        result.append(q.tolist())
        if index % 50 == 0 or index + 1 == len(positions):
            write_json(checkpoint, {"binding": binding, "positions": result, "changed": changed})
    return {
        "status": "SAMPLED_FREE_AXIS_PROPOSAL",
        "positions": result,
        "tcp_targets_m": targets,
        "repaired_intermediate_poses": changed,
    }


@_budget_status
def stabilize_axis_path(planner, drawing, proposal, material, *, max_refinements=4):
    """Bound tracking after pose repairs, without accepting any motion here."""
    if _fixed_drawing_policy(planner, drawing):
        return {"status": "FIXED_ORIENTATION_REQUIRED"}
    from .cell_planner import tracked_drawing_path

    if proposal["status"] != "SAMPLED_FREE_AXIS_PROPOSAL":
        return proposal
    arm = planner.arms[drawing["arm"]]
    result = dict(proposal)
    evidence = []
    for refinement in range(max_refinements + 1):
        candidate = {**drawing, **result, "free_tool_orientation": True}
        tracking = tracked_drawing_path(arm, candidate, True, planner.deadline)
        if tracking["status"] == "BOUNDED":
            return {**result, "tracking_refinements": evidence}
        if tracking["status"] == "TIME_BUDGET" or refinement == max_refinements:
            return {
                **result,
                "status": "REPAIRED_TRACKING_UNRESOLVED",
                "tracking": tracking,
                "tracking_refinements": evidence,
            }
        positions = np.asarray(result["positions"])
        targets = np.asarray(result["tcp_targets_m"])
        record = {"pass": refinement + 1, "input_poses": len(positions)}
        if refinement == 0:
            # An independently valid endpoint can retain an unrelated IK
            # branch after its predecessor was repaired. Try exactly one
            # continuation solve at the largest resulting jump.
            index = int(np.argmax(np.max(abs(np.diff(positions, axis=0)), axis=1))) + 1
            scene = material.copy()
            radius = planner.actions[drawing["stroke_id"]]["diameter_m"] / 2 + 0.00002
            for i in range(index + 1):
                scene.add(targets[max(0, i - 1)], targets[i], radius, f"tracking-guide/{i}")
            axis = -arm.robot.forward(positions[index], arm.flange_to_tcp)[:3, 2]
            solved, error, gaps = solve_joint_pose(
                planner,
                arm,
                targets[index],
                axis,
                positions[index - 1],
                positions[index],
                scene,
                planner.margin + 0.00002,
                reference_material=material,
                outward_required=requires_outward_body(planner, drawing),
            )
            if getattr(solved, "deadline_exceeded", False):
                return {"status": "TIME_BUDGET", "sample": index}
            old_step = float(np.max(abs(positions[index] - positions[index - 1])))
            new_step = float(np.max(abs(solved.x - positions[index - 1])))
            accepted = bool(
                np.isfinite(solved.x).all()
                and np.all(solved.x >= arm.robot.lower)
                and np.all(solved.x <= arm.robot.upper)
                and np.linalg.norm(error) <= 0.000001
                and gaps.min() >= planner.margin + 0.000018
                and new_step < old_step
                and getattr(solved, "outward_valid", True)
                and not arm.collision.check(
                    solved.x,
                    planner.obstacles,
                    planner.margin + 0.000018,
                    scene,
                    deadline=planner.deadline,
                )
            )
            record["continuation"] = {
                "index": index,
                "old_step_rad": old_step,
                "new_step_rad": new_step,
                "position_error_m": float(np.linalg.norm(error)),
                "minimum_clearance_m": float(gaps.min()),
                "used": accepted,
            }
            if accepted:
                positions[index] = solved.x
        refined, targets = refine_axis_path(
            arm, positions.tolist(), targets.tolist(), joint_step=0.025 / 2**refinement
        )
        print(
            f"Stabilizing {drawing['stroke_id']}: pass {refinement + 1}, {len(refined)} poses",
            flush=True,
        )
        result = clear_intermediate_poses(planner, drawing, refined, targets, material)
        record["output_poses"] = len(result.get("positions", []))
        evidence.append(record)
        if result["status"] != "SAMPLED_FREE_AXIS_PROPOSAL":
            return {**result, "tracking_refinements": evidence}
    raise AssertionError("Bounded refinement loop must return")


@_budget_status
def repair_collision_witness(planner, drawing, material, witness):
    """Correct a low-clearance interior pose on the unchanged target polyline.

    A continuous checker stops at the first unresolved threshold crossing. That
    point need not be the deepest violation. Reconstruct its exact polynomial
    piece, locate a low-clearance collocation by bounded scalar minimization,
    and retain the original continuous checker as the sole acceptance gate.
    The finite samples/minimizer produce a correction proposal, not a proof.
    """
    if _fixed_drawing_policy(planner, drawing):
        return {"status": "FIXED_ORIENTATION_REQUIRED"}
    from .cell_planner import tracked_drawing_path

    arm = planner.arms[drawing["arm"]]
    evidence = None
    reference_point = None
    if isinstance(witness, dict):
        tracked = tracked_drawing_path(arm, drawing, True, planner.deadline)
        if tracked["status"] != "BOUNDED":
            return {"status": "COLLOCATION_TRACKING_REJECTED", "tracking": tracked}
        index = witness.get("segment_index")
        if not isinstance(index, int) or not 0 <= index < len(tracked["pieces"]):
            return {"status": "COLLOCATION_SEGMENT_MISSING"}
        piece = tracked["pieces"][index]
        scene = material.copy()
        radius = planner.actions[drawing["stroke_id"]]["diameter_m"] / 2
        for i, prior in enumerate(tracked["pieces"][: index + 1]):
            planner.add_piece(scene, arm.identity, prior, radius, f"collocation-guide/{i}")

        def clearance(alpha):
            q = piece.position(alpha)
            gaps = [
                gap
                for _, _, gap, _ in arm.collision.distances(
                    q, planner.obstacles, deadline=planner.deadline
                )
            ]
            gaps.extend(
                distance
                for _, _, distance in arm.collision.material_distances(
                    q, scene, deadline=planner.deadline
                )
            )
            return float(min(gaps))

        samples = np.linspace(0, 1, 17)
        gaps = [clearance(alpha) for alpha in samples]
        low_index = int(np.argmin(gaps))
        low, high = samples[max(0, low_index - 1)], samples[min(16, low_index + 1)]
        optimum = minimize_scalar(
            clearance,
            bounds=(low, high),
            method="bounded",
            options={"xatol": 1e-8, "maxiter": 64},
        )
        alpha, gap = min(
            (float(samples[low_index]), gaps[low_index]),
            (float(optimum.x), float(optimum.fun)),
            key=lambda row: row[1],
        )
        q = piece.position(alpha)
        # Use the source parameter at the collocation, not the actual FK tip:
        # its permitted tracking error can otherwise project to a distant knot.
        reference_point = (1 - alpha) * tracked["targets"][index, :3, 3] + alpha * tracked[
            "targets"
        ][index + 1, :3, 3]
        evidence = {
            "method": "bounded_low_clearance_collocation",
            "segment_index": index,
            "alpha": alpha,
            "clearance_m": gap,
            "reference_point_m": reference_point.tolist(),
            "first_failure_alpha": witness.get("alpha"),
            "sample_count": len(samples),
            "minimizer_evaluations": int(optimum.nfev),
            "certified": False,
        }
    else:
        q = np.asarray(witness)
    tip = (
        arm.robot.forward(q, arm.flange_to_tcp)[:3, 3]
        if reference_point is None
        else reference_point
    )
    targets = np.asarray(drawing["tcp_targets_m"])
    positions = np.asarray(drawing["positions"])
    matches = []
    for i, (a, b) in enumerate(zip(targets[:-1], targets[1:], strict=True)):
        delta = b - a
        fraction = np.clip(np.dot(tip - a, delta) / max(np.dot(delta, delta), 1e-20), 0, 1)
        point = a + fraction * delta
        matches.append((np.linalg.norm(tip - point), i, fraction, point))
    _, index, fraction, point = min(matches, key=lambda item: item[0])
    if fraction < 1e-6 or fraction > 1 - 1e-6:
        index += int(fraction > 0.5)
        positions[index] = q
    else:
        positions = np.insert(positions, index + 1, q, axis=0)
        targets = np.insert(targets, index + 1, point, axis=0)
    result = clear_intermediate_poses(
        planner, drawing, positions.tolist(), targets.tolist(), material
    )
    if result["status"] == "SAMPLED_FREE_AXIS_PROPOSAL":
        # Correcting a knot changes the joint interpolant. Reproject one
        # bounded refinement onto the same target polyline before preparation.
        refined, refined_targets = refine_axis_path(
            arm, result["positions"], result["tcp_targets_m"]
        )
        result = clear_intermediate_poses(planner, drawing, refined, refined_targets, material)
    result = stabilize_axis_path(planner, drawing, result, material)
    if evidence is not None:
        result["collocation"] = evidence
    return result


@_budget_status
def repair_drawing_axes(planner, drawing, material, *, progress=None, refine=True):
    """Keep every tip target, letting redundant joints turn the pen away from material.

    Each constrained solve starts from the previous configuration. A preference
    for the original pose limits unnecessary posture changes; it is not an
    orientation constraint. Material along the proposed centerline is only a
    search guide. Continuous replay rebuilds the actual FK deposition geometry.
    """
    if _fixed_drawing_policy(planner, drawing):
        return {
            "status": "FIXED_ORIENTATION_REQUIRED",
            "reason": "Free-axis repair is disabled for fixed drawing policy",
        }
    arm = planner.arms[drawing["arm"]]
    robot = arm.robot
    targets = np.asarray(drawing["tcp_targets_m"], dtype=float)
    original = np.asarray(drawing["positions"], dtype=float)
    if drawing.get("orientation_policy") == "outward_radial":
        from .cell_motion import radial_pen_rotations

        transform = (
            planner.turntable.build_transform(drawing.get("table_index", 0))
            if planner.turntable is not None
            else np.asarray(planner.profile["world_from_build"])
        )
        preferred_axes = -radial_pen_rotations(
            arm,
            targets,
            transform,
            drawing.get("pen_tilt_deg", 20.0),
            drawing.get("roll_rad", 0.0),
        )[:, :, 2]
    else:
        preferred_axes = transported_pen_axes(targets, -np.asarray(drawing["tcp_rotation"])[:, 2])
    scene = material.copy()
    radius = planner.actions[drawing["stroke_id"]]["diameter_m"] / 2
    clearance = planner.margin + 0.00002
    checkpoint = planner.run / "axis_repairs" / f"work_{drawing['arm']}_{drawing['stroke_id']}.json"
    binding = {
        "revision": AXIS_WORK_REVISION,
        "cell_sha256": sha256(planner.run / "cell_manifest.json"),
        "construction_sha256": sha256(planner.run / "construction_plan.json"),
        "drawing_sha256": hashlib.sha256(json.dumps(drawing, sort_keys=True).encode()).hexdigest(),
        "material_sha256": planner.material_key(material),
    }
    saved = read_json(checkpoint) if checkpoint.exists() else {}
    positions = saved.get("positions", []) if saved.get("binding") == binding else []
    diagnostics = saved.get("diagnostics", []) if positions else []
    if positions:
        cached = np.asarray(positions)
        if (
            cached.shape != (len(positions), len(robot.lower))
            or not np.isfinite(cached).all()
            or len(positions) > len(targets)
            or len(diagnostics) != len(positions)
            or np.any(cached < robot.lower)
            or np.any(cached > robot.upper)
        ):
            raise ValueError("Invalid pen-axis work checkpoint")
        print(
            f"Restored {len(positions)}/{len(targets)} pen poses for {drawing['stroke_id']}",
            flush=True,
        )
    restored = len(positions)
    previous = np.asarray(positions[-1]).copy() if positions else original[0].copy()
    for index, (point, preferred, axis) in enumerate(
        zip(targets, original, preferred_axes, strict=True)
    ):
        if monotonic() >= planner.deadline:
            return {"status": "TIME_BUDGET", "sample": index, "positions": positions}
        if index:
            scene.add(
                targets[index - 1],
                point,
                radius + 0.00002,
                f"axis-guide/{drawing['arm']}/{drawing['stroke_id']}/{index}",
            )
        else:
            scene.add(point, point, radius + 0.00002, "axis-guide/start")
        if index < restored:
            continue
        solved, error, distances = solve_joint_pose(
            planner,
            arm,
            point,
            axis,
            previous,
            preferred,
            scene,
            clearance,
            reference_material=material,
            check_warm_preferred=(
                drawing.get("warm_seed_preferred_check_revision")
                == WARM_SEED_PREFERRED_CHECK_REVISION
            ),
            outward_required=requires_outward_body(planner, drawing),
        )
        if getattr(solved, "deadline_exceeded", False):
            return {"status": "TIME_BUDGET", "sample": index, "positions": positions}
        record = {
            "sample": index,
            "solver_success": bool(solved.success),
            "iterations": int(solved.nit),
            "position_error_m": float(np.linalg.norm(error)),
            "minimum_clearance_m": float(distances.min()),
            "outward_value": outward_pen_value(
                planner, arm, solved.x, point, required=requires_outward_body(planner, drawing)
            ),
        }
        if hasattr(solved, "directed_seed"):
            record["directed_seed"] = solved.directed_seed
        if getattr(solved, "warm_preferred_revalidated", False):
            record["warm_preferred_revalidated"] = True
        diagnostics.append(record)
        if (
            not np.isfinite(solved.x).all()
            or not getattr(solved, "outward_valid", True)
            or np.linalg.norm(error) > 0.00005
            or (distances.min() < clearance - 0.000002)
        ):
            return {
                "status": "NO_CLEAR_AXIS_CONFIGURATION",
                "sample": index,
                "positions": positions,
                "diagnostics": diagnostics,
                "solver_message": str(solved.message),
            }
        previous = solved.x
        positions.append(previous.tolist())
        if index % 25 == 0 or index + 1 == len(targets):
            write_json(
                checkpoint, {"binding": binding, "positions": positions, "diagnostics": diagnostics}
            )
        if progress is not None and (index % 25 == 0 or index + 1 == len(targets)):
            progress(index + 1, len(targets), record)
    if not refine:
        return {
            "status": "SAMPLED_FREE_AXIS_PROPOSAL",
            "positions": positions,
            "tcp_targets_m": targets.tolist(),
            "diagnostics": diagnostics,
            "free_tool_orientation": True,
            "scope": "Position and sampled clearance only; full continuous validation required",
        }
    positions, refined_targets = refine_axis_path(arm, positions, targets.tolist())
    intermediate = clear_intermediate_poses(planner, drawing, positions, refined_targets, material)
    intermediate = stabilize_axis_path(planner, drawing, intermediate, material)
    if intermediate["status"] != "SAMPLED_FREE_AXIS_PROPOSAL":
        return {**intermediate, "diagnostics": diagnostics}
    return {
        "status": "SAMPLED_FREE_AXIS_PROPOSAL",
        "positions": intermediate["positions"],
        "diagnostics": diagnostics,
        "tcp_targets_m": intermediate["tcp_targets_m"],
        "tracking_refinements": intermediate.get("tracking_refinements", []),
        "repaired_intermediate_poses": intermediate["repaired_intermediate_poses"],
        "free_tool_orientation": True,
        "scope": "Position and sampled clearance only; full continuous validation required",
    }
