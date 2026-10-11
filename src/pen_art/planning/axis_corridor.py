"""A finite, whole-contour orientation graph with lazy robot-edge checks."""

from time import monotonic

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation, Slerp

from ..kinematics import inverse_candidates
from .axis_repair import optimization_distance
from .collision import placed


def orientation_lattice(original):
    rotations = [original]
    for z in trimesh.creation.icosphere(subdivisions=1).vertices:
        x = original[:, 0] - z * np.dot(original[:, 0], z)
        if np.linalg.norm(x) < 1e-6:
            x = original[:, 1] - z * np.dot(original[:, 1], z)
        x /= np.linalg.norm(x)
        frame = np.column_stack([x, np.cross(z, x), z])
        rotations.extend(
            frame @ Rotation.from_rotvec([0, 0, roll]).as_matrix()
            for roll in (0, np.pi / 2, np.pi, 3 * np.pi / 2)
        )
    return np.asarray(rotations)


def pen_clear(planner, arm, point, rotation, scene, clearance):
    target = np.eye(4)
    target[:3, :3], target[:3, 3] = rotation, point
    flange = target @ np.linalg.inv(arm.flange_to_tcp)
    for body in arm.tool.bodies:
        obj = placed(body.geometry, flange @ body.origin)
        if any(
            optimization_distance(obj, other) < clearance for other in planner.obstacles.values()
        ):
            return False
        if scene.possibly_near_body(obj, clearance):
            nearest = scene.nearest(obj)
            if nearest and nearest["distance_m"] < clearance:
                return False
    return True


def corridor_proposal(planner, drawing, material, maximum_repairs=64):
    """Find a global pen-axis corridor, then lazily reject failed robot edges.

    The fixed graph changes orientation, never the source tip trajectory. A
    failure only rejects this finite lattice; it does not prove infeasibility.
    Every returned path still needs the ordinary continuous motion checks.
    """
    arm = planner.arms[drawing["arm"]]
    points = np.asarray(drawing["tcp_targets_m"])
    rotations = orientation_lattice(np.asarray(drawing["tcp_rotation"]))
    clearance = planner.margin + 0.0001
    radius = planner.actions[drawing["stroke_id"]]["diameter_m"] / 2 + 0.00002
    allowed = np.zeros((len(points), len(rotations)), dtype=bool)
    scene = material.copy()
    for i, point in enumerate(points):
        if monotonic() >= planner.deadline:
            return {"status": "TIME_BUDGET"}
        scene.add(points[max(0, i - 1)], point, radius, f"corridor-guide/{i}")
        allowed[i] = [pen_clear(planner, arm, point, r, scene, clearance) for r in rotations]
        if not allowed[i].any():
            return {"status": "NO_LATTICE_ORIENTATION", "sample": i}
        if i % 25 == 0:
            print(f"Orientation corridor {drawing['stroke_id']}: {i + 1}/{len(points)}", flush=True)
    quaternions = Rotation.from_matrix(rotations).as_quat()
    angles = 2 * np.arccos(np.clip(abs(quaternions @ quaternions.T), 0, 1))
    edge_cost = angles**2 + 1e-8
    edge_cost[angles > np.pi / 2 + 1e-9] = np.inf
    rejected = set()
    for attempt in range(maximum_repairs + 1):
        cost = np.where(allowed[0], angles[0] ** 2 * 0.01, np.inf)
        parents = []
        for i in range(1, len(points)):
            transitions = cost[:, None] + edge_cost
            for layer, first, second in rejected:
                if layer == i:
                    transitions[first, second] = np.inf
            parents.append(np.argmin(transitions, axis=0))
            cost = np.min(transitions, axis=0)
            cost[~allowed[i]] = np.inf
            if not np.isfinite(cost).any():
                return {
                    "status": "NO_LATTICE_CORRIDOR",
                    "sample": i,
                    "rejected_edges": len(rejected),
                    "minimum_adjacent_angle_deg": float(
                        np.rad2deg(angles[np.ix_(allowed[i - 1], allowed[i])].min())
                    ),
                    "adjacent_orientation_counts": [
                        int(allowed[i - 1].sum()),
                        int(allowed[i].sum()),
                    ],
                }
        if not np.isfinite(cost).any():
            return {"status": "NO_LATTICE_CORRIDOR", "rejected_edges": len(rejected)}
        nodes = [int(np.argmin(cost))]
        for parent in reversed(parents):
            nodes.append(int(parent[nodes[-1]]))
        nodes.reverse()
        scene = material.copy()
        positions, targets = [], []
        previous = np.asarray(drawing["positions"][0])
        failure = None
        for i, point in enumerate(points):
            first_point = points[max(0, i - 1)]
            first_rotation = rotations[nodes[max(0, i - 1)]]
            rotation = rotations[nodes[i]]
            angle = angles[nodes[max(0, i - 1)], nodes[i]]
            count = max(
                1,
                int(np.ceil(angle / 0.05)),
                int(np.ceil(np.linalg.norm(point - first_point) / 0.0005)),
            )
            interpolator = Slerp([0, 1], Rotation.from_matrix([first_rotation, rotation]))
            last_point = first_point
            for fraction in np.linspace(1 / count, 1, count):
                if monotonic() >= planner.deadline:
                    return {"status": "TIME_BUDGET"}
                target_point = first_point + fraction * (point - first_point)
                target_rotation = interpolator(fraction).as_matrix()
                scene.add(last_point, target_point, radius, f"corridor/{i}/{fraction}")
                last_point = target_point
                if not pen_clear(planner, arm, target_point, target_rotation, scene, clearance):
                    failure = (i, nodes[max(0, i - 1)], nodes[i])
                    break
                target = np.eye(4)
                target[:3, :3], target[:3, 3] = target_rotation, target_point
                solutions = inverse_candidates(
                    arm.robot,
                    target,
                    arm.flange_to_tcp,
                    [previous, drawing["positions"][i]],
                    maximum_evaluations=100,
                    position_tolerance=0.00005,
                    orientation_tolerance=0.001,
                )
                good = [
                    np.asarray(row["positions"])
                    for row in solutions
                    if (not positions or np.max(abs(np.asarray(row["positions"]) - previous)) < 0.2)
                    and not arm.collision.check(
                        row["positions"], planner.obstacles, clearance, scene
                    )
                ]
                if not good:
                    failure = (i, nodes[max(0, i - 1)], nodes[i])
                    break
                previous = min(good, key=lambda q: np.linalg.norm(q - previous))
                positions.append(previous.tolist())
                targets.append(target_point.tolist())
            if failure:
                break
        if failure is None:
            return {
                "status": "SAMPLED_FREE_AXIS_PROPOSAL",
                "positions": positions,
                "tcp_targets_m": targets,
                "free_tool_orientation": True,
                "orientation_method": "finite_orientation_graph",
                "lattice_size": len(rotations),
                "rejected_edges": len(rejected),
                "scope": "Finite orientation graph and sampled IK; continuous validation required",
            }
        if failure[0] == 0:
            allowed[0, failure[2]] = False
        else:
            rejected.add(failure)
        print(
            f"Orientation corridor {drawing['stroke_id']}: "
            f"rejected edge {failure}, repair {attempt + 1}",
            flush=True,
        )
    return {"status": "LATTICE_REPAIR_LIMIT", "rejected_edges": len(rejected)}
