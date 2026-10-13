"""Screen hypothetical nozzle sizes without changing the active tool profile.

This reports sampled static tool access, not arm feasibility or a complete
print. A failed finite sample is not an infeasibility certificate. Dimensions
are counterfactual inputs to help resolve the existing hardware question.
"""

import argparse
from dataclasses import replace
from pathlib import Path
from time import monotonic

import fcl
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from pen_art.curves import flatten_curve, transform_points
from pen_art.io import read_json, sha256, write_json
from pen_art.planning.axis_corridor import pen_clear
from pen_art.planning.axis_repair import optimization_distance
from pen_art.planning.collision import placed
from pen_art.planning.section_build import SectionBuild


def inner_balls(arm):
    centers, radii = [], []
    inverse = np.linalg.inv(arm.flange_to_tcp)
    for body in arm.tool.bodies:
        geometry = body.geometry
        if isinstance(geometry, fcl.Cylinder):
            radius = geometry.radius - 1e-8
            axis, length = 2, geometry.lz
        elif isinstance(geometry, fcl.Box):
            radius = min(geometry.side) / 2 - 1e-8
            axis = int(np.argmax(geometry.side))
            length = geometry.side[axis]
        else:
            raise ValueError("This diagnostic only handles the declared primitive tool")
        transform = inverse @ body.origin
        for offset in np.linspace(
            -length / 2 + radius, length / 2 - radius, min(32, max(1, int(length / radius)))
        ):
            local = np.zeros(3)
            local[axis] = offset
            centers.append(transform[:3, :3] @ local + transform[:3, 3])
            radii.append(radius)
    return np.asarray(centers), np.asarray(radii)


def verify_tool_curve(build, arm, row, rotation, material):
    """Bound constant-orientation tool clearance over every exact cubic."""
    pending = [(np.asarray(segment) + build.transform[:3, 3], 0) for segment in row["segments"]]
    inverse = np.linalg.inv(arm.flange_to_tcp)
    deadline = min(build.planner.deadline, monotonic() + 30)
    checked, certified, minimum_bound = 0, 0, float("inf")
    while pending:
        if monotonic() >= deadline:
            return {"status": "TIME_BUDGET", "checked_intervals": checked}
        control, depth = pending.pop()
        first = (control[:-1] + control[1:]) / 2
        second = (first[:-1] + first[1:]) / 2
        middle = (second[0] + second[1]) / 2
        target = np.eye(4)
        target[:3, :3], target[:3, 3] = rotation, middle
        flange = target @ inverse
        gap = float("inf")
        blocker = None
        for body in arm.tool.bodies:
            transform = flange @ body.origin
            obj = placed(body.geometry, transform)
            for name, obstacle in build.planner.obstacles.items():
                distance = optimization_distance(obj, obstacle)
                if distance < gap:
                    gap, blocker = distance, (body.name, name)
            nearest = material.nearest(
                obj, deadline=deadline, bounding_sphere=(transform[:3, 3], body.radius)
            )
            if nearest:
                if nearest["distance_m"] < gap:
                    gap, blocker = nearest["distance_m"], (body.name, nearest["name"])
        checked += 1
        # A Bezier curve lies in the convex hull of its controls. Translation
        # by any point in this hull moves every tool point by at most this
        # radius. Distance to static material is 1-Lipschitz under translation.
        bound = gap - np.linalg.norm(control - middle, axis=1).max()
        if bound > build.planner.margin + 1e-6:
            certified += 1
            minimum_bound = min(minimum_bound, float(bound))
            continue
        if gap <= build.planner.margin or depth == 20:
            return {
                "status": "CLEARANCE_REJECTED" if gap <= build.planner.margin else "UNRESOLVED",
                "checked_intervals": checked,
                "point_clearance_m": gap,
                "depth": depth,
                "blocker": blocker,
                "point_world_m": middle.tolist(),
            }
        pending.append((np.array([middle, second[1], first[2], control[3]]), depth + 1))
        pending.append((np.array([control[0], first[0], second[0], middle]), depth + 1))
    return {
        "status": "CERTIFIED_TOOL_ONLY_CURVE",
        "checked_intervals": checked,
        "certified_intervals": certified,
        "minimum_clearance_lower_bound_m": minimum_bound,
        "required_clearance_m": build.planner.margin,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--extended-tip", action="store_true")
    parser.add_argument("--verify-saved", action="store_true")
    args = parser.parse_args()
    profile = Path("research/planning_cell/cell_turntable.yaml")
    build = SectionBuild(Path("runs/palace_indexed_corridor_dense_v1"), profile, 600)
    row = build.actions["section-01212"]
    material = build.scene({k for k, r in build.actions.items() if r["stage"] == "framework"})
    points, _ = flatten_curve(row, 0.00005, 0.003)
    points = transform_points(points, build.transform)
    output = Path(
        "research/blondie/palace_extended_tip_probe.json"
        if args.extended_tip
        else "research/blondie/palace_nozzle_radius_probe.json"
    )
    saved = read_json(output) if args.verify_saved else None
    saved_digest = sha256(output) if saved else None
    if saved:
        assert saved["profile_sha256"] == sha256(profile)
        assert saved["graph_sha256"] == sha256(build.run / "stroke_graph.json")
    starts = np.asarray([r["start"] for r in material.segments])
    ends = np.asarray([r["end"] for r in material.segments])
    tree = cKDTree(np.concatenate([starts, ends, (starts + ends) / 2]))
    strand_radius = min(r["radius_m"] for r in material.segments)
    table = build.planner.profile["turntable"]
    cylinders = [
        (build.planner.obstacles[name].getTranslation(), table[r], table[h] / 2)
        for name, r, h in (
            ("platform", "plate_radius_m", "plate_thickness_m"),
            ("turntable_housing", "housing_radius_m", "housing_height_m"),
        )
    ]
    rotations = Rotation.random(50000, random_state=7).as_matrix()
    results = []
    configurations = (
        [(0.0025, extension) for extension in (0.02, 0.04, 0.06, 0.1)]
        if args.extended_tip
        else [(radius, 0.0) for radius in (0.0025, 0.002, 0.0015, 0.001, 0.0005)]
    )
    for radius, extension in configurations:
        began = monotonic()
        bodies = [
            replace(
                body,
                geometry=fcl.Cylinder(radius, body.geometry.lz),
                radius=float(np.hypot(radius, body.geometry.lz / 2)),
            )
            if body.name.endswith("nozzle_shank")
            else body
            for body in build.arm.tool.bodies
        ]
        tcp = build.arm.flange_to_tcp.copy()
        if extension:
            # Move the tip along the declared nozzle axis and explicitly keep
            # the added slender tube in the collision model. The original
            # shank, barrel and mount keep their flange-frame geometry.
            nozzle = next(body for body in bodies if body.name.endswith("nozzle_shank"))
            axis = tcp[:3, 2]
            origin = nozzle.origin.copy()
            origin[:3, 3] = tcp[:3, 3] + axis * (extension / 2 - 0.003)
            bodies.append(
                replace(
                    nozzle,
                    name="probe/slender_extension",
                    origin=origin,
                    geometry=fcl.Cylinder(0.0005, extension),
                    radius=float(np.hypot(0.0005, extension / 2)),
                )
            )
            tcp[:3, 3] += axis * extension
        arm = replace(build.arm, flange_to_tcp=tcp, tool=replace(build.arm.tool, bodies=bodies))
        if saved:
            result = next(
                r
                for r in saved["results"]
                if r["nozzle_shank_radius_m"] == radius and r["tip_extension_m"] == extension
            )
            scene = material.copy()
            for i, (a, b) in enumerate(zip(points[:-1], points[1:], strict=True)):
                scene.add(a, b, row["diameter_m"] / 2 + 0.00035, f"curve-envelope/{i}")
            result["continuous_tool_checks"] = [
                verify_tool_curve(build, arm, row, np.asarray(rotation), scene)
                for rotation in result["constant_tool_rotations_passing_sampled_points"]
            ]
            result["continuous_framework_checks"] = [
                verify_tool_curve(build, arm, row, np.asarray(rotation), material)
                for rotation in result["constant_tool_rotations_passing_sampled_points"]
            ]
            print(
                extension,
                {
                    key: [r["status"] for r in result[key]]
                    for key in ("continuous_tool_checks", "continuous_framework_checks")
                },
                flush=True,
            )
            results.append(result)
            continue
        local, radii = inner_balls(arm)
        scores = np.empty(len(rotations))
        for begin in range(0, len(rotations), 512):
            world = np.einsum("bij,nj->bni", rotations[begin : begin + 512], local) + points[0]
            gaps = (
                tree.query(world.reshape(-1, 3), workers=2)[0].reshape(world.shape[:2])
                - radii
                - strand_radius
            )
            for center, extent, half_height in cylinders:
                delta = world - center
                radial = np.maximum(np.linalg.norm(delta[:, :, :2], axis=2) - extent, 0)
                vertical = np.maximum(abs(delta[:, :, 2]) - half_height, 0)
                gaps = np.minimum(gaps, np.hypot(radial, vertical) - radii)
            scores[begin : begin + len(world)] = gaps.min(axis=1)
        survivors = np.flatnonzero(scores >= build.planner.margin + 0.000002)
        accepted, checked = [], 0
        for index in survivors[np.argsort(scores[survivors])[::-1]][:512]:
            checked += 1
            if all(
                pen_clear(build.planner, arm, point, rotations[index], material, 0.001002)
                for point in points
            ):
                accepted.append(rotations[index].tolist())
                if len(accepted) == 4:
                    break
        result = {
            "nozzle_shank_radius_m": radius,
            "tip_extension_m": extension,
            "extension_radius_m": 0.0005 if extension else None,
            "unchanged_barrel_radius_m": 0.012,
            "necessary_screen_survivors": len(survivors),
            "exact_tool_checks": checked,
            "sampled_curve_points": len(points),
            "constant_tool_rotations_passing_sampled_points": accepted,
            "seconds": monotonic() - began,
        }
        results.append(result)
        print({k: v for k, v in result.items() if not isinstance(v, list)}, flush=True)
    write_json(
        output,
        {
            "scope": "Counterfactual static tool poses only; not an approved tool, arm path, "
            "or complete-build acceptance. Optional continuous checks cover only the rigid "
            "tool and this cubic. Framework-only checks omit current-stroke deposition; "
            "full-curve-envelope checks include future stroke material conservatively.",
            "active_profile_changed": False,
            "profile_sha256": sha256(profile),
            "graph_sha256": sha256(build.run / "stroke_graph.json"),
            "section": row["id"],
            "parent_curve": row["parent_curve"],
            "orientations_per_radius": len(rotations),
            "sampled_input_sha256": saved_digest,
            "results": results,
        },
    )


if __name__ == "__main__":
    main()
