"""Bounded horizontal-pen geometry/reach diagnostic, not a motion certificate."""

from pathlib import Path
from time import monotonic

import fcl
import numpy as np
import trimesh

from pen_art.cell import load_cell
from pen_art.curves import flatten_curve, transform_points
from pen_art.io import read_json, sha256, write_json
from pen_art.kinematics import inverse_candidates
from pen_art.planning.collision import placed
from pen_art.planning.scene import MaterialScene

profile_path = Path("research/planning_cell/cell_turntable.yaml")
profile, arms, obstacles = load_cell(profile_path)
arm = arms["arm_1"]
build = np.asarray(profile["world_from_build"])
center = build[:3, 3]
bodies = [b for b in arm.collision.bodies if b.name.startswith("tool/")]
output = Path("runs/horizontal_pen_evaluation_v1")
output.mkdir(exist_ok=True)


def pose(point, tilt, roll=0):
    radial = point[:2] - center[:2]
    radial /= np.linalg.norm(radial)
    angle = np.deg2rad(tilt)
    z = np.r_[-np.sin(angle) * radial, -np.cos(angle)]
    x = np.array([0.0, 0.0, 1.0])
    x -= x @ z * z
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    transform = np.eye(4)
    transform[:3, :3] = np.column_stack(
        [
            np.cos(roll) * x + np.sin(roll) * y,
            -np.sin(roll) * x + np.cos(roll) * y,
            z,
        ]
    )
    transform[:3, 3] = point
    return transform


# Illustrative shape only: retain current body/adapter; replace the entire
# 27mm shank by a taper, small end r=.8mm at 3mm behind tip, large end r=12mm
# at30mm behind tip. No dimensions inferred as measured from product photos.
angles = np.arange(32) * 2 * np.pi / 32
vertices = np.concatenate(
    [
        np.column_stack(
            [
                r / np.cos(np.pi / 32) * np.cos(angles),
                r / np.cos(np.pi / 32) * np.sin(angles),
                np.full(32, z),
            ]
        )
        for r, z in [(0.0008, -0.003), (0.012, -0.030)]
    ]
)
hull = trimesh.convex.convex_hull(vertices)
faces = np.column_stack([np.full(len(hull.faces), 3), hull.faces]).ravel()
taper = fcl.Convex(np.asarray(hull.vertices), len(hull.faces), faces)
results = {}
for obj in ["cube", "bottle"]:
    report = read_json(Path(f"runs/{obj}_straight_outward_cache_probe_v1/result.json"))
    record = read_json(
        Path("runs/motion_library_phase1_centered_pen_v1/motions")
        / (report["runtime_parent_ids"][0] + ".json")
    )["record"]
    raw = np.asarray(record["trajectory"]["source_points_m"])
    points = np.concatenate(
        [np.linspace(a, b, 3, endpoint=False) for a, b in zip(raw[:-1], raw[1:], strict=True)]
        + [raw[-1:]]
    )
    base = read_json(Path(f"runs/{obj}_straight_outward_phase1_v1/construction_plan.json"))[
        "actions"
    ][0]
    bp, _ = flatten_curve(base, 0.00005, 0.003)
    bp = transform_points(bp, build)
    scene = MaterialScene()
    for j, (a, b) in enumerate(zip(bp[:-1], bp[1:], strict=True)):
        scene.add(a, b, 0.0005, f"base/{j}")
    base_count = len(scene.objects)
    for j, (a, b) in enumerate(zip(points[:-1], points[1:], strict=True)):
        scene.add(a, b, 0.0005, f"prefix/{j}")
    rows = []
    for tilt in [70, 80, 90]:
        for geometry in ["current_cylindrical", "illustrative_taper"]:
            minima = {"wire": 1.0, "static": 1.0}
            witnesses = {}
            collided = {"wire": set(), "static": set()}
            for i, p in enumerate(points):
                tcp = pose(p, tilt)
                shapes = [
                    (b.name, placed(b.geometry, tcp @ np.linalg.inv(arm.flange_to_tcp) @ b.origin))
                    for b in bodies
                    if geometry == "current_cylindrical" or b.name != "tool/nozzle_shank"
                ]
                if geometry == "illustrative_taper":
                    shapes.append(("tool/illustrative_taper", placed(taper, tcp)))
                for name, shape in shapes:
                    for group, obs in [
                        ("static", list(obstacles.items())),
                        (
                            "wire",
                            [
                                (scene.segments[j]["name"], wire)
                                for j, wire in enumerate(scene.objects[: base_count + i])
                            ],
                        ),
                    ]:
                        for other, obstacle in obs:
                            gap = float(fcl.distance(shape, obstacle))
                            if gap < 0:
                                collided[group].add(i)
                            if gap < minima[group]:
                                minima[group] = gap
                                witnesses[group] = {
                                    "sample": i,
                                    "body": name,
                                    "obstacle": other,
                                    "tcp_m": p.tolist(),
                                }
            row = {
                "tilt_deg": tilt,
                "geometry": geometry,
                "minimum_gap_mm": {k: v * 1000 if v >= 0 else None for k, v in minima.items()},
                "colliding_sample_counts": {k: len(v) for k, v in collided.items()},
                "witnesses": witnesses,
                "passes_sampled_1mm": {k: v >= 0.001 for k, v in minima.items()},
            }
            rows.append(row)
            print(obj, row, flush=True)
    # Sparse pointwise IK with bounded multiple rolls/seeds, no path connectivity claim.
    deadline = monotonic() + 35
    reach = []
    previous = arm.initial_positions
    for index in np.unique(np.linspace(0, len(raw) - 1, 17).astype(int)):
        if monotonic() > deadline:
            break
        solutions = []
        for roll in [0.0, np.pi]:
            solutions += inverse_candidates(
                arm.robot,
                pose(raw[index], 90, roll),
                arm.flange_to_tcp,
                [previous, arm.initial_positions, (arm.robot.lower + arm.robot.upper) / 2],
                maximum_evaluations=70,
            )
        if solutions:
            previous = np.asarray(solutions[0]["positions"])
        reach.append(
            {
                "source_index": int(index),
                "point_m": raw[index].tolist(),
                "solutions": len(solutions),
            }
        )
    results[obj] = {
        "source_samples": len(points),
        "geometry_checks": rows,
        "horizontal_sparse_ik": reach,
    }
    write_json(
        output / "result.json",
        {
            "scope": __doc__,
            "cases": results,
            "profile_sha256": sha256(profile_path),
            "script_sha256": sha256(Path(__file__)),
            "taper_assumptions": {
                "tip_end_radius_mm": 0.8,
                "proximal_radius_mm": 12,
                "tip_gap_mm": 3,
                "length_mm": 27,
                "measured": False,
            },
            "limitations": [
                "Sampled tool geometry and sparse pointwise IK only",
                "No connected robot motion, timing, or full build acceptance",
                "All full-object searches remain paused",
            ],
        },
    )
