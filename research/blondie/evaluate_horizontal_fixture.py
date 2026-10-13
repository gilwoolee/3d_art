"""Bounded horizontal-pen geometry/reach diagnostic, not a motion certificate."""

import argparse
from pathlib import Path

import fcl
import numpy as np
import trimesh

from pen_art.cell import load_cell
from pen_art.io import read_json, write_json
from pen_art.kinematics import inverse_candidates
from pen_art.planning.collision import placed

profile_path = Path("research/planning_cell/cell_turntable.yaml")
profile, arms, obstacles = load_cell(profile_path)
arm = arms["arm_1"]
build = np.asarray(profile["world_from_build"])
center = build[:3, 3]
bodies = [b for b in arm.collision.bodies if b.name.startswith("tool/")]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--tilts", type=float, nargs="+", default=[80, 90])
parser.add_argument("--ik-tilts", type=float, nargs="+", default=[90])
parser.add_argument("--output", type=Path, default=Path("runs/horizontal_pen_evaluation_v1"))
args = parser.parse_args()
output = args.output
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
    old = read_json(Path(f"runs/{obj}_straight_outward_cache_probe_v1/result.json"))
    record = read_json(
        Path("runs/motion_library_phase1_centered_pen_v1/motions")
        / (old["runtime_parent_ids"][0] + ".json")
    )["record"]
    raw = np.asarray(record["trajectory"]["source_points_m"])
    points = np.concatenate(
        [np.linspace(a, b, 3, endpoint=False) for a, b in zip(raw[:-1], raw[1:], strict=True)]
        + [raw[-1:]]
    )
    stand = np.eye(4)
    stand[:3, 3] = center + [0, 0, 0.010]
    riser = placed(
        fcl.Box(0.080, 0.080, 0.020) if obj == "cube" else fcl.Cylinder(0.025, 0.020), stand
    )
    rows = []
    for tilt in args.tilts:
        for model in ["current_cylindrical", "illustrative_taper"]:
            minimum = 1.0
            witness = None
            for i, p in enumerate(points):
                tcp = pose(p + [0, 0, 0.020], tilt)
                shapes = [
                    (b.name, placed(b.geometry, tcp @ np.linalg.inv(arm.flange_to_tcp) @ b.origin))
                    for b in bodies
                    if model == "current_cylindrical" or b.name != "tool/nozzle_shank"
                ]
                if model == "illustrative_taper":
                    shapes.append(("tool/illustrative_taper", placed(taper, tcp)))
                for name, shape in shapes:
                    for obstacle, objshape in [*obstacles.items(), ("20mm_narrow_riser", riser)]:
                        gap = float(fcl.distance(shape, objshape))
                        if gap < minimum:
                            minimum = gap
                            witness = {"sample": i, "body": name, "obstacle": obstacle}
            rows.append(
                {
                    "tilt": tilt,
                    "model": model,
                    "static_gap_mm": minimum * 1000 if minimum >= 0 else None,
                    "passes_1mm": minimum >= 0.001,
                    "witness": witness,
                }
            )
    reach = []
    for ik_tilt in args.ik_tilts:
        for dx in [0, -0.10, -0.15]:
            poses = []
            previous = arm.initial_positions
            for index in np.unique(np.linspace(0, len(raw) - 1, 17).astype(int)):
                sols = []
                for roll in [0.0, np.pi]:
                    target = pose(raw[index], ik_tilt, roll)
                    target[:3, 3] += [dx, 0, 0.02]
                    sols += inverse_candidates(
                        arm.robot,
                        target,
                        arm.flange_to_tcp,
                        [previous, arm.initial_positions, (arm.robot.lower + arm.robot.upper) / 2],
                        maximum_evaluations=70,
                    )
                if sols:
                    previous = np.asarray(sols[0]["positions"])
                poses.append({"index": int(index), "solutions": len(sols)})
            reach.append(
                {
                    "tilt_deg": ik_tilt,
                    "cell_x_shift_m": dx,
                    "object_lift_m": 0.02,
                    "solved": sum(x["solutions"] > 0 for x in poses),
                    "tested": len(poses),
                    "poses": poses,
                }
            )
    results[obj] = {"narrow_riser": rows, "sparse_ik": reach}
    print(obj, results[obj], flush=True)
write_json(
    output / "fixture_followup.json",
    {
        "scope": (
            "Sampled tool fixture clearance with hypothetical 20mm object-footprint riser; "
            "separate pointwise IK placement experiments, not full-arm/path certificate"
        ),
        "cases": results,
    },
)
