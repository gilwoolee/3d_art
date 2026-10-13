"""Bounded horizontal-pen geometry/reach diagnostic, not a motion certificate."""

from pathlib import Path

import numpy as np

from pen_art.cell import load_cell
from pen_art.io import read_json, write_json
from pen_art.kinematics import inverse_candidates

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


follow = read_json(output / "fixture_followup.json")
results = {}
for obj, data in follow["cases"].items():
    old = read_json(Path(f"runs/{obj}_straight_outward_cache_probe_v1/result.json"))
    rec = read_json(
        Path("runs/motion_library_phase1_centered_pen_v1/motions")
        / (old["runtime_parent_ids"][0] + ".json")
    )["record"]
    raw = np.asarray(rec["trajectory"]["source_points_m"])
    rows = []
    for trial in data["sparse_ik"]:
        recovered = []
        for row in trial["poses"]:
            if row["solutions"]:
                continue
            index = row["index"]
            found = []
            for degrees in [90, -90, 45, -45, 135, -135]:
                target = pose(raw[index], 90, np.deg2rad(degrees))
                target[:3, 3] += [trial["cell_x_shift_m"], 0, 0.02]
                sols = inverse_candidates(
                    arm.robot,
                    target,
                    arm.flange_to_tcp,
                    [arm.initial_positions, (arm.robot.lower + arm.robot.upper) / 2],
                    maximum_evaluations=90,
                )
                if sols:
                    found = [degrees, sols[0]["positions"]]
                    break
            recovered.append({"index": index, "recovered": bool(found), "solution": found})
        rows.append(
            {
                "cell_x_shift_m": trial["cell_x_shift_m"],
                "solved": trial["solved"] + sum(r["recovered"] for r in recovered),
                "tested": trial["tested"],
                "extra_roll_attempts": recovered,
            }
        )
    results[obj] = rows
    print(obj, [(r["cell_x_shift_m"], r["solved"], r["tested"]) for r in rows], flush=True)
write_json(
    output / "roll_followup.json",
    {
        "scope": ("Sparse pointwise IK with additional pen rolls; "
                  "no collision or connectivity certificate"),
        "cases": results,
    },
)
