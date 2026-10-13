from pathlib import Path

import fcl
import numpy as np

from pen_art.cell import load_cell
from pen_art.io import read_json, write_json
from pen_art.planning.collision import placed
from pen_art.planning.scene import MaterialScene

profile, arms, _ = load_cell(Path("research/planning_cell/cell_turntable.yaml"))
arm = arms["arm_1"]
center = np.asarray(profile["world_from_build"])[:3, 3]
body = next(b for b in arm.collision.bodies if b.name == "tool/nozzle_shank")
results = {}
for obj in ["cube", "bottle"]:
    report = read_json(Path(f"runs/{obj}_straight_outward_cache_probe_v1/result.json"))
    record = read_json(
        Path("runs/motion_library_phase1_centered_pen_v1/motions")
        / (report["runtime_parent_ids"][0] + ".json")
    )["record"]
    points = np.asarray(record["trajectory"]["source_points_m"])
    # First complete ascent/descent; split knots densely along the saved source.
    points = np.concatenate(
        [
            np.linspace(a, b, 3, endpoint=False)
            for a, b in zip(points[:16], points[1:17], strict=True)
        ]
        + [points[16:17]]
    )
    scene = MaterialScene()
    for i in range(len(points) - 1):
        scene.add(points[i], points[i + 1], 0.0005, f"source/{i}")

    def assess(tilt, yaw, gap=3, policy="radial", points=points, scene=scene):
        worst = 1.0
        witness = None
        for i, p in enumerate(points[1:], 1):
            radial = p[:2] - center[:2]
            radial /= np.linalg.norm(radial)
            if policy == "travel":
                radial = (p - points[i - 1])[:2]
                radial /= np.linalg.norm(radial)
            a = np.deg2rad(yaw)
            radial = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]) @ radial
            a = np.deg2rad(tilt)
            z = np.r_[-np.sin(a) * radial, -np.cos(a)]
            x = np.array([1.0, 0, 0])
            x -= x @ z * z
            x /= np.linalg.norm(x)
            tcp = np.eye(4)
            tcp[:3, :3] = np.column_stack([x, np.cross(z, x), z])
            tcp[:3, 3] = p
            origin = body.origin.copy()
            length = body.geometry.lz - (gap - 3) / 1000
            origin[2, 3] -= (gap - 3) / 2000
            shaft = placed(
                fcl.Cylinder(body.geometry.radius, length),
                tcp @ np.linalg.inv(arm.flange_to_tcp) @ origin,
            )
            for j, wire in enumerate(scene.objects[:i]):
                d = float(fcl.distance(shaft, wire))
                if d < worst:
                    worst = d
                    witness = {"sample": i, "source_segment": j, "tip_mm": (p * 1000).tolist()}
        return {
            "tilt": tilt,
            "yaw": yaw,
            "gap": gap,
            "policy": policy,
            "gap_mm": None if worst < 0 else worst * 1000,
            "overlap": worst < 0,
            "passes_1mm": worst >= 0.001,
            "witness": witness,
        }

    rows = [
        assess(t, y)
        for t in [10, 20, 30, 40, 45, 50, 60, 70]
        for y in [-90, -60, -30, 0, 30, 60, 90]
    ]
    rows += [assess(t, 0, g) for t in [20, 30] for g in [4, 5, 6, 8]]
    rows += [assess(t, 0, policy="travel") for t in [20, 30, 40, 50, 60, 70]]
    results[obj] = {"rows": rows, "source_points": points.tolist()}
    print(
        obj,
        "radial",
        [
            (r["tilt"], r["gap_mm"])
            for r in rows
            if r["yaw"] == 0 and r["gap"] == 3 and r["policy"] == "radial"
        ],
    )
    print(
        obj,
        "passing",
        [
            (r["tilt"], r["yaw"], r["gap"], r["policy"], r["gap_mm"])
            for r in rows
            if r["passes_1mm"]
        ],
    )
write_json(
    Path("runs/outward_peak_diagnosis_v1/sweep.json"),
    {
        "scope": (
            "Sampled tool-only first ascent/descent, nominal strand; "
            "not continuous/IK/process certificate"
        ),
        "cases": results,
    },
)
