from pathlib import Path

import fcl
import numpy as np

from pen_art.cell import load_cell
from pen_art.curves import flatten_curve, transform_points
from pen_art.io import read_json, write_json
from pen_art.planning.collision import placed
from pen_art.planning.scene import MaterialScene

profile, arms, _ = load_cell(Path("research/planning_cell/cell_turntable.yaml"))
arm = arms["arm_1"]
build = np.asarray(profile["world_from_build"])
center = build[:3, 3]
bodies = [b for b in arm.collision.bodies if b.name.startswith("tool/")]
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
    scene = MaterialScene()
    for i in range(len(points) - 1):
        scene.add(points[i], points[i + 1], 0.0005, f"source/{i}")
    base = read_json(Path(f"runs/{obj}_straight_outward_phase1_v1/construction_plan.json"))[
        "actions"
    ][0]
    basepts, _ = flatten_curve(base, 0.00005, 0.003)
    basepts = transform_points(basepts, build)
    base_scene = MaterialScene()
    for j, (a, b) in enumerate(zip(basepts[:-1], basepts[1:], strict=True)):
        base_scene.add(a, b, 0.0005, f"base/{j}")
    rows = []
    for tilt, yaw in [(20, 0), (20, 60), (20, 90), (30, 30), (30, 60), (30, 90)]:
        worst = 1.0
        witness = None
        for i, p in enumerate(points[1:], 1):
            radial = p[:2] - center[:2]
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
            for body in bodies:
                shape = placed(body.geometry, tcp @ np.linalg.inv(arm.flange_to_tcp) @ body.origin)
                for j, wire in enumerate(base_scene.objects + scene.objects[:i]):
                    d = float(fcl.distance(shape, wire))
                    if d < worst:
                        worst = d
                        witness = {"sample": i, "obstacle": j, "body": body.name}
        rows.append(
            {
                "tilt": tilt,
                "yaw": yaw,
                "min_gap_mm": None if worst < 0 else worst * 1000,
                "overlap": worst < 0,
                "passes_1mm": worst >= 0.001,
                "witness": witness,
            }
        )
        print(obj, rows[-1], flush=True)
    results[obj] = {"source_samples": len(points), "rows": rows}
write_json(
    Path("runs/outward_peak_diagnosis_v1/full_curve.json"),
    {
        "scope": (
            "Sampled complete first zigzag tool bodies versus nominal prior wire plus base; "
            "no IK/continuous/process certificate"
        ),
        "cases": results,
    },
)
