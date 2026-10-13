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
    base = read_json(Path(f"runs/{obj}_straight_outward_phase1_v1/construction_plan.json"))[
        "actions"
    ][0]
    bp, _ = flatten_curve(base, 0.00005, 0.003)
    bp = transform_points(bp, build)
    scene = MaterialScene()
    mid = []
    radii = []
    for j, (a, b) in enumerate(zip(bp[:-1], bp[1:], strict=True)):
        scene.add(a, b, 0.0005, f"base/{j}")
        mid.append((a + b) / 2)
        radii.append(np.linalg.norm(a - b) / 2)
    initial = len(mid)
    for j, (a, b) in enumerate(zip(points[:-1], points[1:], strict=True)):
        scene.add(a, b, 0.0005, f"wire/{j}")
        mid.append((a + b) / 2)
        radii.append(np.linalg.norm(a - b) / 2)
    mid = np.array(mid)
    radii = np.array(radii)
    body = next(b for b in bodies if b.name == "tool/nozzle_shank")
    rows = []
    for i, p in enumerate(points):
        nearby = np.where(
            np.linalg.norm(mid[: initial + i] - p, axis=1) < 0.033 + radii[: initial + i]
        )[0]
        radial = p[:2] - center[:2]
        radial /= np.linalg.norm(radial)
        options = []
        for tilt in [20, 30, 40, 50, 60]:
            for yaw in [-90, -60, -30, 0, 30, 60, 90]:
                a = np.deg2rad(yaw)
                r = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]) @ radial
                a = np.deg2rad(tilt)
                z = np.r_[-np.sin(a) * r, -np.cos(a)]
                x = np.array([1.0, 0, 0])
                x -= x @ z * z
                x /= np.linalg.norm(x)
                tcp = np.eye(4)
                tcp[:3, :3] = np.column_stack([x, np.cross(z, x), z])
                tcp[:3, 3] = p
                shape = placed(body.geometry, tcp @ np.linalg.inv(arm.flange_to_tcp) @ body.origin)
                d = min([float(fcl.distance(shape, scene.objects[j])) for j in nearby], default=1.0)
                options.append(
                    {
                        "tilt": tilt,
                        "yaw": yaw,
                        "gap_mm": d * 1000 if d >= 0 else None,
                        "passes": d >= 0.001,
                    }
                )
        rows.append({"sample": i, "options": options})
    summary = {}
    for limit in [30, 40, 50, 60]:
        best = [
            max(
                (o["gap_mm"] if o["gap_mm"] is not None else -1)
                for o in row["options"]
                if o["tilt"] <= limit
            )
            for row in rows
        ]
        summary[limit] = {
            "minimum_best_gap_mm": min(best),
            "failing_samples": sum(v < 1 for v in best),
            "worst_sample": int(np.argmin(best)),
        }
    results[obj] = {"summary": summary, "samples": rows}
    print(obj, summary, flush=True)
write_json(
    Path("runs/outward_peak_diagnosis_v1/adaptive_axes.json"),
    {
        "scope": (
            "Pointwise shank orientation feasibility only; discontinuous choices, "
            "not IK/continuous/fulltool certificate"
        ),
        "cases": results,
    },
)
