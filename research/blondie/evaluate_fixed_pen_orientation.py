"""Sample constant TCP rotations against source-prefix wire and actual tool/fixture."""

import argparse
from pathlib import Path
from time import monotonic

import fcl
import numpy as np

from pen_art.cell import load_cell
from pen_art.curves import flatten_curve, transform_points
from pen_art.io import read_json, sha256, write_json
from pen_art.planning.collision import placed
from pen_art.planning.scene import MaterialScene


def rotation(tilt, azimuth, roll):
    t, a, r = np.deg2rad([tilt, azimuth, roll])
    z = np.array([np.sin(t) * np.cos(a), np.sin(t) * np.sin(a), -np.cos(t)])
    x = np.array([0.0, 0.0, 1.0] if abs(z[2]) < 0.99 else [1.0, 0.0, 0.0])
    x -= x @ z * z
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([np.cos(r) * x + np.sin(r) * y, -np.sin(r) * x + np.cos(r) * y, z])


def evaluate(obj, args):
    run = Path(f"runs/{obj}_{args.run_suffix}")
    profile_path = run / "profile.yaml"
    profile, arms, obstacles = load_cell(profile_path)
    arm = arms["arm_1"]
    build = np.asarray(profile["world_from_build"])
    actions = read_json(run / "construction_plan.json")["actions"]
    zigzag = next(a for a in actions if a.get("parent_curve", "").startswith("zigzag-"))
    base = next(a for a in actions if a.get("parent_curve", "").startswith("contour-"))
    points, _ = flatten_curve(zigzag, 0.00005, args.step_m)
    points = transform_points(points, build)
    bp, _ = flatten_curve(base, 0.00005, args.step_m)
    bp = transform_points(bp, build)
    scene = MaterialScene()
    prefix = MaterialScene()
    for i, (a, b) in enumerate(zip(bp[:-1], bp[1:], strict=True)):
        scene.add(a, b, base["diameter_m"] / 2, f"base/{i}")
    offsets = [
        (b, np.linalg.inv(arm.flange_to_tcp) @ b.origin)
        for b in arm.collision.bodies
        if b.name.startswith("tool/")
    ]
    candidates = []
    for tilt in args.tilts:
        for azimuth in (args.azimuths or np.arange(0.0, 360.0, args.azimuth_step)):
            for roll in args.rolls:
                pose = np.eye(4)
                pose[:3, :3] = rotation(tilt, azimuth, roll)
                shapes = [
                    (b.name, placed(b.geometry, pose @ offset), offset) for b, offset in offsets
                ]
                candidates.append(
                    {
                        "tilt_deg": tilt,
                        "azimuth_deg": float(azimuth),
                        "roll_deg": roll,
                        "minimum_gap_m": float("inf"),
                        "colliding_samples": 0,
                        "below_margin_samples": 0,
                        "first_failure": None,
                        "worst": None,
                        "group_minima_m": {},
                        "group_witnesses": {},
                        "_pose": pose,
                        "_shapes": shapes,
                    }
                )
    started = monotonic()
    for i, p in enumerate(points):
        if i:
            prefix.add(points[i - 1], p, zigzag["diameter_m"] / 2, f"prefix/{i - 1}")
        for c in candidates:
            c["_pose"][:3, 3] = p
            sample_gap, sample_witness = float("inf"), None
            for name, shape, offset in c["_shapes"]:
                transform = c["_pose"] @ offset
                shape.setTransform(fcl.Transform(transform[:3, :3], transform[:3, 3]))
                tests = []
                for kind, material in [("base", scene), ("own_prefix", prefix)]:
                    nearest = material.nearest(shape)
                    if nearest is not None:
                        tests.append((kind, nearest["name"], nearest["distance_m"]))
                tests += [
                    ("static", key, float(fcl.distance(shape, other)))
                    for key, other in obstacles.items()
                ]
                for kind, other, gap in tests:
                    if gap < c["group_minima_m"].get(kind, float("inf")):
                        c["group_minima_m"][kind] = gap
                        c["group_witnesses"][kind] = {
                            "sample": i,
                            "tcp_m": p.tolist(),
                            "body": name,
                            "other": other,
                            "distance_m": gap,
                        }
                    if gap < sample_gap:
                        sample_gap = gap
                        sample_witness = {
                            "sample": i,
                            "tcp_m": p.tolist(),
                            "body": name,
                            "kind": kind,
                            "other": other,
                            "distance_m": gap,
                        }
            if sample_gap < c["minimum_gap_m"]:
                c["minimum_gap_m"], c["worst"] = sample_gap, sample_witness
            c["colliding_samples"] += int(sample_gap < 0)
            c["below_margin_samples"] += int(sample_gap < 0.001)
            if sample_gap < 0.001 and c["first_failure"] is None:
                c["first_failure"] = sample_witness
        if i % 40 == 0:
            print(
                obj,
                "sample",
                i,
                "/",
                len(points),
                "seconds",
                round(monotonic() - started, 2),
                flush=True,
            )
    for c in candidates:
        c.pop("_pose")
        c.pop("_shapes")
        c["sampled_clearance_pass"] = c["minimum_gap_m"] >= 0.001
    candidates.sort(
        key=lambda c: (-c["minimum_gap_m"], c["colliding_samples"], c["below_margin_samples"])
    )
    return {
        "run": str(run),
        "profile_sha256": sha256(profile_path),
        "source_sha256": sha256(run / "construction_plan.json"),
        "section": zigzag["id"],
        "source_legs": len(zigzag["segments"]),
        "samples": len(points),
        "base_section": base["id"],
        "base_capsules": len(bp) - 1,
        "seconds": monotonic() - started,
        "passing_candidates": sum(c["sampled_clearance_pass"] for c in candidates),
        "candidates": candidates,
        "interpretation": (
            "FCL negative distances are overlap sentinels, not measured penetration depth."
        ),
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--objects", nargs="+", default=["cube", "bottle"])
    p.add_argument("--tilts", nargs="+", type=float, default=[70.0, 80.0])
    p.add_argument("--run-suffix", default="fixed70_phase1_v1")
    p.add_argument("--rolls", nargs="+", type=float, default=[0.0, 45.0])
    p.add_argument("--azimuth-step", type=float, default=15.0)
    p.add_argument("--azimuths", nargs="+", type=float)
    p.add_argument("--step-m", type=float, default=0.0015)
    p.add_argument("--output", type=Path, default=Path("runs/fixed_pen_orientation_v1"))
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    results = {}
    for obj in args.objects:
        results[obj] = evaluate(obj, args)
        write_json(
            args.output / "summary.json",
            {
                "scope": (
                    "Sampled full first zigzag tool-only fixed-rotation feasibility with "
                    "deposited base and own prefix; no robot IK or continuous motion certificate."
                ),
                "script_sha256": sha256(Path(__file__)),
                "step_m": args.step_m,
                "azimuth_step_deg": args.azimuth_step,
                "results": results,
            },
        )
        print(
            obj,
            "COMPLETE",
            "passing",
            results[obj]["passing_candidates"],
            "best",
            results[obj]["candidates"][0],
            flush=True,
        )
