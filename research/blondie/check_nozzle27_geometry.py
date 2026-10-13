"""Screen full fixed-upright source order against nominal growing material."""

import argparse
from pathlib import Path
from time import monotonic

import fcl
import numpy as np

from pen_art.cell import load_cell
from pen_art.curves import flatten_curve, transform_points
from pen_art.io import read_json, sha256, write_json
from pen_art.planning.cell_motion import pen_rotation
from pen_art.planning.collision import placed
from pen_art.planning.scene import MaterialScene


def evaluate(run, step):
    profile, arms, obstacles = load_cell(run / "profile.yaml")
    arm = arms["arm_1"]
    transform = np.asarray(profile["world_from_build"])
    actions = read_json(run / "construction_plan.json")["actions"]
    outcomes = []
    for roll in (0., np.pi / 4):
        pose = np.eye(4)
        pose[:3, :3] = pen_rotation(arm, transform, roll, 0.)
        shapes = [(body.name, placed(body.geometry, pose),
                   np.linalg.inv(arm.flange_to_tcp) @ body.origin)
                  for body in arm.collision.bodies if body.name.startswith("tool/")]
        scene = MaterialScene()
        rows = []
        for action in actions:
            points, _ = flatten_curve(action, .00005, step)
            points = transform_points(points, transform)
            row = {"section": action["id"], "source": action["parent_curve"],
                   "samples": len(points), "minimum_gap_m": float("inf"),
                   "overlapping_samples": 0, "below_margin_samples": 0, "worst": None}
            for i, point in enumerate(points):
                if i:
                    scene.add(points[i-1], point, action["diameter_m"]/2,
                              f"{action['id']}/{i}")
                pose[:3, 3] = point
                gap = float("inf")
                witness = None
                for name, shape, offset in shapes:
                    frame = pose @ offset
                    shape.setTransform(fcl.Transform(frame[:3, :3], frame[:3, 3]))
                    near = scene.nearest(shape)
                    pairs = [] if near is None else [(near["name"], near["distance_m"])]
                    pairs.extend((key, float(fcl.distance(shape, other)))
                                 for key, other in obstacles.items())
                    for other, distance in pairs:
                        if distance < gap:
                            gap = distance
                            witness = {"sample": i, "body": name, "other": other,
                                       "tcp_m": point.tolist(), "distance_m": distance}
                row["overlapping_samples"] += gap < 0
                row["below_margin_samples"] += gap < .001
                if gap < row["minimum_gap_m"]:
                    row["minimum_gap_m"], row["worst"] = gap, witness
            row["overlapping_samples"] = int(row["overlapping_samples"])
            row["below_margin_samples"] = int(row["below_margin_samples"])
            rows.append(row)
            print(run.name, round(np.rad2deg(roll)), action["id"],
                  round(row["minimum_gap_m"]*1000, 3), flush=True)
        outcomes.append({"roll_deg": float(np.rad2deg(roll)), "actions": rows,
                         "passing_actions": sum(r["below_margin_samples"] == 0 for r in rows),
                         "minimum_gap_m": min(r["minimum_gap_m"] for r in rows)})
    return {"run": str(run), "profile_sha256": sha256(run/"profile.yaml"),
            "construction_sha256": sha256(run/"construction_plan.json"),
            "total_actions": len(actions), "orientations": outcomes}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", nargs="+", type=Path, required=True)
    parser.add_argument("--step-m", type=float, default=.0005)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    started = monotonic()
    report = {"scope": "Sampled tool-only screen of nominal source order and growing wire; "
              "no robot IK, actual deposition inflation, support, timing or replay certificate",
              "step_m": args.step_m, "margin_m": .001, "results": {}}
    for run in args.runs:
        report["results"][run.name] = evaluate(run, args.step_m)
        report["seconds"] = monotonic() - started
        write_json(args.output, report)
