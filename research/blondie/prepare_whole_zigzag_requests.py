"""Inventory actual object zigzags and propose one complete static-only seed.

The manifest retains exact source curves. Seeded paths are proposals requiring
the cache generator's independent self/static/tracking checks before admission.
"""

import argparse
from pathlib import Path
from time import monotonic

import numpy as np

from pen_art.curves import flatten_curve
from pen_art.io import read_json, sha256, write_json
from pen_art.planning.cell_motion import drawing_alternatives
from pen_art.planning.clock_path import ClockPath
from pen_art.planning.section_build import SectionBuild


def inventory(output):
    requests, geometries, families = [], [], {}
    for name in ("cube", "bottle"):
        run = Path(f"runs/{name}_continuous_phase1_v1")
        construction = read_json(run / "construction_plan.json")
        for row in construction["actions"]:
            if not row["parent_curve"].startswith("zigzag-"):
                continue
            points, _ = flatten_curve(row, 0.00005, 0.003)
            points = np.asarray(points)
            family = f"{name}-perimeter-zigzag-{len(row['segments'])}-legs"
            dimensions = np.ptp(points, axis=0)
            geometry = {
                "run": str(run),
                "section": row["id"],
                "parent_curve": row["parent_curve"],
                "family_id": family,
                "members": row["members"],
                "segments": row["segments"],
                "source_intervals": row["source_intervals"],
                "source_points_m": points.tolist(),
                "bounds_m": [points.min(axis=0).tolist(), points.max(axis=0).tolist()],
                "dimensions_m": dimensions.tolist(),
                "length_m": row["length_m"],
                "leg_count": len(row["segments"]),
                "source_graph_sha256": sha256(run / "stroke_graph.json"),
                "construction_sha256": sha256(run / "construction_plan.json"),
            }
            geometries.append(geometry)
            families.setdefault(family, []).append(geometry)
            for index in range(4):
                for reverse in (False, True):
                    requests.append(
                        {
                            "run": str(run),
                            "section": row["id"],
                            "parent_curve": row["parent_curve"],
                            "table_index": index,
                            "reverse": reverse,
                            "start_visit": 0,
                            "family_id": family,
                            "posture_alternatives_requested": 3,
                        }
                    )
    summaries = []
    for name, rows in families.items():
        dims = np.asarray([r["dimensions_m"] for r in rows])
        summaries.append(
            {
                "family_id": name,
                "actual_actions": len(rows),
                "leg_count": rows[0]["leg_count"],
                "representative": {key: rows[0][key] for key in ("run", "section", "parent_curve")},
                "dimension_min_m": dims.min(axis=0).tolist(),
                "dimension_max_m": dims.max(axis=0).tolist(),
                "height_range_m": [
                    min(r["bounds_m"][0][2] for r in rows),
                    max(r["bounds_m"][1][2] for r in rows),
                ],
                "required_variations": [
                    "exact object-specific member lengths",
                    "both traversal directions",
                    "all four stationary table indices",
                    "several redundant arm postures per request",
                ],
            }
        )
    write_json(
        output / "geometry_inventory.json",
        {"status": "EXACT_SOURCE_INVENTORY", "families": summaries, "actions": geometries},
    )
    write_json(
        output / "request_manifest.json",
        {
            "requests": requests,
            "scope": "actual full cube and bottle zigzags; self/static motion proposals only",
            "actual_source_actions": len(geometries),
        },
    )
    return geometries, requests


def prototype(output, geometries, seconds, *, roll_angles=(0.0, np.pi), reverse=None):
    run = Path(geometries[0]["run"])
    profile = Path("research/planning_cell/cell_turntable.yaml")
    build = SectionBuild(run, profile, seconds, motion_library=output / "unused_library")
    row = build.actions[geometries[0]["section"]]
    seeds, provenance = [build.initial], []
    old_run = Path("runs/cube_stl_forward_turntable_v1")
    checkpoint = old_run / "forward_checkpoint.json"
    if checkpoint.exists():
        references = read_json(checkpoint).get("witnesses", [])
        if len(references) > 1:
            path = old_run / references[1]["file"]
            saved = read_json(path)
            clock = ClockPath.from_record(saved["trajectory"])
            first = next((e.q0 for e in clock.edges if e.kind == "DRAW"), None)
            if first is not None:
                seeds.insert(0, first)
                provenance.append(
                    {
                        "kind": "single verified leg used only as an IK seed",
                        "file": str(path),
                        "sha256": sha256(path),
                    }
                )
    started = monotonic()
    failures = []
    available = (
        {row["start"], row["end"]} if reverse is None else {row["end"] if reverse else row["start"]}
    )
    for drawing in drawing_alternatives(
        build.arm,
        row,
        available,
        build.transform,
        build.planner.obstacles,
        deadline=build.planner.deadline,
        seeds=seeds,
        settings={"maximum_subdivision_depth": 20},
        clearance_m=build.planner.margin,
        pen_tilt_deg=build.planner.profile.get("drawing_pen_tilt_deg", 0),
        roll_angles=roll_angles,
    ):
        if reverse is not None and drawing.get("reverse") != reverse:
            continue
        if drawing["status"] != "SAMPLED_DRAWING_PATH":
            failures.append(
                {k: v for k, v in drawing.items() if k not in {"tcp_targets_m", "positions"}}
            )
            continue
        drawing.update(table_index=0, path_kind="zigzag")
        write_json(output / "drawing.json", drawing)
        report = {
            "status": "UNTRUSTED_WHOLE_ZIGZAG_PROPOSAL",
            "run": str(run),
            "section": row["id"],
            "parent_curve": row["parent_curve"],
            "table_index": 0,
            "seed_drawing_path": str(output / "drawing.json"),
            "seed_provenance": provenance,
            "members": len(row["members"]),
            "leg_count": len(row["segments"]),
            "path_samples": len(drawing["positions"]),
            "seconds": monotonic() - started,
            "failures": failures,
            "next_gate": "independent whole-path self/static/source validation before admission",
        }
        write_json(output / "prototype.json", report)
        return report
    report = {
        "status": "NO_COMPLETE_STATIC_PROPOSAL_WITHIN_BUDGET",
        "failures": failures,
        "seconds": monotonic() - started,
    }
    write_json(output / "prototype.json", report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("runs/whole_zigzag_seed_probe_v1"))
    parser.add_argument("--maximum-seconds", type=float, default=180)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    geometries, requests = inventory(args.output)
    report = prototype(args.output, geometries, args.maximum_seconds)
    if report["status"] == "UNTRUSTED_WHOLE_ZIGZAG_PROPOSAL":
        requests[0]["seed_drawing_path"] = report["seed_drawing_path"]
        write_json(
            args.output / "request_manifest.json",
            {
                "requests": requests,
                "scope": "actual full cube and bottle zigzags; self/static motion proposals only",
                "actual_source_actions": len(geometries),
            },
        )
    print(report["status"], flush=True)


if __name__ == "__main__":
    main()
