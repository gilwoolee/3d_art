"""Publish immutable viewer copies of verified cache/learned reference builds."""

import argparse
import json
import re
import shutil
from pathlib import Path

import numpy as np

from pen_art.io import read_json, sha256, write_json
from pen_art.planning.build_metrics import execution_metrics
from pen_art.planning.motion_library import MotionLibrary, digest
from pen_art.planning.process_clock import PROCESS_CLOCK_POLICY


def benchmark_summary(root):
    directory = root / "runs/motion_learning_final"
    combined = read_json(directory / "benchmark_test_combined.json")
    reports = [
        read_json(directory / name / "benchmark.json")
        for name in ("benchmark_test", "benchmark_test_remaining")
    ]
    rows = [row for report in reports for row in report["results"]]
    requests = {}
    for row in rows:
        request = requests.setdefault(
            row["id"],
            {
                "id": row["id"],
                "source_object": row["source_object"],
                "family_id": row["family_id"],
                "variation": row["variation"],
                "scene_kind": row["scene_kind"],
                "modes": {},
            },
        )
        request["modes"][row["mode"]] = {
            key: row[key] for key in ("verified", "seconds", "accepted_source", "generated_count")
        }
    diagnostics = read_json(directory / "teacher_failure_diagnostic.json")
    return {
        "requests": list(requests.values()),
        "summary": combined["summary"],
        "objects": sorted({row["source_object"] for row in rows}),
        "stroke_families": sorted({row["family_id"] for row in rows}),
        "budget_s": reports[0]["seconds_budget"],
        "training_records": reports[0]["training_records"],
        "failure_diagnostics": diagnostics["results"],
        "scope": "15 variants of one torus stroke; local DRAW queries, not 15 objects or builds",
    }


def publish(root, source, library_path, run_id, label, benchmark=None, *, continuous=False):
    source = (root / source).resolve()
    destination = root / "runs" / run_id
    if destination.exists():
        raise FileExistsError(f"Keep existing viewer evidence; choose a fresh ID: {destination}")
    plan = read_json(source / "offline_dual_arm_plan.json")
    replay = read_json(source / "offline_dual_arm_validation.json")
    state = read_json(source / "section_build.json")
    if (
        state["status"] != "FULL_BUILD_VERIFIED"
        or replay["status"] != "CERTIFIED_GEOMETRIC_REPLAY"
        or replay["source_plan_sha256"] != sha256(source / "offline_dual_arm_plan.json")
    ):
        raise ValueError(f"Source must have a complete matching replay: {source}")
    for name, expected in plan["inputs"].items():
        if sha256(source / name) != expected:
            raise ValueError(f"Stale source input: {name}")
    if continuous and any(
        action["trajectory"].get("process_clock_policy") != PROCESS_CLOCK_POLICY
        for action in plan["actions"]
        if not action.get("auxiliary", False)
    ):
        raise ValueError("Continuous publication requires current process clocks")
    records = MotionLibrary(root / library_path).records
    accepted = [r for r in records if (root / r["provenance"].get("run", "")).resolve() == source]
    by_section = {r["provenance"]["section"]: r for r in accepted}
    by_digest = {digest(record): record for record in records}
    inventory = []
    for record in records:
        points = np.asarray(record["request"]["source_points_m"])
        inventory.append(
            {
                "record_id": digest(record),
                "section": record["provenance"].get("section"),
                "source_run": record["provenance"].get("run"),
                "origin": record["provenance"].get("proposal_source", "planner"),
                "length_mm": float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum() * 1000),
                "table_index": record["request"]["table_index"],
                "joint_samples": len(record["trajectory"]["q_path"]),
                "path_validation": record["trajectory"].get("q_path_validation"),
                "has_exact_clock": record["trajectory"].get("exact_clock") is not None,
            }
        )
    strokes = []
    for action in plan["actions"]:
        if action.get("auxiliary"):
            continue
        if continuous:
            key = action.get("motion_library_record")
            if key not in by_digest:
                raise ValueError(f"Missing final accepted motion record: {action['stroke_id']}")
            record = by_digest[key]
        else:
            record = by_section[action["stroke_id"]]
        points = record["request"]["source_points_m"]
        draw_edges = [edge for edge in action["trajectory"]["edges"] if edge["kind"] == "DRAW"]
        strokes.append(
            {
                "section": action["stroke_id"],
                "parent_curve": action.get("parent_curve"),
                "origin": record["provenance"].get("proposal_source", "planner"),
                "record_id": digest(record),
                "source_points_m": points,
                "length_mm": float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum() * 1000),
                "table_index": record["request"]["table_index"],
                "start_s": action["start_s"],
                "end_s": action["end_s"],
                "draw_start_s": action["start_s"] + draw_edges[0]["start_s"],
            }
        )
    # Plain copies keep evidence independent; source plans and their hashes are unchanged.
    destination.mkdir()
    for path in source.iterdir():
        if path.is_file():
            shutil.copy2(path, destination / path.name)
    metadata = read_json(destination / "run.json")
    metadata.update(
        id=run_id,
        name=label,
        workbench_primary=continuous,
        workbench_collection="motion_reuse",
        viewer_source_run=str(source),
    )
    write_json(destination / "run.json", metadata)
    reuse = read_json(source / "motion_reuse.json")
    computation = None
    for name in ("integration.json", "progress.json"):
        receipt = source.parent / name
        if receipt.exists():
            rows = read_json(receipt).get("runs", [])
            computation = next(
                (
                    {
                        key: row[key]
                        for key in ("seconds", "planning_seconds", "export_replay_seconds")
                        if key in row
                    }
                    for row in rows
                    if (root / row["run"]).resolve() == source
                ),
                None,
            )
            if computation:
                break
    write_json(
        destination / "viewer_motion_reuse.json",
        {
            "schema_version": 1,
            "label": label,
            "source_run": str(source.relative_to(root)),
            "source_plan_sha256": sha256(source / "offline_dual_arm_plan.json"),
            "inputs": {
                name: sha256(destination / name)
                for name in (
                    "offline_dual_arm_plan.json",
                    "offline_dual_arm_validation.json",
                    "motion_reuse.json",
                )
            },
            "phase": 2 if reuse["mode"] == "cache_and_diffusion" else 1,
            "runtime": reuse,
            "strokes": strokes,
            "library_entries": inventory,
            "benchmark": benchmark,
            "execution": execution_metrics(plan),
            "computation": computation,
            "continuous_actions": continuous,
            "notes": [
                "Cached drawing seeds are adapted and validated against the current structure.",
                "Exact accepted clocks are stored; sampled seed paths are not new certificates.",
                "Self/static clearance reuse excludes changing deposited material.",
                (
                    "Continuous-action simulation; physical extrusion remains uncalibrated."
                    if continuous
                    else "Reference prism simulation; not a generated full-object acceptance test."
                ),
            ],
        },
    )
    write_json(
        destination / "viewer_publication.json",
        {
            "source_run": str(source),
            "unchanged_plan_sha256": sha256(source / "offline_dual_arm_plan.json"),
            "unchanged_replay_sha256": sha256(source / "offline_dual_arm_validation.json"),
            "recorded_strokes": len(strokes),
            "status": "PUBLISHED_VERIFIED_COPY",
        },
    )
    return {"id": run_id, "url": f"/?run={run_id}&stage=motion", "strokes": len(strokes)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path.cwd())
    parser.add_argument("--suffix", default="", help="Use a new suffix to preserve existing copies")
    parser.add_argument("--source-build", type=Path, help="Publish a new continuous-action build")
    parser.add_argument("--library", type=Path)
    parser.add_argument("--run-id")
    parser.add_argument("--label", default="Phase 1 · continuous drawing")
    args = parser.parse_args()
    if args.suffix and not re.fullmatch(r"[A-Za-z0-9_-]+", args.suffix):
        parser.error("--suffix must contain only letters, digits, underscores or hyphens")
    root = args.root.resolve()
    if args.source_build is not None:
        if not args.library or not args.run_id:
            parser.error("--source-build requires --library and --run-id")
        if not re.fullmatch(r"[A-Za-z0-9_-]+", args.run_id):
            parser.error("--run-id must contain only letters, digits, underscores or hyphens")
        print(
            json.dumps(
                publish(
                    root, args.source_build, args.library, args.run_id, args.label, continuous=True
                ),
                indent=2,
            )
        )
        return
    benchmark = benchmark_summary(root)
    cases = [
        (
            "motion_runtime_prism_v2/cold",
            "motion_runtime_prism_v2/library",
            "phase1_cache_cold",
            "Phase 1 · prism · first build",
        ),
        (
            "motion_runtime_prism_v2/warm",
            "motion_runtime_prism_v2/library",
            "phase1_cache_warm",
            "Phase 1 · prism · cached replay",
        ),
        (
            "motion_runtime_diffusion_v2/cold",
            "motion_runtime_diffusion_v2/library",
            "phase2_diffusion_cold",
            "Phase 2 · prism · learned fallback",
        ),
    ]
    suffix = f"_{args.suffix}" if args.suffix else ""
    print(
        json.dumps(
            [
                publish(
                    root, f"runs/{source}", f"runs/{library}", run_id + suffix, label, benchmark
                )
                for source, library, run_id, label in cases
            ],
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
