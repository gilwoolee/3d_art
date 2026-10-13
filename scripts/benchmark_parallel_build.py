"""Time complete parallel builds of one source at several worker counts.

Each worker count gets a fresh run directory and process tree. All runs use
the same source, profile, frozen motion library and budgets. Wall time covers
the whole command: setup, planning, export and the full-robot replay. A run
only counts as complete when that replay certifies it.
"""

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path
from time import perf_counter, sleep

from pen_art.io import read_json, sha256, write_json


def tree(pid):
    found, pending = [], [pid]
    while pending:
        current = pending.pop()
        found.append(current)
        for task in Path(f"/proc/{current}/task").glob("*/children"):
            try:
                pending.extend(int(child) for child in task.read_text().split())
            except OSError:
                continue
    return found


def pss_mb(pid):
    total = 0.0
    for member in tree(pid):
        try:
            for line in Path(f"/proc/{member}/smaps_rollup").read_text().splitlines():
                if line.startswith("Pss:"):
                    total += int(line.split()[1]) / 1024
        except OSError:
            continue
    return total


def launch(args, workers):
    run = Path(f"{args.output}_w{workers}")
    if run.exists():
        raise SystemExit(f"Refusing to reuse existing run directory: {run}")
    command = [
        sys.executable,
        "-m",
        "pen_art.planning.section_build",
        "--cell-source-run",
        str(args.source_run),
        "--run",
        str(run),
        "--profile",
        str(args.profile),
        "--motion-library",
        str(args.motion_library),
        "--progression-only",
        "--maximum-seconds",
        str(args.maximum_seconds),
        "--action-budget",
        str(args.action_budget),
        "--workers",
        str(workers),
        *args.build_option,
    ]
    verification_command = [
        sys.executable, "-m", "pen_art.planning.cell_replay",
        "--run", str(run), "--config", str(args.profile),
        "--plan-name", "offline_dual_arm_plan",
        "--workers", str(workers), "--maximum-seconds", str(args.maximum_seconds),
    ]
    # Time the complete workflow while keeping verification a separate command.
    commands = [command, verification_command]
    launcher = [
        sys.executable, "-c",
        "import json, subprocess, sys; "
        "[subprocess.run(cmd, check=True) for cmd in json.loads(sys.argv[1])]",
        json.dumps(commands),
    ]
    log = Path(f"{run}.log").open("w")
    environment = {**os.environ, **dict.fromkeys(args.single_thread_variables, "1")}
    process = subprocess.Popen(launcher, stdout=log, stderr=subprocess.STDOUT, env=environment)
    return {
        "workers": workers,
        "run": run,
        "command": command,
        "verification_command": verification_command,
        "process": process,
        "started": perf_counter(),
        "peak_pss_mb": 0.0,
    }


def collect(job):
    run = job["run"]
    result = {
        "workers": job["workers"],
        "run": str(run),
        "command": job["command"],
        "verification_command": job["verification_command"],
        "exit_code": job["process"].returncode,
        "wall_seconds": job["wall_seconds"],
        "peak_process_tree_pss_mb": job["peak_pss_mb"],
    }
    build = read_json(run / "section_build.json")
    result.update(
        status=build["status"],
        completed_sections=build["completed_sections"],
        required_sections=build["required_sections"],
    )
    if (run / "parallel_build.json").exists():
        parallel = read_json(run / "parallel_build.json")
        result.update(
            {
                name: parallel[name]
                for name in (
                    "wall_seconds",
                    "job_seconds",
                    "serialization_seconds",
                    "jobs",
                    "outcomes",
                    "stale_results",
                    "lost_worker_jobs",
                    "resumed_job_results",
                    "selected",
                    "worker_peak_pss_total_mb",
                    "planning_status",
                    "replay_status",
                    "blocked",
                )
            }
        )
        result["phase_wall_seconds"] = result.pop("wall_seconds")
        result["wall_seconds"] = job["wall_seconds"]
        result["selected_deposits"] = [row["deposit_key"] for row in parallel["actions"]]
        result["repair_attempts"] = sum(row["attempts"] for row in parallel["actions"])
    validation = run / "offline_dual_arm_validation.json"
    if validation.exists():
        replay = read_json(validation)
        result["replay"] = {
            name: replay.get(name)
            for name in (
                "status",
                "collision_scope",
                "verification_raw",
                "checked_intervals",
                "checked_edges",
                "completed_strokes",
                "requested_strokes",
                "final_table_index",
                "final_material",
                "parallel_workers",
                "failures",
                "phase_seconds",
                "worker_stage_seconds",
                "verification_efficiency",
                "optimize_contact_geometry",
            )
        }
        result["plan_sha256"] = sha256(run / "offline_dual_arm_plan.json")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-run", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--output", required=True, help="Run prefix; _w<N> is appended")
    parser.add_argument(
        "--motion-library", type=Path, default=Path("runs/motion_library_phase1_upright27_v1")
    )
    parser.add_argument("--workers", type=int, nargs="+", default=[1, 2, 4])
    parser.add_argument("--maximum-seconds", type=float, default=14400)
    parser.add_argument(
        "--build-option",
        action="append",
        default=[],
        help="Extra planning flag passed through, e.g. --build-option=--geometry-first",
    )
    parser.add_argument("--action-budget", type=float, default=240.0)
    parser.add_argument(
        "--concurrent",
        action="store_true",
        help="Run all worker counts at once. Timings then share the machine; say so when reported",
    )
    parser.add_argument(
        "--single-thread-variables",
        nargs="*",
        default=["OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"],
        help="Set to 1 so each process uses one numerical thread",
    )
    args = parser.parse_args()
    groups = [args.workers] if args.concurrent else [[count] for count in args.workers]
    results = []
    for group in groups:
        jobs = [launch(args, count) for count in group]
        while any("wall_seconds" not in job for job in jobs):
            for job in jobs:
                if "wall_seconds" in job:
                    continue
                job["peak_pss_mb"] = max(job["peak_pss_mb"], pss_mb(job["process"].pid))
                if job["process"].poll() is not None:
                    job["wall_seconds"] = perf_counter() - job["started"]
            sleep(2)
        results.extend(collect(job) for job in jobs)
        for row in results[-len(jobs) :]:
            print(row["workers"], row["status"], f"{row['wall_seconds']:.1f}s", flush=True)
    complete = [row for row in results if row["status"] == "FULL_BUILD_VERIFIED"]
    baseline = next((row for row in complete if row["workers"] == min(args.workers)), None)
    summary = {
        "scope": "Planning/export followed by a separate full-robot verification command",
        "source_run": str(args.source_run),
        "profile": str(args.profile),
        "motion_library": str(args.motion_library),
        "motion_library_frozen": True,
        "concurrent_runs": args.concurrent,
        "cpu_count": os.cpu_count(),
        "memory_sampling_interval_s": 2,
        "memory_scope": "Maximum observed simultaneous process-tree PSS, sampled every 2 s",
        "runs": results,
        "all_complete": len(complete) == len(results),
        # Different worker counts may select different candidates. Each schedule
        # is certified by its own replay; equality is reported, never assumed.
        "identical_selected_deposits": len({tuple(r["selected_deposits"]) for r in complete})
        <= 1,
        "identical_final_material": len(
            {(r["replay"]["final_material"] or {}).get("geometry_sha256") for r in complete}
        )
        <= 1,
        "speedup_over_fewest_workers": {
            str(row["workers"]): baseline["wall_seconds"] / row["wall_seconds"] for row in complete
        }
        if baseline
        else {},
    }
    write_json(Path(f"{args.output}_benchmark.json"), summary)
    return 0 if summary["all_complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
