"""Use precomputed whole-cube zigzags with fresh material/process checks.

Only local cache retrieval/adaptation is allowed. Cold whole-curve sampling is
disabled. The existing base is an explicit cooled-prefix precondition.
"""

import argparse
from pathlib import Path
from shutil import copytree
from time import monotonic

from pen_art.cell import prepare_cell
from pen_art.io import read_json, sha256, write_json
from pen_art.planning.dual_arm import ScheduledMotion
from pen_art.planning.indexed_build import IndexedMotions
from pen_art.planning.section_build import SectionBuild, prepare_candidates


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--library", type=Path, default=Path("runs/motion_cache_precompute_v1/library")
    )
    parser.add_argument("--output", type=Path, default=Path("runs/whole_zigzag_cache_runtime_v1"))
    parser.add_argument("--maximum-seconds", type=float, default=90)
    parser.add_argument("--source-run", type=Path, default=Path("runs/cube_continuous_phase1_v1"))
    parser.add_argument(
        "--profile", type=Path, default=Path("research/planning_cell/cell_turntable.yaml")
    )
    parser.add_argument("--section", default="section-00001")
    parser.add_argument("--base-section", default="section-00000")
    args = parser.parse_args()
    source_run = args.source_run
    source = Path(read_json(source_run / "run.json")["cell_source_run"])
    profile = args.profile
    prepare_cell(profile, args.output, source)
    prepare_candidates(args.output)
    copytree(args.library, args.output / "library")
    build = SectionBuild(
        args.output, profile, args.maximum_seconds, motion_library=args.output / "library"
    )
    model = IndexedMotions(build)
    model.warm.clear()
    cold_calls = 0

    class ColdSolverDisabled(RuntimeError):
        pass

    def no_cold_solver(*args, **kwargs):
        nonlocal cold_calls
        cold_calls += 1
        raise ColdSolverDisabled("Cold geometric sampling is disabled in this cache-only probe")

    model.drawings = no_cold_solver
    state = build.state({args.base_section}, build.initial)
    state.payload.table_index = model.table.home_index
    started = monotonic()
    accepted = None
    report = {
        "scope": "cached whole zigzag after explicitly cooled base",
        "source_run": str(source_run),
        "section": args.section,
        "base_section": args.base_section,
        "library_source": str(args.library),
        "full_object_execution": False,
        "graph_sha256": sha256(args.output / "stroke_graph.json"),
    }
    try:
        accepted = next(iter(model.forward_motions(build.actions[args.section], state)), None)
        report["status"] = (
            "NO_ACCEPTED_CACHED_MOTION_WITHIN_BUDGET" if accepted is None else "ACCEPTED"
        )
    except ColdSolverDisabled:
        report["status"] = "NO_ACCEPTED_CACHED_MOTION_BEFORE_COLD_FALLBACK"
    except TimeoutError as error:
        report.update(status="CACHE_RUNTIME_TIME_BUDGET", reason=str(error))
    if accepted is not None:
        move, drawing, _ = accepted
        write_json(
            args.output / "prepared_witness.json",
            {"drawing": drawing, "trajectory": move.payload.record()},
        )
        write_json(
            args.output / "result.json",
            {
                **report,
                "status": "PREPARED_PENDING_INDEPENDENT_REPLAY",
                "seconds": monotonic() - started,
            },
        )
        # Recheck the actual accepted clock with fresh collision models; cached
        # clearance is only an acceleration mechanism in the first pass.
        build.arm.collision = build.arm.collision.model
        identity = build.arm.identity
        build.planner.tip_models[identity] = build.planner.tip_models[identity].model
        try:
            _, failure = build.planner.validate_interval(
                state, (ScheduledMotion(move, 0),), move.duration_s
            )
            report.update(
                status="LOCAL_CACHE_MOTION_INDEPENDENTLY_VERIFIED"
                if failure is None
                else "INDEPENDENT_REPLAY_REJECTED",
                independent_failure=failure,
            )
            if failure is None:
                write_json(
                    args.output / "accepted_witness.json",
                    {"drawing": drawing, "trajectory": move.payload.record()},
                )
        except TimeoutError as error:
            report.update(status="INDEPENDENT_REPLAY_TIME_BUDGET", reason=str(error))
    report.update(
        seconds=monotonic() - started,
        cold_solver_calls=cold_calls,
        cold_geometric_sampling_performed=False,
        blocked_cold_fallback_attempts=cold_calls,
        motion_reuse=build.motion_runtime.report(),
        failures=build.planner.attempts,
    )
    build.motion_runtime.flush()
    report["runtime_parent_ids"] = sorted(
        {
            record["provenance"]["motion_library_parent"]
            for path in (args.output / "library/attempts").glob("*.json")
            for record in [read_json(path)["record"]]
            if record.get("provenance", {}).get("run") == str(args.output)
            and record["provenance"].get("motion_library_parent")
        }
    )
    write_json(args.output / "result.json", report)
    print(report["status"], report["seconds"], report["motion_reuse"]["counts"], flush=True)


if __name__ == "__main__":
    main()
