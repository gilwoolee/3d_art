"""Build and independently replay a reference object through the shared cache.

Runs two fresh indexed builds: the first collects accepted motions and the
second retrieves them. An optional learned checkpoint adds generated proposals
to that same planner path. This exercises approach, drawing, departure, timing,
construction readiness and terminal return, beyond local drawing benchmarks.
"""

import argparse
from pathlib import Path
from time import perf_counter

import numpy as np

from pen_art.cell import prepare_cell
from pen_art.curves import flatten_curve, transform_points
from pen_art.io import read_json, write_json
from pen_art.planning.dual_arm import ScheduledMotion
from pen_art.planning.full_build_export import export_sections
from pen_art.planning.indexed_build import IndexedMotions
from pen_art.planning.regression import greedy_forward_order
from pen_art.planning.section_build import SectionBuild, prepare_candidates
from pen_art.planning.source_coverage import audit_source_curve


def check_backward(build, model, key):
    """Exercise cached backward search, then replay its result without caches."""
    predecessor = set(build.actions) - {key}
    terminal = (*build.initial, model.table.home_index)
    row = build.actions[key]
    available = build.state(predecessor, build.initial).ready_nodes()
    result = next(
        (item for item in model.motions(row, predecessor, available, terminal) if item is not None),
        None,
    )
    if result is None:
        raise RuntimeError("No verified backward motion returned")
    initial, (_, move, drawing) = result
    if drawing.get("proposal_source") != "cache":
        raise RuntimeError("Backward check did not exercise cached adaptation")
    points, _ = flatten_curve(row, 0.00005, 0.003)
    source = audit_source_curve(
        build.arm, [move.payload], transform_points(points, model.table.build_transform(initial[7]))
    )
    if source["status"] != "BOUNDED":
        raise RuntimeError("Backward source coverage rejected")
    # Independent models enforce the same actual geometry with no cache hits.
    arm_cache = build.arm.collision
    tip_cache = build.planner.tip_models[build.arm.identity]
    try:
        build.arm.collision = arm_cache.model
        build.planner.tip_models[build.arm.identity] = tip_cache.model
        state = build.state(predecessor, initial[:7], model.scene(predecessor, initial[7]))
        _, failure = build.planner.validate_interval(
            state, (ScheduledMotion(move, 0),), move.duration_s
        )
        if failure:
            raise RuntimeError(f"Backward independent replay rejected: {failure}")
    finally:
        build.arm.collision = arm_cache
        build.planner.tip_models[build.arm.identity] = tip_cache
    return {"status": "VERIFIED", "section": key, "source": "cache", "source_audit": source}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-build", type=Path, required=True)
    parser.add_argument(
        "--variation-source-build",
        type=Path,
        help="A nearby changed-geometry build; run it against the same populated library",
    )
    parser.add_argument(
        "--allow-straight-diagnostic",
        action="store_true",
        help="Permit a legacy straight-stroke diagnostic instead of whole-path acceptance",
    )
    parser.add_argument(
        "--profile", type=Path, default=Path("research/planning_cell/cell_turntable.yaml")
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--maximum-seconds", type=float, default=600)
    parser.add_argument("--diffusion-checkpoint", type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Use a fresh output directory")
    args.output.mkdir(parents=True)
    reports = []
    cases = [("cold", args.source_build), ("warm", args.source_build)]
    if args.variation_source_build is not None:
        cases.append(("variation", args.variation_source_build))
    for name, source_build in cases:
        run = args.output / name
        prepare_cell(args.profile, run, source_build)
        prepare_candidates(run)
        build = SectionBuild(
            run,
            args.profile,
            args.maximum_seconds,
            motion_library=args.output / "library",
            diffusion_checkpoint=args.diffusion_checkpoint,
        )
        model = IndexedMotions(build)
        model.round = 3
        multi_leg = []
        for row in build.actions.values():
            points, _ = flatten_curve(row, 0.00005, 0.003)
            if len(points) > 2:
                delta = np.diff(points, axis=0)
                unit = delta / np.linalg.norm(delta, axis=1)[:, None]
                if np.any(np.linalg.norm(np.diff(unit, axis=0), axis=1) > 1e-5):
                    multi_leg.append(row["id"])
        if not multi_leg and not args.allow_straight_diagnostic:
            raise RuntimeError("Whole-path validation needs a connected multi-leg action")
        plan_file = source_build / "offline_dual_arm_plan.json"
        source = read_json(plan_file) if plan_file.exists() else {"actions": []}
        keys = [row["stroke_id"] for row in source["actions"] if not row.get("auxiliary", False)]
        if set(keys) != set(build.actions):
            order = greedy_forward_order(
                build.actions.values(),
                build.planner.ground,
                build.stable,
                deadline=build.planner.deadline,
            )
            if order is None:
                raise RuntimeError("No supported order for the current connected action geometry")
            keys = [row["id"] for row in order]
        started = perf_counter()
        try:
            result = build.forward([build.actions[key] for key in keys], indexed_model=model)
            if result is None:
                build.report("INTEGRATION_BUILD_INCOMPLETE")
                raise RuntimeError(f"{name} build did not complete: {len(build.best)}/{len(keys)}")
            planning_seconds = perf_counter() - started
            replay_started = perf_counter()
            export_sections(build, *result)
            report = read_json(run / "section_build.json")
            if report["status"] != "FULL_BUILD_VERIFIED":
                raise RuntimeError(f"{name} replay rejected: {report['status']}")
            reports.append(
                {
                    "run": str(run),
                    "status": report["status"],
                    "seconds": perf_counter() - started,
                    "planning_seconds": planning_seconds,
                    "export_replay_seconds": perf_counter() - replay_started,
                    "sections": len(keys),
                    "motion_reuse": build.motion_runtime.report(),
                    "multi_leg_actions": multi_leg,
                    "source_build": str(source_build),
                }
            )
            if name == "warm":
                reports[-1]["backward"] = check_backward(build, model, keys[-1])
            write_json(args.output / "progress.json", {"status": "RUNNING", "runs": reports})
        finally:
            build.motion_runtime.flush()
    for report in reports[1:]:
        if report["motion_reuse"]["counts"].get("accepted_cache", 0) == 0:
            raise RuntimeError(f"{report['run']} did not accept a cached motion")
    write_json(
        args.output / "integration.json",
        {
            "status": "VERIFIED",
            "scope": "connected indexed builds and independent replay",
            "geometry_variation_verified": args.variation_source_build is not None,
            "source_build": str(args.source_build),
            "runs": reports,
        },
    )
    print(reports)


if __name__ == "__main__":
    main()
