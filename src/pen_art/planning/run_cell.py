"""Run the fixed-catalogue one-/two-arm planner on one saved, supported sculpture.

This command never changes the sculpture or starts another abstraction search.
All generated stages are exposed by the existing local workbench. A partial
result is preserved when the catalogue, allocation or motion budget runs out.
"""

import argparse
from pathlib import Path
from time import monotonic

import yaml

from ..cell import prepare_cell
from ..io import artifact, now, read_json, sha256, stage_update, write_json
from .allocation_motion import realize_allocation
from .allocation_repair import obstructing_material_modes
from .cell_allocation import allocate_cell, build_problem
from .cell_motion import build_cell_variants
from .partition import partition_run


def own_material_failure(result):
    """A fixed path hitting its own strand cannot be repaired by reordering."""
    for failure in result.get("failures", []):
        parts = failure.get("mode_id", "").split("/")
        if len(parts) >= 2 and obstructing_material_modes(
            failure.get("evidence", []), {tuple(parts[:2]): "own_material"}
        ):
            return True
    return False


def constrain_failed_prefix(run, result):
    """Carry witnessed material orders or exact prefix failures into the fixed solver."""
    if result.get("status") != "SELECTED_MOTION_REJECTED":
        return None
    if any(sha256(run / name) != digest for name, digest in result["inputs"].items()):
        return None
    if own_material_failure(result):
        return None
    problem_path = run / "allocation_problem.json"
    problem = read_json(problem_path)
    if any(sha256(run / name) != digest for name, digest in problem["inputs"].items()):
        return None
    failure = result["failures"][0]
    if any(row.get("phase") == "TRACKING" for row in failure.get("evidence", [])):
        return None
    modes = {m["id"]: m for m in problem["modes"]}
    by_stroke = {(m["arm"], m["task"]): key for key, m in modes.items()}
    observations = [(failure["mode_id"], failure["evidence"])] + [
        (row["constraint"].get("mode_id"), row["evidence"])
        for row in problem.get("prefix_failure_evidence", [])
    ]
    ordering = set()
    for moving, witness in observations:
        if moving not in modes:
            continue
        for blocker in obstructing_material_modes(witness, by_stroke):
            # Material is still present if another arm deposits that same task.
            # This is a conservative ordering restriction, not global infeasibility.
            for key, mode in modes.items():
                if mode["task"] == modes[blocker]["task"] != modes[moving]["task"]:
                    ordering.add((moving, key))
    prior_order = problem.setdefault("precedence_modes", [])
    new_order = [list(pair) for pair in sorted(ordering) if list(pair) not in prior_order]
    if new_order:
        prior_order.extend(new_order)
        evidence = {
            "kind": "MATERIAL_ORDER",
            "constraint": {"mode_id": failure["mode_id"], "precedence_modes": new_order},
            "evidence": failure["evidence"],
        }
        problem.setdefault("prefix_failure_evidence", []).append(evidence)
        write_json(problem_path, problem)
        return evidence
    restriction = {
        "mode_id": failure["mode_id"],
        "predecessor_mode": failure["predecessor_mode"],
        "completed_tasks": sorted(modes[m]["task"] for m in failure["completed_modes"]),
    }
    prior = problem.setdefault("forbidden_prefixes", [])
    if restriction in prior:
        return None
    prior.append(restriction)
    evidence = {"constraint": restriction, "evidence": failure["evidence"]}
    problem.setdefault("prefix_failure_evidence", []).append(evidence)
    write_json(problem_path, problem)
    return evidence


def resumable_allocation(run, resource_mode="dual"):
    """Reuse interrupted or fully prepared selections while their inputs match."""
    paths = [run / name for name in ("allocation_motions.json", "cell_allocation.json")]
    if not all(path.is_file() for path in paths):
        return None
    motion, allocation = map(read_json, paths)
    if motion.get("resource_mode", "dual") != resource_mode:
        return None
    interrupted = motion.get("status") in {
        "PLANNING_TIME_BUDGET",
        "PLANNING_INTERRUPTED",
        "RETRY_REQUESTED",
    }
    repaired_path_retry = motion.get(
        "status"
    ) == "SELECTED_MOTION_REJECTED" and own_material_failure(motion)
    prepared = motion.get("prepared_strokes") == motion.get("requested_strokes") and motion.get(
        "status"
    ) in {
        "SELECTED_MOTIONS_PREPARED",
        "SCHEDULE_REPLAY_REJECTED",
        "REALIZATION_ERROR",
        "SELECTED_RETURN_REJECTED",
    }
    if not (interrupted or prepared or repaired_path_retry):
        return None
    for document in (motion, allocation):
        if any(
            not (run / name).is_file() or sha256(run / name) != digest
            for name, digest in document["inputs"].items()
        ):
            return None
    if len(allocation["results"][resource_mode]["actions"]) != allocation["requested_strokes"]:
        return None
    return allocation


def record_interruption(output, reason="Operator interrupted offline planning"):
    """Retain an exact partial selection for a deliberate, resumable restart.

    Call only after the planner has stopped writing its checkpoint files.
    Completed motions remain bound to their original allocation/input hashes.
    """
    output = Path(output)
    motion_path = output / "allocation_motions.json"
    if motion_path.is_file():
        motion = read_json(motion_path)
        if motion.get("status") in {"PREPARING_SELECTED_MOTIONS", "SELECTED_MOTIONS_PREPARED"}:
            motion.update(status="PLANNING_INTERRUPTED", interruption_reason=reason)
            write_json(motion_path, motion)
    replay_path = None
    run_path = output / "run.json"
    if run_path.is_file():
        for name, stage in list(read_json(run_path)["stages"].items()):
            if name not in {"motion", "validation"} or stage.get("status") != "running":
                continue
            if name == "validation" and stage.get("artifact"):
                replay_path = output / stage["artifact"]
                if replay_path.is_file():
                    replay = read_json(replay_path)
                    if replay.get("status") == "RUNNING":
                        replay.update(
                            status="REPLAY_INTERRUPTED",
                            failures=[{"status": "INTERRUPTED", "message": reason}],
                        )
                        write_json(replay_path, replay)
            stage_update(output, name, "partial", artifact=stage.get("artifact"), message=reason)
    path = output / "cell_pipeline_result.json"
    if path.is_file():
        report = read_json(path)
        report.update(status="PLANNING_INTERRUPTED", detail=reason, updated_at=now())
        if motion_path.is_file() and "inputs" in report:
            report["inputs"][motion_path.name] = sha256(motion_path)
        if replay_path is not None and replay_path.is_file() and "inputs" in report:
            report["inputs"][replay_path.name] = sha256(replay_path)
        write_json(path, report)


def validate_restart_options(single_arm_restarts, resource_mode, no_time_limit, profile):
    """Reject incompatible opt-in settings before creating or modifying a run."""
    if not single_arm_restarts:
        return
    if resource_mode != "single":
        raise ValueError("--single-arm-restarts requires --resource-mode single")
    if not no_time_limit:
        raise ValueError("--single-arm-restarts requires --no-time-limit")
    settings = yaml.safe_load(Path(profile).read_text())
    if len(settings.get("arms", [])) != 1:
        raise ValueError("--single-arm-restarts requires a cell profile containing exactly one arm")


def plan_cell(
    source,
    output,
    profile,
    *,
    maximum_seconds=600,
    catalogue_seconds=300,
    maximum_allocation_repairs=2,
    timing_path=None,
    resource_mode="dual",
    no_time_limit=False,
    single_arm_restarts=False,
):
    """Build each fixed drawing mode once, allocate, then realize and replay."""
    if maximum_seconds <= 0 or catalogue_seconds <= 0 or maximum_allocation_repairs < 0:
        raise ValueError("Planning budgets must be positive and repairs nonnegative")
    if resource_mode not in {"single", "dual"}:
        raise ValueError("Resource mode must be single or dual")
    validate_restart_options(single_arm_restarts, resource_mode, no_time_limit, profile)
    if single_arm_restarts:
        timing_path = (
            Path(timing_path)
            if timing_path is not None
            else (Path(__file__).resolve().parents[3] / "configs/timing_continuous_bounded.yaml")
        )
        timing_settings = yaml.safe_load(timing_path.read_text())
        if timing_settings.get("drawing_clock") != "cubic_toppra":
            raise ValueError("--single-arm-restarts requires drawing_clock: cubic_toppra")
    source, output, profile = Path(source), Path(output), Path(profile)
    saved_motion = output / "allocation_motions.json"
    if saved_motion.is_file():
        saved = read_json(saved_motion)
        if saved.get("resource_mode", "dual") != resource_mode:
            raise ValueError("Resource mode changed; use a separate output run")
    started = monotonic()
    deadline = float("inf") if no_time_limit else started + maximum_seconds
    prepare_cell(profile, output, source, source_variant="backbone")
    run_state = read_json(output / "run.json")
    run_state["resource_mode"] = resource_mode
    write_json(output / "run.json", run_state)
    if len(read_json(output / "cell_manifest.json")["arms"]) == 2:
        partition_run(output)
    report = artifact(
        "cell_pipeline_result",
        status="RUNNING",
        simulation_only=True,
        resource_mode=resource_mode,
        executable=False,
        started_at=now(),
        source_run=str(source.resolve()),
        maximum_seconds=None if no_time_limit else maximum_seconds,
        catalogue_seconds=None if no_time_limit else catalogue_seconds,
        no_time_limit=no_time_limit,
        motion_method="single_arm_restarts" if single_arm_restarts else "fixed_mode_realization",
        allocation_repairs=[],
        limitations=[
            "Geometry and pen calibration remain separate from simulated planning",
            "One fixed drawing mode per eligible arm/action; no catalogue expansion",
            "Hardware execution and physical validation are not performed",
        ],
    )

    def finish(status, **values):
        names = (
            "cell_manifest.json",
            "cell_drawing_variants.json",
            "cell_allocation.json",
            "allocation_motions.json",
            "offline_dual_arm_plan.json",
            "offline_dual_arm_validation.json",
            "joint_path.json",
            "trajectory.json",
            "trajectory_manifest.json",
            "full_build/state.json",
            "single_arm_build_manifest.json",
        )
        report.update(
            status=status,
            updated_at=now(),
            elapsed_s=monotonic() - started,
            inputs={name: sha256(output / name) for name in names if (output / name).is_file()},
            **values,
        )
        write_json(output / "cell_pipeline_result.json", report)
        if status not in {
            "GENERATING_FIXED_CATALOGUE",
            "ALLOCATING_FIXED_MODES",
            "REALIZING_SELECTED_MOTIONS",
            "REALIZING_SINGLE_ARM_RESTARTS",
        }:
            from ..validation import validation_report

            validation_report(output)
            report["inputs"]["validation_report.json"] = sha256(output / "validation_report.json")
            write_json(output / "cell_pipeline_result.json", report)
        return report

    finish("GENERATING_FIXED_CATALOGUE")
    if monotonic() >= deadline:
        return finish("PLANNING_TIME_BUDGET")
    variants = build_cell_variants(
        output,
        profile,
        maximum_seconds=(
            float("inf") if no_time_limit else min(catalogue_seconds, deadline - monotonic())
        ),
        per_arm_stroke=1,
        resume=(output / "cell_drawing_variants.json").is_file(),
    )
    construction = read_json(output / "construction_plan.json")
    represented = {row["stroke_id"] for row in variants["variants"]}
    missing = [a["id"] for a in construction["actions"] if a["id"] not in represented]
    if variants["status"] != "BOUNDED_CANDIDATES":
        return finish("CATALOGUE_TIME_BUDGET", missing_strokes=missing)
    if missing:
        return finish("NO_DRAWING_MODE_FOUND", missing_strokes=missing)
    saved_motion = output / "allocation_motions.json"
    if not single_arm_restarts and maximum_allocation_repairs and saved_motion.is_file():
        evidence = constrain_failed_prefix(output, read_json(saved_motion))
        if evidence:
            report["allocation_repairs"].append(evidence)
    for repair_index in range(maximum_allocation_repairs + 1):
        remaining = deadline - monotonic()
        if remaining <= 0:
            return finish("PLANNING_TIME_BUDGET")
        finish("ALLOCATING_FIXED_MODES")
        try:
            # Refresh the configuration/input binding before accepting a cache.
            # Solving the same problem again can change its incumbent and discard
            # the validated incoming states of all already prepared motions.
            build_problem(output, profile, deadline=deadline, timing_path=timing_path)
            if single_arm_restarts and (output / "full_build/state.json").is_file():
                allocation = read_json(output / "cell_allocation.json")
                if any(
                    not (output / name).is_file() or sha256(output / name) != digest
                    for name, digest in allocation["inputs"].items()
                ):
                    raise ValueError("Restart allocation inputs changed; use a new output run")
            else:
                allocation = (
                    resumable_allocation(output, resource_mode) if repair_index == 0 else None
                )
            if allocation is not None:
                report["resumed_selected_allocation"] = True
            else:
                allocation = allocate_cell(
                    output,
                    profile,
                    maximum_seconds=min(15, max(0.001, (deadline - monotonic()) / 2)),
                    maximum_total_seconds=max(0.001, deadline - monotonic()),
                    timing_path=timing_path,
                )
        except TimeoutError as error:
            return finish("PLANNING_TIME_BUDGET", detail=str(error))
        selected = allocation["results"][resource_mode]
        if len(selected["actions"]) != len(construction["actions"]):
            return finish("NO_FULL_ALLOCATION_FOUND", solver_status=selected["status"])
        remaining = deadline - monotonic()
        if remaining <= 0:
            return finish("PLANNING_TIME_BUDGET")
        if single_arm_restarts:
            from .full_build_export import export_full_build
            from .single_arm_build import SingleArmBuild

            finish("REALIZING_SINGLE_ARM_RESTARTS")
            builder = SingleArmBuild(output, profile)
            builder.run_remaining()
            if builder.state["status"] == "ALL_STROKES_PREPARED_REPLAY_PENDING":
                export_full_build(builder)
            return finish(
                builder.state["status"],
                prepared_strokes=len(builder.state["completed_strokes"]),
                requested_strokes=len(builder.selection),
                restart_checkpoint="full_build/state.json",
            )
        finish("REALIZING_SELECTED_MOTIONS")
        result = realize_allocation(
            output, profile, maximum_seconds=remaining, resource_mode=resource_mode
        )
        if result["status"] == "SELECTED_MOTION_REJECTED" and own_material_failure(result):
            return finish(
                "DRAWING_PATH_REQUIRES_REPAIR",
                prepared_strokes=result["prepared_strokes"],
                detail="The fixed drawing path contacts its own deposited material; "
                "reordering cannot repair this geometry/orientation conflict",
            )
        if (
            result["status"] != "SELECTED_MOTION_REJECTED"
            or repair_index == maximum_allocation_repairs
        ):
            return finish(result["status"], prepared_strokes=result["prepared_strokes"])
        if any(
            row.get("phase") == "TRACKING"
            for failure in result.get("failures", [])
            for row in failure.get("evidence", [])
        ):
            return finish("DRAWING_PATH_REJECTED", prepared_strokes=result["prepared_strokes"])
        # Preserve evidence before the next solve replaces its selected schedule.
        evidence = constrain_failed_prefix(output, result)
        if evidence is None:
            return finish("REPEATED_PREFIX_FAILURE", prepared_strokes=result["prepared_strokes"])
        report["allocation_repairs"].append(
            {**evidence, "prior_estimated_seconds": selected["makespan_s"]}
        )
    raise AssertionError("Allocation loop must return a recorded result")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=Path("configs/cell/two_flexiv.yaml"))
    parser.add_argument("--maximum-seconds", type=float, default=600)
    parser.add_argument("--catalogue-seconds", type=float, default=300)
    parser.add_argument(
        "--no-time-limit",
        action="store_true",
        help="Remove overall and catalogue timeouts; retain the fixed catalogue and repair count",
    )
    parser.add_argument("--maximum-allocation-repairs", type=int, default=2)
    parser.add_argument("--timing-config", type=Path)
    parser.add_argument("--resource-mode", choices=["single", "dual"], default="dual")
    parser.add_argument(
        "--single-arm-restarts",
        action="store_true",
        help="Plan from empty base with supported same-point pen-off restarts; "
        "requires --resource-mode single, --no-time-limit, and a one-arm profile",
    )
    args = parser.parse_args()
    try:
        result = plan_cell(
            args.source_run,
            args.output,
            args.config,
            maximum_seconds=args.maximum_seconds,
            catalogue_seconds=args.catalogue_seconds,
            maximum_allocation_repairs=args.maximum_allocation_repairs,
            timing_path=args.timing_config,
            resource_mode=args.resource_mode,
            no_time_limit=args.no_time_limit,
            single_arm_restarts=args.single_arm_restarts,
        )
    except KeyboardInterrupt:
        record_interruption(args.output)
        print("PLANNING_INTERRUPTED (saved checkpoint)", flush=True)
        return
    except Exception as error:
        path = args.output / "cell_pipeline_result.json"
        if path.is_file():
            report = read_json(path)
            if report.get("resource_mode", "dual") == args.resource_mode:
                report.update(status="PIPELINE_ERROR", error=str(error), updated_at=now())
                write_json(path, report)
        raise
    print(f"{result['status']} ({result['elapsed_s']:.1f} s)", flush=True)


if __name__ == "__main__":
    main()
