"""Construct only the motions selected by one offline allocation.

No alternate drawings are generated here. On failure, preserve the selected
transition and its material context for a later scheduling constraint. A failure
is not evidence that the entire stroke is impossible.
"""

import argparse
import copy
from pathlib import Path
from time import monotonic

import numpy as np
import yaml

from ..io import artifact, now, read_json, sha256, stage_update, write_json
from .allocation_repair import material_precedence, park_idle_arm
from .cell_planner import CellPlanner
from .cell_replay import replay_cell
from .clock_path import ClockPath, timed_path
from .dual_arm import MotionProposal, ScheduleState
from .resource_schedule import solve_allocation
from .scene import MaterialScene


def _realize_allocation(
    run, profile_path, maximum_seconds=180, maximum_refinements=6, *, resource_mode="dual"
):
    if maximum_seconds <= 0 or maximum_refinements < 0:
        raise ValueError("Positive time budget and nonnegative refinement count required")
    if resource_mode not in {"single", "dual"}:
        raise ValueError("Resource mode must be single or dual")
    deadline = monotonic() + maximum_seconds
    run = Path(run)
    previous_path = run / "allocation_motions.json"
    previous = read_json(previous_path) if previous_path.exists() else {}
    if previous and previous.get("resource_mode", "dual") != resource_mode:
        raise ValueError("Resource mode changed; use a separate output run")
    allocation = read_json(run / "cell_allocation.json")
    problem = read_json(run / "allocation_problem.json")
    for doc in (allocation, problem):
        for name, digest in doc["inputs"].items():
            if sha256(run / name) != digest:
                raise ValueError(f"Stale allocation input: {name}")
    chosen = allocation["results"][resource_mode]
    if len(chosen["actions"]) != len(problem["tasks"]):
        raise ValueError("A full offline allocation is required")
    if any(
        first["start_s"] > second["start_s"]
        for first, second in zip(chosen["actions"][:-1], chosen["actions"][1:], strict=True)
    ):
        raise ValueError("Selected actions must be ordered by start time")
    settings = yaml.safe_load(Path("configs/dual_arm.yaml").read_text())
    config = read_json(run / "allocation_config.json")
    settings["assumed_cooling_s"] = config["cooling_s"]
    if len(config["initial_positions"]) == 1:
        # Tight pen clearances can require more than twelve bisections to
        # certify. Keep the same margin and allow a deeper enclosure proof.
        settings["maximum_subdivision_depth"] = max(20, settings["maximum_subdivision_depth"])
    planner = CellPlanner(run, profile_path, settings, deadline, timing_override=config["timing"])
    if planner.process != config["process"] or planner.timing != config["timing"]:
        raise ValueError("Process or clock assumptions changed")
    initial = config["initial_positions"]
    for identity, arm in planner.arms.items():
        arm.initial_positions = np.asarray(initial[identity])
    rows = {(r["arm"], r["stroke_id"], r["variant"]): r for r in planner.candidates["variants"]}
    modes = {m["id"]: m for m in problem["modes"]}
    prepared = {}
    completed_material = set()
    scene = MaterialScene()
    q = {identity: tuple(value) for identity, value in initial.items()}
    report = artifact(
        "allocation_motion",
        status="PREPARING_SELECTED_MOTIONS",
        executable=False,
        simulation_only=True,
        resource_mode=resource_mode,
        inputs={
            name: sha256(run / name) for name in ("cell_allocation.json", "allocation_problem.json")
        },
        requested_strokes=len(chosen["actions"]),
        prepared_strokes=0,
        motions=[],
        failures=[],
    )
    cached = (
        {row["mode_id"]: row for row in previous.get("motions", [])}
        if previous.get("inputs") == report["inputs"]
        else {}
    )
    # Retain the entire matching checkpoint during restoration. An interrupt
    # while decoding its first path must not discard the later saved motions.
    report["motions"] = list(cached.values())
    report["prepared_strokes"] = len(cached)
    report["refinements"] = previous.get("refinements", []) if cached else []
    report["replay_attempts"] = previous.get("replay_attempts", []) if cached else []
    write_json(run / "allocation_motions.json", report)
    stage_update(
        run,
        "motion",
        "running",
        artifact="allocation_motions.json",
        message="Preparing selected motions; shared replay pending",
    )
    for selected in chosen["actions"]:
        key, identity = selected["mode_id"], selected["arm"]
        if key in cached:
            path = ClockPath.from_record(cached[key]["trajectory"])
            if not np.allclose(path.edges[0].q0, q[identity], atol=1e-8, rtol=0):
                raise ValueError("Cached selected path has a different incoming configuration")
            attempt = MotionProposal(
                selected["stroke_id"],
                identity,
                selected["variant"],
                path.duration_s,
                selected["start_anchor"],
                tuple(path.edges[-1].q1),
                path,
            )
        else:
            report["active_motion"] = {
                "mode_id": key,
                "arm": identity,
                "stroke_id": selected["stroke_id"],
                "started_at": now(),
            }
            write_json(run / "allocation_motions.json", report)
            # The cache is bound to this exact allocation and input hashes.
            # A restored motion does not need its unchanged material prefix
            # reconstructed; only new motion preparation consumes that scene.
            # Start times are nondecreasing, so this completed-material prefix
            # only grows. Keep each immutable drawing enclosure once. Do not
            # insert a prepared motion until its scheduled end has passed.
            for earlier in chosen["actions"]:
                earlier_key = earlier["mode_id"]
                if (
                    earlier["end_s"] > selected["start_s"]
                    or earlier_key not in prepared
                    or earlier_key in completed_material
                ):
                    continue
                move = prepared[earlier_key]
                for index, edge in enumerate(move.payload.edges):
                    if edge.kind == "DRAW":
                        planner.add_clock_edge(
                            scene, move.arm, edge, f"{move.arm}/{move.task}/{index}"
                        )
                completed_material.add(earlier_key)
            state = ScheduleState(
                selected["start_s"], tuple(q.items()), frozenset(), (), payload=scene
            )
            attempt = planner.prepare_drawing(
                state,
                identity,
                rows[(identity, selected["stroke_id"], selected["variant"])],
                np.asarray(q[identity]),
            )
        if attempt is None:
            timed_out = monotonic() >= deadline
            report.update(
                status="PLANNING_TIME_BUDGET" if timed_out else "SELECTED_MOTION_REJECTED",
                failures=[
                    {
                        "mode_id": key,
                        "predecessor_mode": selected["predecessor_mode"],
                        "completed_modes": [
                            a["mode_id"]
                            for a in chosen["actions"]
                            if a["end_s"] <= selected["start_s"]
                        ],
                        "evidence": planner.attempts[-3:],
                        "scope": "Selected transition in this partial structure; "
                        "not global infeasibility",
                    }
                ],
            )
            write_json(run / "allocation_motions.json", report)
            stage_update(
                run,
                "motion",
                "partial",
                artifact="allocation_motions.json",
                message=("Planning budget ended" if timed_out else "Selected motion rejected")
                + f" after {len(prepared)} prepared strokes",
            )
            return report
        prepared[key] = attempt
        q[identity] = attempt.end_configuration
        if key in cached:
            continue
        report["motions"].append(
            {
                "mode_id": key,
                "arm": identity,
                "stroke_id": attempt.task,
                "trajectory": attempt.payload.record(),
            }
        )
        report["prepared_strokes"] = len(report["motions"])
        report.pop("active_motion", None)
        write_json(run / "allocation_motions.json", report)

    # Preserve this selected assignment/order while replacing setup estimates by
    # actual selected-path clocks. Re-optimization of ownership is a later solve.
    exact = copy.deepcopy(problem)
    exact["forbidden_modes"] = list(set(modes) - prepared.keys())
    allowed = {(a["predecessor_mode"], a["mode_id"]) for a in chosen["actions"]}
    exact["forbidden_transitions"] = [
        [r["from"], r["to"]] for r in exact["transitions"] if (r["from"], r["to"]) not in allowed
    ] + [[None, key] for key in prepared if (None, key) not in allowed]

    def update_costs():
        for selected in chosen["actions"]:
            key, predecessor = selected["mode_id"], selected["predecessor_mode"]
            cost = max(0, prepared[key].duration_s - modes[key]["duration_s"])
            if predecessor is None:
                exact["initial_setup_s"][key] = cost
            else:
                next(
                    r for r in exact["transitions"] if r["from"] == predecessor and r["to"] == key
                )["duration_s"] = cost

    update_costs()

    final_scene = MaterialScene()
    for move in prepared.values():
        for index, edge in enumerate(move.payload.edges):
            if edge.kind == "DRAW":
                planner.add_clock_edge(
                    final_scene,
                    move.arm,
                    edge,
                    f"{move.arm}/{move.task}/{index}",
                )
    returns = {}
    for identity, arm in planner.arms.items():
        last = next((a for a in reversed(chosen["actions"]) if a["arm"] == identity), None)
        if last is None:
            continue
        result = planner.travel(
            identity, final_scene, np.asarray(q[identity]), arm.initial_positions
        )
        if result["status"] != "CONNECTED":
            report.update(
                status="SELECTED_RETURN_REJECTED",
                failures=[
                    {
                        "arm": identity,
                        "evidence": result,
                        "scope": "Return through final structure",
                    }
                ],
            )
            write_json(run / "allocation_motions.json", report)
            return report
        if np.allclose(result["positions"], arm.initial_positions, atol=1e-10, rtol=0):
            exact["final_setup_s"][last["mode_id"]] = 0
            continue
        returns[identity] = timed_path(
            [{"kind": "RETURN", "positions": result["positions"]}],
            arm,
            arm.robot.velocity * planner.timing["velocity_fraction"],
            np.asarray(planner.timing["joint_acceleration_rad_s2"]),
            planner.process["nominal_speed_m_s"],
            planner.timing["travel_speed_m_s"],
        )
        exact["final_setup_s"][last["mode_id"]] = returns[identity].duration_s
    exact["exclusive_modes"] = [
        r["modes"] for r in report["refinements"] if r["kind"] == "NO_OVERLAP"
    ]
    exact.setdefault("precedence_modes", [])
    for repair in report["refinements"]:
        if repair["kind"] == "MATERIAL_ORDER":
            for pair in repair["pairs"]:
                if pair not in exact["precedence_modes"]:
                    exact["precedence_modes"].append(pair)
    path_certificates = set()
    for refinement in range(maximum_refinements + 1):
        remaining = deadline - monotonic()
        if remaining <= 0:
            report["status"] = "PLANNING_TIME_BUDGET"
            break
        schedule = solve_allocation(exact, maximum_seconds=min(5, remaining))
        if not schedule["actions"]:
            report.update(status="RETIMING_REJECTED", timing_result=schedule)
            write_json(run / "allocation_motions.json", report)
            return report
        actions = []
        for selected in schedule["actions"]:
            move = prepared[selected["mode_id"]]
            actions.append(
                {
                    "arm": move.arm,
                    "stroke_id": move.task,
                    "variant": move.variant,
                    "start_anchor": move.start_anchor,
                    "start_s": selected["start_s"],
                    "end_s": selected["start_s"] + move.duration_s,
                    "trajectory": move.payload.record(),
                    "auxiliary": False,
                }
            )
        for identity, path in returns.items():
            start = max(a["end_s"] for a in actions if a["arm"] == identity)
            actions.append(
                {
                    "arm": identity,
                    "stroke_id": f"return-{identity}",
                    "variant": "return",
                    "start_anchor": "parking",
                    "start_s": start,
                    "end_s": start + path.duration_s,
                    "trajectory": path.record(),
                    "auxiliary": True,
                }
            )
        config_name = "offline_dual_arm_config.json"
        write_json(
            run / config_name,
            {
                "search": settings,
                "transit": planner.transit,
                "timing": planner.timing,
                "process": planner.process,
            },
        )
        output = artifact(
            "dual_arm_plan",
            status="FULL_SCHEDULE_GEOMETRY_UNVERIFIED",
            full_schedule=True,
            executable=False,
            simulation_only=True,
            resource_mode=resource_mode,
            active_arms=sorted({a["arm"] for a in actions if not a["auxiliary"]}),
            frame="world",
            units="m",
            inputs={
                name: sha256(run / name)
                for name in (
                    "cell_manifest.json",
                    "cell_drawing_variants.json",
                    "construction_plan.json",
                    "stroke_graph.json",
                    "cell_allocation.json",
                    config_name,
                )
            },
            actions=sorted(actions, key=lambda a: (a["start_s"], a["arm"])),
            events=sorted(
                [
                    {
                        "time_s": a["start_s"] + e["start_s"],
                        "arm": a["arm"],
                        "stroke_id": a["stroke_id"],
                        "command": "FEED_ON" if e["kind"] == "PEN_START_WAIT" else "FEED_OFF",
                    }
                    for a in actions
                    for e in a["trajectory"]["edges"]
                    if e["kind"] in ("PEN_START_WAIT", "PEN_STOP_WAIT")
                ],
                key=lambda e: (e["time_s"], e["arm"]),
            ),
            modeled_duration_s=max(schedule["makespan_s"], max(a["end_s"] for a in actions)),
            completed_strokes=0,
            requested_strokes=len(prepared),
            checked_intervals=0,
            settings_artifact=config_name,
            enabled_arms=sorted({a["arm"] for a in actions if not a["auxiliary"]}),
            initial_positions=initial,
            initial_pose_repairs={
                "precondition": (
                    "Start the robot at its saved staged pose"
                    if len(planner.arms) == 1
                    else "Start both robots at the saved staged poses"
                )
            },
            limitations=problem["limitations"],
            failures=[],
        )
        for move in prepared.values():
            repair = move.payload.metadata.get("axis_repair_artifact")
            if repair:
                digest = sha256(run / repair)
                if digest != move.payload.metadata["axis_repair_sha256"]:
                    raise ValueError("Pen-axis repair evidence changed after motion preparation")
                output["inputs"][repair] = digest
        write_json(run / "offline_dual_arm_plan.json", output)
        stage_update(
            run,
            "timing",
            "partial",
            artifact="offline_dual_arm_plan.json",
            message="Complete shared clock and pen events generated; "
            "replay and calibration separate",
        )
        report.update(status="SELECTED_MOTIONS_PREPARED", retimed_schedule=schedule)
        write_json(run / "allocation_motions.json", report)
        remaining = deadline - monotonic()
        if remaining <= 0:
            report["status"] = "PLANNING_TIME_BUDGET"
            break
        audit = replay_cell(
            run,
            profile_path,
            remaining,
            "offline_dual_arm_plan",
            path_certificates=path_certificates,
        )

        report["replay_attempts"].append(
            {
                "makespan_s": output["modeled_duration_s"],
                "status": audit["status"],
                "failures": audit.get("failures", []),
            }
        )
        report["replay_status"] = audit["status"]
        if audit["status"] == "CERTIFIED_GEOMETRIC_REPLAY":
            report["status"] = "SELECTED_SCHEDULE_REPLAYED"
            break
        if audit["status"] == "REPLAY_TIME_BUDGET":
            report["status"] = "PLANNING_TIME_BUDGET"
            break
        report["status"] = "SCHEDULE_REPLAY_REJECTED"
        failures = audit.get("failures", [])
        if refinement == maximum_refinements or not failures:
            break
        failure = failures[0]
        precedence = [
            pair
            for pair in material_precedence(output, failure, prepared)
            if pair not in exact["precedence_modes"]
        ]
        if precedence:
            exact["precedence_modes"].extend(precedence)
            report["refinements"].append(
                {"kind": "MATERIAL_ORDER", "pairs": precedence, "witness": failure}
            )
            write_json(run / "allocation_motions.json", report)
            print("Added material ordering:", precedence, flush=True)
            continue
        if failure.get("phase") != "INTER_ARM" or failure.get("status") not in (
            "COLLISION",
            "CLEARANCE_UNRESOLVED",
        ):
            break
        time = failure["time_s"]
        active = [
            a for a in output["actions"] if not a["auxiliary"] and a["start_s"] <= time < a["end_s"]
        ]
        if len(active) == 1:
            repair = park_idle_arm(planner, output, failure, prepared)
            report["parking_result"] = repair
            if repair["status"] != "PARK_ADDED":
                break
            report["refinements"].append({"kind": "PARK_IDLE_ARM", **repair})
            if repair["next_mode"] is None:
                returns.pop(repair["arm"], None)
                exact["final_setup_s"][repair["after_mode"]] = 0
            update_costs()
            for row in report["motions"]:
                row["trajectory"] = prepared[row["mode_id"]].payload.record()
            write_json(run / "allocation_motions.json", report)
            print("Added idle-arm parking:", repair["arm"], flush=True)
            continue
        if len(active) != 2:
            break
        by_task = {(p.arm, p.task): key for key, p in prepared.items()}
        pair = sorted(by_task[a["arm"], a["stroke_id"]] for a in active)
        if pair in exact["exclusive_modes"]:
            break
        exact["exclusive_modes"].append(pair)
        report["refinements"].append({"kind": "NO_OVERLAP", "modes": pair, "witness": failure})
        print("Added resource exclusion:", [a["stroke_id"] for a in active], flush=True)
    write_json(run / "allocation_motions.json", report)
    if report.get("retimed_schedule") is not None:
        from ..trajectory import export_trajectory

        export_trajectory(run)
    stage_update(
        run,
        "motion",
        "complete" if len(prepared) == len(chosen["actions"]) else "partial",
        artifact="allocation_motions.json",
        message=f"Offline allocation: {report['status']}",
    )
    return report


def realize_allocation(
    run, profile_path, maximum_seconds=180, maximum_refinements=6, *, resource_mode="dual"
):
    try:
        return _realize_allocation(
            run, profile_path, maximum_seconds, maximum_refinements, resource_mode=resource_mode
        )
    except Exception as error:
        path = Path(run) / "allocation_motions.json"
        if path.exists():
            report = read_json(path)
            if report.get("resource_mode", "dual") == resource_mode:
                report.update(status="REALIZATION_ERROR", error=str(error))
                write_json(path, report)
        raise


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, default=Path("runs/palace_15cm_dual"))
    parser.add_argument("--config", type=Path, default=Path("configs/cell/two_flexiv.yaml"))
    parser.add_argument("--maximum-seconds", type=float, default=180)
    parser.add_argument("--maximum-refinements", type=int, default=6)
    parser.add_argument("--resource-mode", choices=["single", "dual"], default="dual")
    args = parser.parse_args()
    result = realize_allocation(
        args.run,
        args.config,
        args.maximum_seconds,
        args.maximum_refinements,
        resource_mode=args.resource_mode,
    )
    print(result["status"], result["prepared_strokes"], flush=True)
