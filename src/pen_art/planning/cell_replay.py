"""Replay a saved common clock, resuming only an identical verified prefix."""

import argparse
import hashlib
import json
from dataclasses import replace
from pathlib import Path
from time import monotonic

import numpy as np

from ..io import artifact, read_json, sha256, stage_update, write_json
from ..timing import polynomial_range
from .cell_planner import CellPlanner
from .clock_path import ClockPath, polynomial_tcp_speed_bound, tcp_tangent_bound
from .dual_arm import MotionProposal, ScheduledMotion, ScheduleState
from .replay_checkpoint import (
    IMPLEMENTATION,
    read_checkpoint,
    restore_state,
    save_checkpoint,
)
from .replay_checkpoint import (
    digest as context_digest,
)
from .scene import MaterialScene


def _replay_cell(
    run,
    profile_path,
    maximum_seconds=300,
    plan_name="dual_arm_plan",
    return_state=False,
    *,
    path_certificates=None,
):
    validation_name = plan_name.removesuffix("_plan") + "_validation.json"
    plan_file = plan_name + ".json"
    run = Path(run)
    raw = (run / plan_file).read_bytes()
    plan_digest = hashlib.sha256(raw).hexdigest()
    saved = json.loads(raw)
    for name, digest in saved["inputs"].items():
        if sha256(run / name) != digest:
            raise ValueError(f"Stale plan input: {name}")
    settings = read_json(run / saved.get("settings_artifact", "dual_arm_config.json"))
    planner = CellPlanner(
        run,
        profile_path,
        settings["search"],
        monotonic() + maximum_seconds,
        timing_override=settings["timing"],
    )
    planner.transit, planner.timing, planner.process = (
        settings[k] for k in ("transit", "timing", "process")
    )
    # Path certificates can survive a scheduling repair. Material checkpoints
    # additionally bind the entire plan: changed timing must replay from zero.
    certificate_context = context_digest(
        {
            "inputs": saved["inputs"],
            "settings": settings,
            "profile": planner.profile,
            "implementation": IMPLEMENTATION,
            "robots": {
                identity: {
                    "urdf": sha256(arm.urdf),
                    "collision_meshes": {
                        mesh.get("filename"): sha256(Path(mesh.get("filename")))
                        for mesh in arm.robot.xml.findall("link/collision/geometry/mesh")
                    },
                    "tool": arm.tool.metadata,
                }
                for identity, arm in planner.arms.items()
            },
        }
    )
    checkpoint_context = context_digest([certificate_context, plan_digest])
    checkpoint_path = run / (plan_name.removesuffix("_plan") + "_replay_checkpoint.json")
    checkpoint = read_checkpoint(checkpoint_path, checkpoint_context)
    initial = {identity: tuple(q) for identity, q in saved["initial_positions"].items()}
    if set(initial) != set(planner.arms):
        raise ValueError("Saved initial poses do not match the cell")
    tasks = {task.identity: task for task in planner.tasks}
    horizon = saved["modeled_duration_s"]
    if horizon < 0 or not np.isfinite(horizon):
        raise ValueError("Invalid replay horizon")
    state = ScheduleState(
        0,
        tuple(initial.items()),
        frozenset(),
        tuple((n, 0) for n in planner.ground),
        payload=MaterialScene(),
    )
    output = artifact(
        "dual_arm_validation",
        status="RUNNING",
        phase="PATH_CHECKS",
        simulation_only=True,
        executable=False,
        source_plan_sha256=plan_digest,
        inputs={plan_file: plan_digest},
        horizon_s=horizon,
        requested_strokes=len(tasks),
        completed_strokes=0,
        checked_intervals=0,
        checked_actions=0,
        reused_path_checks=0,
        requested_actions=len(saved["actions"]),
        checked_edges=0,
        requested_edges=sum(len(row["trajectory"]["edges"]) for row in saved["actions"]),
        failures=[],
        scope=[
            "Saved joint paths and clocks, joint limits, velocity and acceleration",
            "Both arms, static obstacles and growing material on the shared clock; "
            "an identical verified prefix may resume from its exact material checkpoint",
            "Start-anchor readiness and final return for a complete plan",
        ],
        limitations=[
            *saved["limitations"],
            "Cartesian reference tracking uses the planner's preparation audit; "
            "it is not recomputed here",
        ],
    )
    write_json(run / validation_name, output)
    last_write = monotonic()

    def budget_result():
        output.update(
            status="REPLAY_TIME_BUDGET",
            failures=[{"status": "TIME_BUDGET", "phase": output["phase"]}],
        )
        write_json(run / validation_name, output)
        return (output, state) if return_state else output

    moves = []
    previous = {identity: (0.0, q) for identity, q in initial.items()}
    for row in sorted(saved["actions"], key=lambda row: row["start_s"]):
        if monotonic() >= planner.deadline:
            return budget_result()
        output["active_action"] = {"arm": row["arm"], "stroke_id": row["stroke_id"]}
        path = ClockPath.from_record(row["trajectory"])
        arm = planner.arms[row["arm"]]
        start, end = row["start_s"], row["end_s"]
        if (
            not np.isfinite([start, end]).all()
            or start < 0
            or abs(end - start - path.duration_s) > 1e-8
        ):
            raise ValueError("Saved action times do not match its trajectory")
        previous_end, previous_q = previous[row["arm"]]
        if start < previous_end - 1e-9 or not np.allclose(
            previous_q, path.edges[0].q0, atol=1e-8, rtol=0
        ):
            raise ValueError("Overlapping or discontinuous arm channel")
        key = (
            certificate_context,
            row["arm"],
            hashlib.sha256(json.dumps(row["trajectory"], sort_keys=True).encode()).hexdigest(),
        )
        reuse = checkpoint is not None or (
            path_certificates is not None and key in path_certificates
        )
        if reuse:
            output["reused_path_checks"] += 1
            output["checked_edges"] += len(path.edges)
        for edge_index, edge in () if reuse else enumerate(path.edges):
            if monotonic() >= planner.deadline:
                return budget_result()
            arm.robot.frames(edge.q0)
            arm.robot.frames(edge.q1)
            duration = edge.end_s - edge.start_s
            dq = abs(edge.q1 - edge.q0)
            limit = (
                planner.process["nominal_speed_m_s"]
                if edge.kind == "DRAW"
                else planner.timing["travel_speed_m_s"]
            )
            if edge.coefficients is not None:
                piece = edge.piece()
                ranges = np.array([polynomial_range(p, 0, 1) for p in piece.polynomials])
                if np.any(ranges[:, 0] < arm.robot.lower - 1e-9) or np.any(
                    ranges[:, 1] > arm.robot.upper + 1e-9
                ):
                    raise ValueError("Saved polynomial exceeds joint position limits")
                first, second = piece.derivatives(0, 1)
                velocity, acceleration = first / duration, second / duration**2
                try:
                    speed_bound = polynomial_tcp_speed_bound(
                        edge, arm, limit, deadline=planner.deadline
                    )
                except TimeoutError:
                    return budget_result()
            else:
                velocity = 1.875 * dq / duration
                acceleration = (10 / np.sqrt(3)) * dq / duration**2
                speed_bound = 1.875 * tcp_tangent_bound(arm, edge.q0, edge.q1) / duration
            if np.any(velocity > arm.robot.velocity * planner.timing["velocity_fraction"] + 1e-8):
                raise ValueError("Saved path exceeds velocity limits")
            if np.any(acceleration > np.array(planner.timing["joint_acceleration_rad_s2"]) + 1e-8):
                raise ValueError("Saved path exceeds acceleration limits")
            if speed_bound > limit + 1e-8:
                raise ValueError(
                    "Cannot certify saved tip speed: "
                    f"{row['arm']}/{row['stroke_id']} edge {edge_index}; "
                    f"upper bound {speed_bound:.9g} m/s, limit {limit:.9g} m/s"
                )
            output["checked_edges"] += 1
            if monotonic() - last_write >= 1:
                write_json(run / validation_name, output)
                last_write = monotonic()
        if path_certificates is not None:
            path_certificates.add(key)
        proposal = MotionProposal(
            row["stroke_id"],
            row["arm"],
            row["variant"],
            path.duration_s,
            row["start_anchor"],
            tuple(path.edges[-1].q1),
            path,
            auxiliary=row.get("auxiliary", False),
        )
        moves.append(ScheduledMotion(proposal, start))
        previous[row["arm"]] = (end, proposal.end_configuration)
        output["checked_actions"] += 1
    drawings = [m for m in moves if m.proposal.task in tasks]
    if len({m.proposal.task for m in drawings}) != len(drawings):
        raise ValueError("A stroke is scheduled more than once")
    expected_events = sorted(
        [
            {
                "time_s": move.start_s + edge.start_s,
                "arm": move.proposal.arm,
                "stroke_id": move.proposal.task,
                "command": "FEED_ON" if edge.kind == "PEN_START_WAIT" else "FEED_OFF",
            }
            for move in moves
            for edge in move.proposal.payload.edges
            if edge.kind in ("PEN_START_WAIT", "PEN_STOP_WAIT")
            and move.start_s + edge.start_s <= horizon
        ],
        key=lambda event: (event["time_s"], event["arm"]),
    )
    if saved.get("events") != expected_events:
        raise ValueError("Saved feed commands do not match the per-arm path clock")
    ready = {node: 0 for node in planner.ground}
    for motion in drawings:
        for node in tasks[motion.proposal.task].deposits:
            ready[node] = min(
                ready.get(node, float("inf")), motion.end_s + tasks[motion.proposal.task].cooling_s
            )
    for motion in drawings:
        if ready.get(motion.proposal.start_anchor, float("inf")) > motion.start_s + 1e-9:
            raise ValueError("A stroke starts on future or still-cooling material")
        if motion.proposal.start_anchor not in tasks[motion.proposal.task].anchors:
            raise ValueError("Saved start anchor does not belong to its stroke")
    events = sorted(
        {0.0, horizon, *[t for m in moves for t in (m.start_s, m.end_s) if 0 < t < horizon]}
    )
    if checkpoint is not None:
        state = restore_state(checkpoint, moves, tasks, initial, planner.ground, events)
        planner.checked_intervals = checkpoint["checked_intervals"]
        output.update(
            resumed_from_s=state.time_s,
            completed_strokes=len(state.completed),
            checked_intervals=planner.checked_intervals,
        )
    verified_intervals = planner.checked_intervals
    last_checkpoint = monotonic()
    output.update(phase="COLLISION_REPLAY")
    output.pop("active_action", None)
    write_json(run / validation_name, output)
    if not moves or horizon == 0:
        output["status"] = "NO_MOTION_TO_REPLAY"
        write_json(run / validation_name, output)
        return (output, state) if return_state else output
    save_checkpoint(checkpoint_path, checkpoint_context, state, verified_intervals)
    for a, b in zip(events[:-1], events[1:], strict=True):
        if b <= state.time_s:
            continue
        new = tuple(m for m in moves if abs(m.start_s - a) < 1e-9)
        scene, failure = planner.validate_interval(state, new, b)
        if failure:
            output.update(
                status="REPLAY_TIME_BUDGET"
                if failure.get("status") == "TIME_BUDGET"
                else "REPLAY_REJECTED",
                failures=[failure],
                checked_intervals=planner.checked_intervals,
            )
            save_checkpoint(checkpoint_path, checkpoint_context, state, verified_intervals)
            break
        active = state.active + new
        q = dict(state.configurations)
        done = set(state.completed)
        support_ready = dict(state.support_ready)
        for motion in active:
            if motion.end_s <= b + 1e-9:
                q[motion.proposal.arm] = motion.proposal.end_configuration
                if motion.proposal.task in tasks:
                    done.add(motion.proposal.task)
                    task = tasks[motion.proposal.task]
                    for node in task.deposits:
                        support_ready[node] = min(
                            support_ready.get(node, float("inf")), motion.end_s + task.cooling_s
                        )
        state = replace(
            state,
            time_s=b,
            configurations=tuple(q.items()),
            completed=frozenset(done),
            support_ready=tuple(sorted(support_ready.items())),
            active=tuple(m for m in active if m.end_s > b + 1e-9),
            history=state.history + new,
            payload=scene,
        )
        output.update(completed_strokes=len(done), checked_intervals=planner.checked_intervals)
        verified_intervals = planner.checked_intervals
        if monotonic() - last_checkpoint >= 30 or b == horizon:
            save_checkpoint(checkpoint_path, checkpoint_context, state, verified_intervals)
            last_checkpoint = monotonic()
        write_json(run / validation_name, output)
    else:
        full = (
            saved.get("full_schedule", False) or saved["status"] == "COORDINATED_GEOMETRIC_PREVIEW"
        )
        if full and (
            len(state.completed) != len(tasks)
            or state.active
            or any(
                not np.allclose(dict(state.configurations)[i], q, atol=1e-8, rtol=0)
                for i, q in initial.items()
            )
        ):
            output.update(
                status="REPLAY_REJECTED", failures=[{"status": "INCOMPLETE_OR_NOT_PARKED"}]
            )
        else:
            output["status"] = "CERTIFIED_GEOMETRIC_REPLAY" if full else "CERTIFIED_PREFIX_REPLAY"
    write_json(run / validation_name, output)
    return (output, state) if return_state else output


def replay_cell(
    run, profile_path, maximum_seconds=300, plan_name="dual_arm_plan", *, path_certificates=None
):
    plan_file = plan_name + ".json"
    validation_name = plan_name.removesuffix("_plan") + "_validation.json"
    stage_update(
        Path(run),
        "validation",
        "running",
        artifact=validation_name,
        message="Replaying saved shared clock",
    )
    try:
        output = _replay_cell(
            run, profile_path, maximum_seconds, plan_name, path_certificates=path_certificates
        )
        stage_update(
            Path(run),
            "validation",
            "complete" if output["status"] == "CERTIFIED_GEOMETRIC_REPLAY" else "partial",
            artifact=validation_name,
            message=output["status"] + "; modeled geometry only, physical fabrication unverified",
        )
        return output
    except Exception as error:
        path = Path(run) / validation_name
        digest = sha256(Path(run) / plan_file)
        try:
            output = read_json(path)
        except (OSError, ValueError):
            output = {}
        if output.get("source_plan_sha256") != digest:
            output = artifact("dual_arm_validation")
        output.update(
            status="REPLAY_ERROR",
            simulation_only=True,
            executable=False,
            source_plan_sha256=digest,
            inputs={plan_file: digest},
            failures=[{"type": type(error).__name__, "message": str(error)}],
        )
        write_json(path, output)
        stage_update(
            Path(run), "validation", "failed", artifact=validation_name, message=str(error)
        )
        raise


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, default=Path("runs/palace_15cm_dual"))
    parser.add_argument("--config", type=Path, default=Path("configs/cell/two_flexiv.yaml"))
    parser.add_argument("--maximum-seconds", type=float, default=300)
    parser.add_argument("--plan-name", default="dual_arm_plan")
    args = parser.parse_args()
    result = replay_cell(args.run, args.config, args.maximum_seconds, args.plan_name)
    print(result["status"], result["completed_strokes"], result["checked_intervals"])
