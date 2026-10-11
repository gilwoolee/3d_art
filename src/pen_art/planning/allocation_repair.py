"""Targeted parking repair for a recorded moving-versus-idle arm conflict."""

from dataclasses import replace

import numpy as np

from .clock_path import ClockPath, timed_path
from .scene import MaterialScene


def obstructing_material_modes(evidence, modes):
    """Resolve named deposited strands in preparation or shared-replay evidence."""
    found = set()

    def visit(value):
        if isinstance(value, dict):
            if value.get("status") in {"COLLISION", "CLEARANCE_UNRESOLVED"}:
                for pair in value.get("pairs", []):
                    for name in (pair.get("first", ""), pair.get("second", "")):
                        parts = name.split("/")
                        # Linear edges use arm/stroke/edge; polynomial pieces
                        # add another component. Replay adds a material/ prefix.
                        # Only known arm/stroke IDs count.
                        for offset in (0, 1):
                            if len(parts) >= offset + 3:
                                mode = modes.get(tuple(parts[offset : offset + 2]))
                                if mode is not None:
                                    found.add(mode)
            for child in value.values():
                visit(child)
        elif isinstance(value, list):
            for child in value:
                visit(child)

    visit(evidence)
    return sorted(found)


def material_precedence(output, failure, prepared):
    """Identify fixed motions that must precede a witnessed material obstruction."""
    time, arm = failure.get("time_s"), failure.get("arm")
    if time is None or arm is None:
        return []
    active = [
        a
        for a in output["actions"]
        if a["arm"] == arm and not a.get("auxiliary") and a["start_s"] <= time < a["end_s"]
    ]
    if len(active) != 1:
        return []
    modes = {(p.arm, p.task): key for key, p in prepared.items()}
    moving = modes.get((arm, active[0]["stroke_id"]))
    if moving is None:
        return []

    return [
        [moving, other] for other in obstructing_material_modes(failure, modes) if other != moving
    ]


def join_paths(*paths):
    """Concatenate exact clocks, preserving geometry and boundary stops."""
    edges, elapsed = [], 0.0
    for path in paths:
        if edges and not np.allclose(edges[-1].q1, path.edges[0].q0, atol=1e-8, rtol=0):
            raise ValueError("Cannot join discontinuous joint paths")
        edges.extend(
            replace(e, start_s=e.start_s + elapsed, end_s=e.end_s + elapsed) for e in path.edges
        )
        elapsed += path.duration_s
    joined = ClockPath(edges)
    joined.metadata = {key: value for path in paths for key, value in path.metadata.items()}
    return joined


def material_at(planner, actions, time):
    """Construct the actual deposited prefix, including partial drawing edges."""
    scene = MaterialScene()
    for action in actions:
        if action["start_s"] >= time:
            continue
        path = ClockPath.from_record(action["trajectory"])
        for index, edge in enumerate(path.edges):
            if edge.kind != "DRAW" or action["start_s"] + edge.start_s >= time:
                continue
            end = min(edge.end_s, time - action["start_s"])
            planner.add_clock_edge(
                scene,
                action["arm"],
                edge,
                f"prefix/{action['arm']}/{action['stroke_id']}/{index}",
                end=end,
            )
    return scene


def park_idle_arm(planner, output, failure, prepared):
    """Park after the idle arm's preceding stroke and rebase its next transit.

    The assigned drawing paths are unchanged. The new motions are checked against
    the recorded material prefixes here and against both arms during full replay.
    This mutates ``prepared`` only after both transit plans succeed.
    """
    time = failure.get("time_s")
    if time is None:
        return {"status": "NO_CONFLICT_TIME"}
    actions = output["actions"]
    moving = [a for a in actions if a["start_s"] <= time < a["end_s"]]
    idle = set(planner.arms) - {a["arm"] for a in moving}
    if len(idle) != 1 or len(moving) != 1:
        return {"status": "NOT_A_MOVING_IDLE_CONFLICT"}
    identity = idle.pop()
    past = [a for a in actions if a["arm"] == identity and a["end_s"] <= time]
    if not past:
        return {"status": "INITIAL_POSE_CONFLICT", "arm": identity}
    previous = max(past, key=lambda a: a["end_s"])
    by_task = {(p.arm, p.task): key for key, p in prepared.items()}
    key = by_task.get((identity, previous["stroke_id"]))
    if key is None:
        return {"status": "IDLE_AFTER_AUXILIARY_MOTION", "arm": identity}
    arm, before = planner.arms[identity], prepared[key]
    if np.allclose(before.end_configuration, arm.initial_positions, atol=1e-8, rtol=0):
        return {"status": "PARKED_POSE_CONFLICT", "arm": identity}

    def transfer(start, end, scene, kind):
        result = planner.travel(identity, scene, np.asarray(start), np.asarray(end))
        if result["status"] != "CONNECTED":
            return None, result
        return timed_path(
            [{"kind": kind, "positions": result["positions"]}],
            arm,
            arm.robot.velocity * planner.timing["velocity_fraction"],
            np.asarray(planner.timing["joint_acceleration_rad_s2"]),
            planner.process["nominal_speed_m_s"],
            planner.timing["travel_speed_m_s"],
        ), None

    parked, error = transfer(
        before.end_configuration,
        arm.initial_positions,
        material_at(planner, actions, previous["end_s"]),
        "PARK",
    )
    if error:
        return {"status": "PARK_PATH_REJECTED", "arm": identity, "evidence": error}
    updated = join_paths(before.payload, parked)
    changes = {
        key: replace(
            before,
            payload=updated,
            duration_s=updated.duration_s,
            end_configuration=tuple(arm.initial_positions),
        )
    }
    future = sorted(
        [a for a in actions if a["arm"] == identity and a["start_s"] >= time],
        key=lambda a: a["start_s"],
    )
    next_key = None
    if future:
        following = future[0]
        next_key = by_task.get((identity, following["stroke_id"]))
        if next_key is None:
            # An immediate final return becomes unnecessary after this park.
            if following["stroke_id"] != f"return-{identity}":
                return {"status": "UNSUPPORTED_AUXILIARY_SUCCESSOR"}
        else:
            after = prepared[next_key]
            tail = [e for e in after.payload.edges if e.kind != "TRAVEL"]
            offset = tail[0].start_s
            tail_path = ClockPath(
                [replace(e, start_s=e.start_s - offset, end_s=e.end_s - offset) for e in tail]
            )
            tail_path.metadata = after.payload.metadata.copy()
            approach, error = transfer(
                arm.initial_positions,
                tail[0].q0,
                material_at(planner, actions, following["start_s"]),
                "TRAVEL",
            )
            if error:
                return {"status": "REBASED_TRAVEL_REJECTED", "arm": identity, "evidence": error}
            path = join_paths(approach, tail_path)
            changes[next_key] = replace(after, payload=path, duration_s=path.duration_s)
    prepared.update(changes)
    return {
        "status": "PARK_ADDED",
        "arm": identity,
        "after_mode": key,
        "next_mode": next_key,
        "park_duration_s": parked.duration_s,
        "witness": failure,
    }
