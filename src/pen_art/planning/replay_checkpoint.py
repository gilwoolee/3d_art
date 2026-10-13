"""Persist only fully checked shared-clock boundaries and their exact material."""

import hashlib
import json
import sys
from importlib.metadata import version
from pathlib import Path

from ..io import artifact, read_json, sha256, write_json
from .dual_arm import ScheduleState
from .process_clock import PROCESS_CLOCK_POLICY
from .scene import MaterialScene


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


# Capture the implementation when this process imports it. A later source edit
# must not label an already running verifier with the new implementation hash.
IMPLEMENTATION = digest(
    {
        "sources": {
            str(path.relative_to(Path(__file__).parents[1])): sha256(path)
            for path in sorted(Path(__file__).parents[1].rglob("*.py"))
        },
        "python": sys.version,
        "packages": {name: version(name) for name in ("numpy", "scipy", "python-fcl", "toppra")},
    }
)


def read_checkpoint(path, context):
    if not path.is_file():
        return None
    try:
        row = read_json(path)
        checksum = row.pop("checksum")
        if (
            row.get("kind") != "cell_replay_checkpoint"
            or row.get("context") != context
            or checksum != digest(row)
            or row["time_s"] < 0
            or row["checked_intervals"] < 0
            or not isinstance(row["material_segments"], list)
        ):
            return None
    except (OSError, ValueError, KeyError, TypeError):
        return None
    return row


def save_checkpoint(path, context, state, checked_intervals):
    row = artifact(
        "cell_replay_checkpoint",
        context=context,
        time_s=state.time_s,
        checked_intervals=checked_intervals,
        material_segments=state.payload.segments,
        table_index=state.table_index,
    )
    row["checksum"] = digest(row)
    write_json(path, row)


def restore_state(checkpoint, moves, tasks, initial, ground, events):
    """Derive robot/support state from the bound plan, retaining exact capsules."""
    time = checkpoint["time_s"]
    if time not in events:
        raise ValueError("Replay checkpoint is not at a saved event boundary")
    history = tuple(move for move in moves if move.start_s < time)
    completed = [move for move in history if move.end_s <= time + 1e-9]
    q = dict(initial)
    ready = dict.fromkeys(ground, 0.0)
    done = set()
    for move in sorted(completed, key=lambda move: move.end_s):
        q[move.proposal.arm] = move.proposal.end_configuration
        if move.proposal.task in tasks:
            done.add(move.proposal.task)
            task = tasks[move.proposal.task]
            path = move.proposal.payload
            if path.metadata.get("process_clock_policy") == PROCESS_CLOCK_POLICY:
                # Events were reconstructed against source geometry before restore.
                values = {}
                for event in path.metadata["deposition_events"]:
                    node = event["node"]
                    values[node] = min(values.get(node, float("inf")), event["ready_at_s"])
                if not set(task.deposits) <= values.keys():
                    raise ValueError("Checkpoint material readiness is incomplete")
                for node in task.deposits:
                    ready[node] = min(ready.get(node, float("inf")), move.start_s + values[node])
            else:
                for node in task.deposits:
                    ready[node] = min(ready.get(node, float("inf")), move.end_s + task.cooling_s)
    scene = MaterialScene()
    scene.table_index = checkpoint.get("table_index", 0)
    for segment in checkpoint["material_segments"]:
        scene.add(segment["start"], segment["end"], segment["radius_m"], segment["name"])
    return ScheduleState(
        time,
        tuple(q.items()),
        frozenset(done),
        tuple(sorted(ready.items())),
        active=tuple(move for move in history if move.end_s > time + 1e-9),
        history=history,
        payload=scene,
    )
