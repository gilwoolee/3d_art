"""Resume an exact fully qualified, cooled forward prefix without replaying it."""

from pathlib import Path

import numpy as np

from ..io import read_json, sha256, write_json
from .dual_arm import ScheduleState
from .path_certificates import _canonical
from .replay_checkpoint import IMPLEMENTATION, digest
from .scene import MaterialScene

POLICY = "qualified-cooled-forward-prefix-v1"
GATES = {
    "source": "BOUNDED",
    "structure": "MODELED_PASS",
    "timing": "CHECKED",
    "interval": "VERIFIED_VALID",
}


def snapshot_gates(context):
    """A checked pen proposal is never a full-robot qualification."""
    if context.get("physical", {}).get("collision_scope") == "pen_candidate":
        return {**GATES, "interval": "VERIFIED_PEN_CANDIDATE"}
    return dict(GATES)


def _kind(context):
    return (
        "qualified_pen_candidate_state"
        if snapshot_gates(context)["interval"] == "VERIFIED_PEN_CANDIDATE"
        else "qualified_forward_state"
    )


def _binding(context, order, witnesses):
    return {
        "policy": POLICY,
        "implementation": IMPLEMENTATION,
        "context_sha256": digest(_canonical(context)),
        "order": list(order),
        "witnesses": _canonical(witnesses),
    }


def _check_witnesses(root, order, witnesses):
    previous = None
    last = None
    for index, reference in enumerate(witnesses):
        relative = Path(reference["file"])
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError("Unsafe forward witness path")
        path = root / relative
        if sha256(path) != reference["sha256"]:
            raise ValueError("Forward witness changed")
        saved = read_json(path)
        if saved["section"] != order[index] or saved["drawing"]["stroke_id"] != order[index]:
            raise ValueError("Forward witness order changed")
        edges = saved["trajectory"]["edges"]
        if previous is not None and not np.allclose(previous, edges[0]["q0"], atol=1e-10, rtol=0):
            raise ValueError("Forward witness joint channel is discontinuous")
        previous = edges[-1]["q1"]
        last = saved
    return last


def _boundary_matches(body, witness):
    identity = witness["drawing"]["arm"]
    q = dict(body["configurations"])
    if set(q) != {identity} or not np.allclose(
        q[identity], witness["trajectory"]["edges"][-1]["q1"], atol=1e-10, rtol=0
    ):
        raise ValueError("Forward state joint boundary differs from its exact clock")
    edge = witness["trajectory"]["edges"][-1]
    index = edge.get("table_to_index") if edge["kind"] == "INDEX_TURN" else edge.get("table_index")
    if index is not None and index != body["table_index"]:
        raise ValueError("Forward state table boundary differs from its exact clock")


def save_forward_state(path, context, order, witnesses, state, gates, *, witness_root, raw=False):
    if raw:
        return None
    if gates != snapshot_gates(context) or state.active or state.history:
        raise ValueError("Only a fully checked inactive forward boundary can be saved")
    if not witnesses or set(state.completed) != set(order[: len(witnesses)]):
        raise ValueError("Forward state must complete exactly the witness prefix")
    if (
        not np.isfinite(state.time_s)
        or state.time_s < 0
        or any(not np.isfinite(ready) or ready > state.time_s for _, ready in state.support_ready)
    ):
        raise ValueError("Forward snapshot must be a cooled boundary")
    last = _check_witnesses(Path(witness_root), order, witnesses)
    body = _canonical(
        {
            "time_s": state.time_s,
            "configurations": state.configurations,
            "completed": sorted(state.completed),
            "support_ready": state.support_ready,
            "table_index": state.table_index,
            "material_segments": state.payload._physical_segments,
            "material_geometry_sha256": state.payload.geometry_prefix()[0].hex(),
        }
    )
    if any(not np.isfinite(q).all() for _, q in state.configurations):
        raise ValueError("Forward configurations must be finite")
    _boundary_matches(body, last)
    row = {
        "kind": _kind(context),
        "requires_full_robot_validation": _kind(context) == "qualified_pen_candidate_state",
        "binding": _binding(context, order, witnesses),
        "gates": gates,
        "state": body,
        "material_sha256": digest(body["material_segments"]),
    }
    row["checksum"] = digest(row)
    write_json(Path(path), row)
    return row["checksum"]


def restore_forward_state(path, context, order, witnesses, *, witness_root, raw=False):
    if raw:
        return None
    try:
        row = read_json(Path(path))
        checksum = row.pop("checksum")
        refs = row["binding"]["witnesses"]
        if (
            row.get("kind") != _kind(context)
            or row.get("gates") != snapshot_gates(context)
            or row.get("requires_full_robot_validation")
            != (_kind(context) == "qualified_pen_candidate_state")
            or checksum != digest(row)
            or not refs
            or refs != _canonical(witnesses[: len(refs)])
            or len(refs) > len(order)
            or row["binding"] != _binding(context, order, refs)
        ):
            return None
        last = _check_witnesses(Path(witness_root), order, refs)
        body = row["state"]
        if (
            digest(body["material_segments"]) != row["material_sha256"]
            or set(body["completed"]) != set(order[: len(refs)])
            or not np.isfinite(body["time_s"])
            or body["time_s"] < 0
            or any(not np.isfinite(t) or t > body["time_s"] for _, t in body["support_ready"])
            or any(not np.isfinite(q).all() for _, q in body["configurations"])
            or len(dict(body["configurations"])) != len(body["configurations"])
            or not isinstance(body["table_index"], int)
            or body["table_index"] not in range(4)
        ):
            return None
        _boundary_matches(body, last)
        scene = MaterialScene()
        scene.table_index = body["table_index"]
        for segment in body["material_segments"]:
            scene.add(segment["start"], segment["end"], segment["radius_m"], segment["name"])
        if scene.geometry_prefix() != (
            bytes.fromhex(body["material_geometry_sha256"]),
            body["table_index"],
        ):
            return None
        state = ScheduleState(
            body["time_s"],
            tuple((name, tuple(q)) for name, q in body["configurations"]),
            frozenset(body["completed"]),
            tuple((name, time) for name, time in body["support_ready"]),
            payload=scene,
        )
        return {"state": state, "witnesses": refs, "snapshot_sha256": checksum}
    except (OSError, ValueError, KeyError, TypeError, IndexError):
        return None
