"""Plan different whole drawing actions of one supplied order in parallel.

Physical printing stays sequential. Only computation is concurrent: every
action first fixes a local drawing interpolant and its DRAW clock, from which
its deposited enclosures follow without any collision verdict. A later action
is then checked against those *predicted* deposits while earlier checks are
still running. Each result is bound to the exact candidates it assumed, so a
replaced upstream candidate silently invalidates everything computed from it.

Nothing here certifies a build. Workers use the planner's pen-candidate
collision scope; acceptance remains the independent ordered full-robot replay.
"""

import hashlib
import os
import pickle
import traceback
from concurrent.futures import FIRST_COMPLETED, ProcessPoolExecutor, wait
from concurrent.futures.process import BrokenProcessPool
from dataclasses import replace
from multiprocessing import get_context
from pathlib import Path
from time import monotonic, perf_counter

import numpy as np

from ..curves import flatten_curve, transform_points
from ..io import read_json, write_json
from .replay_checkpoint import IMPLEMENTATION, digest
from .scene import MaterialScene
from .schedule import MotionProposal, ScheduledMotion

POLICY = "parallel-forward-v1"
HOME = "home"
# Outcomes that say nothing about feasibility of the candidate itself.
UNRESOLVED = {"UNRESOLVED", "ERROR", "WORKER_LOST"}
# Seconds between live progress reports while jobs are finishing.
PROGRESS_INTERVAL_S = 5.0


def deposit_key(operations):
    """Bind deposited geometry and turn order, independently of segment names."""
    state = hashlib.sha256(b"material-operations-v1")
    for operation in operations:
        if operation[0] == "turn":
            state.update(b"turn" + bytes([operation[1], operation[2]]))
        else:
            rows = operation[1]
            state.update(b"add" + len(rows).to_bytes(8, "little"))
            state.update(
                np.asarray(
                    [[*row["start"], *row["end"], row["radius_m"]] for row in rows], dtype="<f8"
                ).tobytes()
            )
    return state.hexdigest()


def deposit_difference(first, second):
    """Largest endpoint/radius difference between two deposit journals, or None."""
    first = [row for operation in first if operation[0] == "add" for row in operation[1]]
    second = [row for operation in second if operation[0] == "add" for row in operation[1]]
    if len(first) != len(second):
        return None
    if not first:
        return 0.0
    values = [
        np.asarray([[*row["start"], *row["end"], row["radius_m"]] for row in rows])
        for rows in (first, second)
    ]
    return float(np.max(np.abs(values[0] - values[1])))


class MaterialPrefix:
    """Rebuild the material preceding an action from journaled deposits.

    One append-only scene is kept per process. A longer chain extends it; a
    shorter or diverging one shares its leading capsules. Insertions repeat
    the validator's own sequence, so the result is the scene sequential
    checking would hold.
    """

    def __init__(self, load):
        self.load = load
        self.rebuilds = 0
        self.reset()

    def reset(self):
        self.scene = MaterialScene()
        self.chain, self.counts = [], []

    def _apply(self, element):
        for operation in self.load(element[0]):
            for row in operation[1]:
                self.scene.add(row["start"], row["end"], row["radius_m"], row["name"])
        self.chain.append(element)
        self.counts.append(len(self.scene.objects))

    def scene_for(self, chain):
        chain = [tuple(element) for element in chain]
        common = 0
        while (
            common < min(len(chain), len(self.chain)) and chain[common] == self.chain[common]
        ):
            common += 1
        if common == len(self.chain):
            for element in chain[common:]:
                self._apply(element)
            return self.scene
        base = self.scene.prefix(self.counts[common - 1]) if common else MaterialScene()
        if common == len(chain):
            return base  # Serve a pure prefix; keep the longer scene held.
        self.rebuilds += 1
        self.scene = base
        del self.chain[common:], self.counts[common:]
        for element in chain[common:]:
            self._apply(element)
        return self.scene


def _failure(row):
    """Where a failed attempt failed (propose, check or connect) and its most specific status."""
    detail = row.get("detail")
    status = (detail.get("status") if isinstance(detail, dict) else None) or row["status"]
    return {"stage": row["stage"], "status": status}


class ForwardSchedule:
    """Dependency bookkeeping for one fixed construction order.

    This class holds no planner and runs no job. It decides which jobs are
    currently wanted, binds each to the exact upstream results it assumes, and
    discards any result whose assumptions have since been replaced.
    """

    def __init__(self, actions, binding, *, maximum_attempts=6, cold_modes=4, rows=None):
        """``rows`` gives each action's start node, end node, junction nodes and
        whether it is a closed loop. With them, a stroke that can start where
        its predecessor ended is proposed from that predecessor's final joint
        position, and the pair is drawn without lifting the pen.
        """
        self.actions, self.binding = list(actions), binding
        self.maximum_attempts, self.cold_modes = maximum_attempts, cold_modes
        self.rows = rows
        self.slots = [
            {
                "candidate": None,
                "check": None,
                "connect": None,
                "attempts": 0,
                "cold": 0,
                "history": [],
                "unresolved": None,
                # Set once a chained attempt failed; the stroke then lifts as usual.
                "unchained": False,
                "lift_reason": None,
            }
            for _ in self.actions
        ]
        self.returned = None
        self.stale = 0
        # Specs derived from the current slots; emptied whenever a slot changes.
        self._memo = {}

    def _remembered(self, name, index, compute):
        if (name, index) not in self._memo:
            self._memo[name, index] = compute(index)
        return self._memo[name, index]

    def _selected_chain(self, _=None):
        """Deposits of the leading selected candidates, with a digest per prefix.

        Every spec names the material before it. Hashing each prefix afresh for
        every action on every finished job grew with the cube of the action
        count; one running digest binds the same prefix.
        """
        chain, digests = [], [digest([])]
        for slot in self.slots:
            if slot["candidate"] is None:
                break
            chain.append([slot["candidate"]["deposit_key"], slot["candidate"]["table_index"]])
            digests.append(digest([digests[-1], chain[-1]]))
        return chain, digests

    def chain(self, count):
        chain, _ = self._remembered("chain", None, self._selected_chain)
        return None if count > len(chain) else chain[:count]

    def _prefix_digest(self, count):
        return self._remembered("chain", None, self._selected_chain)[1][count]

    def _key(self, *parts):
        return digest([POLICY, self.binding, *parts])

    def _check_spec(self, index):
        return self._remembered("check", index, self._derive_check_spec)

    def _derive_check_spec(self, index):
        chain = self.chain(index + 1)
        if chain is None:
            return None
        candidate = self.slots[index]["candidate"]
        # Whether this stroke withdraws depends on how its successor starts.
        chained_out = False
        if index + 1 < len(self.actions):
            following = self.slots[index + 1]["candidate"]
            if following is None:
                if self._may_chain(index + 1):
                    return None
            else:
                chained_out = following.get("chain_previous") == candidate["key"]
        chained_in = bool(candidate.get("chain_previous"))
        return {
            "kind": "check",
            "index": index,
            "candidate": candidate["key"],
            "deposit_key": candidate["deposit_key"],
            "chain": chain[:index],
            "chained_in": chained_in,
            "chained_out": chained_out,
            "key": self._key(
                "check", self.actions[index], candidate["key"], self._prefix_digest(index),
                chained_in, chained_out,
            ),
        }

    def _checked(self, index):
        """The current check result, or None when missing or bound to old inputs."""
        spec = self._check_spec(index)
        check = self.slots[index]["check"]
        return check if spec is not None and check and check["key"] == spec["key"] else None

    def _connect_spec(self, index):
        """Incoming transit of action ``index``; ``len(actions)`` returns home."""
        return self._remembered("connect", index, self._derive_connect_spec)

    def _derive_connect_spec(self, index):
        count = len(self.actions)
        chain = self.chain(index)
        previous = HOME if index == 0 else self._checked(index - 1)
        current = HOME if index == count else self._checked(index)
        if chain is None or previous is None or current is None:
            return None
        ends = [row if row == HOME else row["key"] for row in (previous, current)]
        return {
            "kind": "connect",
            "index": index,
            "previous": ends[0],
            "current": ends[1],
            "chain": chain,
            "start": None if previous == HOME else [previous["q_end"], previous["table_index"]],
            "end": None if current == HOME else [current["q_start"], current["table_index"]],
            "key": self._key("connect", index, ends, self._prefix_digest(index)),
        }

    def _connected(self, index):
        spec = self._connect_spec(index)
        held = self.returned if index == len(self.actions) else self.slots[index]["connect"]
        return held if spec is not None and held and held["key"] == spec["key"] else None

    def _starts(self, index):
        row = self.rows[index]
        return set(row["nodes"]) if row["closed"] else {row["start"]}

    def _may_chain(self, index):
        """Whether action ``index`` could start where its predecessor ends."""
        if self.rows is None or index <= 0 or index >= len(self.actions):
            return False
        slot = self.slots[index]
        if slot["unchained"] or slot["unresolved"] is not None:
            return False
        previous = self.rows[index - 1]
        ends = set(previous["nodes"]) if previous["closed"] else {previous["end"]}
        return bool(ends & self._starts(index))

    def _propose_spec(self, index):
        return self._remembered("propose", index, self._derive_propose_spec)

    def _derive_propose_spec(self, index):
        slot = self.slots[index]
        if slot["unresolved"] is not None or slot["candidate"] is not None:
            return None
        chain_from = None
        if self._may_chain(index):
            previous = self.slots[index - 1]
            if previous["candidate"] is None:
                if previous["unresolved"] is None:
                    return None  # Its starting joint position is not known yet.
            elif previous["candidate"].get("end_node") in self._starts(index):
                chain_from = {
                    "action": self.actions[index - 1],
                    "node": previous["candidate"]["end_node"],
                    "q": previous["candidate"]["q_draw_end"],
                    "rotation": previous["candidate"]["tcp_rotation"],
                    "pen_tilt_deg": previous["candidate"].get("pen_tilt_deg"),
                    "roll_rad": previous["candidate"].get("roll_rad"),
                    "table_index": previous["candidate"]["table_index"],
                    "previous": previous["candidate"]["key"],
                }
        chain = self.chain(index)
        if chain is None:
            return None  # A proposal screens against the preceding material.
        mode = {"mode": "cold", "ordinal": slot["cold"], "chain": chain}
        if chain_from is not None:
            mode["chain_from"] = chain_from
        elif self.rows is not None and index + 1 < len(self.rows):
            # A loop may start anywhere; start it where the next stroke begins.
            row, following = self.rows[index], self.rows[index + 1]
            if row["closed"] and not following["closed"] and following["start"] in row["nodes"]:
                mode["prefer_start"] = following["start"]
        return {
            "kind": "propose",
            "index": index,
            **mode,
            "key": self._key("propose", self.actions[index], mode),
        }

    def wanted(self):
        """Every job whose inputs exist and whose current result does not."""
        result = []
        for index in range(len(self.actions)):
            spec = self._propose_spec(index)
            if spec is not None:
                result.append(dict(spec))
            spec = self._check_spec(index)
            if spec is not None and self._checked(index) is None:
                result.append(dict(spec))
        for index in range(len(self.actions) + 1):
            spec = self._connect_spec(index)
            if spec is not None and self._connected(index) is None:
                result.append(dict(spec))
        return result

    def complete(self):
        return all(self._checked(index) for index in range(len(self.actions))) and all(
            self._connected(index) for index in range(len(self.actions) + 1)
        )

    def lift_offs(self):
        """Every selected stroke that starts with an approach, and why it lifts."""
        result = []
        for index, slot in enumerate(self.slots):
            candidate = slot["candidate"]
            if candidate is None or candidate.get("chain_previous"):
                continue
            if index == 0:
                reason = "first stroke"
            elif self.rows is None or not self._continues(index):
                reason = "not continuous"
            else:
                reason = slot["lift_reason"] or "chain not attempted"
            result.append({"before": self.actions[index], "reason": reason})
        return result

    def _continues(self, index):
        previous = self.rows[index - 1]
        ends = set(previous["nodes"]) if previous["closed"] else {previous["end"]}
        return bool(ends & self._starts(index))

    def blocked(self):
        """Actions with no remaining alternative; never a proof of infeasibility."""
        return [
            {
                "action": self.actions[index],
                "outcome": slot["unresolved"],
                "last_failure": _failure(slot["history"][-1]) if slot["history"] else None,
            }
            for index, slot in enumerate(self.slots)
            if slot["unresolved"] is not None
        ]

    def _exhaust(self, slot):
        if slot["attempts"] >= self.maximum_attempts or slot["cold"] >= self.cold_modes:
            statuses = {row["status"] for row in slot["history"]}
            slot["unresolved"] = (
                "UNRESOLVED_TIME_OR_WORKER" if statuses <= UNRESOLVED else "NO_VERIFIED_CANDIDATE"
            )

    def reject(self, index, stage, summary):
        """Replace one selected candidate; dependents invalidate through their keys."""
        slot = self.slots[index]
        candidate = slot["candidate"]
        slot["history"].append(
            {
                "stage": stage,
                "status": summary.get("status", "REJECTED"),
                "candidate": None if candidate is None else candidate["key"],
                "detail": summary.get("failure"),
            }
        )
        if candidate is not None:
            if candidate.get("chain_previous"):
                # One failed chained attempt is enough; this stroke lifts as usual.
                slot["unchained"] = True
                failure = summary.get("failure") or {}
                slot["lift_reason"] = "chain blocked: " + str(
                    failure.get("status") or summary.get("status", "REJECTED")
                )
            # Strokes chained after this one started from its joint position.
            for later in self.slots[index + 1 :]:
                if later["candidate"] is None or not later["candidate"].get("chain_previous"):
                    break
                later["candidate"] = later["check"] = later["connect"] = None
            slot["cold"] += 1
        slot["candidate"] = slot["check"] = slot["connect"] = None
        slot["attempts"] += 1
        self._exhaust(slot)
        self._memo.clear()

    def apply(self, spec, summary):
        """Record a finished job. Returns False for a result nothing wants now."""
        try:
            return self._record(spec, summary)
        finally:
            self._memo.clear()

    def _record(self, spec, summary):
        current = {row["key"] for row in self.wanted()}
        if spec["key"] not in current:
            self.stale += 1
            return False
        index, kind = spec["index"], spec["kind"]
        summary = {**summary, "key": spec["key"]}
        accepted = summary["status"] == "OK"
        if kind == "propose":
            slot = self.slots[index]
            if accepted:
                slot["candidate"] = {**summary, "mode": spec["mode"]}
                return True
            slot["history"].append(
                {"stage": "propose", "status": summary["status"], "detail": summary.get("failure")}
            )
            if spec.get("chain_from") is not None:
                # No drawing from the predecessor's joint position: lift instead.
                slot["unchained"] = True
                slot["lift_reason"] = f"chain blocked: {summary['status']}"
                return True
            slot["attempts"] += 1
            slot["cold"] += 1
            self._exhaust(slot)
        elif kind == "check":
            if accepted:
                self.slots[index]["check"] = summary
            else:
                self.reject(index, "check", summary)
        elif accepted:
            if index == len(self.actions):
                self.returned = summary
            else:
                self.slots[index]["connect"] = summary
        else:
            # A transit joins two selected local paths. Replace the later one:
            # it invalidates the fewest dependent material results.
            self.reject(min(index, len(self.actions) - 1), "connect", summary)
        return True

    def report(self):
        return [
            {
                "action": self.actions[index],
                "attempts": slot["attempts"],
                "selected": None
                if slot["candidate"] is None
                else {
                    name: slot["candidate"].get(name)
                    for name in ("mode", "source", "variant", "parent", "table_index", "capsules")
                },
                "candidate_key": None if slot["candidate"] is None else slot["candidate"]["key"],
                "deposit_key": None
                if slot["candidate"] is None
                else slot["candidate"]["deposit_key"],
                "check_key": (self._checked(index) or {}).get("key"),
                "connect_key": (self._connected(index) or {}).get("key"),
                "checked": self._checked(index) is not None,
                "connected": self._connected(index) is not None,
                "history": slot["history"],
                "outcome": slot["unresolved"],
            }
            for index, slot in enumerate(self.slots)
        ]


# --------------------------------------------------------------------------
# Worker side. A forked worker inherits the parent's prepared planner and its
# frozen motion library; everything it mutates afterwards is process-local.

_CONTEXT = None
_WORKER = {}


def _memory():
    values = {}
    try:
        for line in Path("/proc/self/smaps_rollup").read_text().splitlines():
            name, _, value = line.partition(":")
            if name in ("Rss", "Pss"):
                values[name.lower() + "_mb"] = int(value.split()[0]) / 1024
    except OSError:
        pass
    return values


def _store(root, key, payload):
    path = root / "jobs" / f"{key}.pkl"
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(f".{os.getpid()}.tmp")
    started = perf_counter()
    temporary.write_bytes(pickle.dumps(payload, protocol=pickle.HIGHEST_PROTOCOL))
    os.replace(temporary, path)
    return perf_counter() - started


def _load(root, key):
    return pickle.loads((root / "jobs" / f"{key}.pkl").read_bytes())


def _deposits(root, key):
    from collections import OrderedDict

    from .material_journal import load_journal

    cache = _WORKER.setdefault("deposits", OrderedDict())
    if key not in cache:
        path = root / "deposits" / f"{key}.json"
        value = (load_journal(path) if path.exists() else
                 pickle.loads(path.with_suffix(".pkl").read_bytes()))
        if deposit_key(value) != key:
            raise ValueError("Changed material journal")
        cache[key] = value
        while len(cache) > 4:
            cache.popitem(last=False)
    cache.move_to_end(key)
    return cache[key]


def _prefix(context):
    if "prefix" not in _WORKER:
        _WORKER["prefix"] = MaterialPrefix(lambda key: _deposits(context["root"], key))
    return _WORKER["prefix"]


def _completed(context, index):
    return frozenset(row["id"] for row in context["order"][:index])


def _points(context, row, index):
    points, _ = flatten_curve(row, 0.00005, 0.003)
    return transform_points(points, context["model"].table.build_transform(index))


def _journal(context, index, clock, table_index):
    """Deposits implied by a clock, from the validator's own deposition path."""
    from .indexed_build import at_index

    build = context["build"]
    scene = MaterialScene()
    scene.table_index = table_index
    clock = at_index(clock, table_index)
    move = MotionProposal(
        context["order"][index]["id"],
        build.arm.identity,
        "predicted-deposit",
        clock.duration_s,
        "",
        tuple(clock.edges[-1].q1),
        clock,
    )
    operations = []
    _, failure = build.planner.validate_interval(
        build.state(_completed(context, index), build.initial, scene),
        (ScheduledMotion(move, 0.0),),
        clock.duration_s,
        deposit_only=True,
        journal=operations,
    )
    if failure:
        raise ValueError(f"Deposition could not be derived: {failure}")
    return operations


def _propose(context, spec, seconds):
    build, model, planner = context["build"], context["model"], context["build"].planner
    index = spec["index"]
    row, completed = context["order"][index], _completed(context, index)
    empty = MaterialScene()
    available = build.state(completed, build.initial, empty).ready_nodes()
    # The planner's own sampler. It screens the nozzle against the predicted
    # preceding material. Each retry (``ordinal``) samples a finer path and
    # the other pen roll, for a stroke that missed the path-accuracy check.
    chain_from = spec.get("chain_from")
    if chain_from is not None:
        # Carry on from the predecessor's end: same node, joint position, pen
        # orientation and table position, with the pen still down.
        available = {chain_from["node"]}
    table_index = 0

    scene = _prefix(context).scene_for(spec["chain"])
    started = perf_counter()
    for drawing in model.drawings(
        row, available, build.initial, table_index, scene,
        chain=chain_from, prefer_start=spec.get("prefer_start"), retry=spec["ordinal"],
    ):
        seconds["proposal"] += perf_counter() - started
        started = perf_counter()
        if not model.structure_admissible(row, drawing, completed):
            continue
        local = planner.prepare_local_drawing(build.arm.identity, drawing)
        seconds["local_drawing"] += perf_counter() - started
        if local is None:
            started = perf_counter()
            continue
        started = perf_counter()
        operations = _journal(context, index, local["draw_clock"], table_index)
        local["deposits"] = [row for op in operations if op[0] == "add" for row in op[1]]
        operations = _merged(planner, operations)
        seconds["predicted_deposit"] += perf_counter() - started
        key = deposit_key(operations)
        from .material_journal import write_journal

        path = context["root"] / "deposits" / f"{key}.json"
        if not path.exists():
            write_journal(path, operations)
        seconds["serialization"] += _store(
            context["root"],
            spec["key"],
            {"drawing": drawing, "local": local, "move": None, "table_index": table_index},
        )
        return {
            "status": "OK",
            "deposit_key": key,
            "table_index": table_index,
            "capsules": sum(len(op[1]) for op in operations if op[0] == "add"),
            "source": drawing.get("proposal_source", "planner"),
            "variant": drawing.get("variant"),
            # What a stroke chained after this one starts from.
            "end_node": drawing.get("end_anchor"),
            "q_draw_end": np.asarray(drawing["positions"][-1], float).tolist(),
            "tcp_rotation": np.asarray(drawing["tcp_rotation"], float).tolist(),
            "pen_tilt_deg": drawing.get("pen_tilt_deg"),
            "roll_rad": drawing.get("roll_rad"),
            "chain_previous": (
                None if spec.get("chain_from") is None else spec["chain_from"]["previous"]
            ),
        }
    seconds["proposal"] += perf_counter() - started
    return {
        "status": "UNRESOLVED" if monotonic() >= planner.deadline else "NO_CANDIDATE",
        "failure": {"attempts": planner.attempts[-3:]},
    }


def _check(context, spec, seconds):
    from .clock_path import posture_change
    from .indexed_build import at_index

    build, planner = context["build"], context["build"].planner
    index = spec["index"]
    row, completed = context["order"][index], _completed(context, index)
    started = perf_counter()
    candidate = _load(context["root"], spec["candidate"])
    seconds["serialization"] += perf_counter() - started
    table_index, drawing = candidate["table_index"], candidate["drawing"]
    started = perf_counter()
    scene = _prefix(context).scene_for(spec["chain"])
    seconds["material_reconstruction"] += perf_counter() - started
    state = build.state(completed, build.initial, scene)
    move = candidate["move"]
    if move is None:
        started = perf_counter()
        move = planner.complete_local_drawing(
            state,
            build.arm.identity,
            drawing,
            candidate["local"],
            None,
            defer_drawing_check=True,
            chained_in=spec.get("chained_in", False),
            chained_out=spec.get("chained_out", False),
        )
        seconds["approach_withdraw"] += perf_counter() - started
        if move is None:
            return {
                "status": "UNRESOLVED" if monotonic() >= planner.deadline else "REJECTED",
                "failure": planner.attempts[-1] if planner.attempts else None,
            }
    local = at_index(move.payload, table_index)
    local.metadata["drawing_table_index"] = table_index
    swing = posture_change(local, build.arm)
    if swing:
        return {"status": "REJECTED", "failure": swing}
    move = replace(move, payload=local, duration_s=local.duration_s)
    operations = []
    started = perf_counter()
    _, failure = planner.validate_interval(
        state, (ScheduledMotion(move, 0.0),), move.duration_s, journal=operations
    )
    seconds["material_validation"] += perf_counter() - started
    if failure:
        return {
            "status": "UNRESOLVED" if failure.get("status") == "TIME_BUDGET" else "REJECTED",
            "failure": failure,
        }
    if not _untimed(local):
        started = perf_counter()
        failure = _timing_failure(build, row, local)
        seconds["timing_audit"] += perf_counter() - started
        if failure:
            return failure
    difference = deposit_difference(
        _merged(planner, operations), _deposits(context["root"], spec["deposit_key"])
    )
    if difference is None or difference > 1e-9:
        return {
            "status": "REJECTED",
            "failure": {"status": "PREDICTED_DEPOSIT_MISMATCH", "difference_m": difference},
        }
    seconds["serialization"] += _store(
        context["root"], spec["key"], {"clock": local, "drawing": drawing}
    )
    return {
        "status": "OK",
        "table_index": table_index,
        "q_start": local.edges[0].q0.tolist(),
        "q_end": local.edges[-1].q1.tolist(),
        "duration_s": local.duration_s,
        "edges": len(local.edges),
        "preceding_capsules": len(scene.objects),
        "predicted_deposit_difference_m": difference,
    }


def _merged(planner, operations):
    """Deposits as later actions will see them; an action's own wire stays fine."""
    budget = planner.settings.get("merge_deposits_m", 0.0)
    if not budget:
        return operations
    from .material_journal import merge_deposits

    return merge_deposits(operations, budget)


def _untimed(path):
    return path.metadata.get("parameterization") == "geometric"


def _timing_failure(build, row, clock):
    try:
        build.check_forward_timing(row, clock)
    except TimeoutError as error:
        return {"status": "UNRESOLVED", "failure": {"status": "TIME_BUDGET", "detail": str(error)}}
    except ValueError as error:
        return {"status": "REJECTED", "failure": {"status": "TIMING", "detail": str(error)}}
    return None


def _connect(context, spec, seconds):
    from .plate import audit_plate_clock
    from .section_build import combine_clocks
    from .source_coverage import audit_source_curve

    build, model, planner = context["build"], context["model"], context["build"].planner
    index, home = spec["index"], context["model"].table.home_index
    start, start_index = (build.initial, home) if spec["start"] is None else spec["start"]
    end, end_index = (build.initial, home) if spec["end"] is None else spec["end"]
    start, end = np.asarray(start, float), np.asarray(end, float)
    started = perf_counter()
    scene = _prefix(context).scene_for(spec["chain"])
    seconds["material_reconstruction"] += perf_counter() - started
    started = perf_counter()
    # A chained stroke begins exactly where its predecessor stopped.
    chained = (
        spec["start"] is not None
        and spec["end"] is not None
        and start_index == end_index
        and np.array_equal(start, end)
    )
    incoming = None if chained else model.transition(scene, start, start_index, end, end_index)
    seconds["transit"] += perf_counter() - started
    if incoming is False:
        return {
            "status": "UNRESOLVED" if monotonic() >= planner.deadline else "REJECTED",
            "failure": {"status": "NO_CONNECTING_TRANSIT"},
        }
    if incoming is not None:
        move = MotionProposal(
            f"transit-{index}",
            build.arm.identity,
            "transit",
            incoming.duration_s,
            "parking",
            tuple(incoming.edges[-1].q1),
            incoming,
            auxiliary=True,
        )
        started = perf_counter()
        _, failure = planner.validate_interval(
            build.state(_completed(context, index), start, scene),
            (ScheduledMotion(move, 0.0),),
            move.duration_s,
        )
        seconds["transit_validation"] += perf_counter() - started
        if failure:
            return {
                "status": "UNRESOLVED" if failure.get("status") == "TIME_BUDGET" else "REJECTED",
                "failure": failure,
            }
    if spec["end"] is None:
        seconds["serialization"] += _store(context["root"], spec["key"], {"returned": incoming})
        return {"status": "OK", "duration_s": 0.0 if incoming is None else incoming.duration_s}
    row = context["order"][index]
    started = perf_counter()
    checked = _load(context["root"], spec["current"])
    seconds["serialization"] += perf_counter() - started
    path = combine_clocks(incoming, checked["clock"])
    path.metadata["drawing_table_index"] = end_index
    untimed = _untimed(checked["clock"])
    if not untimed:
        try:
            audit_plate_clock(path)
        except ValueError as error:
            return {
                "status": "REJECTED",
                "failure": {"status": "INDEXED_CLOCK", "detail": str(error)},
            }
    started = perf_counter()
    store = (None if planner.verification_raw else
             getattr(planner, "path_certificates", None))
    coverage_context = {
        "invariants": planner.path_certificate_context(build.arm.identity, kinematic=True),
        "source_points_m": _points(context, row, end_index),
    }
    audit = None if store is None else store.get(coverage_context, path.record(), "source_coverage")
    if audit is None:
        audit = audit_source_curve(build.arm, [path], coverage_context["source_points_m"])
        if store is not None and audit["status"] == "BOUNDED":
            store.admit(coverage_context, path.record(), "source_coverage", audit)
    seconds["source_coverage"] += perf_counter() - started
    if audit["status"] != "BOUNDED":
        return {"status": "REJECTED", "failure": {"status": "INDEXED_SOURCE", "audit": audit}}
    seconds["serialization"] += _store(
        context["root"],
        spec["key"],
        {
            "clock": path,
            "drawing": checked["drawing"],
            "source_audit": audit,
            # The final timing pass rebuilds this action around its own clock.
            **({"incoming": incoming, "local": checked["clock"]} if untimed else {}),
        },
    )
    return {
        "status": "OK",
        "duration_s": path.duration_s,
        "edges": len(path.edges),
        "untimed": untimed,
    }


def _time(context, spec, seconds):
    """Time one accepted untimed action; its checked sweep must be unchanged."""
    from ..timing import TimingParameterizationError
    from .geometric_path import same_sweep
    from .indexed_build import at_index
    from .plate import audit_plate_clock
    from .section_build import combine_clocks

    build, planner = context["build"], context["build"].planner
    row = context["order"][spec["index"]]
    started = perf_counter()
    payload = _load(context["root"], spec["connected"])
    seconds["serialization"] += perf_counter() - started
    local, table_index = payload["local"], payload["clock"].metadata["drawing_table_index"]
    started = perf_counter()
    try:
        timed = planner.time_local_drawing(build.arm.identity, local, row["id"], table_index)
    except TimingParameterizationError as error:
        return {"status": "REJECTED", "failure": {"status": "TIMING", "detail": str(error)}}
    except ValueError as error:
        return {"status": "REJECTED", "failure": {"status": "SWEEP", "detail": str(error)}}
    timed = at_index(timed, table_index)
    timed.metadata["drawing_table_index"] = table_index
    seconds["retiming"] += perf_counter() - started
    path = combine_clocks(payload["incoming"], timed)
    path.metadata["drawing_table_index"] = table_index
    # Audit the clock exactly as it is saved, transit included: a speed proof
    # binds each floating-point duration, and joining the transit shifts them.
    started = perf_counter()
    failure = _timing_failure(build, row, path)
    seconds["timing_audit"] += perf_counter() - started
    if failure:
        return failure
    guard = same_sweep(payload["clock"], path)
    if guard["status"] != "SAME_SWEEP":
        return {"status": "REJECTED", "failure": {"status": "SWEEP", "detail": guard}}
    # The timed clock visits exactly the configurations of the untimed path, so
    # that path's proof of covering the source curve holds for it too.
    store = None if planner.verification_raw else getattr(planner, "path_certificates", None)
    if store is not None:
        coverage_context = {
            "invariants": planner.path_certificate_context(build.arm.identity, kinematic=True),
            "source_points_m": _points(context, row, table_index),
        }
        # Its check and transit cleared the pen against exactly this chain of
        # earlier deposits, under the pen-only model. Verification may accept
        # that for this clock when the plan it replays has the same chain.
        store.admit(
            planner.path_certificate_context(build.arm.identity, pen=True),
            path.record(),
            "pen_material_path",
            {
                "status": "CHECKED",
                "deposit": spec["deposit"],
                "chain_sha256": digest(spec["chain"]),
                "derived_from": "same_sweep_of_checked_untimed_path",
            },
        )
        covered = store.get(coverage_context, payload["clock"].record(), "source_coverage")
        if covered is not None:
            store.admit(
                coverage_context,
                path.record(),
                "source_coverage",
                {**covered, "derived_from": "same_sweep_of_untimed_path"},
            )
    try:
        audit_plate_clock(path)
    except ValueError as error:
        return {"status": "REJECTED", "failure": {"status": "INDEXED_CLOCK", "detail": str(error)}}
    seconds["serialization"] += _store(
        context["root"], spec["key"], {"clock": path, "drawing": payload["drawing"]}
    )
    return {
        "status": "OK",
        "duration_s": path.duration_s,
        "edges": len(path.edges),
        "untimed_edges": guard["geometric_edges"],
    }


def run_job(spec):
    """Execute one job in a worker; a failure is a result, never an exception."""
    from collections import Counter

    from .collision import QUERY_STATS

    context = _CONTEXT
    planner = context["build"].planner
    seconds = Counter()
    started = perf_counter()
    # Worker-side workload, reported as this job's own change.
    sources = {
        "collision": QUERY_STATS,
        "verification_counts": getattr(planner, "verification_stats", {}),
        "verification_seconds": getattr(planner, "verification_seconds", {}),
    }
    before = {name: Counter(values) for name, values in sources.items()}
    planner.deadline = min(context["deadline"], monotonic() + context["budget"])
    try:
        summary = {"propose": _propose, "check": _check, "connect": _connect, "time": _time}[
            spec["kind"]
        ](context, spec, seconds)
    except TimeoutError as error:
        summary = {
            "status": "UNRESOLVED",
            "failure": {"status": "TIME_BUDGET", "detail": str(error)},
        }
    except Exception as error:  # A worker must report, not take the pool down.
        summary = {
            "status": "ERROR",
            "failure": {
                "type": type(error).__name__,
                "detail": str(error),
                "traceback": traceback.format_exc(limit=8),
            },
        }
    finally:
        planner.deadline = context["deadline"]
    prefix = _WORKER.get("prefix")
    return {
        **summary,
        "seconds": {**seconds, "total": perf_counter() - started},
        "counters": {
            name: dict(Counter(values) - before[name]) for name, values in sources.items()
        },
        "worker": os.getpid(),
        "memory": _memory(),
        "prefix_rebuilds": 0 if prefix is None else prefix.rebuilds,
    }


# --------------------------------------------------------------------------
# Coordinator.


class Coordinator:
    """Dispatch wanted jobs to a bounded pool and apply only current results."""

    def __init__(
        self,
        schedule,
        submit,
        workers,
        root,
        *,
        deadline,
        restart=None,
        progress=None,
        stall_seconds=float("inf"),
        kill=None,
        intercept=None,
    ):
        self.intercept = intercept
        self.schedule, self.submit, self.workers = schedule, submit, workers
        self.root, self.deadline = Path(root), deadline
        self.restart, self.progress = restart, progress
        self.stall_seconds, self.kill = stall_seconds, kill
        self.inflight, self.memo, self.retried, self.started = {}, {}, set(), {}
        self.jobs, self.resumed, self.lost = [], 0, 0

    def _recorded(self, key):
        """A summary published by an earlier interrupted run of this binding."""
        if key in self.memo:
            return self.memo[key]
        path = self.root / "jobs" / f"{key}.json"
        if path.is_file():
            try:
                self.memo[key] = read_json(path)
                self.resumed += 1
                return self.memo[key]
            except (OSError, ValueError):
                pass
        return None

    def _finish(self, spec, summary):
        key = spec["key"]
        if self.intercept is not None:
            summary = self.intercept(spec, summary)
        # Timeouts and lost workers are not reusable facts about a candidate.
        if summary["status"] not in UNRESOLVED:
            self.memo[key] = summary
            write_json(self.root / "jobs" / f"{key}.json", summary)
        current = self.schedule.apply(spec, summary)
        self.jobs.append(
            {
                "kind": spec["kind"],
                "index": spec["index"],
                "status": summary["status"],
                "current": current,
                "failure": summary.get("failure"),
                "seconds": summary.get("seconds", {}),
                "counters": summary.get("counters", {}),
                "worker": summary.get("worker"),
                "memory": summary.get("memory", {}),
            }
        )
        if self.progress is not None:
            self.progress(self)

    def _dispatch(self):
        """Apply recorded results, then fill idle workers in construction order."""
        while True:
            wanted = self.schedule.wanted()
            known = next(
                (
                    (spec, self._recorded(spec["key"]))
                    for spec in wanted
                    if spec["key"] not in self.inflight and self._recorded(spec["key"])
                ),
                None,
            )
            if known is None:
                break
            self.schedule.apply(*known)
        for spec in sorted(wanted, key=lambda row: (row["index"], row["kind"] != "propose")):
            if len(self.inflight) >= self.workers:
                break
            if spec["key"] in self.inflight:
                continue
            self.inflight[spec["key"]] = (self.submit(spec), spec)
            self.started[spec["key"]] = monotonic()

    def run(self):
        while monotonic() < self.deadline:
            if self.schedule.complete():
                return "COMPLETE"
            self._dispatch()
            if not self.inflight:
                # Nothing is running and nothing can start: either recorded
                # results finished the plan, or some action has exhausted its
                # alternatives.
                return "COMPLETE" if self.schedule.complete() else "BLOCKED"
            futures = {future: spec for future, spec in self.inflight.values()}
            done, _ = wait(futures, timeout=1.0, return_when=FIRST_COMPLETED)
            broken = False
            if self.kill is not None and any(
                monotonic() - self.started[key] > self.stall_seconds for key in self.inflight
            ):
                # Planner loops honor their deadline; a native call that does
                # not must not hold the whole build. Its jobs become unresolved.
                self.kill()
                done, _ = wait(futures, timeout=30.0)
            for future in done:
                spec = futures[future]
                del self.inflight[spec["key"]]
                try:
                    summary = future.result()
                except BrokenProcessPool:
                    broken = True
                    summary = {"status": "WORKER_LOST", "failure": {"status": "WORKER_LOST"}}
                except Exception as error:
                    summary = {
                        "status": "ERROR",
                        "failure": {"type": type(error).__name__, "detail": str(error)},
                    }
                if summary["status"] == "WORKER_LOST" and spec["key"] not in self.retried:
                    # One lost process may be unrelated to this job. Run it
                    # once more before counting it against the candidate.
                    self.retried.add(spec["key"])
                    self.lost += 1
                    continue
                self._finish(spec, summary)
            if broken:
                for _, spec in self.inflight.values():
                    self.retried.add(spec["key"])
                    self.lost += 1
                self.inflight.clear()
                if self.restart is None:
                    return "WORKERS_LOST"
                self.submit = self.restart()
        return "TIME_BUDGET"


def build_binding(build, order, budget):
    return digest(
        {
            "policy": POLICY,
            "implementation": IMPLEMENTATION,
            "checkpoint": build.checkpoint_binding,
            "profile": build.planner.profile,
            "order": [row["id"] for row in order],
            "action_budget_s": budget,
            "geometry_first": True,
        }
    )


def _summarize(coordinator, schedule, workers, phases):
    stages = {}
    for job in coordinator.jobs:
        row = stages.setdefault(job["kind"], {"jobs": 0, "seconds": {}, "counters": {}})
        row["jobs"] += 1
        for name, value in job["seconds"].items():
            row["seconds"][name] = row["seconds"].get(name, 0.0) + value
        for group, values in job.get("counters", {}).items():
            totals = row["counters"].setdefault(group, {})
            for name, value in values.items():
                totals[name] = totals.get(name, 0) + value
    peak = {}
    for job in coordinator.jobs:
        if job["worker"] is not None:
            row = peak.setdefault(job["worker"], {"rss_mb": 0.0, "pss_mb": 0.0})
            for name in row:
                row[name] = max(row[name], job["memory"].get(name, 0.0))
    selected = [slot["candidate"] for slot in schedule.slots if slot["candidate"] is not None]
    return {
        "policy": POLICY,
        "workers": workers,
        "wall_seconds": phases,
        "job_seconds": stages,
        "serialization_seconds": sum(
            job["seconds"].get("serialization", 0.0) for job in coordinator.jobs
        ),
        "jobs": len(coordinator.jobs),
        "outcomes": {
            status: sum(job["status"] == status for job in coordinator.jobs)
            for status in sorted({job["status"] for job in coordinator.jobs})
        },
        "stale_results": schedule.stale,
        "lost_worker_jobs": coordinator.lost,
        "resumed_job_results": coordinator.resumed,
        "selected": len(selected),
        "worker_peak_memory": peak,
        "worker_peak_pss_total_mb": sum(row["pss_mb"] for row in peak.values()),
        "coordinator_memory": _memory(),
        "actions": schedule.report(),
    }


_SCREENED = None


def _screen_prefix(ids):
    _SCREENED.stable(ids)
    return _SCREENED.structure[ids]


def _prescreen(build, workers):
    """Screen the expected order's material prefixes on the workers.

    The order search asks for one structural screen per step, each independent
    of the others. This only fills the build's cache with the screens the
    search will request if every step passes; the search itself is unchanged
    and computes any other subset it turns out to need.
    """
    from .regression import greedy_forward_order

    global _SCREENED
    expected = greedy_forward_order(
        build.actions.values(), build.planner.ground, lambda ids: True
    )
    if not expected:
        return
    built, prefixes = set(), []
    for row in expected[:-1]:
        built.add(row["id"])
        if frozenset(built) not in build.structure:
            prefixes.append(frozenset(built))
    _SCREENED = build
    try:
        with ProcessPoolExecutor(max_workers=workers, mp_context=get_context("fork")) as pool:
            for ids, screen in zip(
                prefixes, pool.map(_screen_prefix, prefixes, chunksize=2), strict=True
            ):
                build.structure[ids] = screen
    finally:
        _SCREENED = None


def prepare_context(build, action_budget=240.0, workers=1):
    """Fix the order, freeze the library and publish the state workers inherit."""
    from .indexed_build import IndexedMotions
    from .regression import greedy_forward_order

    global _CONTEXT
    if workers > 1:
        _prescreen(build, workers)
    order = greedy_forward_order(
        build.actions.values(), build.planner.ground, build.stable, deadline=build.planner.deadline
    )
    if order is None:
        return None
    write_json(
        build.run / "forward_order.json",
        {
            "binding": build.checkpoint_binding,
            "order": [row["id"] for row in order],
            "scope": "One support/stiffness-guided order; parallel workers replace motion "
            "candidates within it, but do not search alternate construction orders.",
        },
    )
    # The order search screened the structure after every action. Save those
    # verdicts so verification need not solve the same frames again.
    from ..io import sha256 as file_sha256
    from ..structure import screen_key

    graph_sha256, members, screens = file_sha256(build.run / "stroke_graph.json"), set(), {}
    built = set()
    for row in order:
        built.add(row["id"])
        members.update(row["members"])
        screen = build.structure.get(frozenset(built))
        if screen is not None and screen["modeled_pass"]:
            screens[screen_key(graph_sha256, build.material, members)] = {
                name: value for name, value in screen.items() if name not in {"nodes", "edges"}
            }
    write_json(build.run / "structural_screens.json", {"screens": screens})
    model = IndexedMotions(build)
    model.round = max(model.round, 3)
    root = build.run / "parallel"
    _CONTEXT = {
        "build": build,
        "model": model,
        "order": order,
        "root": root,
        "budget": action_budget,
        "deadline": build.planner.deadline,
        "binding": build_binding(build, order, action_budget),
    }
    return _CONTEXT


def run_parallel_forward(
    build, workers, *, action_budget=240.0, inject_rejections=(), stage="all"
):
    """Plan one supported order with parallel workers, then export playback.

    ``stage`` "geometry" stops after the untimed paths and saves them in
    ``geometry_plan.json``; "timing" times those saved paths and exports.

    ``inject_rejections`` is a test facility: the first accepted check of each
    listed action index is treated as rejected, forcing a real replacement
    candidate and the invalidation of everything computed from the old one.
    """
    from .full_build_export import export_sections

    if workers < 1:
        raise ValueError("Parallel planning needs at least one worker")
    started = launched = perf_counter()
    phases = {}
    build.report("PARALLEL_FORWARD_ORDER_RUNNING")
    context = prepare_context(build, action_budget, workers)
    if context is None:
        build.report(
            "SEARCH_TIME_BUDGET"
            if monotonic() >= build.planner.deadline
            else "NO_GREEDY_FORWARD_ORDER"
        )
        return None
    order, root = context["order"], context["root"]
    schedule = ForwardSchedule(
        [row["id"] for row in order],
        context["binding"],
        rows=[
            {
                "start": row["start"],
                "end": row["end"],
                "closed": bool(row.get("closed")),
                "nodes": sorted(
                    {visit["node"] for visit in row.get("junction_visits", [])}
                    | {row["start"], row["end"]}
                ),
            }
            for row in order
        ]
        if context["build"].planner.settings.get("chain_strokes")
        else None,
    )
    pending_injections = set(inject_rejections)

    def intercept(spec, summary):
        if (
            spec["kind"] == "check"
            and spec["index"] in pending_injections
            and summary["status"] == "OK"
        ):
            pending_injections.discard(spec["index"])
            return {
                **summary,
                "status": "REJECTED",
                "failure": {"status": "INJECTED_TEST_REJECTION"},
            }
        return summary

    phases["setup"] = perf_counter() - started
    build.search_metadata = {
        "strategy": "parallel_local_actions_in_greedy_supported_order",
        "material": "predicted_preceding_deposition_enclosures",
        "workers": workers,
    }
    pools = []

    def start_pool():
        pool = ProcessPoolExecutor(max_workers=workers, mp_context=get_context("fork"))
        pools.append(pool)
        return lambda spec: pool.submit(run_job, spec)

    def submit(spec):
        # Forked on first use, and again after the replay released the workers.
        if not pools or pools[-1] is None:
            start_pool()
        return pools[-1].submit(run_job, spec)

    def release_pool():
        if pools and pools[-1] is not None:
            pools[-1].shutdown(wait=True, cancel_futures=True)
            pools.append(None)

    def kill_pool():
        for process in list(getattr(pools[-1], "_processes", {}).values()):
            process.kill()

    published = [-np.inf]

    def progress(coordinator, final=False):
        # The report grows with the plan (40 MB for 366 actions); writing it
        # after every job kept the coordinator busier than the workers.
        if not final and perf_counter() - published[0] < PROGRESS_INTERVAL_S:
            return
        published[0] = perf_counter()
        build.best = [
            schedule.actions[index]
            for index in range(len(schedule.actions))
            if schedule._checked(index) is not None
        ]
        build.report(
            "PARALLEL_FORWARD_RUNNING",
            parallel={"workers": workers, "jobs": len(coordinator.jobs), "stale": schedule.stale},
        )
        write_json(
            root / "progress.json",
            {
                "elapsed_s": perf_counter() - launched,
                "checked": len(build.best),
                "required": len(order),
                "in_flight": [
                    [spec["kind"], spec["index"]] for _, spec in coordinator.inflight.values()
                ],
                "recent_jobs": coordinator.jobs[-12:],
                "actions": schedule.report(),
            },
        )

    coordinator = Coordinator(
        schedule,
        submit,
        workers,
        root,
        deadline=build.planner.deadline,
        restart=start_pool,
        progress=progress,
        stall_seconds=3 * action_budget + 120,
        kill=kill_pool,
        intercept=intercept if inject_rejections else None,
    )
    exported, timing_rejections = None, []
    try:
        timed_keys = {}
        geometry_file = build.run / "geometry_plan.json"
        if stage == "timing":
            # The geometry stage planned the paths; time them.
            saved = read_json(geometry_file)
            if saved["status"] != "COMPLETE" or saved["binding"] != context["binding"]:
                raise ValueError("The paths were planned for other inputs; plan the geometry again")
            planned, returned_key, lift_offs = (
                saved["actions"], saved["returned_key"], saved["lift_offs"]
            )
            status = "COMPLETE"
        else:
            started = perf_counter()
            status = coordinator.run()
            progress(coordinator, final=True)
            phases["planning_0"] = perf_counter() - started
            lift_offs, planned, returned_key = schedule.lift_offs(), None, None
            if status == "COMPLETE":
                planned = [
                    {
                        "connect_key": schedule._connected(index)["key"],
                        "untimed": bool(schedule._connected(index).get("untimed")),
                        # The wire this action's pen was checked against, and its own.
                        "chain": schedule.chain(index),
                        "deposit": [
                            schedule.slots[index]["candidate"]["deposit_key"],
                            schedule.slots[index]["candidate"]["table_index"],
                        ],
                    }
                    for index in range(len(order))
                ]
                returned_key = schedule.returned["key"]
            if stage == "geometry":
                write_json(
                    geometry_file,
                    {
                        "status": status,
                        "binding": context["binding"],
                        "order": [row["id"] for row in order],
                        "actions": planned,
                        "returned_key": returned_key,
                        "lift_offs": lift_offs,
                        "minimum_trails": _minimum_trails(build),
                    },
                )
                if status == "COMPLETE":
                    status = "GEOMETRY_PLANNED"
        if status == "COMPLETE":
            # The whole object now has a checked chain. Time its untimed
            # actions; each is independent of the others.
            started = perf_counter()
            futures = {}
            for index, action in enumerate(planned):
                if not action["untimed"]:
                    continue
                spec = {
                    "kind": "time",
                    "index": index,
                    "connected": action["connect_key"],
                    "chain": action["chain"],
                    "deposit": action["deposit"],
                    "key": digest({"time": action["connect_key"], "binding": context["binding"]}),
                }
                futures[submit(spec)] = spec
            for future, spec in futures.items():
                summary = future.result()
                coordinator.jobs.append(
                    {
                        "kind": "time",
                        "index": spec["index"],
                        "status": summary["status"],
                        "current": True,
                        "failure": summary.get("failure"),
                        "seconds": summary.get("seconds", {}),
                        "counters": summary.get("counters", {}),
                        "worker": summary.get("worker"),
                        "memory": summary.get("memory", {}),
                    }
                )
                if summary["status"] == "OK":
                    timed_keys[spec["index"]] = spec["key"]
                else:
                    status = "TIMING_REJECTED"
                    timing_rejections.append(
                        {
                            "action": order[spec["index"]]["id"],
                            "status": summary["status"],
                            "failure": summary.get("failure"),
                        }
                    )
            phases["timing_0"] = perf_counter() - started
        if status == "COMPLETE":
            started = perf_counter()
            witnesses = []
            for index, row in enumerate(order):
                payload = _load(root, timed_keys.get(index, planned[index]["connect_key"]))
                clock, drawing = payload["clock"], payload["drawing"]
                witnesses.append(
                    (
                        row["id"],
                        MotionProposal(
                            row["id"],
                            build.arm.identity,
                            drawing["variant"],
                            clock.duration_s,
                            drawing["start_anchor"],
                            tuple(clock.edges[-1].q1),
                            clock,
                        ),
                        drawing,
                    )
                )
            returned = _load(root, returned_key)["returned"]
            phases["assembly_0"] = perf_counter() - started
            # Finished workers no longer need their library/material copies.
            release_pool()
            # Selected seeds remain available in the frozen originals.
            import gc

            _WORKER.clear()
            gc.collect()
            coordinator.submit = submit
            started = perf_counter()
            build.lift_offs = lift_offs
            build.minimum_trails = _minimum_trails(build)
            exported = export_sections(
                build,
                witnesses,
                returned,
                release_witnesses=True,
            )
            phases["export_0"] = perf_counter() - started
    finally:
        for pool in pools:
            if pool is not None:
                pool.shutdown(wait=False, cancel_futures=True)
    phases["total"] = sum(phases.values())
    summary = _summarize(coordinator, schedule, workers, phases)
    summary["injected_test_rejections"] = sorted(inject_rejections)
    summary["planning_status"] = status
    summary["replay_status"] = "NOT_RUN"
    summary["export_status"] = None if exported is None else exported["status"]
    summary["blocked"] = schedule.blocked()
    summary["timing_rejections"] = timing_rejections
    summary["lift_offs"] = lift_offs
    summary["minimum_trails"] = _minimum_trails(build)
    write_json(build.run / "parallel_build.json", summary)
    compact = {
        key: summary[key]
        for key in ("workers", "wall_seconds", "jobs", "stale_results", "selected", "blocked")
    }
    if status == "COMPLETE":
        report = read_json(build.run / "section_build.json")
        report["parallel"] = compact
        report["lift_offs"] = summary["lift_offs"]
        report["minimum_trails"] = summary["minimum_trails"]
        write_json(build.run / "section_build.json", report)
    if status != "COMPLETE":
        build.report(
            {
                "GEOMETRY_PLANNED": "GEOMETRY_PLANNED",
                "TIME_BUDGET": "SEARCH_TIME_BUDGET",
                "TIMING_REJECTED": "PARALLEL_FORWARD_TIMING_REJECTED",
                "BLOCKED": "PARALLEL_FORWARD_INCOMPLETE",
                "WORKERS_LOST": "PARALLEL_FORWARD_WORKERS_LOST",
            }[status],
            parallel=compact,
            lift_offs=summary["lift_offs"],
            minimum_trails=summary["minimum_trails"],
        )
    return summary


def _minimum_trails(build):
    """Fewest pen-down trails the frame allows (Euler's rule), as generated."""
    return build.planner.graph.get("generation", {}).get("eulerization", {}).get(
        "minimum_trails_after"
    )
