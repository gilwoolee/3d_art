"""Replay a saved plan with the complete robot model, from a clean start."""

import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from multiprocessing import get_context
from pathlib import Path
from shutil import rmtree
from time import monotonic, perf_counter

import numpy as np

from ..curves import source_points, transform_points
from ..io import artifact, read_json, sha256, stage_update, write_json
from .cell_planner import CellPlanner
from .clock_path import (
    POSTURE_LIMIT_DEG_PER_MM,
    ClockPath,
    check_clock_limits,
    posture_change,
)
from .material_journal import JournalFolder, _merged, deposit_key
from .path_certificates import digest
from .process_clock import add_ready_times, audit_process_clock, feed_events
from .scene import MaterialScene
from .schedule import MotionProposal, ScheduledMotion, ScheduleState
from .sections import audit_source_provenance, coverage_audit, replay_sections
from .smooth_audit import audit_source_curve

PLAN = "plan.json"
VERIFICATION = "verification.json"
# Smallest parallel replay job, in clock edges, and the number of jobs to aim
# for per worker. Smaller chunks balance load; each still pays for a copy of
# the material preceding it.
PARALLEL_CHUNK_EDGES = 256
PARALLEL_JOBS_PER_WORKER = 8
# Inherited by forked replay workers; never pickled.
_PARALLEL = None
_LOCAL = {}


# --------------------------------------------------------------------------
# Worker side. Each job reads what the parent prepared in ``_PARALLEL``.


def _journals():
    """This process's deposit journals of the replay."""
    if "journals" not in _LOCAL:
        _LOCAL["journals"] = JournalFolder(_PARALLEL["root"])
    return _LOCAL["journals"]


def _parallel_audit(index):
    row = _PARALLEL["rows"][index]
    path = ClockPath.from_record(row["trajectory"])
    return path, audit_action(_PARALLEL["planner"], row, path)


def _parallel_structural(index):
    shared = _PARALLEL
    return structural_screen(
        shared["planner"].graph, shared["material"], shared["sections"], shared["deposited"], index
    )


def _parallel_deposit(index):
    """Journal one interval's deposits, without any collision check."""
    planner = _PARALLEL["planner"]
    _, b, new = _PARALLEL["intervals"][index]
    operations = []
    # A rejected interval is reported by its own full check. Later intervals
    # are never accepted past it, so a partial journal cannot be relied upon.
    planner.validate_interval(
        replace(_PARALLEL["states"][index], payload=MaterialScene()),
        new,
        b,
        deposit_only=True,
        journal=operations,
    )
    return _journals().keep(_merged(planner, operations))


def _parallel_check(index, chain):
    """Fully check one interval against material rebuilt from earlier journals."""
    planner = _PARALLEL["planner"]
    journals = _journals()
    _, b, new = _PARALLEL["intervals"][index]
    scene = journals.prefix.scene_for(chain)
    before = (planner.checked_intervals, Counter(planner.verification_stats))
    seconds = Counter(planner.verification_seconds)
    operations = []
    _, failure = planner.validate_interval(
        replace(_PARALLEL["states"][index], payload=scene), new, b, journal=operations
    )
    return {
        "failure": failure,
        # What this full check deposited, in the form later checks receive.
        "deposited": deposit_key(_merged(planner, operations)),
        "checked_intervals": planner.checked_intervals - before[0],
        "counts": Counter(planner.verification_stats) - before[1],
        "seconds": Counter(planner.verification_seconds) - seconds,
    }


def _parallel_arm_check(index):
    """Check the arm, not the pen, on an interval whose pen holds planning's proof.

    Every edge is checked continuously with the full robot against itself, and
    its arm links against the cell and one volume enclosing all printed wire.
    The tool is left out of the obstacle tests only: its proof covers them.
    """
    from .collision import QUERY_STATS, ArmLinksModel

    planner = _PARALLEL["planner"]
    a, b, new = _PARALLEL["intervals"][index]
    obstacles = {**planner.obstacles, "printed_volume": _PARALLEL["envelope"]}
    depth, edges = planner.settings["maximum_subdivision_depth"], 0
    if "arm_links" not in _LOCAL:
        arm = next(iter(planner.arms.values()))
        _LOCAL["arm_links"] = ArmLinksModel(arm.collision, {body.name for body in arm.tool.bodies})
    model = _LOCAL["arm_links"]
    before, started = Counter(QUERY_STATS), perf_counter()
    for motion in _PARALLEL["states"][index].active + new:
        for edge in motion.proposal.payload.edges:
            if not a - 1e-9 <= motion.start_s + edge.start_s < b - 1e-9:
                continue
            if edge.coefficients is None:
                check = model.check_edge(
                    edge.q0, edge.q1, obstacles, planner.margin, depth, None, planner.deadline
                )
            else:
                piece = edge.piece()
                check = model.check_bounded_path(
                    piece.position,
                    piece.deviation,
                    obstacles,
                    planner.margin,
                    depth,
                    None,
                    planner.deadline,
                )
            if check["status"] != "CERTIFIED_MODELED_EDGE":
                return {"certified": False, "detail": check}
            edges += 1
    return {
        "certified": True,
        "edges": edges,
        "counts": Counter(QUERY_STATS) - before,
        "seconds": perf_counter() - started,
    }


def _fork(workers, pools):
    """A pool of forked workers; they inherit ``_PARALLEL`` as it is now."""
    pool = ProcessPoolExecutor(max_workers=workers, mp_context=get_context("fork"))
    pools.append(pool)
    return pool


# --------------------------------------------------------------------------
# The phases of a replay, in order.


def load_plan(run, profile_path, maximum_seconds):
    """The saved plan bound to its inputs, a planner restoring the complete
    robot model and the plan's settings, and the arm's home position."""
    saved = read_json(run / PLAN)
    for name, value in saved["inputs"].items():
        if sha256(run / name) != value:
            raise ValueError(f"Stale plan input: {name}")
    settings = read_json(run / saved["settings_artifact"])
    # Planning may deliberately defer robot collision checks. The replay
    # always restores the complete model.
    planner = CellPlanner(
        run,
        profile_path,
        {**settings["search"], "collision_scope": "full_robot"},
        monotonic() + maximum_seconds,
        timing_override=settings["timing"],
        verification_raw=True,
        optimize_contact_geometry=True,
    )
    planner.transit, planner.timing, planner.process = (
        settings[k] for k in ("transit", "timing", "process")
    )
    initial = saved["initial_positions"]
    if set(initial) != set(planner.arms) or len(initial) != 1:
        raise ValueError("Saved initial poses do not match the one-arm cell")
    (home,) = (tuple(q) for q in initial.values())
    return saved, planner, home


def audit_action(planner, row, path):
    """Every saved-clock audit that needs no other action. Raises on violation.

    Speed, source-coverage and pen proofs planning wrote for these exact
    clocks are accepted; they bind the path, kinematics, source and process
    settings. Collision, orientation, deposits and structure are still checked.
    """
    arm = planner.arms[row["arm"]]
    store = planner.path_certificates
    result = {"pen_proof": None}
    if not row.get("auxiliary", False):
        # Looked up here, on the workers: binding a proof digests the whole clock.
        result["pen_proof"] = store.get(
            planner.path_certificate_context(row["arm"], pen=True),
            row["trajectory"],
            "pen_material_path",
        )
        action = planner.actions[row["stroke_id"]]
        transform = np.asarray(planner.profile["world_from_build"])
        coverage_context = {
            "invariants": planner.path_certificate_context(row["arm"], kinematic=True),
            "source_points_m": source_points(action, transform),
        }
        coverage = store.get(coverage_context, row["trajectory"], "source_coverage")
        if coverage is None:
            coverage = audit_source_curve(arm, [path], coverage_context["source_points_m"])
            if coverage["status"] == "BOUNDED":
                store.admit(coverage_context, row["trajectory"], "source_coverage", coverage)
        if coverage["status"] != "BOUNDED":
            raise ValueError("Drawing misses its source curve")
        if path.metadata.get("source_reverse", False) != row.get(
            "reverse", False
        ) or path.metadata.get("source_start_visit", 0) != row.get("start_visit", 0):
            raise ValueError("Saved clock traversal differs from its source action")
        audit_process_clock(
            path,
            action,
            planner.process,
            planner.settings["assumed_cooling_s"],
            transform,
            require_release=True,
        )
    swing = posture_change(path, arm)
    if swing:
        raise ValueError(f"Arm changes posture while the pen works: {swing}")
    result["reuse"] = check_clock_limits(planner, arm, path, row["trajectory"])
    return result


def audit_sections(planner, saved):
    """The saved drawing rows, bound to the construction source, and their
    coverage, provenance and stage audits. Raises on a mismatch."""
    sections = [row for row in saved["actions"] if not row.get("auxiliary", False)]
    for row in sections:
        expected = planner.actions.get(row["stroke_id"])
        if expected is None or any(
            row[key] != expected[key]
            for key in ("id", "parent_curve", "source_interval", "members", "stage", "start", "end")
        ):
            raise ValueError("Saved section provenance differs from construction source")
    # Join locations come from the bound source, never untrusted annotations
    # in the saved schedule.
    for row in sections:
        expected = planner.actions[row["stroke_id"]]
        for key in ("junction_visits", "closed", "source_intervals"):
            if key in row and row[key] != expected.get(key):
                raise ValueError("Saved section junctions differ from construction source")
            if key in expected:
                row[key] = expected[key]
    audits = {
        "source_coverage": coverage_audit(sections, list(planner.actions.values())),
        "source_provenance": audit_source_provenance(sections, planner.graph),
        "stage_audit": replay_sections(
            sections, planner.graph, cooling_s=planner.settings["assumed_cooling_s"]
        ),
    }
    return sections, audits


def members_before(sections):
    """The members built before each section."""
    deposited, members = [], set()
    for row in sections:
        deposited.append(frozenset(members))
        members.update(row["members"])
    return deposited


def structural_screen(graph, material, sections, deposited, index):
    """The structure once section ``index`` is built.

    Each section's release state depends only on the ordered members before
    it, never on another section's verdict.
    """
    from ..structure import analyze_structure

    row = sections[index]
    screen = analyze_structure(graph, material, edge_ids=deposited[index] | set(row["members"]))
    # Keep the deformed shape of the finished object only. Every earlier
    # release keeps its verdict and extremes; their per-node and
    # per-member arrays made the report over a gigabyte on a large build.
    if index < len(sections) - 1:
        screen = {k: v for k, v in screen.items() if k not in {"nodes", "edges"}}
    return screen


def saved_screens(run, material, sections, deposited):
    """Planning's structural verdicts this replay accepts, by section index.

    Planning screened the same frames to choose the build order. A saved
    verdict counts when it names this graph, this material and exactly the
    members built so far; the finished object is always solved here.
    """
    from ..structure import screen_key

    accepted = {}
    if (run / "structural_screens.json").exists():
        screens = read_json(run / "structural_screens.json")["screens"]
        graph_sha256 = sha256(run / "stroke_graph.json")
        for index, row in enumerate(sections[:-1]):
            key = screen_key(graph_sha256, material, deposited[index] | set(row["members"]))
            if key in screens:
                accepted[index] = {**screens[key], "accepted_from_planning": True}
    return accepted


def structural_releases(pool, sections, accepted):
    """Every section's release state, solving on the workers those planning did
    not screen. Raises on a rejected release."""
    pending = [index for index in range(len(sections)) if index not in accepted]
    solved = dict(
        zip(
            pending,
            [future.result() for future in [pool.submit(_parallel_structural, i) for i in pending]],
            strict=True,
        )
    )
    releases = []
    for index, row in enumerate(sections):
        screen = accepted.get(index) or solved[index]
        if not screen["modeled_pass"]:
            raise ValueError(f"Structural release rejected after {row['id']}: {screen}")
        releases.append({"section": row["id"], "screen": screen})
    return releases


def start_report(saved, plan_digest, horizon, tasks, section_audits):
    """The verification report before any path is checked."""
    return artifact(
        "verification",
        status="RUNNING",
        phase="PATH_CHECKS",
        simulation_only=True,
        executable=False,
        source_plan_sha256=plan_digest,
        inputs={PLAN: plan_digest},
        horizon_s=horizon,
        requested_strokes=len(tasks),
        completed_strokes=0,
        checked_intervals=0,
        checked_actions=0,
        reused_path_checks=0,
        posture_limit_deg_per_mm=POSTURE_LIMIT_DEG_PER_MM,
        pen_proof_intervals=0,
        requested_actions=len(saved["actions"]),
        checked_edges=0,
        requested_edges=sum(len(row["trajectory"]["edges"]) for row in saved["actions"]),
        failures=[],
        **section_audits,
        scope=[
            "Saved joint paths and clocks, joint limits, velocity and acceleration",
            "The whole arm against itself, static obstacles and the growing wire",
            "Start-anchor readiness and final return",
        ],
        limitations=saved["limitations"],
    )


class Report:
    """The verification report as the replay publishes it.

    Progress goes out at most every five seconds and without the structural
    release states, which can be tens of megabytes and belong only in the
    finished report.
    """

    def __init__(self, run, output):
        self.path, self.output, self.published = run / VERIFICATION, output, -np.inf

    def save(self, final=False):
        if not final and monotonic() - self.published < 5:
            return
        self.published = monotonic()
        output = self.output
        write_json(
            self.path,
            output if final else {k: v for k, v in output.items() if k != "release_states"},
        )

    def out_of_time(self):
        """Close the report on the replay's time budget."""
        self.output.update(
            status="REPLAY_TIME_BUDGET",
            failures=[{"status": "TIME_BUDGET", "phase": self.output["phase"]}],
        )
        self.save(final=True)
        return self.output


def audit_actions(pool, ordered):
    """Every saved clock parsed and audited on the workers, or None when time runs out.

    The parsed clock returns with its audit, so the parent neither reparses
    nor trusts raw rows.
    """
    try:
        return [
            future.result()
            for future in [pool.submit(_parallel_audit, i) for i in range(len(ordered))]
        ]
    except TimeoutError:
        return None


def arm_motions(planner, ordered, parsed, home, report):
    """The saved actions as the arm's motions, in order.

    Each must last as long as its clock, start no earlier than the one before
    ends and start where that one stopped. Raises on violation. Returns the
    motions and each drawing's pen proof, or None when time runs out.
    """
    (identity,) = planner.arms
    output = report.output
    moves, pen_proofs = [], {}
    previous_end, previous_q = 0.0, home
    for row, (path, audit) in zip(ordered, parsed, strict=True):
        if monotonic() >= planner.deadline:
            return None
        start, end = row["start_s"], row["end_s"]
        if (
            not np.isfinite([start, end]).all()
            or start < 0
            or abs(end - start - path.duration_s) > 1e-8
        ):
            raise ValueError("Saved action times do not match its trajectory")
        if (
            row["arm"] != identity
            or start < previous_end - 1e-9
            or not np.allclose(previous_q, path.edges[0].q0, atol=1e-8, rtol=0)
        ):
            raise ValueError("Overlapping or discontinuous arm channel")
        if not row.get("auxiliary", False):
            pen_proofs[row["stroke_id"]] = audit["pen_proof"]
        output["reused_path_checks"] += audit["reuse"]
        output["checked_edges"] += len(path.edges)
        report.save()
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
        previous_end, previous_q = end, proposal.end_configuration
        output["checked_actions"] += 1
    return moves, pen_proofs


def check_chaining(drawings):
    """A chained stroke takes over from its predecessor with the pen down: the
    two must name each other, touch in time and meet at one joint position."""
    in_order = sorted(drawings, key=lambda motion: motion.start_s)
    for first, second in zip(in_order[:-1], in_order[1:], strict=True):
        leaves = bool(first.proposal.payload.metadata.get("chained_out"))
        if leaves != bool(second.proposal.payload.metadata.get("chained_in")):
            raise ValueError("Chained strokes do not agree on their join")
        if leaves and (
            abs(first.end_s - second.start_s) > 1e-9
            or not np.allclose(
                first.proposal.payload.edges[-1].q1,
                second.proposal.payload.edges[0].q0,
                atol=1e-9,
                rtol=0,
            )
        ):
            raise ValueError("Chained strokes do not meet in time and joint position")
    if in_order and (
        in_order[0].proposal.payload.metadata.get("chained_in")
        or in_order[-1].proposal.payload.metadata.get("chained_out")
    ):
        raise ValueError("A chained stroke has no neighbour to chain with")


def check_drawings(planner, saved, moves, tasks):
    """Each stroke drawn once, the feed commands matching the clocks, chained
    strokes meeting, and every other stroke starting on cooled material of its
    own. Raises on violation; returns the drawing motions."""
    drawings = [m for m in moves if m.proposal.task in tasks]
    if len({m.proposal.task for m in drawings}) != len(drawings):
        raise ValueError("A stroke is scheduled more than once")
    if saved.get("events") != feed_events(saved["actions"]):
        raise ValueError("Saved feed commands do not match the path clocks")
    ready = {node: 0 for node in planner.ground}
    for motion in drawings:
        add_ready_times(ready, motion.proposal.payload, motion.start_s)
    check_chaining(drawings)
    # A chained stroke does not wait for its joint to cool: nothing is re-attached.
    for motion in drawings:
        chained = bool(motion.proposal.payload.metadata.get("chained_in"))
        available = ready.get(motion.proposal.start_anchor, float("inf"))
        if not np.isfinite(available) or (not chained and available > motion.start_s + 1e-9):
            raise ValueError("A stroke starts on future or still-cooling material")
        if motion.proposal.start_anchor not in tasks[motion.proposal.task].anchors:
            raise ValueError("Saved start anchor does not belong to its stroke")
    return drawings


def interval_schedule(moves, horizon, workers):
    """The scheduling intervals ``(start, end, motions starting there)``.

    They are cut at every action's start and end, and long actions also at
    their own edge boundaries. The validator already checks an action edge by
    edge, so these chunks are the same checks and the same deposits in the
    same order, only on several workers. Every job copies and re-indexes its
    preceding material, so aim for a few jobs per worker rather than the
    smallest possible chunk.
    """
    events = sorted(
        {0.0, horizon, *[t for m in moves for t in (m.start_s, m.end_s) if 0 < t < horizon]}
    )
    total = sum(len(move.proposal.payload.edges) for move in moves)
    step = max(PARALLEL_CHUNK_EDGES, total // (workers * PARALLEL_JOBS_PER_WORKER))
    intervals = []
    for a, b in zip(events[:-1], events[1:], strict=True):
        new = tuple(m for m in moves if abs(m.start_s - a) < 1e-9)
        cuts = []
        if len(new) == 1:
            ends = [new[0].start_s + edge.end_s for edge in new[0].proposal.payload.edges]
            cuts = [t for t in ends[step - 1 :: step] if a < t < b]
        for first, last in zip([a, *cuts], [*cuts, b], strict=True):
            intervals.append((first, last, new if first == a else ()))
    return intervals


def advance(state, new, end, tasks):
    """The arm's state after an interval; independent of the collision work."""
    active = state.active + new
    finished = [m for m in active if m.end_s <= end + 1e-9]
    return replace(
        state,
        time_s=end,
        configuration=finished[-1].proposal.end_configuration if finished else state.configuration,
        completed=state.completed | {m.proposal.task for m in finished if m.proposal.task in tasks},
        active=tuple(m for m in active if m.end_s > end + 1e-9),
    )


def interval_states(intervals, home, tasks):
    """The state each interval starts from, then the state after the last."""
    states = [ScheduleState(0, home, frozenset())]
    for _, b, new in intervals:
        states.append(advance(states[-1], new, b, tasks))
    return states


def pen_proven(moves, pen_proofs):
    """The drawings whose pen check planning's proof covers.

    Planning proved each action's pen clear of the cell and of the wire of the
    actions before it. Accept that for the leading actions whose saved clock
    holds such a proof and whose chain of preceding deposits is exactly the
    one the earlier accepted proofs name.
    """
    proven, chain = set(), []
    for move in sorted(moves, key=lambda move: move.start_s):
        if move.proposal.task not in pen_proofs:
            continue  # A transit between actions; always checked in full.
        proof = pen_proofs[move.proposal.task]
        if proof is None or proof.get("chain_sha256") != digest(chain):
            break
        proven.add(move.proposal.task)
        chain.append(proof["deposit"])
    return proven


def locate(failure, motions, drawings):
    """Name the drawing action a rejected interval belongs to."""
    order = [m.proposal.task for m in drawings]
    for motion in motions:
        if motion.proposal.task in order:
            failure = {
                **failure,
                "stroke_id": motion.proposal.task,
                "action_index": order.index(motion.proposal.task),
            }
    return failure


def _printed_volume(planner):
    """One upright cylinder about the build axis holding every wire of the object."""
    import fcl

    from .collision import placed

    points = np.array([node["position"] for node in planner.graph["nodes"]])
    radius = max(edge["diameter_m"] for edge in planner.graph["edges"]) / 2 + 0.001
    transform = np.asarray(planner.profile["world_from_build"])
    axis = transform[:3, 3]
    world = transform_points(points, transform)
    reach = float(np.linalg.norm(world[:, :2] - axis[:2], axis=1).max()) + radius
    low, high = world[:, 2].min() - radius, world[:, 2].max() + radius
    pose = np.eye(4)
    pose[:3, 3] = [axis[0], axis[1], (low + high) / 2]
    return placed(fcl.Cylinder(reach, float(high - low)), pose)


def collision_replay(run, planner, intervals, states, accepted, drawings, workers, pools, report):
    """Check every interval on the workers, in order of the material it adds.

    Each interval is checked by one worker against material rebuilt from
    deposit journals; the hash chain proves every rebuilt prefix is exactly
    the material its predecessor's full check produced. An interval whose pen
    holds planning's proof (``accepted``) checks the arm only. Returns the
    final material, or None after a rejection.
    """
    output = report.output
    journal_root = run / "verification_journals"
    rmtree(journal_root, ignore_errors=True)
    journal_root.mkdir(parents=True)
    _PARALLEL.update(
        envelope=_printed_volume(planner) if any(accepted) else None,
        intervals=intervals,
        states=states,
        root=journal_root,
    )
    # A second fork, so workers inherit the interval states computed above.
    pool = _fork(workers, pools)
    try:
        keys = [
            future.result()
            for future in [pool.submit(_parallel_deposit, k) for k in range(len(intervals))]
        ]
        checks = [
            pool.submit(_parallel_arm_check, k)
            if accepted[k]
            else pool.submit(_parallel_check, k, keys[:k])
            for k in range(len(intervals))
        ]
        for k, future in enumerate(checks):
            result = future.result()
            running = states[k].active + intervals[k][2]
            if accepted[k] and result["certified"]:
                # The arm is clear and the pen holds its proof; the material
                # this interval adds is the journal later checks receive.
                planner.checked_intervals += result["edges"]
                output["pen_proof_intervals"] += 1
                planner.verification_seconds["arm_check"] += result["seconds"]
                for name in ("path_calls", "path_evaluations", "reference_certified"):
                    planner.verification_stats[f"arm_{name}"] += result["counts"][name]
            elif accepted[k]:
                # The arm must stay out of the volume the object occupies and
                # clear of itself and the cell. Failing that is a rejection.
                output.update(
                    status="REPLAY_REJECTED",
                    failures=[
                        locate({"status": "ARM_NOT_CLEAR", **result["detail"]}, running, drawings)
                    ],
                    checked_intervals=planner.checked_intervals,
                )
                return None
            else:
                # Merged journals replace a check's fine wire for its successors,
                # so compare what the check deposited with the journal they use.
                if not result["failure"] and result["deposited"] != keys[k]:
                    raise ValueError("Parallel replay material prefix differs from its check")
                planner.checked_intervals += result["checked_intervals"]
                planner.verification_stats.update(result["counts"])
                planner.verification_seconds.update(result["seconds"])
                if result["failure"]:
                    failure = locate(result["failure"], running, drawings)
                    output.update(
                        status="REPLAY_TIME_BUDGET"
                        if failure.get("status") == "TIME_BUDGET"
                        else "REPLAY_REJECTED",
                        failures=[failure],
                        checked_intervals=planner.checked_intervals,
                    )
                    return None
            output.update(
                completed_strokes=len(states[k + 1].completed),
                checked_intervals=planner.checked_intervals,
            )
            report.save()
        # The material every later check received, not the last check's own.
        whole = _journals().prefix.scene_for(keys)
        return {
            "capsules": len(whole.objects),
            "geometry_sha256": whole.geometry_prefix().hex(),
        }
    finally:
        pool.shutdown(wait=False, cancel_futures=True)
        rmtree(journal_root, ignore_errors=True)


def verdict(planner, report, final_material, final, home, tasks, workers):
    """Close the report. A replay is certified when every interval passed, every
    stroke was drawn and the arm ends parked at home."""
    output = report.output
    output["parallel_workers"] = workers
    if final_material is not None:
        output["final_material"] = final_material
        if (
            len(final.completed) != len(tasks)
            or final.active
            or not np.allclose(final.configuration, home, atol=1e-8, rtol=0)
        ):
            output.update(
                status="REPLAY_REJECTED", failures=[{"status": "INCOMPLETE_OR_NOT_PARKED"}]
            )
        else:
            output["status"] = "CERTIFIED_GEOMETRIC_REPLAY"
    output["collision_scope"] = "full_robot"
    output["verification"] = {
        "counts": dict(planner.verification_stats),
        "seconds": dict(planner.verification_seconds),
    }
    report.save(final=True)
    return output


def _replay(run, profile_path, maximum_seconds, workers, pools):
    """Run the replay's phases in order; returns the verification report."""
    global _PARALLEL
    plan_digest = sha256(run / PLAN)
    saved, planner, home = load_plan(run, profile_path, maximum_seconds)
    tasks = {task.identity: task for task in planner.tasks}
    sections, section_audits = audit_sections(planner, saved)
    material = read_json(run / "fabrication_config.json")["structure"]
    deposited = members_before(sections)
    screens = saved_screens(run, material, sections, deposited)
    section_audits["structural_screens_accepted"] = len(screens)
    ordered = sorted(saved["actions"], key=lambda row: row["start_s"])
    # Fork once everything the audits read exists; workers inherit it all.
    _PARALLEL = {
        "planner": planner,
        "material": material,
        "sections": sections,
        "deposited": deposited,
        "rows": ordered,
    }
    pool = _fork(workers, pools)
    section_audits["release_states"] = structural_releases(pool, sections, screens)
    horizon = saved["modeled_duration_s"]
    if horizon < 0 or not np.isfinite(horizon):
        raise ValueError("Invalid replay horizon")
    report = Report(run, start_report(saved, plan_digest, horizon, tasks, section_audits))
    report.save()
    parsed = audit_actions(pool, ordered)
    sequence = None if parsed is None else arm_motions(planner, ordered, parsed, home, report)
    if sequence is None:
        return report.out_of_time()
    moves, pen_proofs = sequence
    drawings = check_drawings(planner, saved, moves, tasks)
    report.output.update(phase="COLLISION_REPLAY")
    report.save()
    if not moves or horizon == 0:
        report.output["status"] = "NO_MOTION_TO_REPLAY"
        report.save(final=True)
        return report.output
    intervals = interval_schedule(moves, horizon, workers)
    states = interval_states(intervals, home, tasks)
    proven = pen_proven(moves, pen_proofs)
    # The motion running during each interval: one arm, one motion at a time.
    accepted = []
    for state, (_, _, new) in zip(states[:-1], intervals, strict=True):
        running = state.active + new
        accepted.append(len(running) == 1 and running[0].proposal.task in proven)
    pool.shutdown(wait=True, cancel_futures=True)
    final_material = collision_replay(
        run, planner, intervals, states, accepted, drawings, workers, pools, report
    )
    return verdict(planner, report, final_material, states[-1], home, tasks, workers)


def replay_cell(run, profile_path, maximum_seconds=300, *, workers=1):
    """Replay a saved plan with the complete robot model, and record its verdict.

    Each scheduling interval receives the full check on one of ``workers``,
    against material rebuilt from deposit journals; a hash chain ties every
    rebuilt prefix to the checked result of the interval before it.
    """
    run = Path(run)
    stage_update(
        run, "validation", "running", artifact=VERIFICATION, message="Replaying the saved plan"
    )
    pools = []
    try:
        try:
            output = _replay(run, profile_path, maximum_seconds, workers, pools)
        finally:
            for pool in pools:
                pool.shutdown(wait=False, cancel_futures=True)
        passed = output["status"] == "CERTIFIED_GEOMETRIC_REPLAY"
        stage_update(
            run,
            "validation",
            "complete" if passed else "partial",
            artifact=VERIFICATION,
            message=output["status"] + "; modeled geometry only, physical fabrication unverified",
        )
        # Planning only saves clocks. This independent step publishes their
        # verdict; the viewer never runs this verifier or certifies a plan.
        stage_update(
            run,
            "motion",
            "complete" if passed else "partial",
            artifact=PLAN,
            message="Verified playback ready" if passed else "Playback awaiting verification",
        )
        report_path = run / "section_build.json"
        if report_path.exists():
            report = read_json(report_path)
            report.update(
                status=(
                    "FULL_BUILD_VERIFIED"
                    if passed
                    else "FULL_BUILD_REPLAY_REJECTED"
                    if output["status"] == "REPLAY_REJECTED"
                    else "FULL_BUILD_VERIFICATION_INCOMPLETE"
                ),
                replay_status=output["status"],
                collision_scope="full_robot" if passed else report.get("planning_collision_scope"),
                requires_full_robot_validation=not passed,
                final_collision_scope=output.get("collision_scope"),
            )
            write_json(report_path, report)
        return output
    except Exception as error:
        path = run / VERIFICATION
        plan_digest = sha256(run / PLAN)
        try:
            output = read_json(path)
        except (OSError, ValueError):
            output = {}
        if output.get("source_plan_sha256") != plan_digest:
            output = artifact("verification")
        output.update(
            status="REPLAY_ERROR",
            simulation_only=True,
            executable=False,
            source_plan_sha256=plan_digest,
            inputs={PLAN: plan_digest},
            failures=[{"type": type(error).__name__, "message": str(error)}],
        )
        write_json(path, output)
        stage_update(run, "validation", "failed", artifact=VERIFICATION, message=str(error))
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True, help="The cell profile planned with")
    parser.add_argument("--maximum-seconds", type=float, default=300)
    parser.add_argument("--workers", type=int, default=1, help="Parallel interval checks")
    args = parser.parse_args()
    if args.workers < 1:
        parser.error("--workers needs at least one worker")
    result = replay_cell(args.run, args.config, args.maximum_seconds, workers=args.workers)
    print(result["status"], result["completed_strokes"], result["checked_intervals"])
    return 0 if result["status"] == "CERTIFIED_GEOMETRIC_REPLAY" else 1


if __name__ == "__main__":
    raise SystemExit(main())
