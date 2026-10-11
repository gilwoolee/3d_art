"""Bounded event search for one or two arms sharing a growing sculpture.

The motion backend supplies arm-specific alternatives and validates every elapsed
interval (including idle arms, partial deposition and simultaneous starts).
This module never treats scheduling or endpoint IK as collision validation.
"""

from dataclasses import dataclass, field
from itertools import product
from math import isfinite
from time import monotonic


@dataclass(frozen=True)
class StrokeTask:
    identity: str
    anchors: frozenset[str]
    deposits: frozenset[str]
    cooling_s: float = 0.0


@dataclass(frozen=True)
class MotionProposal:
    """A complete travel/approach/draw/withdraw alternative for one arm.

    ``payload`` belongs to the backend (e.g. exact timed joint polynomials).
    ``start_anchor`` must already be cool when dispatched. End configurations
    are used only after the entire action finishes. Different path/orientation
    alternatives must have different variant IDs.
    """

    task: str
    arm: str
    variant: str
    duration_s: float
    start_anchor: str
    end_configuration: tuple[float, ...]
    payload: object = field(default=None, compare=False, repr=False)
    auxiliary: bool = False


@dataclass(frozen=True)
class ScheduledMotion:
    proposal: MotionProposal
    start_s: float

    @property
    def end_s(self):
        return self.start_s + self.proposal.duration_s


@dataclass(frozen=True)
class ScheduleState:
    time_s: float
    configurations: tuple[tuple[str, tuple[float, ...]], ...]
    completed: frozenset[str]
    support_ready: tuple[tuple[str, float], ...]
    active: tuple[ScheduledMotion, ...] = ()
    history: tuple[ScheduledMotion, ...] = ()
    payload: object = field(default=None, compare=False, repr=False)

    def ready_nodes(self):
        return frozenset(node for node, ready in self.support_ready if ready <= self.time_s)

    def idle_arms(self):
        busy = {motion.proposal.arm for motion in self.active}
        return tuple(arm for arm, _ in self.configurations if arm not in busy)


def tasks_from_construction(construction, graph, cooling_s=0.0):
    """Extract alternative anchors, without importing the serial chosen order.

    Readiness is only a necessary geometric precondition. The backend must check
    support/stability along the proposed direction and growing material state.
    """
    tasks = []
    for action in construction["actions"]:
        deposits = frozenset(v["node"] for v in action["junction_visits"])
        # The previous regression plan's alternatives depend on its chosen
        # prefix. Reopen every closed-curve junction and both open endpoints.
        anchors = deposits if action["closed"] else frozenset([action["start"], action["end"]])
        if not anchors or not anchors <= deposits:
            raise ValueError("Every alternative anchor must lie on its stroke")
        tasks.append(StrokeTask(action["id"], anchors, deposits, cooling_s))
    ground = frozenset(node["id"] for node in graph["nodes"] if node["ground"])
    return tuple(tasks), ground


def search_schedule(
    tasks,
    initial_configurations,
    ground_nodes,
    prepare,
    validate_interval,
    *,
    finish,
    complete=None,
    initial_payload=None,
    start_offsets_s=(0.25, 0.5, 1.0),
    maximum_states=2000,
    beam_width=64,
    maximum_dispatch_batches=None,
    maximum_successors_per_state=None,
    completion_first=False,
    deadline=float("inf"),
    estimate_remaining=None,
    observe=None,
    prepare_idle=None,
    initial_state=None,
):
    """Search assignment, overlap, serialization and bounded start offsets.

    ``prepare(state, task, arm)`` returns MotionProposals for an idle arm.
    ``validate_interval(state, new_motions, end_s)`` returns (payload, failure):
    failure=None is acceptance, even when payload=None. It must validate ALL
    moving/waiting arms and material over [state.time_s, end_s], with the supplied
    new motions starting simultaneously. Only this check admits a successor.
    ``complete(payload, finished_motions)`` optionally updates backend state.
    ``finish(state)`` must validate/plan final parking and any added intervals;
    it returns (final_state, failure), including the return time in final_state.

    Candidate generation and callbacks must obey the deadline and not mutate
    their input states. A FOUND result need not be optimal: beam/state/time
    limits are reported separately. No geometry-only dominance cache is used.
    """
    tasks = tuple(tasks)
    by_id = {task.identity: task for task in tasks}
    configurations = tuple((arm, tuple(q)) for arm, q in initial_configurations.items())
    offsets = tuple(sorted(set(start_offsets_s)))
    if len(by_id) != len(tasks) or not all(task.anchors for task in tasks):
        raise ValueError("Tasks need unique IDs and nonempty alternative anchors")
    if len(configurations) not in (1, 2) or any(
        not arm or not q or not all(isfinite(value) for value in q) for arm, q in configurations
    ):
        raise ValueError("One or two identified arms with finite configurations are required")
    if any(not isfinite(task.cooling_s) or task.cooling_s < 0 for task in tasks):
        raise ValueError("Cooling durations must be finite and nonnegative")
    if any(not isfinite(value) or value <= 0 for value in offsets):
        raise ValueError("Start offsets must be finite and positive")
    if maximum_states < 1 or beam_width < 1:
        raise ValueError("Positive state and beam budgets are required")
    root = ScheduleState(
        0.0,
        configurations,
        frozenset(),
        tuple((node, 0.0) for node in sorted(ground_nodes)),
        payload=initial_payload,
    )
    if initial_state is not None:
        if not initial_state.completed <= by_id.keys() or set(
            dict(initial_state.configurations)
        ) != set(initial_configurations):
            raise ValueError("Resume state does not match the task/cell")
        root = initial_state
    frontier = [root]
    best_partial = root
    best = None
    failures = []
    explored = 0
    proposals_tried = 0
    pruned = 0
    batches_pruned = 0
    termination = "EXHAUSTED_FRONTIER"

    def reject(phase, state, failure):
        if len(failures) < 100:
            failures.append({"phase": phase, "time_s": state.time_s, **failure})

    def rank(state):
        remaining = estimate_remaining(state) if estimate_remaining else 0.0
        if not isfinite(remaining) or remaining < 0:
            raise ValueError("Remaining-time estimates must be finite and nonnegative")
        # The optional estimate is an ordering heuristic, never an admissibility
        # claim. Completed count breaks ties without merging distinct histories.
        if completion_first:
            return (-len(state.completed), -len(state.active), state.time_s + remaining)
        return (state.time_s + remaining, -len(state.completed), -len(state.active))

    while frontier:
        if monotonic() >= deadline:
            termination = "TIME_BUDGET"
            break
        if explored >= maximum_states:
            termination = "STATE_BUDGET"
            break
        frontier.sort(key=rank)
        state = frontier.pop(0)
        explored += 1
        if (len(state.completed), -state.time_s) > (
            len(best_partial.completed),
            -best_partial.time_s,
        ):
            best_partial = state
            if observe:
                observe(state, explored)
        if best is not None and state.time_s >= best.time_s:
            continue
        if len(state.completed) == len(tasks) and not state.active:
            # Final cooling is part of makespan even without a dependent stroke.
            final_ready = max((value for _, value in state.support_ready), default=state.time_s)
            if final_ready <= state.time_s:
                result, failure = finish(state)
                if failure is None:
                    if (
                        not isinstance(result, ScheduleState)
                        or result.time_s < state.time_s
                        or not isfinite(result.time_s)
                        or result.completed != state.completed
                        or result.active
                    ):
                        raise ValueError("Finish must return a complete, time-consistent state")
                    if best is None or result.time_s < best.time_s:
                        best = result
                    if completion_first:
                        termination = "FIRST_FEASIBLE"
                        break
                else:
                    reject("FINISH", state, failure)
                continue
        ready = state.ready_nodes()
        occupied = state.completed | frozenset(m.proposal.task for m in state.active)
        available = [
            task for task in tasks if task.identity not in occupied and task.anchors & ready
        ]
        choices = []
        for arm in state.idle_arms():
            options = [None]
            for task in available:
                if monotonic() >= deadline:
                    break
                for proposal in prepare(state, task, arm):
                    if (
                        proposal.task != task.identity
                        or proposal.arm != arm
                        or proposal.start_anchor not in task.anchors & ready
                        or not proposal.variant
                        or not isfinite(proposal.duration_s)
                        or proposal.duration_s <= 0
                        or len(proposal.end_configuration) != len(dict(state.configurations)[arm])
                        or not all(isfinite(q) for q in proposal.end_configuration)
                    ):
                        raise ValueError("Backend returned an invalid motion proposal")
                    options.append(proposal)
            if prepare_idle and monotonic() < deadline:
                for proposal in prepare_idle(state, arm):
                    if (
                        not proposal.auxiliary
                        or proposal.arm != arm
                        or proposal.task in by_id
                        or proposal.duration_s <= 0
                        or not isfinite(proposal.duration_s)
                        or len(proposal.end_configuration) != len(dict(state.configurations)[arm])
                        or not all(isfinite(q) for q in proposal.end_configuration)
                    ):
                        raise ValueError("Backend returned an invalid idle-arm motion")
                    options.append(proposal)
            choices.append(options)
        if monotonic() >= deadline:
            termination = "TIME_BUDGET"
            break
        batches = [
            batch
            for batch in product(*choices)
            if len({p.task for p in batch if p is not None}) == sum(p is not None for p in batch)
        ]

        def dispatch_cost(batch):
            durations = [p.duration_s for p in batch if p is not None]
            cost = max(durations, default=0) - 0.2 * (sum(durations) - max(durations, default=0))
            real = sum(p is not None and not p.auxiliary for p in batch)
            return (-real if completion_first else 0, cost)

        batches.sort(key=dispatch_cost)
        if maximum_dispatch_batches is not None:
            # Reserve alternatives for single-arm work, parking and waiting;
            # a large number of pairs must not consume the whole dispatch cap.
            categories = {}
            for batch in batches:
                key = (
                    sum(p is not None and not p.auxiliary for p in batch),
                    sum(p is not None and p.auxiliary for p in batch),
                )
                categories.setdefault(key, []).append(batch)
            retained = []
            while len(retained) < maximum_dispatch_batches and any(categories.values()):
                for group in categories.values():
                    if group and len(retained) < maximum_dispatch_batches:
                        retained.append(group.pop(0))
            batches_pruned += len(batches) - len(retained)
            batches = retained
        successors_added = 0
        for batch in batches:
            if (
                maximum_successors_per_state is not None
                and successors_added >= maximum_successors_per_state
            ):
                break
            new = tuple(ScheduledMotion(p, state.time_s) for p in batch if p is not None)
            if len({motion.proposal.task for motion in new}) != len(new):
                continue
            motions = state.active + new
            # Advance to completion/cooling, or allow a delayed dispatch while an
            # arm is still moving. Never invent future supports at dispatch.
            events = [motion.end_s for motion in motions]
            events.extend(ready_s for _, ready_s in state.support_ready if ready_s > state.time_s)
            if not events:
                continue
            next_event = min(events)
            advances = [next_event]
            if motions and len(motions) < len(configurations):
                advances += [
                    state.time_s + offset
                    for offset in offsets
                    if state.time_s + offset < next_event
                ]
            for end_s in advances:
                if monotonic() >= deadline:
                    break
                if best is not None and end_s >= best.time_s:
                    continue
                proposals_tried += 1
                payload, failure = validate_interval(state, new, end_s)
                if failure is not None:
                    reject("INTERVAL", state, failure)
                    continue
                finished = tuple(m for m in motions if m.end_s <= end_s)
                active = tuple(m for m in motions if m.end_s > end_s)
                q = dict(state.configurations)
                supports = dict(state.support_ready)
                completed_ids = state.completed
                for motion in finished:
                    proposal = motion.proposal
                    q[proposal.arm] = tuple(proposal.end_configuration)
                    if proposal.auxiliary:
                        continue
                    completed_ids = completed_ids | {proposal.task}
                    ready_s = motion.end_s + by_id[proposal.task].cooling_s
                    for node in by_id[proposal.task].deposits:
                        supports[node] = min(supports.get(node, float("inf")), ready_s)
                if complete and finished:
                    payload = complete(payload, finished)
                successor = ScheduleState(
                    end_s,
                    tuple(q.items()),
                    frozenset(completed_ids),
                    tuple(sorted(supports.items())),
                    active,
                    state.history + new,
                    payload,
                )
                if (len(successor.completed), -successor.time_s) > (
                    len(best_partial.completed),
                    -best_partial.time_s,
                ):
                    best_partial = successor
                    if observe:
                        observe(successor, explored)
                frontier.append(successor)
                successors_added += 1
                if (
                    maximum_successors_per_state is not None
                    and successors_added >= maximum_successors_per_state
                ):
                    break
        if len(frontier) > beam_width:
            frontier.sort(key=rank)
            pruned += len(frontier) - beam_width
            del frontier[beam_width:]
    selected = best if best is not None else best_partial
    return {
        "status": "FOUND" if best is not None else "NO_PLAN_FOUND_WITHIN_BUDGET",
        "termination": termination,
        "optimality_proven": False,
        "state": selected,
        "explored_states": explored,
        "intervals_tried": proposals_tried,
        "beam_pruned_states": pruned,
        "dispatch_batches_pruned": batches_pruned,
        "completion_first": completion_first,
        "maximum_successors_per_state": maximum_successors_per_state,
        "failures": failures,
    }
