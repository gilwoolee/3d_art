"""Lazy backward construction search over material subsets and robot configurations.

The motion callback yields alternative *forward* section-and-transit witnesses.
Its returned configuration is the beginning of that witness, before extrusion.
It may yield None after bounded failed work so another source gets a turn.
Only structural results may be cached by material subset.
"""

from collections import deque
from time import monotonic

from ..structure import analyze_structure


def greedy_forward_order(actions, ground, stable, *, deadline=float("inf")):
    """Keep suggested priorities when support and cooled stiffness permit.

    This chooses one complete geometric order before motion sampling. It does
    not backtrack earlier order choices or prove impossibility on failure.
    """
    remaining = sorted(actions, key=lambda row: (row["priority"], row["id"]))
    available, built, order = set(ground), set(), []
    while remaining and monotonic() < deadline:
        framework_pending = any(row["stage"] == "framework" for row in remaining)
        for row in remaining:
            if monotonic() >= deadline:
                return None
            if framework_pending and row["stage"] != "framework":
                continue
            anchors = (
                {visit["node"] for visit in row["junction_visits"]}
                if row.get("closed")
                else {row["start"], row["end"]}
            )
            if available.intersection(anchors) and stable(built | {row["id"]}):
                break
        else:
            return None
        remaining.remove(row)
        order.append(row)
        built.add(row["id"])
        available.update((row["start"], row["end"]))
        available.update(visit["node"] for visit in row.get("junction_visits", []))
    return None if remaining else order


def search_regression(
    actions,
    graph,
    material,
    terminal,
    motions,
    initial_travel,
    *,
    maximum_states=2000,
    deadline=float("inf"),
    progress=None,
    motion_priority=None,
):
    by_id = {row["id"]: row for row in actions}
    if len(by_id) != len(actions):
        raise ValueError("Construction section IDs must be unique")
    ground = {row["id"] for row in graph["nodes"] if row["ground"]}
    structural = {}
    attempts, failures = 0, []

    def anchors(row):
        return (
            {visit["node"] for visit in row["junction_visits"]}
            if row.get("closed")
            else {row["start"], row["end"]}
        )

    def stable(ids):
        if ids not in structural:
            members = {m for key in ids for m in by_id[key]["members"]}
            screen = analyze_structure(graph, material, edge_ids=members)
            structural[ids] = (
                screen
                if len(ids) == len(by_id)
                else {key: value for key, value in screen.items() if key not in {"nodes", "edges"}}
            )
        return structural[ids]["modeled_pass"]

    full = frozenset(by_id)
    if not stable(full):
        return {"status": "GOAL_STRUCTURE_REJECTED", "structure": structural[full]}

    # A relaxed forward construction supplies a stiffness-aware removal order.
    # It ignores robot motion, so it is guidance rather than an admissibility
    # claim. Robot configurations and alternate paths remain in the search.
    guide, built, available = {}, frozenset(), set(ground)
    guide_started = monotonic()
    guide_deadline = min(deadline, guide_started + min(60.0, (deadline - guide_started) * 0.1))
    while len(built) < len(by_id) and monotonic() < guide_deadline:
        remaining = set(by_id) - built
        base_pending = any(by_id[key]["stage"] == "framework" for key in remaining)
        candidates = sorted(
            (
                by_id[key]
                for key in remaining
                if (not base_pending or by_id[key]["stage"] == "framework")
                and available.intersection(anchors(by_id[key]))
            ),
            key=lambda row: (row["priority"], row["id"]),
        )
        feasible = []
        for row in candidates:
            if monotonic() >= guide_deadline:
                break
            next_ids = built | {row["id"]}
            if stable(next_ids):
                screen = structural[next_ids]
                feasible.append((screen.get("maximum_displacement_m", 0.0), row))
            # Keep this a small relaxed guide, not an exhaustive geometry solver.
            if len(feasible) >= 4:
                break
        if not feasible:
            break
        _, row = min(feasible, key=lambda item: (item[0], item[1]["priority"]))
        guide[row["id"]] = len(built)
        built = built | {row["id"]}
        available.update(visit["node"] for visit in row.get("junction_visits", []))
        available.update((row["start"], row["end"]))
    guide_seconds = monotonic() - guide_started

    class Limit(Exception):
        pass

    def successors(remaining, q, depth):
        nonlocal attempts
        if monotonic() >= deadline or attempts >= maximum_states:
            raise Limit
        art = any(by_id[key]["stage"] == "artistic" for key in remaining)
        candidates = sorted(
            (by_id[key] for key in remaining if not art or by_id[key]["stage"] == "artistic"),
            key=lambda row: (
                motion_priority(row, remaining, q) if motion_priority is not None else 0,
                -guide.get(row["id"], row["priority"]),
                row["id"],
            ),
        )

        def row_choices(row):
            nonlocal attempts
            if monotonic() >= deadline or attempts >= maximum_states:
                raise Limit
            predecessor = remaining - {row["id"]}
            if not stable(predecessor):
                return
            available = ground | {
                by_id[key][end] for key in predecessor for end in ("start", "end")
            }
            available.update(
                visit["node"]
                for key in predecessor
                for visit in by_id[key].get("junction_visits", [])
            )
            if not available.intersection(anchors(row)):
                return
            # Do not collapse alternative modes to the first feasible local path.
            for choice in motions(row, predecessor, available, q):
                if choice is None:
                    yield None
                    continue
                before, witness = choice
                attempts += 1
                if progress:
                    progress(depth, attempts, row["id"])
                yield predecessor, before, witness
                if attempts >= maximum_states or monotonic() >= deadline:
                    raise Limit
            if len(failures) < 100:
                failures.append({"section": row["id"], "remaining": len(remaining)})

        # Construct an iterator only when its row is visited. Long frameworks
        # keep many suspended depth frames, so eager generators waste memory.
        pending = deque(candidates)
        while pending:
            if monotonic() >= deadline or attempts >= maximum_states:
                raise Limit
            entry = pending.popleft()
            iterator = row_choices(entry) if isinstance(entry, dict) else entry
            try:
                choice = next(iterator)
            except StopIteration:
                continue
            pending.append(iterator)
            if choice is not None:
                yield choice

    def search(full):
        # Explicit frames retain lazy motion alternatives for long frameworks
        # without tying the number of sections to Python's recursion limit.
        stack = [(full, terminal, iter(successors(full, terminal, 0)))]
        reversed_suffix = []
        while stack:
            if monotonic() >= deadline or attempts >= maximum_states:
                raise Limit
            remaining, q, choices = stack[-1]
            if not remaining:
                travel = initial_travel(q)
                if travel is not None:
                    return travel, list(reversed(reversed_suffix))
                stack.pop()
                if reversed_suffix:
                    reversed_suffix.pop()
                continue
            try:
                predecessor, before, witness = next(choices)
            except StopIteration:
                stack.pop()
                if reversed_suffix:
                    reversed_suffix.pop()
                continue
            reversed_suffix.append(witness)
            stack.append(
                (predecessor, before, iter(successors(predecessor, before, len(reversed_suffix))))
            )
        return None

    try:
        result = search(full)
        status = "FOUND" if result is not None else "NO_PLAN_FOUND_WITHIN_CANDIDATES"
    except Limit:
        result, status = None, "SEARCH_BUDGET"
    return {
        "status": status,
        "result": result,
        "attempts": attempts,
        "structural_states": len(structural),
        "forward_stiffness_guide": guide,
        "forward_guide_complete": len(guide) == len(by_id),
        "forward_guide_seconds": guide_seconds,
        "failures": failures,
    }
