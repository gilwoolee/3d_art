"""Bounded order repair over complete, collision-checked robot transitions.

States belong to the caller and include the actual robot configuration and
material scene. No geometry-only failed-state cache is used: two prefixes with
the same material can have different robot configurations and transit access.
"""

from time import monotonic


def search_order(
    actions,
    initial,
    eligible,
    transition,
    finish,
    *,
    maximum_states=2000,
    deadline=float("inf"),
    observe=None,
    progress=None,
):
    """Try the supplied order first; backtrack when a robot transition fails.

    Transition and finish return (next_state, failure). A None next_state is a
    rejection, while a successful transition must leave its input state intact.
    Only fully completed drawing-and-withdrawal transitions enter a prefix.
    """
    actions = tuple(actions)
    if len(set(actions)) != len(actions) or maximum_states < 1:
        raise ValueError("Unique actions and a positive search budget are required")
    attempts, backtracks = 0, 0
    failures = []
    last_failure = None
    best_state, best_order = initial, ()

    class Limit(Exception):
        pass

    def record(order, identity, failure):
        nonlocal last_failure
        last_failure = {"prefix": list(order), "action": identity, **(failure or {})}
        if len(failures) < 100:
            failures.append(last_failure)
        if progress:
            progress(order, attempts, backtracks, last_failure)

    def reserve():
        nonlocal attempts
        if monotonic() >= deadline:
            raise Limit("TIME_BUDGET")
        if attempts >= maximum_states:
            raise Limit("STATE_BUDGET")
        attempts += 1

    def search(state, order):
        nonlocal best_state, best_order, backtracks
        if progress:
            progress(order, attempts, backtracks, None)
        if len(order) > len(best_order):
            best_state, best_order = state, order
            if observe:
                observe(state, order, attempts, backtracks)
        if len(order) == len(actions):
            reserve()
            result, failure = finish(state)
            if result is not None:
                return result, order
            record(order, None, failure)
            backtracks += 1
            return None
        completed = frozenset(order)
        available = [a for a in actions if a not in completed and eligible(state, a)]
        if not available:
            record(order, None, {"phase": "ANCHOR", "status": "NO_AVAILABLE_START"})
        for identity in available:
            reserve()
            result, failure = transition(state, identity, attempts)
            if result is None:
                record(order, identity, failure)
                continue
            answer = search(result, (*order, identity))
            if answer is not None:
                return answer
        backtracks += 1
        return None

    try:
        answer = search(initial, ())
        status = "FOUND" if answer is not None else "NO_PLAN_FOUND_WITHIN_BUDGET"
    except Limit as limit:
        answer = None
        status = str(limit)
    state, order = answer if answer is not None else (best_state, best_order)
    return {
        "status": status,
        "state": state,
        "order": list(order),
        "attempts": attempts,
        "backtracks": backtracks,
        "failures": failures,
        "last_failure": last_failure,
    }
