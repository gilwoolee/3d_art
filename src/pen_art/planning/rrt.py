"""Bounded bidirectional RRT-Connect over joint configurations.

Every accepted edge is delegated to the caller's full interpolation validator.
Search exhaustion is not evidence of geometric impossibility.
"""

from time import monotonic

import numpy as np


def connect(
    start,
    goal,
    lower,
    upper,
    valid_edge,
    *,
    seed=42,
    step_rad=0.15,
    maximum_nodes=2000,
    maximum_seconds=3,
):
    start, goal, lower, upper = [np.asarray(v, dtype=float) for v in (start, goal, lower, upper)]
    if (
        start.shape != goal.shape
        or start.shape != lower.shape
        or start.shape != upper.shape
        or not np.isfinite([start, goal, lower, upper]).all()
        or np.any(upper <= lower)
        or step_rad <= 0
        or maximum_nodes < 2
        or maximum_seconds <= 0
    ):
        raise ValueError("Invalid RRT bounds or search budget")
    if (
        np.any(start < lower)
        or np.any(start > upper)
        or np.any(goal < lower)
        or np.any(goal > upper)
    ):
        raise ValueError("RRT endpoint is outside the joint limits")
    deadline = monotonic() + maximum_seconds
    edges = 0

    def checked(a, b):
        nonlocal edges
        edges += 1
        return valid_edge(a, b)

    if not checked(start, start) or not checked(goal, goal):
        return {"status": "INVALID_ENDPOINT", "checked_edges": edges, "nodes": 2}
    if checked(start, goal):
        return {
            "status": "CONNECTED",
            "positions": [start.tolist(), goal.tolist()],
            "checked_edges": edges,
            "nodes": 2,
            "method": "direct",
        }
    trees = [[(start, None)], [(goal, None)]]
    rng = np.random.default_rng(seed)
    scale = upper - lower

    def extend(tree, target):
        nearest = min(range(len(tree)), key=lambda i: np.linalg.norm((tree[i][0] - target) / scale))
        old = tree[nearest][0]
        delta = target - old
        fraction = min(1.0, step_rad / max(float(np.max(np.abs(delta))), 1e-12))
        new = old + fraction * delta
        if not checked(old, new):
            return "TRAPPED", nearest
        tree.append((new, nearest))
        return ("REACHED" if fraction == 1 else "ADVANCED"), len(tree) - 1

    def trace(tree, index):
        path = []
        while index is not None:
            q, parent = tree[index]
            path.append(q)
            index = parent
        return list(reversed(path))

    active = 1
    while sum(map(len, trees)) < maximum_nodes and monotonic() < deadline:
        active = 1 - active
        first, second = trees[active], trees[1 - active]
        target = second[0][0] if rng.random() < 0.1 else rng.uniform(lower, upper)
        state, a = extend(first, target)
        if state == "TRAPPED":
            continue
        bridge_state = "BUDGET"
        while sum(map(len, trees)) < maximum_nodes and monotonic() < deadline:
            bridge_state, b = extend(second, first[a][0])
            if bridge_state != "ADVANCED":
                break
        if bridge_state == "REACHED":
            left, right = trace(first, a), trace(second, b)
            path = left + list(reversed(right))[1:]
            if active:
                path.reverse()
            # Preserve endpoints, shorten only through newly validated edges.
            for _ in range(min(30, len(path) * 2)):
                if len(path) <= 2 or monotonic() >= deadline:
                    break
                i, j = sorted(rng.choice(len(path), 2, replace=False))
                if j > i + 1 and checked(path[i], path[j]):
                    path = path[: i + 1] + path[j:]
            return {
                "status": "CONNECTED",
                "positions": [q.tolist() for q in path],
                "checked_edges": edges,
                "nodes": sum(map(len, trees)),
                "method": "rrt_connect",
            }
    return {
        "status": "NO_PLAN_FOUND_WITHIN_BUDGET",
        "checked_edges": edges,
        "nodes": sum(map(len, trees)),
    }
