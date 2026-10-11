"""Ground-base classification and stable, anchor-ready serial construction order."""

import numpy as np


def action_dependencies(construction):
    """Construction artifacts store predecessor action indices, not only IDs."""
    actions = construction["actions"]
    by_index = {action.get("index", i): action["id"] for i, action in enumerate(actions)}
    ids = {action["id"] for action in actions}
    result = {}
    for action in actions:
        dependencies = {
            by_index[dependency] if isinstance(dependency, int) else dependency
            for dependency in action.get("depends_on", [])
        }
        if not dependencies <= ids:
            raise ValueError("Construction contains an unknown dependency")
        result[action["id"]] = dependencies
    return result


def ground_base_actions(construction, graph):
    """Identify explicit base strokes on the substrate, not grounded branches."""
    nodes = {node["id"]: node for node in graph["nodes"]}
    actions = {action["id"]: action for action in construction["actions"]}
    dependencies = action_dependencies(construction)
    bases = {
        key
        for key, action in actions.items()
        if action.get("role") == "base"
        and all(nodes[action[end]].get("ground", False) for end in ("start", "end"))
        and action.get("segments")
        and np.all(np.abs(np.asarray(action["segments"], float)[..., 2]) <= 1e-6)
    }
    # A nominal base depending on another phase cannot safely lead that phase.
    while True:
        eligible = {key for key in bases if dependencies[key] <= bases}
        if eligible == bases:
            return bases
        bases = eligible


def stable_ground_base_order(selection, construction, graph):
    """Move eligible bases first while respecting explicit and selected-anchor dependencies."""
    actions = {action["id"]: action for action in construction["actions"]}
    if len(selection) != len(actions) or {row["stroke_id"] for row in selection} != set(actions):
        raise ValueError("Selection must cover each construction action exactly once")
    bases = ground_base_actions(construction, graph)
    dependencies = action_dependencies(construction)
    available = {node["id"] for node in graph["nodes"] if node.get("ground", False)}
    pending, result, done = list(selection), [], set()
    while pending:
        ready = [
            (index, row)
            for index, row in enumerate(pending)
            if dependencies[row["stroke_id"]] <= done and row["start_anchor"] in available
        ]
        if not ready:
            raise ValueError("No dependency-valid selected-anchor construction order")
        index, row = min(ready, key=lambda pair: (pair[1]["stroke_id"] not in bases, pair[0]))
        result.append(row)
        pending.pop(index)
        done.add(row["stroke_id"])
        available.update(v["node"] for v in actions[row["stroke_id"]]["junction_visits"])
    if {row["stroke_id"] for row in result[: len(bases)]} != bases:
        raise ValueError("Ground bases cannot form a dependency-valid first phase")
    return result
