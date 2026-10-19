"""Layered redundant-arm IK for drawing strokes, with sampled tracking checks.

Outputs remain simulation previews until tool collisions, pen-off transits,
physical pen calibration and coupled construction search have been validated.
"""

from time import monotonic

import numpy as np

from ..kinematics import pose_error


def search_layers(targets, solve, edge_check, initial_seeds, beam_width=3, deadline=np.inf):
    """Retain alternate configurations, then backtrace the lowest-cost full path.

    Each target is solved using every retained previous configuration. Endpoint
    reachability alone is insufficient: edge_check must accept each connection.
    """
    if beam_width < 1:
        raise ValueError("beam_width must be positive")
    layers = []
    seeds = [np.asarray(seed) for seed in initial_seeds]
    for index, target in enumerate(targets):
        if monotonic() >= deadline:
            return {"status": "TIME_BUDGET", "failed_sample": index}
        candidates = solve(target, seeds)
        nodes = []
        for q in candidates:
            q = np.asarray(q)
            if index == 0:
                cost = min(float(np.linalg.norm(q - seed)) for seed in seeds)
                nodes.append({"q": q, "cost": cost, "parent": None, "edge_error": 0.0})
                continue
            parents = []
            for parent_index, previous in enumerate(layers[-1]):
                error = edge_check(previous["q"], q, targets[index - 1], target)
                if error is not None:
                    cost = previous["cost"] + float(np.linalg.norm(q - previous["q"]))
                    parents.append((cost, parent_index, error))
            if parents:
                cost, parent, error = min(parents)
                nodes.append({"q": q, "cost": cost, "parent": parent, "edge_error": error})
        nodes.sort(key=lambda node: node["cost"])
        distinct = []
        for node in nodes:
            if not any(np.linalg.norm(node["q"] - other["q"]) < 1e-3 for other in distinct):
                distinct.append(node)
        if not distinct:
            return {"status": "NO_CONNECTED_IK_PATH", "failed_sample": index}
        layers.append(distinct[:beam_width])
        seeds = [node["q"] for node in layers[-1]]
    if not layers:
        raise ValueError("At least one target pose is required")
    def backtrace(index):
        selected = []
        for layer in reversed(layers):
            node = layer[index]
            selected.append(node)
            index = node["parent"]
        selected.reverse()
        return {
            "positions": [node["q"].tolist() for node in selected],
            "maximum_sampled_interpolation_error_m": max(node["edge_error"] for node in selected),
        }

    alternatives = [backtrace(index) for index in range(len(layers[-1]))]
    return {
        "status": "SAMPLED_DRAWING_PATH",
        **alternatives[0],
        "path_alternatives": alternatives,
        "endpoint_alternatives": [node["q"].tolist() for node in layers[-1]],
        "retained_states": sum(len(layer) for layer in layers),
    }


def tracking_edge(robot, offset, q0, q1, p0, p1, tolerance_m, tolerance_rad, maximum_step):
    """Sample joint interpolation; reject branch jumps and excessive TCP drift."""
    if np.max(np.abs(q1 - q0)) > maximum_step:
        return None
    worst = 0.0
    for alpha in (0.25, 0.5, 0.75):
        target = p0.copy()
        target[:3, 3] = (1 - alpha) * p0[:3, 3] + alpha * p1[:3, 3]
        actual = robot.forward((1 - alpha) * q0 + alpha * q1, offset)
        error = pose_error(actual, target)
        distance = float(np.linalg.norm(error[:3]))
        if distance > tolerance_m or np.linalg.norm(error[3:]) > tolerance_rad:
            return None
        worst = max(worst, distance)
    return worst
