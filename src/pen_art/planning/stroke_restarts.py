"""Screen pen-off breaks without adding geometry or assuming pen support."""

from copy import deepcopy

import numpy as np

from ..graph import straight_edge
from ..structure import analyze_structure


def release_screen(graph, material, printed_members, targets_build, diameter_m, start_node):
    """Check a cooled partial stroke with the pen removed, in build coordinates.

    The build frame must have world-aligned gravity. Only exact existing endpoints
    are welded (1 um numerical tolerance); proximity does not invent a support.
    """
    partial = deepcopy(graph)
    partial["edges"] = [e for e in partial["edges"] if e["id"] in printed_members]
    used = {e[k] for e in partial["edges"] for k in ("start", "end")}
    # An empty build already has the substrate. Ground anchors do not require
    # a previously printed member; elevated anchors still do.
    nodes = {
        n["id"]: n for n in partial["nodes"] if n["id"] in used or n.get("ground", False)
    }
    if start_node not in nodes:
        return {"modeled_pass": False, "failure": "START_NOT_PRINTED"}
    if np.linalg.norm(np.array(nodes[start_node]["position"]) - targets_build[0]) > 1e-6:
        raise ValueError("Partial stroke start differs from its existing anchor")
    previous = start_node
    for i, point in enumerate(targets_build[1:], 1):
        if np.linalg.norm(np.array(nodes[previous]["position"]) - point) < 1e-9:
            continue
        matches = [
            key
            for key, node in nodes.items()
            if np.linalg.norm(np.array(node["position"]) - point) <= 1e-6
        ]
        key = matches[0] if matches else f"restart-prefix-{i}"
        if not matches:
            nodes[key] = {
                "id": key,
                "position": np.asarray(point).tolist(),
                "ground": bool(abs(point[2]) <= 1e-6),
            }
        if previous != key:
            partial["edges"].append(
                straight_edge(previous, key, nodes, f"restart-segment-{i}", "artistic", diameter_m)
            )
        previous = key
    partial["nodes"] = list(nodes.values())
    result = analyze_structure(partial, material)
    result["release_node"] = previous
    result["pen_support_assumed"] = False
    return result


def posture_break_candidates(positions, maximum_step_rad=0.14):
    """Suggest cuts before abrupt posture changes; these are not yet approved."""
    q = np.asarray(positions)
    indices = np.flatnonzero(np.max(np.abs(np.diff(q, axis=0)), axis=1) > maximum_step_rad)
    return indices[indices > 0].tolist()


def audit_restart_clock(path, arm, breakpoints, tolerance_m=0.0003):
    """Check feed state and same-point returns in a serialized, continuous clock."""
    feed_on = False
    stops, starts = [], []
    for edge in path.edges:
        if edge.kind == "PEN_START_WAIT":
            if feed_on:
                raise ValueError("Repeated feed-on without a feed-off")
            feed_on = True
            starts.append(arm.robot.forward(edge.q0, arm.flange_to_tcp)[:3, 3])
        elif edge.kind == "PEN_STOP_WAIT":
            if not feed_on:
                raise ValueError("Feed-off without a matching feed-on")
            feed_on = False
            stops.append(arm.robot.forward(edge.q0, arm.flange_to_tcp)[:3, 3])
        elif edge.kind == "DRAW" and not feed_on:
            raise ValueError("Drawing while feed is off")
        elif edge.kind in {"TRAVEL", "APPROACH", "WITHDRAW", "WAIT"} and feed_on:
            raise ValueError("Repositioning or cooling while feed is on")
    if feed_on or len(starts) != len(stops) or len(starts) != len(breakpoints) + 1:
        raise ValueError("Incomplete restart clock")
    errors = []
    for end, start, target in zip(stops[:-1], starts[1:], breakpoints, strict=True):
        error = max(
            np.linalg.norm(end - start),
            np.linalg.norm(end - target),
            np.linalg.norm(start - target),
        )
        if error > tolerance_m:
            raise ValueError("Restart does not return to the source break point")
        errors.append(float(error))
    return {
        "feed_state_valid": True,
        "restart_count": len(errors),
        "same_point_errors_m": errors,
        "tolerance_m": tolerance_m,
    }


def attach_support_to_member(graph, support, support_nodes, source_members, tolerance_m=0.0001):
    """Split a beam at an added post; preserve the robot's source toolpath.

    The beam model uses a polyline approximation of the original curved member.
    The attachment discrepancy is bounded and reported, not silently welded to
    distant material. The source drawing trajectories themselves are untouched.
    """
    from ..curves import flatten_curve

    top = next(n for n in support_nodes if n["id"] == support["end"])
    target = np.asarray(top["position"])
    best = None
    for edge in graph["edges"]:
        if edge["id"] not in source_members:
            continue
        points, _ = flatten_curve(edge, tolerance=0.00001, max_chord=0.001)
        points = np.asarray(points)
        for i, (a, b) in enumerate(zip(points[:-1], points[1:], strict=True)):
            delta = b - a
            if np.dot(delta, delta) < 1e-18:
                continue
            t = np.clip(np.dot(target - a, delta) / np.dot(delta, delta), 0, 1)
            distance = float(np.linalg.norm(target - (a + t * delta)))
            if best is None or distance < best[0]:
                best = (distance, edge, points, i)
    if best is None or best[0] > tolerance_m:
        raise ValueError("Support top does not meet its source member")
    distance, original, points, index = best
    original_nodes = {n["id"]: n for n in graph["nodes"]}
    for endpoint in (original["start"], original["end"]):
        if np.linalg.norm(np.asarray(original_nodes[endpoint]["position"]) - target) <= 1e-6:
            return (
                {
                    **graph,
                    "nodes": [*graph["nodes"], *[n for n in support_nodes if n["id"] != top["id"]]],
                    "edges": [*graph["edges"], {**support, "end": endpoint}],
                },
                {},
                distance,
            )
    nodes = {n["id"]: n for n in [*graph["nodes"], *support_nodes]}
    replacements = []
    for suffix, start, end, chain in [
        ("before", original["start"], support["end"], [*points[: index + 1], target]),
        ("after", support["end"], original["end"], [target, *points[index + 1 :]]),
    ]:
        segments = []
        length = 0.0
        for a, b in zip(chain[:-1], chain[1:], strict=True):
            if np.linalg.norm(b - a) < 1e-9:
                continue
            segments.append(
                [a.tolist(), (a + (b - a) / 3).tolist(), (a + 2 * (b - a) / 3).tolist(), b.tolist()]
            )
            length += float(np.linalg.norm(b - a))
        if not segments:
            raise ValueError("Support attachment coincides with member endpoint")
        replacements.append(
            {
                **original,
                "id": original["id"] + "-support-" + suffix,
                "start": start,
                "end": end,
                "segments": segments,
                "length_m": length,
                "source_member_id": original["id"],
            }
        )
    result = {
        **graph,
        "nodes": list(nodes.values()),
        "edges": [e for e in graph["edges"] if e["id"] != original["id"]]
        + replacements
        + [support],
    }
    return result, {original["id"]: [e["id"] for e in replacements]}, distance
