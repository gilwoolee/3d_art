"""Material readiness and feed events derived from the saved motion clock."""

import numpy as np

from ..curves import transform_points
from ..strokes import orient_stroke

PROCESS_CLOCK_POLICY = "whole-connected-action-v1"


def feed_events(actions):
    return sorted(
        [
            {
                "time_s": row["start_s"] + edge["start_s"],
                "arm": row["arm"],
                "stroke_id": row["stroke_id"],
                "command": "FEED_ON" if edge["kind"] == "PEN_START_WAIT" else "FEED_OFF",
            }
            for row in actions
            for edge in row["trajectory"]["edges"]
            if edge["kind"] in ("PEN_START_WAIT", "PEN_STOP_WAIT")
        ],
        key=lambda event: (event["time_s"], event["arm"]),
    )


def derive_deposition_events(path, action, cooling_s, transform):
    """Match source junctions to ordered, saved DRAW endpoints and their actual times.

    The clock contains every retained geometric knot. Matching uses geometry,
    never cached timing annotations; callers bind the source and transform.
    """
    oriented = orient_stroke(
        action,
        reverse=path.metadata.get("source_reverse", False),
        start_visit=path.metadata.get("source_start_visit", 0),
    )
    draws = [edge for edge in path.edges if edge.kind == "DRAW"]
    if not draws:
        raise ValueError("Material clock has no drawing")
    points = np.array([draws[0].tcp0, *[edge.tcp1 for edge in draws]])
    times = np.array([draws[0].start_s, *[edge.end_s for edge in draws]])
    distance = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    events, previous = [], 0
    visits = oriented.get("junction_visits", [])
    for number, visit in enumerate(visits):
        boundary = visit["segment_boundary"]
        point = np.asarray(
            oriented["segments"][0][0] if boundary == 0 else oriented["segments"][boundary - 1][-1]
        )
        target = transform_points([point], transform)[0]
        if number == 0:
            index = 0
        elif number == len(visits) - 1:
            index = len(points) - 1
        else:
            expected = distance[-1] * visit["distance_m"] / oriented["length_m"]
            costs = np.linalg.norm(points[previous:] - target, axis=1) ** 2
            costs += 1e-12 * (distance[previous:] - expected) ** 2
            index = previous + int(np.argmin(costs))
        if np.linalg.norm(points[index] - target) > 0.00035 + 1e-9:
            raise ValueError("Saved drawing misses a source junction")
        deposited = float(times[index])
        events.append(
            {
                "node": visit["node"],
                "source_boundary": boundary,
                "deposited_at_s": deposited,
                "ready_at_s": deposited + cooling_s,
            }
        )
        previous = index
    return events


def action_ready_times(path, action, cooling_s):
    """Relative material availability; legacy clocks retain conservative semantics."""
    if path.metadata.get("process_clock_policy") != PROCESS_CLOCK_POLICY:
        nodes = {visit["node"] for visit in action.get("junction_visits", [])}
        nodes.update(action[key] for key in ("start", "end") if key in action)
        return {node: path.duration_s + cooling_s for node in nodes}
    result = {}
    for event in path.metadata["deposition_events"]:
        node, ready = event["node"], event["ready_at_s"]
        if not np.isfinite(ready) or ready < event["deposited_at_s"]:
            raise ValueError("Invalid material readiness event")
        result[node] = min(result.get(node, float("inf")), ready)
    return result


def source_vertex_schedule(action, drawing, targets, transform, cooling_s, process=None):
    """Locate actual tangent discontinuities, never smooth-curve sampling knots.

    The pen stops at each one. It holds with the feed off after a rising
    member, and after a falling one too unless the policy is
    ``rising_edge_end``, which trusts a foot laid on solid wire to need none.
    """
    process = process or {}
    feet_hold = process.get("vertex_cooling_policy") != "rising_edge_end"
    oriented = orient_stroke(
        action, reverse=drawing.get("reverse", False), start_visit=drawing.get("start_visit", 0)
    )
    segments = transform_points(np.asarray(oriented["segments"]).reshape(-1, 3), transform)
    segments = segments.reshape(-1, 4, 3)
    breaks, holds = [], {}
    previous = 0
    for before, after in zip(segments[:-1], segments[1:], strict=True):
        incoming = next(
            (
                before[-1] - point
                for point in before[-2::-1]
                if np.linalg.norm(before[-1] - point) > 1e-10
            ),
            None,
        )
        outgoing = next(
            (point - after[0] for point in after[1:] if np.linalg.norm(point - after[0]) > 1e-10),
            None,
        )
        if incoming is None or outgoing is None:
            continue
        incoming /= np.linalg.norm(incoming)
        outgoing /= np.linalg.norm(outgoing)
        if np.linalg.norm(incoming - outgoing) < 1e-5:
            continue
        index = previous + int(np.argmin(np.linalg.norm(targets[previous:] - before[-1], axis=1)))
        if np.linalg.norm(targets[index] - before[-1]) > 0.00035:
            raise ValueError("Tracked path omits a physical source vertex")
        previous = index
        if not 0 < index < len(targets) - 1:
            raise ValueError("Physical source vertex does not have an interior path knot")
        breaks.append(index)
        # Under ``rising_edge_end`` only a riser counts: a strand that climbs
        # gently, like one turn of a coil tower, is drawn without holding.
        if (abs(incoming[2]) > 1e-5 if feet_hold else incoming[2] > 0.5) and cooling_s:
            holds[index] = {"duration_s": cooling_s, "contact_m": targets[index].tolist()}
    return breaks, holds, abs(segments[-1][-1, 2] - segments[-1][0, 2]) > 1e-8


def audit_process_clock(path, action, process, cooling_s, transform, *, require_release=False):
    """Recheck process events against source vertices and the serialized clock."""
    expected = derive_deposition_events(path, action, cooling_s, transform)
    saved = path.metadata.get("deposition_events", [])
    # Clock concatenation adds the same offset in a different arithmetic order
    # than source replay. Compare seconds at the clock's absolute tolerance,
    # while preserving exact node identity and source-boundary correspondence.
    if len(saved) != len(expected) or any(
        not isinstance(actual, dict)
        or set(actual) != set(wanted)
        or actual.get("node") != wanted["node"]
        or actual.get("source_boundary") != wanted["source_boundary"]
        or not np.allclose(
            [actual.get("deposited_at_s", np.nan), actual.get("ready_at_s", np.nan)],
            [wanted["deposited_at_s"], wanted["ready_at_s"]],
            atol=1e-9,
            rtol=0.0,
        )
        for actual, wanted in zip(saved, expected, strict=True)
    ):
        raise ValueError("Saved material readiness differs from independent source replay")
    if (
        abs(
            max(event["ready_at_s"] for event in expected)
            - path.metadata.get("release_ready_s", -1)
        )
        > 1e-8
    ):
        raise ValueError("Saved release readiness differs from deposited material")
    # A stroke chained to its predecessor takes over with the pen down and
    # feeding; one chained to its successor hands over the same way.
    feeding = bool(path.metadata.get("chained_in"))
    for edge in path.edges:
        if edge.kind in ("PEN_START_WAIT", "PEN_STOP_WAIT"):
            lag = process["start_lag_s"] if edge.kind == "PEN_START_WAIT" else process["stop_lag_s"]
            if edge.end_s - edge.start_s < lag - 1e-8:
                raise ValueError("Feed response interval is shorter than its process lag")
            feeding = edge.kind == "PEN_START_WAIT"
        elif edge.kind == "DRAW" and not feeding:
            raise ValueError("Drawing with feed off")
        elif edge.kind in ("VERTEX_COOL", "WITHDRAW", "INDEX_TURN") and feeding:
            raise ValueError("Cooling, withdrawal or table movement while feeding")
    if feeding != bool(path.metadata.get("chained_out")):
        raise ValueError("Process clock finishes with the wrong feed state for its successor")
    draws = [edge for edge in path.edges if edge.kind == "DRAW"]
    points = np.array([draws[0].tcp0, *[edge.tcp1 for edge in draws]])
    corners, holds, vertical_end = source_vertex_schedule(
        action,
        {
            "reverse": path.metadata.get("source_reverse", False),
            "start_visit": path.metadata.get("source_start_visit", 0),
        },
        points,
        transform,
        float(process.get("vertex_cooling_s", 1.0)),
        process,
    )
    for index in corners:
        before, after = draws[index - 1], draws[index]
        for edge, endpoint in ((before, 1), (after, 0)):
            velocity = np.array([p.deriv()(endpoint) for p in edge.piece().polynomials])
            velocity /= edge.end_s - edge.start_s
            if np.max(abs(velocity)) > 1e-6:
                raise ValueError("Source vertex was traversed without stopping")
        if index in holds:
            between = [
                edge
                for edge in path.edges
                if edge.start_s >= before.end_s - 1e-9 and edge.end_s <= after.start_s + 1e-9
            ]
            kinds = [edge.kind for edge in between]
            if (
                "PEN_STOP_WAIT" not in kinds
                or "PEN_START_WAIT" not in kinds
                or after.start_s - before.end_s < holds[index]["duration_s"] - 1e-8
            ):
                raise ValueError("Source vertex is missing its feed-off cooling hold")
    required = max(
        cooling_s if require_release else 0.0,
        float(process.get("vertex_cooling_s", 1.0)) if vertical_end else 0.0,
    )
    withdraws = [edge for edge in path.edges if edge.kind == "WITHDRAW"]
    if withdraws and withdraws[0].start_s < draws[-1].end_s + required - 1e-8:
        raise ValueError("Pen withdrawal precedes material release readiness")
    if path.metadata.get("chained_out"):
        if withdraws:
            raise ValueError("A stroke chained to its successor must not withdraw")
        # The pen stays down, so only a free riser top needs its hold.
        hold = float(process.get("vertex_cooling_s", 1.0)) if vertical_end else 0.0
        if path.edges[-1].end_s < draws[-1].end_s + hold - 1e-8:
            raise ValueError("Chained stroke leaves a free riser top before its hold")
    elif not withdraws and require_release:
        raise ValueError("An unchained stroke must withdraw")
    return {
        "status": "SOURCE_PROCESS_CLOCK_VERIFIED",
        "vertices": len(corners),
        "cooling_holds": len(holds),
        "deposition_events": len(expected),
    }
