"""Account for complete execution timelines independently of planner clocks."""

from collections import Counter

import numpy as np

_GROUPS = {
    "DRAW": "drawing",
    "TRAVEL": "travel",
    "RETURN": "travel",
    "PARK": "travel",
    "APPROACH": "approach_withdraw",
    "WITHDRAW": "approach_withdraw",
    "VERTEX_COOL": "cooling",
    "WAIT": "cooling",
    "PEN_START_WAIT": "feed_join",
    "PEN_STOP_WAIT": "feed_join",
    "JOIN": "feed_join",
    "INDEX_TURN": "turntable",
    "TABLE_SETTLE": "turntable",
}


def execution_metrics(plan):
    """Count serial action clocks and every unrepresented gap, rejecting overlap."""
    if len(plan.get("active_arms", ["arm_1"])) != 1:
        raise ValueError("Execution metrics require a single-arm timeline")
    seconds, kinds = Counter(), Counter()
    occupied = []
    length = 0.0
    draws = []
    for action in plan["actions"]:
        offset = float(action["start_s"])
        for edge in action["trajectory"]["edges"]:
            start, end = offset + edge["start_s"], offset + edge["end_s"]
            if end < start or not np.isfinite([start, end]).all():
                raise ValueError("Invalid execution interval")
            occupied.append((start, end))
            kind = edge["kind"]
            kinds[kind] += end - start
            seconds[_GROUPS.get(kind, "other")] += end - start
            if kind == "DRAW" and edge.get("tcp0_m") is not None:
                length += float(np.linalg.norm(np.subtract(edge["tcp1_m"], edge["tcp0_m"])))
        if not action.get("auxiliary", False):
            draws.append(action)
    cursor = 0.0
    for start, end in sorted(occupied):
        if start < cursor - 1e-6:
            raise ValueError("Overlapping execution intervals")
        seconds["timeline_gaps"] += max(0.0, start - cursor)
        cursor = max(cursor, end)
    total = float(plan["modeled_duration_s"])
    if total < cursor - 1e-6:
        raise ValueError("Plan ends before its execution intervals")
    seconds["timeline_gaps"] += max(0.0, total - cursor)
    if not np.isclose(sum(seconds.values()), total, atol=1e-6, rtol=0):
        raise ValueError("Incomplete execution accounting")
    return {
        "modeled_duration_s": total,
        "seconds_by_activity": dict(seconds),
        "seconds_by_kind": dict(kinds),
        "drawing_actions": len(draws),
        "source_curves": len({a.get("parent_curve", a["stroke_id"]) for a in draws}),
        "draw_tcp_chord_length_m": length,
        "mean_drawing_speed_mm_s": 1000 * length / seconds["drawing"] if seconds["drawing"] else 0,
        "wireprint_reference_s": 14 * 60,
        "difference_from_reference_s": total - 14 * 60,
        "scope": (
            "Modeled execution; chord-length speed is approximate; reference geometry may differ"
        ),
    }
