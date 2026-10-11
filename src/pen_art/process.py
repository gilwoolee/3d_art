"""Pen-oriented deposition questions, with measured limits separated from assumptions."""

import numpy as np

from .curves import flatten_curve
from .io import artifact


def stroke_features(edge, reverse=False):
    points, _ = flatten_curve(edge, tolerance=0.00001, max_chord=0.001)
    if reverse:
        points = points[::-1]
    delta = np.diff(points, axis=0)
    distance = np.linalg.norm(delta, axis=1)
    valid = distance > 1e-9
    delta, distance = delta[valid], distance[valid]
    radii = []
    for a, b in zip(delta, delta[1:], strict=False):
        area = np.linalg.norm(np.cross(a, b)) / 2
        if area > 1e-15:
            radii.append(np.linalg.norm(a) * np.linalg.norm(b) * np.linalg.norm(a + b) / (4 * area))
    return {
        "length_m": float(distance.sum()),
        "maximum_horizontal_reach_m": float(
            np.linalg.norm(points[:, :2] - points[0, :2], axis=1).max()
        ),
        "downward_travel_m": float(np.maximum(0.0, -delta[:, 2]).sum()),
        "minimum_curvature_radius_m": float(min(radii)) if radii else None,
        "minimum_z_m": float(points[:, 2].min()),
        "maximum_z_m": float(points[:, 2].max()),
    }


def free_intervals(edge, nodes, printed_nodes, reverse=False):
    """Intervals between already existing anchors, never between numerical samples.

    The far endpoint supports an interval only after the pen has traversed it. A
    measured free-tip limit, if provided, therefore applies to the whole interval.
    Progressive cooling may allow longer strokes; that belongs in the pen model.
    """
    segments = edge["segments"]
    boundaries = [0]
    for visit in edge.get("junction_visits", [])[1:-1]:
        if nodes[visit["node"]]["ground"] or visit["node"] in printed_nodes:
            boundaries.append(visit["segment_boundary"])
    boundaries.append(len(segments))
    intervals = []
    for low, high in zip(boundaries, boundaries[1:], strict=False):
        features = stroke_features({"segments": segments[low:high]}, reverse)
        # Continuous contact with the bed is supported along the whole stroke.
        if features["minimum_z_m"] >= -1e-8 and features["maximum_z_m"] <= 1e-8:
            continue
        intervals.append(features)
    return intervals


def evaluate_action(edge, nodes, profile, printed_nodes=None, node_ages=None, reverse=False):
    """Unknown pen capabilities produce calibration requests, not invented limits."""
    printed_nodes, node_ages = printed_nodes or set(), node_ages or {}
    start = edge["end"] if reverse else edge["start"]
    features = stroke_features(edge, reverse)
    failures, advisories = [], []
    if not nodes[start]["ground"] and start not in printed_nodes:
        failures.append("NO_EXISTING_START_ANCHOR")
    cooling_wait = 0.0
    cooling_age = profile.get("cooled_anchor_age_s")
    if not nodes[start]["ground"] and start in printed_nodes:
        cooling_wait = (
            max(0.0, cooling_age - node_ages.get(start, 0.0)) if cooling_age is not None else None
        )
    if features["minimum_z_m"] < -profile["strand_diameter_m"] / 2:
        failures.append("BELOW_BUILD_SURFACE")
    intervals = free_intervals(edge, nodes, printed_nodes, reverse)
    features["maximum_unanchored_length_m"] = max(
        (interval["length_m"] for interval in intervals), default=0.0
    )
    features["maximum_unanchored_horizontal_m"] = max(
        (interval["maximum_horizontal_reach_m"] for interval in intervals), default=0.0
    )
    features["maximum_unanchored_downward_m"] = max(
        (interval["downward_travel_m"] for interval in intervals), default=0.0
    )
    limits = [
        ("length_m", "minimum_stroke_m", "STROKE_SHORTER_THAN_PROCESS_RESOLUTION", "min"),
        ("maximum_unanchored_length_m", "maximum_free_tip_length_m", "UNSUPPORTED_LENGTH", "max"),
        (
            "maximum_unanchored_horizontal_m",
            "maximum_horizontal_reach_m",
            "HORIZONTAL_REACH",
            "max",
        ),
        (
            "maximum_unanchored_downward_m",
            "maximum_downward_motion_m",
            "DOWNWARD_DEPOSITION",
            "max",
        ),
        (
            "minimum_curvature_radius_m",
            "minimum_curvature_radius_m",
            "CURVATURE_OUTSIDE_ENVELOPE",
            "min",
        ),
    ]
    unknown = []
    for feature, setting, code, bound in limits:
        value, threshold = features[feature], profile.get(setting)
        if threshold is None:
            unknown.append(setting)
            continue
        outside = value is not None and (
            value < threshold - 1e-9 if bound == "min" else value > threshold + 1e-9
        )
        if outside:
            (failures if profile["calibrated"] else advisories).append(code)
    return {
        "edge": edge["id"],
        "reverse": reverse,
        "modeled_pass": not failures,
        "physical_status": (
            "requires_calibration"
            if not profile["calibrated"] or unknown
            else "requires_forward_replay"
        ),
        "features": features,
        "bed_supported": not intervals,
        "cooling_wait_s": cooling_wait,
        "failures": failures,
        "advisories": advisories,
        "unknown_limits": unknown,
    }


def screen_graph(graph, profile, strokes=None):
    nodes = {node["id"]: node for node in graph["nodes"]}
    # A generous screening context, not a claim that these anchors exist in order.
    all_nodes = set(nodes)
    ages = dict.fromkeys(nodes, profile.get("cooled_anchor_age_s") or 0.0)
    rows = []
    actions = strokes["curves"] if strokes is not None else graph["edges"]
    for edge in actions:
        directions = [
            evaluate_action(edge, nodes, profile, all_nodes, ages, reverse=reverse)
            for reverse in (False, True)
        ]
        rows.append(
            {
                "id": edge["id"],
                "modeled_pass": any(d["modeled_pass"] for d in directions),
                "requires_calibration": any(
                    d["physical_status"] == "requires_calibration" for d in directions
                ),
                "directions": directions,
            }
        )
    length = sum(edge["length_m"] for edge in actions)
    assumed_speed = profile["nominal_speed_m_s"]
    return artifact(
        "process_screen",
        calibrated=profile["calibrated"],
        settings=profile,
        physical_status="unverified",
        unit="continuous_pen_stroke" if strokes is not None else "material_member",
        modeled_pass=all(row["modeled_pass"] for row in rows),
        acceptable_actions=sum(row["modeled_pass"] for row in rows),
        rejected_actions=sum(not row["modeled_pass"] for row in rows),
        calibration_required_actions=sum(row["requires_calibration"] for row in rows),
        edges=rows,
        timing_assumption={
            "speed_m_s": assumed_speed,
            "deposition_only_s": length / assumed_speed,
            "not_a_job_prediction": True,
            "excluded": ["travel", "joins", "cooling", "startup", "shutdown"],
        },
        limitations=[
            "Continuous strokes can traverse material junctions without stopping feed",
            "All potential anchors assumed present for this initial screen",
            "Unknown pen limits require experiments, not automatic rejection",
            "Far-end support is not assumed before the pen reaches it",
            "Free-tip limits do not model progressive cooling during drawing",
            "No robot clearance or construction-order proof yet",
        ],
    )
