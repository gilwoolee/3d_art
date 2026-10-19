"""Cubic Bézier geometry; numerical samples never imply physical anchors."""

from copy import deepcopy

import numpy as np


def transform_points(points, transform):
    points = np.asarray(points, dtype=float)
    matrix = np.asarray(transform, dtype=float)
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def sample_bezier(control_points, count=32):
    points = np.asarray(control_points, dtype=float)
    if points.shape != (4, 3) or not np.isfinite(points).all():
        raise ValueError("A cubic curve requires four finite 3D control points")
    t = np.linspace(0, 1, count)[:, None]
    return (
        (1 - t) ** 3 * points[0]
        + 3 * (1 - t) ** 2 * t * points[1]
        + (3 * (1 - t) * t**2 * points[2] + t**3 * points[3])
    )


def sample_curve(curve, count_per_segment=24):
    segments = curve["segments"]
    return np.concatenate(
        [sample_bezier(segment, count_per_segment)[:-1] for segment in segments]
        + [np.asarray(segments[-1][-1:], dtype=float)]
    )


def curve_length(curve):
    return float(np.linalg.norm(np.diff(sample_curve(curve, 48), axis=0), axis=1).sum())


def split_bezier(control, t):
    """Exact de Casteljau split; no physical junction is implied."""
    p = np.asarray(control, dtype=float)
    first = (1 - t) * p[:-1] + t * p[1:]
    second = (1 - t) * first[:-1] + t * first[1:]
    point = (1 - t) * second[0] + t * second[1]
    return np.array([p[0], first[0], second[0], point]), np.array(
        [point, second[1], first[2], p[3]]
    )


def curve_interval(curve, start, end):
    """Extract exact cubic pieces from a piece-index-plus-parameter interval."""
    count = len(curve["segments"])
    if not 0 <= start < end <= count:
        raise ValueError("Curve interval must be nonempty and inside its domain")
    result = []
    for index in range(int(np.floor(start)), min(count, int(np.ceil(end)))):
        low, high = max(0.0, start - index), min(1.0, end - index)
        if high - low < 1e-12:
            continue
        control = np.array(curve["segments"][index], dtype=float)
        if high < 1:
            control = split_bezier(control, high)[0]
        if low > 0:
            control = split_bezier(control, low / high)[1]
        result.append(control.tolist())
    return result


def flatten_curve(curve, tolerance=0.00005, max_chord=0.003):
    """Adaptive geometric samples with parameter values; never fabricable sub-strokes."""
    points, parameters = [], []

    def visit(control, low, high, depth=0):
        # The convex hull bounds distance to the line segment, including overshoot.
        chord = control[-1] - control[0]
        squared = float(chord @ chord)
        fractions = np.clip((control - control[0]) @ chord / max(squared, 1e-24), 0, 1)
        error = np.linalg.norm(control - (control[0] + fractions[:, None] * chord), axis=1).max()
        if (error <= tolerance and np.sqrt(squared) <= max_chord) or depth >= 22:
            points.append(control[0])
            parameters.append(low)
            return
        left, right = split_bezier(control, 0.5)
        mid = (low + high) / 2
        visit(left, low, mid, depth + 1)
        visit(right, mid, high, depth + 1)

    for index, control in enumerate(curve["segments"]):
        visit(np.asarray(control), float(index), float(index + 1))
    points.append(np.asarray(curve["segments"][-1][-1]))
    parameters.append(float(len(curve["segments"])))
    return np.array(points), np.array(parameters)


def orient_stroke(stroke, reverse=False, start_visit=0):
    """Re-root a closed contour at a real junction, preserving its material exactly."""
    result = deepcopy(stroke)
    if start_visit:
        if not stroke["closed"] or not 0 <= start_visit < len(stroke["members"]):
            raise ValueError("Only closed strokes can start at another junction")
        boundary = stroke["junction_visits"][start_visit]["segment_boundary"]
        offset = stroke["junction_visits"][start_visit]["distance_m"]
        result["segments"] = result["segments"][boundary:] + result["segments"][:boundary]
        result["members"] = result["members"][start_visit:] + result["members"][:start_visit]
        visits = stroke["junction_visits"][:-1]
        result["junction_visits"] = []
        for i in list(range(start_visit, len(visits))) + list(range(start_visit)):
            visit = deepcopy(visits[i])
            visit["distance_m"] = (visit["distance_m"] - offset) % stroke["length_m"]
            visit["segment_boundary"] = (visit["segment_boundary"] - boundary) % len(
                stroke["segments"]
            )
            result["junction_visits"].append(visit)
        first = result["junction_visits"][0]
        result["junction_visits"].append(
            {
                "node": first["node"],
                "distance_m": stroke["length_m"],
                "segment_boundary": len(stroke["segments"]),
            }
        )
        result["start"] = result["end"] = first["node"]
    if reverse:
        result["segments"] = [segment[::-1] for segment in result["segments"][::-1]]
        result["members"].reverse()
        result["start"], result["end"] = result["end"], result["start"]
        result["junction_visits"] = [
            {
                "node": visit["node"],
                "distance_m": result["length_m"] - visit["distance_m"],
                "segment_boundary": len(result["segments"]) - visit["segment_boundary"],
            }
            for visit in result["junction_visits"][::-1]
        ]
    result["reverse"] = reverse
    result["start_visit"] = start_visit
    return result
