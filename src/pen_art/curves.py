"""Cubic Bézier geometry; numerical samples never imply physical anchors."""

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


def bezier_at(control, t):
    p = np.asarray(control, dtype=float)
    return (
        (1 - t) ** 3 * p[0] + 3 * (1 - t) ** 2 * t * p[1] + 3 * (1 - t) * t * t * p[2] + t**3 * p[3]
    )


def curve_coordinate_bounds(curve):
    """Exact axis bounds of cubic pieces, including interior derivative roots."""
    values = []
    for segment in curve["segments"]:
        p = np.asarray(segment, dtype=float)
        parameters = {0.0, 1.0}
        cubic = -p[0] + 3 * p[1] - 3 * p[2] + p[3]
        quadratic = 3 * p[0] - 6 * p[1] + 3 * p[2]
        linear = -3 * p[0] + 3 * p[1]
        for axis in range(3):
            polynomial = np.trim_zeros([3 * cubic[axis], 2 * quadratic[axis], linear[axis]], "f")
            for root in np.roots(polynomial) if len(polynomial) > 1 else []:
                if abs(root.imag) < 1e-12 and 0 < root.real < 1:
                    parameters.add(float(root.real))
        values.extend(bezier_at(p, t) for t in parameters)
    return np.array([np.min(values, axis=0), np.max(values, axis=0)])


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


def fit_polyline(points, tolerance, max_depth=16):
    """Fit cubics with a continuous bound to the chord-parameterized source polyline."""
    points = np.asarray(points, dtype=float)
    if tolerance <= 0 or not np.isfinite(tolerance):
        raise ValueError("Fit tolerance must be finite and positive")
    keep = np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-10]
    points = points[keep]
    if len(points) < 2:
        return []

    def fit(p, depth):
        if len(p) == 2:
            delta = p[-1] - p[0]
            return [
                [
                    p[0].tolist(),
                    (p[0] + delta / 3).tolist(),
                    (p[0] + 2 * delta / 3).tolist(),
                    p[-1].tolist(),
                ]
            ]
        if len(p) == 3:
            return fit(p[:2], depth + 1) + fit(p[1:], depth + 1)
        lengths = np.r_[0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
        t = lengths / lengths[-1]
        a = np.column_stack([3 * (1 - t) ** 2 * t, 3 * (1 - t) * t**2])
        b = p - (1 - t[:, None]) ** 3 * p[0] - t[:, None] ** 3 * p[-1]
        middle = np.linalg.lstsq(a, b, rcond=None)[0]
        control = np.vstack([p[0], middle, p[-1]])
        predicted = a @ middle + (1 - t[:, None]) ** 3 * p[0] + t[:, None] ** 3 * p[-1]
        error = np.linalg.norm(predicted - p, axis=1)
        # Endpoint samples alone miss arbitrarily large overshoot in unevenly
        # sampled gaps. On every source segment, bound the cubic-minus-line by
        # the convex hull of its four Bezier difference control points.
        derivative = (
            3 * (1 - t[:, None]) ** 2 * (control[1] - control[0])
            + 6 * (1 - t[:, None]) * t[:, None] * (control[2] - control[1])
            + 3 * t[:, None] ** 2 * (control[3] - control[2])
        )
        dt = np.diff(t)[:, None]
        line_delta = np.diff(p, axis=0)
        interior_one = predicted[:-1] + dt * derivative[:-1] / 3 - (p[:-1] + line_delta / 3)
        interior_two = predicted[1:] - dt * derivative[1:] / 3 - (p[1:] - line_delta / 3)
        interval_error = np.maximum.reduce(
            [
                error[:-1],
                error[1:],
                np.linalg.norm(interior_one, axis=1),
                np.linalg.norm(interior_two, axis=1),
            ]
        )
        split = int(interval_error.argmax()) + 1
        if interval_error.max() <= tolerance:
            return [control.tolist()]
        if depth >= max_depth:
            # Preserve the bound when recursion runs out: use exact polyline edges.
            return [segment for i in range(len(p) - 1) for segment in fit(p[i : i + 2], depth + 1)]
        split = max(1, min(len(p) - 2, split))
        return fit(p[: split + 1], depth + 1) + fit(p[split:], depth + 1)

    return fit(points, 0)
