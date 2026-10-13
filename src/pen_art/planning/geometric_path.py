"""Untimed joint paths, and the proof that timing leaves their sweep unchanged.

A geometric path holds the same edges ``timed_path`` would time, with the edge
index in place of seconds. Collision, deposition, tracking and orientation
depend only on this sweep. ``same_sweep`` certifies that a timed clock visits
exactly these configurations in the same order, so a geometric proof carries
over to it. Neither function establishes speed, acceleration or cooling.
"""

import numpy as np
from numpy.polynomial import Polynomial

from ..timing import make_path
from .clock_path import ClockEdge, ClockPath, coalesce_travel, split_drawing_operations

# Largest joint difference, in radians, between a timed edge and its geometry.
SWEEP_TOLERANCE_RAD = 1e-9


def geometric_path(operations, arm, *, drawing_clock="quintic_stop_at_geometric_knots"):
    """Return the untimed sweep of ``operations``; edge ``k`` spans ``[k, k+1]``.

    Expansion, travel coalescing and the dropping of zero-length holds mirror
    ``timed_path``. A continuous drawing contributes one edge per cubic spline
    piece instead of one per retiming grid interval.
    """
    if drawing_clock not in {"quintic_stop_at_geometric_knots", "cubic_toppra"}:
        raise ValueError(f"Unsupported drawing clock: {drawing_clock}")
    expanded = []
    for operation in operations:
        expanded.extend(
            split_drawing_operations(operation) if operation.get("break_indices") else [operation]
        )
    edges = []

    def add(first, second, op, coefficients=None):
        contact = op.get("contact_m")
        edges.append(
            ClockEdge(
                float(len(edges)),
                float(len(edges) + 1),
                first,
                second,
                op["kind"],
                op.get("stroke_id"),
                None if contact is None or coefficients is not None else np.asarray(contact),
                op.get("material_radius_m", 0.0),
                arm.robot.forward(first, arm.flange_to_tcp)[:3, 3],
                arm.robot.forward(second, arm.flange_to_tcp)[:3, 3],
                coefficients=coefficients,
            )
        )

    for op in expanded:
        q = np.asarray(op["positions"], float)
        if op["kind"] == "DRAW" and drawing_clock == "cubic_toppra":
            path = op.get("path")
            if path is None:
                points = [arm.robot.forward(joints, arm.flange_to_tcp)[:3, 3] for joints in q]
                path = make_path(q, points, interpolation="pchip", subdivisions=2)
            knots, cubic = path.cspl.x, path.cspl.c
            for piece, width in enumerate(np.diff(knots)):
                # Ascending powers of the normalized piece parameter.
                coefficients = [
                    [float(c * width**power) for power, c in enumerate(cubic[::-1, piece, joint])]
                    for joint in range(cubic.shape[2])
                ]
                first = np.array([row[0] for row in coefficients])
                second = np.array([sum(row) for row in coefficients])
                add(first, second, op, coefficients)
            continue
        if op["kind"] in ("TRAVEL", "RETURN", "PARK"):
            q = coalesce_travel(q)
        if "hold_s" in op:
            if op["hold_s"] > 0:
                add(q[0], q[0], op)
            continue
        for first, second in zip(q[:-1], q[1:], strict=True):
            if np.max(abs(second - first)) >= 1e-12:
                add(first, second, op)
    result = ClockPath(edges)
    result.metadata["parameterization"] = "geometric"
    return result


def _padded(polynomial, size):
    coefficients = np.zeros(size)
    coefficients[: len(polynomial.coef)] = polynomial.coef
    return coefficients


def _clock_onto(geometry, timed, start):
    """Fit the quadratic clock placing ``timed`` on ``geometry`` from ``start``.

    Returns the reached piece parameter and the summed coefficient difference.
    A timed edge that is not this cubic under a nondecreasing quadratic clock
    cannot produce a small difference.
    """
    slope = np.array([p.deriv()(start) for p in geometry])
    scale = float(slope @ slope)
    if scale < 1e-24:
        return None, np.inf
    curvature = np.array([p.deriv(2)(start) for p in geometry])
    size = max(max(len(p.coef) for p in timed), 7)
    target = np.array([_padded(p, size) for p in timed])
    linear = float(slope @ target[:, 1]) / scale
    quadratic = float(slope @ (target[:, 2] - curvature * linear**2 / 2)) / scale
    if linear < -1e-12 or linear + 2 * quadratic < -1e-12:
        return None, np.inf
    clock = Polynomial([start, linear, quadratic])
    difference = max(
        float(np.abs(_padded(p(clock), size) - row).sum())
        for p, row in zip(geometry, target, strict=True)
    )
    return start + linear + quadratic, difference


def same_sweep(geometric, timed, tolerance=SWEEP_TOLERANCE_RAD):
    """Certify that ``timed`` sweeps exactly ``geometric``, edge by edge.

    Every timed drawing edge must be one geometric cubic composed with a
    nondecreasing quadratic clock, consecutive timed edges must tile each cubic
    from its start to its end, and every other edge must match its geometric
    edge exactly. Any mismatch is a rejection.
    """

    def rejected(reason, geometric_index, timed_index):
        return {
            "status": "SWEEP_MISMATCH",
            "reason": reason,
            "geometric_edge": geometric_index,
            "timed_edge": timed_index,
        }

    if geometric.metadata.get("parameterization") != "geometric":
        return rejected("not_a_geometric_path", None, None)
    worst, cursor = 0.0, 0
    for index, edge in enumerate(geometric.edges):
        if cursor >= len(timed.edges):
            return rejected("timed_path_ends_early", index, cursor)
        if edge.coefficients is None:
            other = timed.edges[cursor]
            if (
                other.coefficients is not None
                or other.kind != edge.kind
                or other.stroke != edge.stroke
                or not np.array_equal(other.q0, edge.q0)
                or not np.array_equal(other.q1, edge.q1)
                or (edge.contact is None) != (other.contact is None)
                or (edge.contact is not None and not np.array_equal(edge.contact, other.contact))
            ):
                return rejected("edge_differs", index, cursor)
            cursor += 1
            continue
        geometry = [Polynomial(row) for row in edge.coefficients]
        reached = 0.0
        while reached < 1 - tolerance:
            if cursor >= len(timed.edges):
                return rejected("timed_path_ends_early", index, cursor)
            other = timed.edges[cursor]
            if (
                other.coefficients is None
                or other.kind != edge.kind
                or other.stroke != edge.stroke
                or len(other.coefficients) != len(geometry)
                or other.radius_m != edge.radius_m
            ):
                return rejected("edge_differs", index, cursor)
            following, difference = _clock_onto(
                geometry, [Polynomial(row) for row in other.coefficients], reached
            )
            if following is None or difference > tolerance:
                return rejected("not_a_reparameterization", index, cursor)
            if following <= reached or following > 1 + tolerance:
                return rejected("clock_leaves_piece", index, cursor)
            worst, reached, cursor = max(worst, difference), following, cursor + 1
    if cursor != len(timed.edges):
        return rejected("timed_path_has_extra_edges", len(geometric.edges), cursor)
    return {
        "status": "SAME_SWEEP",
        "geometric_edges": len(geometric.edges),
        "timed_edges": len(timed.edges),
        "maximum_difference_rad": worst,
    }
