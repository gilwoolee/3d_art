"""Independent geometric coverage of original curves by saved drawing clocks."""

import numpy as np

from .smooth_audit import bounded_segments


def directed_bound(segments, reference, allowance, limit):
    """Bound distance of entire segments to a segment union, including interiors."""
    starts, ends = reference[:, 0], reference[:, 1]
    vectors = ends - starts
    squares = np.einsum("ij,ij->i", vectors, vectors)

    def distance(point):
        numerators = np.einsum("ij,ij->i", point - starts, vectors)
        t = np.divide(numerators, squares, out=np.zeros_like(squares), where=squares > 0)
        closest = starts + np.clip(t, 0, 1)[:, None] * vectors
        return float(np.min(np.linalg.norm(closest - point, axis=1)))

    def visit(a, b, depth):
        middle = (a + b) / 2
        observed = distance(middle) + allowance
        # Distance to a closed set is 1-Lipschitz.
        bound = observed + np.linalg.norm(b - a) / 2
        if bound <= limit or observed > limit or depth == 20:
            return float(bound)
        return max(visit(a, middle, depth + 1), visit(middle, b, depth + 1))

    return max(visit(a, b, 0) for a, b in segments)


def audit_source_curve(arm, paths, targets, limit=0.0003):
    """Enclose FK of every DRAW polynomial and check both coverage directions.

    Supports must be excluded by the caller. This checks geometric coverage;
    feed timing, collision, and source ordering remain separate replay checks.
    """
    targets = np.asarray(targets, float)
    if len(targets) < 2 or not np.isfinite(targets).all():
        raise ValueError("Invalid source polyline")
    reference = np.stack((targets[:-1], targets[1:]), axis=1)
    chords, inflation = [], 0.0
    for path in paths:
        for edge in path.edges:
            if edge.kind != "DRAW":
                continue
            piece = edge.piece()
            for a, b, error in bounded_segments(
                piece, arm.robot, arm.flange_to_tcp, 0.00001, maximum_depth=20
            ):
                chords.append(
                    [arm.robot.forward(piece.position(t), arm.flange_to_tcp)[:3, 3] for t in (a, b)]
                )
                inflation = max(inflation, error)
    if not chords:
        raise ValueError("No deposited path for original source")
    chords = np.asarray(chords)
    outgoing = directed_bound(chords, reference, inflation, limit)
    incoming = directed_bound(reference, chords, inflation, limit)
    return {
        "status": "BOUNDED" if max(outgoing, incoming) <= limit else "REJECTED",
        "deposition_to_source_bound_m": outgoing,
        "source_to_deposition_bound_m": incoming,
        "fk_chord_inflation_m": float(inflation),
        "position_limit_m": limit,
        "drawing_chords": len(chords),
    }
