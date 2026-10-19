"""Re-audit timed cubic geometry against the arm, work-cell and growing strands.

The clock does not change the geometric sweep. This audit evaluates the saved
cubic itself and bounds between samples; it never inherits a joint-linear
collision certificate. Pen-tip contact and hot-material behavior are separate.
"""


import numpy as np
from numpy.polynomial import Polynomial

from ..kinematics import pose_error
from ..timing import (
    _stationary_roots,
    maximum_absolute,
    polynomial_range,
    tcp_acceleration_bound,
)


def _horner(coefficients, x):
    # The recurrence numpy's polyval uses, on plain floats.
    value = coefficients[-1] + x * 0
    for coefficient in coefficients[-2::-1]:
        value = coefficient + value * x
    return value


def _extent(coefficients, stationary, a, b):
    values = [_horner(coefficients, a), _horner(coefficients, b)]
    values.extend(_horner(coefficients, x) for x in stationary if a < x < b)
    return min(values), max(values)


class PolynomialPiece:
    """Joint polynomials on [0,1], with exact extrema for continuous checking.

    Checking subdivides a piece thousands of times. The piece therefore keeps,
    for the joint polynomials and their first two derivatives, plain
    coefficient rows and stationary points, and evaluates them directly; the
    numpy polynomial objects are built only when a caller asks for them.
    """

    def __init__(self, polynomials):
        self._polynomials = list(polynomials)
        self._rows = None
        if not self._polynomials or any(not np.isfinite(p.coef).all() for p in self._polynomials):
            raise ValueError("Expected finite joint polynomials")

    @property
    def polynomials(self):
        if self._polynomials is None:
            self._polynomials = [Polynomial(row) for row in self._rows[0]]
        return self._polynomials

    def _tables(self):
        """Coefficient rows of every joint, per derivative order; False if unsupported."""
        if self._rows is None:
            if any(not np.array_equal(p.domain, p.window) for p in self._polynomials):
                self._rows = False  # Mapped domains keep the general numpy path.
                return False
            rows = [[[float(c) for c in p.coef] for p in self._polynomials]]
            for _ in range(2):
                rows.append([[k * c for k, c in enumerate(row)][1:] or [0.0] for row in rows[-1]])
            self._rows = rows
            self._stationary = [None, None, None]
        return self._rows

    def _extents(self, order, a, b):
        """Lowest and highest value of each joint's ``order``-th derivative on ``[a, b]``."""
        stationary = self._stationary[order]
        if stationary is None:
            # Found on first use: many pieces are only evaluated or subdivided.
            stationary = self._stationary[order] = [
                tuple(
                    float(root.real)
                    for root in _stationary_roots(Polynomial, tuple(row), (-1.0, 1.0), (-1.0, 1.0))
                    if abs(root.imag) < 1e-8
                )
                for row in self._rows[order]
            ]
        rows = zip(self._rows[order], stationary, strict=True)
        return [_extent(row, points, a, b) for row, points in rows]

    def position(self, t):
        rows = self._tables()
        if rows is False or not isinstance(t, (int, float)):
            return np.array([p(t) for p in self.polynomials])
        return np.array([_horner(row, t) for row in rows[0]])

    def velocity(self, t):
        """Derivative with respect to the piece parameter."""
        rows = self._tables()
        if rows is False:
            return np.array([p.deriv()(t) for p in self.polynomials])
        return np.array([_horner(row, t) for row in rows[1]])

    def ranges(self, a, b):
        """Lowest and highest value of every joint on ``[a, b]``."""
        rows = self._tables()
        if rows is False:
            return np.array([polynomial_range(p, a, b) for p in self.polynomials])
        return np.array(self._extents(0, a, b))

    def deviation(self, a, b):
        q = self.position((a + b) / 2)
        ranges = self.ranges(a, b)
        return np.array(
            [max(abs(lo - m), abs(hi - m)) for (lo, hi), m in zip(ranges, q, strict=True)]
        )

    def derivatives(self, a, b):
        rows = self._tables()
        if rows is False:
            return tuple(
                np.array([maximum_absolute(p.deriv(order), a, b) for p in self.polynomials])
                for order in (1, 2)
            )
        return tuple(
            np.array([max(abs(low), abs(high)) for low, high in self._extents(order, a, b)])
            for order in (1, 2)
        )

    def subpiece(self, a, b):
        # Composition preserves all extrema, including nonmonotonic excursions.
        rows = self._tables()
        if rows is False:
            clock = Polynomial([a, b - a])
            return PolynomialPiece(p(clock) for p in self.polynomials)
        width = b - a
        composed = []
        for row in rows[0]:
            # Horner's rule in polynomial arithmetic: row(a + width * u).
            result = [row[-1]]
            for coefficient in row[-2::-1]:
                shifted = [0.0, *(c * width for c in result)]
                result = [c * a + s for c, s in zip([*result, 0.0], shifted, strict=True)]
                result[0] = coefficient + result[0]
            composed.append(result)
        piece = object.__new__(PolynomialPiece)
        piece._polynomials = None
        piece._rows = [composed]
        for _ in range(2):
            piece._rows.append(
                [[k * c for k, c in enumerate(row)][1:] or [0.0] for row in piece._rows[-1]]
            )
        # Stationary points already found carry over; the rest wait for first use.
        piece._stationary = [
            None
            if found is None or not width
            else [tuple((x - a) / width for x in points) for points in found]
            for found in self._stationary
        ]
        return piece


class CubicPiece(PolynomialPiece):
    """One saved spline piece, reparameterized to [0,1] without changing shape."""

    def __init__(self, coefficients, width):
        coefficients = np.asarray(coefficients, float)
        if coefficients.ndim != 2 or coefficients.shape[0] != 4 or width <= 0:
            raise ValueError("Expected cubic coefficients and positive knot spacing")
        if not np.isfinite(coefficients).all() or not np.isfinite(width):
            raise ValueError("Nonfinite cubic geometry")
        super().__init__(
            Polynomial(coefficients[::-1, j] * width ** np.arange(4))
            for j in range(coefficients.shape[1])
        )


def tracking_bound(piece, robot, offset, start, end, rotation, a=0.0, b=1.0):
    """Bound FK error to a linear Cartesian reference on a subinterval."""
    actual = [robot.forward(piece.position(t), offset) for t in (a, b)]
    targets = [np.asarray(start) * (1 - t) + np.asarray(end) * t for t in (a, b)]
    first, second = piece.derivatives(a, b)
    acceleration = tcp_acceleration_bound(robot, offset, first, second)
    bound = max(np.linalg.norm(pose[:3, 3] - p) for pose, p in zip(actual, targets, strict=True))
    bound += acceleration * (b - a) ** 2 / 8 + 1e-12
    orientation = 0.0
    if rotation is not None:
        target = np.eye(4)
        target[:3, :3] = rotation
        endpoint = max(np.linalg.norm(pose_error(pose, target)[3:]) for pose in actual)
        matrix_error = 2 * np.sin(endpoint / 2)
        matrix_error += (first.sum() ** 2 + second.sum()) * (b - a) ** 2 / 8
        orientation = float(2 * np.arcsin(min(1.0, matrix_error / 2))) + 1e-12
    return float(bound), orientation


def bounded_segments(piece, robot, offset, tolerance, maximum_depth=12):
    """Enclose every FK point by a chord plus an analytic radius inflation."""
    result = []

    def visit(a, b, depth):
        first, second = piece.derivatives(a, b)
        error = tcp_acceleration_bound(robot, offset, first, second) * (b - a) ** 2 / 8
        if error > tolerance and depth < maximum_depth:
            mid = (a + b) / 2
            visit(a, mid, depth + 1)
            visit(mid, b, depth + 1)
        else:
            result.append((a, b, float(error)))

    visit(0.0, 1.0, 0)
    return result


def check_tracking(piece, robot, offset, start, end, rotation, position_limit, angle_limit):
    maxima = [0.0, 0.0]
    resolved = True
    witness = None
    worst_score = -1.0

    def visit(a, b, depth):
        nonlocal resolved, witness, worst_score
        position, angle = tracking_bound(piece, robot, offset, start, end, rotation, a, b)
        observed_violation = False
        for t in (a, (a + b) / 2, b):
            q = piece.position(t)
            pose = robot.forward(q, offset)
            target = np.asarray(start) * (1 - t) + np.asarray(end) * t
            error = float(np.linalg.norm(pose[:3, 3] - target))
            angle_error = 0.0
            if rotation is not None:
                target_pose = np.eye(4)
                target_pose[:3, :3] = rotation
                angle_error = float(np.linalg.norm(pose_error(pose, target_pose)[3:]))
            score = max(error / position_limit, angle_error / angle_limit)
            if score > worst_score:
                worst_score = score
                witness = {
                    "parameter": t,
                    "positions": q.tolist(),
                    "actual_m": pose[:3, 3].tolist(),
                    "reference_m": target.tolist(),
                    "position_error_m": error,
                    "orientation_error_rad": angle_error,
                }
            observed_violation |= score > 1
        if (
            (position > position_limit or angle > angle_limit)
            and not observed_violation
            and depth < 8
        ):
            mid = (a + b) / 2
            visit(a, mid, depth + 1)
            visit(mid, b, depth + 1)
            return
        maxima[0], maxima[1] = max(maxima[0], position), max(maxima[1], angle)
        resolved &= position <= position_limit and angle <= angle_limit

    visit(0.0, 1.0, 0)
    return {
        "status": "BOUNDED" if resolved else "BOUND_EXCEEDS_TOLERANCE",
        "position_error_bound_m": maxima[0],
        "orientation_error_bound_rad": maxima[1],
        "witness": witness,
    }
