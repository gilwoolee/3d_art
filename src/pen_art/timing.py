"""Joint splines for drawing and the clock that times them, for simulation.

No artifact from this module is an executable hardware program.
"""

from functools import lru_cache

import numpy as np
from numpy.polynomial import Polynomial
from scipy.interpolate import PchipInterpolator, PPoly

from .kinematics import geometric_jacobian


class TimingParameterizationError(ValueError):
    """A valid geometric path did not receive a certified finite clock."""


@lru_cache(maxsize=32768)
def _stationary_roots(kind, coefficients, domain, window):
    # Subdivision changes the interval, not the polynomial's stationary points.
    # Snapshot coefficients and coordinate mapping so later edits cannot reuse
    # roots from a different polynomial. Keep only immutable cached results.
    return tuple(kind(coefficients, domain=domain, window=window).deriv().roots())


def polynomial_range(poly, start, end):
    """Extrema of a scalar polynomial on a closed interval, including endpoints."""
    roots = _stationary_roots(type(poly), tuple(poly.coef), tuple(poly.domain), tuple(poly.window))
    points = [start, end] + [
        float(root.real) for root in roots if abs(root.imag) < 1e-8 and start < root.real < end
    ]
    if np.array_equal(poly.domain, poly.window):
        # Horner's rule as numpy evaluates it, without its per-call overhead.
        x, coefficients = np.asarray(points), poly.coef
        values = coefficients[-1] + x * 0
        for coefficient in coefficients[-2::-1]:
            values = coefficient + values * x
    else:
        values = poly(points)
    return float(np.min(values)), float(np.max(values))


def maximum_absolute(poly, start, end):
    return max(abs(value) for value in polynomial_range(poly, start, end))


def tcp_acceleration_bound(robot, offset, velocity, acceleration):
    """Serial-chain translation bound for any joint values in an interval."""
    omega, alpha, result = 0.0, 0.0, 0.0
    for joint in robot.joint_data:
        result += np.linalg.norm(joint["origin"][:3, 3]) * (omega**2 + alpha)
        if joint["index"] is not None:
            omega += velocity[joint["index"]]
            alpha += acceleration[joint["index"]]
    return float(result + np.linalg.norm(offset[:3, 3]) * (omega**2 + alpha))


def refine_samples(values, subdivisions=2):
    """Numerical subdivision of existing straight segments, not new pen actions."""
    values = np.asarray(values, float)
    if not isinstance(subdivisions, int) or subdivisions < 1 or len(values) < 2:
        raise ValueError("Refinement needs two samples and a positive integer subdivision count")
    fractions = np.arange(subdivisions) / subdivisions
    interior = values[:-1, None] + fractions.reshape(-1, *([1] * (values.ndim - 1))) * (
        values[1:, None] - values[:-1, None]
    )
    return np.concatenate([interior.reshape(-1, *values.shape[1:]), values[-1:]])


class SplinePath:
    """A joint spline ``cspl`` (a piecewise cubic) over path parameter [0, 1]."""

    def __init__(self, cspl):
        self.cspl = cspl

    @property
    def dof(self):
        return self.cspl.c.shape[2]


def make_path(positions, tcp_points, *, subdivisions=1, breaks=None):
    """Shape-preserving (pchip) joint spline through ``positions``, parameterized
    by tool travel.

    ``breaks`` names knots where the pen stops. The interpolant is then fitted
    to each run between stops on its own, so a run between two knots is
    exactly the straight joint segment and no corner bends its neighbours.
    """
    q, p = np.asarray(positions, float), np.asarray(tcp_points, float)
    if q.ndim != 2 or p.shape != (len(q), 3) or not np.isfinite(q).all():
        raise ValueError("Expected finite joint positions and corresponding TCP points")
    if not np.isfinite(p).all() or len(q) < 2:
        raise ValueError("At least two finite path points are required")
    keep = np.r_[True, np.linalg.norm(np.diff(q, axis=0), axis=1) > 1e-10]
    q, p = q[keep], p[keep]
    if len(q) < 2:
        raise ValueError("Path contains no motion")
    if subdivisions != 1:
        q, p = refine_samples(q, subdivisions), refine_samples(p, subdivisions)
    # TCP arc length improves resolution for drawing; the joint term also
    # parameterizes pure orientation changes and near-stationary tip motion.
    steps = np.linalg.norm(np.diff(p, axis=0), axis=1)
    steps += 0.01 * np.linalg.norm(np.diff(q, axis=0), axis=1)
    knots = np.r_[0.0, np.cumsum(steps)]
    knots /= knots[-1]
    if breaks and (subdivisions != 1 or not keep.all()):
        raise ValueError("Stops need an unrefined path without repeated points")
    cuts = sorted({0, len(knots) - 1, *(breaks or ())})
    if len(cuts) == 2:
        return SplinePath(PchipInterpolator(knots, q, axis=0))
    return SplinePath(
        PPoly(
            np.concatenate(
                [
                    PchipInterpolator(knots[a : b + 1], q[a : b + 1], axis=0).c
                    for a, b in zip(cuts[:-1], cuts[1:], strict=True)
                ],
                axis=1,
            ),
            knots,
        )
    )


def check_position_limits(path, lower, upper):
    maximum_deviation = np.zeros(path.dof)
    for i, (a, b) in enumerate(zip(path.cspl.x[:-1], path.cspl.x[1:], strict=True)):
        width = b - a
        for j in range(path.dof):
            poly = Polynomial(path.cspl.c[:, i, j][::-1])
            lo, hi = polynomial_range(poly, 0, width)
            if lo < lower[j] - 1e-10 or hi > upper[j] + 1e-10:
                raise ValueError(f"Smoothed path exceeds joint {j + 1} position limits")
            chord = Polynomial([poly(0), (poly(width) - poly(0)) / width])
            maximum_deviation[j] = max(
                maximum_deviation[j], maximum_absolute(poly - chord, 0, width)
            )
    return maximum_deviation


def clock_polynomials(path, grid, velocities):
    """Compose each cubic geometric piece with its quadratic path clock."""
    result, elapsed = [], 0.0
    for i, ds in enumerate(np.diff(grid)):
        speed_sum = velocities[i] + velocities[i + 1]
        if speed_sum <= 1e-12:
            raise TimingParameterizationError(
                "Retimer produced a stationary interval with nonzero length"
            )
        duration = 2 * ds / speed_sum
        acceleration = (velocities[i + 1] ** 2 - velocities[i] ** 2) / (2 * ds)
        piece = min(len(path.cspl.x) - 2, np.searchsorted(path.cspl.x, grid[i], side="right") - 1)
        if grid[i + 1] > path.cspl.x[piece + 1] + 1e-10:
            raise ValueError("Retiming grid must include every spline knot")
        clock = Polynomial([grid[i] - path.cspl.x[piece], velocities[i], acceleration / 2])
        polynomials = [Polynomial(path.cspl.c[:, piece, j][::-1])(clock) for j in range(path.dof)]
        result.append((elapsed, duration, polynomials))
        elapsed += duration
    return result


def _bernstein(degree):
    """Matrix taking ascending power coefficients on [0, 1] to Bernstein coefficients."""
    from math import comb

    return np.array(
        [
            [comb(k, i) / comb(degree, i) if i <= k else 0.0 for i in range(degree + 1)]
            for k in range(degree + 1)
        ]
    )


def _bernstein_eighths(degree):
    """Power coefficients to the Bernstein coefficients of eight equal subintervals.

    Halving an interval by de Casteljau's rule keeps the polynomial inside each
    half's coefficient range, and that range tightens fourfold per halving.
    """
    from math import comb

    left = np.array(
        [
            [comb(k, i) / 2**k if i <= k else 0.0 for i in range(degree + 1)]
            for k in range(degree + 1)
        ]
    )
    right = np.array(
        [
            [
                comb(degree - k, i - k) / 2 ** (degree - k) if i >= k else 0.0
                for i in range(degree + 1)
            ]
            for k in range(degree + 1)
        ]
    )
    parts = [_bernstein(degree)]
    for _ in range(3):
        parts = [half @ part for part in parts for half in (left, right)]
    return np.concatenate(parts)


_BERNSTEIN = {degree: _bernstein_eighths(degree) for degree in (4, 5)}


def scan_clock(path, grid, robot, offset, velocity_limits, acceleration_limits, speed):
    """Closed-form path clock: a forward and a backward pass over the grid.

    A drawing is short straight runs at a slow tip speed with a stop at each
    end, so the fastest clock is nominal speed with ramps. Each grid point gets
    the largest squared path speed its joint-velocity, tip-speed and curvature
    limits allow; the two passes then limit how fast that speed may rise and
    fall under the joint-acceleration limits. This is a proposal only. The
    caller bounds the resulting polynomials over every whole interval and
    slows the clock uniformly by whatever those bounds require.

    Returns the path speed at every grid point, a bound on the tip tangent
    over every interval, and the largest joint velocity and acceleration of
    the composed motion at unit time scale.
    """
    knots, grid = path.cspl.x, np.asarray(grid, float)
    ds = np.diff(grid)
    index = np.clip(np.searchsorted(knots, grid[:-1], side="right") - 1, 0, len(knots) - 2)
    if np.any(grid[1:] > knots[index + 1] + 1e-10):
        raise ValueError("Retiming grid must include every spline knot")
    c3, c2, c1, c0 = (path.cspl.c[k][index] for k in range(4))  # cubic to constant
    left = (grid[:-1] - knots[index])[:, None]
    right = left + ds[:, None]

    def first(x):
        return 3 * c3 * x**2 + 2 * c2 * x + c1

    def second(x):
        return 6 * c3 * x + 2 * c2

    slope = np.maximum(abs(first(left)), abs(first(right)))
    # A quadratic slope can peak inside the interval, where its derivative is zero.
    with np.errstate(divide="ignore", invalid="ignore"):
        peak = np.where(abs(c3) > 1e-300, -c2 / (3 * c3), np.nan)
    inside = (peak > left) & (peak < right)
    slope = np.where(inside, np.maximum(slope, abs(first(np.where(inside, peak, left)))), slope)
    bend = np.maximum(abs(second(left)), abs(second(right)))
    # Tip tangent over each interval: its value at the middle of four equal
    # parts, plus how far it can change within half a part.
    parts = 4
    tangent = np.zeros(len(ds))
    for part in range(parts):
        at = left + (right - left) * (part + 0.5) / parts
        positions = ((c3 * at + c2) * at + c1) * at + c0
        slopes = first(at)
        for i in range(len(ds)):
            observed = np.linalg.norm(
                geometric_jacobian(robot, positions[i], offset)[:3] @ slopes[i]
            )
            tangent[i] = max(tangent[i], observed)
    for i in range(len(ds)):
        tangent[i] += (
            tcp_acceleration_bound(robot, offset, slope[i], bend[i]) * ds[i] / (2 * parts) + 1e-12
        )
    tiny = 1e-300
    limit = np.minimum(
        np.min((velocity_limits / np.maximum(slope, tiny)) ** 2, axis=1),
        np.minimum(
            (speed / tangent) ** 2,
            # Beyond this a joint exceeds its acceleration limit at constant speed.
            np.min(acceleration_limits / np.maximum(bend, tiny), axis=1),
        ),
    )
    cap = np.minimum(np.r_[np.inf, limit], np.r_[limit, np.inf])
    cap[0] = cap[-1] = 0.0
    # Joint acceleration is q' u + q'' x for path acceleration u and squared
    # path speed x. With u constant over an interval, x is linear in it, so the
    # limits at both interval ends are linear in u: a one-variable problem.
    d1 = np.stack([first(left), first(right)], axis=1)
    d2 = np.stack([second(left), second(right)], axis=1)

    def steepest(i, x, forward):
        """Largest gain (forward) or loss (backward) of x across interval ``i``."""
        step = 2 * ds[i]
        if forward:  # x is known at the left end; the right end has x + step * u.
            factor, known = d1[i] + np.array([[0.0], [step]]) * d2[i], d2[i] * x
        else:  # x is known at the right end; the left end has x - step * u.
            factor, known = d1[i] - np.array([[step], [0.0]]) * d2[i], d2[i] * x
        with np.errstate(divide="ignore", invalid="ignore"):
            high = (np.sign(factor) * acceleration_limits - known) / factor
            low = (-np.sign(factor) * acceleration_limits - known) / factor
        fixed = abs(factor) < tiny
        best = (
            np.min(np.where(fixed, np.inf, high))
            if forward
            else np.max(np.where(fixed, -np.inf, low))
        )
        return step * (best if forward else -best)

    x = cap.copy()
    for i in range(len(ds)):
        x[i + 1] = max(0.0, min(x[i + 1], x[i] + steepest(i, x[i], True)))
    for i in reversed(range(len(ds))):
        x[i] = max(0.0, min(x[i], x[i + 1] + steepest(i, x[i + 1], False)))
    velocities = np.sqrt(x)
    if np.any(velocities[:-1] + velocities[1:] <= 1e-12) or not np.isfinite(velocities).all():
        return None
    # Compose each cubic with its quadratic clock on normalized interval time
    # and bound joint velocity and acceleration by Bernstein coefficients: the
    # polynomial lies inside their range on the whole interval.
    duration = 2 * ds / (velocities[:-1] + velocities[1:])
    p0 = left[:, 0]
    p1 = velocities[:-1] * duration
    p2 = (velocities[1:] ** 2 - velocities[:-1] ** 2) / (4 * ds) * duration**2
    zero = np.zeros_like(p0)
    one = np.array([p0, p1, p2, zero, zero, zero, zero]).T
    two = np.array([p0**2, 2 * p0 * p1, p1**2 + 2 * p0 * p2, 2 * p1 * p2, p2**2, zero, zero]).T
    three = np.array(
        [
            p0**3,
            3 * p0**2 * p1,
            3 * p0 * p1**2 + 3 * p0**2 * p2,
            p1**3 + 6 * p0 * p1 * p2,
            3 * p1**2 * p2 + 3 * p0 * p2**2,
            3 * p1 * p2**2,
            p2**3,
        ]
    ).T
    position = (
        c3[:, :, None] * three[:, None, :]
        + c2[:, :, None] * two[:, None, :]
        + c1[:, :, None] * one[:, None, :]
    )  # The constant term does not affect derivatives.
    powers = np.arange(7)
    rate = (position * powers)[:, :, 1:]
    change = (rate * np.arange(6))[:, :, 1:]
    vmax = np.max(abs(rate @ _BERNSTEIN[5].T) / duration[:, None, None], axis=(0, 2))
    amax = np.max(abs(change @ _BERNSTEIN[4].T) / duration[:, None, None] ** 2, axis=(0, 2))
    return velocities, tangent, vmax, amax


# Clock knots at these tool distances from each stop, doubling until half way,
# let a run ramp to its drawing speed without a knot every fraction of a
# millimetre. Added knots keep at least the gap below from every other knot.
RAMP_KNOT_START_M = 0.001
RAMP_KNOT_GAP_M = 0.00025


def ramp_knots(path, robot, offset):
    """Path parameters for clock knots that resolve the ramps of one run."""
    knots = path.cspl.x
    tips = np.array([robot.forward(q, offset)[:3, 3] for q in path.cspl(knots)])
    arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(tips, axis=0), axis=1))]
    wanted, distance = [arc[-1] / 2], RAMP_KNOT_START_M
    while distance < arc[-1] / 2:
        wanted.extend([distance, arc[-1] - distance])
        distance *= 2
    present, added = list(arc), []
    for value in wanted:
        # A run of one piece always gets its middle: a clock needs three knots.
        if min(abs(value - other) for other in present) >= RAMP_KNOT_GAP_M or (
            len(knots) == 2 and not added and arc[-1] > 0
        ):
            present.append(value)
            added.append(value)
    return np.interp(added, arc, knots)


# An interior clock knot slower than this share of the fastest marks a stall.
STALLED_CLOCK_FRACTION = 0.02


def retime_operation(
    path,
    robot,
    offset,
    velocity_limits,
    acceleration_limits,
    speed,
    *,
    grid_samples=101,
    grid_midpoints=True,
    grid_ramps=False,
):
    """Time one path, refining the clock grid only where a coarse one stalls.

    On a coarse grid the solver can return a valid clock that almost stops at
    an interior knot, taking minutes over a fraction of a millimetre. Such a
    part is retimed on the next finer grid and the shortest clock is kept.
    """
    grids = [(grid_samples, grid_midpoints)]
    for finer in ((0, True), (101, True)):
        if finer[0] >= grids[-1][0] and finer[1] >= grids[-1][1] and finer != grids[-1]:
            grids.append(finer)
    best, failure = None, None
    for samples, midpoints in grids:
        try:
            result = _retime_operation(
                path,
                robot,
                offset,
                velocity_limits,
                acceleration_limits,
                speed,
                grid_samples=samples,
                grid_midpoints=midpoints,
                grid_ramps=grid_ramps,
            )
        except TimingParameterizationError as error:
            # A checked path can always be traversed slowly enough; a numerical
            # failure on one grid is a reason to refine it, not to give up.
            failure = error
            continue
        if best is None or result["duration_s"] < best["duration_s"]:
            best = result
        speeds = np.asarray(result["path_clock"]["speeds"])
        if len(speeds) < 3 or speeds[1:-1].min() >= STALLED_CLOCK_FRACTION * speeds.max():
            break
    if best is None:
        raise failure
    return best


def _retime_operation(
    path,
    robot,
    offset,
    velocity_limits,
    acceleration_limits,
    speed,
    *,
    grid_samples=101,
    grid_midpoints=True,
    grid_ramps=False,
):
    """Time one path with ``scan_clock``. Every spline knot is a clock knot;
    ``grid_midpoints``, ``grid_samples`` uniformly spaced values and
    ``grid_ramps`` add resolution between them. A coarser grid can only
    lengthen the clock: the exact bounds below rescale it.
    """
    deviation = check_position_limits(path, robot.lower, robot.upper)
    knots = path.cspl.x
    # Numerically equivalent knots must not create near-zero clock intervals:
    # tiny differences can amplify solver roundoff into enormous accelerations.
    # Keep every original knot, adding extra samples only when well separated.
    grid = list(knots)
    extra = np.r_[
        (knots[:-1] + knots[1:]) / 2 if grid_midpoints else [],
        np.linspace(0, 1, grid_samples) if grid_samples else [],
        ramp_knots(path, robot, offset) if grid_ramps else [],
    ]
    for value in extra:
        if min(abs(value - present) for present in grid) > 1e-10:
            grid.append(float(value))
    grid = np.sort(grid)
    scanned = scan_clock(path, grid, robot, offset, velocity_limits, acceleration_limits, speed)
    if scanned is None:
        raise TimingParameterizationError("No finite scanned path clock")
    velocities, tangent_bounds, vmax, amax = scanned
    tcp_max = float(np.max(tangent_bounds * np.maximum(velocities[:-1], velocities[1:])))
    if not (np.isfinite(vmax).all() and np.isfinite(amax).all() and np.isfinite(tcp_max)):
        raise TimingParameterizationError("Retiming bounds are nonfinite")
    scale = (
        max(
            1.0,
            float(np.max(vmax / velocity_limits)),
            float(np.sqrt(np.max(amax / acceleration_limits))),
            tcp_max / speed,
        )
        * 1.00001
    )
    if not np.isfinite(scale) or scale > 1e6:
        raise TimingParameterizationError("Retiming correction is numerically ill-conditioned")
    times = np.r_[0.0, np.cumsum(2 * np.diff(grid) / (velocities[:-1] + velocities[1:]))]
    return {
        "duration_s": float(times[-1] * scale),
        "path_clock": {
            "s": grid.tolist(),
            "t_s": (times * scale).tolist(),
            "speeds": (velocities / scale).tolist(),
        },
        "bounds": {
            "velocity_rad_s": (vmax / scale).tolist(),
            "acceleration_rad_s2": (amax / scale**2).tolist(),
            "tcp_speed_m_s": tcp_max / scale,
        },
        "maximum_joint_deviation_rad": deviation.tolist(),
        "time_scale_correction": scale,
        "retiming_method": "forward_backward_scan",
    }
