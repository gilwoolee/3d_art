"""Smooth-path retiming and pen-event schedules for simulation.

Natural cubic joint splines preserve input waypoints, but change the path
between them. Earlier linear-path collision certificates do not transfer.
No artifact from this module is an executable hardware program.
"""

import argparse
from functools import lru_cache
from importlib.metadata import version
from pathlib import Path

import numpy as np
import toppra as ta
import yaml
from numpy.polynomial import Polynomial
from scipy.interpolate import PchipInterpolator

from .io import artifact, read_json, sha256, stage_update, write_json
from .kinematics import geometric_jacobian, rigid_transform
from .robot import SerialArm


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


def make_path(positions, tcp_points, *, interpolation="natural", subdivisions=1):
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
    result = ta.SplineInterpolator(knots, q, bc_type="natural")
    if interpolation == "pchip":
        result.cspl = PchipInterpolator(knots, q, axis=0)
        result.cspld, result.cspldd = result.cspl.derivative(), result.cspl.derivative(2)
    elif interpolation != "natural":
        raise ValueError(f"Unsupported interpolation: {interpolation}")
    return result


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


class TcpSpeedConstraint(ta.constraint.LinearConstraint):
    """Bound path tangent over each interval, then constrain both clock endpoints."""

    def __init__(self, robot, offset, speed):
        super().__init__()
        self.robot, self.offset, self.speed = robot, offset, speed
        self.dof = len(robot.names)

    def compute_constraint_params(self, path, gridpoints):
        tangent = []
        for left, right in zip(gridpoints[:-1], gridpoints[1:], strict=True):
            piece = min(len(path.cspl.x) - 2, np.searchsorted(path.cspl.x, left, side="right") - 1)
            polys = [Polynomial(path.cspl.c[:, piece, j][::-1]) for j in range(path.dof)]
            bound = 0.0
            samples = np.linspace(left, right, 5)
            for a, b in zip(samples[:-1], samples[1:], strict=True):
                middle = (a + b) / 2
                first = [
                    maximum_absolute(p.deriv(), a - path.cspl.x[piece], b - path.cspl.x[piece])
                    for p in polys
                ]
                second = [
                    maximum_absolute(p.deriv(2), a - path.cspl.x[piece], b - path.cspl.x[piece])
                    for p in polys
                ]
                speed = np.linalg.norm(
                    geometric_jacobian(self.robot, path(middle), self.offset)[:3] @ path(middle, 1)
                )
                bound = max(
                    bound,
                    speed
                    + tcp_acceleration_bound(self.robot, self.offset, first, second) * (b - a) / 2,
                )
            tangent.append(bound + 1e-12)
        self.tangent_bounds = np.array(tangent)
        # Constant path acceleration makes the maximum squared path speed occur
        # at an endpoint. Each endpoint must satisfy both adjacent intervals.
        endpoint_bounds = np.maximum(np.r_[0, self.tangent_bounds], np.r_[self.tangent_bounds, 0])
        bounds = np.column_stack([np.zeros(len(gridpoints)), (self.speed / endpoint_bounds) ** 2])
        return None, None, None, None, None, None, bounds


class CubicAccelerationConstraint(ta.constraint.LinearConstraint):
    """Bound acceleration throughout each cubic interval, including knot sides.

    With constant path acceleration u and initial squared speed x, each joint's
    acceleration is a quadratic in normalized interval position. Its three
    Bernstein control values are linear in u and x. Bounding all three bounds
    the entire interval without assuming that the spline is C2 at its knots.
    """

    def __init__(self, limits):
        super().__init__()
        self.identical = True
        self.limits = np.asarray(limits, float)
        self.dof = len(self.limits)

    def compute_constraint_params(self, path, gridpoints):
        if path.dof != self.dof:
            raise ValueError("Acceleration limits must match path joints")
        left = np.asarray(gridpoints[:-1])
        ds = np.diff(gridpoints)[:, None]
        indices = np.searchsorted(path.cspl.x, left, side="right") - 1
        if np.any(np.asarray(gridpoints[1:]) > path.cspl.x[indices + 1] + 1e-10):
            raise ValueError("Acceleration grid must include every cubic knot")
        first = path.cspl(left, 1)
        second = path.cspl(left, 2)
        third = path.cspl(left, 3)
        a = np.concatenate(
            [first, first + 1.5 * ds * second, first + 3 * ds * second + 2.5 * ds**2 * third],
            axis=1,
        )
        b = np.concatenate([second, second + 0.5 * ds * third, second + ds * third], axis=1)
        # TOPPRA expects a row at the terminal grid point as well. Its boundary
        # speed is prescribed; the interval certificates above do the work.
        a = np.vstack([a, np.tile(path.cspl(gridpoints[-1], 1), 3)])
        b = np.vstack([b, np.tile(path.cspl(gridpoints[-1], 2), 3)])
        identity = np.eye(3 * self.dof)
        return (
            a,
            b,
            np.zeros_like(a),
            np.vstack([identity, -identity]),
            np.tile(self.limits, 6),
            None,
            None,
        )


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


def dynamics_bounds(pieces, robot):
    velocity, acceleration = np.zeros(len(robot.names)), np.zeros(len(robot.names))
    for _, duration, polynomials in pieces:
        v = [poly.deriv() for poly in polynomials]
        a = [poly.deriv(2) for poly in polynomials]
        velocity = np.maximum(velocity, [maximum_absolute(poly, 0, duration) for poly in v])
        acceleration = np.maximum(acceleration, [maximum_absolute(poly, 0, duration) for poly in a])
    return velocity, acceleration


# An interior clock knot slower than this share of the fastest marks a stall.
STALLED_CLOCK_FRACTION = 0.02


def retime_operation(
    operation,
    robot,
    offset,
    velocity_limits,
    acceleration_limits,
    speed,
    period,
    *,
    sample_output=True,
    path=None,
    continuous_acceleration=False,
    grid_samples=101,
    grid_midpoints=True,
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
                operation, robot, offset, velocity_limits, acceleration_limits, speed, period,
                sample_output=sample_output, path=path,
                continuous_acceleration=continuous_acceleration,
                grid_samples=samples, grid_midpoints=midpoints,
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
    operation,
    robot,
    offset,
    velocity_limits,
    acceleration_limits,
    speed,
    period,
    *,
    sample_output=True,
    path=None,
    continuous_acceleration=False,
    grid_samples=101,
    grid_midpoints=True,
):
    """Time one path. Every spline knot is a clock knot; ``grid_midpoints`` and
    ``grid_samples`` uniformly spaced values add resolution between them. A
    coarser grid can only lengthen the clock: the exact bounds below rescale it.
    """
    if path is None:
        path = make_path(operation["positions"], operation["tcp_path_m"])
    deviation = check_position_limits(path, robot.lower, robot.upper)
    knots = path.cspl.x
    # Numerically equivalent knots must not create near-zero clock intervals:
    # tiny differences can amplify solver roundoff into enormous accelerations.
    # Keep every original knot, adding extra samples only when well separated.
    grid = list(knots)
    extra = np.r_[
        (knots[:-1] + knots[1:]) / 2 if grid_midpoints else [],
        np.linspace(0, 1, grid_samples) if grid_samples else [],
    ]
    for value in extra:
        if min(abs(value - present) for present in grid) > 1e-10:
            grid.append(float(value))
    grid = np.sort(grid)
    tcp_constraint = TcpSpeedConstraint(robot, offset, speed)
    constraints = [
        ta.constraint.JointVelocityConstraint(velocity_limits),
        (
            CubicAccelerationConstraint(acceleration_limits)
            if continuous_acceleration
            else ta.constraint.JointAccelerationConstraint(
                acceleration_limits,
                discretization_scheme=ta.constraint.DiscretizationType.Interpolation,
            )
        ),
        tcp_constraint,
    ]
    solver = ta.algorithm.TOPPRA(constraints, path, gridpoints=grid, solver_wrapper="seidel")
    _, velocities, _ = solver.compute_parameterization(0, 0)
    method, fallback_reason = "toppra", None
    if velocities is None or not np.isfinite(velocities).all():
        fallback_reason = "TOPPRA returned no finite parameterization"
    else:
        velocities = np.maximum(velocities, 0)
        if np.any(velocities[:-1] + velocities[1:] <= 1e-12):
            fallback_reason = "TOPPRA returned a stationary nonzero path interval"
    if fallback_reason is not None:
        # A fixed positive interior speed profile supplies a finite traversal of
        # the same spline when the optimizer fails numerically. This is only a
        # proposal: exact polynomial dynamics and TCP bounds below determine
        # its required global slowdown, with every original limit unchanged.
        method = "bounded_tent_speed_fallback"
        velocities = np.sqrt(np.maximum(0.0, np.minimum(grid - grid[0], grid[-1] - grid)))
        if (
            len(grid) < 3
            or not np.isfinite(velocities).all()
            or np.any(velocities[:-1] + velocities[1:] <= 1e-12)
        ):
            raise TimingParameterizationError("No finite fallback path clock found")
    pieces = clock_polynomials(path, grid, velocities)
    vmax, amax = dynamics_bounds(pieces, robot)
    tcp_max = float(
        np.max(tcp_constraint.tangent_bounds * np.maximum(velocities[:-1], velocities[1:]))
    )
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
    durations = np.array([piece[1] * scale for piece in pieces])
    times = np.r_[0.0, np.cumsum(durations)]
    display_period = max(period, times[-1] / 2000)
    sample_times = (
        np.unique(np.r_[times, np.arange(0, times[-1], display_period)])
        if sample_output
        else np.array([])
    )
    q, qd, qdd, tcp, tcp_speeds = [], [], [], [], []
    for t in sample_times:
        index = min(len(pieces) - 1, np.searchsorted(times, t, side="right") - 1)
        u = (t - times[index]) / scale
        polynomials = pieces[index][2]
        position = np.array([poly(u) for poly in polynomials])
        velocity = np.array([poly.deriv()(u) / scale for poly in polynomials])
        acceleration = np.array([poly.deriv(2)(u) / scale**2 for poly in polynomials])
        q.append(position.tolist())
        qd.append(velocity.tolist())
        qdd.append(acceleration.tolist())
        tcp.append(robot.forward(position, offset)[:3, 3].tolist())
        tcp_speeds.append(
            float(np.linalg.norm(geometric_jacobian(robot, position, offset)[:3] @ velocity))
        )
    return {
        "duration_s": float(times[-1]),
        "times_s": sample_times.tolist(),
        "positions": q,
        "velocities": qd,
        "accelerations": qdd,
        "tcp_path_m": tcp,
        "tcp_speeds_m_s": tcp_speeds,
        "geometric_path": {"knots": knots.tolist(), "coefficients": path.cspl.c.tolist()},
        "path_clock": {
            "s": grid.tolist(),
            "t_s": times.tolist(),
            "speeds": (velocities / scale).tolist(),
        },
        "bounds": {
            "velocity_rad_s": (vmax / scale).tolist(),
            "acceleration_rad_s2": (amax / scale**2).tolist(),
            "tcp_speed_m_s": tcp_max / scale,
        },
        "maximum_joint_deviation_rad": deviation.tolist(),
        "time_scale_correction": scale,
        "retiming_method": method,
        "retiming_fallback_reason": fallback_reason,
    }


def check_profile(profile, process, dof):
    acceleration = np.asarray(profile["joint_acceleration_rad_s2"], float)
    if (
        acceleration.shape != (dof,)
        or not np.isfinite(acceleration).all()
        or np.any(acceleration <= 0)
    ):
        raise ValueError("Positive finite acceleration limits are required for each joint")
    for name in ("travel_speed_m_s", "sample_period_s", "target_seconds", "velocity_fraction"):
        if not np.isfinite(profile[name]) or profile[name] <= 0:
            raise ValueError(f"Timing {name} must be positive and finite")
    if profile["velocity_fraction"] > 1:
        raise ValueError("Velocity fraction cannot exceed URDF limits")
    for name in ("startup_s", "cooling_s", "shutdown_s"):
        value = profile[name]
        if value is not None and (not np.isfinite(value) or value < 0):
            raise ValueError(f"Timing {name} must be null or nonnegative and finite")
    for name in ("nominal_speed_m_s", "start_lag_s", "stop_lag_s", "join_s"):
        if (
            not np.isfinite(process[name])
            or process[name] < 0
            or (name == "nominal_speed_m_s" and process[name] == 0)
        ):
            raise ValueError(f"Invalid process timing: {name}")


def _build_schedule(run, profile_path=Path("configs/timing.yaml")):
    run, profile_path = Path(run), Path(profile_path)
    profile = yaml.safe_load(profile_path.read_text())
    sequence = read_json(run / "sequence_motion.json")
    if sequence["status"] != "GEOMETRIC_SEQUENCE_PREVIEW":
        raise ValueError("A complete geometric sequence is required before timing")
    for name, digest in sequence["inputs"].items():
        if sha256(run / name) != digest:
            raise ValueError(f"Sequence input changed before timing: {name}")
    robot = SerialArm(run / "robot.urdf")
    offset = rigid_transform(sequence["flange_to_tcp"])
    process = read_json(run / "fabrication_config.json")["process"]
    check_profile(profile, process, len(robot.names))
    write_json(run / "timing_config.json", profile)
    output = artifact(
        "timed_schedule",
        status="RUNNING",
        simulation_only=True,
        executable=False,
        frame="robot_base",
        units="m",
        angle_units="rad",
        time_units="s",
        joint_names=robot.names,
        velocity_limits_rad_s=(robot.velocity * profile["velocity_fraction"]).tolist(),
        acceleration_limits_rad_s2=profile["joint_acceleration_rad_s2"],
        profile=profile,
        process=process,
        flange_to_tcp=offset.tolist(),
        retimer={
            "library": "toppra",
            "version": version("toppra"),
            "path": "natural_cubic_joint_spline",
            "clock": "piecewise_constant_path_acceleration",
        },
        inputs={
            name: sha256(run / name)
            for name in (
                "sequence_motion.json",
                "robot.urdf",
                "fabrication_config.json",
                "timing_config.json",
            )
        },
        operations=[],
        events=[],
        timing_conflicts=[],
        modeled_duration_s=0.0,
        unknown_durations=[
            name for name in ("startup_s", "cooling_s", "shutdown_s") if profile[name] is None
        ],
        geometry_validation={
            "status": "REQUIRES_SMOOTH_PATH_AUDIT",
            "reason": "Cubic interpolation changes the previously checked joint-linear path",
        },
        limitations=[
            "Uncalibrated dynamics and pen settings are development assumptions",
            "Timing excludes unknown durations listed in this artifact",
            "Smoothed paths require new collision and tracking validation",
            "No validated flow, cooling, jerk, controller interpolation or hardware execution",
        ],
    )
    stage_update(
        run, "timing", "running", message="Retiming smooth paths and scheduling pen events"
    )
    cursor = 0.0
    q = sequence["parking_pose"]["positions"]

    def hold(kind, seconds, unknown=False, stroke_id=None):
        nonlocal cursor
        duration = seconds or 0.0
        output["operations"].append(
            {
                "kind": kind,
                "start_s": cursor,
                "end_s": cursor + duration,
                "duration_s": duration,
                "unknown_duration": unknown,
                "stroke_id": stroke_id,
                "hold_positions": list(q),
                "extrusion_expected": False,
            }
        )
        cursor += duration

    hold("STARTUP", profile["startup_s"], profile["startup_s"] is None)
    for operation in sequence["operations"]:
        identity = operation["stroke_id"]
        drawing = operation["kind"] == "DRAW"
        if not np.allclose(operation["positions"][0], q, atol=1e-8, rtol=0):
            raise ValueError(f"Discontinuous operation: {operation['id']}")
        if drawing:
            hold("JOIN", process["join_s"], stroke_id=identity)
            output["events"].append(
                {"time_s": cursor, "command": "START_FEED", "stroke_id": identity}
            )
            hold("START_RESPONSE", process["start_lag_s"], stroke_id=identity)
        speed = process["nominal_speed_m_s"] if drawing else profile["travel_speed_m_s"]
        result = retime_operation(
            operation,
            robot,
            offset,
            robot.velocity * profile["velocity_fraction"],
            np.asarray(profile["joint_acceleration_rad_s2"]),
            speed,
            profile["sample_period_s"],
        )
        output["operations"].append(
            {
                **result,
                "id": operation["id"],
                "stroke_id": identity,
                "kind": operation["kind"],
                "start_s": cursor,
                "end_s": cursor + result["duration_s"],
                "extrusion_expected": drawing,
            }
        )
        if drawing:
            if process["stop_lag_s"] > result["duration_s"]:
                output["timing_conflicts"].append(
                    {
                        "code": "ASSUMED_STOP_LAG_EXCEEDS_STROKE",
                        "stroke_id": identity,
                        "duration_s": result["duration_s"],
                        "stop_lag_s": process["stop_lag_s"],
                        "message": "Merge this short stroke or calibrate a different pen response; "
                        "the clamped stop event is a preview, not a valid feed schedule",
                    }
                )
            output["events"].append(
                {
                    "time_s": cursor + max(0.0, result["duration_s"] - process["stop_lag_s"]),
                    "command": "STOP_FEED",
                    "stroke_id": identity,
                }
            )
        cursor += result["duration_s"]
        q = result["positions"][-1]
        if drawing:
            hold("COOL", profile["cooling_s"], profile["cooling_s"] is None, identity)
        output["modeled_duration_s"] = cursor
        write_json(run / "timed_schedule.json", output)
        print(f"Timed {operation['id']}: {result['duration_s']:.2f} s", flush=True)
    hold("SHUTDOWN", profile["shutdown_s"], profile["shutdown_s"] is None)
    output["modeled_duration_s"] = cursor
    output["within_modeled_budget"] = cursor <= profile["target_seconds"]
    output["status"] = "TIMED_SIMULATION_PREVIEW"
    output["events"].sort(key=lambda event: event["time_s"])
    write_json(run / "timed_schedule.json", output)
    stage_update(
        run,
        "timing",
        "partial",
        artifact="timed_schedule.json",
        message="Smooth-path timing preview; geometry, pen and controller validation pending",
    )
    return output


def build_schedule(run, profile_path=Path("configs/timing.yaml")):
    try:
        return _build_schedule(run, profile_path)
    except Exception as error:
        run = Path(run)
        path = run / "timed_schedule.json"
        if path.is_file():
            partial = read_json(path)
            partial.update(status="INCOMPLETE_TIMING_PREVIEW", error=str(error))
            write_json(path, partial)
        if (run / "run.json").is_file():
            stage_update(run, "timing", "failed", message=str(error))
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, default=Path("runs/palace_15cm_selected"))
    parser.add_argument("--profile", type=Path, default=Path("configs/timing.yaml"))
    args = parser.parse_args()
    result = build_schedule(args.run, args.profile)
    print(f"{result['status']}: {result['modeled_duration_s'] / 60:.1f} modeled minutes")


if __name__ == "__main__":
    main()
