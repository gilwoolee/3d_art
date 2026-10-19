"""Exact serialized clocks for linear travel and continuous spline drawing.

The default quintic clock stops at edge boundaries. The opt-in drawing clock
uses midpoint-refined, shape-preserving cubic interpolation and TOPPRA. Its
changed geometry requires fresh tracking and collision checks. Neither clock
establishes the behavior of an uncalibrated physical pen.
"""

from copy import copy
from dataclasses import dataclass
from time import monotonic

import numpy as np
from numpy.polynomial import Polynomial
from scipy.interpolate import PPoly

from ..kinematics import geometric_jacobian
from ..timing import clock_polynomials, make_path, retime_operation, tcp_acceleration_bound
from .smooth_audit import PolynomialPiece


def smoothstep(u):
    return u**3 * (10 + u * (-15 + 6 * u))


def coalesce_travel(points):
    """Remove subdivisions of the same directed joint-space line, not corners."""
    kept = []
    for point in points:
        if kept and np.linalg.norm(point - kept[-1]) < 1e-12:
            continue
        if len(kept) >= 2:
            direction = kept[-1] - kept[-2]
            extension = point - kept[-1]
            fraction = np.dot(extension, direction) / np.dot(direction, direction)
            if fraction >= 0 and np.linalg.norm(extension - fraction * direction) < 1e-10:
                kept[-1] = point
                continue
        kept.append(point)
    return np.asarray(kept)


def tcp_tangent_bound(arm, first, second):
    """Bound |d TCP / d s| along one joint-linear edge on a uniform grid."""
    delta = second - first
    second_derivative = tcp_acceleration_bound(
        arm.robot, arm.flange_to_tcp, abs(delta), np.zeros(len(delta))
    )
    subdivisions = min(128, max(1, int(np.ceil(np.max(abs(delta)) / 0.03))))
    sampled = max(
        np.linalg.norm(
            geometric_jacobian(arm.robot, first + delta * s, arm.flange_to_tcp)[:3] @ delta
        )
        for s in np.linspace(0, 1, subdivisions + 1)
    )
    return sampled + second_derivative / (2 * subdivisions)


def polynomial_tcp_speed_bound(edge, arm, limit, *, deadline=None):
    """Bound the actual polynomial's tip speed, refining only unresolved spans."""
    piece = edge.piece()
    duration = edge.end_s - edge.start_s

    def visit(a, b, depth):
        if deadline is not None and monotonic() >= deadline:
            raise TimeoutError("Tip-speed verification exceeded its planning budget")
        first, second = piece.derivatives(a, b)
        middle = (a + b) / 2
        tangent = piece.velocity(middle)
        observed = np.linalg.norm(
            geometric_jacobian(arm.robot, piece.position(middle), arm.flange_to_tcp)[:3] @ tangent
        )
        bound = (
            observed
            + tcp_acceleration_bound(arm.robot, arm.flange_to_tcp, first, second) * (b - a) / 2
        ) / duration
        # Near a fast endpoint, the whole-interval acceleration enclosure can
        # remain loose after ten bisections even for a valid TOPPRA clock.
        # Refine that enclosure further; keep the speed limit and tolerance.
        if bound > limit + 1e-8 and observed / duration <= limit + 1e-8 and depth < 16:
            return max(visit(a, middle, depth + 1), visit(middle, b, depth + 1))
        return float(bound)

    return visit(0, 1, 0)


# While the pen draws, approaches or withdraws, the arm must follow the tip and
# not change posture: a seven-joint arm can swing its elbow with the tip still.
# Degrees of the largest joint motion allowed per millimetre of tip travel.
# Verified builds stay below 0.25; travel between actions is exempt.
POSTURE_LIMIT_DEG_PER_MM = 1.0
POSTURE_KINDS = frozenset({"DRAW", "APPROACH", "WITHDRAW"})


def posture_change(path, arm, limit_deg_per_mm=POSTURE_LIMIT_DEG_PER_MM):
    """The first edge or run of edges that changes posture, or None.

    Each edge is held to the limit over at least 0.1 mm of tip travel, and each
    unbroken run of drawing, approach or withdraw edges to the limit over its
    whole travel plus 1 mm, so a swing cannot hide in many small edges.
    """
    limit = np.radians(limit_deg_per_mm) * 1000

    def finish(run):
        if run and run["joints"].max() > limit * (run["travel"] + 0.001):
            return {
                "status": "POSTURE_CHANGE",
                "scope": "run",
                "kind": run["kind"],
                "first_edge": run["first"],
                "joint_motion_deg": float(np.degrees(run["joints"].max())),
                "tip_travel_mm": 1000 * run["travel"],
                "limit_deg_per_mm": limit_deg_per_mm,
            }
        return None

    run = None
    for index, edge in enumerate(path.edges):
        if edge.kind not in POSTURE_KINDS:
            failure, run = finish(run), None
            if failure:
                return failure
            continue
        if edge.coefficients is None:
            joints = np.abs(edge.q1 - edge.q0)
        else:
            ranges = edge.piece().ranges(0, 1)
            joints = ranges[:, 1] - ranges[:, 0]
        ends = [
            arm.robot.forward(q, arm.flange_to_tcp)[:3, 3] if tcp is None else tcp
            for q, tcp in ((edge.q0, edge.tcp0), (edge.q1, edge.tcp1))
        ]
        travel = float(np.linalg.norm(ends[1] - ends[0]))
        if joints.max() > limit * max(travel, 0.0001):
            return {
                "status": "POSTURE_CHANGE",
                "scope": "edge",
                "kind": edge.kind,
                "first_edge": index,
                "joint_motion_deg": float(np.degrees(joints.max())),
                "tip_travel_mm": 1000 * travel,
                "limit_deg_per_mm": limit_deg_per_mm,
            }
        if run is None or run["kind"] != edge.kind:
            failure = finish(run)
            if failure:
                return failure
            run = {
                "kind": edge.kind,
                "first": index,
                "joints": np.zeros(len(joints)),
                "travel": 0.0,
            }
        run["joints"] += joints
        run["travel"] += travel
    return finish(run)


@dataclass
class ClockEdge:
    start_s: float
    end_s: float
    q0: np.ndarray
    q1: np.ndarray
    kind: str
    stroke: str | None
    contact: np.ndarray | None = None
    radius_m: float = 0.0
    tcp0: np.ndarray | None = None
    tcp1: np.ndarray | None = None
    # Ascending powers of normalized edge time, one row per joint.
    coefficients: list[list[float]] | None = None
    table_index: int | None = None
    table_to_index: int | None = None

    def __getstate__(self):
        return {name: value for name, value in self.__dict__.items() if name != "_piece"}

    def piece(self, a=None, b=None):
        # An edge is not edited after construction; checks ask for its piece often.
        piece = self.__dict__.get("_piece")
        if piece is None:
            if self.coefficients is None:
                polynomials = [
                    Polynomial([first, 0, 0, 10 * d, -15 * d, 6 * d])
                    for first, d in zip(self.q0, self.q1 - self.q0, strict=True)
                ]
            else:
                polynomials = [Polynomial(c) for c in self.coefficients]
            piece = self.__dict__["_piece"] = PolynomialPiece(polynomials)
        if (a is None and b is None) or (a == self.start_s and b == self.end_s):
            return piece
        width = self.end_s - self.start_s
        return piece.subpiece((a - self.start_s) / width, (b - self.start_s) / width)

    def position(self, time):
        u = float(np.clip((time - self.start_s) / (self.end_s - self.start_s), 0, 1))
        if self.coefficients is not None:
            # Evaluate the current mutable coefficients directly. Horner uses
            # the same interpolant as Polynomial without rebuilding seven
            # Polynomial objects for every FK/collision sample. Joint degrees
            # may differ; do not retain a stale coefficient or piece cache.
            result = np.empty(len(self.coefficients), dtype=float)
            for i, coefficients in enumerate(self.coefficients):
                value = 0.0
                for coefficient in reversed(coefficients):
                    value = value * u + coefficient
                result[i] = value
            return result
        return self.q0 + (self.q1 - self.q0) * smoothstep(u)

    def deviation(self, a, b):
        if self.coefficients is not None:
            width = self.end_s - self.start_s
            return self.piece().deviation((a - self.start_s) / width, (b - self.start_s) / width)
        middle = self.position((a + b) / 2)
        return np.maximum(abs(self.position(a) - middle), abs(self.position(b) - middle))

    def record(self):
        result = {
            "start_s": self.start_s,
            "end_s": self.end_s,
            "q0": self.q0.tolist(),
            "q1": self.q1.tolist(),
            "kind": self.kind,
            "stroke_id": self.stroke,
            "contact_m": None if self.contact is None else self.contact.tolist(),
            "material_radius_m": self.radius_m,
            "tcp0_m": None if self.tcp0 is None else self.tcp0.tolist(),
            "tcp1_m": None if self.tcp1 is None else self.tcp1.tolist(),
        }
        if self.coefficients is not None:
            result["coefficients"] = [list(row) for row in self.coefficients]
        if self.table_index is not None:
            result["table_index"] = self.table_index
        if self.table_to_index is not None:
            result["table_to_index"] = self.table_to_index
        return result


class ClockPath:
    def __init__(self, edges):
        self.edges = tuple(edges)
        self.ends = np.array([edge.end_s for edge in self.edges])
        if not self.edges:
            raise ValueError("A clock path needs at least one edge")
        self.duration_s = float(self.edges[-1].end_s)
        self.metadata = {}

    def edge_at(self, time):
        return self.edges[
            min(len(self.edges) - 1, int(np.searchsorted(self.ends, time, side="right")))
        ]

    def position(self, time):
        return self.edge_at(time).position(time)

    @classmethod
    def from_record(cls, data):
        if data["clock"] not in {
            "quintic_stop_at_geometric_knots",
            "continuous_drawing_mixed_clock",
        }:
            raise ValueError("Unsupported saved path clock")
        edges = []
        for row in data["edges"]:
            edge = ClockEdge(
                row["start_s"],
                row["end_s"],
                np.asarray(row["q0"]),
                np.asarray(row["q1"]),
                row["kind"],
                row["stroke_id"],
                None if row.get("contact_m") is None else np.asarray(row["contact_m"]),
                row["material_radius_m"],
                None if row.get("tcp0_m") is None else np.asarray(row["tcp0_m"]),
                None if row.get("tcp1_m") is None else np.asarray(row["tcp1_m"]),
                row.get("coefficients"),
                row.get("table_index"),
                row.get("table_to_index"),
            )
            if (
                edge.table_index not in (None, 0)
                or edge.table_to_index is not None
                or edge.kind in ("INDEX_TURN", "TABLE_SETTLE")
            ):
                raise ValueError("A fixed build plate never turns")
            if (
                not np.isfinite([edge.start_s, edge.end_s]).all()
                or edge.end_s <= edge.start_s
                or edge.start_s < 0
                or edge.q0.ndim != 1
                or len(edge.q0) == 0
                or edge.q0.shape != edge.q1.shape
                or not np.isfinite([edge.q0, edge.q1]).all()
            ):
                raise ValueError("Malformed saved clock edge")
            if edge.coefficients is not None:
                if edge.kind != "DRAW" or len(edge.coefficients) != len(edge.q0):
                    raise ValueError(
                        "Polynomial clock requires one coefficient row per drawing joint"
                    )
                piece = edge.piece()
                if not all(
                    np.allclose(piece.position(t), q, atol=1e-9, rtol=0)
                    for t, q in ((0, edge.q0), (1, edge.q1))
                ):
                    raise ValueError("Polynomial endpoints do not match saved joint positions")
            if edges and (
                abs(edges[-1].end_s - edge.start_s) > 1e-9
                or not np.allclose(edges[-1].q1, edge.q0, atol=1e-9, rtol=0)
            ):
                raise ValueError("Discontinuous saved clock path")
            if edges and (edge.coefficients is not None or edges[-1].coefficients is not None):
                prior = edges[-1]
                before = prior.piece().velocity(1) / (prior.end_s - prior.start_s)
                after = edge.piece().velocity(0) / (edge.end_s - edge.start_s)
                if not np.allclose(before, after, atol=1e-6, rtol=1e-6):
                    raise ValueError("Discontinuous saved joint velocity")
            edges.append(edge)
        path = cls(edges)
        for edge, value in ((edges[0], 0), (edges[-1], 1)):
            if edge.coefficients is not None and np.any(
                abs(edge.piece().velocity(value) / (edge.end_s - edge.start_s)) > 1e-6
            ):
                raise ValueError("Saved path must start and finish at rest")
        if edges[0].start_s != 0 or abs(path.duration_s - data["duration_s"]) > 1e-9:
            raise ValueError("Saved path duration does not match edges")
        path.metadata = {
            key: value for key, value in data.items() if key not in {"clock", "duration_s", "edges"}
        }
        return path

    def record(self):
        return {
            "clock": (
                "continuous_drawing_mixed_clock"
                if any(e.coefficients is not None for e in self.edges)
                else "quintic_stop_at_geometric_knots"
            ),
            "duration_s": self.duration_s,
            "edges": [edge.record() for edge in self.edges],
            **self.metadata,
        }


def restrict_spline(path, first, last):
    """Restrict a checked spline at its knots without fitting new geometry."""
    if not 0 <= first < last < len(path.cspl.x):
        raise ValueError("Invalid drawing spline interval")
    result = copy(path)
    start, end = path.cspl.x[[first, last]]
    width = end - start
    knots = (path.cspl.x[first : last + 1] - start) / width
    coefficients = path.cspl.c[:, first:last].copy()
    for index in range(len(coefficients)):
        coefficients[index] *= width ** (len(coefficients) - 1 - index)
    result.cspl = PPoly(coefficients, knots)
    result.cspld, result.cspldd = result.cspl.derivative(), result.cspl.derivative(2)
    result.ss_waypoints = knots
    result._q_waypoints = result.cspl(knots)
    return result


def split_drawing_operations(operation):
    """Stop at true source corners and hold in place, without pen restarts elsewhere."""
    positions = np.asarray(operation["positions"])
    cuts = sorted(set([0, len(positions) - 1, *operation.get("break_indices", [])]))
    if cuts[0] != 0 or cuts[-1] != len(positions) - 1:
        raise ValueError("Drawing break lies outside its joint path")
    holds = operation.get("vertex_holds", {})
    for first, last in zip(cuts[:-1], cuts[1:], strict=True):
        part = {
            key: value
            for key, value in operation.items()
            if key not in {"break_indices", "vertex_holds", "stop_lag_s", "start_lag_s"}
        }
        part["positions"] = positions[first : last + 1]
        if operation.get("path") is not None:
            part["path"] = restrict_spline(operation["path"], first, last)
        yield part
        if last == len(positions) - 1 or not holds.get(last):
            continue
        contact = np.asarray(holds[last]["contact_m"])
        stop_lag = operation.get("stop_lag_s", 0.0)
        duration = holds[last]["duration_s"]
        # Cooling starts at arrival. The feed's decay and the response to
        # restarting it are both part of this elapsed time: the restart is
        # commanded early enough for material to arrive as the hold ends.
        start_lag = operation.get("start_lag_s", 0.0)
        for kind, seconds in (
            ("PEN_STOP_WAIT", stop_lag),
            ("VERTEX_COOL", max(0.0, duration - stop_lag - start_lag)),
            ("PEN_START_WAIT", start_lag),
        ):
            if seconds > 0:
                yield {
                    "kind": kind,
                    "positions": [positions[last]],
                    "hold_s": seconds,
                    "contact_m": contact,
                    "stroke_id": operation.get("stroke_id"),
                }


def timed_path(
    operations,
    arm,
    velocity,
    acceleration,
    drawing_speed,
    travel_speed,
    *,
    drawing_clock="quintic_stop_at_geometric_knots",
    acceleration_constraint="interpolated",
    drawing_capture=None,
    drawing_reuse=None,
    grid_samples=101,
    grid_midpoints=True,
    solver="toppra",
    grid_ramps=False,
):
    """Analytically time checked edges without changing their geometric sweep.

    ``drawing_capture`` collects each retimed DRAW part in order. Supplying the
    same list as ``drawing_reuse`` assembles a longer clock around those exact
    polynomials instead of retiming them, so deposits predicted from a local
    drawing clock remain those of the complete action.
    """
    edges, elapsed, diagnostics = [], 0.0, []
    reused = 0
    if drawing_clock not in {"quintic_stop_at_geometric_knots", "cubic_toppra"}:
        raise ValueError(f"Unsupported drawing clock: {drawing_clock}")
    if acceleration_constraint not in {"interpolated", "cubic_bernstein"}:
        raise ValueError(f"Unsupported acceleration constraint: {acceleration_constraint}")
    expanded = []
    for operation in operations:
        expanded.extend(
            split_drawing_operations(operation) if operation.get("break_indices") else [operation]
        )
    for op in expanded:
        q = np.asarray(op["positions"], float)
        if op["kind"] == "DRAW" and drawing_clock == "cubic_toppra":
            points = [arm.robot.forward(joints, arm.flange_to_tcp)[:3, 3] for joints in q]
            if drawing_reuse is not None:
                if reused >= len(drawing_reuse) or not np.array_equal(
                    drawing_reuse[reused]["positions"], q
                ):
                    raise ValueError("Retained drawing clock belongs to a different joint path")
                timed = drawing_reuse[reused]
                reused += 1
            else:
                path = op.get("path")
                if path is None:
                    path = make_path(q, points, interpolation="pchip", subdivisions=2)
                result = retime_operation(
                    {"positions": q, "tcp_path_m": points},
                    arm.robot,
                    arm.flange_to_tcp,
                    velocity,
                    acceleration,
                    drawing_speed,
                    0.2,
                    sample_output=False,
                    path=path,
                    continuous_acceleration=acceleration_constraint == "cubic_bernstein",
                    grid_samples=grid_samples,
                    grid_midpoints=grid_midpoints,
                    solver=solver,
                    grid_ramps=grid_ramps,
                )
                timed = {
                    "positions": q,
                    "duration_s": result["duration_s"],
                    "pieces": clock_polynomials(
                        path,
                        np.array(result["path_clock"]["s"]),
                        np.array(result["path_clock"]["speeds"]),
                    ),
                    "diagnostic": {
                        "stroke_id": op.get("stroke_id"),
                        "time_scale_correction": result["time_scale_correction"],
                        "retiming_method": result["retiming_method"],
                        "retiming_fallback_reason": result["retiming_fallback_reason"],
                        "bounds": result["bounds"],
                        "acceleration_constraint": acceleration_constraint,
                    },
                }
            if drawing_capture is not None:
                drawing_capture.append(timed)
            pieces, result = timed["pieces"], timed
            diagnostics.append(timed["diagnostic"])
            for start, duration, polynomials in pieces:
                normalized = [p(Polynomial([0, duration])) for p in polynomials]
                first = np.array([p(0) for p in normalized])
                second = np.array([p(1) for p in normalized])
                edges.append(
                    ClockEdge(
                        elapsed + start,
                        elapsed + start + duration,
                        first,
                        second,
                        "DRAW",
                        op.get("stroke_id"),
                        radius_m=op.get("material_radius_m", 0),
                        tcp0=arm.robot.forward(first, arm.flange_to_tcp)[:3, 3],
                        tcp1=arm.robot.forward(second, arm.flange_to_tcp)[:3, 3],
                        coefficients=[p.coef.tolist() for p in normalized],
                    )
                )
            elapsed += result["duration_s"]
            continue
        if op["kind"] in ("TRAVEL", "RETURN", "PARK"):
            q = coalesce_travel(q)
        if "hold_s" in op:
            pairs = [(q[0], q[0])]
        else:
            pairs = zip(q[:-1], q[1:], strict=True)
        for first, second in pairs:
            dq = abs(second - first)
            if "hold_s" in op:
                duration = op["hold_s"]
            elif np.max(dq) < 1e-12:
                continue
            else:
                # h'(u) <= 15/8 and |h''(u)| <= 10/sqrt(3).
                cartesian = tcp_tangent_bound(arm, first, second)
                speed = drawing_speed if op["kind"] == "DRAW" else travel_speed
                duration = (
                    max(
                        0.001,
                        float(np.max(1.875 * dq / velocity)),
                        float(np.sqrt(np.max((10 / np.sqrt(3)) * dq / acceleration))),
                        float(1.875 * cartesian / speed),
                    )
                    * 1.00001
                )
            if duration <= 0:
                continue
            edges.append(
                ClockEdge(
                    elapsed,
                    elapsed + duration,
                    first,
                    second,
                    op["kind"],
                    op.get("stroke_id"),
                    None if op.get("contact_m") is None else np.asarray(op["contact_m"]),
                    op.get("material_radius_m", 0.0),
                    arm.robot.forward(first, arm.flange_to_tcp)[:3, 3],
                    arm.robot.forward(second, arm.flange_to_tcp)[:3, 3],
                )
            )
            elapsed += duration
    if drawing_reuse is not None and reused != len(drawing_reuse):
        raise ValueError("Retained drawing clock has unused parts")
    result = ClockPath(edges)
    if diagnostics:
        result.metadata["drawing_timing"] = diagnostics
    return result
