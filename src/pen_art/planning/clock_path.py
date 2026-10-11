"""Exact serialized clocks for linear travel and continuous spline drawing.

The default quintic clock stops at edge boundaries. The opt-in drawing clock
uses midpoint-refined, shape-preserving cubic interpolation and TOPPRA. Its
changed geometry requires fresh tracking and collision checks. Neither clock
establishes the behavior of an uncalibrated physical pen.
"""

from dataclasses import dataclass
from time import monotonic

import numpy as np
from numpy.polynomial import Polynomial

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
        tangent = np.array([p.deriv()(middle) for p in piece.polynomials])
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

    def piece(self, a=None, b=None):
        if self.coefficients is None:
            polynomials = [
                Polynomial([first, 0, 0, 10 * d, -15 * d, 6 * d])
                for first, d in zip(self.q0, self.q1 - self.q0, strict=True)
            ]
        else:
            polynomials = [Polynomial(c) for c in self.coefficients]
        piece = PolynomialPiece(polynomials)
        if a is None and b is None:
            return piece
        width = self.end_s - self.start_s
        return piece.subpiece((a - self.start_s) / width, (b - self.start_s) / width)

    def position(self, time):
        u = float(np.clip((time - self.start_s) / (self.end_s - self.start_s), 0, 1))
        if self.coefficients is not None:
            return self.piece().position(u)
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
            )
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
                before = np.array([p.deriv()(1) for p in prior.piece().polynomials]) / (
                    prior.end_s - prior.start_s
                )
                after = np.array([p.deriv()(0) for p in edge.piece().polynomials]) / (
                    edge.end_s - edge.start_s
                )
                if not np.allclose(before, after, atol=1e-6, rtol=1e-6):
                    raise ValueError("Discontinuous saved joint velocity")
            edges.append(edge)
        path = cls(edges)
        for edge, value in ((edges[0], 0), (edges[-1], 1)):
            if edge.coefficients is not None and any(
                abs(p.deriv()(value) / (edge.end_s - edge.start_s)) > 1e-6
                for p in edge.piece().polynomials
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
):
    """Analytically time checked edges without changing their geometric sweep."""
    edges, elapsed, diagnostics = [], 0.0, []
    if drawing_clock not in {"quintic_stop_at_geometric_knots", "cubic_toppra"}:
        raise ValueError(f"Unsupported drawing clock: {drawing_clock}")
    if acceleration_constraint not in {"interpolated", "cubic_bernstein"}:
        raise ValueError(f"Unsupported acceleration constraint: {acceleration_constraint}")
    for op in operations:
        q = np.asarray(op["positions"], float)
        if op["kind"] == "DRAW" and drawing_clock == "cubic_toppra":
            points = [arm.robot.forward(joints, arm.flange_to_tcp)[:3, 3] for joints in q]
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
            )
            pieces = clock_polynomials(
                path, np.array(result["path_clock"]["s"]), np.array(result["path_clock"]["speeds"])
            )
            diagnostics.append(
                {
                    "stroke_id": op.get("stroke_id"),
                    "time_scale_correction": result["time_scale_correction"],
                    "retiming_method": result["retiming_method"],
                    "retiming_fallback_reason": result["retiming_fallback_reason"],
                    "bounds": result["bounds"],
                    "acceleration_constraint": acceleration_constraint,
                }
            )
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
    result = ClockPath(edges)
    if diagnostics:
        result.metadata["drawing_timing"] = diagnostics
    return result
