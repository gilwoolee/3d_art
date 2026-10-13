"""Re-audit timed cubic geometry against the arm, work-cell and growing strands.

The clock does not change the geometric sweep. This audit evaluates the saved
cubic itself and bounds between samples; it never inherits a joint-linear
collision certificate. Pen-tip contact and hot-material behavior are separate.
"""

import argparse
import hashlib
import json
from pathlib import Path
from time import monotonic

import numpy as np
from numpy.polynomial import Polynomial

from ..io import artifact, read_json, sha256, stage_update, write_json
from ..kinematics import pose_error, rigid_transform
from ..robot import SerialArm
from ..timing import (
    _stationary_roots,
    make_path,
    maximum_absolute,
    polynomial_range,
    tcp_acceleration_bound,
)
from .collision import RobotCollisionModel, work_surfaces
from .scene import MaterialScene
from .tool import load_tool, require_matching_tool


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


def _audit_smooth_paths(run, maximum_seconds=300):
    run = Path(run)
    if not np.isfinite(maximum_seconds) or maximum_seconds <= 0:
        raise ValueError("Audit time budget must be positive and finite")
    raw = (run / "timed_schedule.json").read_bytes()
    schedule = json.loads(raw)
    if schedule["status"] != "TIMED_SIMULATION_PREVIEW":
        raise ValueError("A complete timed preview is required")
    sequence = read_json(run / "sequence_motion.json")
    for source in (schedule, sequence):
        for name, digest in source["inputs"].items():
            if sha256(run / name) != digest:
                raise ValueError(f"Stale smooth-audit input: {name}")
    robot = SerialArm(run / "robot.urdf")
    offset = rigid_transform(schedule["flange_to_tcp"])
    if not np.allclose(offset, sequence["flange_to_tcp"], rtol=0, atol=1e-12):
        raise ValueError("Timed and source TCP transforms differ")
    config = sequence["collision_settings"]
    tool = load_tool(config.get("tool_geometry"), Path("configs"), robot, offset)
    require_matching_tool(tool, sequence)
    model = RobotCollisionModel(robot, tool.bodies)
    placement = read_json(run / "placement_search.json")
    selected = placement["selected"]
    asset = read_json(run / "asset_manifest.json")
    width, depth = np.subtract(asset["bounds_m"][1], asset["bounds_m"][0])[:2]
    obstacles = work_surfaces(
        selected["center_xy_m"],
        selected["height_above_table_m"],
        width + config["platform_padding_m"],
        depth + config["platform_padding_m"],
        placement["assumptions"]["table_z_m"],
        config["table_size_m"],
    )
    drawing = read_json(run / "drawing_motion.json")
    source_ops = {op["id"]: op for op in sequence["operations"]}
    drawing_paths = {op["id"]: op for op in drawing["strokes"]}
    movements = [op for op in schedule["operations"] if "geometric_path" in op]
    if [op["id"] for op in movements] != list(source_ops):
        raise ValueError("Timed operations differ from the source sequence")
    output = artifact(
        "smooth_path_audit",
        status="RUNNING",
        simulation_only=True,
        executable=False,
        source_schedule_sha256=hashlib.sha256(raw).hexdigest(),
        inputs={
            **sequence["inputs"],
            **schedule["inputs"],
            "timed_schedule.json": hashlib.sha256(raw).hexdigest(),
        },
        interpolation="natural_cubic_joint_spline",
        tool_model=tool.metadata,
        config=config,
        requested_operations=len(movements),
        operations=[],
        model_scope=[
            "nonadjacent_URDF_links",
            "table_box",
            "raised_platform_box",
            "growing_material_capsules",
        ]
        + (["configured_noncontact_pen_and_mount"] if tool.bodies else []),
        numerical_reserve_m=1e-6,
        limitations=[
            "Pen-tip contact and hot-material behavior are not validated",
            "Pen and mount bodies are checked only when a measured profile is supplied",
            "Checks follow the commanded spline, not a measured controller trajectory",
            "Strand radius and work-cell dimensions remain provisional",
        ],
    )
    stage_update(run, "validation", "running", message="Auditing timed cubic geometry")
    deadline, material = monotonic() + maximum_seconds, MaterialScene()
    for op in movements:
        if monotonic() >= deadline:
            break
        source = source_ops[op["id"]]
        rebuilt = make_path(source["positions"], source["tcp_path_m"])
        saved = op["geometric_path"]
        if not np.allclose(rebuilt.cspl.x, saved["knots"], atol=1e-12, rtol=0) or not np.allclose(
            rebuilt.cspl.c, saved["coefficients"], atol=1e-10, rtol=1e-10
        ):
            raise ValueError(f"Saved spline differs from its source: {op['id']}")
        q = np.asarray(source["positions"])
        keep = np.r_[True, np.linalg.norm(np.diff(q, axis=0), axis=1) > 1e-10]
        reference = np.asarray(source["tcp_path_m"])[keep]
        rotation = None
        if op["kind"] == "DRAW":
            target = drawing_paths[op["stroke_id"]]
            if not np.allclose(q, target["positions"], rtol=0, atol=1e-10):
                raise ValueError("Drawing reference does not match sequence waypoints")
            reference = np.asarray(target["tcp_targets_m"])[keep]
            rotation = target["tcp_rotation"]
        row = {
            "id": op["id"],
            "stroke_id": op["stroke_id"],
            "kind": op["kind"],
            "status": "CERTIFIED_MODELED_PATH",
            "pieces_checked": 0,
            "evaluations": 0,
            "tracking_status": "BOUNDED",
            "position_error_bound_m": 0.0,
            "orientation_error_bound_rad": 0.0,
            "witnesses": [],
        }
        for index, width in enumerate(np.diff(saved["knots"])):
            if monotonic() >= deadline:
                row["status"] = "TIME_BUDGET"
                break
            piece = CubicPiece(np.asarray(saved["coefficients"])[:, index], width)
            # Validate the full cubic's joint range, not just displayed samples.
            for j, poly in enumerate(piece.polynomials):
                lo, hi = polynomial_range(poly, 0, 1)
                if lo < robot.lower[j] - 1e-10 or hi > robot.upper[j] + 1e-10:
                    raise ValueError("Saved cubic exceeds robot joint limits")
            if op["kind"] in ("DRAW", "APPROACH", "WITHDRAW"):
                settings = drawing["settings"] if op["kind"] == "DRAW" else sequence["settings"]
                position_limit = settings.get(
                    "interpolation_tolerance_m", settings.get("lift_position_tolerance_m")
                )
                angle_limit = settings.get(
                    "interpolation_tolerance_rad", settings.get("lift_orientation_tolerance_rad")
                )
                target_rotation = (
                    rotation if rotation is not None else robot.forward(q[0], offset)[:3, :3]
                )
                tracked = check_tracking(
                    piece,
                    robot,
                    offset,
                    reference[index],
                    reference[index + 1],
                    target_rotation,
                    position_limit,
                    angle_limit,
                )
                if tracked["status"] != "BOUNDED":
                    row["tracking_status"] = tracked["status"]
                    old = row.get("tracking_witness", {}).get("position_error_m", -1)
                    if tracked["witness"]["position_error_m"] > old:
                        row["tracking_witness"] = {"piece": index, **tracked["witness"]}
                for key in ("position_error_bound_m", "orientation_error_bound_rad"):
                    row[key] = max(row[key], tracked[key])
            for a, b, inflation in bounded_segments(piece, robot, offset, 0.00005):
                local = piece.subpiece(a, b)
                if op["kind"] == "DRAW":
                    material.add(
                        robot.forward(local.position(0), offset)[:3, 3],
                        robot.forward(local.position(1), offset)[:3, 3],
                        source["material_radius_m"] + inflation,
                        f"material/{op['id']}/{index}/{a:.12g}",
                    )
                result = model.check_bounded_path(
                    local.position,
                    local.deviation,
                    obstacles,
                    config["clearance_m"],
                    config["maximum_subdivision_depth"],
                    material,
                    deadline,
                )
                row["evaluations"] += result["evaluations"]
                if result["status"] != "CERTIFIED_MODELED_EDGE":
                    row["status"] = result["status"]
                    if len(row["witnesses"]) < 3:
                        row["witnesses"].append({"piece": index, "subinterval": [a, b], **result})
            row["pieces_checked"] += 1
        if row["pieces_checked"] != len(saved["knots"]) - 1:
            row["tracking_status"] = "PARTIAL_CHECK"
        output["operations"].append(row)
        output["material_segments"] = len(material.segments)
        write_json(run / "smooth_collisions.json", output)
        print(
            f"{len(output['operations'])}/{len(movements)} {op['id']}: "
            f"{row['status']} / {row['tracking_status']}",
            flush=True,
        )
    rows = output["operations"]
    output["counts"] = {
        status: sum(row["status"] == status for row in rows)
        for status in ("CERTIFIED_MODELED_PATH", "COLLISION", "CLEARANCE_UNRESOLVED", "TIME_BUDGET")
    }
    output["tracking_counts"] = {
        status: sum(row["tracking_status"] == status for row in rows)
        for status in ("BOUNDED", "BOUND_EXCEEDS_TOLERANCE", "PARTIAL_CHECK")
    }
    output["status"] = (
        "COMPLETE_SMOOTH_ARM_AUDIT"
        if len(rows) == len(movements) and not output["counts"]["TIME_BUDGET"]
        else "PARTIAL_SMOOTH_ARM_AUDIT"
    )
    write_json(run / "smooth_collisions.json", output)
    stage_update(
        run,
        "validation",
        "partial",
        artifact="smooth_collisions.json",
        message="Cubic arm/material audit; pen-tip, process and physical validation remain",
    )
    return output


def audit_smooth_paths(run, maximum_seconds=300):
    try:
        return _audit_smooth_paths(run, maximum_seconds)
    except Exception as error:
        run = Path(run)
        path = run / "smooth_collisions.json"
        if path.exists():
            output = read_json(path)
            output.update(status="INCOMPLETE_SMOOTH_ARM_AUDIT", error=str(error))
            write_json(path, output)
        if (run / "run.json").exists():
            stage_update(run, "validation", "failed", message=str(error))
        raise


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, default=Path("runs/palace_15cm_selected"))
    parser.add_argument("--maximum-seconds", type=float, default=300)
    args = parser.parse_args()
    audit_smooth_paths(args.run, args.maximum_seconds)
