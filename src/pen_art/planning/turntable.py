"""Four indexed plate poses and conservative pen-off quarter-turn checks.

Angles are limited to [0, 270] degrees. Returning from index 3 to index 0
requires three reverse quarter-turns; there is no implicit cable wrap.
"""

from dataclasses import dataclass
from os.path import relpath
from pathlib import Path
from time import monotonic

import fcl
import numpy as np
import yaml

from ..kinematics import rigid_transform
from .collision import placed
from .coordinated_motion import arm_objects
from .fixture import riser_object
from .scene import MaterialScene


def audit_indexed_clock(table, path, parked, cooling_s):
    """Check index continuity and mutually exclusive arm/plate/pen operation."""
    current = path.edges[0].table_index
    index_angle(current)
    feeding, settle_pending = bool(path.metadata.get("chained_in")), False
    last_deposit = -float("inf")
    cycle_drew = False
    turns = 0
    for edge in path.edges:
        if edge.table_index != current:
            raise ValueError("Discontinuous table index in saved clock")
        duration = edge.end_s - edge.start_s
        if settle_pending and edge.kind != "TABLE_SETTLE":
            raise ValueError("Table must settle before any further action")
        if edge.kind == "INDEX_TURN":
            index_angle(edge.table_to_index)
            if (
                abs(current - edge.table_to_index) != 1
                or feeding
                or edge.start_s < last_deposit + cooling_s - 1e-9
                or edge.contact is not None
                or not np.allclose(edge.q0, parked, atol=1e-9, rtol=0)
                or not np.allclose(edge.q1, parked, atol=1e-9, rtol=0)
                or duration < table.move_duration() - 1e-9
            ):
                raise ValueError("Unsafe quarter-turn: pen, parking, cooling, or speed")
            current = edge.table_to_index
            settle_pending = True
            turns += 1
        elif edge.kind == "TABLE_SETTLE":
            if (
                not settle_pending
                or duration < table.settling_s - 1e-9
                or not np.allclose(edge.q0, parked, atol=1e-9, rtol=0)
                or not np.allclose(edge.q1, parked, atol=1e-9, rtol=0)
            ):
                raise ValueError("Invalid table settling interval")
            settle_pending = False
        elif edge.table_to_index is not None:
            raise ValueError("Only an INDEX_TURN may change table index")
        if edge.kind == "PEN_START_WAIT":
            feeding = True
            cycle_drew = False
        elif edge.kind == "PEN_STOP_WAIT":
            feeding = False
            if not cycle_drew:
                # A feed-only pulse can still deposit material. Keep the
                # conservative stop-time bound when there is no drawing edge.
                last_deposit = edge.end_s
        elif edge.kind == "DRAW":
            if not feeding:
                raise ValueError("Drawing with feed off")
            cycle_drew = True
            last_deposit = edge.end_s
    if settle_pending or feeding != bool(path.metadata.get("chained_out")):
        raise ValueError("Clock ends before the table settles or pen stops")
    return {"initial_index": path.edges[0].table_index, "final_index": current, "turns": turns}


def prepare_profile(source, output):
    """Add explicit provisional hardware to an existing host-local arm profile."""
    source, output = Path(source).resolve(), Path(output).resolve()
    profile = yaml.safe_load(source.read_text())
    for arm in profile.get("arms", []):
        for key in ("robot_urdf", "robot_manifest", "robot_visual"):
            arm[key] = relpath((source.parent / arm[key]).resolve(), output.parent)
        for body in (arm.get("tool_geometry") or {}).get("bodies", []):
            shape = body["shape"]
            if shape["type"] == "mesh":
                shape["path"] = relpath((source.parent / shape["path"]).resolve(), output.parent)
    profile["turntable"] = {
        "mode": "indexed_90",
        "calibrated": False,
        "world_from_axis": [list(row) for row in profile["world_from_build"]],
        "axis_from_build_zero": np.eye(4).tolist(),
        "home_index": 0,
        "velocity_rad_s": 0.25,
        "acceleration_rad_s2": 0.25,
        "settling_s": 1.0,
        "plate_radius_m": 0.12,
        "plate_thickness_m": 0.01,
        "housing_radius_m": 0.09,
        "housing_height_m": 0.18,
        "load_acceleration_limit_m_s2": 0.1,
        "assumptions": [
            "Provisional dimensions and actuator limits; no hardware calibration.",
            "Model rigidly attached to horizontal plate; completed strands cooled before rotation.",
            "Rotation screened by conservative acceleration bound; adhesion unverified.",
        ],
    }
    if "build_riser" in profile["environment"]:
        height = float(profile["environment"]["build_riser"]["height_m"])
        profile["turntable"]["world_from_axis"][2][3] -= height
        profile["turntable"]["axis_from_build_zero"][2][3] = height
    IndexedTurntable.from_profile(profile)
    if output.exists():
        raise ValueError("Use a fresh profile path; existing calibration must not be overwritten")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(profile, sort_keys=False))
    return profile


def index_angle(index):
    if isinstance(index, bool) or not isinstance(index, (int, np.integer)) or index not in range(4):
        raise ValueError("Turntable index must be an integer in [0, 3]")
    return int(index) * np.pi / 2


def rotation_z(angle):
    if not np.isfinite(angle):
        raise ValueError("Turntable angle must be finite")
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])


@dataclass(frozen=True)
class IndexedTurntable:
    world_from_axis: np.ndarray
    axis_from_build_zero: np.ndarray
    velocity_rad_s: float
    acceleration_rad_s2: float
    settling_s: float
    plate_radius_m: float
    plate_thickness_m: float
    housing_radius_m: float
    housing_height_m: float
    load_acceleration_limit_m_s2: float
    home_index: int = 0
    build_riser: dict | None = None

    @classmethod
    def from_profile(cls, profile):
        data = profile["turntable"]
        if data.get("mode") != "indexed_90" or data.get("calibrated") is not False:
            raise ValueError("Only explicitly uncalibrated indexed_90 simulation is implemented")
        axis = rigid_transform(data["world_from_axis"], "world-from-turntable-axis")
        mount = rigid_transform(data["axis_from_build_zero"], "turntable-mount")
        if not np.allclose(axis[:3, 2], [0, 0, 1], atol=1e-9, rtol=0):
            raise ValueError("Turntable rotation axis must be vertical")
        if not np.allclose(mount[:3, 2], [0, 0, 1], atol=1e-9, rtol=0):
            raise ValueError("Turntable build plate must remain horizontal")
        names = (
            "velocity_rad_s",
            "acceleration_rad_s2",
            "settling_s",
            "plate_radius_m",
            "plate_thickness_m",
            "housing_radius_m",
            "housing_height_m",
            "load_acceleration_limit_m_s2",
        )
        values = [data[name] for name in names]
        if any(isinstance(x, bool) or not np.isfinite(x) or x <= 0 for x in values):
            raise ValueError("Turntable dimensions and limits must be finite and positive")
        home = data.get("home_index", 0)
        index_angle(home)
        riser = profile.get("environment", {}).get("build_riser")
        result = cls(axis, mount, *map(float, values), home_index=int(home), build_riser=riser)
        if not np.allclose(
            result.build_transform(home), profile["world_from_build"], atol=1e-9, rtol=0
        ):
            raise ValueError("Turntable home does not match the cell's build transform")
        if mount[2, 3] < 0:
            raise ValueError("Build surface cannot be below the turntable plate")
        if riser is not None:
            _, radius = riser_object(riser, result.build_transform(home))
            if not np.isclose(mount[2, 3], riser["height_m"]):
                raise ValueError("Riser must bridge turntable plate and build surface")
            if radius > result.plate_radius_m:
                raise ValueError("Riser footprint must fit on the turntable plate")
        return result

    def build_transform(self, index):
        return self.transform_at_angle(index_angle(index))

    def transform_at_angle(self, angle):
        return self.world_from_axis @ rotation_z(angle) @ self.axis_from_build_zero

    def steps(self, first, last):
        index_angle(first)
        index_angle(last)
        direction = 1 if first < last else -1
        return list(range(first + direction, last + direction, direction)) if first != last else []

    def move_duration(self):
        # Max first/second derivatives of 10u^3 - 15u^4 + 6u^5.
        delta = np.pi / 2
        return max(
            1.875 * delta / self.velocity_rad_s,
            np.sqrt((10 / np.sqrt(3)) * delta / self.acceleration_rad_s2),
        )

    def obstacles(self):
        result = {}
        for name, radius, height, z in (
            (
                "platform",
                self.plate_radius_m,
                self.plate_thickness_m,
                -self.plate_thickness_m / 2,
            ),
            (
                "turntable_housing",
                self.housing_radius_m,
                self.housing_height_m,
                -self.plate_thickness_m - self.housing_height_m / 2,
            ),
        ):
            local = np.eye(4)
            local[2, 3] = z
            result[name] = placed(fcl.Cylinder(radius, height), self.world_from_axis @ local)
        if self.build_riser is not None:
            result["build_riser"] = riser_object(
                self.build_riser, self.build_transform(self.home_index)
            )[0]
        return result

    def reposition(self, material, from_angle, to_angle):
        """Rigidly rotate the entire accumulated material, retaining its identifiers."""
        delta = self.transform_at_angle(to_angle) @ np.linalg.inv(
            self.transform_at_angle(from_angle)
        )
        result = MaterialScene()
        index = to_angle / (np.pi / 2)
        result.table_index = int(round(index)) if abs(index - round(index)) < 1e-9 else None
        for row in material.segments:
            points = np.asarray([row["start"], row["end"]]) @ delta[:3, :3].T + delta[:3, 3]
            result.add(*points, row["radius_m"], row["name"])
        return result

    def check_quarter_turn(
        self,
        material,
        first,
        last,
        stationary,
        *,
        margin_m=0.001,
        maximum_depth=20,
        deadline=float("inf"),
    ):
        """Bound the whole material sweep against stationary collision objects.

        Material is expressed at index ``first``. The plate is an axisymmetric
        cylinder, checked separately as an ordinary fixed robot obstacle. Its
        intended contact with anchored material is excluded from this sweep.
        ``stationary`` must include all parked robot/pen bodies and other
        obstacles. Queries move those bodies into the material's original frame,
        preserving the material's acceleration structure.
        """
        a0, a1 = index_angle(first), index_angle(last)
        if abs(first - last) != 1:
            raise ValueError("A turn action must be exactly one adjacent quarter-turn")
        if not np.isfinite(margin_m) or margin_m < 0 or maximum_depth < 0:
            raise ValueError("Invalid turntable clearance limits")
        center = self.world_from_axis[:3, 3]
        radius = max(
            (
                np.linalg.norm(np.asarray(row[end])[:2] - center[:2]) + row["radius_m"]
                for row in material.segments
                for end in ("start", "end")
            ),
            default=0.0,
        )
        riser = None
        if self.build_riser is not None:
            riser, riser_radius = riser_object(self.build_riser, self.build_transform(first))
            radius = max(radius, riser_radius)
        load_bound = radius * (self.acceleration_rad_s2 + self.velocity_rad_s**2)
        if load_bound > self.load_acceleration_limit_m_s2:
            return {"status": "ROTATION_LOAD_BOUND_EXCEEDED", "bound_m_s2": load_bound}
        objects = []
        for name, obj in stationary.items():
            if name in {"turntable_plate", "platform", "build_riser"}:
                continue  # Intended welded/anchored plate contact.
            transform = np.eye(4)
            transform[:3, :3] = obj.getRotation()
            transform[:3, 3] = obj.getTranslation()
            objects.append((name, obj, transform))
        evaluations = 0
        deepest = 0

        def inspect(fraction, bound):
            nonlocal evaluations
            evaluations += 1
            angle = a0 + fraction * (a1 - a0)
            inverse = self.transform_at_angle(a0) @ np.linalg.inv(self.transform_at_angle(angle))
            if riser is not None:
                moving, _ = riser_object(self.build_riser, self.transform_at_angle(angle))
                for name, obj, _ in objects:
                    gap = float(fcl.distance(moving, obj))
                    pair = {"first": "build_riser", "second": name, "distance_m": gap}
                    if gap <= margin_m:
                        return {"status": "COLLISION", "fraction": fraction, "pairs": [pair]}
                    if gap <= margin_m + bound + 1e-6:
                        return {
                            "status": "CLEARANCE_UNRESOLVED",
                            "fraction": fraction,
                            "pairs": [pair],
                        }
            for name, obj, transform in objects:
                # python-fcl does not expose an object's underlying geometry.
                # Move the query object only for this synchronous distance call,
                # restoring it even if FCL raises. The material tree stays fixed.
                query = inverse @ transform
                try:
                    obj.setTransform(fcl.Transform(query[:3, :3], query[:3, 3]))
                    nearest = material.nearest(obj, deadline=deadline)
                finally:
                    obj.setTransform(fcl.Transform(transform[:3, :3], transform[:3, 3]))
                if nearest is None:
                    continue
                gap = nearest["distance_m"]
                if not np.isfinite(gap):
                    raise ValueError("Non-finite turntable collision distance")
                pair = {"first": nearest["name"], "second": name, "distance_m": gap}
                if gap <= margin_m:
                    return {"status": "COLLISION", "fraction": fraction, "pairs": [pair]}
                if gap <= margin_m + bound + 1e-6:
                    return {"status": "CLEARANCE_UNRESOLVED", "fraction": fraction, "pairs": [pair]}
            return {"status": "CERTIFIED_TURNTABLE_INTERVAL"}

        def visit(left, right, depth):
            nonlocal deepest
            if monotonic() >= deadline:
                return {"status": "TIME_BUDGET"}
            deepest = max(deepest, depth)
            middle = (left + right) / 2
            bound = radius * abs(a1 - a0) * (right - left) / 2
            result = inspect(middle, bound)
            if result["status"] != "CLEARANCE_UNRESOLVED" or depth >= maximum_depth:
                return result
            result = visit(left, middle, depth + 1)
            return (
                visit(middle, right, depth + 1)
                if result["status"] == "CERTIFIED_TURNTABLE_INTERVAL"
                else result
            )

        try:
            for fraction in (0, 1):
                result = inspect(fraction, 0.0)
                if result["status"] != "CERTIFIED_TURNTABLE_INTERVAL":
                    break
            else:
                result = visit(0, 1, 0)
        except TimeoutError:
            result = {"status": "TIME_BUDGET"}
        return {
            **result,
            "evaluations": evaluations,
            "maximum_depth": deepest,
            "load_acceleration_bound_m_s2": load_bound,
        }

    def check_parked_turn(self, material, first, last, arm, q, obstacles, **settings):
        """Check a stationary arm, plate/housing, and rotating material together."""
        contacts = arm.collision.check(q, obstacles, settings.get("margin_m", 0.001))
        if contacts:
            return {"status": "PARKED_ARM_COLLISION", "pairs": contacts}
        stationary = dict(obstacles)
        stationary.update(
            {f"robot/{body.name}": obj for body, obj in arm_objects(arm.collision, q)}
        )
        return self.check_quarter_turn(material, first, last, stationary, **settings)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Prepare a provisional indexed-turntable profile")
    parser.add_argument("--source", type=Path, default=Path("research/planning_cell/cell.yaml"))
    parser.add_argument(
        "--output", type=Path, default=Path("research/planning_cell/cell_turntable.yaml")
    )
    args = parser.parse_args()
    prepare_profile(args.source, args.output)
