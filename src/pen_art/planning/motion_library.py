"""Persistent drawing proposals and conservative robot/static clearance reuse.

Records never authorize execution: every retrieved/adapted path passes tracking
and the build's material/process checks. Clearance samples are lower bounds on
pair separation at *any* nearby configuration, consumed by the existing
continuous swept-motion checker, not a sampled collision acceptance test.
"""

import hashlib
import json
import os
import xml.etree.ElementTree as ET
from collections import OrderedDict
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path
from time import monotonic, perf_counter, time_ns

import numpy as np
from numpy.polynomial import Polynomial

from ..kinematics import inverse_candidates, pose_error
from .collision import RobotCollisionModel
from .smooth_audit import PolynomialPiece, check_tracking

ADMISSION_IMPLEMENTATION = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
SCHEMA_VERSION = 2
FIXED_ORIENTATION_POLICY = "fixed-tcp-orientation-v1"
ORIENTATION_LIMIT_RAD = 0.003
# Checks an adaptation candidate passes in order; a failure reports the furthest.
ADAPTATION_STAGES = ("no_inverse_kinematics", "off_target", "tracking", "collision")
# Closest a retained cached knot may lie to a request sample or to another one.
SEED_SAMPLE_GAP_M = 0.0005
# Loaded records each lookup keeps per process (about 14 MB apiece).
SELECTED_RECORDS = 8
# Total backing-up steps per candidate, and how far behind the furthest point.
ADAPTATION_BACKTRACKS, ADAPTATION_BACKTRACK_DEPTH = 24, 8
VALIDATOR_VERSION = "motion-clearance-lipschitz-v1"
DRAWING_POLICY_VERSION = "whole-connected-action-v1"
SELF_STATIC_SCOPE = "SELF_STATIC_ONLY"
GROWING_PREFIX_SCOPE = "SELF_STATIC_AND_GROWING_PREFIX"
OFFLINE_GEOMETRY_SCOPES = {SELF_STATIC_SCOPE, GROWING_PREFIX_SCOPE}
GROWING_PREFIX_POLICY = "conservative-current-piece-and-prior-prefix-v1"


def _plain(value):
    if hasattr(value, "tolist"):
        return value.tolist()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(f"Cannot serialize {type(value).__name__}")


def _bytes(value):
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False, default=_plain
    ).encode()


def digest(value):
    return hashlib.sha256(_bytes(value)).hexdigest()


def _save(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.{time_ns()}.tmp")
    temporary.write_bytes(_bytes(value))
    temporary.replace(path)


@dataclass
class DrawingRequest:
    source_points_m: object
    table_index: int = 0
    q_start: object = None
    q_goal: object = None
    allowed_orientations: list = field(default_factory=list)
    source_object: str = "unknown"
    family_id: str = "unknown"
    context_hash: str = ""
    path_kind: str = "unknown"
    fixed_tcp_rotation: object = None

    def __post_init__(self):
        points = np.asarray(self.source_points_m, float)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2:
            raise ValueError("A drawing request needs at least two Cartesian points")
        if not np.isfinite(points).all() or not np.linalg.norm(np.diff(points, axis=0)):
            raise ValueError("Drawing source must be finite with nonzero length")
        if self.table_index not in (0, 1, 2, 3):
            raise ValueError("Table index must be 0, 1, 2, or 3")
        self.source_points_m = points.tolist()
        for name in ("q_start", "q_goal"):
            value = getattr(self, name)
            if value is not None:
                q = np.asarray(value, float)
                if q.ndim != 1 or not np.isfinite(q).all():
                    raise ValueError("Joint constraints must be finite vectors")
                setattr(self, name, q.tolist())
        rotations = self.allowed_orientations + (
            [] if self.fixed_tcp_rotation is None else [self.fixed_tcp_rotation]
        )
        for rotation in rotations:
            r = np.asarray(rotation, float)
            if (
                r.shape != (3, 3)
                or not np.isfinite(r).all()
                or not np.allclose(r.T @ r, np.eye(3), atol=1e-7)
                or np.linalg.det(r) < 0.999999
            ):
                raise ValueError("Allowed orientations must be rotation matrices")
        if self.fixed_tcp_rotation is not None:
            self.fixed_tcp_rotation = np.asarray(self.fixed_tcp_rotation).tolist()

    def as_dict(self):
        return asdict(self)


def _arc(points):
    return np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]


def _resample(points, count=16):
    points = np.asarray(points, float)
    arc = _arc(points)
    return np.column_stack([np.interp(np.linspace(0, arc[-1], count), arc, p) for p in points.T])


def _preserve_seed_samples(points, old_points):
    """Retain fine cached knots by mapping them between corresponding source legs.

    The regular request grid still participates. Added points affect resolution,
    never acceptance: projection, tracking, and collision checks remain required.
    """
    source_vertices, old_vertices = _vertices(points), _vertices(old_points)
    source_arc, old_arc = _arc(points), _arc(old_points)
    if old_arc[-1] <= 0:
        return points
    if source_vertices.shape == old_vertices.shape:
        mapped = np.interp(old_arc, _arc(old_vertices), _arc(source_vertices))
    else:
        # Fully timed records retain actual FK points, which can have tiny
        # curvature rather than the request's exact straight source legs.
        mapped = old_arc / old_arc[-1] * source_arc[-1]
    # A cached knot within the gap of a request sample adds no resolution, and
    # beside a source vertex it makes the corner's own knot ambiguous: a timed
    # record brakes into each corner, leaving knots microns short of it. Every
    # request sample, and so every corner, is preserved.
    all_arc, last = list(source_arc), None
    for value in np.unique(mapped):
        right = int(np.searchsorted(source_arc, value))
        gap = min(
            abs(value - source_arc[i]) for i in (right - 1, right) if 0 <= i < len(source_arc)
        )
        if gap > SEED_SAMPLE_GAP_M and (last is None or value - last > SEED_SAMPLE_GAP_M):
            all_arc.append(value)
            last = value
    all_arc = np.unique(all_arc)
    return np.column_stack([np.interp(all_arc, source_arc, axis) for axis in points.T])


def _vertices(points):
    """Remove only redundant straight-line samples, never a source corner."""
    points = np.asarray(points, float)
    keep = np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-12]
    points = points[keep]
    delta = np.diff(points, axis=0)
    directions = delta / np.linalg.norm(delta, axis=1)[:, None]
    turns = np.linalg.norm(np.diff(directions, axis=0), axis=1) > 1e-7
    return points[np.r_[True, turns, True]]


def _geometry_distance(first, second):
    """Compare all source vertices instead of aliasing a zigzag onto 16 samples."""
    first, second = _vertices(first), _vertices(second)
    first_arc, second_arc = _arc(first), _arc(second)
    first_progress, second_progress = first_arc / first_arc[-1], second_arc / second_arc[-1]
    grid = np.unique(np.r_[first_progress, second_progress])
    a = np.column_stack([np.interp(grid, first_progress, axis) for axis in first.T])
    b = np.column_stack([np.interp(grid, second_progress, axis) for axis in second.T])
    distances = np.linalg.norm(a - b, axis=1)
    # Both integrated error and the worst corner matter: a short extra leg
    # must not disappear merely because most of the path is unchanged.
    integral = np.sum(np.diff(grid) * (distances[:-1] + distances[1:]) / 2)
    return float(integral + distances.max())


def _compact_timed_seed(points, joints, source, *, step_m=0.003, joint_step_rad=0.05):
    """Keep a small IK seed, never simplify or certify an accepted timed clock.

    Source turns and substantial joint changes participate even when Cartesian
    distance is small. Every resulting adapted interpolant earns fresh tracking
    and collision checks. The untouched dense seed remains a bounded fallback.
    """
    points, joints = np.asarray(points), np.asarray(joints)
    arc = _arc(points)
    if arc[-1] <= 1e-12:
        return points, joints
    vertices = _vertices(source)
    required = _arc(vertices)
    required = required / required[-1] * arc[-1]
    indices = {0, len(points) - 1}
    for distance in required:
        right = min(int(np.searchsorted(arc, distance)), len(arc) - 1)
        left = max(0, right - 1)
        indices.add(left if abs(arc[left] - distance) < abs(arc[right] - distance) else right)
    last = 0
    for index in range(1, len(points)):
        if (
            arc[index] - arc[last] >= step_m
            or np.max(np.abs(joints[index] - joints[last])) >= joint_step_rad
            or index in indices
        ):
            indices.add(index)
            last = index
    keep = sorted(indices)
    return points[keep], joints[keep]


def _geometry_kind(value):
    if value.get("path_kind", "unknown") != "unknown":
        return value["path_kind"]
    points = np.asarray(value["source_points_m"])
    # Legacy records have no family metadata. A closed horizontal path is
    # a contour, and is a poor seed for a rising/falling WirePrint zigzag.
    if np.linalg.norm(points[-1] - points[0]) < 1e-6 and np.ptp(points[:, 2]) < 1e-7:
        return "contour"
    return "unknown"


def compatible_geometry(request, record):
    """Screen incompatible cache families, never the complete geometric solver."""
    requested, stored = _geometry_kind(request.as_dict()), _geometry_kind(record["request"])
    return requested == "unknown" or stored == "unknown" or requested == stored


def collision_context_hash(model, geometry_context=None):
    """Bind source meshes, kinematics and supplied extra geometry/calibration.

    ``geometry_context`` must describe any opaque FCL convex extras (tool
    metadata already includes mesh hashes). Static obstacle identities/poses are
    additionally bound per query; their replacement invalidates cached bounds.
    """
    robot = model.robot
    meshes = {}
    for mesh in robot.xml.findall(".//mesh"):
        path = Path(mesh.get("filename"))
        meshes[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    return digest(
        {
            "validator": VALIDATOR_VERSION,
            "xml": ET.tostring(robot.xml).decode(),
            "meshes": meshes,
            "world_from_base": getattr(robot, "world_from_base", np.eye(4)),
            "lower": robot.lower,
            "upper": robot.upper,
            "extra_geometry": geometry_context,
            "bodies": [
                {
                    "name": b.name,
                    "link": b.link,
                    "origin": b.origin,
                    "radius": b.radius,
                    "touch_links": b.touch_links,
                    "shape": _shape(b.geometry),
                }
                for b in model.bodies
            ],
            "pairs": [(a, b, r) for (a, b), r in model._pair_radii.items()],
        }
    )


def _shape(geometry):
    return {
        "type": type(geometry).__name__,
        **{
            name: getattr(geometry, name)
            for name in ("side", "radius", "lz")
            if hasattr(geometry, name)
        },
    }


def obstacle_context(obstacles, static_geometry=None, references=None):
    """Bind actual poses; unknown opaque shapes are reusable only in-process."""
    objects, persistent = {}, True
    for name, obj in (obstacles or {}).items():
        shape = (static_geometry or {}).get(name)
        if shape is not None and references is not None:
            original = references.setdefault(("declared", name), obj)
            if original is not obj:
                # FCL does not expose replacement geometry dimensions. The
                # caller's old shape signature cannot certify a new object.
                shape = None
        if shape is None:
            persistent = False
            if references is not None:
                references[id(obj)] = obj
            shape = {"process_identity": id(obj)}
        objects[name] = {
            "shape": shape,
            "node_type": obj.getNodeType(),
            "rotation": obj.getRotation(),
            "translation": obj.getTranslation(),
        }
    return ("static:" if persistent else "process:") + digest(objects)


class MaterialOnlyCollisionModel:
    """Recheck fresh material after an exact-path self/static proof is found.

    This wrapper grants no proof itself. Its caller must hold a certificate for
    the complete unchanged interpolant and physical/static context.
    """

    check_bounded_path = RobotCollisionModel.check_bounded_path
    check_edge = RobotCollisionModel.check_edge
    check = RobotCollisionModel.check

    def __init__(self, model):
        self.model = model

    def __getattr__(self, name):
        return getattr(self.model, name)

    def distances(self, positions, obstacles=None, *, deadline=float("inf")):
        if monotonic() >= deadline:
            raise TimeoutError("Material verification exceeded its budget")
        self.robot.frames(positions)
        yield from ()


class CachedCollisionModel:
    """Duck-typed RobotCollisionModel using persistent clearance samples.

    Material queries are always current. Each returned static/self distance is
    an exact measured distance minus a conservative pair-motion bound. Unknown
    neighborhoods fall back to FCL. Objects must be replaced when their geometry
    changes; FCL pose changes are detected directly. Opaque extra geometry must
    be included in geometry_context, which is immutable for this wrapper.
    """

    check_edge = RobotCollisionModel.check_edge

    def check(
        self, positions, obstacles=None, margin_m=0.0, material=None, *, deadline=float("inf")
    ):
        previous = getattr(self, "_reuse_margin", 0.0)
        self._reuse_margin = margin_m
        try:
            return RobotCollisionModel.check(
                self, positions, obstacles, margin_m, material, deadline=deadline
            )
        finally:
            self._reuse_margin = previous

    def check_bounded_path(
        self,
        position,
        deviation,
        obstacles=None,
        margin_m=0.001,
        maximum_depth=12,
        material=None,
        deadline=float("inf"),
    ):
        previous = getattr(self, "_reuse_margin", 0.0)
        self._reuse_margin = margin_m
        try:
            return RobotCollisionModel.check_bounded_path(
                self, position, deviation, obstacles, margin_m, maximum_depth, material, deadline
            )
        finally:
            self._reuse_margin = previous

    def __init__(
        self, model, directory, geometry_context=None, maximum_samples=512, static_geometry=None
    ):
        if maximum_samples < 1:
            raise ValueError("Clearance cache needs a positive sample limit")
        urdf_names = {
            f"{link.get('name')}:{index}"
            for link in model.robot.xml.findall("link")
            for index, _ in enumerate(link.findall("collision"))
        }
        if geometry_context is None and any(
            body.name not in urdf_names and type(body.geometry).__name__ == "Convex"
            for body in model.bodies
        ):
            raise ValueError("Opaque extra geometry needs an explicit context fingerprint")
        self.model = model
        self.context_hash = collision_context_hash(model, geometry_context)
        self.directory = Path(directory) / "clearance" / self.context_hash
        self.directory.mkdir(parents=True, exist_ok=True)
        self.maximum_samples = maximum_samples
        self.static_geometry = static_geometry or {}
        self._obstacle_refs = {}
        self.samples = {}
        self._atlas_arrays = {}
        self.hits = self.misses = 0
        self.self_static_seconds = self.material_seconds = 0.0
        self._pending = 0
        self._last_flush = monotonic()
        self._load()

    def __getattr__(self, name):
        return getattr(self.model, name)

    def _load(self):
        path = self.directory / "samples.json"
        if not path.exists():
            return
        try:
            envelope = json.loads(path.read_text())
            rows = envelope["rows"]
            if (
                envelope.get("schema_version") != SCHEMA_VERSION
                or envelope.get("validator_version") != VALIDATOR_VERSION
                or envelope.get("context_hash") != self.context_hash
                or not isinstance(rows, list)
                or envelope["digest"] != digest(rows)
            ):
                return
        except (OSError, ValueError, KeyError, TypeError, AttributeError):
            return
        expected = set(self.model._pair_radii)
        body_names = {body.name for body in self.bodies}
        for row in rows:
            try:
                q, ds = np.asarray(row["q"], float), row["distances"]
                key = row["key"]
                if (
                    q.shape != (len(self.robot.names),)
                    or not np.isfinite(q).all()
                    or np.any(q < self.robot.lower)
                    or np.any(q > self.robot.upper)
                    or not isinstance(ds, list)
                    or not isinstance(key, str)
                ):
                    continue
                if not all(
                    isinstance(d, list)
                    and len(d) == 4
                    and d[0] in body_names
                    and isinstance(d[1], str)
                    and isinstance(d[2], (int, float))
                    and not isinstance(d[2], bool)
                    and np.isfinite(d[2])
                    and d[2] >= 0
                    and d[3] is False
                    for d in ds
                ):
                    continue
                pairs = {(d[0], d[1]) for d in ds}
                if len(pairs) != len(ds) or not expected <= pairs:
                    continue
                if key == "self" and pairs != expected:
                    continue
                # Static rows require a complete caller-supplied shape signature;
                # unknown obstacles are process-local and are never restored.
                if key != "self" and not key.startswith("static:"):
                    continue
                self.samples.setdefault(key, []).append((q, ds))
            except (ValueError, KeyError, TypeError, AttributeError):
                continue

    def flush(self):
        rows = [
            {"key": key, "q": q.tolist(), "distances": ds}
            for key, values in self.samples.items()
            if key == "self" or key.startswith("static:")
            for q, ds in values[-self.maximum_samples :]
        ]
        _save(
            self.directory / "samples.json",
            {
                "schema_version": SCHEMA_VERSION,
                "validator_version": VALIDATOR_VERSION,
                "context_hash": self.context_hash,
                "rows": rows,
                "digest": digest(rows),
            },
        )
        self._pending = 0
        self._last_flush = monotonic()

    def _key(self, obstacles):
        return obstacle_context(obstacles, self.static_geometry, self._obstacle_refs)

    def material_distances(
        self, positions, material, clearance_bounds=None, *, deadline=float("inf")
    ):
        started = perf_counter()
        try:
            yield from self.model.material_distances(
                positions, material, clearance_bounds, deadline=deadline
            )
        finally:
            self.material_seconds += perf_counter() - started

    def distances(self, positions, obstacles=None, *, deadline=float("inf")):
        started = perf_counter()
        try:
            yield from self._distances(positions, obstacles, deadline=deadline)
        finally:
            self.self_static_seconds += perf_counter() - started

    def _distances(self, positions, obstacles=None, *, deadline=float("inf")):
        if monotonic() >= deadline:
            raise TimeoutError("Clearance cache exceeded its budget")
        q = np.asarray(positions, float)
        self.robot.frames(q)
        key = self._key(obstacles) if obstacles else "self"
        pool = self.samples.get(key, [])
        expected = list(self.model._pair_radii)
        expected.extend(
            (b.name, name)
            for b in self.bodies
            for name in (obstacles or {})
            if not (name == "table" and b.link in self.mount_links)
        )
        lower = {}
        if pool:
            signature = (id(pool), len(pool), id(pool[-1]))
            cached = self._atlas_arrays.get(key)
            if cached is None or cached[0] != signature or cached[1] != expected:
                qs = np.asarray([p[0] for p in pool])
                sample_maps = [
                    {(a, b): d for a, b, d, _ in rows} for _, rows in pool
                ]
                distances = np.asarray(
                    [
                        [sample.get(pair, -np.inf) for pair in expected]
                        for sample in sample_maps
                    ]
                )
                radii = np.asarray([
                    self._pair_radii[pair] if pair in self._pair_radii
                    else self.motion_radii[pair[0]]
                    for pair in expected
                ])
                cached = (signature, expected, qs, distances, radii)
                self._atlas_arrays[key] = cached
            _, _, qs, distances, radii = cached
            # Rank nearby configurations to bound overhead as the library grows.
            count = min(8, len(qs))
            nearest = np.argpartition(np.linalg.norm(qs - q, axis=1), count - 1)[:count]
            if expected:
                bounds = np.max(distances[nearest] - np.abs(qs[nearest] - q) @ radii.T, axis=0)
                lower = dict(zip(expected, bounds.tolist(), strict=True))
        # Positive lower bounds remain useful to the continuous checker even
        # when too small to certify its whole current interval.
        if set(lower) == set(expected) and all(
            lower[p] > getattr(self, "_reuse_margin", 0.0) + 1e-6 for p in expected
        ):
            self.hits += 1
            for first, second in expected:
                yield first, second, lower[first, second], False
            return
        self.misses += 1
        rows = list(self.model.distances(q, obstacles, deadline=deadline))
        if all(not d[3] and d[2] >= 0 for d in rows):
            pool = self.samples.setdefault(key, [])
            pool.append((q.copy(), rows))
            del pool[: -self.maximum_samples]
            self._atlas_arrays.pop(key, None)
            if obstacles:
                self_rows = [d for d in rows if (d[0], d[1]) in self._pair_radii]
                self_pool = self.samples.setdefault("self", [])
                self_pool.append((q.copy(), self_rows))
                del self_pool[: -self.maximum_samples]
                self._atlas_arrays.pop("self", None)
            self._pending += 1
            # Serialization hashes and rewrites the complete bounded atlas.
            # Avoid turning short collision queries into repeated multi-MB
            # writes. Runtime owners flush explicitly at lifecycle boundaries;
            # long-running users also get at most one periodic save per minute.
            if self._pending >= 512 and monotonic() - self._last_flush >= 60:
                self.flush()
        yield from rows


def fixed_orientation_path_hash(drawing, clock=None):
    return digest(
        {
            "positions": drawing["positions"],
            "tcp_targets_m": drawing["tcp_targets_m"],
            "tcp_rotation": drawing["tcp_rotation"],
            "clock": clock,
        }
    )


def certify_fixed_orientation(arm, drawing, *, clock=None, deadline=float("inf")):
    """Compute an actual continuous FK bound; metadata never substitutes for this check."""
    from .clock_path import ClockPath

    if drawing.get("free_tool_orientation", False) or drawing.get("tcp_rotation") is None:
        raise ValueError("A cached drawing must bind one fixed TCP rotation")
    DrawingRequest(drawing["tcp_targets_m"], fixed_tcp_rotation=drawing["tcp_rotation"])
    rotation = np.asarray(drawing["tcp_rotation"])
    rotations = drawing.get("tcp_rotations")
    if rotations is not None and not np.allclose(rotations, rotation, atol=1e-10, rtol=0):
        raise ValueError("Per-point orientation variation is forbidden in cached drawings")
    if clock is None:
        from ..timing import make_path
        from .smooth_audit import CubicPiece

        joints = np.asarray(drawing["positions"])
        points = np.asarray([arm.robot.forward(q, arm.flange_to_tcp)[:3, 3] for q in joints])
        keep = np.r_[True, np.linalg.norm(np.diff(joints, axis=0), axis=1) > 1e-10]
        joints, points = joints[keep], points[keep]
        if len(joints) < 2:
            pieces = [PolynomialPiece(Polynomial([q]) for q in joints[0])]
        else:
            spline = make_path(joints, points, interpolation="pchip", subdivisions=2)
            pieces = [
                CubicPiece(spline.cspl.c[:, i, :], width)
                for i, width in enumerate(np.diff(spline.cspl.x))
            ]
        worst = 0.0
        for piece in pieces:
            if monotonic() >= deadline:
                raise TimeoutError("Fixed orientation validation deadline")
            ends = [arm.robot.forward(piece.position(t), arm.flange_to_tcp)[:3, 3] for t in (0, 1)]
            checked = check_tracking(
                piece,
                arm.robot,
                arm.flange_to_tcp,
                *ends,
                rotation,
                float("inf"),
                ORIENTATION_LIMIT_RAD,
            )
            if checked["status"] != "BOUNDED":
                raise ValueError("Cached DRAW contains a TCP orientation excursion")
            worst = max(worst, checked["orientation_error_bound_rad"])
        interpolation = "pchip"
    else:
        worst, count = 0.0, 0
        for edge in ClockPath.from_record(clock).edges:
            if edge.kind != "DRAW":
                continue
            if monotonic() >= deadline:
                raise TimeoutError("Fixed orientation validation deadline")
            piece = edge.piece(edge.start_s, edge.end_s)
            ends = [arm.robot.forward(piece.position(t), arm.flange_to_tcp)[:3, 3] for t in (0, 1)]
            checked = check_tracking(
                piece,
                arm.robot,
                arm.flange_to_tcp,
                *ends,
                rotation,
                float("inf"),
                ORIENTATION_LIMIT_RAD,
            )
            if checked["status"] != "BOUNDED":
                raise ValueError("Timed DRAW contains a TCP orientation excursion")
            worst = max(worst, checked["orientation_error_bound_rad"])
            count += 1
        if not count:
            raise ValueError("A cached clock needs DRAW intervals")
        interpolation = "exact_clock_draw"
    return {
        "policy": FIXED_ORIENTATION_POLICY,
        "status": "BOUNDED",
        "fixed_tcp_rotation": rotation.tolist(),
        "orientation_limit_rad": ORIENTATION_LIMIT_RAD,
        "orientation_error_bound_rad": worst,
        "interpolation": interpolation,
        "subdivisions": 2 if clock is None else None,
        "path_sha256": fixed_orientation_path_hash(drawing, clock),
    }


class PublishedRecord(dict):
    """Selected runtime payload retaining its original immutable evidence identity."""
    __slots__ = ("published_digest",)


class MotionLibrary:
    def __init__(self, directory, *, historical=False, load_records=True, lazy=False):
        self.directory = Path(directory)
        self.historical = historical
        self.lazy = lazy and not historical
        self._selected_records = OrderedDict()
        (self.directory / "motions").mkdir(parents=True, exist_ok=True)
        (self.directory / "attempts").mkdir(parents=True, exist_ok=True)
        self.records = []
        self._loaded_files = {}
        self._record_keys = {}
        self.generation = 0
        # A frozen library supplies proposals but accepts no records or attempts.
        self.frozen = False
        if load_records:
            self.refresh()

    def record_key(self, record):
        """Published digest of a loaded record, without rehashing its dense clock."""
        key = getattr(record, "published_digest", self._record_keys.get(id(record)))
        return digest(record) if key is None else key

    def refresh(self):
        """Admit published records without reparsing unchanged dense clocks.

        Independent cache producers may publish while an object is planned.
        New/changed files still pass checksum, schema and fixed-orientation
        proof checks. Removed or corrupt files cease to supply proposals.
        """
        if self.frozen:
            return False
        loaded, records = {}, []
        for path in sorted((self.directory / "motions").glob("*.json")):
            try:
                stat = path.stat()
                signature = (stat.st_mtime_ns, stat.st_size)
                previous = self._loaded_files.get(path)
                if previous is not None and previous[0] == signature:
                    record = previous[1]
                else:
                    if self.lazy:
                        record, key = self._index_record(path, signature)
                    else:
                        envelope = json.loads(path.read_text())
                        record, key = envelope["record"], envelope["digest"]
                        if digest(record) != key:
                            continue
                        if not self.historical:
                            self._validate_record(record)
                    self._record_keys[id(record)] = key
                loaded[path] = (signature, record)
                records.append(record)
            except (OSError, ValueError, TypeError, KeyError):
                continue
        changed = set(loaded) != set(self._loaded_files) or any(
            old[0] != loaded[path][0] for path, old in self._loaded_files.items() if path in loaded
        )
        if changed:
            self.generation += 1
        self._loaded_files, self.records = loaded, records
        return changed

    def _index_record(self, path, signature):
        """Admit a compact descriptor once; selected original files are reverified."""
        index_path = self.directory / "index" / path.name
        try:
            indexed = json.loads(index_path.read_text())
            contents = {key: value for key, value in indexed.items() if key != "checksum"}
            if (indexed["checksum"] == digest(contents)
                    and indexed["signature"] == list(signature)
                    and indexed["policy"] == "lazy-motion-index-v1"
                    and indexed.get("admission_implementation") == ADMISSION_IMPLEMENTATION):
                return indexed["descriptor"], indexed["record_digest"]
        except (OSError, ValueError, KeyError, TypeError):
            pass
        from ..io import sha256

        envelope = json.loads(path.read_text())
        record, key = envelope["record"], envelope["digest"]
        if digest(record) != key:
            raise ValueError("Corrupt motion record")
        self._validate_record(record)
        trajectory = record["trajectory"]
        q = np.asarray(trajectory["q_path"], float)
        progress = _arc(np.asarray(trajectory["source_points_m"], float))
        vertices = _vertices(record["request"]["source_points_m"])
        corners = _arc(vertices)
        grid = np.unique(np.r_[np.linspace(0, progress[-1], 32),
                               corners / corners[-1] * progress[-1]])
        sampled = np.column_stack([np.interp(grid, progress, joint) for joint in q.T])
        signature_curve = [list(sampled.shape), np.round(sampled, 5).tobytes().hex(),
                           np.round(vertices, 7).tobytes().hex()]
        descriptor = {"request": record["request"],
                      "validation": {key: record["validation"].get(key)
                                     for key in ("scope", "context_hash", "status")},
                      "_lazy": {"file": path.name, "file_sha256": sha256(path),
                                "record_digest": key, "curve_signature": signature_curve,
                                "endpoints": [q[0].tolist(), q[-1].tolist()]}}
        contents = {"policy": "lazy-motion-index-v1", "signature": list(signature),
                    "admission_implementation": ADMISSION_IMPLEMENTATION,
                    "record_digest": key, "descriptor": descriptor}
        _save(index_path, {**contents, "checksum": digest(contents)})
        return descriptor, key

    def _materialize(self, record):
        lazy = record.get("_lazy")
        if lazy is None:
            return record
        from ..io import sha256

        path = self.directory / "motions" / lazy["file"]
        # Check the original bytes even when a selected object is already cached.
        if sha256(path) != lazy["file_sha256"]:
            raise ValueError("Stale selected motion record")
        key = lazy["record_digest"]
        if key in self._selected_records:
            self._selected_records.move_to_end(key)
            return self._selected_records[key]
        envelope = json.loads(path.read_text())
        full = envelope["record"]
        # The file's bytes were just checked against the index, which also
        # names this digest; re-serializing megabytes to recompute it per
        # lookup cost more than the lookup.
        if envelope["digest"] != key:
            raise ValueError("Corrupt selected motion record")
        self._validate_record(full)
        # Historical scenes and duplicate validation clocks remain in the
        # original evidence file, rather than every planning worker.
        full = PublishedRecord({name: value for name, value in full.items() if name != "scene"})
        full.published_digest = key
        full["validation"] = {name: value for name, value in full["validation"].items()
                              if name != "clock_record"}
        self._selected_records[key] = full
        while len(self._selected_records) > SELECTED_RECORDS:
            self._selected_records.popitem(last=False)
        return full

    @staticmethod
    def _validate_record(record):
        if record["schema_version"] != SCHEMA_VERSION:
            raise ValueError("Unsupported motion schema")
        DrawingRequest(**record["request"])
        q = np.asarray(record["trajectory"]["q_path"], float)
        p = np.asarray(record["trajectory"]["source_points_m"], float)
        if q.ndim != 2 or len(q) < 2 or p.shape != (len(q), 3):
            raise ValueError("Path/target dimensions do not match")
        if not np.isfinite(q).all() or not np.isfinite(p).all():
            raise ValueError("Nonfinite trajectory")
        validation = record["validation"]
        if validation["status"] != "VERIFIED_VALID":
            raise ValueError("Only verified motions belong in the proposal library")
        trajectory = record["trajectory"]
        proof = validation.get("fixed_orientation", {})
        rotation = record["request"].get("fixed_tcp_rotation")
        declared_rotations = trajectory.get("tcp_rotations")
        if declared_rotations is not None and not np.allclose(
            declared_rotations, rotation, atol=1e-10, rtol=0
        ):
            raise ValueError("Stored DRAW target orientations vary within an action")
        if (
            rotation is None
            or trajectory.get("free_tool_orientation", False)
            or proof.get("policy") != FIXED_ORIENTATION_POLICY
            or proof.get("status") != "BOUNDED"
            or proof.get("orientation_limit_rad") != ORIENTATION_LIMIT_RAD
            or proof.get("interpolation")
            != ("exact_clock_draw" if trajectory.get("exact_clock") is not None else "pchip")
            or (trajectory.get("exact_clock") is None and proof.get("subdivisions") != 2)
            or not 0
            <= proof.get("orientation_error_bound_rad", float("inf"))
            <= ORIENTATION_LIMIT_RAD
            or not np.array_equal(rotation, proof.get("fixed_tcp_rotation"))
            or not np.array_equal(rotation, trajectory.get("tcp_rotation"))
            or proof.get("path_sha256")
            != fixed_orientation_path_hash(
                {
                    "positions": trajectory["q_path"],
                    "tcp_targets_m": trajectory["source_points_m"],
                    "tcp_rotation": trajectory["tcp_rotation"],
                },
                trajectory.get("exact_clock"),
            )
        ):
            raise ValueError("Cached DRAW requires a bound, unchanged fixed TCP orientation proof")
        if validation.get("scope") in OFFLINE_GEOMETRY_SCOPES:
            required = {
                "source_tracking": "BOUNDED",
                "self_static_status": "CERTIFIED_MODELED_PATH",
                "drawing_policy": DRAWING_POLICY_VERSION,
            }
            if (
                any(validation.get(key) != value for key, value in required.items())
                or not record["request"].get("context_hash")
                or validation.get("context_hash") != record["request"]["context_hash"]
                or record["trajectory"].get("exact_clock") is not None
                or record["trajectory"].get("q_path_validation") != "SEED_ONLY"
            ):
                raise ValueError(
                    "Self/static candidates require scoped geometry proof "
                    "and fresh process validation"
                )
        if validation.get("scope") == GROWING_PREFIX_SCOPE:
            trajectory = record["trajectory"]
            checked_path = {
                key: trajectory[key]
                for key in ("q_path", "source_points_m", "tcp_rotation", "free_tool_orientation")
            }
            if (
                validation.get("progressive_material_checked") is not True
                or validation.get("growing_prefix_status") != "CERTIFIED_MODELED_PATH"
                or validation.get("growing_prefix_policy") != GROWING_PREFIX_POLICY
                or validation.get("growing_prefix_trajectory_sha256") != digest(checked_path)
                or validation.get("growing_prefix_intervals", 0) < 1
                or not np.isfinite(validation.get("material_radius_m", np.nan))
                or validation.get("material_radius_m", 0) <= 0
                or not validation.get("initial_material_sha256")
                or validation.get("timing_checked") is not False
            ):
                raise ValueError("Growing-prefix candidates require bound path/material evidence")

    def retrieve(
        self,
        request,
        limit=4,
        reference_q=None,
        reference_endpoint="start",
        *,
        compatible_only=False,
    ):
        """Rank geometry and posture without making an endpoint constraint.

        Score is integrated and worst-vertex Cartesian error in meters plus
        5 mm per RMS radian of endpoint joint difference. The hint only orders proposals;
        their exact request and continuous acceptance checks remain unchanged.
        """
        if self.historical:
            raise ValueError("Historical libraries cannot supply motion proposals")
        if reference_endpoint not in ("start", "end"):
            raise ValueError("reference_endpoint must be 'start' or 'end'")
        if limit < 1:
            raise ValueError("Retrieval limit must be positive")
        source = request.source_points_m
        if reference_q is None:
            reference_q = request.q_start if reference_endpoint == "start" else request.q_goal
        reference = None if reference_q is None else np.asarray(reference_q, float)
        if reference is not None and (reference.ndim != 1 or not np.isfinite(reference).all()):
            raise ValueError("Reference posture must be a finite joint vector")

        def score(record):
            other = record["request"]["source_points_m"]
            geometry = _geometry_distance(source, other)
            if reference is None:
                return geometry
            endpoint = 0 if reference_endpoint == "start" else -1
            q = np.asarray(record["_lazy"]["endpoints"][endpoint] if "_lazy" in record
                           else record["trajectory"]["q_path"][endpoint])
            if q.shape != reference.shape:
                return float("inf")
            return geometry + 0.005 * float(np.sqrt(np.mean((q - reference) ** 2)))

        # Repeated validations of the same motion against different scenes must
        # not consume the alternative slots. Compare the joint curve at shared
        # source-progress samples so different knot counts do not create copies.
        # Keep the best-ranked source record for each equivalent seed curve.
        selected, signatures = [], set()
        records = (
            record
            for record in self.records
            if not compatible_only or compatible_geometry(request, record)
        )
        for record in sorted(records, key=score):
            if "_lazy" in record:
                shape, joints, vertices = record["_lazy"]["curve_signature"]
                signature = (tuple(shape), bytes.fromhex(joints), bytes.fromhex(vertices))
            else:
                trajectory = record["trajectory"]
                q = np.asarray(trajectory["q_path"], float)
                progress = _arc(np.asarray(trajectory["source_points_m"], float))
                vertices = _vertices(record["request"]["source_points_m"])
                corners = _arc(vertices)
                grid = np.unique(
                    np.r_[np.linspace(0, progress[-1], 32), corners / corners[-1] * progress[-1]]
                )
                sampled = np.column_stack([np.interp(grid, progress, joint) for joint in q.T])
                signature = (
                    sampled.shape,
                    np.round(sampled, decimals=5).tobytes(),
                    np.round(vertices, decimals=7).tobytes(),
                )
            if signature in signatures:
                continue
            try:
                full = self._materialize(record)
            except (OSError, ValueError, KeyError, TypeError):
                continue
            signatures.add(signature)
            selected.append(full)
            if len(selected) == limit:
                break
        # Different contexts remain valid *seeds*, never inherited certificates.
        return selected

    def record_attempt(self, request, drawing, validation, scene=None, provenance=None):
        if self.historical:
            raise ValueError("Historical libraries are read-only")
        if validation.get("status") not in {
            "VERIFIED_VALID",
            "COLLISION",
            "CONSTRAINT_VIOLATION",
            "UNRESOLVED",
        }:
            raise ValueError("Attempt needs an explicit outcome label")
        if self.frozen:
            return None, None
        record = {
            "schema_version": SCHEMA_VERSION,
            "request": request.as_dict(),
            "scene": scene or {"capsules": []},
            "trajectory": None
            if drawing is None
            else {
                "q_path": drawing["positions"],
                "source_points_m": drawing["tcp_targets_m"],
                "tcp_rotation": drawing.get("tcp_rotation"),
                "free_tool_orientation": drawing.get("free_tool_orientation", False),
                "exact_clock": validation.get("clock_record"),
                # Every q path is a seed for fresh runtime interpolation checks.
                "q_path_validation": "SEED_ONLY",
            },
            "validation": validation,
            "provenance": provenance or {},
        }
        key = digest(record)
        _save(self.directory / "attempts" / f"{key}.json", {"record": record, "digest": key})
        return key, record

    def remember(self, request, drawing, validation, scene=None, provenance=None, *, arm=None):
        if self.historical:
            raise ValueError("Historical libraries are read-only")
        if self.frozen:
            raise ValueError("A frozen motion library accepts no new records")
        if arm is None:
            raise ValueError("Actual robot model is required to certify cached fixed orientation")
        rotation = drawing.get("tcp_rotation")
        if request.fixed_tcp_rotation is not None and not np.array_equal(
            request.fixed_tcp_rotation, rotation
        ):
            raise ValueError("Drawing and request must use the same fixed TCP rotation")
        proof = certify_fixed_orientation(arm, drawing, clock=validation.get("clock_record"))
        request = replace(request, fixed_tcp_rotation=rotation)
        validation = {**validation, "fixed_orientation": proof}
        key, record = self.record_attempt(request, drawing, validation, scene, provenance)
        self._validate_record(record)
        if not (self.directory / "motions" / f"{key}.json").exists():
            _save(self.directory / "motions" / f"{key}.json", {"record": record, "digest": key})
            self.records.append(record)
        return key

    def remember_self_static(self, request, drawing, validation, provenance=None, *, arm=None):
        """Admit an offline geometric candidate, with no material or process claim.

        The caller supplies continuous self/static and source-tracking evidence.
        Retrieval must still adapt/recheck the candidate and normal construction
        planning must validate the current growing material and process clock.
        """
        if (
            validation.get("scope") != SELF_STATIC_SCOPE
            or validation.get("clock_record") is not None
        ):
            raise ValueError(
                "Offline candidates must have SELF_STATIC_ONLY scope and no timed clock"
            )
        return self.remember(
            request,
            drawing,
            validation,
            scene={
                "capsules": [],
                "kind": "offline_self_static_only",
                "material_validation": "NOT_PERFORMED",
            },
            provenance={
                **(provenance or {}),
                "drawing_policy": DRAWING_POLICY_VERSION,
                "requires_current_material_process_validation": True,
            },
            arm=arm,
        )

    def remember_growing_prefix(self, request, drawing, validation, provenance=None, *, arm=None):
        """Admit a whole path checked against its own ordered deposited material.

        The cache remains a seed: adaptation, other printed structure, approach,
        departure, cooling, and the full process clock require current validation.
        """
        if validation.get("scope") != GROWING_PREFIX_SCOPE or validation.get("clock_record"):
            raise ValueError("Expected growing-prefix geometry proof without a timed clock")
        return self.remember(
            request,
            drawing,
            validation,
            scene={
                "capsules": [],
                "kind": "offline_growing_prefix",
                "initial_material_sha256": validation.get("initial_material_sha256"),
                "material_validation": "OWN_ORDERED_PREFIX_ONLY",
            },
            provenance={
                **(provenance or {}),
                "drawing_policy": DRAWING_POLICY_VERSION,
                "requires_current_material_process_validation": True,
            },
            arm=arm,
        )

    def coverage(self, requests):
        """Report available seeds by exact source/context, without claiming printability.

        ``requests`` maps caller-owned identifiers (e.g. section/index/direction)
        to DrawingRequest objects. Family-compatible seeds have the same vertex
        count; a single straight leg does not cover a whole zigzag request.
        """
        # Full-motion records may embed tens of MB of timing polynomials. Hash
        # each record once per audit, not once for every matching source root.
        # Scope stays unchanged: all requested starts/directions/indices count.
        descriptors = [
            (
                record,
                _vertices(record["request"]["source_points_m"]),
                _geometry_kind(record["request"]),
            )
            for record in self.records
        ]
        # A loaded record's published digest was verified against its content.
        fingerprints = {id(record): self.record_key(record) for record, _, _ in descriptors}
        rows = []
        for identity, request in requests.items():
            vertices = _vertices(request.source_points_m)
            exact, compatible, other_context = [], [], []
            requested_kind = _geometry_kind(request.as_dict())
            for record, previous, stored_kind in descriptors:
                old = record["request"]
                kinds_match = (
                    requested_kind == "unknown"
                    or stored_kind == "unknown"
                    or requested_kind == stored_kind
                )
                if (
                    not kinds_match
                    or previous.shape != vertices.shape
                    or old["table_index"] != request.table_index
                ):
                    continue
                context_matches = (
                    bool(request.context_hash)
                    and old.get("context_hash") == request.context_hash
                    and record["validation"].get("context_hash") == request.context_hash
                )
                if not context_matches:
                    other_context.append(record)
                elif np.allclose(previous, vertices, atol=1e-9, rtol=0):
                    exact.append(record)
                else:
                    compatible.append(record)
            available = exact + compatible
            rows.append(
                {
                    "id": identity,
                    "source_object": request.source_object,
                    "source_fingerprint": digest(
                        {"vertices": vertices.tolist(), "table_index": request.table_index}
                    ),
                    "context_hash": request.context_hash,
                    "table_index": request.table_index,
                    "path_kind": request.path_kind,
                    "source_vertices": len(vertices),
                    "exact_source_seeds": len(exact),
                    "compatible_family_seeds": len(compatible),
                    "other_context_seeds": len(other_context),
                    "self_static_only_seeds": sum(
                        record["validation"].get("scope") == SELF_STATIC_SCOPE
                        for record in available
                    ),
                    "growing_prefix_seeds": sum(
                        record["validation"].get("scope") == GROWING_PREFIX_SCOPE
                        for record in available
                    ),
                    "previous_full_motion_seeds": sum(
                        record["validation"].get("scope")
                        == "indexed_local_motion_with_progressive_material_and_timing"
                        for record in available
                    ),
                    "record_ids": [fingerprints[id(record)] for record in available],
                    "readiness": (
                        "EXACT_SOURCE_SEEDS"
                        if exact
                        else "COMPATIBLE_FAMILY_SEEDS"
                        if compatible
                        else "MISSING"
                    ),
                    "requires_current_material_process_validation": True,
                }
            )
        missing = [row["id"] for row in rows if row["readiness"] == "MISSING"]
        return {
            "status": "SEED_COVERAGE_COMPLETE"
            if rows and not missing
            else "SEED_COVERAGE_INCOMPLETE",
            "drawing_policy": DRAWING_POLICY_VERSION,
            "record_count": len(self.records),
            "counts": {
                "requests": len(rows),
                "requests_with_seeds": len(rows) - len(missing),
                "requests_without_seeds": len(missing),
            },
            "requests": rows,
            "missing_request_ids": missing,
            "material_process_validation": "REQUIRED_PER_CURRENT_REQUEST",
        }

    def adapt_proposal(self, drawing, request, arm, obstacles, *, deadline, clearance_m=0.001):
        """Project an untrusted generated seed through the same continuous checks."""
        if self.historical:
            raise ValueError("Historical libraries cannot adapt motion proposals")
        record = {
            "trajectory": {
                "q_path": drawing["positions"],
                "source_points_m": drawing["tcp_targets_m"],
                "tcp_rotation": drawing.get("tcp_rotation"),
                "free_tool_orientation": drawing.get("free_tool_orientation", False),
            }
        }
        q, p = np.asarray(drawing["positions"], float), np.asarray(drawing["tcp_targets_m"], float)
        if q.ndim != 2 or len(q) < 2 or p.shape != (len(q), 3):
            return None
        if not np.isfinite(q).all() or not np.isfinite(p).all():
            return None
        return self._adapt(
            record, request, arm, obstacles, deadline=deadline, clearance_m=clearance_m
        )

    def adapt(self, record, request, arm, obstacles, *, deadline, clearance_m=0.001):
        if self.historical:
            raise ValueError("Historical libraries cannot adapt motion proposals")
        self._validate_record(record)
        proposal = self._adapt(
            record, request, arm, obstacles, deadline=deadline, clearance_m=clearance_m
        )
        if (
            proposal is None
            and record["trajectory"].get("exact_clock") is not None
            and len(record["trajectory"]["q_path"]) > 3 * len(request.source_points_m)
            and monotonic() < deadline
        ):
            proposal = self._adapt(
                record, request, arm, obstacles, deadline=deadline,
                clearance_m=clearance_m, compact_seed=False,
            )
        return proposal

    def _adapt(
        self, record, request, arm, obstacles, *, deadline, clearance_m=0.001, compact_seed=True
    ):
        """Trim or extend in physical stroke distance, then locally project IK.

        Returned drawing is self/static and source-tracking checked. It is still
        a proposal for material, deposition, timing, approach and withdrawal.
        ``last_adaptation_failure`` names the reason and point of a failure.
        """
        self.last_adaptation_failure = None

        def failed(reason, point=None, **detail):
            self.last_adaptation_failure = {"reason": reason, "point": point, **detail}

        old = record["trajectory"]
        source, old_source = np.asarray(request.source_points_m), np.asarray(old["source_points_m"])
        old_q = np.asarray(old["q_path"])
        original_seed_points = len(old_q)
        if (
            compact_seed and old.get("exact_clock") is not None
            and len(old_q) > 3 * len(source)
        ):
            old_source, old_q = _compact_timed_seed(
                old_source, old_q, record["request"]["source_points_m"]
            )
        if old_q.shape[1] != len(arm.robot.names):
            return failed("seed_shape")
        for fixed in (request.q_start, request.q_goal):
            if fixed is not None and (
                np.shape(fixed) != (len(arm.robot.names),)
                or np.any(fixed < arm.robot.lower)
                or np.any(fixed > arm.robot.upper)
            ):
                return failed("fixed_endpoint")
        old_q = np.clip(old_q, arm.robot.lower, arm.robot.upper)
        # Preserve original source breakpoints; add intermediate targets only.
        points = [source[0]]
        for a, b in zip(source[:-1], source[1:], strict=True):
            count = max(1, int(np.ceil(np.linalg.norm(b - a) / 0.003)))
            points.extend(a + (b - a) * t for t in np.linspace(0, 1, count + 1)[1:])
        points = _preserve_seed_samples(np.asarray(points), old_source)
        arc, old_arc = _arc(points), _arc(old_source)
        if old_arc[-1] <= 0:
            return failed("seed_shape")
        direction = old_source[-1] - old_source[0]
        length = np.linalg.norm(direction)
        aligned = (
            length > 1e-12
            and np.max(
                np.linalg.norm(np.cross(old_source - old_source[0], direction / length), axis=1)
            )
            < 1e-8
        )
        if aligned:
            coordinates = (points - old_source[0]) @ (direction / length)
        else:
            # Match corresponding vertices before interpolating along each
            # leg. Global arclength alone shifts corners when individual leg
            # lengths change, seeding an unrelated robot posture at the turn.
            old_geometry = np.asarray(record.get("request", {}).get("source_points_m", old_source))
            vertices, old_vertices = _vertices(source), _vertices(old_geometry)
            if len(vertices) == len(old_vertices):
                coordinates = np.interp(
                    arc, _arc(vertices), _arc(old_vertices) / _arc(old_vertices)[-1] * old_arc[-1]
                )
            else:
                coordinates = arc / arc[-1] * old_arc[-1]
        seeds = np.column_stack([np.interp(coordinates, old_arc, values) for values in old_q.T])
        original_rotation = old.get("tcp_rotation")
        if original_rotation is None or old.get("free_tool_orientation", False):
            return failed("seed_orientation")
        rotation = np.asarray(
            request.fixed_tcp_rotation
            if request.fixed_tcp_rotation is not None
            else min(
                request.allowed_orientations,
                key=lambda r: np.linalg.norm(np.asarray(r) - original_rotation),
            )
            if request.allowed_orientations
            else original_rotation
        )
        edge_checks = getattr(self, "adaptation_edge_checks", True)

        def candidates(i, previous, wide):
            """Joint candidates for point ``i``; ``wide`` searches harder for alternatives."""
            point, seed = points[i], seeds[i]
            fixed = request.q_start if i == 0 else request.q_goal if i == len(points) - 1 else None
            if fixed is not None:
                return [np.asarray(fixed)]
            target = arm.robot.forward(seed, arm.flange_to_tcp).copy()
            target[:3, 3] = point
            if rotation is not None:
                target[:3, :3] = rotation
            found = [seed]
            if wide or (
                np.linalg.norm(pose_error(arm.robot.forward(seed, arm.flange_to_tcp), target))
                > 1e-5
            ):
                starts = [seed, *([] if previous is None else [previous])]
                if wide:
                    starts.extend(seeds[max(0, i - 1) : i + 2])
                found = [
                    np.asarray(row["positions"])
                    for row in inverse_candidates(
                        arm.robot,
                        target,
                        arm.flange_to_tcp,
                        starts,
                        position_tolerance=0.00005,
                        orientation_tolerance=0.001,
                        maximum_evaluations=240 if wide else 60,
                    )
                ]
            reference = seed if previous is None else previous
            return sorted(found, key=lambda q: np.linalg.norm(q - reference))

        def rejection(i, previous, q):
            """The check ``q`` fails at point ``i``, or None when it is acceptable."""
            if np.linalg.norm(arm.robot.forward(q, arm.flange_to_tcp)[:3, 3] - points[i]) > 0.0003:
                return "off_target"
            if previous is None or not edge_checks:
                if arm.collision.check(q, obstacles, clearance_m, deadline=deadline):
                    return "collision"
            if previous is None:
                return None
            piece = PolynomialPiece(
                Polynomial([a, b - a]) for a, b in zip(previous, q, strict=True)
            )
            tracked = check_tracking(
                piece, arm.robot, arm.flange_to_tcp, points[i - 1], points[i], rotation,
                0.0003, 0.003,
            )
            if tracked["status"] != "BOUNDED":
                return "tracking"
            if edge_checks:
                checked = arm.collision.check_edge(
                    previous, q, obstacles, clearance_m, maximum_depth=20, deadline=deadline
                )
                if checked["status"] != "CERTIFIED_MODELED_EDGE":
                    return "collision"
            return None

        # Depth-first over joint candidates: a dead end backs up over a few
        # preceding points and retries their alternatives before giving up.
        path, banned, backtracks = [], {}, 0
        furthest = {"point": -1}
        while len(path) < len(points):
            i, previous = len(path), path[-1] if path else None
            if monotonic() >= deadline:
                return failed("deadline", i, points=len(points), backtracks=backtracks)
            selected, stage = None, "no_inverse_kinematics"
            seen = list(banned.get(i, ()))
            # Search wider only where the nearest candidates already failed.
            for wide in (False, True):
                for q in candidates(i, previous, wide):
                    if any(np.allclose(q, other, atol=1e-9, rtol=0) for other in seen):
                        continue
                    seen.append(q)
                    reason = rejection(i, previous, q)
                    if reason is None:
                        selected = q
                        break
                    stage = max(stage, reason, key=ADAPTATION_STAGES.index)
                if selected is not None:
                    break
            tried = len(seen) - len(banned.get(i, ()))
            if selected is not None:
                path.append(selected)
                continue
            if i >= furthest["point"]:
                furthest = {"point": i, "stage": stage, "candidates": tried}
            if (
                not path
                or backtracks >= ADAPTATION_BACKTRACKS
                or furthest["point"] - (i - 1) > ADAPTATION_BACKTRACK_DEPTH
            ):
                return failed(
                    furthest["stage"],
                    furthest["point"],
                    points=len(points),
                    candidates=furthest["candidates"],
                    backtracks=backtracks,
                )
            # The predecessor led here; alternatives at ``i`` depend on it.
            banned.pop(i, None)
            banned.setdefault(i - 1, []).append(path.pop())
            backtracks += 1
        provenance = record.get("provenance", {})
        preference = provenance.get("orientation_preference", {})
        seed_preferences = {}
        for name in ("pen_tilt_deg", "roll_rad", "policy"):
            direct_name = "orientation_policy" if name == "policy" else name
            value = provenance.get(direct_name, preference.get(name))
            if value is not None:
                seed_preferences[name] = value
        return {
            **({"seed_orientation_preference": seed_preferences} if seed_preferences else {}),
            "status": "SAMPLED_DRAWING_PATH",
            "arm": arm.identity,
            "positions": np.asarray(path).tolist(),
            "tcp_targets_m": points.tolist(),
            "tcp_rotation": (
                rotation
                if rotation is not None
                else arm.robot.forward(path[0], arm.flange_to_tcp)[:3, :3]
            ).tolist(),
            "free_tool_orientation": rotation is None,
            "static_collision_status": (
                "CERTIFIED_MODELED_PATH" if edge_checks else "CHECKED_AT_WAYPOINTS_ONLY"
            ),
            "tracking_status": "BOUNDED",
            "adaptation_backtracks": backtracks,
            "table_index": request.table_index,
            "motion_library_parent": digest(record),
            "variant": "cache-adapted",
            "drawing_policy": DRAWING_POLICY_VERSION,
            "adaptation_seed_points": len(old_q),
            "original_seed_points": original_seed_points,
        }
