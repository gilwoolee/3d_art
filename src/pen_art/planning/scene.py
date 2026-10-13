"""Incremental capsule scene for conservative deposited-strand geometry."""

from collections import Counter
from hashlib import sha256
from time import monotonic, perf_counter

import fcl
import numpy as np

from .collision import placed


class MaterialScene:
    def __init__(self):
        self.table_index = 0
        self.manager = fcl.DynamicAABBTreeCollisionManager()
        self.manager.setup()
        self.objects = []
        self.geometry_ids = []
        self.names = {}
        self.segment_names = set()
        self.segments = []
        # Physical snapshots belong to immutable FCL capsules, independently of
        # editable report metadata in ``segments``. Prefixes bind world geometry
        # and insertion order; scene copies can safely share unchanged capsules.
        self._physical_segments = []
        self._geometry_prefixes = [sha256(b"material-capsules-v1").digest()]
        self._dirty = False
        self._enclosure = None
        self.lower = np.full(3, np.inf)
        self.upper = np.full(3, -np.inf)
        self._cells = {}
        self._indexed_objects = 0
        self._cell_spheres = None
        self.clearance_query_enabled = True
        self.clearance_stats = Counter()

    def add(self, start, end, radius, name):
        start, end = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
        if start.shape != (3,) or end.shape != (3,) or not np.isfinite([start, end]).all():
            raise ValueError("Material endpoints must be finite 3D points")
        if not np.isfinite(radius) or radius <= 0:
            raise ValueError("Material radius must be finite and positive")
        if name in self.segment_names:
            raise ValueError("Material segment names must be unique")
        delta = end - start
        length = float(np.linalg.norm(delta))
        transform = np.eye(4)
        transform[:3, 3] = (start + end) / 2
        if length < 1e-12:
            geometry = fcl.Sphere(radius)
        else:
            z = delta / length
            x = np.cross([0, 1, 0] if abs(z[1]) < 0.9 else [1, 0, 0], z)
            x /= np.linalg.norm(x)
            transform[:3, :3] = np.column_stack([x, np.cross(z, x), z])
            geometry = fcl.Capsule(radius, length)
        obj = placed(geometry, transform)
        self._append_object(
            obj,
            id(geometry),
            {"name": name, "start": start.tolist(), "end": end.tolist(), "radius_m": radius},
        )

    def _append_object(self, obj, geometry_id, segment):
        """Register an immutable capsule, which may be shared by scene copies."""
        name = segment["name"]
        start, end = np.asarray(segment["start"]), np.asarray(segment["end"])
        radius = segment["radius_m"]
        self.objects.append(obj)  # Keep both geometry and object alive for FCL.
        self.geometry_ids.append(geometry_id)
        self.names[geometry_id] = name
        self.segment_names.add(name)
        self.segments.append({**segment, "start": start.tolist(), "end": end.tolist()})
        self._physical_segments.append(
            {**segment, "start": start.tolist(), "end": end.tolist()}
        )
        geometry_bytes = np.asarray([*start, *end, radius], dtype="<f8").tobytes()
        self._geometry_prefixes.append(
            sha256(self._geometry_prefixes[-1] + geometry_bytes).digest()
        )
        self.manager.registerObject(obj)
        self._dirty = True
        self.lower = np.minimum(self.lower, np.minimum(start, end) - radius)
        self.upper = np.maximum(self.upper, np.maximum(start, end) + radius)
        self._enclosure = None

    def geometry_prefix(self, count=None):
        """Bind an immutable world-space capsule prefix and turntable state.

        FCL objects are immutable after insertion. Repositioned material must
        be represented by a new scene, as for indexed turntable transforms.
        """
        count = len(self.objects) if count is None else count
        if not isinstance(count, int) or not 0 <= count <= len(self.objects):
            raise ValueError("Material prefix must lie within the scene")
        return self._geometry_prefixes[count], self.table_index

    def prefix(self, count):
        """Share the first capsules of an append-only scene, in insertion order."""
        self.geometry_prefix(count)  # Validate before allocating a manager.
        result = MaterialScene()
        result.clearance_query_enabled = self.clearance_query_enabled
        result.table_index = self.table_index
        for index in range(count):
            segment = {**self._physical_segments[index], "name": self.segments[index]["name"]}
            result._append_object(self.objects[index], self.geometry_ids[index], segment)
        return result

    def suffix(self, start):
        """Share unchanged capsules after a previously certified prefix."""
        self.geometry_prefix(start)  # Validate before allocating a manager.
        result = MaterialScene()
        result.clearance_query_enabled = self.clearance_query_enabled
        result.table_index = self.table_index
        for index in range(start, len(self.objects)):
            segment = {**self._physical_segments[index], "name": self.segments[index]["name"]}
            result._append_object(self.objects[index], self.geometry_ids[index], segment)
        return result

    def possibly_near_body(self, obj, clearance):
        """Cull only when the actual body clears the entire material enclosure.

        This tighter follow-up to the sphere test handles long, rotated links
        whose enclosing spheres overlap the sculpture. Every deposited capsule
        is contained in this filled box. Its distance is therefore a lower bound
        on distance to material, including an interval's caller-supplied motion
        allowance. An overlapping or numerically uncertain box never culls.
        """
        if not np.isfinite(clearance) or clearance < 0:
            raise ValueError("Body clearance must be finite and nonnegative")
        if not self.objects:
            return False
        if self._enclosure is None:
            transform = np.eye(4)
            transform[:3, 3] = (self.lower + self.upper) / 2
            self._enclosure = placed(fcl.Box(*(self.upper - self.lower)), transform)
        distance = fcl.distance(obj, self._enclosure, fcl.DistanceRequest(), fcl.DistanceResult())
        if not np.isfinite(distance):
            raise ValueError("Material enclosure returned a non-finite distance")
        return distance <= clearance + 1e-6

    def possibly_near(self, center, radius, clearance):
        """Conservatively cull a body sphere outside the entire material box.

        Every capsule lies inside this box. The body lies inside the caller's
        sphere. A separation greater than clearance therefore needs no FCL
        query; overlapping envelopes still receive the full narrowphase check.
        """
        center = np.asarray(center, dtype=float)
        if (
            center.shape != (3,)
            or not np.isfinite(center).all()
            or not np.isfinite([radius, clearance]).all()
            or radius < 0
            or clearance < 0
        ):
            raise ValueError("Body sphere and clearance must be finite and nonnegative")
        if not self.objects:
            return False
        offset = np.maximum(np.maximum(self.lower - center, center - self.upper), 0)
        return float(np.linalg.norm(offset)) <= radius + clearance + 1e-9

    def nearest(self, obj, *, deadline=float("inf"), bounding_sphere=None):
        """Find the nearest strand, optionally bounding the whole query body.

        ``bounding_sphere`` is a world-space (center, radius) that must contain
        the query geometry. It only accelerates broadphase rejection; retained
        cells still use the original FCL box and strand distance queries.
        """
        if bounding_sphere is not None:
            center, radius = bounding_sphere
            center = np.asarray(center, dtype=float)
            if (
                center.shape != (3,)
                or not np.isfinite(center).all()
                or not np.isfinite(radius)
                or radius < 0
            ):
                raise ValueError("Query bounding sphere must be finite and nonnegative")
            bounding_sphere = center, radius
        if not self.objects:
            return None
        if len(self.objects) >= 64:
            return self._nearest_spatial(obj, deadline, bounding_sphere)
        if self._dirty:
            # registerObject inserts into FCL's tree but does not invalidate
            # its setup flag. update explicitly refits and balances the tree;
            # sequential curve insertion can otherwise leave a deep chain.
            self.manager.update()
            self._dirty = False
        return self._nearest_in_manager(obj, self.manager, deadline)

    def within_clearance(self, obj, clearance, *, deadline=float("inf"), bounding_sphere=None):
        """Return a nearest witness only if the required gap cannot be certified.

        Each spatial cell contains complete capsules. Its sphere and filled-box
        distances bound every enclosed strand. Cells above the caller's required
        margin plus motion allowance need no exact capsule distance. No verdict
        survives insertion: expanded cell bounds are rebuilt by the next query.
        Disable ``clearance_query_enabled`` to use exact-nearest as a baseline.
        """
        if not np.isfinite(clearance) or clearance < 0:
            raise ValueError("Material clearance must be finite and nonnegative")
        started = perf_counter()
        self.clearance_stats["queries"] += 1
        try:
            if monotonic() >= deadline:
                raise TimeoutError("Material-distance query exceeded its budget")
            if not self.clearance_query_enabled or len(self.objects) < 64:
                self.clearance_stats["exact_nearest_queries"] += 1
                result = self.nearest(obj, deadline=deadline, bounding_sphere=bounding_sphere)
            else:
                if bounding_sphere is not None:
                    center, radius = bounding_sphere
                    center = np.asarray(center, dtype=float)
                    if (
                        center.shape != (3,)
                        or not np.isfinite(center).all()
                        or not np.isfinite(radius)
                        or radius < 0
                    ):
                        raise ValueError("Query bounding sphere must be finite and nonnegative")
                    bounding_sphere = center, radius
                result = self._nearest_spatial(
                    obj, deadline, bounding_sphere, clearance_limit=clearance
                )
            return result if result is not None and result["distance_m"] <= clearance else None
        finally:
            self.clearance_stats["query_seconds"] += perf_counter() - started

    def _nearest_in_manager(self, obj, manager, deadline, *, names=None):
        data = fcl.DistanceData()
        failure = None

        def callback(first, second, state):
            nonlocal failure
            # python-fcl invokes callbacks through a noexcept C++ boundary.
            # Return its stop flag first, then raise on the Python side.
            try:
                if monotonic() >= deadline:
                    raise TimeoutError("Material-distance query exceeded its budget")
                self.clearance_stats["narrowphase_pairs"] += 1
                return fcl.defaultDistanceCallback(first, second, state)
            except BaseException as error:
                failure = error
                return True, state.result.min_distance

        manager.distance(obj, data, callback)
        if failure is not None:
            raise failure
        result = data.result
        names = self.names if names is None else names
        name = names.get(id(result.o1), names.get(id(result.o2)))
        if name is None or not np.isfinite(result.min_distance):
            raise ValueError("Material broadphase returned no identifiable nearest body")
        return {"name": name, "distance_m": float(result.min_distance)}

    def _ensure_spatial_index(self, deadline=float("inf")):
        """Insert each immutable capsule once, retaining its complete bounds."""
        started = perf_counter()
        for index in range(self._indexed_objects, len(self.objects)):
            if monotonic() >= deadline:
                raise TimeoutError("Material indexing exceeded its budget")
            self._cell_spheres = None
            segment = self._physical_segments[index]
            start, end = np.asarray(segment["start"]), np.asarray(segment["end"])
            radius = segment["radius_m"]
            key = tuple(np.floor((start + end) / (2 * 0.015)).astype(np.int64))
            if key not in self._cells:
                manager = fcl.DynamicAABBTreeCollisionManager()
                manager.setup()
                self._cells[key] = {
                    "manager": manager,
                    "lower": np.full(3, np.inf),
                    "upper": np.full(3, -np.inf),
                    "indices": [],
                }
            cell = self._cells[key]
            cell["manager"].registerObject(self.objects[index])
            cell["indices"].append(index)
            cell["lower"] = np.minimum(cell["lower"], np.minimum(start, end) - radius)
            cell["upper"] = np.maximum(cell["upper"], np.maximum(start, end) + radius)
            cell["enclosure"] = None
            cell["dirty"] = True
            self._indexed_objects = index + 1
        self._indexed_objects = len(self.objects)
        self.clearance_stats["spatial_index_seconds"] += perf_counter() - started

    def _nearest_spatial(self, obj, deadline, bounding_sphere=None, *, clearance_limit=None):
        """Query complete-capsule cells using conservative geometric bounds."""
        self._ensure_spatial_index(deadline)
        return self._query_cells(
            obj, list(self._cells.values()), deadline, bounding_sphere, clearance_limit
        )

    def _query_cells(self, obj, cells, deadline, bounding_sphere, clearance_limit):
        if not cells:
            return None

        def cell_distance(cell):
            if monotonic() >= deadline:
                raise TimeoutError("Material-distance query exceeded its budget")
            if cell["enclosure"] is None:
                transform = np.eye(4)
                transform[:3, 3] = (cell["lower"] + cell["upper"]) / 2
                cell["enclosure"] = placed(fcl.Box(*(cell["upper"] - cell["lower"])), transform)
            lower = fcl.distance(
                obj, cell["enclosure"], fcl.DistanceRequest(), fcl.DistanceResult()
            )
            if not np.isfinite(lower):
                raise ValueError("Material cell returned a non-finite distance")
            return lower

        if bounding_sphere is None:
            candidates = [(cell_distance(cell), index, cell) for index, cell in enumerate(cells)]
        else:
            # Cell spheres contain their complete capsule boxes. Sphere gaps
            # are inexpensive lower bounds, even for a rotated query body.
            # Rebuild after deposition so newly expanded cells cannot be culled
            # using stale bounds. No distance result is cached across poses.
            if self._cell_spheres is None:
                self._cell_spheres = (
                    np.asarray([(cell["lower"] + cell["upper"]) / 2 for cell in cells]),
                    np.asarray(
                        [np.linalg.norm(cell["upper"] - cell["lower"]) / 2 for cell in cells]
                    ),
                    np.asarray([cell["lower"][2] for cell in cells]),
                    np.asarray([cell["upper"][2] for cell in cells]),
                )
            centers, radii, bottoms, tops = self._cell_spheres
            center, radius = bounding_sphere
            lower = np.maximum(np.linalg.norm(centers - center, axis=1) - radii - radius, 0)
            # Each slab spans the *entire* capsules in its cell, including end
            # caps and long strands crossing layers. A vertical gap bounds every
            # strand distance even when a cell's enclosing sphere is very wide.
            slab_lower = np.maximum(np.maximum(bottoms - center[2], center[2] - tops) - radius, 0)
            lower = np.maximum(lower, slab_lower)
            candidates = [(bound, index, cells[index]) for index, bound in enumerate(lower)]
        best = None
        for lower, cell_index, cell in sorted(candidates):
            if monotonic() >= deadline:
                raise TimeoutError("Material-distance query exceeded its budget")
            limit = float("inf") if clearance_limit is None else clearance_limit
            if best is not None:
                limit = min(limit, best["distance_m"])
            if lower > limit + 1e-6:
                if clearance_limit is None:
                    break
                rejection = (
                    "sphere_cells_rejected" if bounding_sphere is not None else "box_cells_rejected"
                )
                if bounding_sphere is not None and slab_lower[cell_index] > limit + 1e-6:
                    self.clearance_stats["layer_cells_rejected"] += 1
                self.clearance_stats[rejection] += 1
                continue
            if bounding_sphere is not None:
                lower = cell_distance(cell)
                if lower > limit + 1e-6:
                    self.clearance_stats["box_cells_rejected"] += 1
                    continue
            if cell["dirty"]:
                cell["manager"].update()
                cell["dirty"] = False
            result = self._nearest_in_manager(
                obj, cell["manager"], deadline, names=cell.get("names")
            )
            if best is None or result["distance_m"] < best["distance_m"]:
                best = result
            if best["distance_m"] < 0:
                break
        return best

    def copy(self):
        result = MaterialScene()
        result.clearance_query_enabled = self.clearance_query_enabled
        result.table_index = self.table_index
        for obj, geometry_id, segment in zip(
            self.objects, self.geometry_ids, self._physical_segments, strict=True
        ):
            result._append_object(obj, geometry_id, segment)
        return result

    def outside_ball(self, center, radius):
        """Remove only capsule portions guaranteed to lie inside a contact ball.

        A centerline point can be removed only within radius minus the capsule
        radius. Retained capsule end caps over-cover the boundary conservatively.
        """
        center = np.asarray(center)
        if (
            center.shape != (3,)
            or not np.isfinite(center).all()
            or not np.isfinite(radius)
            or radius <= 0
        ):
            raise ValueError("Contact region must be a finite positive-radius 3D ball")
        clipped = MaterialScene()
        clipped.clearance_query_enabled = self.clearance_query_enabled
        clipped.table_index = self.table_index
        self._clip_into(clipped, range(len(self.objects)), center, radius)
        return clipped

    def contact_view(self, center, radius, *, local=None):
        """Borrow distant cell managers and clip only geometry near a join.

        Geometry construction is independent of proof/threshold-query reuse.
        ``local=True`` also permits exact-nearest verification to borrow distant
        cell managers. The view refreshes after source material insertion.
        """
        if local is None:
            local = self.clearance_query_enabled
        if not local:
            return self.outside_ball(center, radius)
        return ContactMaterialView(self, center, radius)

    def _clip_into(self, clipped, indices, center, radius):
        for source_index in indices:
            obj, geometry_id = self.objects[source_index], self.geometry_ids[source_index]
            segment = self._physical_segments[source_index]
            a, b = np.array(segment["start"]), np.array(segment["end"])
            delta = b - a
            inner = radius - segment["radius_m"]
            intervals = [(0.0, 1.0)]
            if inner > 0:
                squared = float(delta @ delta)
                if squared < 1e-24:
                    intervals = [] if np.linalg.norm(a - center) <= inner else intervals
                else:
                    linear = float((a - center) @ delta)
                    constant = float((a - center) @ (a - center)) - inner**2
                    discriminant = linear**2 - squared * constant
                    if discriminant > 0:
                        low = max(0.0, (-linear - np.sqrt(discriminant)) / squared)
                        high = min(1.0, (-linear + np.sqrt(discriminant)) / squared)
                        if low < high:
                            intervals = [(0.0, low), (high, 1.0)]
            for index, (low, high) in enumerate(intervals):
                if high - low > 1e-12:
                    name = f"{segment['name']}/contact-clip-{index}"
                    if low == 0.0 and high == 1.0:
                        clipped._append_object(obj, geometry_id, {**segment, "name": name})
                    else:
                        clipped.add(a + low * delta, a + high * delta, segment["radius_m"], name)


class _ContactNames:
    """Match baseline names for capsules borrowed without any clipping."""

    def __init__(self, names):
        self.names = names

    def get(self, key, default=None):
        name = self.names.get(key)
        return default if name is None else f"{name}/contact-clip-0"


class ContactMaterialView(MaterialScene):
    """Read-only local clipping over immutable, incrementally indexed material.

    Only the virtual tip uses this view. Real robot and tool bodies still check
    the original scene. Prefix certificates are disabled because its capsule
    population differs from that scene's insertion count.
    """

    supports_prefix_cache = False

    def __init__(self, source, center, radius):
        self.source = source
        self.center = np.asarray(center, dtype=float).copy()
        self.radius = float(radius)
        if (
            self.center.shape != (3,)
            or not np.isfinite(self.center).all()
            or not np.isfinite(self.radius)
            or self.radius <= 0
        ):
            raise ValueError("Contact region must be a finite positive-radius 3D ball")
        self.clearance_query_enabled = source.clearance_query_enabled
        self.clearance_stats = source.clearance_stats
        self.names = _ContactNames(source.names)
        self._source_prefix = None
        self._view_cells = []
        self._local_scenes = []
        self._cell_spheres = None

    @property
    def objects(self):
        # Used only for population/emptiness checks; queries always use cells.
        return self.source.objects

    @property
    def lower(self):
        return self.source.lower

    @property
    def upper(self):
        return self.source.upper

    @property
    def table_index(self):
        return self.source.table_index

    def geometry_prefix(self, count=None):
        return (
            self.source.geometry_prefix(count), self.center.tobytes(), self.radius
        )

    def possibly_near_body(self, obj, clearance):
        return self.source.possibly_near_body(obj, clearance)

    def _refresh(self, deadline):
        prefix = self.source.geometry_prefix()
        if self._source_prefix == prefix:
            return
        started = perf_counter()
        self.source._ensure_spatial_index(deadline)
        cells, local_scenes = [], []
        for cell in self.source._cells.values():
            if monotonic() >= deadline:
                raise TimeoutError("Contact material view exceeded its budget")
            gap = np.maximum(
                np.maximum(cell["lower"] - self.center, self.center - cell["upper"]), 0
            )
            if np.linalg.norm(gap) > self.radius + 1e-9:
                cells.append(cell)
                self.clearance_stats["contact_cells_shared"] += 1
                continue
            clipped = MaterialScene()
            self.source._clip_into(clipped, cell["indices"], self.center, self.radius)
            clipped._ensure_spatial_index(deadline)
            local_scenes.append(clipped)
            for local in clipped._cells.values():
                local["names"] = clipped.names
                cells.append(local)
            self.clearance_stats["contact_capsules_clipped"] += len(cell["indices"])
            self.clearance_stats["contact_cells_rebuilt"] += 1
        self._view_cells, self._local_scenes = cells, local_scenes
        self._cell_spheres = None
        self._source_prefix = prefix
        self.clearance_stats["contact_views_refreshed"] += 1
        self.clearance_stats["contact_view_setup_seconds"] += perf_counter() - started

    def nearest(self, obj, *, deadline=float("inf"), bounding_sphere=None):
        if bounding_sphere is not None:
            center, radius = bounding_sphere
            center = np.asarray(center, dtype=float)
            if (
                center.shape != (3,)
                or not np.isfinite(center).all()
                or not np.isfinite(radius)
                or radius < 0
            ):
                raise ValueError("Query bounding sphere must be finite and nonnegative")
            bounding_sphere = center, radius
        return self._nearest_spatial(obj, deadline, bounding_sphere)

    def _nearest_spatial(self, obj, deadline, bounding_sphere=None, *, clearance_limit=None):
        self._refresh(deadline)
        return self._query_cells(obj, self._view_cells, deadline, bounding_sphere, clearance_limit)
