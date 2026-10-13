"""FCL checks for the robot's URDF collision meshes and static work surfaces.

The mounted pen and growing material must be added before this can serve as a
whole-tool collision model. Configuration checks alone do not cover motions.
"""

from collections import Counter, OrderedDict
from dataclasses import dataclass
from time import monotonic, perf_counter

import fcl
import numpy as np
import trimesh

from ..robot import origin_matrix

# Process-wide continuous-check workload: calls, evaluations and seconds spent
# in self/static versus material queries. Measurement only; never a verdict.
QUERY_STATS = Counter()


@dataclass
class Body:
    name: str
    link: str
    origin: np.ndarray
    geometry: object
    radius: float
    # None uses URDF adjacency. An attached tool lists the links it is allowed
    # to touch explicitly, so forearm contacts are not silently excluded.
    touch_links: tuple[str, ...] | None = None


def collision_geometry(element):
    mesh = element.find("mesh")
    if mesh is not None:
        loaded = trimesh.load_mesh(mesh.get("filename"), process=False)
        if not isinstance(loaded, trimesh.Trimesh):
            raise ValueError("Collision mesh must contain one triangle mesh")
        vertices = loaded.vertices * np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
        # A triangle BVH tests surface intersection and can miss a body wholly
        # contained inside another. Filled per-link convex hulls are conservative
        # for concave source meshes and also detect containment via solid GJK.
        hull = trimesh.convex.convex_hull(vertices)
        faces = np.column_stack([np.full(len(hull.faces), 3), hull.faces]).ravel()
        model = fcl.Convex(np.asarray(hull.vertices, dtype=np.float64), len(hull.faces), faces)
        return model, float(np.max(np.linalg.norm(hull.vertices, axis=1)))
    box = element.find("box")
    if box is not None:
        size = np.fromstring(box.get("size"), sep=" ")
        return fcl.Box(*size), float(np.linalg.norm(size) / 2)
    cylinder = element.find("cylinder")
    if cylinder is not None:
        radius, length = float(cylinder.get("radius")), float(cylinder.get("length"))
        return fcl.Cylinder(radius, length), float(np.hypot(radius, length / 2))
    sphere = element.find("sphere")
    if sphere is not None:
        radius = float(sphere.get("radius"))
        return fcl.Sphere(radius), radius
    raise ValueError("Unsupported URDF collision geometry")


def placed(geometry, matrix):
    return fcl.CollisionObject(geometry, fcl.Transform(matrix[:3, :3], matrix[:3, 3]))


class RobotCollisionModel:
    """Fixed robot/tool geometry; rebuild this model after calibration changes."""

    def __init__(self, robot, extra_bodies=(), *, include_robot=True):
        self.robot = robot
        self.bodies = []
        # Candidate tool-only checks retain robot kinematics and joint limits,
        # but do not construct URDF collision bodies or their pair caches.
        for link in robot.xml.findall("link") if include_robot else ():
            name = link.get("name")
            for index, element in enumerate(link.findall("collision")):
                geometry, radius = collision_geometry(element.find("geometry"))
                self.bodies.append(
                    Body(
                        f"{name}:{index}",
                        name,
                        origin_matrix(element.find("origin")),
                        geometry,
                        radius,
                    )
                )
        self.bodies.extend(extra_bodies)
        self.adjacent = {frozenset((joint["parent"], joint["child"])) for joint in robot.joint_data}
        if not self.bodies:
            raise ValueError("Collision model has no bodies")
        # A fixed world-to-base joint is part of the mounting assembly. Exclude
        # only those links from the table check, never a moving link.
        self.mount_links = {robot.root}
        # Fixed obstacles, configuration, motion radii and clearances last measured.
        self._reference = None
        for joint in robot.joint_data:
            if joint["index"] is not None:
                break
            self.mount_links.add(joint["child"])
        self.motion_radii = {}
        for body in self.bodies:
            radii = np.zeros(len(robot.names))
            ancestors = []
            for joint in robot.joint_data:
                if body.link == robot.root:
                    break
                ancestors.append(joint)
                if joint["child"] == body.link:
                    break
            if body.link not in {robot.root, *(joint["child"] for joint in robot.joint_data)}:
                raise ValueError(f"Collision body outside the serial chain: {body.link}")
            reach = np.linalg.norm(body.origin[:3, 3]) + body.radius
            for joint in reversed(ancestors):
                if joint["index"] is not None:
                    radii[joint["index"]] = reach
                reach += np.linalg.norm(joint["origin"][:3, 3])
            self.motion_radii[body.name] = radii
        self._self_pairs = [[] for _ in self.bodies]
        self._pair_radii = {}
        for index, body in enumerate(self.bodies):
            for other_index in range(index + 1, len(self.bodies)):
                other = self.bodies[other_index]
                if body.link == other.link:
                    continue
                if body.touch_links is not None or other.touch_links is not None:
                    if (body.touch_links is not None and other.link in body.touch_links) or (
                        other.touch_links is not None and body.link in other.touch_links
                    ):
                        continue
                elif frozenset((body.link, other.link)) in self.adjacent:
                    continue
                first, second = self.motion_radii[body.name], self.motion_radii[other.name]
                # Common ancestor joints transform both bodies rigidly and
                # cannot change their separation. Cache only relative joints.
                radii = np.where((first > 0) & (second > 0), 0.0, first + second)
                self._self_pairs[index].append((other_index, radii > 0))
                self._pair_radii[body.name, other.name] = radii
        self._self_cache = OrderedDict()
        self.self_cache_hits = self.self_cache_misses = 0
        self._material_poses = OrderedDict()
        self.material_cache_stats = Counter()

    def _material_pose(self, positions, *, enabled):
        """Reuse exact world placements for this immutable robot/tool model.

        No joint quantization is permitted. Every hit still validates current
        joint limits, and the actual base matrix participates in its key. Other
        calibration/geometry changes require rebuilding the collision model.
        """
        q = np.asarray(positions, dtype=np.float64)
        if q.shape != (len(self.robot.names),) or not np.isfinite(q).all():
            raise ValueError("Expected finite joint positions matching the arm DOF")
        if np.any(q < self.robot.lower) or np.any(q > self.robot.upper):
            raise ValueError("Joint positions exceed URDF limits")
        base = np.asarray(getattr(self.robot, "world_from_base", np.eye(4)), dtype=np.float64)
        key = q.tobytes(), base.tobytes()
        pose = self._material_poses.get(key) if enabled else None
        if pose is not None:
            self._material_poses.move_to_end(key)
            self.material_cache_stats["pose_hits"] += 1
            return pose
        self.material_cache_stats["pose_misses"] += 1
        frames = self.robot.frames(q)
        pose = {
            "transforms": [frames[body.link] @ body.origin for body in self.bodies],
            "objects": {},
            "proofs": {},
        }
        if enabled:
            self._material_poses[key] = pose
            if len(self._material_poses) > 2048:
                self._material_poses.popitem(last=False)
        return pose

    def pairs(self, positions, obstacles=None):
        frames = self.robot.frames(positions)
        objects = [
            (body, placed(body.geometry, frames[body.link] @ body.origin)) for body in self.bodies
        ]
        for index, (body, obj) in enumerate(objects):
            for other_index, _ in self._self_pairs[index]:
                other, other_obj = objects[other_index]
                yield body.name, other.name, obj, other_obj
            for name, obstacle in (obstacles or {}).items():
                if name == "table" and body.link in self.mount_links:
                    continue
                yield body.name, name, obj, obstacle

    def distances(self, positions, obstacles=None, *, deadline=float("inf"), only=None):
        """Yield names, signed clearance and overlap; cache only self pairs.

        ``only`` restricts the query to the named pairs.

        Keys retain exact relative joint values, without angle quantization.
        The bounded, in-memory cache belongs to this fixed robot/tool model.
        Obstacle poses and deposited material never enter its certificates.
        """
        q = np.asarray(positions, dtype=np.float64)
        frames = self.robot.frames(q)  # Validate every configuration, including hits.
        objects = {}

        def object_at(index):
            if index not in objects:
                body = self.bodies[index]
                objects[index] = placed(body.geometry, frames[body.link] @ body.origin)
            return objects[index]

        def measure(first, second):
            result = fcl.CollisionResult()
            contact = bool(
                fcl.collide(first, second, fcl.CollisionRequest(enable_contact=True), result)
            )
            distance = (
                -max((row.penetration_depth for row in result.contacts), default=1e-9)
                if contact
                else fcl.distance(first, second, fcl.DistanceRequest(), fcl.DistanceResult())
            )
            if not np.isfinite(distance):
                raise ValueError("Invalid collision distance")
            if distance < 0 and not contact:
                # FCL's overlap and distance queries can disagree at a GJK
                # tolerance boundary. A negative distance must reject the
                # candidate, never certify clearance or abort the whole search.
                contact = True
                distance = -1e-9
            return float(distance), contact

        for index, body in enumerate(self.bodies):
            for other_index, relative in self._self_pairs[index]:
                if only is not None and (body.name, self.bodies[other_index].name) not in only:
                    continue
                if monotonic() >= deadline:
                    raise TimeoutError("Self-collision check exceeded its budget")
                key = (index, other_index, q[relative].tobytes())
                cached = self._self_cache.get(key)
                if cached is None:
                    self.self_cache_misses += 1
                    cached = measure(object_at(index), object_at(other_index))
                    self._self_cache[key] = cached
                    if len(self._self_cache) > 8192:
                        self._self_cache.popitem(last=False)
                else:
                    self.self_cache_hits += 1
                    self._self_cache.move_to_end(key)
                yield body.name, self.bodies[other_index].name, *cached
            for name, obstacle in (obstacles or {}).items():
                if name == "table" and body.link in self.mount_links:
                    continue
                if only is not None and (body.name, name) not in only:
                    continue
                if monotonic() >= deadline:
                    raise TimeoutError("Obstacle collision check exceeded its budget")
                yield body.name, name, *measure(object_at(index), obstacle)

    def material_distances(
        self, positions, material, clearance_bounds=None, *, deadline=float("inf"), only=None
    ):
        """Yield each body's nearest material within its bound; ``only`` names bodies."""
        enabled = material.clearance_query_enabled
        prefix_cache = enabled and getattr(material, "supports_prefix_cache", True)
        pose = self._material_pose(positions, enabled=enabled)
        suffixes = {}
        for index, body in enumerate(self.bodies):
            if only is not None and body.name not in only:
                continue
            if monotonic() >= deadline:
                raise TimeoutError("Material-distance query exceeded its budget")
            transform = pose["transforms"][index]
            threshold = None if clearance_bounds is None else clearance_bounds[body.name]
            if threshold is not None and (not np.isfinite(threshold) or threshold < 0):
                raise ValueError("Material clearance must be finite and nonnegative")
            query_scene = material
            proof = pose["proofs"].get(index) if prefix_cache and threshold is not None else None
            if proof is not None:
                count, prefix, gap = proof
                if (
                    threshold <= gap
                    and count <= len(material.objects)
                    and material.geometry_prefix(count) == prefix
                ):
                    self.material_cache_stats["prefix_hits"] += 1
                    if count == len(material.objects):
                        continue
                    if count not in suffixes:
                        suffixes[count] = material.suffix(count)
                        suffixes[count].clearance_stats = material.clearance_stats
                    query_scene = suffixes[count]
                    self.material_cache_stats["suffix_capsules"] += len(query_scene.objects)
                else:
                    self.material_cache_stats["prefix_misses"] += 1
            clear = threshold is not None and not query_scene.possibly_near(
                transform[:3, 3], body.radius, threshold
            )
            if not clear:
                if index not in pose["objects"]:
                    pose["objects"][index] = placed(body.geometry, transform)
                    self.material_cache_stats["objects_created"] += 1
                obj = pose["objects"][index]
                clear = threshold is not None and not query_scene.possibly_near_body(obj, threshold)
                if not clear:
                    query = {
                        "deadline": deadline,
                        "bounding_sphere": (transform[:3, 3], body.radius),
                    }
                    nearest = (
                        query_scene.nearest(obj, **query)
                        if threshold is None
                        else query_scene.within_clearance(obj, threshold, **query)
                    )
                    if nearest:
                        yield body.name, nearest["name"], nearest["distance_m"]
                        continue
                    clear = threshold is not None
            if prefix_cache and clear:
                # The previous proof covers the entire prefix and this query
                # covers every appended capsule, with the current swept bound.
                pose["proofs"][index] = (
                    len(material.objects),
                    material.geometry_prefix(),
                    threshold,
                )

    @staticmethod
    def _obstacle_key(obstacles):
        """Identity and pose of the fixed obstacles a clearance was measured against."""
        return tuple(
            (
                name,
                id(obstacle),
                obstacle.getTranslation().tobytes(),
                obstacle.getRotation().tobytes(),
            )
            for name, obstacle in sorted((obstacles or {}).items())
        )

    def check(
        self, positions, obstacles=None, margin_m=0.0, material=None, *, deadline=float("inf")
    ):
        if margin_m < 0:
            raise ValueError("Collision margin must be nonnegative")
        overlaps = []

        for name1, name2, distance, contact in self.distances(
            positions, obstacles, deadline=deadline
        ):
            if contact:
                overlaps.append({"first": name1, "second": name2, "kind": "overlap"})
            elif distance < margin_m:
                overlaps.append(
                    {
                        "first": name1,
                        "second": name2,
                        "kind": "clearance",
                        "distance_m": distance,
                    }
                )
        if material is not None:
            bounds = dict.fromkeys((body.name for body in self.bodies), margin_m)
            for name1, name2, distance in self.material_distances(
                positions, material, bounds, deadline=deadline
            ):
                if distance <= margin_m:
                    overlaps.append(
                        {
                            "first": name1,
                            "second": name2,
                            "kind": "overlap" if distance <= 0 else "clearance",
                            "distance_m": distance,
                        }
                    )
        return overlaps

    def check_edge(
        self,
        q0,
        q1,
        obstacles=None,
        margin_m=0.001,
        maximum_depth=12,
        material=None,
        deadline=float("inf"),
    ):
        """Bound all modeled point motion over a linearly interpolated joint edge.

        At each interval midpoint, pair clearance must exceed the sum of both
        bodies' maximum possible displacement plus the requested margin. A
        joint's displacement bound is |delta angle| times an upper bound on
        reach to every body point. Triangle inequality combines joint bounds.
        Intervals without that certificate subdivide; a depth limit is reported
        as unresolved, never passed. FCL distances have a 1 um numerical reserve.
        This certifies only the supplied rigid geometry and interpolation.
        """
        q0, q1 = np.asarray(q0), np.asarray(q1)
        return self.check_bounded_path(
            lambda t: q0 * (1 - t) + q1 * t,
            lambda start, end: np.abs(q1 - q0) * (end - start) / 2,
            obstacles,
            margin_m,
            maximum_depth,
            material,
            deadline,
        )

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
        """Check a continuous path on [0, 1] with a supplied joint envelope.

        ``deviation(a, b)`` must bound each joint's absolute displacement from
        ``position((a+b)/2)`` throughout [a,b]. Unlike an endpoint chord this
        bound must include excursions and reversals inside curved paths.
        """
        if not np.isfinite(margin_m) or margin_m < 0 or maximum_depth < 0:
            raise ValueError("Invalid edge clearance settings")
        # Also validate endpoints and their joint limits before interpolation.
        self.robot.frames(position(0))
        self.robot.frames(position(1))
        evaluations, greatest_depth = 0, 0
        fixed = self._obstacle_key(obstacles)

        def visit(start, end, depth, pairs=None, bodies=None):
            """Certify [start, end]. ``pairs`` and ``bodies`` name what the parent
            interval left unresolved; everything else it already certified for
            the whole of its span, which contains this one.
            """
            nonlocal evaluations, greatest_depth
            if monotonic() >= deadline:
                return {"status": "TIME_BUDGET"}
            evaluations += 1
            greatest_depth = max(greatest_depth, depth)
            alpha = (start + end) / 2
            q = np.asarray(position(alpha), float)
            half_delta = np.asarray(deviation(start, end), float)
            if (
                half_delta.shape != (len(self.robot.names),)
                or not np.isfinite(half_delta).all()
                or np.any(half_delta < 0)
            ):
                raise ValueError("Path deviation must be finite and nonnegative for each joint")
            uncertain = []
            QUERY_STATS["path_evaluations"] += 1
            started = perf_counter()
            # The arm moves a fraction of a degree per edge. Clearances measured
            # at one configuration certify every configuration whose bounded
            # motion from it stays inside them, so most edges need no new
            # self or fixed-obstacle query at all.
            reference = self._reference if pairs is None else None
            certified = (
                reference is not None
                and reference[0] == fixed
                and bool(
                    np.all(
                        reference[2] @ (np.abs(q - reference[1]) + half_delta) + margin_m + 1e-6
                        < reference[3]
                    )
                )
            )
            measured = []
            try:
                if certified:
                    QUERY_STATS["reference_certified"] += 1
                for name1, name2, distance, contact in (
                    ()
                    # Nothing left unresolved there: no query, not even kinematics.
                    if certified or pairs == set()
                    else self.distances(q, obstacles, deadline=deadline, only=pairs)
                ):
                    if contact:
                        return {
                            "status": "COLLISION",
                            "alpha": alpha,
                            "positions": q.tolist(),
                            "pairs": [{"first": name1, "second": name2}],
                        }
                    radius = (
                        self._pair_radii[name1, name2]
                        if name2 not in (obstacles or {}) and (name1, name2) in self._pair_radii
                        else self.motion_radii[name1]
                    )
                    measured.append((radius, distance))
                    bound = float(radius @ half_delta)
                    if distance <= bound + margin_m + 1e-6:
                        uncertain.append(
                            {
                                "first": name1,
                                "second": name2,
                                "distance_m": float(distance),
                                "motion_bound_m": bound,
                            }
                        )
            finally:
                QUERY_STATS["self_static_seconds"] += perf_counter() - started
            if pairs is None and not certified and measured:
                self._reference = (
                    fixed,
                    q.copy(),
                    np.array([radius for radius, _ in measured]),
                    np.array([distance for _, distance in measured]),
                )
            unresolved_pairs = {(row["first"], row["second"]) for row in uncertain}
            if material is not None and bodies != set():
                bounds = {
                    body.name: float(self.motion_radii[body.name] @ half_delta) + margin_m + 1e-6
                    for body in self.bodies
                }
                started = perf_counter()
                try:
                    for name1, name2, distance in self.material_distances(
                        q, material, bounds, deadline=deadline, only=bodies
                    ):
                        if distance <= 0:
                            return {
                                "status": "COLLISION",
                                "alpha": alpha,
                                "positions": q.tolist(),
                                "pairs": [{"first": name1, "second": name2}],
                            }
                        bound = float(self.motion_radii[name1] @ half_delta)
                        if distance <= bound + margin_m + 1e-6:
                            uncertain.append(
                                {
                                    "first": name1,
                                    "second": name2,
                                    "distance_m": distance,
                                    "motion_bound_m": bound,
                                }
                            )
                finally:
                    QUERY_STATS["material_seconds"] += perf_counter() - started
            if not uncertain:
                return {"status": "CERTIFIED_MODELED_EDGE"}
            if depth >= maximum_depth:
                return {
                    "status": "CLEARANCE_UNRESOLVED",
                    "alpha": alpha,
                    "positions": q.tolist(),
                    "pairs": uncertain,
                }
            unresolved_bodies = {
                row["first"]
                for row in uncertain
                if (row["first"], row["second"]) not in unresolved_pairs
            }
            first = visit(start, alpha, depth + 1, unresolved_pairs, unresolved_bodies)
            return (
                visit(alpha, end, depth + 1, unresolved_pairs, unresolved_bodies)
                if first["status"] == "CERTIFIED_MODELED_EDGE"
                else first
            )

        QUERY_STATS["path_calls"] += 1
        try:
            result = visit(0.0, 1.0, 0)
        except TimeoutError:
            result = {"status": "TIME_BUDGET"}
        return {**result, "evaluations": evaluations, "maximum_depth": greatest_depth}


def work_surfaces(center_xy, height, width, depth, table_z=0.0, table_size=(4.0, 4.0, 0.1)):
    if height < 0 or min(width, depth, *table_size) <= 0:
        raise ValueError("Invalid work-surface dimensions")
    transform = np.eye(4)
    # At zero height the top stays on the table, rather than floating 3 mm above it.
    thickness = max(height, 0.006)
    transform[:3, 3] = [*center_xy, table_z + height - thickness / 2]
    platform = placed(fcl.Box(width, depth, thickness), transform)
    transform = np.eye(4)
    transform[2, 3] = table_z - table_size[2] / 2
    # FCL 0.7's mesh/halfspace distance query can crash. A finite, explicitly
    # sized table uses supported mesh/box distance queries instead.
    table = placed(fcl.Box(*table_size), transform)
    return {"table": table, "platform": platform}
