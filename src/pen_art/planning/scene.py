"""Incremental capsule scene for conservative deposited-strand geometry."""

import fcl
import numpy as np

from .collision import placed


class MaterialScene:
    def __init__(self):
        self.manager = fcl.DynamicAABBTreeCollisionManager()
        self.manager.setup()
        self.objects = []
        self.names = {}
        self.segment_names = set()
        self.segments = []
        self._dirty = False
        self._enclosure = None
        self.lower = np.full(3, np.inf)
        self.upper = np.full(3, -np.inf)

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
        self.objects.append(obj)  # Keep both geometry and object alive for FCL.
        self.names[id(geometry)] = name
        self.segment_names.add(name)
        self.segments.append(
            {"name": name, "start": start.tolist(), "end": end.tolist(), "radius_m": radius}
        )
        self.manager.registerObject(obj)
        self._dirty = True
        self.lower = np.minimum(self.lower, np.minimum(start, end) - radius)
        self.upper = np.maximum(self.upper, np.maximum(start, end) + radius)
        self._enclosure = None

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

    def nearest(self, obj):
        if not self.objects:
            return None
        if self._dirty:
            # registerObject inserts into FCL's tree but does not invalidate
            # its setup flag. update explicitly refits and balances the tree;
            # sequential curve insertion can otherwise leave a deep chain.
            self.manager.update()
            self._dirty = False
        data = fcl.DistanceData()
        self.manager.distance(obj, data, fcl.defaultDistanceCallback)
        result = data.result
        name = self.names.get(id(result.o1), self.names.get(id(result.o2)))
        if name is None or not np.isfinite(result.min_distance):
            raise ValueError("Material broadphase returned no identifiable nearest body")
        return {"name": name, "distance_m": float(result.min_distance)}

    def copy(self):
        result = MaterialScene()
        for segment in self.segments:
            result.add(segment["start"], segment["end"], segment["radius_m"], segment["name"])
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
        for segment in self.segments:
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
                    clipped.add(
                        a + low * delta,
                        a + high * delta,
                        segment["radius_m"],
                        f"{segment['name']}/contact-clip-{index}",
                    )
        return clipped
