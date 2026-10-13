"""The pen as a row of spheres, checked against printed wire at chosen poses.

Each tool body is cut into slabs along its own z axis. A sphere centred on that
axis, with radius ``sqrt(R^2 + (d/2)^2)`` for a slab of thickness ``d`` whose
points lie within ``R`` of the axis, contains the whole slab, so the union of
spheres contains the tool. Clearance to a wire capsule is then the closed-form
distance from a sphere centre to the wire's centre line.

This is a check at the poses it is given. It says nothing about the motion
between two poses; the caller chooses how closely to space them.
"""

from itertools import combinations

import numpy as np

# Recorded with a plan's settings so a proof names the model that produced it.
PEN_SPHERES_REVISION = "spheres-v1"
# Slab thickness as a share of the slab's radius: the spheres then stand proud
# of the true surface by about 1/70 of that radius.
SLAB_RADIUS_SHARE = float(np.sqrt(8 / 70))
MINIMUM_SLAB_M = 0.0002
ROUNDING_M = 1e-9


def _radial_reach(vertices, pairs, low, high):
    """Largest distance from the z axis of a convex body between two heights."""
    inside = vertices[(vertices[:, 2] >= low - 1e-12) & (vertices[:, 2] <= high + 1e-12)]
    reach = float(np.max(np.linalg.norm(inside[:, :2], axis=1))) if len(inside) else 0.0
    first, second = vertices[pairs[:, 0]], vertices[pairs[:, 1]]
    rise = second[:, 2] - first[:, 2]
    for height in (low, high):
        with np.errstate(divide="ignore", invalid="ignore"):
            along = (height - first[:, 2]) / rise
        crossing = np.isfinite(along) & (along >= 0) & (along <= 1)
        if crossing.any():
            points = first[crossing] + along[crossing, None] * (second - first)[crossing]
            reach = max(reach, float(np.max(np.linalg.norm(points[:, :2], axis=1))))
    return reach


def _body_spheres(visual):
    """Sphere centres and radii, in the body's own frame, that contain it."""
    kind = visual["type"]
    if kind == "sphere":
        return np.zeros((1, 3)), np.array([visual["radius_m"]])
    if kind == "cylinder":
        radius, half = visual["radius_m"], visual["length_m"] / 2

        def reach(low, high):
            return radius

        low_end, high_end = -half, half
    else:
        if kind == "box":
            half = np.asarray(visual["size_m"], float) / 2
            vertices = np.array(
                [[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], float
            ) * half
        else:
            vertices = np.asarray(visual["vertices"], float)
        pairs = np.array(list(combinations(range(len(vertices)), 2)))

        def reach(low, high):
            return _radial_reach(vertices, pairs, low, high)

        low_end, high_end = float(vertices[:, 2].min()), float(vertices[:, 2].max())
    centres, radii, low = [], [], low_end
    while low < high_end - 1e-12:
        # Size the slab by the radius where it starts and where it would end.
        thickness = max(MINIMUM_SLAB_M, SLAB_RADIUS_SHARE * reach(low, low))
        thickness = max(
            MINIMUM_SLAB_M,
            min(thickness, SLAB_RADIUS_SHARE * reach(low, min(high_end, low + thickness))),
        )
        high = min(high_end, low + thickness)
        if high_end - high < MINIMUM_SLAB_M:
            high = high_end
        centres.append([0.0, 0.0, (low + high) / 2])
        radii.append(float(np.hypot(reach(low, high), (high - low) / 2)))
        low = high
    return np.asarray(centres), np.asarray(radii)


class PenSpheres:
    """Spheres containing every tool body, in the flange frame."""

    def __init__(self, tool_metadata):
        centres, radii, names, groups = [], [], [], []
        for visual in tool_metadata.get("visual_bodies", []):
            local, size = _body_spheres(visual)
            origin = np.asarray(visual["origin"], float)
            first = len(radii)
            centres.extend(local @ origin[:3, :3].T + origin[:3, 3])
            radii.extend(size)
            names.extend([f"tool/{visual['name']}"] * len(size))
            groups.append((first, len(radii)))
        if not radii:
            raise ValueError("The sphere pen check needs configured tool bodies")
        self.centres, self.radii = np.asarray(centres), np.asarray(radii)
        self.names, self.groups = names, groups

    def world(self, flange):
        flange = np.asarray(flange, float)
        return self.centres @ flange[:3, :3].T + flange[:3, 3]

    def violation(self, flange, wires, margin):
        """The closest wire nearer than ``margin`` to the pen at this pose, or None."""
        starts, ends, wire_radii = wires
        if not len(wire_radii):
            return None
        points = self.world(flange)
        middles, spans = (starts + ends) / 2, ends - starts
        halves = np.linalg.norm(spans, axis=1) / 2
        squared = np.maximum(np.einsum("ij,ij->i", spans, spans), 1e-300)
        worst = None
        for first, last in self.groups:
            # Wires whose middle is far from this body's axis cannot reach it.
            reach = self.radii[first:last].max() + halves + wire_radii + margin + ROUNDING_M
            near = np.flatnonzero(
                _point_segment(middles, points[first], points[last - 1]) <= reach
            )
            if not len(near):
                continue
            offset = points[first:last, None, :] - starts[None, near, :]
            along = np.clip(np.einsum("skj,kj->sk", offset, spans[near]) / squared[near], 0, 1)
            gap = (
                np.linalg.norm(offset - along[:, :, None] * spans[None, near, :], axis=2)
                - self.radii[first:last, None]
                - wire_radii[None, near]
            )
            sphere, wire = np.unravel_index(np.argmin(gap), gap.shape)
            if gap[sphere, wire] < margin + ROUNDING_M and (
                worst is None or gap[sphere, wire] < worst["distance_m"]
            ):
                worst = {
                    "status": "PEN_WIRE_COLLISION",
                    "body": self.names[first + sphere],
                    "wire": int(near[wire]),
                    "distance_m": float(gap[sphere, wire]),
                    "margin_m": margin,
                }
        return worst


def _point_segment(points, start, end):
    """Distance from each point to the segment between ``start`` and ``end``."""
    span = end - start
    along = np.clip((points - start) @ span / max(float(span @ span), 1e-300), 0, 1)
    return np.linalg.norm(points - (start + along[:, None] * span), axis=1)


def wire_arrays(scene):
    """Centre lines and radii of a scene's wire, extended as the scene grows."""
    count, starts, ends, radii = getattr(scene, "_pen_wires", (0, None, None, None))
    rows = scene._physical_segments
    if count != len(rows):
        new = np.asarray(
            [[*row["start"], *row["end"], row["radius_m"]] for row in rows[count:]], float
        )
        starts = new[:, :3] if starts is None else np.concatenate([starts, new[:, :3]])
        ends = new[:, 3:6] if ends is None else np.concatenate([ends, new[:, 3:6]])
        radii = new[:, 6] if radii is None else np.concatenate([radii, new[:, 6]])
        scene._pen_wires = (len(rows), starts, ends, radii)
    if starts is None:
        return np.zeros((0, 3)), np.zeros((0, 3)), np.zeros(0)
    return starts, ends, radii
