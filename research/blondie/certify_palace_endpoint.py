"""Cover all tool orientations using inscribed-sphere clearance upper bounds.

For an Euler-angle box with half-width sum a, each tool sphere center moves
at most 2*d*sin(min(a, pi)/2), where d is its distance from the TCP. Distance
to a fixed obstacle set is 1-Lipschitz. A single sphere whose clearance upper
bound stays below the required margin therefore rejects the whole box.
"""

import argparse
import hashlib
from fractions import Fraction
from pathlib import Path
from time import monotonic

import fcl
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from pen_art.curves import transform_points
from pen_art.io import write_json
from pen_art.planning.section_build import SectionBuild

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--source-tolerance", action="store_true")
args = parser.parse_args()
stem = "source_tolerance" if args.source_tolerance else "orientation"

b = SectionBuild(
    Path("runs/palace_indexed_corridor_dense_v1"),
    Path("research/planning_cell/cell_turntable.yaml"),
    240,
)
row = b.actions["section-01212"]
scene = b.scene({k for k, v in b.actions.items() if v["stage"] == "framework"})
inverse = np.linalg.inv(b.arm.flange_to_tcp)
local = []
radii = []
for body in b.arm.tool.bodies:
    g = body.geometry
    if isinstance(g, fcl.Cylinder):
        radius = g.radius - 1e-8
        positions = [
            np.array([0, 0, z])
            for z in np.linspace(-g.lz / 2 + radius, g.lz / 2 - radius, max(1, int(g.lz / radius)))
        ]
    elif isinstance(g, fcl.Box):
        radius = min(g.side) / 2 - 1e-8
        axis = int(np.argmax(g.side))
        positions = []
        for offset in np.linspace(
            -g.side[axis] / 2 + radius,
            g.side[axis] / 2 - radius,
            max(1, int(g.side[axis] / radius)),
        ):
            p = np.zeros(3)
            p[axis] = offset
            positions.append(p)
    else:
        continue
    T = inverse @ body.origin
    local.extend((T[:3, :3] @ p + T[:3, 3]) for p in positions)
    radii.extend([radius] * len(positions))
local = np.asarray(local)
radii = np.asarray(radii)
a = np.asarray([s["start"] for s in scene.segments])
z = np.asarray([s["end"] for s in scene.segments])
cloud = np.concatenate([a, z, (a + z) / 2])
tree = cKDTree(cloud)
material_radius = min(s["radius_m"] for s in scene.segments)
point = transform_points(np.asarray([row["segments"][0][0]]), b.transform)[0]
turntable = b.planner.profile["turntable"]
cylinders = [
    (b.planner.obstacles[name].getTranslation(), turntable[radius], turntable[height] / 2)
    for name, radius, height in [
        ("platform", "plate_radius_m", "plate_thickness_m"),
        ("turntable_housing", "housing_radius_m", "housing_height_m"),
    ]
]
assert all(
    np.allclose(b.planner.obstacles[name].getRotation(), np.eye(3))
    for name in ["platform", "turntable_housing"]
)

# The stronger variant also permits each framework point to move 0.35 mm
# (tracking plus flattening), and removes its 0.35 mm planning inflation.
translation = 0.001 if args.source_tolerance else 0.0003
reserve = 1e-6
limit = 0.001
radii_offset = np.linalg.norm(local, axis=1)
root_low = np.array([-np.pi, -np.pi / 2, -np.pi])
root_high = -root_low
pending = [(root_low, root_high, "")]
leaves = []
unknown = []
processed = 0
best_lower = -float("inf")
start = monotonic()
last = start
while pending and monotonic() - start < 300 and processed < 2000000:
    batch = [pending.pop() for _ in range(min(512, len(pending)))]
    lo = np.array([v[0] for v in batch])
    hi = np.array([v[1] for v in batch])
    middle = (lo + hi) / 2
    matrices = Rotation.from_euler("ZYX", middle).as_matrix()
    world = np.einsum("bij,nj->bni", matrices, local) + point
    nearest = (
        tree.query(world.reshape(-1, 3), workers=2)[0].reshape(world.shape[:2])
        - radii
        - material_radius
    )
    gaps = nearest
    for center, radius, half_height in cylinders:
        delta = world - center
        radial = np.maximum(np.linalg.norm(delta[:, :, :2], axis=2) - radius, 0)
        vertical = np.maximum(np.abs(delta[:, :, 2]) - half_height, 0)
        gaps = np.minimum(gaps, np.hypot(radial, vertical) - radii)
    angular = np.minimum(np.sum((hi - lo) / 2, axis=1), np.pi)
    motion = 2 * np.sin(angular[:, None] / 2) * radii_offset[None, :]
    upper = gaps + motion + translation
    witness = np.argmin(upper, axis=1)
    bounds = upper[np.arange(len(batch)), witness]
    best_lower = max(best_lower, float(gaps.min(axis=1).max()))
    processed += len(batch)
    for j, (low, high, key) in enumerate(batch):
        if bounds[j] < limit - reserve:
            leaves.append(
                {"path": key, "sphere": int(witness[j]), "clearance_upper_m": float(bounds[j])}
            )
            continue
        if len(key) >= 45:
            unknown.append(
                {"path": key, "center": middle[j].tolist(), "minimum_gap_m": float(gaps[j].min())}
            )
            continue
        axis = int(np.argmax(high - low))
        mid = (low[axis] + high[axis]) / 2
        left_high = high.copy()
        left_high[axis] = mid
        right_low = low.copy()
        right_low[axis] = mid
        pending.append((low, left_high, key + "0"))
        pending.append((right_low, high, key + "1"))
    if monotonic() - last >= 10:
        print(
            "nodes",
            processed,
            "certified leaves",
            len(leaves),
            "pending",
            len(pending),
            "unknown",
            len(unknown),
            "seconds",
            round(monotonic() - start, 1),
            flush=True,
        )
        last = monotonic()
    if unknown:
        break
result = {
    "status": "NO_POSE_FOR_CONSERVATIVE_FRAMEWORK"
    if not pending and not unknown
    else "INCONCLUSIVE_BOUNDED_SEARCH",
    "section": row["id"],
    "source_point_world_m": point.tolist(),
    "translation_allowance_m": translation,
    "allowance_components_m": {
        "tool_tracking": 0.0003,
        "framework_center_motion": 0.00035 if args.source_tolerance else 0.0,
        "framework_radius_reduction": 0.00035 if args.source_tolerance else 0.0,
    },
    "required_clearance_m": limit,
    "numerical_reserve_m": reserve,
    "root_low": root_low.tolist(),
    "root_high": root_high.tolist(),
    "nodes": processed,
    "leaves": leaves,
    "pending": len(pending),
    "unknown": unknown,
    "seconds": monotonic() - start,
    "best_sampled_minimum_gap_m": best_lower,
    "sphere_centers_tcp_m": local.tolist(),
    "sphere_radii_m": radii.tolist(),
    "material_radius_m": material_radius,
    "material_cloud_sha256": hashlib.sha256(cloud.tobytes()).hexdigest(),
    "cylinders": [
        {"center": np.asarray(c).tolist(), "radius_m": float(r), "half_height_m": float(h)}
        for c, r, h in cylinders
    ],
    "scope": (
        "Only the declared conservative completed-framework model and provisional rigid tool. "
        "Inscribed spheres provide sufficient collision witnesses; "
        "incomplete coverage is inconclusive."
    ),
}
if not pending and not unknown:
    assert sum((Fraction(1, 2 ** len(v["path"])) for v in leaves), Fraction()) == 1
write_json(Path(f"research/blondie/palace_endpoint_{stem}_certificate.json"), result)
print(
    {
        k: v
        for k, v in result.items()
        if k not in ["leaves", "sphere_centers_tcp_m", "sphere_radii_m", "cylinders"]
    },
    flush=True,
)
