"""Rebuild and check every leaf of a complete endpoint orientation certificate."""

import argparse
import hashlib
from fractions import Fraction
from pathlib import Path

import fcl
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from pen_art.io import read_json, sha256, write_json
from pen_art.planning.section_build import SectionBuild

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--source-tolerance", action="store_true")
args = parser.parse_args()
stem = "source_tolerance" if args.source_tolerance else "orientation"
proof = Path(f"research/blondie/palace_endpoint_{stem}_certificate.json")
data = read_json(proof)
assert (
    data["status"] == "NO_POSE_FOR_CONSERVATIVE_FRAMEWORK"
    and not data["pending"]
    and not data["unknown"]
)
b = SectionBuild(
    Path("runs/palace_indexed_corridor_dense_v1"),
    Path("research/planning_cell/cell_turntable.yaml"),
    300,
)
row = b.actions[data["section"]]
assert row["stage"] == "artistic" and row["members"]
np.testing.assert_allclose(
    data["source_point_world_m"],
    np.asarray(row["segments"][0][0]) + b.transform[:3, 3],
    atol=1e-12,
    rtol=0,
)
assert (
    data["required_clearance_m"] == b.planner.margin and data["translation_allowance_m"] >= 0.0003
)
if args.source_tolerance:
    assert data["translation_allowance_m"] >= 0.001
np.testing.assert_equal(data["root_low"], [-np.pi, -np.pi / 2, -np.pi])
np.testing.assert_equal(data["root_high"], [np.pi, np.pi / 2, np.pi])
local = np.asarray(data["sphere_centers_tcp_m"])
radii = np.asarray(data["sphere_radii_m"])
offset = np.linalg.norm(local, axis=1)
assert np.all(radii > 0)
for c, r in zip(local, radii, strict=True):
    found = False
    for body in b.arm.tool.bodies:
        assert any(body is candidate for candidate in b.arm.collision.bodies)
        p = (np.linalg.inv(body.origin) @ b.arm.flange_to_tcp @ np.r_[c, 1])[:3]
        g = body.geometry
        if isinstance(g, fcl.Cylinder):
            inside = (
                np.linalg.norm(p[:2]) + r <= g.radius + 1e-10 and abs(p[2]) + r <= g.lz / 2 + 1e-10
            )
        elif isinstance(g, fcl.Box):
            inside = bool(np.all(abs(p) + r <= g.side / 2 + 1e-10))
        else:
            inside = False
        found |= inside
    assert found, "Sphere not contained in a checked tool body"
scene = b.scene({k for k, v in b.actions.items() if v["stage"] == "framework"})
a = np.asarray([s["start"] for s in scene.segments])
z = np.asarray([s["end"] for s in scene.segments])
cloud = np.concatenate([a, z, (a + z) / 2])
tree = cKDTree(cloud)
assert hashlib.sha256(cloud.tobytes()).hexdigest() == data["material_cloud_sha256"]
assert data["material_radius_m"] <= min(s["radius_m"] for s in scene.segments)
if args.source_tolerance:
    nominal_radius = min(
        v["diameter_m"] / 2 for v in b.actions.values() if v["stage"] == "framework"
    )
    assert data["material_radius_m"] - nominal_radius <= 0.00035 + 1e-12
turntable = b.planner.profile["turntable"]
for entry, name, rkey, hkey in zip(
    data["cylinders"],
    ["platform", "turntable_housing"],
    ["plate_radius_m", "housing_radius_m"],
    ["plate_thickness_m", "housing_height_m"],
    strict=True,
):
    obj = b.planner.obstacles[name]
    assert obj.getNodeType() == fcl.NODE_TYPE.GEOM_CYLINDER
    np.testing.assert_allclose(obj.getRotation(), np.eye(3), atol=1e-12, rtol=0)
    np.testing.assert_allclose(obj.getTranslation(), entry["center"], atol=1e-12, rtol=0)
    assert entry["radius_m"] == turntable[rkey] and entry["half_height_m"] == turntable[hkey] / 2
    np.testing.assert_allclose(
        entry["center"][:2], np.asarray(turntable["world_from_axis"])[:2, 3], atol=1e-12, rtol=0
    )
leaves = data["leaves"]
paths = sorted(v["path"] for v in leaves)
assert all(not second.startswith(first) for first, second in zip(paths, paths[1:], strict=False))
assert sum((Fraction(1, 2 ** len(v)) for v in paths), Fraction()) == 1
maximum = -np.inf
maximum_difference = 0
for begin in range(0, len(leaves), 1024):
    batch = leaves[begin : begin + 1024]
    centers = []
    angles = []
    indices = []
    for leaf in batch:
        low = np.asarray(data["root_low"]).copy()
        high = np.asarray(data["root_high"]).copy()
        for digit in leaf["path"]:
            assert digit in "01"
            axis = int(np.argmax(high - low))
            middle = (low[axis] + high[axis]) / 2
            if digit == "0":
                high[axis] = middle
            else:
                low[axis] = middle
        centers.append((low + high) / 2)
        angles.append(min(float(np.sum((high - low) / 2)), np.pi))
        indices.append(leaf["sphere"])
    assert min(indices) >= 0 and max(indices) < len(local)
    matrices = Rotation.from_euler("ZYX", centers).as_matrix()
    points = np.einsum("bij,bj->bi", matrices, local[indices]) + data["source_point_world_m"]
    gap = tree.query(points)[0] - radii[indices] - data["material_radius_m"]
    for cylinder in data["cylinders"]:
        delta = points - np.asarray(cylinder["center"])
        radial = np.maximum(np.linalg.norm(delta[:, :2], axis=1) - cylinder["radius_m"], 0)
        vertical = np.maximum(abs(delta[:, 2]) - cylinder["half_height_m"], 0)
        gap = np.minimum(gap, np.hypot(radial, vertical) - radii[indices])
    bounds = (
        gap + 2 * offset[indices] * np.sin(np.asarray(angles) / 2) + data["translation_allowance_m"]
    )
    assert np.all(bounds < data["required_clearance_m"] - data["numerical_reserve_m"])
    maximum = max(maximum, float(bounds.max()))
    maximum_difference = max(
        maximum_difference, max(abs(bounds - np.asarray([v["clearance_upper_m"] for v in batch])))
    )
assert maximum_difference < 1e-9
original = read_json(Path("runs/palace_turntable_v1/construction_plan.json"))["actions"]
revised = list(b.actions.values())


def framework_rows(rows):
    return sorted(
        (r["id"], r["segments"], r["diameter_m"]) for r in rows if r["stage"] == "framework"
    )


assert framework_rows(original) == framework_rows(revised)
source = next(r for r in original if r["parent_curve"] == row["parent_curve"])
assert source["segments"] == row["segments"] and source["stage"] == "artistic"
assert read_json(Path("runs/palace_turntable_v1/cell_config.json")) == b.planner.profile
result = {
    "status": (
        "VERIFIED_NO_TOOL_POSE_WITH_SOURCE_TOLERANCE"
        if args.source_tolerance
        else "VERIFIED_NO_TOOL_POSE_IN_CONSERVATIVE_FRAMEWORK"
    ),
    "certificate_sha256": sha256(proof),
    "section": data["section"],
    "parent_curve": row["parent_curve"],
    "leaves": len(leaves),
    "complete_orientation_cover": True,
    "distance_allowance_m": data["translation_allowance_m"],
    "tip_translation_allowance_m": 0.0003,
    "allowance_components_m": data.get("allowance_components_m"),
    "maximum_leaf_clearance_upper_m": maximum,
    "required_clearance_m": data["required_clearance_m"],
    "tool_ball_containment_verified": True,
    "applies_to_original_palace": True,
    "all_four_table_indices": (
        "Rigidly equivalent framework and axisymmetric plate/housing; all tool rotations covered"
    ),
    "inputs": {
        str(p): sha256(p)
        for p in [
            Path("runs/palace_turntable_v1/cell_config.json"),
            Path("runs/palace_turntable_v1/stroke_graph.json"),
            b.run / "stroke_graph.json",
            b.run / "construction_plan.json",
        ]
    },
    "verification_sources": {
        str(path): sha256(path)
        for path in [Path(__file__), Path("research/blondie/certify_palace_endpoint.py")]
    },
    "scope": (
        "Declared conservative planning geometry and provisional tool only; not a claim about "
        "unmeasured physical hardware or less conservative framework representations."
    ),
}
if args.source_tolerance:
    result["scope"] = (
        "Declared nominal strand and provisional tool geometry, allowing 0.3 mm tool tracking, "
        "0.35 mm framework point motion, and removal of 0.35 mm material inflation. "
        "This does not establish impossibility for different hardware or physical models."
    )
write_json(Path(f"research/blondie/palace_endpoint_{stem}_certificate_verification.json"), result)
print(result, flush=True)
