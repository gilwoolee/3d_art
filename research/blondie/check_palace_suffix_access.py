"""Inspect the next artistic section after the two accepted suffix removals."""

import argparse
from collections import Counter
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

from pen_art.cell import prepare_cell
from pen_art.io import sha256, write_json
from pen_art.planning.axis_corridor import pen_clear
from pen_art.planning.collision import placed
from pen_art.planning.section_build import SectionBuild, prepare_candidates

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--run", type=Path, default=Path("runs/palace_art_suffix_access_v1"))
parser.add_argument("--section", default="section-00602-part-3")
parser.add_argument("--extra-clearance-m", type=float, default=0.0)
args = parser.parse_args()
if not np.isfinite(args.extra_clearance_m) or args.extra_clearance_m < 0:
    parser.error("Extra clearance must be finite and nonnegative")
source = Path("runs/palace_hybrid_art_order_v1")
run = args.run
assert not run.exists()
prepare_cell(Path("research/planning_cell/cell.yaml"), run, source)
prepare_candidates(run)
build = SectionBuild(run, Path("research/planning_cell/cell.yaml"), 600, gpu=False)
removed = {"section-00602-part-2", "section-00601-part-9"}
row = build.actions[args.section]
remaining = set(build.actions) - removed - {row["id"]}
material = build.scene(remaining)
framework = build.scene(key for key in remaining if build.actions[key]["stage"] == "framework")
state = build.state(remaining, build.initial, material)
drawing = next(build.drawings(row, state.ready_nodes(), build.initial))
points = np.asarray(drawing["tcp_targets_m"])
original = np.asarray(drawing["tcp_rotation"])
rotations = [original]
for z in trimesh.creation.icosphere(subdivisions=3).vertices:
    x = original[:, 0] - z * np.dot(original[:, 0], z)
    if np.linalg.norm(x) < 1e-6:
        x = original[:, 1] - z * np.dot(original[:, 1], z)
    x /= np.linalg.norm(x)
    frame = np.column_stack([x, np.cross(z, x), z])
    rotations.extend(
        frame @ Rotation.from_rotvec([0, 0, angle]).as_matrix()
        for angle in np.arange(8) * np.pi / 4
    )
report = {
    "status": "SUFFIX_ACCESS_SCREEN_RUNNING",
    "section": row["id"],
    "removed_sections": sorted(removed),
    "drawing": drawing,
    "sampled_orientations": len(rotations),
    "required_clearance_m": build.planner.margin,
    "screen_clearance_m": build.planner.margin + args.extra_clearance_m,
    "point_checks": [],
    "simulation_only": True,
    "scope": "Tool-only finite orientation samples; no robot-path or impossibility claim.",
    "inputs": {
        str(p): sha256(p)
        for p in [
            run / "stroke_graph.json",
            run / "construction_plan.json",
            run / "cell_manifest.json",
            Path(__file__),
        ]
    },
}
write_json(run / "access.json", report)
scene = material.copy()
clearance = build.planner.margin + args.extra_clearance_m
radius = row["diameter_m"] / 2 + 0.00002
for index, point in enumerate(points):
    scene.add(points[max(0, index - 1)], point, radius, f"suffix-guide/{index}")
    allowed = [
        r for r in rotations if pen_clear(build.planner, build.arm, point, r, scene, clearance)
    ]
    check = {"index": index, "point_m": point.tolist(), "full_material_clear": len(allowed)}
    if not allowed:
        framework_clear = [
            r
            for r in rotations
            if pen_clear(build.planner, build.arm, point, r, framework, clearance)
        ]
        counts = Counter()
        for rotation in framework_clear:
            target = np.eye(4)
            target[:3, :3], target[:3, 3] = rotation, point
            flange = target @ np.linalg.inv(build.arm.flange_to_tcp)
            blockers = set()
            for body in build.arm.tool.bodies:
                obj = placed(body.geometry, flange @ body.origin)
                nearest = scene.nearest(obj)
                if nearest and nearest["distance_m"] < clearance:
                    blockers.add(nearest["name"].split("/")[0])
            counts.update(blockers)
        check.update(
            framework_clear=len(framework_clear),
            blocker_pose_counts=dict(counts.most_common()),
        )
    report["point_checks"].append(check)
    write_json(run / "access.json", report)
    print(check, flush=True)
report["status"] = "FINITE_SUFFIX_ACCESS_SCREEN"
write_json(run / "access.json", report)
print(report["status"], flush=True)
