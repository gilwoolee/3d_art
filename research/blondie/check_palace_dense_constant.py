from pathlib import Path
from time import monotonic

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

from pen_art.io import sha256, write_json
from pen_art.planning.axis_corridor import pen_clear
from pen_art.planning.constant_axis import constant_axis_path
from pen_art.planning.dual_arm import ScheduledMotion
from pen_art.planning.section_build import SectionBuild

source = Path("runs/palace_spaced_seed_probe_v1")
run = Path("runs/palace_spaced_dense_constant_v1")
run.mkdir(exist_ok=False)
build = SectionBuild(source, Path("research/planning_cell/cell.yaml"), 600, gpu=True)
build.planner.run = run
row = build.actions["section-00602"]
ids = set(build.actions) - {row["id"]}
material = build.scene(ids)
state = build.state(ids, build.initial, material)
drawing = next(build.drawings(row, state.ready_nodes(), build.initial))
points = np.asarray(drawing["tcp_targets_m"])
original = np.asarray(drawing["tcp_rotation"])
rotations = [original]
radius = row["diameter_m"] / 2 + 0.00002
clearance = build.planner.margin + 0.0001
for z in trimesh.creation.icosphere(subdivisions=3).vertices:
    x = original[:, 0] - z * np.dot(original[:, 0], z)
    if np.linalg.norm(x) < 1e-6:
        x = original[:, 1] - z * np.dot(original[:, 1], z)
    x /= np.linalg.norm(x)
    frame = np.column_stack([x, np.cross(z, x), z])
    rotations.extend(
        frame @ Rotation.from_rotvec([0, 0, a]).as_matrix() for a in np.arange(8) * np.pi / 4
    )
report = {
    "status": "NO_ACCEPTED_DENSE_CONSTANT_AXIS",
    "section": row["id"],
    "initial_orientation_count": len(rotations),
    "point_counts": [],
    "attempts": [],
    "inputs": {
        str(source / p): sha256(source / p)
        for p in ["stroke_graph.json", "construction_plan.json", "cell_manifest.json"]
    },
    "simulation_only": True,
}
scene = material.copy()
began = monotonic()
for i, point in enumerate(points):
    scene.add(points[max(0, i - 1)], point, radius, f"constant-guide/{i}")
    rotations = [
        r for r in rotations if pen_clear(build.planner, build.arm, point, r, scene, clearance)
    ]
    report["point_counts"].append(len(rotations))
    print("Point", i, "constant orientations", len(rotations), flush=True)
    write_json(run / "comparison.json", report)
    if not rotations:
        break
ranked = sorted(rotations, key=lambda r: Rotation.from_matrix(r @ original.T).magnitude())
for i, rot in enumerate(ranked[:16]):
    if monotonic() >= build.planner.deadline:
        break
    proposal = constant_axis_path(build.planner, drawing, material, rot)
    record = {"orientation": i, "sampled_path": proposal is not None}
    if proposal:
        candidate = {**drawing, **proposal}
        move = build.planner.prepare_drawing(state, build.arm.identity, candidate, None)
        record["prepared"] = move is not None
        if move:
            after, failure = build.planner.validate_interval(
                state, (ScheduledMotion(move, 0),), move.duration_s
            )
            record["replay_failure"] = failure
            if not failure:
                report.update(
                    status="ACCEPTED_DENSE_CONSTANT_ART_SECTION",
                    drawing=candidate,
                    trajectory=move.payload.record(),
                    duration_s=move.duration_s,
                )
    report["attempts"].append(record)
    write_json(run / "comparison.json", report)
    print(record, flush=True)
    if report["status"] == "ACCEPTED_DENSE_CONSTANT_ART_SECTION":
        break
report["seconds"] = monotonic() - began
write_json(run / "comparison.json", report)
print(report["status"], flush=True)
