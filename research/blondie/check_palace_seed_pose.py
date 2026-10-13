from pathlib import Path
from time import monotonic

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

from pen_art.cell import prepare_cell
from pen_art.io import sha256, write_json
from pen_art.planning.axis_corridor import pen_clear
from pen_art.planning.axis_repair import repair_drawing_axes
from pen_art.planning.dual_arm import ScheduledMotion
from pen_art.planning.section_build import SectionBuild, prepare_candidates

source = Path("runs/palace_hybrid_spaced_1200um_source")
run = Path("runs/palace_spaced_seed_probe_v1")
assert not run.exists()
prepare_cell(Path("research/planning_cell/cell.yaml"), run, source)
prepare_candidates(run)
build = SectionBuild(run, Path("research/planning_cell/cell.yaml"), 600, gpu=True)
row = build.actions["section-00602"]
ids = set(build.actions) - {row["id"]}
scene = build.scene(ids)
state = build.state(ids, build.initial, scene)
drawing = next(build.drawings(row, state.ready_nodes(), build.initial))
point = np.asarray(drawing["tcp_targets_m"][0])
original = np.asarray(drawing["tcp_rotation"])
test_scene = scene.copy()
test_scene.add(point, point, row["diameter_m"] / 2 + 0.00002, "seed-guide")
rotations = []
start = monotonic()
sampled = 0
for z in trimesh.creation.icosphere(subdivisions=3).vertices:
    x = original[:, 0] - z * np.dot(original[:, 0], z)
    if np.linalg.norm(x) < 1e-6:
        x = original[:, 1] - z * np.dot(original[:, 1], z)
    x /= np.linalg.norm(x)
    frame = np.column_stack([x, np.cross(z, x), z])
    for angle in np.arange(8) * np.pi / 4:
        rotation = frame @ Rotation.from_rotvec([0, 0, angle]).as_matrix()
        sampled += 1
        if pen_clear(
            build.planner, build.arm, point, rotation, test_scene, build.planner.margin + 0.0001
        ):
            rotations.append(rotation)
print(
    "Clear first-tip orientations",
    len(rotations),
    "of",
    sampled,
    "seconds",
    monotonic() - start,
    flush=True,
)
report = {
    "status": "NO_ACCEPTED_NEW_SEED",
    "run": str(run),
    "section": row["id"],
    "sampled_orientations": sampled,
    "clear_tool_orientations": len(rotations),
    "attempts": [],
    "simulation_only": True,
    "scope": (
        "One artistic section with the complete framework and other artistic sections present; "
        "finite orientation sample only."
    ),
    "inputs": {
        str(run / p): sha256(run / p)
        for p in ["stroke_graph.json", "construction_plan.json", "cell_manifest.json"]
    },
}
write_json(run / "seed_probe.json", report)
if rotations:
    ranked = sorted(rotations, key=lambda r: Rotation.from_matrix(r @ original.T).magnitude())
    selected = ranked[:16] + ranked[:: max(1, len(ranked) // 48)]
    targets = np.repeat(np.eye(4)[None], len(selected), axis=0)
    targets[:, :3, :3] = selected
    targets[:, :3, 3] = point
    solutions = build.gpu.solve(targets, reference=np.asarray(drawing["positions"][0]))
    seeds = []
    for rot, qs in zip(selected, solutions, strict=True):
        for q in qs:
            if not build.arm.collision.check(
                q, build.planner.obstacles, build.planner.margin + 0.0001, test_scene
            ):
                seeds.append((q, rot))
                break
    print("Reachable clear starting poses", len(seeds), flush=True)
    report["reachable_clear_starts"] = len(seeds)
    for i, (q, rot) in enumerate(seeds[:8]):
        if monotonic() >= build.planner.deadline:
            break
        proposal = {
            **drawing,
            "variant": drawing["variant"] + f"-dense-seed-{i}",
            "positions": [q.tolist(), *drawing["positions"][1:]],
            "tcp_rotation": rot.tolist(),
        }
        repaired = repair_drawing_axes(
            build.planner,
            proposal,
            scene,
            progress=lambda done, total, _, seed=i: print(
                "Seed", seed, "pose", done, total, flush=True
            ),
        )
        item = {"seed": i, "sampled_status": repaired["status"]}
        report["attempts"].append(item)
        if repaired["status"] == "SAMPLED_FREE_AXIS_PROPOSAL":
            proposal.update(
                positions=repaired["positions"],
                tcp_targets_m=repaired["tcp_targets_m"],
                free_tool_orientation=True,
            )
            move = build.planner.prepare_drawing(state, build.arm.identity, proposal, None)
            item["prepared"] = move is not None
            if move:
                after, failure = build.planner.validate_interval(
                    state, (ScheduledMotion(move, 0.0),), move.duration_s
                )
                item["replay_failure"] = failure
                if not failure:
                    report.update(
                        status="ACCEPTED_ARTISTIC_SECTION",
                        drawing=proposal,
                        trajectory=move.payload.record(),
                        duration_s=move.duration_s,
                    )
        write_json(run / "seed_probe.json", report)
        print(item, flush=True)
        if report["status"] == "ACCEPTED_ARTISTIC_SECTION":
            break
report["seconds"] = monotonic() - start
write_json(run / "seed_probe.json", report)
print(report["status"], flush=True)
