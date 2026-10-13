from pathlib import Path
from time import perf_counter

import numpy as np

from pen_art.cell import load_cell
from pen_art.io import read_json, sha256, write_json
from pen_art.kinematics import inverse_candidates
from pen_art.planning.cell_motion import pen_rotation
from pen_art.planning.curobo_adapter import CuroboAdapter

profile, arms, obstacles = load_cell("research/planning_cell/cell.yaml")
arm = arms["arm_1"]
gpu = CuroboAdapter(arm, batch_size=32)
rotation = pen_rotation(arm, np.asarray(profile["world_from_build"]), 0, 10)
reports = []
for name in ["cube", "triangular_prism", "torus", "bottle"]:
    graph = read_json(Path("research/primitive_fixtures") / name / "stroke_graph.json")
    points = np.asarray([n["position"] for n in graph["nodes"]])[
        :: max(1, len(graph["nodes"]) // 16)
    ][:16]
    targets = np.repeat(np.eye(4)[None], len(points), axis=0)
    targets[:, :3, 3] = points + np.asarray(profile["world_from_build"])[:3, 3]
    targets[:, :3, :3] = rotation
    gpu.solve(targets, reference=arm.initial_positions)
    start = perf_counter()
    proposals = gpu.solve(targets, reference=arm.initial_positions)
    gpu_seconds = perf_counter() - start
    start = perf_counter()
    cpu = [
        inverse_candidates(
            arm.robot,
            t,
            arm.flange_to_tcp,
            [arm.initial_positions],
            maximum_evaluations=60,
            position_tolerance=0.00005,
            orientation_tolerance=0.001,
        )
        for t in targets
    ]
    cpu_seconds = perf_counter() - start
    gpu_clear = sum(
        any(not arm.collision.check(q, obstacles, 0.001) for q in rows) for rows in proposals
    )
    reports.append(
        {
            "fixture": name,
            "targets": len(points),
            "gpu_ik_successes": sum(bool(r) for r in proposals),
            "gpu_static_clear_targets": gpu_clear,
            "cpu_ik_successes": sum(bool(r) for r in cpu),
            "gpu_warm_seconds": gpu_seconds,
            "cpu_seconds": cpu_seconds,
        }
    )
    print(reports[-1], flush=True)
write_json(
    Path("research/blondie/flexiv_ik_benchmark.json"),
    {
        "robot_urdf_sha256": sha256(arm.urdf),
        "cell_profile_sha256": sha256(Path("research/planning_cell/cell.yaml")),
        "scope": "IK proposals; exact FK and static mesh validation. "
        "Not complete trajectory certification.",
        "reports": reports,
        "gpu_setup_seconds": gpu.setup_seconds,
    },
)
