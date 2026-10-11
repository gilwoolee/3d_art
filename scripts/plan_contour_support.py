"""Plan an automatic base post for a structurally blocked one-arm contour."""

import json
from pathlib import Path

import numpy as np
import yaml

from pen_art.graph import straight_edge
from pen_art.io import write_json
from pen_art.planning.axis_repair import repair_drawing_axes
from pen_art.planning.cell_motion import drawing_alternatives
from pen_art.planning.cell_planner import CellPlanner
from pen_art.planning.clock_path import ClockPath
from pen_art.planning.dual_arm import ScheduleState
from pen_art.planning.scene import MaterialScene
from pen_art.planning.stroke_restarts import release_screen
from pen_art.structure import analyze_structure


def main():
    run = Path("runs/palace_15cm_one_arm")
    output = run / "full_build"
    cfg = json.loads((run / "allocation_config.json").read_text())
    settings = yaml.safe_load(Path("configs/dual_arm.yaml").read_text())
    settings.update(maximum_subdivision_depth=20, assumed_cooling_s=cfg["cooling_s"])
    planner = CellPlanner(
        run,
        Path("configs/cell/one_flexiv_right_angle_lean.yaml"),
        settings,
        float("inf"),
        timing_override=cfg["timing"],
    )
    arm = planner.arms["arm_1"]
    scene = MaterialScene()
    motions = json.loads((run / "allocation_motions.json").read_text())["motions"]
    printed = {e for m in motions for e in planner.actions[m["stroke_id"]]["members"]}
    for motion in motions:
        for i, edge in enumerate(ClockPath.from_record(motion["trajectory"]).edges):
            if edge.kind == "DRAW":
                planner.add_clock_edge(
                    scene, arm.identity, edge, f"{arm.identity}/{motion['stroke_id']}/{i}"
                )
    partial = ClockPath.from_record(
        json.loads((run / "stroke_restarts/trajectory.json").read_text())
    )
    for i, edge in enumerate(partial.edges):
        if edge.kind == "DRAW":
            planner.add_clock_edge(scene, arm.identity, edge, f"partial/{i}")
    q_start = partial.edges[-1].q1
    drawing = next(x for x in planner.candidates["variants"] if x["stroke_id"] == "draw-0023")
    points = (
        np.array(drawing["tcp_targets_m"]) - np.array(planner.profile["world_from_build"])[:3, 3]
    )
    trial = json.loads((run / "stroke_restarts/trial.json").read_text())
    cut = trial["segments"][-1]["end_index"]
    top = points[cut]
    bottom = top.copy()
    bottom[2] = 0
    identity = f"auto-support-{drawing['stroke_id']}-{cut}"
    nodes = {
        name: {"id": name, "position": p.tolist(), "ground": ground}
        for name, p, ground in [(identity + "-foot", bottom, True), (identity + "-top", top, False)]
    }
    edge = straight_edge(identity + "-foot", identity + "-top", nodes, identity, "support", 0.001)
    support = {
        **edge,
        "closed": False,
        "members": [identity],
        "junction_visits": [
            {"node": edge["start"], "distance_m": 0, "segment_boundary": 0},
            {"node": edge["end"], "distance_m": edge["length_m"], "segment_boundary": 1},
        ],
    }
    graph = {
        **planner.graph,
        "nodes": [*planner.graph["nodes"], *nodes.values()],
        "edges": [*planner.graph["edges"], edge],
    }
    material = json.loads((run / "fabrication_config.json").read_text())["structure"]
    post_screen = analyze_structure({"nodes": list(nodes.values()), "edges": [edge]}, material)
    screens = {
        i: release_screen(
            graph,
            material,
            printed | {identity},
            points[: i + 1],
            0.001,
            planner.actions["draw-0023"]["start"],
        )
        for i in [155, 161, 169, 172, 173, 209, 245]
    }
    write_json(
        output / "support_proposal.json",
        {
            "action": support,
            "nodes": list(nodes.values()),
            "post_screen": post_screen,
            "contour_screens": screens,
        },
    )
    print("Post height", edge["length_m"], "post pass", post_screen["modeled_pass"], flush=True)
    print(
        "Supported releases",
        [(i, s["modeled_pass"], s.get("maximum_displacement_m")) for i, s in screens.items()],
        flush=True,
    )
    if not post_screen["modeled_pass"] or not all(s["modeled_pass"] for s in screens.values()):
        raise ValueError("Proposed post does not resolve the structural screen")
    planner.actions[identity] = support
    state = ScheduleState(0, ((arm.identity, tuple(q_start)),), frozenset(), (), payload=scene)
    world = np.array(planner.profile["world_from_build"])
    bottom_world = bottom + world[:3, 3]
    nearby = sorted(
        (float(np.linalg.norm(np.asarray(point) - bottom_world)), tuple(q))
        for row in planner.candidates["variants"]
        for point, q in zip(row["tcp_targets_m"], row["positions"], strict=True)
    )
    seeds = [np.asarray(nearby[0][1]), np.asarray(q_start), arm.initial_positions]
    for candidate in drawing_alternatives(
        arm,
        support,
        {support["start"]},
        np.array(planner.profile["world_from_build"]),
        planner.obstacles,
        deadline=float("inf"),
        seeds=seeds,
        roll_angles=(0.0, np.pi),
        clearance_m=planner.margin,
        pen_tilt_deg=10,
    ):
        print("Post candidate", candidate["status"], flush=True)
        if candidate["status"] != "SAMPLED_DRAWING_PATH":
            continue
        candidate["release_cooling_s"] = cfg["cooling_s"]
        move = planner._prepare_drawing(state, arm.identity, candidate, np.array(q_start))
        if move:
            write_json(
                output / "support_motion.json",
                {
                    "status": "PREPARED",
                    "action": support,
                    "drawing": candidate,
                    "trajectory": move.payload.record(),
                },
            )
            print("Support motion prepared", move.duration_s, flush=True)
            return
    targets = np.linspace(bottom_world, top + world[:3, 3], 32)
    candidate = {
        "arm": arm.identity,
        "stroke_id": identity,
        "variant": "free-axis-post",
        "start_anchor": support["start"],
        "end_anchor": support["end"],
        "positions": np.repeat(seeds[0][None], len(targets), axis=0).tolist(),
        "tcp_targets_m": targets.tolist(),
        "tcp_rotation": arm.robot.forward(seeds[0], arm.flange_to_tcp)[:3, :3].tolist(),
        "release_cooling_s": cfg["cooling_s"],
    }
    repaired = repair_drawing_axes(planner, candidate, scene)
    write_json(output / "support_axis_repair.json", repaired)
    if repaired["status"] == "SAMPLED_FREE_AXIS_PROPOSAL":
        candidate.update(repaired)
        move = planner._prepare_drawing(state, arm.identity, candidate, np.asarray(q_start))
        if move:
            write_json(
                output / "support_motion.json",
                {
                    "status": "PREPARED",
                    "action": support,
                    "drawing": candidate,
                    "trajectory": move.payload.record(),
                },
            )
            return
    write_json(output / "support_motion.json", {"status": "REJECTED", "failures": planner.attempts})


if __name__ == "__main__":
    main()
