"""Finite, one-arm Palace restart trial; does not replace accepted run artifacts."""

import argparse
import hashlib
import json
from dataclasses import replace
from pathlib import Path

import numpy as np
import yaml

from pen_art.io import sha256, write_json
from pen_art.planning.axis_repair import refine_axis_path, solve_joint_pose
from pen_art.planning.cell_planner import CellPlanner
from pen_art.planning.clock_path import ClockPath
from pen_art.planning.dual_arm import ScheduleState
from pen_art.planning.scene import MaterialScene
from pen_art.planning.stroke_restarts import (
    audit_restart_clock,
    posture_break_candidates,
    release_screen,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, default=Path("runs/palace_15cm_one_arm"))
    parser.add_argument("--stroke", default="draw-0023")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--verify-inputs", action="store_true")
    parser.add_argument(
        "--profile", type=Path, default=Path("configs/cell/one_flexiv_straight.yaml")
    )
    args = parser.parse_args()
    run = args.run
    output = run / "stroke_restarts"
    cfg = json.loads((run / "allocation_config.json").read_text())
    settings = yaml.safe_load(Path("configs/dual_arm.yaml").read_text())
    settings.update(maximum_subdivision_depth=20, assumed_cooling_s=cfg["cooling_s"])
    profile = args.profile
    planner = CellPlanner(run, profile, settings, float("inf"), timing_override=cfg["timing"])
    if len(planner.arms) != 1:
        raise ValueError("This experiment requires exactly one arm")
    identity = next(iter(planner.arms))
    arm = planner.arms[identity]
    scene = MaterialScene()
    motions = json.loads((run / "allocation_motions.json").read_text())["motions"]
    if not motions or any(m["stroke_id"] == args.stroke for m in motions):
        raise ValueError("Trial requires a prepared prefix preceding the unprepared stroke")
    printed = {e for m in motions for e in planner.actions[m["stroke_id"]]["members"]}
    for m in motions:
        for i, edge in enumerate(ClockPath.from_record(m["trajectory"]).edges):
            if edge.kind == "DRAW":
                planner.add_clock_edge(scene, identity, edge, f"{identity}/{m['stroke_id']}/{i}")
    q_start = ClockPath.from_record(motions[-1]["trajectory"]).edges[-1].q1
    drawing = next(d for d in planner.candidates["variants"] if d["stroke_id"] == args.stroke)
    work = run / "axis_repairs" / f"work_{identity}_{args.stroke}.json"
    work_record = json.loads(work.read_text())
    expected = {
        "cell_sha256": sha256(run / "cell_manifest.json"),
        "construction_sha256": sha256(run / "construction_plan.json"),
        "drawing_sha256": hashlib.sha256(json.dumps(drawing, sort_keys=True).encode()).hexdigest(),
        "material_sha256": planner.material_key(scene),
    }
    if any(work_record["binding"].get(k) != value for k, value in expected.items()):
        raise ValueError("Posture proposal was computed for a different drawing or material prefix")
    q = np.array(work_record["positions"])
    points = np.array(drawing["tcp_targets_m"])
    if len(q) != len(points) or not np.isfinite(q).all():
        raise ValueError("Posture proposal does not match source targets")
    if args.verify_inputs:
        error = max(
            np.linalg.norm(arm.robot.forward(qa, arm.flange_to_tcp)[:3, 3] - point)
            for qa, point in zip(q, points, strict=True)
        )
        print(json.dumps({"inputs_match": True, "maximum_tip_error_m": float(error)}))
        return
    transform = np.array(yaml.safe_load(profile.read_text())["world_from_build"])
    if not np.allclose(transform[:3, :3], np.eye(3)):
        raise ValueError("Structural screen needs world-aligned build gravity")
    build_points = points - transform[:3, 3]
    material = json.loads((run / "fabrication_config.json").read_text())["structure"]
    action = planner.actions[drawing["stroke_id"]]
    cuts = posture_break_candidates(q) + [len(q) - 1]
    report = dict(
        status="RUNNING",
        arm=identity,
        stroke_id=drawing["stroke_id"],
        simulation_only=True,
        cooling_calibrated=False,
        cooling_s=cfg["cooling_s"],
        segments=[],
        failures=[],
        source_target_count=len(points),
        inputs={
            name: sha256(run / name)
            for name in (
                "allocation_motions.json",
                "cell_drawing_variants.json",
                "cell_config.json",
                "stroke_graph.json",
                "fabrication_config.json",
            )
        },
        posture_proposal_sha256=sha256(work),
    )
    output.mkdir(parents=True, exist_ok=True)
    saved = json.loads((output / "trial.json").read_text()) if args.resume else None
    edges, start = [], 0
    if saved:
        for key in ("inputs", "posture_proposal_sha256", "stroke_id", "cooling_s"):
            if saved.get(key) != report[key]:
                raise ValueError(f"Restart checkpoint changed: {key}")
        for segment in saved["segments"]:
            lo, hi = segment["start_index"], segment["end_index"]
            if lo != start:
                raise ValueError("Checkpoint leaves a contour gap")
            path = ClockPath.from_record(
                json.loads((output / f"chunk_{lo:03d}_{hi:03d}.json").read_text())
            )
            offset = edges[-1].end_s if edges else 0
            edges.extend(
                replace(e, start_s=e.start_s + offset, end_s=e.end_s + offset) for e in path.edges
            )
            for i, edge in enumerate(path.edges):
                if edge.kind == "DRAW":
                    planner.add_clock_edge(scene, identity, edge, f"restart/{lo}/{i}")
            q_start, start = path.edges[-1].q1, hi
        report["segments"] = saved["segments"]
        report["prior_failures"] = saved.get("failures", [])
        cuts = [cut for cut in cuts if cut > start]
        # One diagnostic-driven subdivision: split at the source vertex nearest
        # the failed continuous interval. No new geometry or orientation search.
        failures = saved.get("failures", [])
        if failures and failures[-1].get("phase") == "DRAW" and "positions" in failures[-1]:
            witness = arm.robot.forward(failures[-1]["positions"], arm.flange_to_tcp)[:3, 3]
            interior = np.arange(start + 1, saved["blocked_index"])
            if len(interior):
                cut = int(interior[np.argmin(np.linalg.norm(points[interior] - witness, axis=1))])
                cuts = sorted(set([cut, *cuts]))
                report["failure_driven_break_index"] = cut
        if saved["status"] == "RELEASE_SCREEN_REJECTED":
            for cut in range(saved["blocked_index"] - 1, start, -1):
                screen = release_screen(
                    planner.graph,
                    material,
                    printed,
                    build_points[: cut + 1],
                    action["diameter_m"],
                    action["start"],
                )
                write_json(output / f"release_{cut:03d}.json", screen)
                if screen["modeled_pass"]:
                    cuts = sorted(set([cut, *cuts]))
                    report["support_driven_break_index"] = cut
                    break
    for stale in ("trajectory.json", "restart_audit.json"):
        (output / stale).unlink(missing_ok=True)
    write_json(output / "trial.json", report)
    for end in cuts:
        screen = release_screen(
            planner.graph,
            material,
            printed,
            build_points[: end + 1],
            action["diameter_m"],
            action["start"],
        )
        write_json(output / f"release_{end:03d}.json", screen)
        if not screen["modeled_pass"]:
            report.update(status="RELEASE_SCREEN_REJECTED", blocked_index=end)
            break
        positions = q[start : end + 1].copy()
        if start:
            # The new chunk begins at the SAME tip point, with a posture seeded
            # from the outgoing side of the previously discontinuous transition.
            seed = q[start + 1]
            solved, error, gaps = solve_joint_pose(
                planner,
                arm,
                points[start],
                -arm.robot.forward(seed, arm.flange_to_tcp)[:3, 2],
                seed,
                seed,
                scene,
                planner.margin + 0.00002,
            )
            if np.linalg.norm(error) > 1e-6 or gaps.min() < planner.margin + 0.000018:
                report.update(status="RESTART_POSE_REJECTED", blocked_index=start)
                break
            positions[0] = solved.x
        refined_q, refined_points = refine_axis_path(
            arm, positions.tolist(), points[start : end + 1].tolist()
        )
        sub = {
            **drawing,
            "positions": refined_q,
            "tcp_targets_m": refined_points,
            "free_tool_orientation": True,
            "release_cooling_s": cfg["cooling_s"],
        }
        state = ScheduleState(0, ((identity, tuple(q_start)),), frozenset(), (), payload=scene)
        print(f"Preparing restart chunk {start}:{end}", flush=True)
        move = planner._prepare_drawing(state, identity, sub, np.asarray(q_start))
        if move is None:
            report.update(
                status="MOTION_REJECTED", blocked_index=end, failures=planner.attempts[-3:]
            )
            break
        write_json(output / f"chunk_{start:03d}_{end:03d}.json", move.payload.record())
        offset = edges[-1].end_s if edges else 0
        edges.extend(
            replace(e, start_s=e.start_s + offset, end_s=e.end_s + offset)
            for e in move.payload.edges
        )
        for i, edge in enumerate(move.payload.edges):
            if edge.kind == "DRAW":
                planner.add_clock_edge(scene, identity, edge, f"restart/{start}/{i}")
        report["segments"].append(
            {"start_index": start, "end_index": end, "duration_s": move.duration_s}
        )
        q_start = np.array(move.end_configuration)
        start = end
        write_json(output / "trial.json", report)
    else:
        report["status"] = "COMPLETE_MODELED_MOTION"
    if edges:
        path = ClockPath(edges)
        path.metadata.update(
            restart_trial=True, complete=report["status"] == "COMPLETE_MODELED_MOTION"
        )
        record = path.record()
        restored = ClockPath.from_record(record)
        report["restart_audit"] = audit_restart_clock(
            restored, arm, [points[s["end_index"]] for s in report["segments"][:-1]]
        )
        write_json(output / "trajectory.json", record)
        write_json(
            run / "stroke_restart_preview.json",
            {
                "inputs": report["inputs"],
                "arm": identity,
                "stroke_id": args.stroke,
                "restart_count": report["restart_audit"]["restart_count"],
                "trajectory": record,
                "prior_paths": [
                    [
                        point
                        for edge in m["trajectory"]["edges"]
                        if edge["kind"] == "DRAW"
                        for point in (edge["tcp0_m"], edge["tcp1_m"])
                    ]
                    for m in motions
                ],
            },
        )
    write_json(output / "trial.json", report)
    print(json.dumps({k: v for k, v in report.items() if k != "failures"}, indent=2), flush=True)


if __name__ == "__main__":
    main()
