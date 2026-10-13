"""Bounded whole-zigzag IK and full local motion validation after a cooled base."""

import argparse
from pathlib import Path
from shutil import copy2
from time import monotonic

import numpy as np

from pen_art.curves import flatten_curve, transform_points
from pen_art.io import read_json, sha256, write_json
from pen_art.planning.axis_repair import repair_drawing_axes
from pen_art.planning.cell_motion import (
    anchored_strokes,
    drawing_alternatives,
    inverse_axis_candidates,
    radial_pen_rotations,
)
from pen_art.planning.cell_planner import tracked_drawing_path
from pen_art.planning.clock_path import ClockPath
from pen_art.planning.dual_arm import ScheduledMotion
from pen_art.planning.indexed_build import IndexedMotions
from pen_art.planning.scene import MaterialScene
from pen_art.planning.section_build import SectionBuild


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--section", default="section-00001")
    parser.add_argument("--maximum-seconds", type=float, default=180)
    parser.add_argument("--gpu", action="store_true")
    parser.add_argument("--skip-home-travel", action="store_true")
    parser.add_argument("--free-roll", action="store_true")
    parser.add_argument("--maximum-chord", type=float, default=0.003)
    parser.add_argument("--beam-width", type=int, default=2)
    parser.add_argument("--seed-count", type=int, default=2)
    parser.add_argument("--guided-axis", action="store_true")
    parser.add_argument("--base-witness", type=Path)
    parser.add_argument("--base-drawing", type=Path)
    parser.add_argument("--seed-drawing", type=Path)
    parser.add_argument("--reuse-seed", action="store_true")
    parser.add_argument("--repair-motion", action="store_true")
    parser.add_argument("--planning-buffer", type=float, default=0.0)
    parser.add_argument("--cached-collision", action="store_true")
    parser.add_argument("--max-candidates", type=int, default=8)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    started = monotonic()
    build = SectionBuild(args.run, args.run / "profile.yaml", args.maximum_seconds, gpu=args.gpu)
    if args.cached_collision:
        IndexedMotions(build)
    state = build.state({"section-00000"}, build.initial)
    start_q = build.initial
    if args.base_witness is not None:
        clock = ClockPath.from_record(read_json(args.base_witness)["trajectory"])
        scene = MaterialScene()
        for index, edge in enumerate(clock.edges):
            if edge.kind == "DRAW":
                build.planner.add_clock_edge(
                    scene, build.arm.identity, edge, f"actual-base/{index}"
                )
        start_q = clock.edges[-1].q1
        state = build.state({"section-00000"}, start_q, scene=scene)
    elif args.base_drawing is not None:
        base = read_json(args.base_drawing)
        tracked = tracked_drawing_path(build.arm, base, True, build.planner.deadline)
        if tracked["status"] != "BOUNDED":
            raise ValueError(f"Base drawing tracking is not bounded: {tracked['status']}")
        scene = MaterialScene()
        radius = build.actions["section-00000"]["diameter_m"] / 2
        for index, piece in enumerate(tracked["pieces"]):
            build.planner.add_piece(
                scene, build.arm.identity, piece, radius, f"base-geometry/{index}"
            )
        start_q = tracked["positions"][-1]
        state = build.state({"section-00000"}, start_q, scene=scene)
    state.payload.table_index = 0
    available = state.ready_nodes()
    arm, planner = build.arm, build.planner
    seeds = [start_q, build.initial, (arm.robot.lower + arm.robot.upper) / 2]
    rng = np.random.default_rng(70)
    seeds.extend(
        rng.uniform(arm.robot.lower, arm.robot.upper)
        for _ in range(max(0, args.seed_count - len(seeds)))
    )
    planner.timing["repair_drawing_axis"] = args.repair_motion
    # Axis work checkpoints belong to this private diagnostic, not the source run.
    for name in ("cell_manifest.json", "construction_plan.json"):
        copy2(args.run / name, args.output / name)
    planner.run = args.output
    report = {
        "scope": "one whole action with explicit cooled-base precondition",
        "run": str(args.run),
        "profile_sha256": sha256(args.run / "profile.yaml"),
        "section": args.section,
        "home_travel_included": not args.skip_home_travel,
        "self_static_clearance_certificates_enabled": args.cached_collision,
        "base_geometry_only_precondition": args.base_drawing is not None,
        "attempts": [],
        "status": "RUNNING",
    }

    def repair_with_buffer(drawing):
        original_margin = planner.margin
        try:
            planner.margin += args.planning_buffer
            return repair_drawing_axes(
                planner,
                drawing,
                state.payload,
                progress=lambda done, total, detail: print("axis", done, total, detail, flush=True),
            )
        finally:
            planner.margin = original_margin

    def proposals():
        if args.seed_drawing is not None:
            drawing = read_json(args.seed_drawing)
            if args.reuse_seed:
                drawing["status"] = "SAMPLED_DRAWING_PATH"
                yield drawing
                return
            repair = repair_with_buffer(drawing)
            drawing.update(repair)
            if repair["status"] == "SAMPLED_FREE_AXIS_PROPOSAL":
                drawing.update(status="SAMPLED_DRAWING_PATH", free_tool_orientation=True)
            yield drawing
            return
        if args.guided_axis:
            for oriented in anchored_strokes(build.actions[args.section], available):
                points, _ = flatten_curve(oriented, 0.0001, args.maximum_chord)
                points = transform_points(points, build.transform)
                rotations = radial_pen_rotations(arm, points, build.transform, 70)
                target = np.eye(4)
                target[:3, :3], target[:3, 3] = rotations[0], points[0]
                starts = inverse_axis_candidates(arm, target, seeds)
                starts = [
                    q
                    for q in starts
                    if not arm.collision.check(q, planner.obstacles, planner.margin)
                ]
                for initial in starts[:2]:
                    drawing = {
                        "arm": arm.identity,
                        "stroke_id": args.section,
                        "start_anchor": oriented["start"],
                        "end_anchor": oriented["end"],
                        "start_visit": oriented["start_visit"],
                        "reverse": oriented["reverse"],
                        "tcp_targets_m": points.tolist(),
                        "tcp_rotation": rotations[0].tolist(),
                        "tcp_rotations": rotations.tolist(),
                        "orientation_policy": "outward_radial",
                        "pen_tilt_deg": 70.0,
                        "roll_rad": 0.0,
                        "table_index": 0,
                        "positions": np.tile(initial, (len(points), 1)).tolist(),
                    }
                    repair = repair_with_buffer(drawing)
                    if repair["status"] == "SAMPLED_FREE_AXIS_PROPOSAL":
                        drawing.update(
                            repair, status="SAMPLED_DRAWING_PATH", free_tool_orientation=True
                        )
                    else:
                        drawing.update(repair)
                    yield drawing
            return
        yield from drawing_alternatives(
            arm,
            build.actions[args.section],
            available,
            build.transform,
            planner.obstacles,
            deadline=planner.deadline,
            seeds=seeds,
            settings={
                "maximum_subdivision_depth": 20,
                "free_roll": args.free_roll,
                "maximum_chord_m": args.maximum_chord,
                "beam_width": args.beam_width,
            },
            clearance_m=planner.margin,
            ik_backend=build.gpu,
        )

    for index, drawing in enumerate(proposals()):
        row = {
            "index": index,
            "status": drawing["status"],
            "pen_tilt_deg": drawing.get("pen_tilt_deg"),
            "roll_rad": drawing.get("roll_rad"),
            "failed_sample": drawing.get("failed_sample"),
            "sampling_diagnostics": drawing.get("sampling_diagnostics"),
            "seconds": monotonic() - started,
        }
        report["attempts"].append(row)
        if drawing["status"] == "SAMPLED_DRAWING_PATH":
            drawing.setdefault("variant", "outward-guided-local")
            if drawing.get("orientation_policy") == "outward_radial":
                drawing["tcp_rotations"] = radial_pen_rotations(
                    arm,
                    drawing["tcp_targets_m"],
                    build.transform,
                    drawing.get("pen_tilt_deg", 70),
                    drawing.get("roll_rad", 0),
                ).tolist()
            drawing.update(table_index=0, release_cooling_s=build.cooling)
            write_json(args.output / f"candidate-{index:03}.json", drawing)
            begin = len(planner.attempts)
            move = planner.prepare_drawing(
                state,
                arm.identity,
                drawing,
                None if args.skip_home_travel else start_q,
            )
            row["motion_failures"] = planner.attempts[begin:]
            row["motion_accepted"] = move is not None
            if move is not None:
                write_json(
                    args.output / "prepared_witness.json",
                    {"drawing": drawing, "trajectory": move.payload.record()},
                )
                _, failure = planner.validate_interval(
                    state, (ScheduledMotion(move, 0),), move.duration_s
                )
                row["replay_failure"] = failure
                if failure is None:
                    report["status"] = "LOCAL_WHOLE_MOTION_VERIFIED"
                    write_json(
                        args.output / "accepted_witness.json",
                        {"drawing": drawing, "trajectory": move.payload.record()},
                    )
        report["seconds"] = monotonic() - started
        write_json(args.output / "result.json", report)
        print(row, flush=True)
        if report["status"] == "LOCAL_WHOLE_MOTION_VERIFIED":
            break
        if index + 1 >= args.max_candidates:
            break
    if report["status"] == "RUNNING":
        report["status"] = "UNRESOLVED_WITHIN_BUDGET"
    report["seconds"] = monotonic() - started
    write_json(args.output / "result.json", report)
    print(report["status"], report["seconds"], flush=True)


if __name__ == "__main__":
    main()
