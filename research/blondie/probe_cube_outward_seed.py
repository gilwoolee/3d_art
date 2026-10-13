"""Untrusted radial pen-axis seed for one whole cube zigzag; no geometry changes."""

import argparse
from dataclasses import replace
from pathlib import Path
from time import monotonic

import numpy as np

from pen_art.cell import prepare_cell
from pen_art.curves import flatten_curve, transform_points
from pen_art.io import read_json, sha256, write_json
from pen_art.kinematics import inverse_candidates
from pen_art.planning.cell_motion import anchored_strokes
from pen_art.planning.clock_path import ClockPath
from pen_art.planning.dual_arm import ScheduledMotion
from pen_art.planning.section_build import SectionBuild, prepare_candidates
from pen_art.planning.source_coverage import audit_source_curve
from pen_art.planning.turntable import IndexedTurntable


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=Path("runs/cube_continuous_phase1_v1"))
    parser.add_argument("--output", type=Path, default=Path("runs/cube_outward_seed_probe_v1"))
    parser.add_argument("--maximum-seconds", type=float, default=180)
    parser.add_argument("--tilts", type=float, nargs="+", default=[45, 30, 60])
    parser.add_argument("--bed-tilt", type=float)
    args = parser.parse_args()
    started = monotonic()
    profile = Path("research/planning_cell/cell_turntable.yaml")
    source = Path(read_json(args.source / "run.json")["cell_source_run"])
    prepare_cell(profile, args.output, source)
    prepare_candidates(args.output)
    build = SectionBuild(
        args.output, profile, args.maximum_seconds, motion_library=args.output / "unused_library"
    )
    table = IndexedTurntable.from_profile(build.planner.profile)
    reference = read_json(args.source / "forward_checkpoint.json")["witnesses"][0]
    prefix = read_json(args.source / reference["file"])
    prefix_clock = ClockPath.from_record(prefix["trajectory"])
    completed = {prefix["section"]}
    index = prefix["drawing"]["table_index"]
    state = build.state(completed, prefix_clock.edges[-1].q1)
    state.payload.table_index = index
    row = build.actions["section-00001"]
    oriented = next(anchored_strokes(row, state.ready_nodes()))
    local, _ = flatten_curve(oriented, 0.00005, 0.002)
    transform = table.build_transform(index)
    points = transform_points(local, transform)
    report = {
        "status": "UNRESOLVED",
        "scope": "whole_first_cube_zigzag_after_cooled_base",
        "source_graph_sha256": sha256(args.source / "stroke_graph.json"),
        "probe_graph_sha256": sha256(args.output / "stroke_graph.json"),
        "members": row["members"],
        "trials": [],
        "prefix_assumption": "Existing base contour is deposited and cooled",
        "physical_status": "UNVERIFIED",
    }
    assert report["source_graph_sha256"] == report["probe_graph_sha256"]
    for tilt in args.tilts:
        if monotonic() >= build.planner.deadline:
            break
        q = np.asarray(prefix_clock.edges[-1].q1)
        positions, axes = [], []
        trial = {
            "outward_tilt_from_vertical_deg": tilt,
            "bed_tilt_deg": args.bed_tilt,
            "status": "IK_UNRESOLVED",
        }
        report["trials"].append(trial)
        for point in points:
            if monotonic() >= build.planner.deadline:
                break
            away = point - transform[:3, 3]
            away[2] = 0
            away /= np.linalg.norm(away)
            angle = tilt
            if args.bed_tilt is not None:
                fraction = local[len(positions), 2] / np.max(local[:, 2])
                angle = args.bed_tilt + fraction * (tilt - args.bed_tilt)
            body_axis = np.sin(np.deg2rad(angle)) * away + [0, 0, np.cos(np.deg2rad(angle))]
            z = -body_axis
            prior = build.arm.robot.forward(q, build.arm.flange_to_tcp)[:3, :3]
            x = prior[:, 0] - z * np.dot(prior[:, 0], z)
            if np.linalg.norm(x) < 1e-8:
                x = prior[:, 1] - z * np.dot(prior[:, 1], z)
            x /= np.linalg.norm(x)
            target = np.eye(4)
            target[:3, :3] = np.column_stack([x, np.cross(z, x), z])
            target[:3, 3] = point
            solved = inverse_candidates(
                build.arm.robot,
                target,
                build.arm.flange_to_tcp,
                [q],
                position_tolerance=1e-6,
                orientation_tolerance=0.001,
                maximum_evaluations=100,
            )
            if not solved:
                break
            q = np.asarray(solved[0]["positions"])
            positions.append(q.tolist())
            axes.append(body_axis.tolist())
        trial["solved_samples"] = len(positions)
        trial["required_samples"] = len(points)
        print(tilt, len(positions), "/", len(points), flush=True)
        write_json(args.output / "probe.json", report)
        if len(positions) != len(points):
            continue
        drawing = {
            "stroke_id": row["id"],
            "variant": f"radial-outward-{tilt}",
            "start_anchor": oriented["start"],
            "end_anchor": oriented["end"],
            "reverse": oriented["reverse"],
            "start_visit": oriented["start_visit"],
            "positions": positions,
            "tcp_targets_m": points.tolist(),
            "tcp_rotation": target[:3, :3].tolist(),
            "free_tool_orientation": True,
            "release_cooling_s": build.cooling,
            "table_index": index,
        }
        write_json(args.output / f"seed_{tilt}.json", {"drawing": drawing, "body_axes": axes})
        structural = build.progressive_structure(row, drawing, completed)
        trial["structure"] = structural
        if not structural["modeled_pass"]:
            trial["status"] = "STRUCTURE_REJECTED"
            continue
        move = build.planner._prepare_drawing(
            state, build.arm.identity, drawing, prefix_clock.edges[-1].q1
        )
        if move is None:
            trial.update(status="WHOLE_PATH_REJECTED", failure=build.planner.attempts[-1])
            write_json(args.output / "probe.json", report)
            continue
        # _prepare_drawing runs before indexed metadata decoration in normal callbacks.
        indexed = ClockPath([replace(edge, table_index=index) for edge in move.payload.edges])
        indexed.metadata.update(move.payload.metadata, drawing_table_index=index)
        move = replace(move, payload=indexed)
        _, failure = build.planner.validate_interval(
            state, (ScheduledMotion(move, 0),), move.duration_s
        )
        source_check = audit_source_curve(build.arm, [move.payload], points)
        trial.update(
            status="VERIFIED_LOCAL_WHOLE_PATH"
            if not failure and source_check["status"] == "BOUNDED"
            else "WHOLE_PATH_REPLAY_REJECTED",
            replay_failure=failure,
            source_audit=source_check,
        )
        write_json(
            args.output / f"witness_{tilt}.json",
            {"drawing": drawing, "trajectory": move.payload.record()},
        )
        if trial["status"] == "VERIFIED_LOCAL_WHOLE_PATH":
            report["status"] = trial["status"]
            break
    report["seconds"] = monotonic() - started
    write_json(args.output / "probe.json", report)
    print(report["status"], report["seconds"], flush=True)


if __name__ == "__main__":
    main()
