"""Independently admit one whole path against its verified actual base deposition."""

import argparse
from pathlib import Path
from time import monotonic

import numpy as np

from pen_art.io import read_json, sha256, write_json
from pen_art.planning.clock_path import ClockPath
from pen_art.planning.motion_cache_precompute import (
    independently_validate_growing_prefix,
    request_for_action,
)
from pen_art.planning.motion_library import MotionLibrary
from pen_art.planning.motion_runtime import load_context_for_run
from pen_art.planning.scene import MaterialScene
from pen_art.planning.section_build import SectionBuild
from pen_art.planning.tool import sampled_pen_tilt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--base-witness", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--maximum-seconds", type=float, default=180)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    profile = args.run / "profile.yaml"
    build = SectionBuild(args.run, profile, args.maximum_seconds, gpu=False)
    bound = load_context_for_run(args.run, profile)
    candidate = read_json(args.candidate)
    row = build.actions[candidate["stroke_id"]]
    scene = MaterialScene()
    clock = ClockPath.from_record(read_json(args.base_witness)["trajectory"])
    for index, edge in enumerate(clock.edges):
        if edge.kind == "DRAW":
            build.planner.add_clock_edge(scene, build.arm.identity, edge, f"actual-base/{index}")
    material_path = args.output / "initial_material.json"
    write_json(material_path, {"capsules": scene.segments})
    request = request_for_action(
        row, args.run, build.planner.turntable, candidate["table_index"], bound["context_hash"],
        reverse=candidate.get("reverse", False), start_visit=candidate.get("start_visit", 0),
    )
    library = MotionLibrary(args.library)
    started = monotonic()

    def progress(stage, detail):
        if stage == "self_static_checked":
            write_json(args.output / "self_static_checkpoint.json", {
                **detail, "request": request.as_dict(),
                "inputs": {str(profile): sha256(profile),
                           str(args.candidate): sha256(args.candidate)},
            })
            detail = {"poses": len(detail["drawing"]["positions"])}
        update = {"stage": stage, "seconds": monotonic() - started, **detail}
        write_json(args.output / "progress.json", update)
        print(update, flush=True)

    checked, validation = independently_validate_growing_prefix(
        library, build.arm, build.planner.obstacles, request, candidate,
        monotonic() + args.maximum_seconds, material_radius_m=row["diameter_m"] / 2,
        initial_scene=scene, clearance_m=build.planner.margin, progress=progress,
    )
    result = {"status": "REJECTED" if checked is None else "VERIFIED_PREFIX",
              "validation": validation, "seconds": monotonic() - started,
              "record_id": None, "initial_material_path": str(material_path)}
    if checked is not None:
        world_center = build.transform[:3, 3]
        points = np.array(checked["tcp_targets_m"])
        radial = points[:, :2] - world_center[:2]
        radial /= np.linalg.norm(radial, axis=1)[:, None]
        axes = np.array([-build.arm.robot.forward(q, build.arm.flange_to_tcp)[:2, 2]
                         for q in checked["positions"]])
        result["minimum_outward_dot"] = float(np.sum(axes * radial, axis=1).min())
        inputs = [args.candidate, args.base_witness, profile, material_path,
                  args.run / "construction_plan.json"]
        result["record_id"] = library.remember_growing_prefix(
            request, checked, validation,
            {"run": str(args.run), "section": candidate["stroke_id"],
             "parent_curve": row["parent_curve"], "generator": "actual_base_prefix_admission",
             "source_sha256": sha256(args.run / "construction_plan.json"),
             "pen_tilt_deg": candidate.get("pen_tilt_deg"),
             "actual_pen_tilt": sampled_pen_tilt(build.arm, checked["positions"]),
             "initial_material_path": str(material_path),
             "inputs": {str(path): sha256(path) for path in inputs}},
        )
        write_json(args.output / "checked_drawing.json", checked)
    write_json(args.output / "result.json", result)
    print(result["status"], result["record_id"], result["seconds"], flush=True)


if __name__ == "__main__":
    main()
