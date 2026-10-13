"""Generate continuously checked drawing episodes from saved real robot motions."""

import argparse
import json
from collections import Counter
from pathlib import Path
from time import monotonic

import numpy as np

from pen_art.io import read_json, sha256, write_json
from pen_art.learning_generation import (
    DrawingTeacher,
    digest,
    freeze_corpus,
    generate_from_attempts,
    obstacle_choice_examples,
    reachable_prefix_scene,
    resample,
    saved_drawings,
)
from pen_art.planning.motion_library import DrawingRequest, MotionLibrary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--runs",
        nargs="+",
        type=Path,
        default=[
            Path("runs") / name
            for name in (
                "cube_turntable_v1",
                "bottle_turntable_v1",
                "torus_turntable_v1",
                "prism_turntable_v1",
                "cube_stl_forward_turntable_v1",
                "bottle_hybrid_restart_turntable_v1",
            )
        ],
    )
    parser.add_argument(
        "--profile", type=Path, default=Path("research/planning_cell/cell_turntable.yaml")
    )
    parser.add_argument("--output", type=Path, default=Path("runs/motion_learning_v1"))
    parser.add_argument(
        "--attempts", type=Path, help="Runtime library/directory/JSONL misses to solve"
    )
    parser.add_argument("--max-attempts", type=int, default=32)
    parser.add_argument("--merge", type=Path, nargs="+", help="Freeze completed generation batches")
    parser.add_argument("--families-per-object", type=int, default=3)
    parser.add_argument("--knots", type=int, default=32)
    parser.add_argument("--seconds-per-attempt", type=float, default=20)
    parser.add_argument("--maximum-seconds", type=float, default=900)
    parser.add_argument("--obstacle-choice-seconds", type=float, default=30)
    parser.add_argument(
        "--variations",
        nargs="+",
        default=[
            "original",
            "shorter",
            "shifted",
            "longer",
            "rotated",
            "curved",
            "index1",
            "index2",
            "index3",
        ],
    )
    args = parser.parse_args()
    if args.attempts:
        print(
            json.dumps(
                generate_from_attempts(
                    args.attempts,
                    args.output,
                    args.profile,
                    count=args.knots,
                    maximum=args.max_attempts,
                    seconds=args.seconds_per_attempt,
                )
            ),
            flush=True,
        )
        return
    if args.merge:
        print(json.dumps(freeze_corpus(args.merge, args.output)), flush=True)
        return
    args.output.mkdir(parents=True, exist_ok=True)
    source_hashes_at_start = {
        "generator": sha256(Path(__file__)),
        "validator": sha256(Path("src/pen_art/learning_generation.py")),
    }
    teacher = DrawingTeacher(args.profile)
    library = MotionLibrary(args.output / "motion_library")
    started = monotonic()
    counts = Counter()
    records = []

    def emit(row):
        records.append(row)
        if row["validation"]["status"] == "VERIFIED_VALID":
            request = DrawingRequest(
                **{
                    k: v
                    for k, v in row["request"].items()
                    if k in DrawingRequest.__dataclass_fields__
                }
            )
            library.remember(
                request,
                {
                    "positions": row["trajectory"]["q_path"],
                    "tcp_targets_m": row["trajectory"]["source_points_m"],
                    "tcp_rotation": row["request"]["allowed_orientations"][0],
                },
                row["validation"],
                row["scene"],
                row["provenance"],
            )
        counts[row["validation"]["status"]] += 1
        with (args.output / "attempts.jsonl").open("a") as handle:
            handle.write(json.dumps(row, separators=(",", ":")) + "\n")
        progress = {
            "status": "RUNNING",
            "records": len(records),
            "counts": dict(counts),
            "seconds": monotonic() - started,
            "output": str(args.output.resolve()),
        }
        write_json(Path("/tmp/3d_art_dataset_progress.json"), progress)
        print(
            json.dumps(
                {
                    "id": row["id"],
                    "status": row["validation"]["status"],
                    "variant": row["provenance"]["variation"],
                    "method": row["provenance"]["method"],
                }
            ),
            flush=True,
        )

    # A new output is deliberate; never silently mix changed validator contexts.
    if (args.output / "attempts.jsonl").exists():
        raise FileExistsError("Choose a new --output directory to preserve the previous dataset")

    def object_name(path):
        from pen_art.planning.motion_runtime import object_identity

        metadata_path = path / "run.json"
        metadata = read_json(metadata_path) if metadata_path.exists() else {}
        return object_identity(path, metadata)

    object_order = sorted({object_name(path) for path in args.runs})
    if len(object_order) < 3:
        raise ValueError("Use at least three source objects for held-out evaluation")
    split_manifest = {
        name: (
            "test"
            if i == len(object_order) - 1
            else "validation"
            if i == len(object_order) - 2
            else "train"
        )
        for i, name in enumerate(object_order)
    }
    write_json(args.output / "split_manifest.json", split_manifest)
    for run in args.runs:
        source_object = object_name(run)
        graph = read_json(run / "stroke_graph.json")
        source_method = graph.get("source_method", graph.get("method", "unknown"))
        construction = read_json(run / "construction_plan.json")
        source_stages = {row["id"]: row.get("stage", "unknown") for row in construction["actions"]}
        # Group assignment occurs before any variant or scene is generated.
        split = split_manifest[source_object]
        selected = []
        seen = set()
        for path, saved, points, joints in saved_drawings(run):
            family = saved["drawing"]["stroke_id"]
            if family not in seen:
                seen.add(family)
                selected.append((path, saved, points, joints))
            if len(selected) >= args.families_per_object:
                break
        for path, saved, native_points, native_joints in selected:
            points, joints = resample(native_points, native_joints, args.knots)
            drawing = saved["drawing"]
            family = f"{run.name}:{drawing['stroke_id']}"
            for variation in args.variations:
                if monotonic() - started >= args.maximum_seconds:
                    break
                target = points.copy()
                reference = native_points.copy()
                index = drawing.get("table_index", 0)
                rotation = np.asarray(drawing["tcp_rotation"])
                if variation.startswith("index"):
                    index = int(variation[-1])
                    transform = teacher.table.build_transform(index) @ np.linalg.inv(
                        teacher.table.build_transform(drawing.get("table_index", 0))
                    )
                    target = target @ transform[:3, :3].T + transform[:3, 3]
                    reference = reference @ transform[:3, :3].T + transform[:3, 3]
                    rotation = transform[:3, :3] @ rotation
                elif variation == "shorter":
                    target = points[0] + 0.85 * (points - points[0])
                    reference = points[0] + 0.85 * (native_points - points[0])
                elif variation == "longer":
                    target = points[0] + 1.08 * (points - points[0])
                    reference = points[0] + 1.08 * (native_points - points[0])
                elif variation == "shifted":
                    target += np.array([0.0005, 0.0003, 0.0004])
                    reference += np.array([0.0005, 0.0003, 0.0004])
                elif variation == "curved":
                    target[:, 2] += 0.0005 * np.sin(np.linspace(0, np.pi, len(target)))
                    arc = np.r_[
                        0, np.cumsum(np.linalg.norm(np.diff(native_points, axis=0), axis=1))
                    ]
                    reference[:, 2] += 0.0005 * np.sin(np.pi * arc / arc[-1])
                elif variation == "rotated":
                    angle = np.deg2rad(1)
                    rot = np.array(
                        [
                            [np.cos(angle), -np.sin(angle), 0],
                            [np.sin(angle), np.cos(angle), 0],
                            [0, 0, 1],
                        ]
                    )
                    target = points[0] + (points - points[0]) @ rot.T
                    reference = points[0] + (native_points - points[0]) @ rot.T
                elif variation != "original":
                    raise ValueError(f"Unknown variation: {variation}")
                request = {
                    "source_points_m": target.tolist(),
                    "table_index": index,
                    "source_reference_points_m": reference.tolist(),
                    "context_hash": teacher.context,
                    "q_start": joints[0].tolist()
                    if variation in ("original", "shorter", "longer", "rotated", "curved")
                    else None,
                    "q_goal": None,
                    "allowed_orientations": [rotation.tolist()],
                    "source_object": source_object,
                    "family_id": family,
                    "physical_length_m": float(
                        np.linalg.norm(np.diff(target, axis=0), axis=1).sum()
                    ),
                }
                deadline = min(
                    started + args.maximum_seconds, monotonic() + args.seconds_per_attempt
                )
                method = "cached_replay"
                candidate = joints.tolist()
                if variation != "original":
                    query = DrawingRequest(
                        **{
                            k: v
                            for k, v in request.items()
                            if k in DrawingRequest.__dataclass_fields__
                        }
                    )
                    candidate = None
                    for record in library.retrieve(query, limit=3):
                        adapted = library.adapt(
                            record, query, teacher.arm, teacher.obstacles, deadline=deadline
                        )
                        if adapted is not None:
                            fitted_points, fitted_q = resample(
                                adapted["tcp_targets_m"], adapted["positions"], args.knots
                            )
                            # Fitted q is evaluated at the request's equally spaced progress.
                            candidate = teacher.adapt(request, fitted_q, deadline=deadline)
                            method = "motion_library_adaptation"
                            if candidate is not None:
                                break
                    if candidate is None:
                        method = "cached_seed_adaptation"
                        candidate = teacher.adapt(request, joints, deadline=deadline)
                    if candidate is not None and request["q_start"] is not None:
                        candidate[0] = request["q_start"]
                if candidate is None:
                    method = "teacher_layered_ik"
                    candidate, search = teacher.plan(
                        request, [joints[0], teacher.arm.initial_positions], deadline=deadline
                    )
                provenance = {
                    "profile": str(args.profile.resolve()),
                    "source_run": str(run.resolve()),
                    "source_witness": str(path.resolve()),
                    "source_sha256": sha256(path),
                    "variation": variation,
                    "method": method,
                    "split_assignment": "source_object_before_variants",
                    "source_family": family,
                    "source_type": "saved_local_motion",
                    "source_method": source_method,
                    "source_stage": source_stages.get(drawing["stroke_id"], "unknown"),
                    "source_graph_sha256": sha256(run / "stroke_graph.json"),
                    "teacher_seconds": args.seconds_per_attempt,
                    "simulation_only": True,
                }
                if candidate is None:
                    emit(
                        {
                            "schema_version": 1,
                            "id": digest([family, variation, "unresolved"]),
                            "split": split,
                            "request": request,
                            "scene": {"capsules": [], "kind": "synthetic_empty"},
                            "trajectory": None,
                            "validation": {
                                "status": "UNRESOLVED",
                                "reason": "teacher_no_path_found",
                                "validator": "continuous_local_draw_v1",
                                "context_hash": teacher.context,
                            },
                            "provenance": provenance,
                        }
                    )
                    continue
                if request["q_start"] is None:
                    request["q_start"] = list(candidate[0])
                # Empty and explicitly synthetic obstructed scenes share exactly one motion.
                # The latter supplies candidate-specific negatives, never infeasible-request labels.
                middle = teacher.arm.robot.forward(
                    candidate[len(candidate) // 2], teacher.arm.flange_to_tcp
                )
                blocked = middle[:3, 3] - 0.02 * middle[:3, 2]
                scenes = [
                    ("synthetic_empty", []),
                    (
                        "synthetic_tool_obstruction",
                        [
                            {
                                "start": (blocked - np.array([0.015, 0, 0])).tolist(),
                                "end": (blocked + np.array([0.015, 0, 0])).tolist(),
                                "radius_m": 0.003,
                                "name": "synthetic-crossing",
                            }
                        ],
                    ),
                ]
                scenes = [
                    {"capsules": capsules, "kind": kind, "reachable_build_prefix": False}
                    for kind, capsules in scenes
                ]
                if variation == "original":
                    prefix = reachable_prefix_scene(teacher, run, drawing["stroke_id"], index)
                    if prefix is not None:
                        scenes.append(prefix)
                for scene in scenes:
                    kind = scene["kind"]
                    validation = teacher.validate(request, candidate, scene, deadline=deadline)
                    emit(
                        {
                            "schema_version": 1,
                            "id": digest([family, variation, kind]),
                            "split": split,
                            "request": request,
                            "scene": scene,
                            "trajectory": {
                                "q_path": candidate,
                                "source_points_m": target.tolist(),
                                "representation": "piecewise_linear_joint_v1",
                            },
                            "validation": validation,
                            "provenance": provenance,
                        }
                    )
                if variation == "original" and args.obstacle_choice_seconds > 0:
                    choice_deadline = min(
                        started + args.maximum_seconds, monotonic() + args.obstacle_choice_seconds
                    )
                    choices = obstacle_choice_examples(
                        teacher, request, candidate, library, deadline=choice_deadline
                    )
                    for choice_index, (
                        choice_request,
                        choice_scene,
                        choice_q,
                        choice_validation,
                        choice_method,
                    ) in enumerate(choices):
                        emit(
                            {
                                "schema_version": 1,
                                "id": digest([family, "posture_choice", choice_index]),
                                "split": split,
                                "request": choice_request,
                                "scene": choice_scene,
                                "trajectory": {
                                    "q_path": np.asarray(choice_q).tolist(),
                                    "source_points_m": target.tolist(),
                                    "representation": "piecewise_linear_joint_v1",
                                },
                                "validation": choice_validation,
                                "provenance": {
                                    **provenance,
                                    "variation": "posture_choice",
                                    "method": choice_method,
                                },
                            }
                        )
    summary = {
        "schema_version": 1,
        "status": "COMPLETE",
        "records": len(records),
        "counts": dict(counts),
        "splits": dict(Counter(r["split"] for r in records)),
        "seconds": monotonic() - started,
        "context_hash": teacher.context,
        "profile": str(args.profile.resolve()),
        "generator_sha256": source_hashes_at_start["generator"],
        "validation_sha256": source_hashes_at_start["validator"],
        "scope": (
            "Local continuously verified DRAW examples; complete build/transit/process "
            "acceptance remains independent"
        ),
    }
    write_json(args.output / "generation_receipt.json", summary)
    write_json(Path("/tmp/3d_art_dataset_progress.json"), summary)
    print(json.dumps(summary), flush=True)


if __name__ == "__main__":
    main()
