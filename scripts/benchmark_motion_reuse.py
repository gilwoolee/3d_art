"""Equal-budget, independently verified teacher/cache/diffusion drawing comparison."""

import argparse
import json
from collections import Counter
from pathlib import Path
from time import monotonic

import numpy as np

from pen_art.io import sha256, write_json
from pen_art.learning_generation import DrawingTeacher, resample
from pen_art.planning.motion_library import (
    CachedCollisionModel,
    DrawingRequest,
    MotionLibrary,
)
from pen_art.planning.motion_runtime import static_shapes


def request_object(request):
    return DrawingRequest(
        **{
            key: value
            for key, value in request.items()
            if key in DrawingRequest.__dataclass_fields__
        }
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attempts", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--curated", type=Path, help="Frozen curated dataset used by checkpoint")
    parser.add_argument(
        "--profile", type=Path, default=Path("research/planning_cell/cell_turntable.yaml")
    )
    parser.add_argument("--output", type=Path, default=Path("runs/motion_reuse_benchmark"))
    parser.add_argument("--split", choices=["validation", "test"], default="test")
    parser.add_argument("--seconds-per-request", type=float, default=15)
    parser.add_argument("--limit", type=int, default=6)
    parser.add_argument(
        "--exclude-completed",
        type=Path,
        help="Evaluate every remaining frozen request, excluding IDs in a prior report",
    )
    parser.add_argument(
        "--include-unresolved",
        action="store_true",
        help="Also evaluate raw held-out unresolved requests, separately labeled",
    )
    parser.add_argument("--table-index", type=int, choices=(0, 1, 2, 3))
    args = parser.parse_args()
    if args.output.exists() and any(args.output.iterdir()):
        raise FileExistsError("Use an empty output directory for independent cold caches")
    args.output.mkdir(parents=True, exist_ok=True)
    core_hashes = {
        str(path): sha256(path)
        for path in [
            Path(__file__),
            Path("src/pen_art/learning_generation.py"),
            Path("src/pen_art/planning/motion_library.py"),
            Path("src/pen_art/planning/collision.py"),
            Path("src/pen_art/planning/smooth_audit.py"),
            Path("src/pen_art/learning/diffusion.py"),
        ]
    }
    rows = [json.loads(line) for line in args.attempts.read_text().splitlines() if line]
    frozen = None
    if args.curated:
        from pen_art.learning.data import digest as episode_digest

        frozen = json.loads((args.curated / "manifest.json").read_text())
        split_by_id = {
            identity: split for split, ids in frozen["split_members"].items() for identity in ids
        }
        selected = []
        for row in rows:
            identity = episode_digest(row)
            if identity in split_by_id:
                if row["split"] != split_by_id[identity]:
                    raise ValueError("Raw and frozen curated split assignments differ")
                selected.append(row)
            elif args.include_unresolved and row["validation"]["status"] == "UNRESOLVED":
                selected.append(row)
        rows = selected
    elif args.checkpoint:
        raise ValueError("--curated is required to bind benchmark to the trained checkpoint")
    training = [
        row
        for row in rows
        if row["split"] == "train" and row["validation"]["status"] == "VERIFIED_VALID"
    ]
    pool = [
        row
        for row in rows
        if row["split"] == args.split
        and (
            row["validation"]["status"] == "VERIFIED_VALID"
            or (args.include_unresolved and row["validation"]["status"] == "UNRESOLVED")
        )
        and (args.table_index is None or row["request"]["table_index"] == args.table_index)
    ]
    priorities = {"curved": 1, "shorter": 2, "shifted": 3, "longer": 4, "rotated": 5}
    pool.sort(
        key=lambda row: (
            0
            if row["scene"].get("kind") == "synthetic_arm_posture_choice"
            else priorities.get(row["provenance"]["variation"], 6)
        )
    )
    groups = {}
    for row in pool:
        key = row["request"]["family_id"]
        choices = groups.setdefault(key, [])
        scene_variant = (row["provenance"]["variation"], row["scene"].get("kind"))
        if scene_variant not in {
            (r["provenance"]["variation"], r["scene"].get("kind")) for r in choices
        }:
            choices.append(row)
    evaluation = []
    for rank in range(max(map(len, groups.values()), default=0)):
        for choices in groups.values():
            if rank < len(choices):
                evaluation.append(choices[rank])
    if args.exclude_completed:
        prior = json.loads(args.exclude_completed.read_text())
        if prior.get("split") != args.split:
            raise ValueError("Prior completed report has a different split")
        excluded = {row["id"] for row in prior["results"]}
        evaluation = [row for row in pool if row["id"] not in excluded]
    evaluation = evaluation[: args.limit]
    if not training or not evaluation:
        raise ValueError("Need verified train and held-out examples")
    if {r["request"]["source_object"] for r in training} & {
        r["request"]["source_object"] for r in evaluation
    }:
        raise ValueError("Source-object split leakage")
    library = MotionLibrary(args.output / "library")
    for row in training:
        library.remember(
            request_object(row["request"]),
            {
                "positions": row["trajectory"]["q_path"],
                "tcp_targets_m": row["trajectory"]["source_points_m"],
                "tcp_rotation": row["request"]["allowed_orientations"][0],
            },
            row["validation"],
            row["scene"],
            row["provenance"],
        )
    plain = DrawingTeacher(args.profile)
    verifiers = {
        mode: DrawingTeacher(args.profile) for mode in ("teacher", "cache", "cache_diffusion")
    }
    pool = np.asarray([row["trajectory"]["q_path"][0] for row in training])
    teacher_seeds = [plain.arm.initial_positions]
    while len(teacher_seeds) < 4:
        distances = np.min(
            np.stack([np.linalg.norm(pool - seed, axis=1) for seed in teacher_seeds]), axis=0
        )
        selected = int(np.argmax(distances))
        if distances[selected] < 1e-3:
            break
        teacher_seeds.append(pool[selected])
    cached = {}
    for mode in ("cache", "cache_diffusion"):
        teacher = DrawingTeacher(args.profile)
        teacher.arm.collision = CachedCollisionModel(
            teacher.arm.collision,
            args.output / f"clearance_{mode}",
            geometry_context=teacher.arm.tool.metadata,
            static_geometry=static_shapes(teacher.profile, teacher.table),
        )
        cached[mode] = teacher
    provider = None
    if args.checkpoint:
        from pen_art.learning.diffusion import MotionDiffusionProvider

        provider = MotionDiffusionProvider(
            args.checkpoint,
            plain.context,
            signature_fn=lambda episode, probes: cached["cache_diffusion"].signature(
                probes, episode["scene"]
            ),
        )
    if provider and frozen["dataset_hash"] != provider.checkpoint["dataset_hash"]:
        raise ValueError("Checkpoint and curated dataset hashes differ")
    results = []
    modes = ["teacher", "cache"] + (["cache_diffusion"] if provider else [])
    for request_index, row in enumerate(evaluation):
        for mode in modes:
            teacher = plain if mode == "teacher" else cached[mode]
            verifier = verifiers[mode]
            counters_before = {
                name: getattr(teacher.arm.collision, name, 0)
                for name in ("hits", "misses", "self_static_seconds", "material_seconds")
            }
            start = monotonic()
            deadline = start + args.seconds_per_request
            phase_times = Counter()
            reasons = Counter()
            source = None
            neural_stats = {}
            generated_count = 0
            planner_diagnostic = None
            request = row["request"]
            query = request_object(request)

            def verify_candidate(
                candidate,
                points,
                candidate_source,
                *,
                deadline=deadline,
                request=request,
                row=row,
                phase_times=phase_times,
                reasons=reasons,
                verifier=verifier,
            ):
                nonlocal source
                if candidate is None or monotonic() >= deadline:
                    return False
                q = np.asarray(candidate)
                target = np.asarray(points)
                if len(q) != len(request["source_points_m"]):
                    target, q = resample(target, q, len(request["source_points_m"]))
                # Same exact request, including original source geometry/endpoints.
                mark = monotonic()
                verdict = verifier.validate(request, q, row["scene"], deadline=deadline)
                phase_times["independent_validation"] += monotonic() - mark
                reasons[verdict["status"]] += 1
                if verdict["status"] == "VERIFIED_VALID":
                    source = candidate_source
                    return True
                return False

            if mode != "teacher":
                mark = monotonic()
                candidates = library.retrieve(query, limit=3)
                phase_times["retrieval"] += monotonic() - mark
                for record in candidates:
                    mark = monotonic()
                    proposal = library.adapt(
                        record,
                        query,
                        teacher.arm,
                        teacher.obstacles,
                        deadline=min(deadline, monotonic() + 3),
                    )
                    phase_times["cache_adaptation"] += monotonic() - mark
                    if proposal and verify_candidate(
                        proposal["positions"], proposal["tcp_targets_m"], "cache"
                    ):
                        break
                    if monotonic() >= start + args.seconds_per_request * 0.5:
                        break
            if source is None and mode == "cache_diffusion" and monotonic() < deadline:
                mark = monotonic()
                proposals = provider.propose(query, row["scene"], count=3, steps=20)
                neural_stats = dict(provider.stats)
                generated_count = len(proposals)
                phase_times["diffusion_with_encoding"] += monotonic() - mark
                for proposal in proposals:
                    mark = monotonic()
                    refined = teacher.adapt(
                        request, proposal.q_path, deadline=min(deadline, monotonic() + 3)
                    )
                    phase_times["diffusion_refinement"] += monotonic() - mark
                    if verify_candidate(refined, request["source_points_m"], "diffusion"):
                        break
                    if monotonic() >= start + args.seconds_per_request * 0.85:
                        break
            if source is None and monotonic() < deadline:
                mark = monotonic()
                # The same farthest-point training-only posture seeds serve
                # every mode, including the uncached teacher baseline.
                seeds = list(teacher_seeds)
                if request.get("q_start") is not None:
                    seeds.insert(0, request["q_start"])
                candidate, diagnostic = teacher.plan(
                    request, seeds, deadline=deadline, scene=row["scene"]
                )
                planner_diagnostic = {
                    key: diagnostic[key] for key in ("status", "failed_sample", "retained_states")
                    if key in diagnostic
                }
                phase_times["teacher_fallback"] += monotonic() - mark
                verify_candidate(candidate, request["source_points_m"], "teacher")
            result = {
                "id": row["id"],
                "mode": mode,
                "cache_state": "cold" if request_index == 0 else "warm",
                "source_object": row["request"]["source_object"],
                "family_id": row["request"]["family_id"],
                "variation": row["provenance"]["variation"],
                "scene_kind": row["scene"].get("kind"),
                "table_index": row["request"]["table_index"],
                "ground_truth_status": row["validation"]["status"],
                "generated_count": generated_count,
                "diffusion_stats": neural_stats,
                "verified": source is not None,
                "accepted_source": source,
                "seconds": monotonic() - start,
                "phase_seconds": dict(phase_times),
                "rejections": dict(reasons),
                "seconds_budget": args.seconds_per_request,
                "planner_diagnostic": planner_diagnostic,
                "cache_hits": getattr(teacher.arm.collision, "hits", 0) - counters_before["hits"],
                "cache_misses": getattr(teacher.arm.collision, "misses", 0)
                - counters_before["misses"],
                "self_static_seconds": getattr(teacher.arm.collision, "self_static_seconds", 0)
                - counters_before["self_static_seconds"],
                "material_seconds": getattr(teacher.arm.collision, "material_seconds", 0)
                - counters_before["material_seconds"],
            }
            results.append(result)
            print(json.dumps(result), flush=True)
            write_json(args.output / "progress.json", {"status": "RUNNING", "results": results})
    report = {
        "schema_version": 1,
        "status": "COMPLETE",
        "core_source_hashes": core_hashes,
        "context_hash": plain.context,
        "scope": "local DRAW only; independent full tracking and progressive material validation",
        "source_sha256": sha256(args.attempts),
        "checkpoint_sha256": sha256(args.checkpoint) if args.checkpoint else None,
        "split": args.split,
        "excluded_report_sha256": sha256(args.exclude_completed)
        if args.exclude_completed
        else None,
        "training_records": len(training),
        "requests": len(evaluation),
        "known_positive_requests": sum(
            r["validation"]["status"] == "VERIFIED_VALID" for r in evaluation
        ),
        "unresolved_input_requests": sum(
            r["validation"]["status"] == "UNRESOLVED" for r in evaluation
        ),
        "seconds_budget": args.seconds_per_request,
        "teacher_initialization": "home plus three farthest training-only start postures",
        "teacher_seeds": [np.asarray(seed).tolist() for seed in teacher_seeds],
        "results": results,
        "summary": {
            mode: {
                "verified": sum(r["verified"] for r in results if r["mode"] == mode),
                "median_seconds": float(
                    np.median([r["seconds"] for r in results if r["mode"] == mode])
                ),
                "cold_seconds": next(r["seconds"] for r in results if r["mode"] == mode),
                "warm_median_seconds": float(
                    np.median([r["seconds"] for r in results if r["mode"] == mode][1:])
                )
                if len(evaluation) > 1
                else None,
            }
            for mode in modes
        },
    }
    for teacher in cached.values():
        teacher.arm.collision.flush()
    report["persistent_cache_bytes"] = {
        mode: sum(
            path.stat().st_size
            for path in (args.output / f"clearance_{mode}").rglob("*")
            if path.is_file()
        )
        for mode in cached
    }
    report["library_bytes"] = sum(
        path.stat().st_size for path in (args.output / "library").rglob("*") if path.is_file()
    )
    write_json(args.output / "benchmark.json", report)
    print(json.dumps(report["summary"]), flush=True)


if __name__ == "__main__":
    main()
