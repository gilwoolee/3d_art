"""Bounded real local-DRAW comparison; no complete construction acceptance.

All proposal sources use the same verifier and profile. Raw, cold accelerated,
and warm accelerated verification must agree. Learned predictions are proposals,
including when rejected; a forced-empty trajectory library makes their source
unambiguous. Timings include generation/refinement/verification separately.
"""

import argparse
from copy import deepcopy
from pathlib import Path
from time import monotonic

import numpy as np

from pen_art.io import read_json, sha256, write_json
from pen_art.learning.data import curate
from pen_art.learning_generation import DrawingTeacher, resample
from pen_art.planning.motion_library import DrawingRequest, MotionLibrary


def query(request):
    return DrawingRequest(
        **{k: v for k, v in request.items() if k in DrawingRequest.__dataclass_fields__}
    )


def verify(teacher, request, positions, scene, seconds):
    """Record the actual verdict, never map an unresolved check to collision."""
    before = teacher.verification_metrics()
    started = monotonic()
    result = teacher.validate(request, positions, scene, deadline=started + seconds)
    return {
        "seconds": monotonic() - started,
        "verdict": result,
        "metrics": teacher.verification_metrics(),
        "metrics_delta": metric_difference(teacher.verification_metrics(), before),
    }


def metric_difference(after, before):
    return {
        key: metric_difference(value, before.get(key, {}))
        if isinstance(value, dict)
        else value - before.get(key, 0)
        if isinstance(value, (float, int)) and not isinstance(value, bool)
        else value
        for key, value in after.items()
    }


def verify_drawing(teacher, request, drawing, scene, seconds):
    actual = {
        **request,
        "source_reference_points_m": request["source_points_m"],
        "source_points_m": drawing["tcp_targets_m"],
    }
    return verify(teacher, actual, drawing["positions"], scene, seconds)


def equivalent(results):
    statuses = {result["verdict"]["status"] for result in results}
    return len(statuses) == 1


def compact_source(points, joints=None, *, length_m=0.012, knots=8):
    """Keep the actual beginning of a real source, including its curvature."""
    points = np.asarray(points, float)
    arc = np.r_[0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    keep = np.r_[True, np.diff(arc) > 1e-12]
    grid = np.linspace(0, min(length_m, arc[-1]), knots)
    source = np.column_stack([np.interp(grid, arc[keep], v[keep]) for v in points.T])
    q = (
        None
        if joints is None
        else np.column_stack([np.interp(grid, arc[keep], v[keep]) for v in np.asarray(joints).T])
    )
    return source, q


def prepare_cases(profile, output, seconds=45):
    """Bind three real object sources to one current cell/model context."""
    from pen_art.curves import flatten_curve, transform_points

    started = monotonic()
    teacher = DrawingTeacher(profile, verification_raw=True)
    cases, seeds = [], []
    rotation = None
    for name in ("cube", "bottle"):
        witness = Path(f"runs/{name}_nozzle27_base_check_v1/accepted_witness.json")
        draw = read_json(witness)["drawing"]
        rotation = draw["tcp_rotation"]
        source, seed = compact_source(draw["tcp_targets_m"], draw["positions"])
        request = {
            "source_points_m": source.tolist(),
            "table_index": 0,
            "fixed_tcp_rotation": rotation,
            "allowed_orientations": [rotation],
            "source_object": name,
            "family_id": name,
            "context_hash": teacher.context,
        }
        adapted = teacher.adapt(request, seed, deadline=monotonic() + seconds)
        if adapted is None:
            raise RuntimeError(f"{name} compact source unresolved under common profile")
        cases.append(
            {
                "source_object": name,
                "request": request,
                "q_path": np.asarray(adapted).tolist(),
                "inputs": {str(witness): sha256(witness)},
            }
        )
        seeds.append(adapted[0])
    torus = Path("runs/general_torus_sparse/geometric_candidates.json")
    curves = read_json(torus)["curves"]
    curve = next(row for row in curves if row.get("axis") == 2)
    points, _ = flatten_curve(curve)
    points = transform_points(points, teacher.table.build_transform(0))
    source, _ = compact_source(points)
    request = {
        "source_points_m": source.tolist(),
        "table_index": 0,
        "fixed_tcp_rotation": rotation,
        "allowed_orientations": [rotation],
        "source_object": "torus",
        "family_id": "torus",
        "context_hash": teacher.context,
    }
    q, result = teacher.plan(request, seeds, deadline=monotonic() + seconds)
    if q is None:
        write_json(output.with_suffix(".unresolved.json"), result)
        raise RuntimeError("Real torus compact source unresolved; preserved diagnostic")
    cases.append(
        {
            "source_object": "torus",
            "request": request,
            "q_path": q,
            "inputs": {str(torus): sha256(torus)},
        }
    )
    write_json(
        output,
        {
            "scope": "real short object contours, common current profile",
            "profile_sha256": sha256(profile),
            "preparation_seconds": monotonic() - started,
            "cases": cases,
        },
    )
    return cases


def forced_miss(library, request):
    proposals = library.retrieve(query(request), limit=1, compatible_only=True)
    if proposals:
        raise ValueError("Phase 2 must use an empty trajectory library")
    return {"library_records": len(library.records), "retrieved": len(proposals)}


def evaluate_generated(teacher, request, scene, proposal, seconds, *, raw_teacher=None):
    """Refine this neural sample, then verify; no planner fallback or seed swap."""
    started = monotonic()
    target, seed = resample(
        proposal.source_points_m, proposal.q_path, len(request["source_points_m"])
    )
    # Keep the original request/source; generated source only indexes its seed.
    del target
    mark = monotonic()
    refined = teacher.adapt(request, seed, deadline=started + seconds)
    refinement = monotonic() - mark
    if refined is None:
        return {
            "status": "UNRESOLVED",
            "reason": "refinement_budget_or_no_solution",
            "refinement_seconds": refinement,
            "verification_seconds": 0.0,
            "seconds": monotonic() - started,
        }
    checked = verify(teacher, request, refined, scene, max(0, seconds - refinement))
    result = {
        "status": checked["verdict"]["status"],
        "refinement_seconds": refinement,
        "verification_seconds": checked["seconds"],
        "seconds": monotonic() - started,
        "verification": checked,
    }
    if raw_teacher is not None:
        independent = verify(raw_teacher, request, refined, scene, seconds)
        result.update(raw_verification=independent, equivalent=equivalent([checked, independent]))
        if not result["equivalent"]:
            raise AssertionError("Generated proposal raw/accelerated verdicts disagree")
    return result


def blocked_scene(teacher, joints):
    """A real material capsule through a tool body, not an invented label."""
    q = np.asarray(joints[len(joints) // 2])
    body = teacher.arm.tool.bodies[-1]
    transform = teacher.arm.robot.frames(q)[body.link] @ body.origin
    point = transform[:3, 3]
    return {
        "kind": "synthetic_material_through_tool_body",
        "reachable_build_prefix": False,
        "capsules": [
            {
                "start": (point - [0.01, 0, 0]).tolist(),
                "end": (point + [0.01, 0, 0]).tolist(),
                "radius_m": 0.01,
                "name": "explicit-blocking-material",
            }
        ],
    }


def runtime_probe(cases_path, profile, output, checkpoint, *, seconds=45):
    """Exercise the production provider/refinement wiring without a build search."""
    from pen_art.curves import transform_points
    from pen_art.planning.motion_runtime import DrawingExperience, ProposalBudget
    from pen_art.planning.scene import MaterialScene
    from pen_art.planning.section_build import SectionBuild

    if output.exists() and any(output.iterdir()):
        raise FileExistsError("Use an empty output for the forced runtime cache miss")
    output.mkdir(parents=True, exist_ok=True)
    cases = read_json(cases_path)["cases"]
    raw = DrawingTeacher(profile, verification_raw=True)
    results = []
    for case in cases:
        identity = case["source_object"]
        private = output / identity
        private.mkdir()
        # Construct the same real planner/model/tool context as production. Only
        # its list of actions is reduced to the declared local source extraction.
        build = SectionBuild(Path("runs/bottle_nozzle27_full_v1"), profile, seconds)
        build.run = private
        build.motion_library_path = private / "empty_library"
        build.diffusion_checkpoint = checkpoint
        build.learning_device = "cpu"
        write_json(private / "run.json", {"cell_source_run": f"runs/{identity}_benchmark_source"})
        points = np.asarray(case["request"]["source_points_m"])
        local = transform_points(points, np.linalg.inv(build.planner.turntable.build_transform(0)))
        length = float(np.linalg.norm(np.diff(local, axis=0), axis=1).sum())
        row = {
            "id": f"{identity}-local",
            "closed": False,
            "start": "start",
            "end": "end",
            "parent_curve": "contour-real-source-extraction",
            "members": [],
            "length_m": length,
            "diameter_m": 0.001,
            "stage": "framework",
            "junction_visits": [
                {"node": "start", "distance_m": 0, "segment_boundary": 0},
                {"node": "end", "distance_m": length, "segment_boundary": len(points) - 1},
            ],
            "segments": [
                [a.tolist(), (a + (b - a) / 3).tolist(), (a + 2 * (b - a) / 3).tolist(), b.tolist()]
                for a, b in zip(local[:-1], local[1:], strict=True)
            ],
        }
        build.actions = {row["id"]: row}
        build.planner.actions = build.actions
        experience = DrawingExperience(build, build.planner.turntable)
        if experience.learning_context_hash != raw.context:
            raise ValueError("Runtime and independent teacher physical contexts differ")
        if experience.library.records:
            raise AssertionError("Runtime forced-miss library is not empty")
        checked = []
        for label in ("clear", "blocked"):
            material = MaterialScene()
            material.table_index = 0
            scene_record = {"capsules": []}
            if label == "blocked":
                scene_record = blocked_scene(raw, case["q_path"])
                for capsule in scene_record["capsules"]:
                    material.add(
                        capsule["start"], capsule["end"], capsule["radius_m"], capsule["name"]
                    )
            build.planner.deadline = monotonic() + seconds
            started = monotonic()
            iterator = experience.candidates(
                row, {"start"}, 0, material, budget=ProposalBudget(seconds, 4)
            )
            drawing = next(iterator, None)
            elapsed = monotonic() - started
            if drawing is None:
                checked.append(
                    {
                        "scene": label,
                        "status": "UNRESOLVED",
                        "candidate_seconds": elapsed,
                        "report": experience.report(),
                    }
                )
                continue
            if drawing["proposal_source"] != "diffusion":
                raise AssertionError("Forced miss yielded a non-neural proposal")
            request = drawing["motion_request"]
            # Runtime carries a stricter timing/process binding. Independent
            # checking uses the same physical geometry, targets, and fixed R.
            request = {
                **request,
                "context_hash": raw.context,
                "fixed_tcp_rotation": drawing["tcp_rotation"],
                "allowed_orientations": [drawing["tcp_rotation"]],
            }
            verdict = verify_drawing(raw, request, drawing, scene_record, seconds)
            write_json(private / f"{label}_drawing.json", drawing)
            checked.append(
                {
                    "scene": label,
                    "status": verdict["verdict"]["status"],
                    "candidate_seconds": elapsed,
                    "verification": verdict,
                    "total_seconds": elapsed + verdict["seconds"],
                    "proposal_source": drawing["proposal_source"],
                    "fixed_rotation": drawing["tcp_rotation"],
                    "report": experience.report(),
                }
            )
        experience.flush()
        results.append(
            {
                "source_object": identity,
                "library_records": len(experience.library.records),
                "results": checked,
            }
        )
        write_json(output / "progress.json", {"status": "RUNNING", "results": results})
    inputs = {
        str(p): sha256(p)
        for p in [
            cases_path,
            profile,
            checkpoint,
            Path(__file__),
            Path("src/pen_art/planning/motion_runtime.py"),
            Path("src/pen_art/planning/motion_library.py"),
            Path("src/pen_art/learning_generation.py"),
        ]
    }
    report = {
        "status": "COMPLETE",
        "scope": "real production neural candidates; local DRAW only",
        "no_fallback": True,
        "inputs": inputs,
        "context_hash": raw.context,
        "results": results,
    }
    write_json(output / "runtime_benchmark.json", report)
    return report


def timed_cache_probe(output, *, seconds=90):
    """Adapt real accepted timed seeds to changed whole contour geometry."""
    from pen_art.planning.motion_library import CachedCollisionModel, digest
    from pen_art.planning.motion_runtime import model_context, static_shapes
    from pen_art.planning.section_build import SectionBuild

    if output.exists() and any(output.iterdir()):
        raise FileExistsError("Use an empty output for cold timed-seed adaptation")
    output.mkdir(parents=True, exist_ok=True)
    library = MotionLibrary(Path("runs/motion_library_phase1_upright27_v1"))
    results = []
    for identity in ("cube", "bottle"):
        profile = Path(f"runs/{identity}_nozzle27_phase1_v1/profile.yaml")
        build = SectionBuild(Path(f"runs/{identity}_nozzle27_full_v1"), profile, seconds)
        arm, planner = build.arm, build.planner
        context = model_context(
            arm, planner.profile, timing=planner.timing, process=planner.process
        )
        records = [
            r
            for r in library.records
            if r["request"]["source_object"] == identity
            and r["request"]["context_hash"] == context
            and r["provenance"].get("section") == "section-00000"
            and r["trajectory"].get("exact_clock")
        ]
        if not records:
            raise ValueError(f"No matching-context actual timed base for {identity}")
        record = records[0]
        original_hash = digest(record)
        clock_hash = digest(record["trajectory"]["exact_clock"])
        request = deepcopy(record["request"])
        points = np.asarray(request["source_points_m"])
        center = points.mean(axis=0)
        request["source_points_m"] = (center + 0.995 * (points - center)).tolist()
        request.update(q_start=None, q_goal=None)
        arm.collision = CachedCollisionModel(
            arm.collision,
            output / identity,
            geometry_context=arm.tool.metadata,
            static_geometry=static_shapes(planner.profile, planner.turntable),
        )
        raw = DrawingTeacher(profile, verification_raw=True)
        measured = []
        for label in ("cold", "warm"):
            started = monotonic()
            hits = library.retrieve(query(request), limit=4, compatible_only=True)
            if not any(digest(r) == original_hash for r in hits):
                raise AssertionError("Changed-source timed seed was not retrieved")
            proposal = library.adapt(
                record, query(request), arm, planner.obstacles, deadline=started + seconds
            )
            elapsed = monotonic() - started
            if proposal is None:
                measured.append(
                    {"mode": label, "status": "UNRESOLVED", "adaptation_seconds": elapsed}
                )
                continue
            write_json(output / f"{identity}_{label}_proposal.json", proposal)
            verdict = verify_drawing(raw, request, proposal, {"capsules": []}, seconds)
            measured.append(
                {
                    "mode": label,
                    "status": verdict["verdict"]["status"],
                    "adaptation_seconds": elapsed,
                    "verification": verdict,
                    "total_seconds": elapsed + verdict["seconds"],
                    "original_seed_points": proposal.get("original_seed_points"),
                    "adaptation_seed_points": proposal.get("adaptation_seed_points"),
                    "adapted_points": len(proposal["positions"]),
                }
            )
            if (
                digest(record) != original_hash
                or digest(record["trajectory"]["exact_clock"]) != clock_hash
            ):
                raise AssertionError("Accepted record/clock changed during seed compaction")
        arm.collision.flush()
        results.append(
            {
                "source_object": identity,
                "profile_sha256": sha256(profile),
                "record_sha256": original_hash,
                "exact_clock_sha256": clock_hash,
                "source_change": "whole closed contour scaled by0.995 aboutitscenter",
                "scope": "local whole DRAW; actual prior material/process not tested here",
                "results": measured,
            }
        )
        write_json(output / "progress.json", {"status": "RUNNING", "results": results})
    report = {
        "status": "COMPLETE",
        "results": results,
        "inputs": {
            str(p): sha256(p)
            for p in [
                Path(__file__),
                Path("src/pen_art/planning/motion_library.py"),
                Path("src/pen_art/learning_generation.py"),
            ]
        },
    }
    write_json(output / "timed_cache_benchmark.json", report)
    return report


def run(cases_path, profile, output, *, seconds=30, training_steps=128):
    if output.exists() and any(output.iterdir()):
        raise FileExistsError("Use an empty output to measure a real cold cache")
    output.mkdir(parents=True, exist_ok=True)
    inputs = {str(cases_path): sha256(cases_path), str(profile): sha256(profile)}
    for path in [
        Path(__file__),
        Path("src/pen_art/learning_generation.py"),
        Path("src/pen_art/planning/collision.py"),
        Path("src/pen_art/planning/motion_library.py"),
        Path("src/pen_art/learning/diffusion.py"),
        Path("src/pen_art/planning/scene.py"),
        Path("src/pen_art/planning/path_certificates.py"),
        Path("src/pen_art/planning/smooth_audit.py"),
    ]:
        inputs[str(path)] = sha256(path)
    source_cases = read_json(cases_path)
    if source_cases.get("profile_sha256") != sha256(profile):
        raise ValueError("Case preparation profile differs from benchmark profile")
    cases = source_cases["cases"]
    if len({case["source_object"] for case in cases}) < 3:
        raise ValueError("Need three distinct real source objects")
    raw = DrawingTeacher(profile, verification_raw=True)
    library = MotionLibrary(output / "phase1_library")
    empty = MotionLibrary(output / "phase2_empty_library")
    rows, episodes = [], []
    started = monotonic()
    for index, case in enumerate(cases):
        fast = DrawingTeacher(profile, verification_directory=output / f"verification_{index}")
        for name, expected in case["inputs"].items():
            if sha256(Path(name)) != expected:
                raise ValueError(f"Case source changed: {name}")
            inputs[name] = expected
        request = deepcopy(case["request"])
        request.update(
            source_object=case["source_object"],
            family_id=case["source_object"],
            context_hash=raw.context,
        )
        rotation = request["fixed_tcp_rotation"]
        request["allowed_orientations"] = [rotation]
        joints = case["q_path"]
        scene = case.get("scene", {"capsules": []})
        measured = [
            verify(raw, request, joints, scene, seconds),
            verify(fast, request, joints, scene, seconds),
            verify(fast, request, joints, scene, seconds),
        ]
        row = {
            "source_object": case["source_object"],
            "verification": dict(zip(("raw", "cold", "warm"), measured, strict=True)),
            "equivalent": equivalent(measured),
        }
        if not row["equivalent"]:
            raise AssertionError(f"Verifier status disagreement: {row}")
        collision_scene = blocked_scene(raw, joints)
        collision = [
            verify(raw, request, joints, collision_scene, seconds),
            verify(fast, request, joints, collision_scene, seconds),
        ]
        row["collision_control"] = {
            "raw": collision[0],
            "fast": collision[1],
            "equivalent": equivalent(collision),
        }
        if not equivalent(collision) or collision[0]["verdict"]["status"] != "COLLISION":
            raise AssertionError("Blocking-material control did not produce a collision witness")
        row["unresolved_control"] = verify(raw, request, joints, scene, 0)
        for label, controlled_scene, controlled_verdict in (
            ("blocked_material", collision_scene, collision[0]["verdict"]),
            ("zero_time_unknown", scene, row["unresolved_control"]["verdict"]),
        ):
            episodes.append(
                {
                    "schema_version": 1,
                    "request": request,
                    "scene": controlled_scene,
                    "trajectory": {"q_path": joints, "source_points_m": request["source_points_m"]},
                    "validation": controlled_verdict,
                    "split": ("train", "validation", "test")[index % 3],
                    "provenance": {
                        "inputs": case["inputs"],
                        "profile": str(profile),
                        "variation": label,
                    },
                }
            )
        if measured[0]["verdict"]["status"] == "VERIFIED_VALID":
            drawing = {
                "positions": joints,
                "tcp_targets_m": request["source_points_m"],
                "tcp_rotation": rotation,
                "fixed_orientation": True,
            }
            record = library.remember(
                query(request),
                drawing,
                measured[0]["verdict"],
                scene,
                {"source": case["inputs"]},
                arm=raw.arm,
            )
            row["phase1_seed_record"] = record
            for label, scale in [("exact", 1.0), ("adapted", 0.95)]:
                changed = deepcopy(request)
                points = np.asarray(request["source_points_m"])
                changed["source_points_m"] = (points[0] + scale * (points - points[0])).tolist()
                mark = monotonic()
                hits = library.retrieve(query(changed), limit=1, compatible_only=True)
                proposed = (
                    library.adapt(
                        hits[0], query(changed), fast.arm, fast.obstacles, deadline=mark + seconds
                    )
                    if hits
                    else None
                )
                proposal_time = monotonic() - mark
                row[f"phase1_{label}"] = {
                    "retrieved": len(hits),
                    "proposal_seconds": proposal_time,
                    "verification": verify_drawing(fast, changed, proposed, scene, seconds)
                    if proposed
                    else None,
                    "status": "PROPOSAL" if proposed else "UNRESOLVED",
                }
                if proposed:
                    verdict = row[f"phase1_{label}"]["verification"]["verdict"]
                    row[f"phase1_{label}"].update(
                        status=verdict["status"],
                        total_seconds=proposal_time
                        + row[f"phase1_{label}"]["verification"]["seconds"],
                    )
                    episodes.append(
                        {
                            "schema_version": 1,
                            "request": changed,
                            "scene": scene,
                            "trajectory": {
                                "q_path": proposed["positions"],
                                "source_points_m": proposed["tcp_targets_m"],
                            },
                            "validation": verdict,
                            "split": ("train", "validation", "test")[index % 3],
                            "provenance": {
                                "inputs": case["inputs"],
                                "profile": str(profile),
                                "variation": label,
                            },
                        }
                    )
        row["request"] = request
        row["scene"] = scene
        rows.append(row)
        write_json(output / "progress.json", {"status": "VERIFICATION_COMPLETE", "results": rows})
    episode_path = output / "episodes.jsonl"
    import json

    episode_path.write_text("".join(json.dumps(e) + "\n" for e in episodes))

    def fitted(episode, q, source):
        req = {**episode["request"], "source_points_m": source.tolist()}
        return (
            verify(raw, req, q, episode["scene"], seconds)["verdict"]["status"] == "VERIFIED_VALID"
        )

    def signature(episode, probes):
        return fast.signature(probes, episode["scene"])

    from pen_art.learning.diffusion import MotionDiffusionProvider, train

    dataset = output / "dataset"
    manifest = curate(
        [episode_path], dataset, knots=8, probes=4, signature_fn=signature, fit_validator=fitted
    )
    checkpoint = output / "diffusion.pt"
    training = train(dataset, checkpoint, steps=training_steps, hidden=32, batch_size=4)
    provider = MotionDiffusionProvider(checkpoint, raw.context, signature_fn=signature)
    for row in rows:
        req, scene = row["request"], row["scene"]
        miss = forced_miss(empty, req)
        mark = monotonic()
        predictions = provider.propose(query(req), scene, count=2, steps=8)
        if not predictions:
            raise AssertionError("No finite learned proposals were generated")
        generation_seconds = monotonic() - mark
        row["phase2"] = {
            "forced_miss": miss,
            "generation_seconds": generation_seconds,
            "provider_stats": dict(provider.stats),
            "proposal_count": len(predictions),
            "proposals": [
                evaluate_generated(fast, req, scene, p, seconds, raw_teacher=raw)
                for p in predictions
            ],
            "source": "actual diffusion predictions; no teacher fallback",
        }
        case = next(c for c in cases if c["source_object"] == row["source_object"])
        obstruction = blocked_scene(raw, case["q_path"])
        mark = monotonic()
        blocked_predictions = provider.propose(query(req), obstruction, count=1, steps=8)
        if not blocked_predictions:
            raise AssertionError("No finite blocked-scene learned proposal was generated")
        row["phase2"]["blocked_generation_seconds"] = monotonic() - mark
        row["phase2"]["blocked_proposals"] = [
            evaluate_generated(fast, req, obstruction, p, seconds, raw_teacher=raw)
            for p in blocked_predictions
        ]
        if any(p["status"] != "COLLISION" for p in row["phase2"]["blocked_proposals"]):
            raise AssertionError("Blocked learned proposal lacks a collision witness")
    report = {
        "status": "COMPLETE",
        "scope": "short local DRAW only; no fullbuild acceptance",
        "seconds": monotonic() - started,
        "inputs": inputs,
        "context_hash": raw.context,
        "profile": str(profile),
        "source_preparation_seconds": source_cases.get("preparation_seconds"),
        "training": training,
        "dataset_hash": manifest["dataset_hash"],
        "checkpoint_sha256": sha256(checkpoint),
        "results": rows,
    }
    write_json(output / "benchmark.json", report)
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--training-steps", type=int, default=128)
    parser.add_argument("--prepare-cases", action="store_true")
    parser.add_argument("--runtime-checkpoint", type=Path)
    parser.add_argument("--timed-cache-probe", action="store_true")
    args = parser.parse_args()
    if args.timed_cache_probe:
        result = timed_cache_probe(args.output, seconds=args.seconds)
        print(result["status"], flush=True)
        raise SystemExit(0)
    if args.runtime_checkpoint:
        result = runtime_probe(
            args.cases, args.profile, args.output, args.runtime_checkpoint, seconds=args.seconds
        )
        print(result["status"], flush=True)
        raise SystemExit(0)
    if args.prepare_cases:
        prepare_cases(args.profile, args.cases, args.seconds)
    result = run(
        args.cases,
        args.profile,
        args.output,
        seconds=args.seconds,
        training_steps=args.training_steps,
    )
    print(result["status"], result["seconds"], flush=True)
