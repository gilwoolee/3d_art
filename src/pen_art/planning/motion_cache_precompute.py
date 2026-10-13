"""Precompute whole-curve robot seeds checked against their own growing prefix.

Offline geometry checks never certify full construction order, cooling, approach
or departure. Runtime must validate those and the current scene for each seed.
"""

from dataclasses import replace
from pathlib import Path
from time import monotonic

import fcl
import numpy as np
import yaml

from ..cell import load_cell
from ..curves import flatten_curve, transform_points
from ..io import read_json, sha256, write_json
from ..strokes import orient_stroke
from .cell_motion import drawing_alternatives
from .collision import Body, RobotCollisionModel
from .motion_library import (
    DRAWING_POLICY_VERSION,
    GROWING_PREFIX_POLICY,
    GROWING_PREFIX_SCOPE,
    CachedCollisionModel,
    DrawingRequest,
    MotionLibrary,
    digest,
)
from .motion_runtime import load_context_for_run, static_shapes
from .scene import MaterialScene
from .tool import sampled_pen_tilt
from .turntable import IndexedTurntable


def representative_actions(source, maximum=None):
    """Select real whole zigzags, preferring the longest multi-leg source."""
    rows = read_json(Path(source) / "construction_plan.json")["actions"]
    zigzags = [row for row in rows if row.get("parent_curve", "").startswith("zigzag-")]
    zigzags.sort(key=lambda row: (-len(row["segments"]), row["parent_curve"], row["id"]))
    selected, shapes = [], set()
    for row in zigzags:
        shape = digest(row["segments"])
        if shape not in shapes:
            selected.append(row)
            shapes.add(shape)
            if maximum is not None and len(selected) == maximum:
                break
    return selected


def request_for_action(row, source, table, index, context, *, reverse=False, start_visit=0):
    oriented = orient_stroke(row, reverse=reverse, start_visit=start_visit)
    points, _ = flatten_curve(oriented, 0.00005, 0.003)
    return DrawingRequest(
        transform_points(points, table.build_transform(index)),
        table_index=index,
        source_object=Path(source).name.split("_")[0],
        family_id=row["parent_curve"],
        context_hash=context,
        path_kind="zigzag",
    )


def independently_validate(
    library,
    arm,
    obstacles,
    request,
    drawing,
    deadline,
    *,
    clearance_m=0.001,
):
    """Reproject and continuously check every source interval with the raw model."""
    proposal = library.adapt_proposal(
        drawing, request, arm, obstacles, deadline=deadline, clearance_m=clearance_m
    )
    if proposal is None:
        return None, {
            "status": "UNRESOLVED",
            "scope": "SELF_STATIC_ONLY",
            "reason": "whole_path_projection_or_continuous_validation",
        }
    return proposal, {
        "status": "VERIFIED_VALID",
        "scope": "SELF_STATIC_ONLY",
        "source_tracking": "BOUNDED",
        "self_static_status": "CERTIFIED_MODELED_PATH",
        "context_hash": request.context_hash,
        "drawing_policy": DRAWING_POLICY_VERSION,
        "clearance_m": clearance_m,
        "tracking_position_limit_m": 0.0003,
        "source_points": len(request.source_points_m),
        "checked_points": len(proposal["positions"]),
        "progressive_material_checked": False,
        "timing_checked": False,
    }


def independently_validate_growing_prefix(
    library,
    arm,
    obstacles,
    request,
    drawing,
    deadline,
    *,
    material_radius_m,
    initial_scene=None,
    clearance_m=0.001,
    progress=None,
):
    """Admit only whole candidates that clear their own growing deposited wire.

    Uses the runtime's exact PCHIP tracking and progressive capsule checker with
    the raw robot model; no cached collision verdict is an admission certificate.
    """
    from .cell_planner import check_growing_drawing_prefix, tracked_drawing_path

    if not np.isfinite(material_radius_m) or material_radius_m <= 0:
        raise ValueError("Material radius must be finite and positive")
    if progress is not None:
        progress("self_static_projection", {})
    checked, validation = independently_validate(
        library,
        arm,
        obstacles,
        request,
        drawing,
        deadline,
        clearance_m=clearance_m,
    )
    if progress is not None and checked is not None:
        progress("self_static_checked", {"drawing": checked, "validation": validation})
    validation = {**validation, "scope": GROWING_PREFIX_SCOPE}
    if checked is None:
        return None, validation
    if progress is not None:
        progress("tracking", {"poses": len(checked["positions"])})
    tracked = tracked_drawing_path(arm, checked, True, deadline, require_fixed_orientation=True)
    if tracked["status"] != "BOUNDED":
        return None, {**validation, "status": "UNRESOLVED", "phase": "TRACKING", "reason": tracked}
    initial_scene = MaterialScene() if initial_scene is None else initial_scene
    scene, failure = check_growing_drawing_prefix(
        arm,
        tracked,
        initial_scene,
        obstacles,
        material_radius_m,
        clearance_m,
        20,
        deadline,
        name_prefix=f"offline/{arm.identity}/{request.family_id}",
        progress=(
            None
            if progress is None
            else lambda done, total: progress(
                "growing_prefix", {"checked_intervals": done, "total_intervals": total}
            )
        ),
    )
    if failure is not None:
        return None, {
            **validation,
            "status": "UNRESOLVED",
            "phase": "GROWING_PREFIX",
            "reason": failure,
            "progressive_material_checked": False,
        }
    return checked, {
        **validation,
        "growing_prefix_status": "CERTIFIED_MODELED_PATH",
        "growing_prefix_policy": GROWING_PREFIX_POLICY,
        "growing_prefix_trajectory_sha256": digest(
            {
                "q_path": checked["positions"],
                "source_points_m": checked["tcp_targets_m"],
                "tcp_rotation": checked.get("tcp_rotation"),
                "free_tool_orientation": checked.get("free_tool_orientation", False),
            }
        ),
        "progressive_material_checked": True,
        "timing_checked": False,
        "growing_prefix_intervals": len(tracked["positions"]) - 1,
        "material_radius_m": material_radius_m,
        "initial_material_sha256": digest(initial_scene.segments),
        "initial_material_capsules": len(initial_scene.segments),
        "deposited_capsules": len(scene.segments) - len(initial_scene.segments),
        "interpolation": "pchip",
        "subdivisions": tracked["subdivisions"],
    }


def precompute(
    sources,
    profile_path,
    output,
    *,
    maximum_seconds=600,
    seconds_per_request=90,
    maximum_families=None,
    variants_per_family=2,
    indices=(0, 1, 2, 3),
    gpu=False,
    manifest=None,
    growing_prefix=True,
):
    output, profile_path = Path(output), Path(profile_path)
    output.mkdir(parents=True, exist_ok=True)
    if (output / "progress.json").exists():
        raise FileExistsError("Use a fresh output directory to preserve generation evidence")
    profile, arms, obstacles = load_cell(profile_path)
    clearance_m = profile["environment"]["clearance_m"]
    if len(arms) != 1:
        raise ValueError("Whole-curve precomputation requires the single-arm cell")
    arm = next(iter(arms.values()))
    table = IndexedTurntable.from_profile(profile)
    library = MotionLibrary(output / "library")
    solver_collision = CachedCollisionModel(
        arm.collision,
        library.directory,
        geometry_context=arm.tool.metadata,
        static_geometry=static_shapes(profile, table),
    )
    solver_arm = replace(arm, collision=solver_collision)
    backend = None
    if gpu:
        from .curobo_adapter import CuroboAdapter

        backend = CuroboAdapter(solver_arm)
    source_rows = []
    if manifest is not None:
        data = read_json(Path(manifest))
        source_rows = data if isinstance(data, list) else data["requests"]
    else:
        for source in sources:
            for row in representative_actions(source, maximum_families):
                source_rows.extend(
                    {"run": str(source), "section": row["id"], "table_index": index}
                    for index in indices
                )
    if not source_rows:
        raise ValueError("No whole-zigzag requests were selected for precomputation")
    scope = GROWING_PREFIX_SCOPE if growing_prefix else "SELF_STATIC_ONLY"
    started, deadline = monotonic(), monotonic() + maximum_seconds
    results = []

    def publish(status):
        covered = sum(bool(entry["accepted"]) for entry in results)
        report = {
            "status": status,
            "scope": scope,
            "library": str(library.directory),
            "seconds": monotonic() - started,
            "requests": results,
            "records": len(library.records),
            "profile_sha256": sha256(profile_path),
            "coverage": {
                "requested": len(source_rows),
                "covered": covered,
                "unresolved": sum(entry.get("status") == "UNRESOLVED" for entry in results),
                "unattempted": len(source_rows) - len(results),
            },
            "coverage_status": "POPULATED" if covered == len(source_rows) else "PARTIAL",
            "generator_sha256": sha256(Path(__file__)),
            "limits": {
                "maximum_seconds": maximum_seconds,
                "seconds_per_request": seconds_per_request,
                "variants_per_family": variants_per_family,
            },
            "limitations": [
                "No full construction-order or other-structure certificate"
                if growing_prefix
                else "No growing material certificate",
                "No feed, cooling, timing, approach or departure certificate",
                "Runtime must adapt and validate each whole-action proposal",
            ],
        }
        write_json(output / "progress.json", report)
        return report

    publish("RUNNING")
    for item in source_rows:
        if monotonic() >= deadline:
            break
        source = Path(item["run"])
        row = next(
            row
            for row in read_json(source / "construction_plan.json")["actions"]
            if row["id"] == item["section"]
        )
        if not row.get("parent_curve", "").startswith("zigzag-"):
            raise ValueError("Whole-zigzag precomputation cannot label another curve as a zigzag")
        bound = load_context_for_run(source, profile_path)
        context = bound["context_hash"]
        index = item.get("table_index", 0)
        entry = {
            **item,
            "parent_curve": row["parent_curve"],
            "source_sha256": sha256(source / "construction_plan.json"),
            "source_legs": len(row["segments"]),
            "accepted": [],
            "attempts": [],
        }
        results.append(entry)
        publish("RUNNING")
        local_deadline = min(deadline, monotonic() + seconds_per_request)
        proposals = []
        if item.get("seed_drawing_path"):
            seed = read_json(Path(item["seed_drawing_path"]))
            proposals.append(seed.get("drawing", seed))
        generated = drawing_alternatives(
            solver_arm,
            row,
            {row["start"]},
            table.build_transform(index),
            obstacles,
            deadline=local_deadline,
            seeds=[arm.initial_positions],
            pen_tilt_deg=profile.get("drawing_pen_tilt_deg", 70),
            pen_tilt_candidates_deg=profile.get("drawing_pen_tilt_candidates_deg", [70, 80]),
            orientation_policy=profile.get("drawing_orientation_policy", "fixed"),
            ik_backend=backend,
            settings={"maximum_subdivision_depth": 20},
            clearance_m=clearance_m,
        )
        seen = set()
        while monotonic() < local_deadline and len(entry["accepted"]) < variants_per_family:
            try:
                from_seed = bool(proposals)
                candidate = proposals.pop(0) if proposals else next(generated, None)
            except TimeoutError:
                entry["attempts"].append({"status": "UNRESOLVED", "reason": "solver_deadline"})
                break
            if candidate is None:
                break
            if candidate.get("status", "SAMPLED_DRAWING_PATH") != "SAMPLED_DRAWING_PATH":
                entry["attempts"].append(
                    {"status": candidate["status"], "failed_sample": candidate.get("failed_sample")}
                )
                publish("RUNNING")
                continue
            if candidate.get("table_index", index) != index:
                entry["attempts"].append(
                    {"status": "SEED_CONTEXT_MISMATCH", "field": "table_index"}
                )
                continue
            if "reverse" in item and candidate.get("reverse", False) != item["reverse"]:
                continue
            if "start_visit" in item and candidate.get("start_visit", 0) != item["start_visit"]:
                continue
            request = request_for_action(
                row,
                source,
                table,
                index,
                context,
                reverse=candidate.get("reverse", False),
                start_visit=candidate.get("start_visit", 0),
            )
            if candidate.get("free_tool_orientation", False):
                entry["attempts"].append({"status": "UNRESOLVED", "reason": "variable_orientation"})
                continue
            request = replace(request, fixed_tcp_rotation=candidate.get("tcp_rotation"))
            # Never present a cached bound as the independent admission check.
            try:
                if growing_prefix:
                    initial_scene = MaterialScene()
                    if item.get("initial_material_path"):
                        material = read_json(Path(item["initial_material_path"]))
                        for capsule in material["capsules"]:
                            initial_scene.add(
                                capsule["start"],
                                capsule["end"],
                                capsule["radius_m"],
                                capsule["name"],
                            )
                    checked, validation = independently_validate_growing_prefix(
                        library,
                        arm,
                        obstacles,
                        request,
                        candidate,
                        local_deadline,
                        material_radius_m=row["diameter_m"] / 2,
                        initial_scene=initial_scene,
                        clearance_m=clearance_m,
                    )
                else:
                    checked, validation = independently_validate(
                        library,
                        arm,
                        obstacles,
                        request,
                        candidate,
                        local_deadline,
                        clearance_m=clearance_m,
                    )
            except TimeoutError:
                checked, validation = None, {"status": "UNRESOLVED", "reason": "validator_deadline"}
            entry["attempts"].append(validation)
            if checked is not None:
                points = np.asarray(checked["tcp_targets_m"])
                progress = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
                source_points = np.asarray(request.source_points_m)
                source_progress = np.r_[
                    0.0, np.cumsum(np.linalg.norm(np.diff(source_points, axis=0), axis=1))
                ]
                grid = np.unique(
                    np.r_[np.linspace(0, 1, 32), source_progress / source_progress[-1]]
                )
                sampled = np.column_stack(
                    [
                        np.interp(grid * progress[-1], progress, axis)
                        for axis in np.asarray(checked["positions"]).T
                    ]
                )
                signature = digest(
                    {"source": request.source_points_m, "joints": np.round(sampled, 5).tolist()}
                )
                if signature not in seen:
                    remember = (
                        library.remember_growing_prefix
                        if growing_prefix
                        else library.remember_self_static
                    )
                    record = remember(
                        request,
                        checked,
                        validation,
                        {
                            "run": str(source),
                            "section": row["id"],
                            "parent_curve": row["parent_curve"],
                            "generator": "whole_curve_precompute",
                            "table_index": index,
                            "reverse": candidate.get("reverse", False),
                            "start_visit": candidate.get("start_visit", 0),
                            "orientation_policy": candidate.get(
                                "orientation_policy", profile.get("drawing_orientation_policy")
                            ),
                            "pen_tilt_deg": candidate.get("pen_tilt_deg"),
                            "roll_rad": candidate.get("roll_rad"),
                            "actual_pen_tilt": sampled_pen_tilt(arm, checked["positions"]),
                            "source_sha256": entry["source_sha256"],
                            "proposal_origin": "supplied_seed"
                            if from_seed
                            else "whole_path_solver",
                            "seed_provenance": item.get("seed_provenance", []) if from_seed else [],
                            "initial_material_path": item.get("initial_material_path"),
                            "inputs": {
                                **bound["inputs"],
                                str(source / "construction_plan.json"): entry["source_sha256"],
                                str(Path(__file__)): sha256(Path(__file__)),
                                **(
                                    {
                                        str(item["initial_material_path"]): sha256(
                                            Path(item["initial_material_path"])
                                        )
                                    }
                                    if item.get("initial_material_path")
                                    else {}
                                ),
                                **(
                                    {
                                        str(item["seed_drawing_path"]): sha256(
                                            Path(item["seed_drawing_path"])
                                        )
                                    }
                                    if from_seed
                                    else {}
                                ),
                            },
                        },
                        arm=arm,
                    )
                    seen.add(signature)
                    entry["accepted"].append(record)
                else:
                    entry["attempts"].append({"status": "DUPLICATE_VALIDATED_POSTURE"})
            publish("RUNNING")
        entry["status"] = "GENERATED" if entry["accepted"] else "UNRESOLVED"
        publish("RUNNING")
        solver_collision.flush()
    result = publish("COMPLETE" if len(results) == len(source_rows) else "TIME_BUDGET")
    write_json(output / "catalogue.json", result)
    return result


def promote_catalogue(source, destination, profile_path):
    """Idempotently activate admitted seeds while preserving existing records."""
    source, destination = Path(source), Path(destination)
    if not (source / "library" / "motions").is_dir():
        raise ValueError("Source catalogue has no admitted motion directory")
    source_library = MotionLibrary(source / "library")
    if not source_library.records:
        raise ValueError("Source catalogue has no valid admitted motion records")
    target = MotionLibrary(destination)
    before = {digest(record) for record in target.records}
    promoted, contexts = [], {}
    for record in source_library.records:
        provenance = record["provenance"]
        run = provenance["run"]
        if run not in contexts:
            contexts[run] = load_context_for_run(run, profile_path)
        if record["request"]["context_hash"] != contexts[run]["context_hash"]:
            raise ValueError("Cannot promote an offline seed for a changed robot/process context")
        construction = Path(run) / "construction_plan.json"
        if provenance["source_sha256"] != sha256(construction):
            raise ValueError("Cannot promote a seed with changed construction geometry")
        initial_path = provenance.get("initial_material_path")
        if initial_path and (
            provenance.get("inputs", {}).get(str(initial_path)) != sha256(Path(initial_path))
        ):
            raise ValueError("Cannot promote a seed with changed initial material")
        trajectory = record["trajectory"]
        remember = (
            target.remember_growing_prefix
            if record["validation"].get("scope") == GROWING_PREFIX_SCOPE
            else target.remember_self_static
        )
        record_id = remember(
            DrawingRequest(**record["request"]),
            {
                "positions": trajectory["q_path"],
                "tcp_targets_m": trajectory["source_points_m"],
                "tcp_rotation": trajectory["tcp_rotation"],
                "free_tool_orientation": trajectory["free_tool_orientation"],
            },
            record["validation"],
            provenance,
            arm=contexts[run]["arm"],
        )
        if record_id != digest(record):
            raise ValueError("Promoted record differs from its validated source")
        promoted.append(record_id)
    after = {digest(record) for record in target.records}
    clearance_rows = 0
    checked_models = set()
    for context in contexts.values():
        arm, profile = context["arm"], context["profile"]
        shapes = static_shapes(profile, IndexedTurntable.from_profile(profile))
        destination_model = CachedCollisionModel(
            arm.collision, destination, geometry_context=arm.tool.metadata, static_geometry=shapes
        )
        if destination_model.context_hash in checked_models:
            continue
        checked_models.add(destination_model.context_hash)
        for base in (source / "library", source / "solver_clearance"):
            if not (base / "clearance" / destination_model.context_hash / "samples.json").exists():
                continue
            source_model = CachedCollisionModel(
                arm.collision, base, geometry_context=arm.tool.metadata, static_geometry=shapes
            )
            for key, rows in source_model.samples.items():
                existing = destination_model.samples.setdefault(key, [])
                known = {digest([q, distances]) for q, distances in existing}
                for q, distances in rows:
                    signature = digest([q, distances])
                    if signature not in known:
                        existing.append((q, distances))
                        known.add(signature)
                        clearance_rows += 1
                del existing[: -destination_model.maximum_samples]
        destination_model.flush()
    receipt = {
        "status": (
            "PROMOTED_GROWING_PREFIX"
            if all(
                r["validation"].get("scope") == GROWING_PREFIX_SCOPE for r in source_library.records
            )
            else "PROMOTED_SELF_STATIC_ONLY"
        ),
        "source_library": str(source_library.directory),
        "destination_library": str(destination),
        "record_ids": promoted,
        "before_record_ids": sorted(before),
        "after_record_ids": sorted(after),
        "added": len(after - before),
        "preserved": before <= after,
        "validated_clearance_samples_imported": clearance_rows,
        "scope": "Each seed still requires current material/process validation in runtime",
    }
    write_json(source / "promotion.json", receipt)
    return receipt


def rebuild_clearance_atlas(
    library_path,
    profile_path,
    output_path,
    *,
    maximum_seconds=60,
    maximum_samples=512,
    reference_library=None,
    filter_profile_context=False,
):
    """Populate and independently exercise both runtime robot/tool atlases.

    Only current-context admitted paths provide sample configurations. Every new
    bound comes from the loaded robot/tool geometry; old atlas rows are excluded
    by their model hash. Growing printed material is outside this atlas.
    """
    started = monotonic()
    deadline = started + maximum_seconds
    library_path, profile_path = Path(library_path), Path(profile_path)
    profile, arms, obstacles = load_cell(profile_path)
    if len(arms) != 1:
        raise ValueError("Clearance rebuild requires the indexed single-arm cell")
    arm = next(iter(arms.values()))
    table = IndexedTurntable.from_profile(profile)
    shapes = static_shapes(profile, table)
    transit = yaml.safe_load(Path("configs/transit.yaml").read_text())
    tip_radius = arm.tool.tip_radius_m or transit["virtual_tip_radius_m"]
    tip = Body("virtual_tip", arm.robot.tip, arm.flange_to_tcp, fcl.Sphere(tip_radius), tip_radius)
    tip_model = RobotCollisionModel(arm.robot, [*arm.tool.bodies, tip])
    models = {
        "robot_tool": (arm.collision, arm.tool.metadata),
        "robot_tool_virtual_tip": (tip_model, {"tool": arm.tool.metadata, "virtual_tip": True}),
    }
    wrappers = {
        name: CachedCollisionModel(
            model,
            library_path,
            geometry_context=metadata,
            static_geometry=shapes,
            maximum_samples=maximum_samples,
        )
        for name, (model, metadata) in models.items()
    }
    library = MotionLibrary(library_path)
    if not library.records:
        raise ValueError("Populate current-context whole-path seeds before rebuilding clearance")
    contexts, positions, record_ids, skipped_ids = {}, [], [], []
    for record in library.records:
        run = record["provenance"]["run"]
        if run not in contexts:
            contexts[run] = load_context_for_run(run, profile_path)
        if record["request"]["context_hash"] != contexts[run]["context_hash"]:
            if filter_profile_context:
                skipped_ids.append(digest(record))
                continue
            raise ValueError(
                "The active library contains a motion from another mount/policy context"
            )
        joints = np.asarray(record["trajectory"]["q_path"])
        selected = np.unique(
            np.linspace(0, len(joints) - 1, min(len(joints), maximum_samples)).astype(int)
        )
        positions.extend(joints[selected])
        record_ids.append(digest(record))
    if not record_ids:
        raise ValueError("No motion records match the requested profile/process context")
    distinct = {}
    for q in positions:
        distinct.setdefault(tuple(np.round(q, 8)), q)
    positions = list(distinct.values())
    sampled = 0
    for q in positions:
        if monotonic() >= deadline:
            break
        try:
            for wrapper in wrappers.values():
                list(wrapper.distances(q, obstacles, deadline=deadline))
        except TimeoutError:
            break
        sampled += 1
    reports = {}
    old_hashes = set()
    if reference_library is not None:
        old_hashes = {
            path.name for path in (Path(reference_library) / "clearance").glob("*") if path.is_dir()
        }
    for name, wrapper in wrappers.items():
        wrapper.flush()
        model, metadata = models[name]
        reloaded = CachedCollisionModel(
            model,
            library_path,
            geometry_context=metadata,
            static_geometry=shapes,
            maximum_samples=maximum_samples,
        )
        probes = []
        # Require actual restored bound use, with fresh-model comparison. Empty
        # or colliding sample sets are reported rather than promoted as reuse.
        for key, rows in reloaded.samples.items():
            if not key.startswith("static:"):
                continue
            for q, distances in reversed(rows):
                if not all(distance > 0.001001 for _, _, distance, _ in distances):
                    continue
                before = reloaded.hits
                cached = reloaded.check(q, obstacles, 0.001)
                fresh = model.check(q, obstacles, 0.001)
                if bool(cached) != bool(fresh):
                    raise ValueError("Restored clearance bounds disagree with the fresh model")
                probes.append(
                    {
                        "q": q.tolist(),
                        "cache_hit": reloaded.hits > before,
                        "fresh_collision": bool(fresh),
                    }
                )
                if len(probes) == 3:
                    break
            if probes:
                break
        reports[name] = {
            "context_hash": wrapper.context_hash,
            "samples": sum(len(rows) for rows in wrapper.samples.values()),
            "generation_hits": wrapper.hits,
            "generation_misses": wrapper.misses,
            "reload_hits": reloaded.hits,
            "reload_misses": reloaded.misses,
            "restored_bound_probes": probes,
            "old_context_reused": wrapper.context_hash in old_hashes,
            "sample_artifact": str(wrapper.directory / "samples.json"),
            "sample_artifact_sha256": sha256(wrapper.directory / "samples.json"),
        }
    result = {
        "status": "REBUILT_AND_REUSE_VERIFIED"
        if all(row["reload_hits"] and not row["old_context_reused"] for row in reports.values())
        else "REBUILT_WITH_UNVERIFIED_REUSE",
        "profile": str(profile_path),
        "profile_sha256": sha256(profile_path),
        "library": str(library_path),
        "record_ids": record_ids,
        "filter_profile_context": filter_profile_context,
        "skipped_other_context_record_ids": skipped_ids,
        "configuration_samples": sampled,
        "seconds": monotonic() - started,
        "models": reports,
        "reference_model_hashes": sorted(old_hashes),
        "scope": "Robot/tool self and static clearance; current material checked at runtime",
    }
    write_json(Path(output_path), result)
    return result
