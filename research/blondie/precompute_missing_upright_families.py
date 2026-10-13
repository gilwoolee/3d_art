"""Targeted fixed whole-action Phase1 seeds; runtime still validates the full print."""

import argparse
import os
from dataclasses import replace
from pathlib import Path
from time import monotonic

from pen_art.curves import flatten_curve, transform_points
from pen_art.io import read_json, sha256, write_json
from pen_art.planning.motion_cache_precompute import (
    independently_validate_growing_prefix,
    promote_catalogue,
)
from pen_art.planning.motion_library import (
    CachedCollisionModel,
    DrawingRequest,
    MotionLibrary,
    digest,
)
from pen_art.planning.motion_runtime import load_context_for_run, static_shapes
from pen_art.planning.scene import MaterialScene
from pen_art.planning.turntable import IndexedTurntable


def run(args):
    args.output.mkdir(parents=True, exist_ok=True)
    receipt = {
        "status": "LOADING",
        "pid": os.getpid(),
        "library": str(args.library),
        "sections": args.sections,
        "maximum_seconds": args.maximum_seconds,
        "seconds_per_family": args.seconds_per_family,
        "results": [],
        "scope": (
            "CPU cached whole-path adaptation, independent self/static+own-prefix+nominal "
            "preceding contour checks. No approach/process/full-build acceptance."
        ),
    }

    def publish(stage, **extra):
        receipt.update(status=stage, **extra)
        write_json(args.output / "progress.json", receipt)
        print(stage, extra, flush=True)

    publish("LOADING")
    library = MotionLibrary(args.library)
    contexts = {}
    requests = {}
    kinds = {}
    actions = {}
    for obj in ("cube", "bottle"):
        source = Path(f"runs/{obj}_nozzle27_full_v" + ("3" if obj == "cube" else "1"))
        profile = Path(f"runs/{obj}_nozzle27_phase1_v1/profile.yaml")
        ctx = load_context_for_run(source, profile)
        ctx["source"] = source
        ctx["profile_path"] = profile
        contexts[obj] = ctx
        table = IndexedTurntable.from_profile(ctx["profile"])
        rows = read_json(source / "construction_plan.json")["actions"]
        actions[obj] = rows
        for row in rows:
            points, _ = flatten_curve(row, 0.00005, 0.003)
            kind = (
                "zigzag"
                if row.get("parent_curve", "").startswith("zigzag-")
                else "contour"
                if row.get("parent_curve", "").startswith("contour-")
                else "unknown"
            )
            identity = f"{obj}/{row['id']}"
            kinds[identity] = kind
            requests[identity] = DrawingRequest(
                transform_points(points, table.build_transform(0)),
                table_index=0,
                path_kind=kind,
                context_hash=ctx["context_hash"],
                source_object=obj,
                family_id=row.get("parent_curve", "unknown"),
            )
    coverage = library.coverage(requests)
    coverage["scope"] = (
        "Every real action at index0/forward; availability only, "
        "not adaptation or printing acceptance."
    )
    coverage["missing_zigzags"] = [
        r["id"]
        for r in coverage["requests"]
        if kinds[r["id"]] == "zigzag"
        and not (r["exact_source_seeds"] or r["compatible_family_seeds"])
    ]
    write_json(args.output / "coverage_before.json", coverage)
    publish(
        "COVERAGE_READY",
        record_count=len(library.records),
        missing_zigzags=coverage["missing_zigzags"],
    )
    if args.coverage_only:
        return receipt
    obj = args.object
    ctx = contexts[obj]
    source = ctx["source"]
    profile = ctx["profile_path"]
    arm = ctx["arm"]
    obstacles = ctx["obstacles"]
    table = IndexedTurntable.from_profile(ctx["profile"])
    private = MotionLibrary(args.output / "library")
    solver = CachedCollisionModel(
        arm.collision,
        args.library,
        geometry_context=arm.tool.metadata,
        static_geometry=static_shapes(ctx["profile"], table),
    )
    # Read active clearance samples, but write this worker's samples privately.
    solver.directory = args.output / "solver_clearance" / solver.context_hash
    solver_arm = replace(arm, collision=solver)
    start = monotonic()
    deadline = start + args.maximum_seconds
    for section in args.sections:
        if monotonic() >= deadline:
            break
        row = next(r for r in actions[obj] if r["id"] == section)
        if not row.get("parent_curve", "").startswith("zigzag-"):
            raise ValueError("Select whole zigzags")
        request = requests[f"{obj}/{section}"]
        result = {
            "section": section,
            "source_legs": len(row["segments"]),
            "accepted": [],
            "attempts": [],
        }
        receipt["results"].append(result)
        local_deadline = min(deadline, monotonic() + args.seconds_per_family)
        scene = MaterialScene()
        position = actions[obj].index(row)
        preceding = actions[obj][position - 1]
        if not preceding.get("parent_curve", "").startswith("contour-"):
            raise ValueError("Expected preceding whole contour")
        pp, _ = flatten_curve(preceding, 0.00005, 0.003)
        pp = transform_points(pp, table.build_transform(0))
        for i, (a, b) in enumerate(zip(pp[:-1], pp[1:], strict=True)):
            scene.add(a, b, preceding["diameter_m"] / 2, f"nominal/{preceding['id']}/{i}")
        material_path = args.output / f"{section}_nominal_initial_material.json"
        write_json(
            material_path,
            {
                "capsules": scene.segments,
                "scope": "Nominal preceding source contour, not accepted runtime deposition",
            },
        )
        matches = library.retrieve(
            request, limit=3, reference_q=arm.initial_positions, compatible_only=True
        )
        # Keep physically matching context seeds ahead of other-object contexts.
        matches.sort(key=lambda r: r["request"]["context_hash"] != ctx["context_hash"])
        for record in matches:
            if monotonic() >= local_deadline:
                break
            parent = digest(record)
            query = replace(request, fixed_tcp_rotation=record["trajectory"]["tcp_rotation"])
            publish("ADAPTING", active_section=section, parent_record=parent)
            proposal = library.adapt(
                record,
                query,
                solver_arm,
                obstacles,
                deadline=min(local_deadline, monotonic() + 35),
                clearance_m=ctx["profile"]["environment"]["clearance_m"],
            )
            if proposal is None:
                result["attempts"].append(
                    {"parent_record": parent, "status": "UNRESOLVED", "phase": "ADAPTATION"}
                )
                continue
            write_json(args.output / f"{section}_proposal.json", proposal)

            def progress(phase, details, active=section):
                publish("VALIDATING", active_section=active, validation_phase=phase)

            checked, validation = independently_validate_growing_prefix(
                private,
                arm,
                obstacles,
                query,
                proposal,
                local_deadline,
                material_radius_m=row["diameter_m"] / 2,
                initial_scene=scene,
                clearance_m=ctx["profile"]["environment"]["clearance_m"],
                progress=progress,
            )
            result["attempts"].append({"parent_record": parent, "validation": validation})
            if checked is None:
                continue
            publish("FIXED_ORIENTATION_ADMISSION", active_section=section)
            record_id = private.remember_growing_prefix(
                query,
                checked,
                validation,
                {
                    "run": str(source),
                    "section": section,
                    "parent_curve": row["parent_curve"],
                    "source_sha256": sha256(source / "construction_plan.json"),
                    "proposal_origin": "cached_whole_path_adaptation",
                    "parent_record": parent,
                    "table_index": 0,
                    "reverse": False,
                    "start_visit": 0,
                    "orientation_policy": "fixed",
                    "pen_tilt_deg": 0.0,
                    "initial_material_path": str(material_path),
                    "inputs": {
                        **ctx["inputs"],
                        str(source / "construction_plan.json"): sha256(
                            source / "construction_plan.json"
                        ),
                        str(material_path): sha256(material_path),
                        str(Path(__file__)): sha256(Path(__file__)),
                    },
                },
                arm=arm,
            )
            result["accepted"].append(record_id)
            # Promotion repeats live context/source/material hashes and fixed-R proof.
            promotion = promote_catalogue(args.output, args.library, profile)
            write_json(args.output / f"{section}_promotion.json", promotion)
            publish(
                "PUBLISHED_SEED",
                active_section=section,
                published_ids=promotion["promoted_record_ids"]
                if "promoted_record_ids" in promotion
                else promotion,
            )
            break
        result["status"] = "SEED_ADMITTED" if result["accepted"] else "UNRESOLVED"
        publish("FAMILY_COMPLETE", active_section=section)
    solver.flush()
    receipt["seconds"] = monotonic() - start
    publish("COMPLETE" if len(receipt["results"]) == len(args.sections) else "TIME_BUDGET")
    write_json(args.output / "result.json", receipt)
    return receipt


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--library", type=Path, default=Path("runs/motion_library_phase1_upright27_v1")
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--object", choices=["cube", "bottle"], default="bottle")
    parser.add_argument(
        "--sections", nargs="+", default=["section-00037", "section-00041", "section-00043"]
    )
    parser.add_argument("--maximum-seconds", type=float, default=420)
    parser.add_argument("--seconds-per-family", type=float, default=120)
    parser.add_argument("--coverage-only", action="store_true")
    run(parser.parse_args())
