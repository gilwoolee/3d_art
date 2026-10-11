"""Run reproducible stages; unavailable stages stay explicitly pending."""

import argparse
import hashlib
import json
from pathlib import Path

import yaml

from .abstraction.comparison import compare_run, comparison_current
from .abstraction.geometric import extract_candidates
from .abstraction.selection import select_run
from .abstraction.sparse import sparse_run
from .abstraction.wir3d_adapter import (
    available_budgets,
    import_curves,
    prepare_job,
    selected_candidate_current,
)
from .assets import prepare_asset
from .backbone import backbone_run
from .construction import construction_preview
from .graph import build_graph
from .io import artifact, now, read_json, sha256, stage_update, write_json
from .placement import search as search_placement
from .planning.collision_report import audit_drawing_paths
from .planning.curve_motion import build_preview
from .planning.smooth_audit import audit_smooth_paths
from .planning.transit import plan_sequence
from .process import screen_graph
from .strokes import pen_strokes
from .structure import repair_structure
from .timing import build_schedule

STAGES = [
    "asset",
    "geometric",
    "wir3d",
    "comparison",
    "selection",
    "graph",
    "backbone",
    "construction",
    "motion",
    "timing",
    "validation",
]


def initialize(config_path: Path):
    config_path = config_path.resolve()
    config = yaml.safe_load(config_path.read_text())
    root = config_path.parent.parent
    source = (root / config["source"]).resolve()
    run = (root / config["run_dir"]).resolve()
    run.mkdir(parents=True, exist_ok=True)
    fingerprint = hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest()
    if (run / "run.json").exists():
        previous = read_json(run / "run.json")
        if previous["config_sha256"] != fingerprint or previous["source_sha256"] != sha256(source):
            raise ValueError(
                "Configuration or source changed. Use a new run_dir to preserve provenance."
            )
    else:
        write_json(
            run / "run.json",
            artifact(
                "run",
                id=run.name,
                name=config["name"],
                created_at=now(),
                updated_at=now(),
                config_sha256=fingerprint,
                source_sha256=sha256(source),
                stages={name: {"status": "pending"} for name in STAGES},
            ),
        )
        write_json(run / "config_snapshot.json", config)
    return config, source, run


def prepare(config, source, run):
    if read_json(run / "run.json")["stages"]["asset"]["status"] == "complete":
        return read_json(run / "asset_manifest.json")
    stage_update(
        run, "asset", "running", message="Converting textured model and processing geometry"
    )
    try:
        result = prepare_asset(source, run, config)
    except Exception as error:
        stage_update(run, "asset", "failed", message=str(error))
        raise
    stage_update(run, "asset", "complete", artifact="asset_manifest.json")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("configs/palace.yaml"))
    parser.add_argument(
        "--stage",
        choices=[
            "prepare",
            "geometric",
            "abstraction",
            "import",
            "compare",
            "select",
            "sparse",
            "graph",
            "backbone",
            "construction",
            "placement",
            "motion",
            "collisions",
            "sequence",
            "timing",
            "validation",
        ],
        default="abstraction",
    )
    parser.add_argument("--curves", type=Path, help="WIR3D runner's curves_<budget>.json")
    parser.add_argument("--budget", type=int, default=40)
    parser.add_argument(
        "--candidate",
        default="auto",
        choices=["auto", "geometric", "selected", "wir3d_20", "wir3d_40", "wir3d_60"],
    )
    parser.add_argument("--fabrication-config", type=Path, default=Path("configs/fabrication.yaml"))
    parser.add_argument("--comparison-config", type=Path, default=Path("configs/comparison.yaml"))
    parser.add_argument("--selection-config", type=Path, default=Path("configs/selection.yaml"))
    parser.add_argument("--placement-config", type=Path, default=Path("configs/placement.yaml"))
    parser.add_argument("--collision-config", type=Path, default=Path("configs/collision.yaml"))
    parser.add_argument("--transit-config", type=Path, default=Path("configs/transit.yaml"))
    parser.add_argument("--timing-config", type=Path, default=Path("configs/timing.yaml"))
    parser.add_argument("--maximum-seconds", type=float, default=300)
    parser.add_argument("--beam-width", type=int, default=2)
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    config, source, run = initialize(args.config)
    prepare(config, source, run)
    if args.stage in {"abstraction", "geometric"}:
        stage_update(run, "geometric", "running", message="Extracting mesh cross-sections")
        try:
            result = extract_candidates(run, config)
        except Exception as error:
            stage_update(run, "geometric", "failed", message=str(error))
            raise
        stage_update(
            run,
            "geometric",
            "complete",
            artifact="geometric_candidates.json",
            curve_count=len(result["curves"]),
            method="geometric_sections",
        )
        existing_graph = read_json(run / "run.json")["stages"]["graph"]
        if existing_graph.get("source") == "geometric":
            for downstream in ("graph", "construction", "motion", "timing", "validation"):
                stage_update(run, downstream, "pending", message="Geometric candidates changed")
    if args.stage == "abstraction":
        prepare_job(run, config)
        existing = read_json(run / "run.json")["stages"]["wir3d"]
        if existing["status"] != "complete":
            stage_update(
                run,
                "wir3d",
                "awaiting_compute",
                message="GPU job bundle ready",
                requires="CUDA GPU with ≥24 GB memory and Blender 3.6",
                bundle="wir3d_bundle",
                budgets=config["wir3d"]["budgets"],
            )
    if args.stage == "import":
        if args.curves is None:
            parser.error("--curves is required for import")
        result = import_curves(run, args.curves, args.budget)
        available = available_budgets(run)
        complete = set(config["wir3d"]["budgets"]).issubset(available)
        stage_update(
            run,
            "wir3d",
            "complete" if complete else "partial",
            budgets=available,
            requested_budgets=config["wir3d"]["budgets"],
            curve_count=len(result["curves"]),
            artifact=f"wir3d_{args.budget}.json",
        )
    if args.stage == "sparse":
        sparse_run(run)
    if args.stage in {"compare", "select"}:
        write_json(
            run / "comparison_config.json",
            {
                "comparison": yaml.safe_load(args.comparison_config.read_text()),
                "fabrication": yaml.safe_load(args.fabrication_config.read_text()),
            },
        )
        if args.stage == "select":
            write_json(
                run / "selection_config.json",
                {
                    **read_json(run / "comparison_config.json"),
                    "selection": yaml.safe_load(args.selection_config.read_text()),
                },
            )
            select_run(run)
        compare_run(run)
    if args.stage == "graph":
        selection = args.candidate
        fabrication = yaml.safe_load(args.fabrication_config.read_text())
        if selection == "auto":
            if (run / "comparison_config.json").is_file():
                if read_json(run / "comparison_config.json")["fabrication"] != fabrication:
                    parser.error(
                        "Fabrication assumptions changed; rerun comparison/selection first"
                    )
                if not comparison_current(run):
                    parser.error("Candidate comparison is stale; run --stage compare first")
                choice = read_json(run / "candidate_comparison.json")["provisional_choice"]
                if not choice:
                    parser.error(
                        "No connected candidate fits the assumed partial time budget; "
                        "revise candidates or calibrated speed, then compare again"
                    )
                selection = Path(choice).stem
                if selection == "geometric_candidates":
                    selection = "geometric"
            else:
                selection = next(
                    (
                        f"wir3d_{budget}"
                        for budget in (40, 20, 60)
                        if (run / f"wir3d_{budget}.json").is_file()
                    ),
                    None,
                )
        if selection in {"selected", "selected_candidates"} and not selected_candidate_current(run):
            parser.error("Selected curves are stale; rerun --stage select first")
        if selection is None:
            parser.error(
                "No WIR3D curves yet. Use --candidate geometric explicitly for development."
            )
        source_curves = run / (
            {"geometric": "geometric_candidates.json", "selected": "selected_candidates.json"}.get(
                selection, f"{selection}.json"
            )
        )
        stage_update(
            run, "graph", "running", message="Finding 3D contacts and physical connections"
        )
        try:
            graph = build_graph(read_json(source_curves), fabrication["graph"])
            graph, structure = repair_structure(
                graph, fabrication["graph"], fabrication["structure"]
            )
            strokes = pen_strokes(graph)
            process = screen_graph(graph, fabrication["process"], strokes)
            graph["metrics"]["proposed_pen_strokes"] = len(strokes["curves"])
            graph["provenance"] = {
                "source_file": source_curves.name,
                "source_sha256": sha256(source_curves),
                "fabrication_config_sha256": sha256(args.fabrication_config),
            }
            graph["checks"]["process_feasible"] = (
                process["modeled_pass"] if process["calibrated"] else None
            )
            write_json(run / "fabrication_config.json", fabrication)
            write_json(run / "stroke_graph.json", graph)
            write_json(run / "pen_strokes.json", strokes)
            placement = run / "placement_search.json"
            if placement.exists() and read_json(placement)["inputs"]["pen_strokes.json"] != sha256(
                run / "pen_strokes.json"
            ):
                placement.replace(run / "placement_search.stale.json")
            write_json(run / "structural_screen.json", structure)
            write_json(run / "process_screen.json", process)
        except Exception as error:
            stage_update(run, "graph", "failed", message=str(error))
            raise
        for filename in ("drawing_motion", "drawing_collisions", "sequence_motion"):
            path = run / f"{filename}.json"
            if path.exists():
                path.replace(run / f"{filename}.stale.json")
        for downstream in ("construction", "motion", "timing", "validation"):
            stage_update(
                run, downstream, "pending", message="Requires planning for the current stroke graph"
            )
        stage_update(
            run,
            "graph",
            "complete",
            artifact="stroke_graph.json",
            source=selection,
            message="Connected geometry and modeled screens; physical feasibility unverified",
        )
    if args.stage == "backbone":
        backbone_run(run, min(args.maximum_seconds, 5))
    if args.stage == "construction":
        stage_update(run, "construction", "running", message="Searching grounded pen-stroke order")
        try:
            result = construction_preview(
                read_json(run / "stroke_graph.json"), read_json(run / "pen_strokes.json")
            )
            result["provenance"] = {
                "graph_sha256": sha256(run / "stroke_graph.json"),
                "strokes_sha256": sha256(run / "pen_strokes.json"),
            }
            write_json(run / "construction_plan.json", result)
        except Exception as error:
            stage_update(run, "construction", "failed", message=str(error))
            raise
        stage_update(
            run,
            "construction",
            "partial" if result["actions"] else "failed",
            artifact="construction_plan.json",
            message="Grounded geometry preview; robot, stiffness and pen checks remain",
            result_status=result["status"],
        )
    if args.stage == "placement":
        search_placement(run, args.placement_config)
    if args.stage == "motion":
        build_preview(run, args.maximum_seconds, args.beam_width, args.resume)
    if args.stage == "collisions":
        audit_drawing_paths(run, args.collision_config, args.maximum_seconds)
    if args.stage == "sequence":
        plan_sequence(
            run, args.transit_config, args.collision_config, args.maximum_seconds, args.resume
        )
    if args.stage == "timing":
        build_schedule(run, args.timing_config)
    if args.stage == "validation":
        audit_smooth_paths(run, args.maximum_seconds)
    print(json.dumps({"run": str(run), "stages": read_json(run / "run.json")["stages"]}, indent=2))


if __name__ == "__main__":
    main()
