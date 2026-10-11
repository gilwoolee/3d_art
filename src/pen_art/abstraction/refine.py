"""Refine a completed mesh/curve bundle without rerunning GPU abstraction."""

import argparse
import shutil
from pathlib import Path

import yaml

from ..backbone import backbone_run
from ..graph import build_graph
from ..io import artifact, now, read_json, sha256, stage_update, write_json
from ..planning.partition import partition_graph
from ..process import screen_graph
from ..strokes import pen_strokes
from ..structure import analyze_structure
from .comparison import compare_run
from .contours import contour_run
from .selection import raw_sources
from .sparse import sparse_run
from .wir3d_adapter import selected_candidate_current


def refine(
    source, output, *, contours=False, curves_only=False, contour_config=None, selection_config=None
):
    contours = contours or contour_config is not None
    source, output = Path(source).resolve(), Path(output).resolve()
    if source == output or output.exists():
        raise ValueError("Use a new output directory to preserve existing experiments")
    root = Path(__file__).resolve().parents[3]
    original = read_json(source / "run.json")
    names = [
        "asset_manifest.json",
        "normalized.glb",
        "geometry.glb",
        "geometry.npz",
        "fabrication_config.json",
        *[
            name
            for name in raw_sources(source)
            if not (contours and name == "surface_contours.json")
        ],
    ]
    if (source / "wir3d_keypoints.json").exists():
        names.append("wir3d_keypoints.json")
    names.extend(p.name for p in source.glob("wir3d_reference_*.png"))
    names.extend(p.name for p in source.glob("wir3d_sketch_*.png"))
    fingerprints = {name: sha256(source / name) for name in names}
    output.mkdir(parents=True)
    for name in names:
        shutil.copyfile(source / name, output / name)
    write_json(
        output / "source_bundle.json",
        artifact("abstraction_source", source_run=str(source), inputs=fingerprints),
    )
    stages = {
        name: {"status": "pending"}
        for name in (
            "asset",
            "geometric",
            "wir3d",
            "selection",
            "graph",
            "construction",
            "cell",
            "motion",
            "timing",
            "validation",
        )
    }
    for name in ("asset", "geometric", "wir3d"):
        stages[name] = {**original["stages"][name], "source_run": source.name}
    stages["wir3d"].update(
        status="complete",
        message="Frozen imported candidate bundle; GPU job unchanged",
        budgets=[
            int(Path(n).stem.split("_")[1]) for n in raw_sources(source) if n.startswith("wir3d_")
        ],
    )
    write_json(
        output / "run.json",
        artifact(
            "run",
            id=output.name,
            name=original["name"].split(" · ")[0]
            + (" · surface contours" if contours else " · visible-feature selection"),
            created_at=now(),
            updated_at=now(),
            source_sha256=original["source_sha256"],
            stages=stages,
        ),
    )
    if contours:
        contour_run(
            output, yaml.safe_load((contour_config or root / "configs/contours.yaml").read_text())
        )
    chosen = sparse_run(output, settings_path=selection_config)
    if curves_only:
        return chosen
    return construct_selected(output)


def construct_selected(output):
    """Connect the exact reviewed selection without rerunning its solver."""
    output = Path(output).resolve()
    root = Path(__file__).resolve().parents[3]
    if not selected_candidate_current(output):
        raise ValueError("Selection is missing or stale; generate a current selection first")
    if (output / "stroke_graph.json").exists():
        raise ValueError("Structure already exists; preserve this experiment and use a new run")
    chosen = read_json(output / "selected_candidates.json")
    fabrication = read_json(output / "fabrication_config.json")
    if not chosen["curves"]:
        return chosen
    graph = build_graph(chosen, fabrication["graph"])
    graph["provenance"] = {
        "source_file": "selected_candidates.json",
        "source_sha256": sha256(output / "selected_candidates.json"),
    }
    strokes = pen_strokes(graph)
    write_json(output / "stroke_graph.json", graph)
    write_json(output / "pen_strokes.json", strokes)
    write_json(
        output / "structural_screen.json", analyze_structure(graph, fabrication["structure"])
    )
    write_json(output / "process_screen.json", screen_graph(graph, fabrication["process"], strokes))
    stage_update(output, "graph", "complete", artifact="stroke_graph.json", source="selected")
    support = backbone_run(output, 5)
    if support.get("complete_actions"):
        write_json(
            output / "construction_plan.json", read_json(output / "backbone_construction.json")
        )
        cell = yaml.safe_load((root / "configs/cell/two_flexiv.yaml").read_text())
        write_json(
            output / "backbone_partition.json",
            partition_graph(read_json(output / "backbone_graph.json"), cell),
        )
        stage_update(
            output,
            "construction",
            "partial",
            artifact="construction_plan.json",
            message="Whole-contour order; pen, stiffness and robot validation pending",
        )
        supported = read_json(output / "backbone_graph.json")
        process = fabrication["process"]
        partial_seconds = supported["metrics"]["total_length_m"] / process["nominal_speed_m_s"]
        partial_seconds += support["complete_actions"] * (
            process["start_lag_s"] + process["stop_lag_s"]
        )
        target_seconds = chosen["selection"]["target_seconds"]
        resources = chosen["selection"].get("drawing_resources", 1)
        construction = read_json(output / "backbone_construction.json")
        longest_action = max(a["length_m"] for a in construction["actions"])
        longest_seconds = longest_action / process["nominal_speed_m_s"]
        longest_seconds += process["start_lag_s"] + process["stop_lag_s"]
        write_json(
            output / "pipeline_result.json",
            artifact(
                "mesh_pipeline_result",
                inputs={
                    name: sha256(output / name)
                    for name in (
                        "selected_candidates.json",
                        "backbone_graph.json",
                        "backbone_construction.json",
                        "fabrication_config.json",
                    )
                },
                total_drawing_length_m=supported["metrics"]["total_length_m"],
                complete_actions=support["complete_actions"],
                intact_contour_order=True,
                timing_assessment={
                    "scope": "aggregate drawing and feed work; excludes travel, joins, cooling",
                    "drawing_resources": resources,
                    "aggregate_work_seconds": partial_seconds,
                    "parallel_lower_bound_seconds": max(
                        partial_seconds / resources, longest_seconds
                    ),
                    "partial_seconds": partial_seconds if resources == 1 else None,
                    "target_seconds": target_seconds,
                    "remaining_seconds": target_seconds - partial_seconds
                    if resources == 1
                    else None,
                    "wall_time_status": "NOT_SCHEDULED",
                    "complete_schedule": False,
                },
                physical_status="UNVERIFIED",
                robot_motion_status="NOT_PLANNED",
                cold_structure_status=support.get("structure_status"),
            ),
        )
    write_json(
        output / "comparison_config.json",
        {
            "comparison": yaml.safe_load((root / "configs/comparison.yaml").read_text()),
            "fabrication": fabrication,
            "visible_edges": read_json(output / "selection_config.json")["selection"].get(
                "visible_edges"
            ),
        },
    )
    compare_run(output)
    return chosen


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-run", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--structure-only",
        action="store_true",
        help="Connect the saved selection without rerunning it",
    )
    parser.add_argument("--selection-config", type=Path, help="Fixed curve-selection settings")
    parser.add_argument(
        "--contour-config", type=Path, help="Image or connected-mesh contour settings"
    )
    parser.add_argument(
        "--contours", action="store_true", help="Add lifted surface contour candidates"
    )
    parser.add_argument(
        "--curves-only", action="store_true", help="Stop for visual abstraction review"
    )
    args = parser.parse_args()
    if args.structure_only:
        if (
            args.source_run
            or args.contours
            or args.curves_only
            or args.contour_config
            or args.selection_config
        ):
            parser.error("--structure-only uses only --output")
        result = construct_selected(args.output)
        print(result["label"], result["selection"]["status"])
        return
    if args.source_run is None:
        parser.error("--source-run is required unless --structure-only is used")
    result = refine(
        args.source_run,
        args.output,
        contours=args.contours,
        curves_only=args.curves_only,
        contour_config=args.contour_config,
        selection_config=args.selection_config,
    )
    print(result["label"], result["selection"]["status"])


if __name__ == "__main__":
    main()
