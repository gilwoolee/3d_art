"""Object-independent mesh-to-supported-strokes entry point.

The local path uses geometric contours. WIR3D jobs use the same downstream
graph/backbone modules after their provenance-checked import in pipeline.py.
With --cell-output it continues through the fixed-catalogue one-/two-arm planner.
Partial results are inspectable; no hardware commands are sent.
"""

import argparse
from pathlib import Path
from time import monotonic

import yaml

from .abstraction.geometric import extract_candidates
from .abstraction.sparse import sparse_run
from .backbone import backbone_run
from .graph import build_graph
from .io import artifact, read_json, sha256, stage_update, write_json
from .pipeline import initialize, prepare
from .planning.partition import partition_graph
from .process import screen_graph
from .strokes import pen_strokes
from .structure import analyze_structure


def build(
    source,
    run,
    source_up,
    max_extent_m=0.15,
    maximum_seconds=5,
    all_curves=False,
    *,
    cell_output=None,
    cell_profile="configs/cell/two_flexiv.yaml",
    timing_path="configs/timing_continuous_bounded.yaml",
    planning_seconds=600,
    resource_mode="dual",
    no_time_limit=False,
    single_arm_restarts=False,
):
    """Apply identical geometry and process settings to an arbitrary mesh."""
    if single_arm_restarts:
        from .planning.run_cell import validate_restart_options

        if cell_output is None:
            raise ValueError("--single-arm-restarts requires --cell-output")
        validate_restart_options(single_arm_restarts, resource_mode, no_time_limit, cell_profile)
    root = Path(__file__).resolve().parents[2]
    source, run = Path(source).resolve(), Path(run).resolve()
    if cell_output is not None:
        cell_output = Path(cell_output).resolve()
        if cell_output == run:
            raise ValueError("Use a separate cell output directory to preserve source geometry")
        if planning_seconds <= 0:
            raise ValueError("The robot planning budget must be positive")
    config = yaml.safe_load((root / "configs/model.yaml").read_text())
    config.update(
        name=source.stem,
        source=str(source),
        run_dir=str(run),
        source_up=source_up,
        max_extent_m=max_extent_m,
        curve_selection="all_geometric" if all_curves else "offline_sparse",
    )
    run.mkdir(parents=True, exist_ok=True)
    config_path = run / "input_config.yaml"
    if config_path.exists() and yaml.safe_load(config_path.read_text()) != config:
        raise ValueError("Input settings changed; use a new run directory")
    config_path.write_text(yaml.safe_dump(config, sort_keys=False))
    config, source, run = initialize(config_path)
    state = read_json(run / "run.json")
    state["name"] = f"{source.stem} · {'all curves' if all_curves else 'sparse'}"
    write_json(run / "run.json", state)
    started = monotonic()
    prepare(config, source, run)
    candidate = extract_candidates(run, config)
    stage_update(run, "geometric", "complete", artifact="geometric_candidates.json")
    fabrication = yaml.safe_load((root / "configs/fabrication.yaml").read_text())
    write_json(run / "fabrication_config.json", fabrication)
    source_file = "geometric_candidates.json"
    raw_count = len(candidate["curves"])
    if not all_curves:
        candidate = sparse_run(run)
        source_file = "selected_candidates.json"
        if not candidate["curves"]:
            raise ValueError(f"No selected sketch: {candidate['selection']['status']}")
    graph = build_graph(candidate, fabrication["graph"])
    graph["provenance"] = {
        "source_file": source_file,
        "source_sha256": sha256(run / source_file),
    }
    write_json(run / "stroke_graph.json", graph)
    strokes = pen_strokes(graph)
    write_json(run / "pen_strokes.json", strokes)
    write_json(run / "structural_screen.json", analyze_structure(graph, fabrication["structure"]))
    write_json(run / "process_screen.json", screen_graph(graph, fabrication["process"], strokes))
    stage_update(
        run,
        "graph",
        "complete",
        artifact="stroke_graph.json",
        source="geometric" if all_curves else "selected",
    )
    report = backbone_run(run, maximum_seconds)
    if (run / "backbone_construction.json").is_file() and report.get("complete_actions"):
        construction = read_json(run / "backbone_construction.json")
        write_json(run / "construction_plan.json", construction)
        stage_update(
            run,
            "construction",
            "partial",
            artifact="construction_plan.json",
            message="Intact contours with automatic start supports; physical checks pending",
        )
        cell = yaml.safe_load(
            (
                Path(cell_profile)
                if cell_output is not None
                else root / "configs/cell/two_flexiv.yaml"
            ).read_text()
        )
        if len(cell["arms"]) == 2:
            partitioned = partition_graph(read_json(run / "backbone_graph.json"), cell)
            write_json(run / "backbone_partition.json", partitioned)
    summary = artifact(
        "mesh_pipeline_result",
        source=source.name,
        curve_method=candidate["method"],
        inputs={
            name: sha256(run / name)
            for name in (
                "input_config.yaml",
                "geometric_candidates.json",
                "fabrication_config.json",
                source_file,
            )
        },
        elapsed_s=monotonic() - started,
        original_contours=raw_count,
        selected_contours=len(candidate["curves"]),
        complete_actions=report.get("complete_actions", 0),
        added_support_length_m=report.get("added_length_m"),
        graph_connection_length_m=graph["metrics"]["added_length_m"],
        total_drawing_length_m=(
            read_json(run / "backbone_graph.json")["metrics"]["total_length_m"]
            if report.get("complete_actions")
            else None
        ),
        bounds_m=read_json(run / "asset_manifest.json")["bounds_m"],
        intact_contour_order=report.get("whole_contours_preserved", False),
        cold_structure_status=report.get("structure_status"),
        robot_motion_status="NOT_PLANNED",
        physical_status="UNVERIFIED",
        limitations=[
            "Geometric contours have no texture-aware recognition optimization",
            "Multi-part geometry supported; multi-material WIR3D requires atlas baking",
            "Order is checked at completed stroke boundaries, not along hot extrusion",
            "Arm regions are proposals, not collision-checked trajectories",
        ],
    )
    if not all_curves and report.get("complete_actions"):
        selection = candidate["selection"]
        process = fabrication["process"]
        partial_s = summary["total_drawing_length_m"] / process["nominal_speed_m_s"] + report[
            "complete_actions"
        ] * (process["start_lag_s"] + process["stop_lag_s"])
        summary["timing_assessment"] = {
            "scope": "connected drawing plus feed delays; excludes travel, joins, cooling",
            "assumed_speed_m_s": process["nominal_speed_m_s"],
            "partial_seconds": partial_s,
            "target_seconds": selection["target_seconds"],
            "remaining_seconds": selection["target_seconds"] - partial_s,
            "fits_with_reserved_overhead": partial_s + selection["overhead_reserve_s"]
            <= selection["target_seconds"],
            "complete_schedule": False,
        }
    write_json(run / "pipeline_result.json", summary)
    if cell_output is not None:
        if not report.get("complete_actions"):
            summary["robot_motion_status"] = "NO_SUPPORTED_CONSTRUCTION_ORDER"
        else:
            from .planning.run_cell import plan_cell

            summary.update(robot_motion_status="PLANNING", cell_run=str(cell_output))
            write_json(run / "pipeline_result.json", summary)
            try:
                planned = plan_cell(
                    run,
                    cell_output,
                    cell_profile,
                    maximum_seconds=planning_seconds,
                    timing_path=timing_path,
                    resource_mode=resource_mode,
                    no_time_limit=no_time_limit,
                    single_arm_restarts=single_arm_restarts,
                )
            except Exception as error:
                summary.update(
                    robot_motion_status="PIPELINE_ERROR",
                    planning_error=str(error),
                    elapsed_s=monotonic() - started,
                )
                write_json(run / "pipeline_result.json", summary)
                raise
            summary["robot_motion_status"] = planned["status"]
            summary["cell_result_sha256"] = sha256(cell_output / "cell_pipeline_result.json")
        summary["elapsed_s"] = monotonic() - started
        write_json(run / "pipeline_result.json", summary)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--source-up", choices=["y", "z"], required=True)
    parser.add_argument("--max-extent-m", type=float, default=0.15)
    parser.add_argument(
        "--all-curves",
        action="store_true",
        help="Keep every geometric candidate as a dense baseline",
    )
    parser.add_argument(
        "--maximum-seconds",
        type=float,
        default=5,
        help="Fixed support-allocation solver limit, not total pipeline time",
    )
    parser.add_argument(
        "--cell-output",
        type=Path,
        help="Continue with one-/two-arm planning in this separate run directory",
    )
    parser.add_argument("--cell-config", type=Path, default=Path("configs/cell/two_flexiv.yaml"))
    parser.add_argument(
        "--timing-config", type=Path, default=Path("configs/timing_continuous_bounded.yaml")
    )
    parser.add_argument("--planning-seconds", type=float, default=600)
    parser.add_argument("--resource-mode", choices=["single", "dual"], default="dual")
    parser.add_argument(
        "--no-time-limit",
        action="store_true",
        help="Remove overall robot-planning and catalogue timeouts",
    )
    parser.add_argument(
        "--single-arm-restarts",
        action="store_true",
        help="Use supported same-point restarts for the full one-arm build; "
        "requires --cell-output, --resource-mode single, --no-time-limit, and a one-arm profile",
    )
    args = parser.parse_args()
    print(
        build(
            args.source,
            args.run,
            args.source_up,
            args.max_extent_m,
            args.maximum_seconds,
            args.all_curves,
            cell_output=args.cell_output,
            cell_profile=args.cell_config,
            timing_path=args.timing_config,
            planning_seconds=args.planning_seconds,
            resource_mode=args.resource_mode,
            no_time_limit=args.no_time_limit,
            single_arm_restarts=args.single_arm_restarts,
        )
    )


if __name__ == "__main__":
    main()
