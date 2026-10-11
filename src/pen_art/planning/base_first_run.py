"""Fork a selected one-arm build into a stable ground-base-first order from empty base."""

import argparse
import shutil
from pathlib import Path

from ..cell import prepare_cell
from ..io import artifact, read_json, sha256, write_json
from .cell_allocation import build_problem
from .construction_order import ground_base_actions, stable_ground_base_order
from .resource_schedule import solve_allocation


def fork_base_first(source, output, profile, timing):
    source, output = Path(source), Path(output)
    if output.exists():
        raise ValueError("Use a new output directory; existing evidence must remain unchanged")
    old = read_json(source / "cell_allocation.json")
    selection = old["results"]["single"]["actions"]
    if len({row["arm"] for row in selection}) != 1:
        raise ValueError("A selected single-arm allocation is required")
    construction = read_json(source / "construction_plan.json")
    graph = read_json(source / "stroke_graph.json")
    ordered = stable_ground_base_order(selection, construction, graph)
    state = read_json(source / "run.json")
    prepare_cell(
        profile, output, state["cell_source_run"], state.get("cell_source_variant", "original")
    )
    preserved = ["construction_plan.json", "stroke_graph.json", "cell_drawing_variants.json"]
    for name in preserved:
        if name == "cell_drawing_variants.json":
            shutil.copy2(source / name, output / name)
        elif sha256(source / name) != sha256(output / name):
            raise ValueError(f"Source geometry changed: {name}")
    problem = build_problem(output, profile, deadline=float("inf"), timing_path=timing)
    chosen = {row["mode_id"] for row in ordered}
    problem["forbidden_modes"] = sorted(
        set(problem.get("forbidden_modes", [])) | {row["id"] for row in problem["modes"]} - chosen
    )
    problem["precedence_modes"] = list(problem.get("precedence_modes", [])) + [
        [a["mode_id"], b["mode_id"]] for a, b in zip(ordered[:-1], ordered[1:], strict=True)
    ]
    provenance = artifact(
        "construction_order_provenance",
        policy="stable_dependency_valid_ground_base_first",
        source_run=str(source.resolve()),
        source_allocation_sha256=sha256(source / "cell_allocation.json"),
        source_identity={name: sha256(source / name) for name in preserved},
        prior_order=[row["stroke_id"] for row in selection],
        selected_order=[row["stroke_id"] for row in ordered],
        ground_base_actions=sorted(ground_base_actions(construction, graph)),
        starts_from_empty_base=True,
        reused_motion_count=0,
        geometry_changed=False,
    )
    write_json(output / "ordering_provenance.json", provenance)
    problem["inputs"]["ordering_provenance.json"] = sha256(output / "ordering_provenance.json")
    write_json(output / "allocation_problem.json", problem)
    result = solve_allocation(problem, enabled_arms=[selection[0]["arm"]], maximum_seconds=15)
    if (
        [row["stroke_id"] for row in result["actions"]] != provenance["selected_order"]
        or [row["mode_id"] for row in result["actions"]] != [row["mode_id"] for row in ordered]
    ):
        raise ValueError(f"Fixed-order dependency allocation failed: {result['status']}")
    allocation = {
        **old,
        "inputs": {"allocation_problem.json": sha256(output / "allocation_problem.json")},
        "results": {"single": result, "dual": result},
        "estimated_speedup": None,
        "comparison_scope": "Single arm, same source geometry and selected modes; base-first order",
    }
    write_json(output / "cell_allocation.json", allocation)
    run_state = read_json(output / "run.json")
    run_state.update(resource_mode="single", name=state["name"] + " · base first")
    write_json(output / "run.json", run_state)
    return provenance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument(
        "--timing-config", type=Path, default=Path("configs/timing_continuous_bounded.yaml")
    )
    args = parser.parse_args()
    result = fork_base_first(args.source_run, args.output, args.profile, args.timing_config)
    print("Prepared base-first order:", ", ".join(result["selected_order"]), flush=True)


if __name__ == "__main__":
    main()
