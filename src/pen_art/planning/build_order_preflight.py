"""Check the fixed goal graph before Caelan-style construction-order search.

This is a prerequisite check, not a construction order or robot feasibility
certificate. It never modifies the source graph or an accepted build prefix.
"""

import argparse
from pathlib import Path

import numpy as np

from ..io import artifact, now, read_json, sha256, write_json
from ..structure import analyze_structure


def preflight(run, graph_file="stroke_graph.json"):
    run = Path(run)
    graph_path = run / graph_file
    graph = read_json(graph_path)
    config_path = run / "fabrication_config.json"
    material = read_json(config_path)["structure"]
    screen = analyze_structure(graph, material)
    numerical = screen.get("failure") in {
        "SINGULAR_STRUCTURE", "ILL_CONDITIONED_STRUCTURE"
    }
    status = (
        "READY_FOR_ORDER_SEARCH" if screen["modeled_pass"] else
        "STRUCTURAL_MODEL_UNRESOLVED" if numerical else "GOAL_STRUCTURE_REJECTED"
    )
    scalars = {
        key: value for key, value in screen.items()
        if key not in {"nodes", "edges"}
    }
    worst = sorted(
        screen.get("nodes", []),
        key=lambda row: np.linalg.norm(row["displacement_m"]), reverse=True,
    )[:5]
    return artifact(
        "build_order_preflight", created_at=now(), status=status,
        inputs={graph_file: sha256(graph_path), "fabrication_config.json": sha256(config_path)},
        graph_file=graph_file, element_count=len(graph["edges"]),
        ground_node_count=sum(bool(node["ground"]) for node in graph["nodes"]),
        final_structure=scalars, largest_nodal_displacements=worst,
        smallest_members=[
            {"id": edge["id"], "length_m": edge["length_m"]}
            for edge in sorted(graph["edges"], key=lambda row: row["length_m"])[:5]
        ],
        actions=[], robot_feasible=None, pen_feasible=None, simulation_only=True,
        interpretation=(
            "Fixed final graph passes the configured cooled-frame model; search must still "
            "check every construction prefix, extrusion and pen-off transit."
            if screen["modeled_pass"] else
            "The numerical structural result is unresolved; do not infer physical "
            "infeasibility or search an order using this result."
            if numerical else
            "The fixed final graph fails the configured model. Reordering cannot change "
            "its final equilibrium. Validate the model and revise explicit supports before "
            "searching this graph; this is not a calibrated physical impossibility claim."
        ),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--graph", default="stroke_graph.json")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = preflight(args.run, args.graph)
    destination = args.output or args.run / "build_order" / "preflight.json"
    write_json(destination, report)
    print(f"{report['status']}: {destination}", flush=True)


if __name__ == "__main__":
    main()
