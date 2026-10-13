"""Regenerate connectors with nozzle-aware anchor spacing; retain contours/style."""

import argparse
import shutil
from collections import defaultdict
from pathlib import Path

import numpy as np
from shapely.geometry import LineString
from shapely.ops import unary_union

from pen_art.abstraction.hybrid import attach_style, select_printable_style
from pen_art.abstraction.wireprint import connect_framework_islands, generate, reinforce_base
from pen_art.cell import prepare_cell
from pen_art.curves import curve_coordinate_bounds
from pen_art.io import now, read_json, sha256, write_json
from pen_art.planning.section_build import prepare_candidates, write_section_artifacts
from pen_art.planning.sections import section_actions
from pen_art.provenance import audit_source_provenance
from pen_art.structure import analyze_structure

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--framework-diameter-mm", type=float, default=1.0)
args = parser.parse_args()
diameter = args.framework_diameter_mm / 1000
assert 0 < diameter < 0.01
suffix = "" if diameter == 0.001 else f"_{round(diameter * 1_000_000)}um"
original = Path("runs/palace_hybrid_v7_source")
source = Path(f"runs/palace_hybrid_spaced{suffix}_source")
run = Path(f"runs/palace_hybrid_spaced{suffix}_v1")
assert not source.exists() and not run.exists()
source.mkdir()
for name in (
    "run.json",
    "asset_manifest.json",
    "normalized.glb",
    "geometry.glb",
    "geometry.npz",
    "fabrication_config.json",
    "wir3d_20.json",
):
    shutil.copy2(original / name, source / name)
metadata = read_json(source / "run.json")
metadata.update(id=source.name, updated_at=now())
write_json(source / "run.json", metadata)

# Shank radius + strand radius + clearance + material/tracking enclosure.
# This is a spacing proposal under the provisional tool model, not calibration.
spacing = 0.0025 + diameter / 2 + 0.001 + 0.00035
config = read_json(source / "fabrication_config.json")
config["graph"]["strand_diameter_m"] = diameter
write_json(source / "fabrication_config.json", config)
graph = generate(source, diameter=diameter, minimum_anchor_spacing=spacing)
old = read_json(original / "wireprint_graph.json")


def contours(g):
    lines = defaultdict(list)
    for edge in g["edges"]:
        if edge.get("parent_curve", "").startswith("contour-"):
            for segment in edge["segments"]:
                lines[edge["parent_curve"]].append(LineString(segment))
    return {key: unary_union(value) for key, value in lines.items()}


before, after = contours(old), contours(graph)
assert before.keys() == after.keys()
differences = {key: before[key].hausdorff_distance(after[key]) for key in before}
assert max(differences.values()) <= 0.00004, max(differences.values())
graph = connect_framework_islands(graph)
material = read_json(source / "fabrication_config.json")["structure"]
initial = analyze_structure(graph, material)
if not initial["modeled_pass"] and "maximum_displacement_m" in initial:
    graph = reinforce_base(
        graph,
        {
            "initial_maximum_displacement_m": initial["maximum_displacement_m"],
            "initial_maximum_stress_pa": initial["maximum_stress_pa"],
        },
    )
write_json(source / "framework_graph.json", graph)
curves, selection = select_printable_style(read_json(source / "wir3d_20.json")["curves"])
assert selection["selected"] == read_json(original / "style_selection.json")["selected"]
write_json(source / "style_selection.json", selection)
graph, attachments = attach_style(graph, curves)
write_json(source / "style_attachments.json", attachments)
graph.setdefault("inputs", {}).update(
    {
        name: sha256(source / name)
        for name in (
            "framework_graph.json",
            "wir3d_20.json",
            "style_selection.json",
            "style_attachments.json",
        )
    }
)
construction = section_actions(graph, group_sources=True)
write_json(source / "stroke_graph.json", graph)
write_json(source / "construction_plan.json", construction)
write_section_artifacts(source, graph, construction)
graph = read_json(source / "stroke_graph.json")
screen = read_json(source / "structural_screen.json")
bounds = [curve_coordinate_bounds(edge) for edge in graph["edges"]]
extents = np.max(
    [b + e["diameter_m"] / 2 for (_, b), e in zip(bounds, graph["edges"], strict=True)], axis=0
) - np.min(
    [a - e["diameter_m"] / 2 for (a, _), e in zip(bounds, graph["edges"], strict=True)], axis=0
)
assert np.all(extents <= 0.15 + 1e-9)
audit = audit_source_provenance(construction["actions"], graph)
report = {
    "status": "FINAL_STRUCTURE_PASSED" if screen["modeled_pass"] else "FINAL_STRUCTURE_REJECTED",
    "at": now(),
    "source": str(source),
    "run": str(run),
    "minimum_anchor_spacing_m": spacing,
    "framework_diameter_m": diameter,
    "spacing_scope": (
        "Distinct endpoints within each paired contour; full nozzle/robot clearance still required."
    ),
    "maximum_contour_hausdorff_distance_m": max(differences.values()),
    "selected_style_curves": selection["selected"],
    "physical_members": len(graph["edges"]),
    "sections": len(construction["actions"]),
    "material_extent_bound_m": extents.tolist(),
    "source_provenance": audit,
    "structure": screen,
    "simulation_only": True,
    "inputs": {
        str(path): sha256(path)
        for path in (
            Path(__file__),
            Path("src/pen_art/abstraction/wireprint.py"),
            original / "normalized.glb",
            source / "normalized.glb",
            source / "stroke_graph.json",
            source / "construction_plan.json",
        )
    },
}
write_json(Path(f"research/blondie/palace_spaced_framework{suffix}.json"), report)
print(report["status"], report["physical_members"], report["sections"], flush=True)
if screen["modeled_pass"]:
    prepare_cell(Path("research/planning_cell/cell.yaml"), run, source)
    prepare_candidates(run)
    write_json(
        run / "section_build.json",
        {
            "status": "PREPARED_PENDING_ROBOT_VALIDATION",
            "completed_sections": 0,
            "required_sections": len(construction["actions"]),
            "best_prefix": [],
            "simulation_only": True,
        },
    )
    write_json(run / "trial_reason.json", report)
