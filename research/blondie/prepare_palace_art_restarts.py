"""Keep the exact artwork but allow pen-off reorientation between short sections."""

import shutil
from copy import deepcopy
from pathlib import Path

import numpy as np

from pen_art.cell import prepare_cell
from pen_art.curves import bezier_at, curve_length
from pen_art.io import now, read_json, sha256, write_json
from pen_art.planning.section_build import prepare_candidates, write_section_artifacts
from pen_art.planning.sections import section_actions, split_member_for_restart
from pen_art.provenance import audit_source_provenance

original = Path("runs/palace_hybrid_spaced_1200um_source")
source = Path("runs/palace_hybrid_art_restarts_source")
run = Path("runs/palace_hybrid_art_restarts_v1")
assert not source.exists() and not run.exists()
graph = read_json(original / "stroke_graph.json")
before = deepcopy(graph)
old_rows = read_json(original / "construction_plan.json")["actions"]
arts = [row for row in before["edges"] if row["stage"] == "artistic"]
cuts = []
for old in arts:
    assert len(old["segments"]) == 1
    count = max(1, int(np.ceil(curve_length(old) / 0.0025)))
    key = old["id"]
    for part in range(1, count):
        graph = split_member_for_restart(graph, key, 1 / (count - part + 1))
        key = graph["restart_boundaries"][-1]["pieces"][1]
    cuts.append({"source": old["parent_curve"], "parts": count})
old_edges = {edge["id"]: edge for edge in before["edges"]}
for edge in graph["edges"]:
    if edge["stage"] != "artistic":
        assert edge == old_edges[edge["id"]]
old_by_source = {row["parent_curve"]: row for row in arts}
error = 0.0
for edge in graph["edges"]:
    if edge["stage"] != "artistic":
        continue
    old = old_by_source[edge["parent_curve"]]
    lo, hi = edge["source_interval"]
    a, b = old["source_interval"]
    for t in np.linspace(0, 1, 101):
        expected = bezier_at(old["segments"][0], (lo + t * (hi - lo) - a) / (b - a))
        error = max(error, float(np.linalg.norm(bezier_at(edge["segments"][0], t) - expected)))
assert error < 1e-12
assert all(not row["ground"] for row in graph["nodes"][len(before["nodes"]) :])
construction = section_actions(graph, group_sources=True)
used = set()
for old in old_rows:
    lo, hi = old["source_interval"]
    parts = sorted(
        [
            row
            for row in construction["actions"]
            if row["parent_curve"] == old["parent_curve"]
            and lo - 1e-10 <= row["source_interval"][0]
            and row["source_interval"][1] <= hi + 1e-10
        ],
        key=lambda row: row["source_interval"][0],
    )
    assert parts
    for index, row in enumerate(parts):
        assert id(row) not in used
        used.add(id(row))
        row["id"] = old["id"] if len(parts) == 1 else f"{old['id']}-part-{index}"
        row["priority"] = old["priority"] + index / len(parts)
assert len(used) == len(construction["actions"])
audit = audit_source_provenance(construction["actions"], graph)
shutil.copytree(original, source)
metadata = read_json(source / "run.json")
metadata.update(id=source.name, updated_at=now())
write_json(source / "run.json", metadata)
write_json(source / "stroke_graph.json", graph)
write_json(source / "construction_plan.json", construction)
write_section_artifacts(source, graph, construction)
screen = read_json(source / "structural_screen.json")
report = {
    "status": "EXACT_RESTARTS_FINAL_SCREEN_PASSED"
    if screen["modeled_pass"]
    else "FINAL_SCREEN_REJECTED",
    "run": str(run),
    "source": str(source),
    "splits": cuts,
    "maximum_curve_difference_m": error,
    "added_ground_anchors": 0,
    "added_material": False,
    "new_nodes": len(graph["nodes"]) - len(before["nodes"]),
    "sections": len(construction["actions"]),
    "source_provenance": audit,
    "maximum_displacement_m": screen.get("maximum_displacement_m"),
    "simulation_only": True,
    "inputs": {
        str(path): sha256(path)
        for path in (
            original / "stroke_graph.json",
            source / "stroke_graph.json",
            source / "construction_plan.json",
            source / "structural_screen.json",
        )
    },
}
write_json(Path("research/blondie/palace_art_restarts.json"), report)
print(report["status"], report["sections"], "sections; curve difference", error, flush=True)
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
