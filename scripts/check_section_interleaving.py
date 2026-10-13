"""Real Flexiv export/replay of A[0,.5], B, A[.5,1], plus tampering rejections."""

from copy import deepcopy
from pathlib import Path

import numpy as np
import trimesh

from pen_art.graph import straight_edge
from pen_art.io import artifact, read_json, sha256, stage_update, write_json
from pen_art.planning.cell_replay import replay_cell
from pen_art.planning.section_build import SectionBuild, prepare_reference

fixture = Path("runs/interleaved_reference")
fixture.mkdir(exist_ok=True)
positions = [[0, 0, 0], [0, 0, 0.03], [0.015, 0, 0.03], [0, 0, 0.06]]
nodes = {
    f"n{i}": {"id": f"n{i}", "position": point, "ground": i == 0}
    for i, point in enumerate(positions)
}
edges, cylinders = [], []
for i, (start, end, parent, interval) in enumerate(
    [
        ("n0", "n1", "A", [0, 0.5]),
        ("n1", "n2", "B", [0, 1]),
        ("n1", "n3", "A", [0.5, 1]),
    ]
):
    edge = straight_edge(start, end, nodes, f"edge{i}", "framework", 0.001)
    edge.update(parent_curve=parent, source_interval=interval, order_hint=i)
    edges.append(edge)
    segment = np.array([nodes[start]["position"], nodes[end]["position"]])
    cylinders.append(trimesh.creation.cylinder(radius=0.0005, segment=segment))
mesh = trimesh.util.concatenate(cylinders)
mesh.apply_scale(1000)
mesh.export(fixture / "model_mm.stl")
write_json(
    fixture / "stroke_graph.json", artifact("stroke_graph", nodes=list(nodes.values()), edges=edges)
)
write_json(
    fixture / "manifest.json",
    {
        "name": "Interleaved source regression",
        "source": "model_mm.stl",
        "source_up": "z",
        "max_extent_m": float(np.ptp(mesh.vertices, axis=0).max()) / 1000,
    },
)
write_json(
    fixture / "fabrication_config.json",
    read_json(Path("research/primitive_fixtures/cube/fabrication_config.json")),
)
run = Path("runs/interleaved_sections_v1")
profile = Path("research/planning_cell/cell.yaml")
prepare_reference(fixture, run, profile)
if (
    not (run / "section_build.json").is_file()
    or read_json(run / "section_build.json")["status"] != "FULL_BUILD_VERIFIED"
):
    build = SectionBuild(run, profile, 1800, gpu=True)
    build.run_build()
state = read_json(run / "section_build.json")
assert state["status"] == "FULL_BUILD_VERIFIED", state
plan = read_json(run / "offline_dual_arm_plan.json")
assert [r["parent_curve"] for r in plan["actions"] if not r["auxiliary"]] == ["A", "B", "A"]
rejections = []
for label in ("gap", "overlap", "cooling", "stage", "junctions", "events"):
    damaged = deepcopy(plan)
    if label == "gap":
        damaged["actions"][2]["source_interval"] = [0.6, 1]
    elif label == "overlap":
        damaged["actions"][2]["source_interval"] = [0.4, 1]
    elif label == "cooling":
        damaged["actions"][1]["start_s"] = damaged["actions"][0]["end_s"]
    elif label == "stage":
        damaged["actions"][1]["stage"] = "artistic"
    elif label == "junctions":
        damaged["actions"][0]["junction_visits"] = [{"node": "invented-anchor"}]
    else:
        damaged["events"] = damaged["events"][1:]
    name = f"rejected_{label}_plan"
    write_json(run / (name + ".json"), damaged)
    try:
        result = replay_cell(run, profile, 300, name)
    except ValueError:
        result = read_json(run / f"rejected_{label}_validation.json")
    assert result["status"] not in ("CERTIFIED_GEOMETRIC_REPLAY", "CERTIFIED_PREFIX_REPLAY"), result
    rejections.append({"case": label, "status": result["status"], "failures": result["failures"]})
write_json(
    run / "interleaving_regression.json",
    {
        "status": "PASSED",
        "verified_plan_sha256": sha256(run / "offline_dual_arm_plan.json"),
        "source_order": ["A[0,.5]", "B", "A[.5,1]"],
        "rejections": rejections,
    },
)
print("Interleaving export, robot replay, and rejection cases PASSED", flush=True)
stage_update(
    run,
    "validation",
    "complete",
    artifact="offline_dual_arm_validation.json",
    message="Complete interleaved-source replay and negative checks passed",
)
