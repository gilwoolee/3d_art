"""Try a support-ready order on the unchanged spaced Palace graph."""

import argparse
from copy import deepcopy
from pathlib import Path
from time import monotonic

from pen_art.cell import prepare_cell
from pen_art.io import now, read_json, sha256, write_json
from pen_art.planning.section_build import prepare_candidates, write_section_artifacts
from pen_art.provenance import audit_source_provenance
from pen_art.structure import analyze_structure

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--resume", action="store_true")
parser.add_argument("--maximum-seconds", type=float, default=300)
args = parser.parse_args()
source = Path("runs/palace_hybrid_spaced_1200um_source")
run = Path("runs/palace_hybrid_supported_order_v1")
assert not run.exists()
graph = read_json(source / "stroke_graph.json")
construction = read_json(source / "construction_plan.json")
original = deepcopy(construction)
material = read_json(source / "fabrication_config.json")["structure"]
remaining = {row["id"]: row for row in construction["actions"]}
available = {node["id"] for node in graph["nodes"] if node["ground"]}
members, ordered, releases = set(), [], []
deadline = monotonic() + args.maximum_seconds
result = Path("research/blondie/palace_supported_order.json")
inputs = {
    str(p): sha256(p)
    for p in (
        source / "stroke_graph.json",
        source / "construction_plan.json",
        source / "fabrication_config.json",
    )
}
resumed = 0
if args.resume:
    saved = read_json(result)
    assert saved["inputs"] == inputs, "Saved order belongs to different geometry or material"
    assert len(saved["order"]) == len(set(saved["order"])) == len(saved["releases"])
    for key, release in zip(saved["order"], saved["releases"], strict=True):
        row = remaining.pop(key)
        assert release["section"] == key
        assert row["stage"] == "framework" or not any(
            candidate["stage"] == "framework" for candidate in remaining.values()
        )
        ordered.append(row)
        members.update(row["members"])
        available.update(visit["node"] for visit in row["junction_visits"])
        available.update((row["start"], row["end"]))
    releases = saved["releases"]
    resumed = len(ordered)
    print(
        "Resumed",
        resumed,
        "structural proposals; full robot validation remains required",
        flush=True,
    )
elif result.exists():
    raise ValueError("An ordering attempt exists; inspect it and use --resume")


def save(status):
    write_json(
        result,
        {
            "status": status,
            "at": now(),
            "source": str(source),
            "run": str(run),
            "ordered_sections": len(ordered),
            "required_sections": len(construction["actions"]),
            "order": [row["id"] for row in ordered],
            "releases": releases,
            "simulation_only": True,
            "resumed_structural_proposals": resumed,
            "scope": (
                "Provisional structural release order; every robot motion still needs validation."
            ),
            "inputs": inputs,
        },
    )


while remaining and monotonic() < deadline:
    base_pending = any(row["stage"] == "framework" for row in remaining.values())
    candidates = sorted(remaining.values(), key=lambda row: (row["priority"], row["id"]))
    for row in candidates:
        if monotonic() >= deadline:
            break
        if base_pending and row["stage"] != "framework":
            continue
        anchors = {row["start"], row["end"]}
        if row["closed"]:
            anchors.update(visit["node"] for visit in row["junction_visits"])
        if not anchors.intersection(available):
            continue
        screen = analyze_structure(graph, material, edge_ids=members | set(row["members"]))
        if not screen["modeled_pass"]:
            continue
        ordered.append(row)
        members.update(row["members"])
        available.update(visit["node"] for visit in row["junction_visits"])
        available.update((row["start"], row["end"]))
        del remaining[row["id"]]
        releases.append(
            {
                "section": row["id"],
                "maximum_displacement_m": screen.get("maximum_displacement_m", 0.0),
            }
        )
        if len(ordered) % 20 == 0:
            save("ORDERING")
            print("Structurally ordered", len(ordered), flush=True)
        break
    else:
        save("NO_READY_STABLE_SECTION")
        raise SystemExit(
            "No ready stable section in this greedy order; other orders remain possible"
        )
if remaining:
    save("ORDERING_TIME_BUDGET")
    raise SystemExit("Structural-order time budget; no full order claimed")
for priority, row in enumerate(ordered):
    row["priority"] = priority
for a, b in zip(original["actions"], construction["actions"], strict=True):
    assert {k: v for k, v in a.items() if k != "priority"} == {
        k: v for k, v in b.items() if k != "priority"
    }
audit_source_provenance(construction["actions"], graph)
prepare_cell(Path("research/planning_cell/cell.yaml"), run, source)
write_json(run / "construction_plan.json", construction)
write_section_artifacts(run, graph, construction, preserve_graph=True)
prepare_candidates(run)
assert sha256(run / "stroke_graph.json") == sha256(source / "stroke_graph.json")
write_json(
    run / "section_build.json",
    {
        "status": "PREPARED_PENDING_ROBOT_VALIDATION",
        "completed_sections": 0,
        "required_sections": len(ordered),
        "best_prefix": [],
        "simulation_only": True,
    },
)
save("COMPLETE_PROVISIONAL_STRUCTURAL_ORDER")
write_json(run / "trial_reason.json", read_json(result))
print("Prepared complete support-ready order", len(ordered), flush=True)
