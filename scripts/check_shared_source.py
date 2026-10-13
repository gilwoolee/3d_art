"""Build three real sections while crediting a coincident fourth source once.

Run check_section_interleaving.py first to prepare the small reference fixture.
"""

from pathlib import Path

from pen_art.curves import curve_length
from pen_art.io import read_json, sha256, write_json
from pen_art.planning.section_build import (
    SectionBuild,
    prepare_candidates,
    prepare_reference,
    write_section_artifacts,
)
from pen_art.planning.sections import section_actions

run = Path("runs/staged_shared_regression_v1")
profile = Path("research/planning_cell/cell.yaml")
prepare_reference(Path("runs/interleaved_reference"), run, profile)
graph = read_json(run / "stroke_graph.json")
for edge in graph["edges"]:
    edge["stage"] = "artistic" if edge["parent_curve"] == "B" else "framework"
    edge["role"] = edge["stage"]
owner = next(edge for edge in graph["edges"] if edge["id"] == "edge0")
source = {
    "id": "shared-detail",
    "segments": owner["segments"],
    "diameter_m": owner["diameter_m"],
    "length_m": curve_length(owner),
}
write_json(run / "shared_detail_source.json", source)
graph.setdefault("inputs", {})["shared_detail_source.json"] = sha256(
    run / "shared_detail_source.json"
)
graph["source_aliases"] = [
    {
        "parent_curve": source["id"],
        "source_interval": [0, 1],
        "stage": "artistic",
        "members": [owner["id"]],
    }
]
graph["required_artistic_sources"] = [
    {"parent_curve": "B", "source_interval": [0, 1]},
    {"parent_curve": source["id"], "source_interval": [0, 1]},
]
construction = section_actions(graph)
write_json(run / "construction_plan.json", construction)
write_section_artifacts(run, graph, construction)
prepare_candidates(run)
build = SectionBuild(run, profile, 1800, gpu=True)
build.run_build(regression_only=True)
state = read_json(run / "section_build.json")
assert state["status"] == "FULL_BUILD_VERIFIED", state["status"]
validation = read_json(run / "offline_dual_arm_validation.json")
plan = read_json(run / "offline_dual_arm_plan.json")
credit = validation["source_provenance"]["shared_intervals"]
assert len(credit) == 1
assert (
    credit[0]["parent_curve"] == "shared-detail"
    and credit[0]["credited_after_section"] == "section-00000"
)
assert len([e for e in plan["events"] if e["command"] == "FEED_ON"]) == 3
write_json(
    Path("research/blondie/staged_shared_regression.json"),
    {
        "status": state["status"],
        "run": str(run),
        "simulation_only": True,
        "physical_status": "unverified",
        "source_provenance": validation["source_provenance"],
        "checked_intervals": validation["checked_intervals"],
        "modeled_duration_s": validation["horizon_s"],
        "physical_sections": 3,
        "logical_sources": 3,
        "feed_on_events": 3,
        "inputs": {
            str(run / f): sha256(run / f)
            for f in (
                "stroke_graph.json",
                "construction_plan.json",
                "offline_dual_arm_plan.json",
                "offline_dual_arm_validation.json",
            )
        },
    },
)
print("Shared source credited without duplicate deposition; real staged replay PASSED", flush=True)
