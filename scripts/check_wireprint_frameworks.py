"""Regenerate the four mesh frameworks and check every proposed release state."""

from pathlib import Path

from pen_art.abstraction.wireprint import connect_framework_islands, generate
from pen_art.io import read_json, sha256, write_json
from pen_art.planning.sections import coverage_audit, section_actions
from pen_art.structure import analyze_structure

rows = []
for name, source in (
    ("cube", "cube"),
    ("triangular_prism", "prism"),
    ("torus", "torus"),
    ("bottle", "bottle"),
):
    fixture = Path("research/primitive_fixtures") / name
    run = Path("runs/wireprint_join_preflight") / name
    source_run = Path("runs") / f"{source}_sections_v1_source"
    graph = connect_framework_islands(generate(source_run))
    construction = section_actions(graph, group_sources=True)
    actions = sorted(construction["actions"], key=lambda row: (row["priority"], row["id"]))
    material = read_json(fixture / "fabrication_config.json")["structure"]
    final = analyze_structure(graph, material)
    members, releases = set(), []
    for action in actions:
        members.update(action["members"])
        screen = analyze_structure(graph, material, edge_ids=members)
        releases.append(
            {
                "section": action["id"],
                "modeled_pass": screen["modeled_pass"],
                "maximum_displacement_m": screen.get("maximum_displacement_m"),
                "failure": screen.get("failure"),
            }
        )
        if not screen["modeled_pass"]:
            break
    write_json(run / "graph.json", graph)
    write_json(run / "construction_plan.json", construction)
    write_json(run / "release_screens.json", {"releases": releases})
    row = {
        "fixture": name,
        "source_run": str(source_run),
        "source_inputs": graph["inputs"],
        "graph_sha256": sha256(run / "graph.json"),
        "construction_sha256": sha256(run / "construction_plan.json"),
        "members": len(graph["edges"]),
        "sections": len(actions),
        "final_frame_pass": final["modeled_pass"],
        "coverage": coverage_audit(actions, construction["actions"]),
        "released_sections": len(releases),
        "all_release_states_pass": len(releases) == len(actions)
        and all(r["modeled_pass"] for r in releases),
    }
    rows.append(row)
    print(row, flush=True)
write_json(
    Path("research/blondie/wireprint_join_sections_preflight.json"),
    {
        "simulation_only": True,
        "physical_status": "unverified",
        "robot_motion_checked": False,
        "fixtures": rows,
    },
)
assert all(r["final_frame_pass"] and r["all_release_states_pass"] for r in rows)
