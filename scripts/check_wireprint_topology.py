"""Exercise nested contours, split/merge events, and an unsupported contour birth."""

from collections import Counter
from pathlib import Path

import numpy as np
import trimesh

from pen_art.abstraction.wireprint import connect_framework_islands, mesh_framework
from pen_art.io import read_json, sha256, write_json
from pen_art.planning.sections import coverage_audit, section_actions
from pen_art.structure import analyze_structure

root = Path("runs/wireprint_topology")
material_path = Path("research/primitive_fixtures/cube/fabrication_config.json")
material = read_json(material_path)["structure"]
tube = trimesh.creation.annulus(r_min=0.018, r_max=0.03, height=0.04, sections=32)
torus = trimesh.creation.torus(
    major_radius=0.03, minor_radius=0.01, major_sections=32, minor_sections=16
)
torus.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
lower = trimesh.creation.box([0.015, 0.015, 0.03])
upper = lower.copy()
upper.apply_translation([0.03, 0, 0.02])
separated = trimesh.util.concatenate([lower, upper])
reports = []
for name, mesh in (("tube", tube), ("upright_torus", torus), ("raised_island", separated)):
    run = root / name
    run.mkdir(parents=True, exist_ok=True)
    mesh.apply_translation([0, 0, -mesh.bounds[0, 2]])
    mesh.export(run / "source.stl")
    graph = connect_framework_islands(mesh_framework(trimesh.load(run / "source.stl")))
    events = Counter(row["event"] for row in graph["contour_events"])
    construction = section_actions(graph, group_sources=True)
    actions = sorted(construction["actions"], key=lambda row: (row["priority"], row["id"]))
    coverage = coverage_audit(actions, graph["edges"])
    available = {node["id"] for node in graph["nodes"] if node["ground"]}
    members, releases, rejection = set(), [], None
    for row in actions:
        visits = {visit["node"] for visit in row["junction_visits"]}
        anchors = visits if row["closed"] else {row["start"], row["end"]}
        if not available.intersection(anchors):
            rejection = {"section": row["id"], "reason": "NO_EXISTING_START_ANCHOR"}
            break
        members.update(row["members"])
        screen = analyze_structure(graph, material, edge_ids=members)
        releases.append({"section": row["id"], "structure": screen})
        assert screen["modeled_pass"], (name, row["id"], screen)
        available.update(visits)
    if name == "raised_island":
        assert events["unsupported_birth"] and graph["component_connections"]["added_members"]
        # A connector in the final graph does not prove its proposed order is buildable.
        assert rejection is not None
    else:
        assert rejection is None and len(releases) == len(actions)
        samples = np.concatenate(
            [
                np.linspace(edge["segments"][0][0], edge["segments"][0][-1], 20)
                for edge in graph["edges"]
            ]
        )
        if name == "tube":
            # Five zigzag cycles on a 32-sided inner ring produce chords;
            # allow their analytic sag plus the declared 20 micron merge.
            radius_bound = 0.018 * np.cos(np.pi / 10) * np.cos(np.pi / 32) - 0.00002
            assert np.linalg.norm(samples[:, :2], axis=1).min() > radius_bound
        else:
            assert events["split"] and events["merge"]
            assert np.linalg.norm(samples[:, [0, 2]] - [0, 0.04], axis=1).min() > 0.01
    write_json(run / "graph.json", graph)
    write_json(run / "construction_plan.json", construction)
    write_json(run / "release_screens.json", {"releases": releases, "rejection": rejection})
    reports.append(
        {
            "case": name,
            "events": dict(events),
            "coverage": coverage,
            "sections": len(actions),
            "verified_release_states": len(releases),
            "proposed_order_rejection": rejection,
            "inputs": {
                str(run / file): sha256(run / file)
                for file in ("source.stl", "graph.json", "construction_plan.json")
            },
        }
    )
    print(name, len(releases), "release states; rejection:", rejection, flush=True)
write_json(
    Path("research/blondie/wireprint_topology.json"),
    {
        "status": "PASSED",
        "simulation_only": True,
        "robot_motion_checked": False,
        "physical_status": "unverified",
        "material_sha256": sha256(material_path),
        "cases": reports,
    },
)
