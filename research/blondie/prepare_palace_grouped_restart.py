"""Join a mutually occluding artistic restart pair through its source interval."""

from copy import deepcopy
from pathlib import Path

import numpy as np

from pen_art.cell import prepare_cell
from pen_art.io import read_json, sha256, write_json
from pen_art.planning.section_build import prepare_candidates, write_section_artifacts
from pen_art.planning.sections import coverage_audit
from pen_art.provenance import audit_source_provenance
from pen_art.structure import analyze_structure

source = Path("runs/palace_hybrid_art_order_v1")
run = Path("runs/palace_hybrid_grouped_restart_v1")
assert not run.exists()
graph = read_json(source / "stroke_graph.json")
original = read_json(source / "construction_plan.json")
construction = deepcopy(original)
keys = [f"section-00602-part-{i}" for i in (1, 2, 3)]
by_id = {row["id"]: row for row in construction["actions"]}
pieces = [by_id[key] for key in keys]
for left, right in zip(pieces[:-1], pieces[1:], strict=True):
    assert left["parent_curve"] == right["parent_curve"]
    assert left["end"] == right["start"]
    assert left["source_interval"][1] == right["source_interval"][0]
    assert left["stage"] == right["stage"] == "artistic"
    assert left["diameter_m"] == right["diameter_m"]
    assert not left["depends_on"] and not right["depends_on"]
evidence = [
    Path("runs/palace_art_suffix_access_v1/access.json"),
    Path("runs/palace_art_suffix_part1_v1/access.json"),
]
for path, target, blocker in zip(evidence, (keys[2], keys[0]), (keys[0], keys[2]), strict=True):
    report = read_json(path)
    assert report["status"] == "FINITE_SUFFIX_ACCESS_SCREEN"
    assert report["section"] == target
    failed = report["point_checks"][0]
    assert failed["full_material_clear"] == 0
    assert failed["framework_clear"] == failed["blocker_pose_counts"][blocker] > 0
    for name, digest in report["inputs"].items():
        recorded = Path(name)
        if recorded.suffix == ".py" and sha256(recorded) != digest:
            # The diagnostic driver evolves; its exact executed version stays
            # beside the report. Never substitute a different geometry input.
            recorded = path.parent / "executed_driver.py"
        assert sha256(recorded) == digest, name

joined = deepcopy(pieces[0])
joined.update(
    id="section-00602-joined-1-3",
    end=pieces[-1]["end"],
    source_interval=[pieces[0]["source_interval"][0], pieces[-1]["source_interval"][1]],
    members=[member for piece in pieces for member in piece["members"]],
    segments=[segment for piece in pieces for segment in piece["segments"]],
    length_m=sum(piece["length_m"] for piece in pieces),
    closed=pieces[0]["start"] == pieces[-1]["end"],
    priority=max(row["priority"] for row in by_id.values() if row["id"] not in keys) + 1,
)
visits = []
distance, boundary = 0.0, 0
for index, piece in enumerate(pieces):
    for visit in piece["junction_visits"][int(index > 0) :]:
        visits.append(
            {
                **visit,
                "distance_m": visit["distance_m"] + distance,
                "segment_boundary": visit["segment_boundary"] + boundary,
            }
        )
    distance += piece["length_m"]
    boundary += len(piece["segments"])
joined["junction_visits"] = visits
construction["actions"] = [
    joined if row["id"] == keys[0] else row
    for row in construction["actions"]
    if row["id"] == keys[0] or row["id"] not in keys
]
for row in construction["actions"]:
    if row["id"] != joined["id"]:
        assert row == by_id[row["id"]]
coverage = coverage_audit(construction["actions"], original["actions"])
provenance = audit_source_provenance(construction["actions"], graph)
material = read_json(source / "fabrication_config.json")["structure"]
predecessor = analyze_structure(
    graph,
    material,
    edge_ids={
        member
        for row in construction["actions"]
        if row["id"] != joined["id"]
        for member in row["members"]
    },
)
assert predecessor["modeled_pass"]
assert np.array_equal(
    np.asarray(joined["segments"]),
    np.asarray([segment for piece in pieces for segment in piece["segments"]]),
)
prepare_cell(Path("research/planning_cell/cell.yaml"), run, source)
write_json(run / "construction_plan.json", construction)
write_section_artifacts(run, graph, construction, preserve_graph=True)
prepare_candidates(run)
assert sha256(run / "stroke_graph.json") == sha256(source / "stroke_graph.json")
receipt = {
    "status": "GROUPED_RESTART_TRIAL_PREPARED",
    "run": str(run),
    "joined_section": joined["id"],
    "replaced_sections": keys,
    "source_interval": joined["source_interval"],
    "sections": len(construction["actions"]),
    "coverage": coverage,
    "source_provenance": provenance,
    "geometry_unchanged": True,
    "predecessor_displacement_m": predecessor["maximum_displacement_m"],
    "simulation_only": True,
    "scope": (
        "Finite mutual occlusion suggests removing two internal stop/restart boundaries. "
        "All source segments and framework members remain unchanged. Robot validation pending."
    ),
    "inputs": {
        str(path): sha256(path)
        for path in [
            source / "stroke_graph.json",
            source / "construction_plan.json",
            run / "stroke_graph.json",
            run / "construction_plan.json",
            *evidence,
            Path(__file__),
        ]
    },
}
write_json(Path("research/blondie/palace_grouped_restart.json"), receipt)
write_json(run / "trial_reason.json", receipt)
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
print(joined["id"], "prepared", len(construction["actions"]), "sections", flush=True)
