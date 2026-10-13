"""Export exact retained forward witnesses as a source-bound partial schedule."""

from copy import deepcopy
from pathlib import Path

import numpy as np

from ..io import artifact, read_json, sha256, stage_update, write_json
from .clock_path import ClockPath
from .process_clock import action_ready_times, feed_events


def prefix_source_graph(graph, actions):
    """Restrict provenance obligations to deposited members, retaining full aliases."""
    members = {member for row in actions for member in row["members"]}
    known = {row["id"] for row in graph["edges"]}
    if not members <= known:
        raise ValueError("Partial schedule refers to unknown source members")
    selected = deepcopy(graph)
    selected["edges"] = [row for row in selected["edges"] if row["id"] in members]
    if "source_aliases" in selected:
        selected["source_aliases"] = [
            row for row in selected["source_aliases"] if set(row["members"]) <= members
        ]
    selected.pop("required_artistic_sources", None)
    return selected


def export_forward_prefix(source, destination):
    """Publish saved clocks without motion solving or altering the source geometry."""
    source, destination = Path(source), Path(destination)
    checkpoint_bytes = (source / "forward_checkpoint.json").read_bytes()
    import json

    checkpoint = json.loads(checkpoint_bytes)
    binding = checkpoint["binding"]
    required = [
        "cell_config.json",
        "cell_manifest.json",
        "cell_drawing_variants.json",
        "construction_plan.json",
        "stroke_graph.json",
        "fabrication_config.json",
    ]
    for name in required:
        if sha256(source / name) != sha256(destination / name):
            raise ValueError(f"Prefix source and destination differ: {name}")
    for name, digest in binding["inputs"].items():
        if sha256(source / name) != digest:
            raise ValueError(f"Stale checkpoint input: {name}")
    originals = {row["id"]: row for row in read_json(source / "construction_plan.json")["actions"]}
    initial = np.asarray(binding["initial_positions"])
    cell = read_json(destination / "cell_manifest.json")
    if len(cell["arms"]) != 1:
        raise ValueError("Partial forward export requires one arm")
    arm = cell["arms"][0]["id"]
    ready = {
        row["id"]: 0.0 for row in read_json(source / "stroke_graph.json")["nodes"] if row["ground"]
    }
    clock, actions, witness_inputs = 0.0, [], {}
    previous = initial
    for reference in checkpoint["witnesses"]:
        relative = Path(reference["file"])
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError("Unsafe witness path")
        path = source / relative
        if sha256(path) != reference["sha256"]:
            raise ValueError(f"Stale witness: {relative}")
        saved = read_json(path)
        drawing = saved["drawing"]
        identity = saved["section"]
        if identity != drawing["stroke_id"] or identity not in originals:
            raise ValueError("Witness identity differs from construction source")
        trajectory = ClockPath.from_record(saved["trajectory"])
        if not np.allclose(previous, trajectory.edges[0].q0, atol=1e-8, rtol=0):
            raise ValueError(
                "Retained witness prefix is not contiguous from the saved initial pose"
            )
        previous = trajectory.edges[-1].q1
        row = originals[identity]
        clock = max(clock, ready.get(drawing["start_anchor"], float("inf")))
        if not np.isfinite(clock):
            raise ValueError("Prefix begins on missing or uncooled source material")
        action = {
            **deepcopy(row),
            "arm": arm,
            "stroke_id": identity,
            "variant": drawing["variant"],
            "start_anchor": drawing["start_anchor"],
            "reverse": drawing.get("reverse", False),
            "start_visit": drawing.get("start_visit", 0),
            "start_s": clock,
            "end_s": clock + trajectory.duration_s,
            "trajectory": saved["trajectory"],
            "auxiliary": False,
            "table_index": drawing["table_index"],
            "extrusion_mode": "fixed_axis",
            "proposal_source": drawing.get("proposal_source", "retained_witness"),
        }
        actions.append(action)
        for node, relative_time in action_ready_times(
            trajectory, row, binding["settings"]["assumed_cooling_s"]
        ).items():
            ready[node] = min(ready.get(node, float("inf")), clock + relative_time)
        clock += trajectory.duration_s
        copied = destination / "prefix_witnesses" / relative.name
        copied.parent.mkdir(parents=True, exist_ok=True)
        copied.write_bytes(path.read_bytes())
        witness_inputs[str(copied.relative_to(destination))] = sha256(copied)
    if not actions or len(actions) >= len(originals):
        raise ValueError("Partial export needs a nonempty, incomplete prefix")
    checkpoint_copy = destination / "prefix_source_checkpoint.json"
    checkpoint_copy.write_bytes(checkpoint_bytes)
    settings_file = "offline_dual_arm_config.json"
    write_json(
        destination / settings_file,
        {
            "search": binding["settings"],
            "transit": binding["transit"],
            "timing": binding["timing"],
            "process": binding["process"],
        },
    )
    manifest_file = "partial_playback_manifest.json"
    write_json(
        destination / manifest_file,
        artifact(
            "partial_playback_manifest",
            source_run=str(source.resolve()),
            source_checkpoint_sha256=sha256(checkpoint_copy),
            completed_sections=len(actions),
            requested_sections=len(originals),
            included_sections=[row["id"] for row in actions],
            witness_inputs=witness_inputs,
            scope="Exact retained prefix; fresh replay required",
        ),
    )
    profile = read_json(destination / "cell_config.json")
    plan = artifact(
        "dual_arm_plan",
        status="PARTIAL_SECTION_SCHEDULE",
        full_schedule=False,
        section_schedule=False,
        partial_section_schedule=True,
        executable=False,
        simulation_only=True,
        resource_mode="single",
        active_arms=[arm],
        enabled_arms=[arm],
        frame="world",
        units="m",
        initial_positions={arm: initial.tolist()},
        initial_table_index=profile["turntable"].get("home_index", 0),
        turntable=profile["turntable"],
        settings_artifact=settings_file,
        inputs={
            name: sha256(destination / name)
            for name in [
                *required,
                settings_file,
                manifest_file,
                "prefix_source_checkpoint.json",
                *witness_inputs,
            ]
        },
        actions=actions,
        events=feed_events(actions),
        modeled_duration_s=clock,
        completed_strokes=len(actions),
        requested_strokes=len(originals),
        checked_intervals=0,
        limitations=[
            "Partial prefix only; remaining construction is not scheduled",
            "Simulation only; physical pen and material behavior are uncalibrated",
        ],
        failures=[],
    )
    write_json(destination / "offline_dual_arm_plan.json", plan)
    for stage in ["motion", "timing"]:
        stage_update(destination, stage, "partial", artifact="offline_dual_arm_plan.json")
    return plan
