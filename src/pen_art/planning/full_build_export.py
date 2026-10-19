"""Export playable schedules without running independent final verification."""

from shutil import copyfile

import numpy as np

from ..io import artifact, sha256, stage_update, write_json
from .process_clock import action_ready_times, feed_events


def seal_repair_evidence(run, key, path):
    """Keep a matching diagnostic, or explicitly mark an old one unavailable."""
    metadata = path.metadata
    name = metadata.get("axis_repair_artifact")
    digest = metadata.get("axis_repair_sha256")
    if name is None:
        return
    source = run / name
    if not source.is_file() or sha256(source) != digest:
        metadata.pop("axis_repair_artifact", None)
        metadata.pop("axis_repair_sha256", None)
        metadata["axis_repair_diagnostic"] = "SUPERSEDED_WORKING_REPORT"
        # Source coverage and independent replay still check the actual clock;
        # an overwritten proposal report is not evidence for that clock.
        return
    if source.parent.name != "accepted":
        accepted = run / "axis_repairs" / "accepted" / f"{key}-{digest}.json"
        accepted.parent.mkdir(parents=True, exist_ok=True)
        copyfile(source, accepted)
        metadata["axis_repair_artifact"] = str(accepted.relative_to(run))


def schedule_destination(run):
    path = run / "plan.json"
    if path.with_suffix(".clocks.json").exists():
        raise ValueError("Use a fresh run to preserve existing packed-clock evidence")
    return path


def save_schedule(run, plan):
    """Save exact clocks; the independent verifier unlocks viewer playback."""
    path = schedule_destination(run)
    plan = {
        key: value for key, value in plan.items()
        if key not in {"packed_clock_binding", "replay", "replay_in_progress"}
    }
    plan.update(
        status="FULL_SCHEDULE_VERIFICATION_PENDING", requires_final_verification=True,
        executable=False,
        simulation_only=True, checked_intervals=0,
    )
    write_json(path, plan, compact=True)
    stage_update(run, "timing", "complete", artifact=path.name)
    stage_update(run, "motion", "partial", artifact=path.name,
                 message="Schedule saved; separate full-robot verification required")
    stage_update(run, "validation", "pending", message="Separate full-robot verification required")


def export_sections(build, witnesses, returned, *, release_witnesses=False):
    """Keep exact planned clocks and readiness timing; skip repeat audits."""

    planner, run = build.planner, build.run
    schedule_destination(run)
    actions, clock = [], 0.0
    ready = {node: 0.0 for node in planner.ground}
    framework_ready = 0.0
    for key, move, drawing in witnesses:
        section = build.actions[key]
        path = move.payload
        # A chained stroke continues from its predecessor with the pen down;
        # only a stroke that attaches afresh waits for its joint to cool.
        if not path.metadata.get("chained_in"):
            clock = max(clock, ready.get(move.start_anchor, float("inf")))
        elif move.start_anchor not in ready:
            raise ValueError("Cannot chain a section onto absent material")
        if section["stage"] == "artistic":
            clock = max(clock, framework_ready)
        if not np.isfinite(clock):
            raise ValueError("Cannot schedule a section on absent material")
        seal_repair_evidence(run, key, path)
        actions.append(
            {
                **{
                    field: section[field]
                    for field in (
                        "id",
                        "parent_curve",
                        "source_interval",
                        "members",
                        "stage",
                        "start",
                        "end",
                        "role",
                        "junction_visits",
                        "closed",
                    )
                },
                **(
                    {"source_intervals": section["source_intervals"]}
                    if "source_intervals" in section
                    else {}
                ),
                "arm": build.arm.identity,
                "stroke_id": key,
                "variant": move.variant,
                "start_anchor": move.start_anchor,
                "reverse": drawing["reverse"],
                "start_visit": drawing.get("start_visit", 0),
                "proposal_source": drawing.get("proposal_source", "planner"),
                "start_s": clock,
                "end_s": clock + path.duration_s,
                "trajectory": path.record(),
                "auxiliary": False,
            }
        )
        availability = action_ready_times(path, section, build.cooling)
        for node, relative in availability.items():
            ready[node] = min(ready.get(node, float("inf")), clock + relative)
        if section["stage"] == "framework":
            framework_ready = max(framework_ready, clock + max(availability.values()))
        clock += path.duration_s
    if returned is not None:
        actions.append(
            {
                "arm": build.arm.identity,
                "stroke_id": "return-" + build.arm.identity,
                "variant": "return",
                "start_anchor": "parking",
                "start_s": clock,
                "end_s": clock + returned.duration_s,
                "trajectory": returned.record(),
                "auxiliary": True,
            }
        )
        clock += returned.duration_s
    settings = "plan_config.json"
    write_json(
        run / settings,
        {
            "search": planner.settings,
            "transit": planner.transit,
            "timing": planner.timing,
            "process": planner.process,
        },
    )
    write_json(
        run / "section_build_manifest.json",
        artifact(
            "section_build_manifest",
            status="VERIFICATION_PENDING",
            final_verification="NOT_RUN",
            physical_status="unverified",
        ),
    )
    events = feed_events(actions)
    save_schedule(
        run,
        artifact(
            "plan",
            status="FULL_SECTION_SCHEDULE_REPLAY_PENDING",
            full_schedule=True,
            section_schedule=True,
            executable=False,
            simulation_only=True,
            resource_mode="single",
            active_arms=[build.arm.identity],
            enabled_arms=[build.arm.identity],
            frame="world",
            units="m",
            inputs={
                name: sha256(run / name)
                for name in (
                    "cell_manifest.json",
                    "cell_drawing_variants.json",
                    "construction_plan.json",
                    "stroke_graph.json",
                    "fabrication_config.json",
                    "section_build_manifest.json",
                    settings,
                )
            },
            actions=actions,
            events=events,
            modeled_duration_s=clock,
            completed_strokes=len(witnesses),
            requested_strokes=len(build.actions),
            checked_intervals=0,
            settings_artifact=settings,
            initial_positions={build.arm.identity: build.initial.tolist()},
            initial_pose_repairs=planner.initial_pose_repairs,
            lift_offs=getattr(build, "lift_offs", None),
            minimum_trails=getattr(build, "minimum_trails", None),
            limitations=[
                "Simulation only",
                "Pen, cooling, material and joint behavior uncalibrated",
            ],
            failures=[],
        ),
    )
    build.best = [key for key, _, _ in witnesses]
    if release_witnesses:
        # The viewer reads the saved JSON; release export-only clocks now.
        import gc

        actions.clear()
        witnesses.clear()
        gc.collect()
    build.report("FULL_BUILD_PLANNED", replay_status="NOT_RUN")
    return {"status": "SCHEDULE_SAVED", "replay_status": "NOT_RUN"}
