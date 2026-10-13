"""Export playable schedules without running independent final verification."""

from dataclasses import replace
from shutil import copyfile

import numpy as np

from ..io import artifact, sha256, stage_update, write_json
from .clock_path import ClockPath, timed_path
from .process_clock import action_ready_times, feed_events
from .source_coverage import audit_source_curve


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
    path = run / "offline_dual_arm_plan.json"
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
        clock = max(clock, ready.get(move.start_anchor, float("inf")))
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
                **(
                    {"motion_library_record": drawing["motion_library_record"]}
                    if drawing.get("motion_library_record")
                    else {}
                ),
                "proposal_source": drawing.get("proposal_source", "planner"),
                **(
                    {
                        "motion_library_parent": drawing["motion_library_parent"],
                        "cache_validation_scope": drawing.get("cache_validation_scope"),
                    }
                    if drawing.get("motion_library_parent")
                    else {}
                ),
                "start_s": clock,
                "end_s": clock + path.duration_s,
                "trajectory": path.record(),
                "auxiliary": False,
                **(
                    {
                        "table_index": path.metadata["drawing_table_index"],
                        "extrusion_mode": (
                            "varying_axis"
                            if path.metadata.get("tracking_orientation_constraint") == "free"
                            else "fixed_axis"
                        ),
                    }
                    if planner.turntable is not None
                    else {}
                ),
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
    settings = "offline_dual_arm_config.json"
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
            "dual_arm_plan",
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
            **(
                {
                    "initial_table_index": planner.turntable.home_index,
                    "turntable": planner.profile["turntable"],
                }
                if planner.turntable is not None
                else {}
            ),
            initial_pose_repairs=planner.initial_pose_repairs,
            limitations=[
                "Simulation only",
                "Pen, cooling, material and joint behavior uncalibrated",
                *(
                    [
                        "Fixed-axis and varying-axis deposition assume a pen that can extrude "
                        "at every planned orientation; material flow behavior is uncalibrated. "
                        "The plate remains fixed for each stroke; TCP tracking and rigid "
                        "collision clearance are checked continuously."
                    ]
                    if planner.turntable is not None
                    else []
                ),
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


def export_full_build(build):
    planner, run = build.planner, build.run
    schedule_destination(run)
    expected = [row["stroke_id"] for row in build.selection]
    if set(build.state["completed_strokes"]) != set(expected):
        raise ValueError("Full build export requires every original stroke")
    groups = []
    for row in build.state["motions"]:
        if row["kind"] == "support":
            if not groups:
                raise ValueError("Unassigned additional support")
            source = groups[-1][0]
        else:
            source = row["source_stroke"]
        if not groups or groups[-1][0] != source:
            groups.append((source, []))
        groups[-1][1].append(row)
    if [source for source, _ in groups] != expected:
        raise ValueError("Full build must cover the selected source order exactly once")
    selected = {row["stroke_id"]: row for row in build.selection}
    actions, coverage, clock = [], [], 0.0
    ready = {node: 0.0 for node in planner.ground}
    for source, records in groups:
        edges, current, preparation, source_paths = [], 0, [], []
        original = any(r["kind"] == "source" for r in records)
        for row in records:
            path = build.read_motion(row)
            if row["kind"] != "support":
                source_paths.append(path)
            preparation.append(
                {
                    **row,
                    "sha256": sha256(run / row["file"]),
                    "tracking": path.metadata,
                }
            )
            offset = edges[-1].end_s if edges else 0
            edges.extend(
                replace(e, start_s=e.start_s + offset, end_s=e.end_s + offset) for e in path.edges
            )
            if row["kind"] == "fragment":
                if row["start_index"] != current:
                    raise ValueError(f"Source coverage gap: {source}")
                current = row["end_index"]
        candidate = next(
            d
            for d in planner.candidates["variants"]
            if d["stroke_id"] == source and d["variant"] == selected[source]["variant"]
        )
        if not original and current != len(candidate["tcp_targets_m"]) - 1:
            raise ValueError(f"Incomplete source geometry: {source}")
        geometry = audit_source_curve(build.arm, source_paths, candidate["tcp_targets_m"])
        if geometry["status"] != "BOUNDED":
            build.state.update(
                status="SOURCE_GEOMETRY_REJECTED",
                source_geometry_failure={"source": source, **geometry},
            )
            build.save()
            return
        path = ClockPath.from_record(ClockPath(edges).record())
        clock = max(clock, ready.get(selected[source]["start_anchor"], float("inf")))
        if not np.isfinite(clock):
            raise ValueError("Cannot schedule a source on absent material")
        actions.append(
            {
                "arm": build.arm.identity,
                "stroke_id": source,
                "variant": "supported-restarts",
                "start_anchor": selected[source]["start_anchor"],
                "start_s": clock,
                "end_s": clock + path.duration_s,
                "trajectory": path.record(),
                "auxiliary": False,
            }
        )
        for node, relative in action_ready_times(
            path, planner.actions[source], build.cooling
        ).items():
            ready[node] = min(ready.get(node, float("inf")), clock + relative)
        clock += path.duration_s
        coverage.append(
            {
                "source_stroke": source,
                "complete": True,
                "motion_records": len(records),
                "source_target_count": len(candidate["tcp_targets_m"]),
                "source_direction": build.state.get("source_directions", {}).get(source, "forward"),
                "preparation_evidence": preparation,
                "independent_geometry_coverage": geometry,
            }
        )
    initial = np.asarray(build.initial)
    returned = planner.travel(build.arm.identity, build.scene, build.q, initial)
    if returned["status"] != "CONNECTED":
        build.state.update(status="FINAL_RETURN_BLOCKED", return_diagnostic=returned)
        build.save()
        return
    path = timed_path(
        [
            {
                "kind": "RETURN",
                "positions": returned["positions"],
                "stroke_id": "return-" + build.arm.identity,
            }
        ],
        build.arm,
        build.arm.robot.velocity * planner.timing["velocity_fraction"],
        np.asarray(planner.timing["joint_acceleration_rad_s2"]),
        planner.process["nominal_speed_m_s"],
        planner.timing["travel_speed_m_s"],
        drawing_clock=planner.timing.get("drawing_clock", "quintic_stop_at_geometric_knots"),
        acceleration_constraint=planner.timing.get("acceleration_constraint", "interpolated"),
    )
    actions.append(
        {
            "arm": build.arm.identity,
            "stroke_id": "return-" + build.arm.identity,
            "variant": "return",
            "start_anchor": "parking",
            "start_s": clock,
            "end_s": clock + path.duration_s,
            "trajectory": path.record(),
            "auxiliary": True,
        }
    )
    clock += path.duration_s
    settings = "offline_dual_arm_config.json"
    write_json(
        run / settings,
        {
            "search": planner.settings,
            "transit": planner.transit,
            "timing": planner.timing,
            "process": planner.process,
        },
    )
    manifest = "single_arm_build_manifest.json"
    snapshot = build.output / "prepared_state.json"
    write_json(snapshot, build.state)
    write_json(
        run / manifest,
        {
            "source_coverage": coverage,
            "prepared_state_file": "full_build/prepared_state.json",
            "full_build_state_sha256": sha256(snapshot),
            "additional_support": build.support,
            "additional_supports": build.supports,
            "structure_sha256": sha256(build.output / "structural_graph.json"),
            "simulation_only": True,
            "physical_status": "unverified",
        },
    )
    events = feed_events(actions)
    plan = artifact(
        "dual_arm_plan",
        status="FULL_SCHEDULE_GEOMETRY_UNVERIFIED",
        full_schedule=True,
        executable=False,
        simulation_only=True,
        resource_mode="single",
        active_arms=[build.arm.identity],
        frame="world",
        units="m",
        inputs={
            name: sha256(run / name)
            for name in (
                "cell_manifest.json",
                "cell_drawing_variants.json",
                "construction_plan.json",
                "stroke_graph.json",
                "cell_allocation.json",
                settings,
                manifest,
            )
        },
        actions=actions,
        events=events,
        modeled_duration_s=clock,
        completed_strokes=0,
        requested_strokes=len(expected),
        checked_intervals=0,
        settings_artifact=settings,
        enabled_arms=[build.arm.identity],
        initial_positions={build.arm.identity: initial.tolist()},
        initial_pose_repairs={"precondition": "Start the arm at its saved staged pose"},
        limitations=[
            "Simulation only",
            "Pen and material behavior uncalibrated",
            "Added automatic support and internal restarts; source geometry retained",
        ],
        failures=[],
    )
    save_schedule(run, plan)
    build.state.update(status="FULL_BUILD_PLANNED", replay_status="NOT_RUN")
    build.save()
