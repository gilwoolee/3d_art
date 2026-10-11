"""Export complete serial source strokes, retaining all internal pen-off breaks."""

from dataclasses import replace

import numpy as np

from ..io import artifact, sha256, stage_update, write_json
from .cell_replay import replay_cell
from .clock_path import ClockPath, timed_path
from .source_coverage import audit_source_curve


def export_and_replay(build):
    planner, run = build.planner, build.run
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
        if actions:
            clock += build.cooling
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
    events = sorted(
        [
            {
                "time_s": row["start_s"] + edge["start_s"],
                "arm": row["arm"],
                "stroke_id": row["stroke_id"],
                "command": "FEED_ON" if edge["kind"] == "PEN_START_WAIT" else "FEED_OFF",
            }
            for row in actions
            for edge in row["trajectory"]["edges"]
            if edge["kind"] in ("PEN_START_WAIT", "PEN_STOP_WAIT")
        ],
        key=lambda e: (e["time_s"], e["arm"]),
    )
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
    write_json(run / "offline_dual_arm_plan.json", plan)
    stage_update(
        run,
        "timing",
        "partial",
        artifact="offline_dual_arm_plan.json",
        message="All source strokes and complete one-arm clock prepared; replay pending",
    )
    build.state.update(status="FULL_BUILD_REPLAY_RUNNING")
    build.save()
    audit = replay_cell(run, build.profile_path, float("inf"), "offline_dual_arm_plan")
    build.state.update(
        status="FULL_BUILD_VERIFIED"
        if audit["status"] == "CERTIFIED_GEOMETRIC_REPLAY"
        else "FULL_BUILD_REPLAY_REJECTED",
        replay_status=audit["status"],
    )
    build.save()
