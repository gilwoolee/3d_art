"""Parallel planning must never keep a result whose assumed inputs were replaced."""


from concurrent.futures import Future
from time import monotonic

import numpy as np
from cell_fixture import setup

from pen_art.planning.clock_path import ClockEdge, ClockPath
from pen_art.planning.parallel_build import Coordinator, ForwardSchedule


def candidate(tag, *, parent=None, index=0):
    return {
        "status": "OK",
        "deposit_key": f"deposit-{tag}",
        "table_index": index,
        "parent": parent or f"parent-{tag}",
    }


def checked(tag, index=0):
    return {"status": "OK", "table_index": index, "q_start": [0.0, tag], "q_end": [1.0, tag]}


def resolved(value):
    future = Future()
    if isinstance(value, BaseException):
        future.set_exception(value)
    else:
        future.set_result(value)
    return future


def answer(spec):
    if spec["kind"] == "propose":
        return candidate(f"{spec['index']}-{spec['ordinal']}")
    return checked(spec["index"]) if spec["kind"] == "check" else {"status": "OK"}


def test_coordinator_repairs_blocked_material_and_drops_stale_results(tmp_path):
    held, log = {}, []

    def submit(spec):
        log.append((spec["kind"], spec["index"], spec.get("ordinal")))
        if spec["kind"] == "check" and spec["index"] == 2 and "late" not in held:
            # Finishes only after the upstream candidate has been replaced,
            # which the held rejection of the first stroke now triggers.
            held["late"] = Future()
            held["rejection"].set_result({"status": "REJECTED", "failure": {"status": "COLLISION"}})
            return held["late"]
        if spec["kind"] == "check" and spec["index"] == 0 and spec["chain"] == []:
            if spec["candidate"] == held.setdefault("first", spec["candidate"]):
                held["rejection"] = Future()
                return held["rejection"]
        if spec["kind"] == "propose" and spec["index"] == 0 and spec["ordinal"]:
            held["late"].set_result(checked(2))
        return resolved(answer(spec))

    schedule = ForwardSchedule(["a", "b", "c"], "binding")
    coordinator = Coordinator(schedule, submit, 3, tmp_path, deadline=monotonic() + 30)
    assert coordinator.run() == "COMPLETE"
    assert schedule.stale >= 1
    assert schedule.slots[0]["attempts"] == 1 and schedule.slots[0]["cold"] == 1
    assert ("propose", 0, 1) in log
    assert [job for job in coordinator.jobs if not job["current"]]



def replay_fixture(tmp_path, monkeypatch, tcp_radius):
    """A two-action saved plan on the one-joint cell, replayed by the real validator."""
    import pen_art.planning.cell_replay as replay
    from pen_art.io import write_json

    planner, _ = setup(tmp_path)
    arm = planner.arms["arm"]
    arm.flange_to_tcp[0, 3] = tcp_radius
    arm.urdf = tmp_path / "arm.urdf"
    planner.tasks, planner.ground = [], []
    knots = [-0.2, -0.05, 0.1, 0.2]
    drawn = [
        ClockEdge(i, i + 1, np.array([a]), np.array([b]), "DRAW", None, radius_m=0.0005)
        for i, (a, b) in enumerate(zip(knots[:-1], knots[1:], strict=True))
    ]
    back = [ClockEdge(0, 4, np.array([0.2]), np.array([-0.2]), "TRAVEL", None)]
    write_json(
        tmp_path / "settings.json",
        {
            "search": {"maximum_subdivision_depth": 12},
            "transit": {},
            "timing": {
                "travel_speed_m_s": 1.0,
                "velocity_fraction": 0.5,
                "joint_acceleration_rad_s2": [2.0],
            },
            "process": {"nominal_speed_m_s": 1.0},
        },
    )
    write_json(
        tmp_path / "dual_arm_plan.json",
        {
            "inputs": {},
            "settings_artifact": "settings.json",
            "initial_positions": {"arm": [-0.2]},
            "modeled_duration_s": 7,
            "limitations": [],
            "events": [],
            "full_schedule": True,
            "status": "TEST",
            "actions": [
                {
                    "arm": "arm",
                    "stroke_id": name,
                    "variant": "test",
                    "start_s": start,
                    "end_s": start + edges[-1].end_s,
                    "start_anchor": "parking",
                    "auxiliary": True,
                    "trajectory": ClockPath(edges).record(),
                }
                for name, start, edges in (("draw", 0, drawn), ("back", 3, back))
            ],
        },
    )
    write_json(tmp_path / "run.json", {"stages": {}})
    plan_file = tmp_path / "dual_arm_plan.json"
    saved = replay.read_json(plan_file)
    saved["requires_final_verification"] = True
    write_json(plan_file, saved)
    write_json(tmp_path / "section_build.json", {
        "status": "FULL_BUILD_PLANNED", "planning_collision_scope": "pen_candidate",
    })

    def cell_planner(run, profile, settings, deadline, **options):
        planner.settings, planner.deadline = settings, deadline
        planner.verification_raw = options["verification_raw"]
        planner.checked_intervals = 0
        return planner

    monkeypatch.setattr(replay, "CellPlanner", cell_planner)
    return replay




def test_parallel_replay_reports_the_serial_rejection(tmp_path, monkeypatch):
    # The link sweeps through the strand it deposits: the first interval fails.
    replay = replay_fixture(tmp_path, monkeypatch, tcp_radius=1)
    serial = replay.replay_cell(tmp_path, "unused", 60)
    parallel = replay.replay_cell(tmp_path, "unused", 60, workers=2)
    assert serial["status"] == parallel["status"] == "REPLAY_REJECTED"
    assert serial["failures"][0]["status"] == parallel["failures"][0]["status"] == "COLLISION"
    assert serial["completed_strokes"] == parallel["completed_strokes"] == 0
    assert "final_material" not in serial and "final_material" not in parallel
    report = replay.read_json(tmp_path / "section_build.json")
    assert report["status"] == "FULL_BUILD_REPLAY_REJECTED"
    assert replay.read_json(tmp_path / "run.json")["stages"]["motion"]["status"] == "partial"


