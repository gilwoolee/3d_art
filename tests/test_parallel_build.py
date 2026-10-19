"""Parallel planning must never keep a result whose assumed inputs were replaced."""


from concurrent.futures import Future
from time import monotonic

import numpy as np
from cell_fixture import setup

from pen_art.detail import choose_detail
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


def test_coordinator_chains_continuing_strokes_and_drops_stale_results(tmp_path):
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

    # A stroke that can start where its predecessor ends is proposed from that
    # end and drawn without lifting; if that fails it lifts, and says why.
    rows = [
        {"start": "n0", "end": "n1", "closed": False, "nodes": ["n0", "n1"]},
        {"start": "n1", "end": "n1", "closed": True, "nodes": ["n1", "n2"]},
        {"start": "n5", "end": "n6", "closed": False, "nodes": ["n5", "n6"]},
    ]

    def jobs(schedule):
        return {(spec["kind"], spec["index"]): spec for spec in schedule.wanted()}

    def offered(end_node, previous=None):
        return {
            "status": "OK", "deposit_key": "deposit", "table_index": 0, "end_node": end_node,
            "q_draw_end": [0.1], "tcp_rotation": np.eye(3).tolist(), "chain_previous": previous,
        }

    chained = ForwardSchedule(["a", "b", "c"], "binding", rows=rows)
    assert ("propose", 1) not in jobs(chained)
    chained.apply(jobs(chained)["propose", 0], offered("n1"))
    second = jobs(chained)["propose", 1]
    assert second["mode"] == "cold"
    assert second["chain_from"]["node"] == "n1" and second["chain_from"]["q"] == [0.1]
    assert second["chain_from"]["action"] == "a"
    assert ("check", 0) not in jobs(chained)
    chained.apply(second, offered("n1", second["chain_from"]["previous"]))
    assert jobs(chained)["check", 0]["chained_out"] and jobs(chained)["check", 1]["chained_in"]
    assert chained.lift_offs() == [{"before": "a", "reason": "first stroke"}]
    chained.reject(1, "check", {"status": "REJECTED", "failure": {"status": "COLLISION"}})
    assert "chain_from" not in jobs(chained)["propose", 1] and chained.slots[1]["unchained"]
    chained.apply(jobs(chained)["propose", 1], offered("n1"))
    assert not jobs(chained)["check", 0]["chained_out"]
    chained.apply(jobs(chained)["propose", 2], offered("n6"))
    assert chained.lift_offs() == [
        {"before": "a", "reason": "first stroke"},
        {"before": "b", "reason": "chain blocked: COLLISION"},
        {"before": "c", "reason": "not continuous"},
    ]
    refused = ForwardSchedule(["a", "b", "c"], "binding", rows=rows)
    refused.apply(jobs(refused)["propose", 0], offered("n1"))
    refused.apply(jobs(refused)["propose", 1], {"status": "NO_CANDIDATE"})
    assert refused.slots[1]["attempts"] == 0 and "chain_from" not in jobs(refused)["propose", 1]
    refused.apply(jobs(refused)["propose", 1], offered("n1"))
    assert refused.lift_offs()[1] == {"before": "b", "reason": "chain blocked: NO_CANDIDATE"}

    # Detail the arm cannot draw is replaced, one for one, by the longest spare
    # that still fits the detail step's budget: 2 m and 15 estimated minutes.
    # Chosen strokes stay chosen, and the number of strokes (lift-offs) never grows.
    lines = [("long", 1.2), ("mid", 0.7), ("short", 0.3), ("tiny", 0.1), ("bit", 0.05)]
    assert choose_detail(lines, 2.0, 15.0) == ["long", "mid", "tiny"]
    kept = ["long", "mid", "tiny"]
    assert choose_detail(lines, 2.0, 15.0, keep=kept, dropped={"long"}, most=1) == [
        "mid", "tiny", "short"
    ]
    assert choose_detail(lines, 2.0, 15.0, keep=kept, dropped={"mid", "tiny"}, most=2) == [
        "long", "short", "bit"
    ]
    # 1.2 m takes 120 s at 10 mm/s, plus 20 s for its stroke: over 2 minutes.
    assert choose_detail(lines, 2.0, 2.0) == ["mid", "tiny"]
    # A stroke with no alternatives left is reported with where it last failed.
    stuck = ForwardSchedule(["a"], "binding", maximum_attempts=1)
    stuck.apply(jobs(stuck)["propose", 0], candidate("a"))
    stuck.reject(0, "check", {"status": "REJECTED", "failure": {"status": "COLLISION"}})
    assert stuck.blocked() == [
        {
            "action": "a",
            "outcome": "NO_VERIFIED_CANDIDATE",
            "last_failure": {"stage": "check", "status": "COLLISION"},
        }
    ]



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
        tmp_path / "plan.json",
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
    plan_file = tmp_path / "plan.json"
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


