"""Parallel planning must never keep a result whose assumed inputs were replaced."""


from concurrent.futures import Future
from time import monotonic

import numpy as np
from test_invariant_verification import setup

from pen_art.planning.clock_path import ClockEdge, ClockPath
from pen_art.planning.parallel_build import Coordinator, ForwardSchedule, MaterialPrefix
from pen_art.planning.scene import MaterialScene
from pen_art.planning.turntable import IndexedTurntable, index_angle


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
        return candidate(f"{spec['index']}-{len(spec.get('skip', []))}")
    return checked(spec["index"]) if spec["kind"] == "check" else {"status": "OK"}


def test_coordinator_repairs_blocked_material_and_drops_stale_results(tmp_path):
    held, log = {}, []

    def submit(spec):
        log.append((spec["kind"], spec["index"], spec.get("skip")))
        if spec["kind"] == "check" and spec["index"] == 2 and "late" not in held:
            # Finishes only after the upstream candidate has been replaced.
            held["late"] = Future()
            return held["late"]
        if spec["kind"] == "check" and spec["index"] == 0 and spec["chain"] == []:
            if spec["candidate"] == held.setdefault("first", spec["candidate"]):
                return resolved({"status": "REJECTED", "failure": {"status": "COLLISION"}})
        if spec["kind"] == "propose" and spec["index"] == 0 and spec["skip"]:
            held["late"].set_result(checked(2))
        return resolved(answer(spec))

    schedule = ForwardSchedule(["a", "b", "c"], "binding")
    coordinator = Coordinator(schedule, submit, 3, tmp_path, deadline=monotonic() + 30)
    assert coordinator.run() == "COMPLETE"
    assert schedule.stale >= 1
    assert schedule.slots[0]["attempts"] == 1 and schedule.slots[0]["skip"] == ["parent-0-0"]
    assert ("propose", 0, ["parent-0-0"]) in log
    assert [job for job in coordinator.jobs if not job["current"]]

    # A stroke that can start where its predecessor ends is proposed from that
    # end and drawn without lifting; if that fails it lifts as usual.
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
    assert ("propose", 1) not in jobs(chained) and ("propose", 2) in jobs(chained)
    chained.apply(jobs(chained)["propose", 0], offered("n1"))
    second = jobs(chained)["propose", 1]
    assert second["chain_from"]["node"] == "n1" and second["chain_from"]["q"] == [0.1]
    assert ("check", 0) not in jobs(chained)
    chained.apply(second, offered("n1", second["chain_from"]["previous"]))
    assert jobs(chained)["check", 0]["chained_out"] and jobs(chained)["check", 1]["chained_in"]
    chained.reject(1, "check", {"status": "REJECTED", "failure": {"status": "COLLISION"}})
    assert "chain_from" not in jobs(chained)["propose", 1] and chained.slots[1]["unchained"]
    chained.apply(jobs(chained)["propose", 1], offered("n1"))
    assert not jobs(chained)["check", 0]["chained_out"]
    refused = ForwardSchedule(["a", "b", "c"], "binding", rows=rows)
    refused.apply(jobs(refused)["propose", 0], offered("n1"))
    refused.apply(jobs(refused)["propose", 1], {"status": "NO_CANDIDATE"})
    assert refused.slots[1]["attempts"] == 0 and "chain_from" not in jobs(refused)["propose", 1]


def table():
    return IndexedTurntable(
        np.eye(4), np.eye(4), 1.0, 1.0, 0.1, 0.2, 0.01, 0.1, 0.05, 1.0, home_index=0
    )


def journal(name, x, count=3):
    return [
        (
            "add",
            [
                {
                    "start": [x, 0.01 * i, 0.0],
                    "end": [x, 0.01 * i, 0.02],
                    "radius_m": 0.001,
                    "name": f"{name}/{i}",
                }
                for i in range(count)
            ],
        )
    ]


def sequential(turntable, journals, chain, final=None):
    """The scene one process would hold after applying the same operations."""
    scene = MaterialScene()
    scene.table_index = turntable.home_index

    def turn(scene, target):
        current = scene.table_index
        for step in turntable.steps(current, target):
            scene = turntable.reposition(scene, index_angle(current), index_angle(step))
            current = step
        return scene

    for key, index in chain:
        if index is not None and index != scene.table_index:
            scene = turn(scene, index)
        for operation in journals[key]:
            if operation[0] == "turn":
                scene = turntable.reposition(
                    scene, index_angle(operation[1]), index_angle(operation[2])
                )
            else:
                for row in operation[1]:
                    scene.add(row["start"], row["end"], row["radius_m"], row["name"])
    return scene if final is None or final == scene.table_index else turn(scene, final)


def test_material_prefix_matches_sequential_material_for_any_request_order():
    turntable = table()
    journals = {name: journal(name, 0.05 * (i + 1)) for i, name in enumerate("abcd")}
    journals["b2"] = journal("b2", 0.5)
    journals["turned"] = [("turn", 0, 1), *journal("turned", 0.3)]
    loads = []
    prefix = MaterialPrefix(turntable, lambda key: loads.append(key) or journals[key])

    def same(chain, final=None):
        got = prefix.scene_for(chain, final_index=final)
        want = sequential(turntable, journals, chain, final)
        assert got.geometry_prefix() == want.geometry_prefix()
        assert [row["name"] for row in got.segments] == [row["name"] for row in want.segments]

    chain = [("a", 0), ("b", 0), ("c", 0)]
    same(chain)
    same(chain + [("d", 0)])
    assert loads == ["a", "b", "c", "d"] and prefix.rebuilds == 0
    # A shorter prefix shares capsules and leaves the longer scene held.
    same(chain[:2])
    same(chain + [("d", 0)])
    assert loads == ["a", "b", "c", "d"] and prefix.rebuilds == 0
    # A replaced upstream deposit keeps only the common leading material.
    same([("a", 0), ("b2", 0), ("c", 0)])
    assert prefix.rebuilds == 1 and loads[-2:] == ["b2", "c"]
    same([])
    # Indexed turns reposition every earlier capsule, exactly as in sequence.
    turned = [("a", 0), ("b", 1), ("c", 1)]
    same(turned)
    same(turned[:1])
    same(turned, final=0)
    same([("a", None), ("turned", None), ("c", None)])
    same([("a", None), ("turned", None)])


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


def test_parallel_replay_reaches_the_serial_verdict_and_material(tmp_path, monkeypatch):
    replay = replay_fixture(tmp_path, monkeypatch, tcp_radius=2)
    serial = replay.replay_cell(tmp_path, "unused", 60)
    # One edge per job: the drawing action itself is split across workers.
    monkeypatch.setattr(replay, "PARALLEL_CHUNK_EDGES", 1)
    monkeypatch.setattr(replay, "PARALLEL_JOBS_PER_WORKER", 100)
    parallel = replay.replay_cell(tmp_path, "unused", 60, workers=2)
    assert serial["status"] == parallel["status"] == "CERTIFIED_GEOMETRIC_REPLAY"
    assert serial["checked_intervals"] == 4
    assert "parallel_workers" not in serial and parallel["parallel_workers"] == 2
    assert serial["final_material"] == parallel["final_material"]
    assert serial["final_material"]["capsules"] > 0
    for name in ("checked_intervals", "checked_edges", "checked_actions", "collision_scope"):
        assert serial[name] == parallel[name]
    assert not (tmp_path / "dual_arm_replay_journals").exists()
    assert replay.read_json(tmp_path / "section_build.json")["status"] == "FULL_BUILD_VERIFIED"
    assert replay.read_json(tmp_path / "run.json")["stages"]["motion"]["status"] == "complete"
    check_export_without_verification(tmp_path / "export", monkeypatch)


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


def check_export_without_verification(run, monkeypatch):
    """Export retains exact clocks/events and never invokes the heavy verifier."""
    from types import SimpleNamespace

    import pen_art.planning.cell_replay as replay
    from pen_art.io import read_json, write_json
    from pen_art.planning.full_build_export import export_sections

    def forbidden(*args, **kwargs):
        raise AssertionError("Export must not run final verification")

    monkeypatch.setattr(replay, "replay_cell", forbidden)
    for name in ("cell_manifest", "cell_drawing_variants", "construction_plan",
                 "stroke_graph", "fabrication_config"):
        write_json(run / f"{name}.json", {})
    write_json(run / "run.json", {"stages": {}})
    section = dict(id="stroke", parent_curve="source", source_interval=[0, 1],
                   members=["wire"], stage="framework", start="a", end="b", role="contour",
                   junction_visits=[], closed=False)
    edges = [
        ClockEdge(0, 0.1, np.array([0.]), np.array([0.]), "PEN_START_WAIT", None),
        ClockEdge(0.1, 1.1, np.array([0.]), np.array([0.1]), "DRAW", None),
        ClockEdge(1.1, 1.2, np.array([0.1]), np.array([0.1]), "PEN_STOP_WAIT", None),
    ]
    clock = ClockPath(edges)
    returned = ClockPath([ClockEdge(0, 1, np.array([0.1]), np.array([0.]), "TRAVEL", None)])
    build = SimpleNamespace(
        run=run, actions={"stroke": section}, initial=np.array([0.]), cooling=1.,
        arm=SimpleNamespace(identity="arm"),
        planner=SimpleNamespace(ground=["a"], turntable=None, settings={}, transit={},
                                timing={}, process={}, initial_pose_repairs={}),
    )
    reports = []
    build.report = lambda status, **extra: reports.append((status, extra))
    witness = ("stroke", SimpleNamespace(payload=clock, start_anchor="a", variant="cache"),
               {"reverse": False})
    result = export_sections(build, [witness], returned)
    assert result == {"status": "SCHEDULE_SAVED", "replay_status": "NOT_RUN"}
    plan = read_json(run / "offline_dual_arm_plan.json")
    assert plan["actions"][0]["trajectory"] == clock.record()
    assert plan["actions"][1]["trajectory"] == returned.record()
    assert plan["modeled_duration_s"] == clock.duration_s + returned.duration_s
    assert [event["command"] for event in plan["events"]] == ["FEED_ON", "FEED_OFF"]
    assert plan["requires_final_verification"] and not plan["executable"]
    assert plan["checked_intervals"] == 0 and "packed_clock_binding" not in plan
    assert reports[-1][0] == "FULL_BUILD_PLANNED"
    assert read_json(run / "run.json")["stages"]["validation"]["status"] == "pending"
    assert not list(run.glob("*.npy")) and not list(run.glob("*.clocks.json"))
    assert not (run / "offline_dual_arm_validation.json").exists()
