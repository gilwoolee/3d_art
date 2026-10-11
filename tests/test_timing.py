import numpy as np
import pytest

from pen_art.io import read_json, sha256, write_json
from pen_art.robot import SerialArm
from pen_art.timing import (
    build_schedule,
    check_position_limits,
    make_path,
    retime_operation,
)


def test_spline_position_limits_cover_between_waypoints():
    path = make_path([[0], [0.9], [0.89], [0]], [[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0]])
    with pytest.raises(ValueError, match="position limits"):
        check_position_limits(path, np.array([-1]), np.array([0.9]))


def test_retimer_keeps_motion_through_waypoints_and_bounds_continuous_dynamics(tmp_path):
    from types import SimpleNamespace

    from pen_art.planning.clock_path import (
        ClockEdge,
        ClockPath,
        polynomial_tcp_speed_bound,
        timed_path,
    )

    urdf = tmp_path / "robot.urdf"
    urdf.write_text("""<robot name="one">
      <link name="base"/><link name="flange"/>
      <joint name="q" type="revolute"><parent link="base"/><child link="flange"/>
      <axis xyz="0 0 1"/><limit lower="-2" upper="2" velocity="1" effort="1"/></joint>
    </robot>""")
    robot = SerialArm(urdf)
    offset = np.eye(4)
    offset[0, 3] = 0.2
    positions = np.linspace(0.1, 0.5, 5)[:, None]
    points = [robot.forward(q, offset)[:3, 3] for q in positions]
    result = retime_operation(
        {"positions": positions, "tcp_path_m": points},
        robot,
        offset,
        np.array([0.4]),
        np.array([0.5]),
        0.02,
        0.03,
    )
    assert result["duration_s"] >= 0.4 * 0.2 / 0.02
    assert result["duration_s"] < 10
    assert result["bounds"]["velocity_rad_s"][0] <= 0.4
    assert result["bounds"]["acceleration_rad_s2"][0] <= 0.5
    assert result["bounds"]["tcp_speed_m_s"] <= 0.02
    np.testing.assert_allclose(np.array(result["velocities"])[[0, -1]], 0, atol=1e-7)
    times, velocity = np.array(result["times_s"]), np.array(result["velocities"])[:, 0]
    assert np.min(velocity[(times > times[-1] * 0.2) & (times < times[-1] * 0.8)]) > 0
    assert result["maximum_joint_deviation_rad"][0] < 1e-12
    arm = SimpleNamespace(robot=robot, flange_to_tcp=offset)
    # The maximum is at the endpoint. Ten subdivisions leave a loose bound
    # above the limit despite the exact speed remaining below it.
    endpoint_fast = ClockEdge(
        0,
        0.004 / 0.003 * 1.00001,
        np.array([0.0]),
        np.array([0.01]),
        "DRAW",
        "endpoint",
        coefficients=[[0, 0.02, -0.01]],
    )
    assert polynomial_tcp_speed_bound(endpoint_fast, arm, 0.003) <= 0.003 + 1e-8
    endpoint_fast.end_s *= 0.99
    assert polynomial_tcp_speed_bound(endpoint_fast, arm, 0.003) > 0.003 + 1e-8
    with pytest.raises(TimeoutError, match="planning budget"):
        polynomial_tcp_speed_bound(endpoint_fast, arm, 0.003, deadline=0)
    operations = [{"kind": "DRAW", "positions": positions, "stroke_id": "arc"}]
    stopped = timed_path(operations, arm, np.array([0.4]), np.array([0.5]), 0.02, 0.03)
    continuous = timed_path(
        operations, arm, np.array([0.4]), np.array([0.5]), 0.02, 0.03, drawing_clock="cubic_toppra"
    )
    assert continuous.duration_s < stopped.duration_s
    saved = ClockPath.from_record(continuous.record())
    np.testing.assert_allclose(saved.position(saved.duration_s / 2), [0.3], atol=1e-6)
    for edge in saved.edges:
        assert polynomial_tcp_speed_bound(edge, arm, 0.02) <= 0.02 + 1e-8
    middle = saved.edge_at(saved.duration_s / 2)
    assert abs(middle.piece().polynomials[0].deriv()(0.5)) > 0
    damaged = continuous.record()
    damaged["edges"][0]["coefficients"][0][0] += 0.1
    with pytest.raises(ValueError, match="endpoints"):
        ClockPath.from_record(damaged)
    turns = np.array([[0.1], [0.5], [0.2], [0.6]])
    turn_points = [robot.forward(q, offset)[:3, 3] for q in turns]
    bounded = retime_operation(
        {"positions": turns, "tcp_path_m": turn_points},
        robot,
        offset,
        np.array([0.4]),
        np.array([0.5]),
        0.02,
        0.03,
        path=make_path(turns, turn_points, interpolation="pchip", subdivisions=2),
        continuous_acceleration=True,
    )
    assert bounded["time_scale_correction"] < 1.01
    assert bounded["bounds"]["acceleration_rad_s2"][0] <= 0.5


@pytest.fixture
def schedule_fixture(tmp_path, monkeypatch):
    from pathlib import Path

    import yaml

    profile = yaml.safe_load(Path("configs/timing.yaml").read_text())
    profile["joint_acceleration_rad_s2"] = [0.5]
    profile["startup_s"] = 2
    profile["shutdown_s"] = 3
    config = tmp_path / "timing.yaml"
    config.write_text(yaml.safe_dump(profile))
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("""<robot name="one"><link name="base"/><link name="flange"/>
      <joint name="q" type="revolute"><parent link="base"/><child link="flange"/>
      <axis xyz="0 0 1"/><limit lower="-2" upper="2" velocity="1" effort="1"/></joint>
    </robot>""")
    write_json(tmp_path / "run.json", {"stages": {}})
    write_json(
        tmp_path / "fabrication_config.json",
        {
            "process": {
                "nominal_speed_m_s": 0.003,
                "join_s": 1,
                "start_lag_s": 0.5,
                "stop_lag_s": 0.5,
            }
        },
    )
    write_json(
        tmp_path / "sequence_motion.json",
        {
            "status": "GEOMETRIC_SEQUENCE_PREVIEW",
            "inputs": {"robot.urdf": sha256(urdf)},
            "flange_to_tcp": np.eye(4).tolist(),
            "parking_pose": {"positions": [0]},
            "operations": [
                {"id": "first", "kind": "TRAVEL", "stroke_id": "a", "positions": [[0], [0.1]]},
                {"id": "stroke", "kind": "DRAW", "stroke_id": "a", "positions": [[0.1], [0.2]]},
                {"id": "return", "kind": "RETURN", "stroke_id": None, "positions": [[0.2], [0]]},
            ],
        },
    )

    def retime(operation, *args):
        return {
            "duration_s": 0.1 if operation["kind"] == "DRAW" else 1,
            "positions": operation["positions"],
        }

    monkeypatch.setattr("pen_art.timing.retime_operation", retime)
    return tmp_path, config


def test_events_use_common_clock_and_short_response_is_reported(schedule_fixture):
    run, profile = schedule_fixture
    result = build_schedule(run, profile)
    assert result["modeled_duration_s"] == pytest.approx(8.6)
    assert len(result["events"]) == 2
    assert [event["command"] for event in result["events"]] == ["START_FEED", "STOP_FEED"]
    draw = next(op for op in result["operations"] if op["kind"] == "DRAW")
    assert result["events"][0]["time_s"] == pytest.approx(draw["start_s"] - 0.5)
    assert result["events"][1]["time_s"] == pytest.approx(draw["start_s"])
    assert result["timing_conflicts"][0]["code"] == "ASSUMED_STOP_LAG_EXCEEDS_STROKE"
    assert result["unknown_durations"] == ["cooling_s"]
    assert result["executable"] is False
    assert result["geometry_validation"]["status"] == "REQUIRES_SMOOTH_PATH_AUDIT"
    assert all(not op["extrusion_expected"] for op in result["operations"] if op["kind"] != "DRAW")


def test_discontinuous_sequence_is_rejected_before_teleporting(schedule_fixture):
    run, profile = schedule_fixture
    sequence = read_json(run / "sequence_motion.json")
    sequence["operations"][1]["positions"][0] = [0.9]
    write_json(run / "sequence_motion.json", sequence)
    with pytest.raises(ValueError, match="Discontinuous"):
        build_schedule(run, profile)
    assert read_json(run / "run.json")["stages"]["timing"]["status"] == "failed"


def test_replay_preserves_progress_and_reports_unfinished_speed_check(
    schedule_fixture, monkeypatch
):
    from types import SimpleNamespace

    import pen_art.planning.cell_replay as replay
    from pen_art.planning.clock_path import ClockEdge, ClockPath

    run, _ = schedule_fixture
    robot = SerialArm(run / "robot.urdf")
    arm = SimpleNamespace(robot=robot, flange_to_tcp=np.eye(4), urdf=run / "robot.urdf")
    path = ClockPath(
        [
            ClockEdge(
                0,
                1,
                np.array([0.0]),
                np.array([0.01]),
                "DRAW",
                None,
                coefficients=[[0, 0, 0.03, -0.02]],
            ),
            ClockEdge(
                1,
                2,
                np.array([0.01]),
                np.array([0.0]),
                "DRAW",
                None,
                coefficients=[[0.01, 0, -0.03, 0.02]],
            ),
        ]
    )
    write_json(
        run / "replay_settings.json",
        {
            "search": {},
            "transit": {},
            "timing": {
                "travel_speed_m_s": 0.03,
                "velocity_fraction": 0.5,
                "joint_acceleration_rad_s2": [0.5],
            },
            "process": {"nominal_speed_m_s": 0.003},
        },
    )
    write_json(
        run / "dual_arm_plan.json",
        {
            "inputs": {},
            "settings_artifact": "replay_settings.json",
            "initial_positions": {"arm_1": [0]},
            "modeled_duration_s": 2,
            "limitations": [],
            "events": [],
            "full_schedule": True,
            "actions": [
                {
                    "arm": "arm_1",
                    "stroke_id": "return",
                    "variant": "test",
                    "start_s": 0,
                    "end_s": 2,
                    "start_anchor": "parking",
                    "auxiliary": True,
                    "trajectory": path.record(),
                }
            ],
        },
    )
    collision_replays = []

    def validate_interval(state, new, end):
        collision_replays.append(end)
        return state.payload, None

    monkeypatch.setattr(
        replay,
        "CellPlanner",
        lambda run, profile, settings, deadline, **kw: SimpleNamespace(
            arms={"arm_1": arm},
            tasks=[],
            ground=[],
            deadline=deadline,
            profile={},
            checked_intervals=0,
            validate_interval=validate_interval,
        ),
    )
    calls = []

    def speed_check(edge, arm, limit, *, deadline):
        progress = read_json(run / "dual_arm_validation.json")
        assert progress["source_plan_sha256"] == sha256(run / "dual_arm_plan.json")
        assert progress["status"] == "RUNNING" and progress["phase"] == "PATH_CHECKS"
        assert deadline > 0
        calls.append(edge)
        if len(calls) == 2:
            raise TimeoutError("unfinished speed enclosure")
        return 0

    monkeypatch.setattr(replay, "polynomial_tcp_speed_bound", speed_check)
    result = replay.replay_cell(run, "unused-profile", maximum_seconds=5)
    assert result["status"] == "REPLAY_TIME_BUDGET"
    assert result["checked_edges"] == 1 and result["requested_edges"] == 2
    assert result["completed_strokes"] == 0
    assert result["failures"] == [{"status": "TIME_BUDGET", "phase": "PATH_CHECKS"}]
    assert read_json(run / "run.json")["stages"]["validation"]["status"] == "partial"

    def rejected_speed(*args, **kwargs):
        raise ValueError("Cannot certify saved tip speed")

    monkeypatch.setattr(replay, "polynomial_tcp_speed_bound", rejected_speed)
    with pytest.raises(ValueError, match="Cannot certify"):
        replay.replay_cell(run, "unused-profile", maximum_seconds=5)
    failed = read_json(run / "dual_arm_validation.json")
    assert failed["status"] == "REPLAY_ERROR"
    assert failed["phase"] == "PATH_CHECKS"
    assert failed["checked_edges"] == 0

    certificates = set()
    monkeypatch.setattr(replay, "polynomial_tcp_speed_bound", lambda *args, **kw: 0)
    result = replay.replay_cell(run, "unused-profile", path_certificates=certificates)
    assert len(certificates) == 1 and result["reused_path_checks"] == 0
    monkeypatch.setattr(replay, "polynomial_tcp_speed_bound", rejected_speed)
    result = replay.replay_cell(run, "unused-profile", path_certificates=certificates)
    assert result["reused_path_checks"] == 1
    assert len(collision_replays) == 2  # Schedule collision checks always run again.
    settings = read_json(run / "replay_settings.json")
    settings["process"]["nominal_speed_m_s"] *= 0.9
    write_json(run / "replay_settings.json", settings)
    with pytest.raises(ValueError, match="Cannot certify"):
        replay.replay_cell(run, "unused-profile", path_certificates=certificates)
