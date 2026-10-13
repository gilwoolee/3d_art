import numpy as np
import pytest

from pen_art.robot import SerialArm
from pen_art.timing import make_path, retime_operation


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


def test_whole_action_vertex_hold_preserves_spline_and_counts_feed_lags(tmp_path):
    from types import SimpleNamespace

    from pen_art.planning.clock_path import (
        ClockPath,
        restrict_spline,
        timed_path,
    )
    from pen_art.planning.process_clock import feed_events

    urdf = tmp_path / "robot.urdf"
    urdf.write_text("""<robot name="one"><link name="base"/><link name="flange"/>
      <joint name="q" type="revolute"><parent link="base"/><child link="flange"/>
      <axis xyz="0 0 1"/><limit lower="-2" upper="2" velocity="1" effort="1"/></joint>
    </robot>""")
    robot = SerialArm(urdf)
    offset = np.eye(4)
    offset[0, 3] = 0.2
    arm = SimpleNamespace(robot=robot, flange_to_tcp=offset)
    positions = np.array([[0.1], [0.2], [0.3], [0.2], [0.1]])
    points = [robot.forward(q, offset)[:3, 3] for q in positions]
    spline = make_path(positions, points, interpolation="pchip")
    restricted = restrict_spline(spline, 0, 2)
    np.testing.assert_allclose(
        restricted(np.linspace(0, 1, 30)),
        spline(np.linspace(spline.cspl.x[0], spline.cspl.x[2], 30)),
    )
    path = timed_path(
        [
            {
                "kind": "DRAW",
                "positions": positions,
                "path": spline,
                "stroke_id": "whole-zigzag",
                "break_indices": [2],
                "vertex_holds": {2: {"duration_s": 1.0, "contact_m": points[2]}},
                "stop_lag_s": 0.5,
                "start_lag_s": 0.5,
            }
        ],
        arm,
        np.array([0.4]),
        np.array([0.5]),
        0.030,
        0.030,
        drawing_clock="cubic_toppra",
        acceleration_constraint="cubic_bernstein",
    )
    restored = ClockPath.from_record(path.record())
    holds = [edge for edge in restored.edges if edge.kind != "DRAW"]
    assert [edge.kind for edge in holds] == ["PEN_STOP_WAIT", "VERTEX_COOL", "PEN_START_WAIT"]
    assert sum(edge.end_s - edge.start_s for edge in holds) == pytest.approx(1.5)
    assert all(
        np.allclose(edge.q0, positions[2]) and np.allclose(edge.q1, positions[2]) for edge in holds
    )
    assert sum(edge.end_s - edge.start_s for edge in restored.edges) == pytest.approx(
        path.duration_s
    )
    assert {edge.stroke for edge in restored.edges} == {"whole-zigzag"}
    assert [
        event["command"]
        for event in feed_events(
            [{"start_s": 3, "stroke_id": "whole-zigzag", "arm": "arm", "trajectory": path.record()}]
        )
    ] == ["FEED_OFF", "FEED_ON"]
    # Both adjacent moving spans arrive/depart at rest; serialization checks this too.
    before = restored.edges[restored.edges.index(holds[0]) - 1]
    after = restored.edges[restored.edges.index(holds[-1]) + 1]
    assert abs(before.piece().polynomials[0].deriv()(1)) < 1e-8
    assert abs(after.piece().polynomials[0].deriv()(0)) < 1e-8
