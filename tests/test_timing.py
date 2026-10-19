import numpy as np

from pen_art.robot import SerialArm
from pen_art.timing import make_path


def test_timing_must_sweep_exactly_the_checked_geometric_path(tmp_path):
    from copy import deepcopy
    from types import SimpleNamespace

    from pen_art.planning.clock_path import ClockPath, timed_path
    from pen_art.planning.geometric_path import geometric_path, same_sweep

    urdf = tmp_path / "robot.urdf"
    urdf.write_text("""<robot name="two">
      <link name="base"/><link name="upper"/><link name="flange"/>
      <joint name="a" type="revolute"><parent link="base"/><child link="upper"/>
      <axis xyz="0 0 1"/><limit lower="-2" upper="2" velocity="1" effort="1"/></joint>
      <joint name="b" type="revolute"><parent link="upper"/><child link="flange"/>
      <origin xyz="0.2 0 0"/><axis xyz="0 0 1"/>
      <limit lower="-2" upper="2" velocity="1" effort="1"/></joint>
    </robot>""")
    offset = np.eye(4)
    offset[0, 3] = 0.2
    arm = SimpleNamespace(robot=SerialArm(urdf), flange_to_tcp=offset)
    # The second joint reverses inside the drawing, so the sweep is not monotone.
    drawing = np.column_stack([np.linspace(0.1, 0.9, 9), 0.4 + 0.3 * np.sin(np.linspace(0, 3, 9))])
    corner = arm.robot.forward(drawing[4], offset)[:3, 3]
    operations = [
        {"kind": "TRAVEL", "positions": [[0.0, 0.2], [0.05, 0.3], drawing[0]]},
        {"kind": "PEN_START_WAIT", "positions": [drawing[0]], "hold_s": 0.2, "contact_m": corner},
        {"kind": "JOIN", "positions": [drawing[0]], "hold_s": 0.0, "contact_m": corner},
        {
            "kind": "DRAW",
            "positions": drawing,
            "stroke_id": "stroke",
            "material_radius_m": 0.0005,
            "break_indices": [4],
            "vertex_holds": {4: {"contact_m": corner, "duration_s": 1.0}},
            "start_lag_s": 0.2,
            "stop_lag_s": 0.2,
        },
        {"kind": "WITHDRAW", "positions": [drawing[-1], [1.0, 0.3]], "contact_m": corner},
    ]
    timed = timed_path(
        deepcopy(operations), arm, np.array([0.4, 0.4]), np.array([0.5, 0.5]), 0.02, 0.05,
        drawing_clock="cubic_toppra",
    )
    geometric = geometric_path(deepcopy(operations), arm, drawing_clock="cubic_toppra")
    verdict = same_sweep(geometric, timed)
    assert verdict["status"] == "SAME_SWEEP", verdict
    drawn = [edge for edge in geometric.edges if edge.coefficients is not None]
    # Sixteen cubic pieces replace several hundred retiming intervals.
    assert len(drawn) == 16 and verdict["timed_edges"] > 10 * verdict["geometric_edges"]
    assert verdict["maximum_difference_rad"] < 1e-9
    # Both describe the same configurations at every piece boundary.
    np.testing.assert_allclose(drawn[0].q0, drawing[0], atol=1e-12)
    np.testing.assert_allclose(drawn[-1].q1, drawing[-1], atol=1e-12)

    def altered(change):
        edges = [deepcopy(edge) for edge in timed.edges]
        edges = change(edges) or edges
        return same_sweep(geometric, ClockPath(edges))["status"]

    index = next(i for i, edge in enumerate(timed.edges) if edge.coefficients is not None)

    def bend(edges):
        edges[index + 3].coefficients[1][2] += 1e-6

    def reverse(edges):
        edges[index + 3].coefficients = [row[:1] + [-c for c in row[1:]] for row in
                                         edges[index + 3].coefficients]

    def move_travel(edges):
        edges[0].q1 = edges[0].q1 + 1e-9

    for change in (bend, reverse, move_travel, lambda edges: edges[:index + 5] + edges[index + 6:],
                   lambda edges: edges[:-1]):
        assert altered(change) == "SWEEP_MISMATCH"
    assert same_sweep(timed, timed)["reason"] == "not_a_geometric_path"
    # A piece may be stationary in its own parameter where a timed edge starts.
    from numpy.polynomial import Polynomial

    from pen_art.planning.geometric_path import _clock_onto

    stationary = [Polynomial([0.2, 0, 0.5, -0.1]), Polynomial([-0.4, 0, -0.3, 0.2])]
    clock = Polynomial([0.0, 0.1, 0.3])
    reached, difference = _clock_onto(stationary, [p(clock) for p in stationary], 0.0)
    assert abs(reached - 0.4) < 1e-9 and difference < 1e-9
    assert _clock_onto(stationary, [p(clock) + 1e-6 for p in stationary], 0.0)[1] > 1e-9

    # The arm must follow the tip while the pen works, never swing with it still.
    from pen_art.planning.clock_path import ClockEdge, posture_change

    assert posture_change(geometric, arm) is None and posture_change(timed, arm) is None
    here, tip = np.array([0.3, 0.4]), arm.robot.forward([0.3, 0.4], offset)[:3, 3]

    def still(first, second):
        return ClockEdge(0.0, 1.0, first, second, "DRAW", "stroke", None, 0.0005, tip, tip)

    swing = posture_change(ClockPath([still(here, here + 0.2)]), arm)
    assert swing["status"] == "POSTURE_CHANGE" and swing["scope"] == "edge"
    creep = [still(here + 0.001 * k, here + 0.001 * (k + 1)) for k in range(30)]
    assert posture_change(ClockPath(creep), arm)["scope"] == "run"


    # A path whose pen stops at a knot is fitted run by run: a run of one piece
    # is the straight joint segment, and its clock gets knots for the ramps.
    from pen_art.timing import ramp_knots

    knots = np.array([[0.1, 0.4], [0.2, 0.6], [0.5, 0.5], [0.9, 0.2]])
    tips = [arm.robot.forward(q, offset)[:3, 3] for q in knots]
    joined = make_path(knots, tips, interpolation="pchip")
    stopped = make_path(knots, tips, interpolation="pchip", breaks=[1])
    assert np.allclose(stopped.cspl(stopped.cspl.x), knots, atol=1e-12)
    assert np.allclose(stopped.cspl.c[:2, 0], 0, atol=1e-9)
    assert not np.allclose(joined.cspl.c[:2, 0], 0, atol=1e-6)
    extra = ramp_knots(stopped, arm.robot, offset)
    arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(tips, axis=0), axis=1))]
    travelled = np.sort(np.interp(extra, stopped.cspl.x, arc))
    assert len(extra) > 4 and abs(travelled[0] - 0.001) < 1e-9
    assert np.min(np.diff(np.sort(np.r_[arc, travelled]))) >= 0.00025 - 1e-12
    member = deepcopy(operations)
    member[3] = {**member[3], "positions": drawing[[0, 4, 8]], "break_indices": [1]}
    member[3]["path"] = make_path(
        drawing[[0, 4, 8]],
        [arm.robot.forward(q, offset)[:3, 3] for q in drawing[[0, 4, 8]]],
        interpolation="pchip",
        breaks=[1],
    )
    member[4]["positions"] = [drawing[8]]
    pieces = geometric_path(member, arm, drawing_clock="cubic_toppra")
    ramped = timed_path(
        member, arm, np.ones(2), np.full(2, 0.5), 0.03, 0.1, drawing_clock="cubic_toppra",
        acceleration_constraint="cubic_bernstein", grid_samples=0, grid_midpoints=False,
        solver="scan", grid_ramps=True,
    )
    assert sum(edge.kind == "DRAW" for edge in pieces.edges) == 2
    assert sum(edge.kind == "DRAW" for edge in ramped.edges) > 6
    assert same_sweep(pieces, ramped)["status"] == "SAME_SWEEP"
