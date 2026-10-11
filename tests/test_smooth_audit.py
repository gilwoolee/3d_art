import numpy as np

from pen_art.planning.smooth_audit import (
    CubicPiece,
    bounded_segments,
    check_tracking,
    tracking_bound,
)
from pen_art.robot import SerialArm


def circle_robot(tmp_path):
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("""<robot name="one"><link name="base"/><link name="tip"/>
    <joint name="q" type="revolute"><parent link="base"/><child link="tip"/>
    <axis xyz="0 0 1"/><limit lower="-3" upper="3" velocity="1" effort="1"/>
    </joint></robot>""")
    robot = SerialArm(urdf, tip="tip")
    offset = np.eye(4)
    offset[0, 3] = 0.2
    return robot, offset


def test_cubic_envelope_covers_reversals_and_subpieces():
    curve = CubicPiece([[0], [-6.4], [6.4], [-0.8]], 1)
    for a, b in [(0, 1), (0, 0.6), (0.2, 0.8)]:
        midpoint = curve.position((a + b) / 2)
        samples = np.array([curve.position(t) for t in np.linspace(a, b, 1001)])
        assert np.all(np.abs(samples - midpoint) <= curve.deviation(a, b) + 1e-12)
        sub = curve.subpiece(a, b)
        np.testing.assert_allclose(sub.position(0), curve.position(a), atol=1e-12)
        np.testing.assert_allclose(sub.position(1), curve.position(b), atol=1e-12)


def test_inflated_chords_enclose_actual_curved_deposition(tmp_path):
    robot, offset = circle_robot(tmp_path)
    curve = CubicPiece([[0.2], [-0.7], [1.1], [0.1]], 1)
    intervals = bounded_segments(curve, robot, offset, 0.00005)
    assert len(intervals) > 1
    assert intervals[0][0] == 0 and intervals[-1][1] == 1
    for a, b, error in intervals:
        p0, p1 = [robot.forward(curve.position(t), offset)[:3, 3] for t in (a, b)]
        for t in np.linspace(a, b, 31):
            p = robot.forward(curve.position(t), offset)[:3, 3]
            u = (t - a) / (b - a)
            assert np.linalg.norm(p - ((1 - u) * p0 + u * p1)) <= error + 1e-12


def test_tracking_bound_encloses_pose_error_between_endpoints(tmp_path):
    robot, offset = circle_robot(tmp_path)
    curve = CubicPiece([[0], [-0.4], [0.4], [0]], 1)
    target = robot.forward([0], offset)
    position_bound, angle_bound = tracking_bound(
        curve, robot, offset, target[:3, 3], target[:3, 3], target[:3, :3]
    )
    for t in np.linspace(0, 1, 1001):
        pose = robot.forward(curve.position(t), offset)
        assert np.linalg.norm(pose[:3, 3] - target[:3, 3]) <= position_bound
        assert abs(curve.position(t)[0]) <= angle_bound
    assert position_bound > 0.01


def test_free_axis_tracking_still_rejects_tip_path_error(tmp_path):
    robot, offset = circle_robot(tmp_path)
    curve = CubicPiece([[0], [0], [0.02], [0]], 1)
    start, end = [robot.forward([q], offset) for q in (0, 0.02)]
    args = (curve, robot, offset, start[:3, 3], end[:3, 3])
    assert check_tracking(*args, None, 0.0003, 0.003)["status"] == "BOUNDED"
    assert check_tracking(*args, start[:3, :3], 0.0003, 0.003)["status"] != "BOUNDED"
    shifted = start[:3, 3] + [0.001, 0, 0]
    assert (
        check_tracking(curve, robot, offset, shifted, end[:3, 3], None, 0.0003, 0.003)["status"]
        != "BOUNDED"
    )
