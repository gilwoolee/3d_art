import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from pen_art.kinematics import geometric_jacobian, inverse_candidates
from pen_art.robot import SerialArm


@pytest.fixture
def arm(tmp_path):
    path = tmp_path / "arm.urdf"
    links = ["base", "one", "two", "flange"]
    joints = []
    for i, axis in enumerate(["0 0 1", "0 1 0", "1 0 0"]):
        joints.append(f'''<joint name="joint{i}" type="revolute">
        <parent link="{links[i]}"/><child link="{links[i + 1]}"/>
        <origin xyz="0.1 0 0.1"/><axis xyz="{axis}"/>
        <limit lower="-2.5" upper="2.5" velocity="1" effort="1"/></joint>''')
    path.write_text(
        '<robot name="test">'
        + "".join(f'<link name="{n}"/>' for n in links)
        + "".join(joints)
        + "</robot>"
    )
    return SerialArm(path)


def test_pen_tip_jacobian_matches_finite_difference(arm):
    q = np.array([0.3, -0.7, 0.2])
    tcp = np.eye(4)
    tcp[:3, 3] = [0.02, 0.03, 0.15]
    analytic = geometric_jacobian(arm, q, tcp)
    epsilon = 1e-6
    for i in range(3):
        delta = np.eye(3)[i] * epsilon
        plus, minus = arm.forward(q + delta, tcp), arm.forward(q - delta, tcp)
        numerical = np.r_[
            (plus[:3, 3] - minus[:3, 3]) / (2 * epsilon),
            Rotation.from_matrix(plus[:3, :3] @ minus[:3, :3].T).as_rotvec() / (2 * epsilon),
        ]
        np.testing.assert_allclose(analytic[:, i], numerical, atol=1e-8)


def test_inverse_uses_tcp_offset_and_rejects_unreachable_pose(arm):
    tcp = np.eye(4)
    tcp[2, 3] = 0.15
    target = arm.forward([0.3, -0.7, 0.2], tcp)
    solutions = inverse_candidates(arm, target, tcp, [np.zeros(3), np.ones(3) * 0.2])
    assert solutions
    np.testing.assert_allclose(arm.forward(solutions[0]["positions"], tcp), target, atol=1e-5)
    assert not solutions[0]["collision_checked"]
    target[0, 3] = 10
    assert not inverse_candidates(arm, target, tcp, [np.zeros(3)], maximum_evaluations=30)


def test_pose_log_jacobian_matches_nonzero_error_finite_difference(arm):
    from pen_art.kinematics import pose_error, pose_jacobian

    q = np.array([0.3, -0.7, 0.2])
    offset = np.eye(4)
    offset[2, 3] = 0.16
    target = arm.forward([-0.4, 0.1, -0.3], offset)
    analytic = pose_jacobian(arm, q, offset, target)
    for index in range(3):
        step = np.eye(3)[index] * 1e-6
        plus = pose_error(arm.forward(q + step, offset), target)
        minus = pose_error(arm.forward(q - step, offset), target)
        np.testing.assert_allclose(analytic[:, index], (plus - minus) / 2e-6, atol=1e-8)
