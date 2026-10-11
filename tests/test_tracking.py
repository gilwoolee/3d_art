import numpy as np
import pytest

from pen_art.planning.tracking import tracking_bounds
from pen_art.robot import SerialArm


def test_bound_contains_bowing_between_exact_endpoint_poses(tmp_path):
    urdf = tmp_path / "arm.urdf"
    urdf.write_text("""<robot name="planar">
      <link name="base"/><link name="one"/><link name="flange"/>
      <joint name="a" type="revolute"><parent link="base"/><child link="one"/>
        <axis xyz="0 0 1"/><limit lower="-2" upper="2" velocity="1" effort="1"/></joint>
      <joint name="b" type="revolute"><parent link="one"/><child link="flange"/>
        <origin xyz="1 0 0"/><axis xyz="0 0 1"/>
        <limit lower="-2" upper="2" velocity="1" effort="1"/></joint></robot>""")
    arm = SerialArm(urdf)
    q0, q1, offset = np.array([-0.1, 0.1]), np.array([0.1, -0.1]), np.eye(4)
    t0, t1 = arm.forward(q0), arm.forward(q1)
    bound = tracking_bounds(arm, offset, q0, q1, t0, t1)
    assert 0.00499 < bound["position_error_bound_m"] < 0.00501
    # Both endpoints are exact, yet the midpoint bows about 5 mm away.
    for alpha in np.linspace(0, 1, 101):
        actual = arm.forward(q0 * (1 - alpha) + q1 * alpha)
        target = t0[:3, 3] * (1 - alpha) + t1[:3, 3] * alpha
        assert np.linalg.norm(actual[:3, 3] - target) <= bound["position_error_bound_m"] + 1e-12
    changed = t1.copy()
    changed[:3, :3] = arm.forward([0.2, 0.1])[:3, :3]
    with pytest.raises(ValueError, match="constant target orientation"):
        tracking_bounds(arm, offset, q0, q1, t0, changed)
