import numpy as np
import pytest

from pen_art.robot import SerialArm


def test_serial_transforms_limits_and_tcp_offset(tmp_path):
    path = tmp_path / "arm.urdf"
    path.write_text("""<robot name="test"><link name="base"/><link name="link"/>
        <link name="flange"/>
        <joint name="shoulder" type="revolute"><parent link="base"/><child link="link"/>
          <origin xyz="1 0 0"/><axis xyz="0 0 1"/>
          <limit lower="-3.14" upper="3.14" velocity="1" effort="1"/></joint>
        <joint name="tool" type="fixed"><parent link="link"/><child link="flange"/>
          <origin xyz="1 0 0"/></joint></robot>""")
    robot = SerialArm(path)
    np.testing.assert_allclose(robot.forward([np.pi / 2])[:3, 3], [1, 1, 0], atol=1e-12)
    tcp = np.eye(4)
    tcp[0, 3] = 0.2
    np.testing.assert_allclose(robot.forward([np.pi / 2], tcp)[:3, 3], [1, 1.2, 0], atol=1e-12)
    with pytest.raises(ValueError, match="limits"):
        robot.forward([4])
    with pytest.raises(ValueError, match="finite"):
        robot.forward([float("nan")])
