import fcl
import numpy as np

from pen_art.planning.collision import RobotCollisionModel, placed
from pen_art.robot import SerialArm


def make_robot(tmp_path):
    urdf = tmp_path / "arm.urdf"
    urdf.write_text("""<robot name="test">
    <link name="base"><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
    <link name="flange"><collision><origin xyz="1 0 0"/>
      <geometry><sphere radius="0.1"/></geometry></collision></link>
    <joint name="joint" type="revolute"><parent link="base"/><child link="flange"/>
      <axis xyz="0 0 1"/><limit lower="-3.14" upper="3.14" velocity="1" effort="1"/></joint>
    </robot>""")
    model = RobotCollisionModel(SerialArm(urdf))
    return model


def test_continuous_check_finds_collision_between_clear_endpoints(tmp_path):
    model = make_robot(tmp_path)
    matrix = np.eye(4)
    angle = 0.13
    matrix[:3, 3] = [np.cos(angle), np.sin(angle), 0]
    obstacles = {"obstacle": placed(fcl.Sphere(0.01), matrix)}
    assert model.check([-0.8], obstacles) == []
    assert model.check([0.8], obstacles) == []
    result = model.check_edge([-0.8], [0.8], obstacles, margin_m=0)
    assert result["status"] == "COLLISION"
    assert model.check(result["positions"], obstacles)
