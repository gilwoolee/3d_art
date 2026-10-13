"""Boundary tests for persistent proposals and continuous clearance reuse."""


import fcl
import numpy as np

from pen_art.planning.collision import RobotCollisionModel, placed
from pen_art.planning.motion_library import CachedCollisionModel
from pen_art.robot import SerialArm


def small_model(tmp_path):
    path = tmp_path / "arm.urdf"
    path.write_text("""<robot name="one"><link name="base"/><link name="tip">
      <collision><origin xyz="1 0 0"/><geometry><sphere radius="0.02"/></geometry></collision>
      </link><joint name="joint" type="revolute"><parent link="base"/><child link="tip"/>
      <origin xyz="0 0 0"/><axis xyz="0 0 1"/><limit lower="-2" upper="2" velocity="1"/>
      </joint></robot>""")
    return RobotCollisionModel(SerialArm(path, tip="tip"))


def test_same_pose_replaced_geometry_cannot_reuse_old_static_bounds(tmp_path):
    model = small_model(tmp_path)
    transform = np.eye(4)
    transform[0, 3] = 4
    obstacles = {"obstacle": placed(fcl.Box(.1, .1, .1), transform)}
    metadata = {"obstacle": {"type": "Box", "side": [.1, .1, .1]}}
    cached = CachedCollisionModel(model, tmp_path, static_geometry=metadata)
    assert not cached.check([0], obstacles)
    assert not cached.check([0], obstacles)
    assert cached.hits > 0
    # Same name/type/pose but the new box encloses the robot's sphere.
    obstacles["obstacle"] = placed(fcl.Box(8, 8, 8), transform)
    assert cached.check([0], obstacles)
    assert model.check([0], obstacles)
