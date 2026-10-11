from copy import deepcopy

import fcl
import numpy as np
import pytest
import trimesh

from pen_art.planning.collision import RobotCollisionModel, placed
from pen_art.planning.tool import load_tool, require_matching_tool
from pen_art.robot import SerialArm


@pytest.fixture
def arm(tmp_path):
    path = tmp_path / "arm.urdf"
    path.write_text("""<robot name="tool-test">
      <link name="base"><collision><geometry><sphere radius="0.1"/>
        </geometry></collision></link><link name="flange"/>
      <joint name="joint" type="revolute"><parent link="base"/><child link="flange"/>
        <axis xyz="0 0 1"/><limit lower="-3" upper="3" velocity="1" effort="1"/>
      </joint></robot>""")
    return SerialArm(path)


def profile(shape=None):
    return {
        "schema_version": 1,
        "units": "m",
        "flange_to_tcp": np.eye(4).tolist(),
        "tip_radius_m": 0.0005,
        "bodies": [
            {
                "name": "pen",
                "origin": np.eye(4).tolist(),
                "shape": shape or {"type": "sphere", "radius_m": 0.02},
            }
        ],
    }


def test_tool_does_not_inherit_adjacent_robot_link_exemption(arm, tmp_path):
    spec = profile()
    tool = load_tool(spec, tmp_path, arm, np.eye(4))
    collision = RobotCollisionModel(arm, tool.bodies).check([0])
    assert collision == [{"first": "base:0", "second": "tool/pen", "kind": "overlap"}]
    spec["bodies"][0]["allowed_touch_links"] = ["base"]
    exempt = load_tool(spec, tmp_path, arm, np.eye(4))
    assert RobotCollisionModel(arm, exempt.bodies).check([0]) == []
    assert tool.metadata["profile_sha256"] != exempt.metadata["profile_sha256"]


@pytest.mark.parametrize(
    "shape",
    [
        {"type": "box", "size_m": [0.04, 0.03, 0.02]},
        {"type": "sphere", "radius_m": 0.02},
        {"type": "cylinder", "radius_m": 0.02, "length_m": 0.04},
    ],
)
def test_primitives_use_flange_frame_and_match_visual_geometry(arm, tmp_path, shape):
    spec = profile(shape)
    spec["bodies"][0]["origin"][0][3] = 1
    tool = load_tool(spec, tmp_path, arm, np.eye(4))
    matrix = np.eye(4)
    matrix[1, 3] = 1
    model = RobotCollisionModel(arm, tool.bodies)
    obstacles = {"probe": placed(fcl.Sphere(0.01), matrix)}
    assert model.check([0], obstacles) == []
    assert model.check([np.pi / 2], obstacles)[0]["second"] == "probe"
    visual = tool.metadata["visual_bodies"][0]
    assert visual["origin"] == spec["bodies"][0]["origin"]
    assert all(visual[key] == value for key, value in shape.items())


def test_mesh_hash_change_invalidates_audit_even_with_same_yaml(arm, tmp_path):
    path = tmp_path / "mount.stl"
    trimesh.creation.box(extents=[0.04, 0.03, 0.02]).export(path)
    spec = profile({"type": "mesh", "path": "mount.stl", "scale": [1, 1, 1]})
    before = load_tool(spec, tmp_path, arm, np.eye(4))
    result = {"tool_model": deepcopy(before.metadata)}
    require_matching_tool(before, result)
    trimesh.creation.box(extents=[0.08, 0.03, 0.02]).export(path)
    after = load_tool(spec, tmp_path, arm, np.eye(4))
    assert before.metadata["profile_sha256"] != after.metadata["profile_sha256"]
    with pytest.raises(ValueError, match="Tool geometry changed"):
        require_matching_tool(after, result)
    points = np.array(after.metadata["visual_bodies"][0]["vertices"])
    assert np.allclose(np.ptp(points, axis=0), [0.08, 0.03, 0.02])


def test_tcp_mismatch_is_rejected_and_null_preserves_legacy_audit(arm, tmp_path):
    shifted = np.eye(4)
    shifted[2, 3] = 0.1
    with pytest.raises(ValueError, match="TCP differs"):
        load_tool(profile(), tmp_path, arm, shifted)
    require_matching_tool(load_tool(None, tmp_path, arm, shifted), {})
    with pytest.raises(ValueError, match="Tool geometry changed"):
        require_matching_tool(load_tool(profile(), tmp_path, arm, np.eye(4)), {})
