import fcl
import numpy as np
import pytest

from pen_art.planning.collision import Body, RobotCollisionModel, placed
from pen_art.planning.scene import MaterialScene
from pen_art.planning.transit import (
    cartesian_lift,
    completed_prefix,
    contact_validator,
    find_departure,
    local_contact_surfaces,
)
from pen_art.robot import SerialArm


@pytest.fixture
def planar_arm(tmp_path):
    joints = []
    links = ["base", "one", "two", "flange"]
    for i in range(3):
        joints.append(f'''<joint name="j{i}" type="revolute">
          <parent link="{links[i]}"/><child link="{links[i + 1]}"/>
          <origin xyz="{0 if i == 0 else 0.3} 0 0"/><axis xyz="0 1 0"/>
          <limit lower="-2.5" upper="2.5" velocity="1" effort="1"/></joint>''')
    path = tmp_path / "planar.urdf"
    path.write_text(
        '<robot name="planar">'
        + "".join(f'<link name="{n}"/>' for n in links)
        + "".join(joints)
        + "</robot>"
    )
    return SerialArm(path)


def test_lift_keeps_contact_configuration_and_fixed_orientation(planar_arm):
    q = np.array([0.3, -0.6, 0.3])
    offset = np.eye(4)
    offset[0, 3] = 0.1
    settings = {
        "lift_sample_m": 0.003,
        "lift_position_tolerance_m": 0.0003,
        "lift_orientation_tolerance_rad": 0.003,
    }
    edges = []

    def valid(a, b):
        edges.append((a, b))
        return True

    result = cartesian_lift(planar_arm, offset, q, 0.01, settings, valid)
    assert result["status"] == "SAMPLED_DRAWING_PATH"
    np.testing.assert_array_equal(result["positions"][0], q)
    start = planar_arm.forward(q, offset)
    end = planar_arm.forward(result["positions"][-1], offset)
    np.testing.assert_allclose(end[:3, :3], start[:3, :3], atol=1e-5)
    np.testing.assert_allclose(end[:3, 3] - start[:3, 3], [0, 0, 0.01], atol=1e-5)
    assert edges
    failure = cartesian_lift(planar_arm, offset, q, 0.01, settings, lambda a, b: False)
    assert failure["status"] == "NO_CONNECTED_IK_PATH"


def test_new_material_blocks_a_previously_clear_joint_edge(planar_arm):
    offset = np.eye(4)
    offset[0, 3] = 0.1
    model = RobotCollisionModel(
        planar_arm, [Body("probe", "flange", offset, fcl.Sphere(0.01), 0.01)]
    )
    a, b = np.array([-0.1, 0, 0]), np.array([0.1, 0, 0])
    material = MaterialScene()
    assert model.check_edge(a, b, material=material)["status"] == "CERTIFIED_MODELED_EDGE"
    point = planar_arm.forward([0, 0, 0], offset)[:3, 3]
    material.add(point, point + [0, 0.05, 0], 0.005, "new-strand")
    assert not model.check(a, material=material) and not model.check(b, material=material)
    result = model.check_edge(a, b, material=material)
    assert result["status"] == "COLLISION"
    assert result["pairs"] == [{"first": "probe", "second": "new-strand"}]


def test_lift_contact_is_local_and_does_not_ignore_a_strand_above_it(planar_arm):
    offset = np.eye(4)
    offset[0, 3] = 0.1
    q = np.array([0.3, -0.6, 0.3])
    contact = planar_arm.forward(q, offset)[:3, 3]
    wrist = Body("wrist", "flange", np.eye(4), fcl.Sphere(0.001), 0.001)
    tip = Body("virtual_tip", "flange", offset, fcl.Sphere(0.0005), 0.0005)
    arm, full = (
        RobotCollisionModel(planar_arm, [wrist]),
        RobotCollisionModel(planar_arm, [wrist, tip]),
    )
    scene = MaterialScene()
    scene.add(contact, contact + [0, 0.02, 0], 0.0006, "anchor")
    assert full.check(
        q, material=scene
    )  # Ordinary collision checks reject intentional tip contact.
    settings = {
        "lift_sample_m": 0.003,
        "lift_position_tolerance_m": 0.0003,
        "lift_orientation_tolerance_rad": 0.003,
    }
    valid = contact_validator(
        planar_arm, arm, full, offset, scene, {}, contact, 0.004, 0.0005, 0.001, 10
    )
    assert cartesian_lift(planar_arm, offset, q, 0.012, settings, valid)["status"] == (
        "SAMPLED_DRAWING_PATH"
    )
    point = contact + [0, 0, 0.008]
    scene.add(point + [0, -0.01, 0], point + [0, 0.01, 0], 0.0006, "overhead-strand")
    blocked = contact_validator(
        planar_arm, arm, full, offset, scene, {}, contact, 0.004, 0.0005, 0.001, 10
    )
    assert cartesian_lift(planar_arm, offset, q, 0.012, settings, blocked)["status"] == (
        "NO_CONNECTED_IK_PATH"
    )
    # An oblique exit can pass around the same overhead strand. The contact
    # exemption stays local and the obstructed vertical attempt is retained.
    settings.update(withdraw_distance_m=0.02, departure_directions=[[0, 0, 1], [1, 0, 1]])
    result = find_departure(planar_arm, offset, q, settings, blocked)
    assert result["status"] == "SAMPLED_DRAWING_PATH"
    assert result["attempts"][0]["status"] == "NO_CONNECTED_IK_PATH"
    evidence = result["attempts"][0]["collision_evidence"]
    assert any(
        pair["second"].startswith("overhead-strand") for row in evidence for pair in row["pairs"]
    )
    np.testing.assert_allclose(result["direction"], [2**-0.5, 0, 2**-0.5])
    end = planar_arm.forward(result["positions"][-1], offset)[:3, 3]
    np.testing.assert_allclose(end - contact, np.array(result["direction"]) * 0.02, atol=1e-5)


def test_near_platform_contact_allowance_covers_only_the_local_tip(planar_arm):
    offset = np.eye(4)
    offset[0, 3] = 0.1
    q = np.array([0.3, -0.6, 0.3])
    contact = planar_arm.forward(q, offset)[:3, 3]
    platform_z = contact[2] - 0.0012
    platform_pose = np.eye(4)
    platform_pose[:3, 3] = contact - [0, 0, 0.0112]
    obstacles = {"platform": placed(fcl.Box(0.1, 0.1, 0.02), platform_pose)}
    wrist = Body("wrist", "flange", np.eye(4), fcl.Sphere(0.001), 0.001)
    tip = Body("virtual_tip", "flange", offset, fcl.Sphere(0.0005), 0.0005)
    arm = RobotCollisionModel(planar_arm, [wrist])
    full = RobotCollisionModel(planar_arm, [wrist, tip])
    surfaces = local_contact_surfaces(contact[2], platform_z - 0.2, platform_z, 0.004)
    assert surfaces == ["platform"]
    assert full.check(q, obstacles, 0.001)
    valid = contact_validator(
        planar_arm,
        arm,
        full,
        offset,
        MaterialScene(),
        obstacles,
        contact,
        0.004,
        0.0005,
        0.001,
        10,
        surfaces,
    )
    settings = {
        "lift_sample_m": 0.003,
        "lift_position_tolerance_m": 0.0003,
        "lift_orientation_tolerance_rad": 0.003,
    }
    assert cartesian_lift(planar_arm, offset, q, 0.02, settings, valid)["status"] == (
        "SAMPLED_DRAWING_PATH"
    )
    assert local_contact_surfaces(platform_z + 0.01, platform_z - 0.2, platform_z, 0.004) == []


def test_reordered_resume_retains_complete_unique_cycles():
    def cycle(identity):
        return [
            {"kind": kind, "stroke_id": identity, "positions": [[0], [0]]}
            for kind in ["TRAVEL", "APPROACH", "DRAW", "WITHDRAW"]
        ]

    operations = cycle("b") + cycle("a")
    prefix, count = completed_prefix(operations, ["a", "b"], allow_reordered=True)
    assert count == 2 and prefix == operations
    with pytest.raises(ValueError, match="order"):
        completed_prefix(operations, ["a", "b"])
    with pytest.raises(ValueError, match="order"):
        completed_prefix(cycle("b") + cycle("b"), ["a", "b"], allow_reordered=True)
    with pytest.raises(ValueError, match="Malformed"):
        completed_prefix(operations, ["a"])
