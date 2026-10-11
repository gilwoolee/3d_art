import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from pen_art.cell import WorldArm, load_cell
from pen_art.io import sha256, write_json
from pen_art.kinematics import geometric_jacobian
from pen_art.planning.cell_motion import pen_rotation
from pen_art.robot import SerialArm


def robot_files(tmp_path):
    path = tmp_path / "robot.urdf"
    links = ["base", *(f"link{i}" for i in range(1, 7)), "flange"]
    joint_names = [f"q{i}" for i in range(7)]
    text = '<robot name="seven">' + "".join(f'<link name="{name}"/>' for name in links[:-1])
    text += (
        '<link name="flange"><collision><geometry><sphere radius="0.01"/>'
        "</geometry></collision></link>"
    )
    for i, name in enumerate(joint_names):
        text += f'''<joint name="{name}" type="revolute"><parent link="{links[i]}"/>
        <child link="{links[i + 1]}"/><origin xyz="0.1 0 0"/><axis xyz="0 0 1"/>
        <limit lower="-3" upper="3" velocity="1" effort="1"/></joint>'''
    path.write_text(text + "</robot>")
    manifest = tmp_path / "robot.json"
    write_json(
        manifest,
        {"joint_names": joint_names, "urdf_sha256": sha256(path), "initial_positions": [0] * 7},
    )
    visual = tmp_path / "robot.glb"
    visual.write_bytes(b"not-rendered-in-loader-test")
    return path, manifest, visual


def test_world_transform_moves_fk_and_jacobian_consistently(tmp_path):
    path, _, _ = robot_files(tmp_path)
    robot = SerialArm(path)
    transform = np.array([[0, -1, 0, 1], [1, 0, 0, -0.3], [0, 0, 1, 0.2], [0, 0, 0, 1]])
    world = WorldArm(robot, transform)
    q = np.zeros(7)
    offset = np.eye(4)
    offset[2, 3] = 0.16
    np.testing.assert_allclose(world.forward(q, offset), transform @ robot.forward(q, offset))
    local_j = geometric_jacobian(robot, q, offset)
    world_j = geometric_jacobian(world, q, offset)
    np.testing.assert_allclose(world_j[:3], transform[:3, :3] @ local_j[:3], atol=1e-12)
    np.testing.assert_allclose(world_j[3:], transform[:3, :3] @ local_j[3:], atol=1e-12)


def profile(tmp_path):
    urdf, manifest, visual = robot_files(tmp_path)
    data = {
        "schema_version": 1,
        "units": "m",
        "simulation_only": True,
        "world_from_build": np.eye(4).tolist(),
        "environment": {
            "table_z_m": 0,
            "table_size_m": [4, 4, 0.1],
            "platform_size_m": [0.19, 0.19],
            "clearance_m": 0.001,
        },
        "arms": [],
    }
    for i in range(2):
        transform = np.eye(4)
        transform[0, 3] = i * 1.2
        data["arms"].append(
            {
                "id": f"arm_{i + 1}",
                "robot_urdf": str(urdf),
                "robot_manifest": str(manifest),
                "robot_visual": str(visual),
                "world_from_base": transform.tolist(),
                "flange_to_tcp": np.eye(4).tolist(),
            }
        )
    path = tmp_path / "cell.yaml"
    path.write_text(yaml.safe_dump(data))
    return data, path


def test_cell_contains_two_distinct_world_bases(tmp_path):
    _, path = profile(tmp_path)
    _, arms, surfaces = load_cell(path)
    assert list(arms) == ["arm_1", "arm_2"]
    assert len(surfaces) == 2
    assert not np.allclose(arms["arm_1"].robot.world_from_base, arms["arm_2"].robot.world_from_base)


def test_tilted_base_keeps_drawing_axis_vertical(tmp_path):
    _, path = profile(tmp_path)
    _, arms, _ = load_cell(path)
    arm = arms["arm_1"]
    for angle in (45, 90):
        arm.robot.world_from_base[:3, :3] = Rotation.from_euler(
            "y", angle, degrees=True
        ).as_matrix()
        rotation = pen_rotation(arm, np.eye(4))
        np.testing.assert_allclose(rotation[:, 2], [0, 0, -1], atol=1e-12)
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
