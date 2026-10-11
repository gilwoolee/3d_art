from time import monotonic
from xml.etree.ElementTree import fromstring

import fcl
import numpy as np
import trimesh

from pen_art.planning.collision import (
    RobotCollisionModel,
    collision_geometry,
    placed,
    work_surfaces,
)
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


def test_robot_mesh_checks_use_fk_and_exclude_adjacent_links(tmp_path):
    model = make_robot(tmp_path)
    matrix = np.eye(4)
    matrix[0, 3] = 1
    obstacle = placed(fcl.Sphere(0.1), matrix)
    assert model.check([0], {"obstacle": obstacle}) == [
        {"first": "flange:0", "second": "obstacle", "kind": "overlap"}
    ]
    assert model.check([np.pi / 2], {"obstacle": obstacle}) == []
    assert model.check([0]) == []


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


def test_clearance_certificate_and_depth_limit_are_distinct(tmp_path):
    model = make_robot(tmp_path)
    matrix = np.eye(4)
    matrix[0, 3] = 1.3
    obstacles = {"obstacle": placed(fcl.Sphere(0.1), matrix)}
    assert model.check_edge([-0.3], [0.3], obstacles, maximum_depth=0)["status"] == (
        "CLEARANCE_UNRESOLVED"
    )
    assert model.check_edge([-0.3], [0.3], obstacles)["status"] == "CERTIFIED_MODELED_EDGE"


def test_fixed_mount_can_touch_table_but_moving_link_cannot(tmp_path):
    path = tmp_path / "mount.urdf"
    path.write_text("""<robot name="mount">
    <link name="world"/><link name="base"><collision><geometry>
      <box size="0.1 0.1 0.1"/></geometry></collision></link>
    <link name="flange"><collision><origin xyz="1 0 0"/>
      <geometry><sphere radius="0.1"/></geometry></collision></link>
    <joint name="mount" type="fixed"><parent link="world"/><child link="base"/></joint>
    <joint name="joint" type="revolute"><parent link="base"/><child link="flange"/>
      <origin xyz="0 0 0.2"/><axis xyz="0 1 0"/>
      <limit lower="-3.14" upper="3.14" velocity="1" effort="1"/></joint></robot>""")
    model = RobotCollisionModel(SerialArm(path))
    obstacles = work_surfaces([0.5, 0], 0, 0.1, 0.1)
    assert not model.check([0], obstacles)
    assert model.check([0.3], obstacles)


def test_mesh_collision_detects_contained_solid(tmp_path):
    mesh = tmp_path / "box.stl"
    trimesh.creation.box().export(mesh)
    geometry, _ = collision_geometry(fromstring(f'<geometry><mesh filename="{mesh}"/></geometry>'))
    outer = placed(geometry, np.eye(4))
    inner = placed(fcl.Sphere(0.1), np.eye(4))
    assert fcl.collide(outer, inner, fcl.CollisionRequest(), fcl.CollisionResult()) > 0


def test_expired_collision_budget_is_not_a_collision_or_a_certificate(tmp_path):
    model = make_robot(tmp_path)
    result = model.check_edge([-0.3], [0.3], deadline=monotonic() - 1)
    assert result["status"] == "TIME_BUDGET"
    assert result["evaluations"] == 0


def test_curved_path_excursion_is_not_replaced_by_its_endpoint_chord(tmp_path):
    from pen_art.planning.scene import MaterialScene
    from pen_art.planning.smooth_audit import CubicPiece

    model = make_robot(tmp_path)
    matrix = np.eye(4)
    angle = 0.13
    matrix[:3, 3] = [np.cos(angle), np.sin(angle), 0]
    obstacles = {"obstacle": placed(fcl.Sphere(0.01), matrix)}
    curve = CubicPiece([[0], [-6.4], [6.4], [-0.8]], 1)
    assert model.check_edge([-0.8], [-0.8], obstacles)["status"] == "CERTIFIED_MODELED_EDGE"
    result = model.check_bounded_path(curve.position, curve.deviation, obstacles)
    assert result["status"] == "COLLISION"
    assert model.check(result["positions"], obstacles)

    # Material broadphase must preserve an interior collision and certified
    # clearance, including when the global material box is far from the arm.
    material = MaterialScene()
    point = matrix[:3, 3]
    material.add(point, point, 0.01, "interior-hit")
    assert (
        model.check_bounded_path(curve.position, curve.deviation, material=material)["status"]
        == "COLLISION"
    )
    far = MaterialScene()
    far.add([5, 0, 0], [5, 1, 0], 0.01, "far")
    culled = model.check_bounded_path(curve.position, curve.deviation, material=far)
    far.possibly_near = lambda *args: True
    far.possibly_near_body = lambda *args: True
    exhaustive = model.check_bounded_path(curve.position, curve.deviation, material=far)
    assert culled == exhaustive
    assert culled["status"] == "CERTIFIED_MODELED_EDGE"
