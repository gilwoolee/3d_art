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

    # The pen as spheres: they contain every tool body, never report more
    # clearance than the exact shapes have, and see only the wire they are given.
    from types import SimpleNamespace

    from pen_art.planning.pen_spheres import PenSpheres, wire_arrays
    from pen_art.planning.tool import load_tool

    def origin(z):
        matrix = np.eye(4)
        matrix[2, 3] = z
        return matrix.tolist()

    profile = {
        "schema_version": 1, "units": "m", "flange_to_tcp": origin(0.225), "tip_radius_m": 0.0005,
        "bodies": [
            {"name": "adapter", "origin": origin(0.0075),
             "shape": {"type": "box", "size_m": [0.04, 0.04, 0.015]}},
            {"name": "body", "origin": origin(0.105),
             "shape": {"type": "cylinder", "radius_m": 0.012, "length_m": 0.18}},
            {"name": "nose", "origin": origin(0.2085),
             "shape": {"type": "frustum", "bottom_radius_m": 0.012, "top_radius_m": 0.0008,
                       "length_m": 0.027}},
        ],
    }
    links = SimpleNamespace(root="base", tip="flange", joint_data=[{"child": "flange"}])
    tool = load_tool(profile, tmp_path, links, np.asarray(profile["flange_to_tcp"]))
    spheres = PenSpheres(tool.metadata)
    assert 40 < len(spheres.radii) < 100
    rng, exceeded, proud = np.random.default_rng(0), 0, []
    for _ in range(400):
        start = np.r_[rng.uniform(-0.04, 0.04, 2), rng.uniform(-0.01, 0.24)]
        end = start + rng.normal(0, 0.004, 3)
        axis = (end - start) / np.linalg.norm(end - start)
        side = np.cross([0, 1, 0] if abs(axis[1]) < 0.9 else [1, 0, 0], axis)
        side /= np.linalg.norm(side)
        pose = np.eye(4)
        pose[:3, :3] = np.column_stack([side, np.cross(axis, side), axis])
        pose[:3, 3] = (start + end) / 2
        wire = placed(fcl.Capsule(0.00075, float(np.linalg.norm(end - start))), pose)
        exact = min(
            fcl.distance(
                placed(body.geometry, body.origin), wire, fcl.DistanceRequest(),
                fcl.DistanceResult(),
            )
            for body in tool.bodies
        )
        wires = (start[None], end[None], np.array([0.00075]))
        found = spheres.violation(np.eye(4), wires, 1.0)["distance_m"]
        if exact > 0:
            exceeded += found > exact + 1e-6
            proud.append(exact - found)
            assert (spheres.violation(np.eye(4), wires, 0.001) is None) == (found >= 0.001 + 1e-9)
    assert exceeded == 0 and np.median(proud) < 0.0003
    assert spheres.violation(np.eye(4), tuple(values[:0] for values in wires), 1.0) is None
    # A body may keep its own clearance; the others keep the shared margin.
    hit = spheres.violation(np.eye(4), wires, 1.0)
    own = {None: 1.0, hit["body"]: hit["distance_m"] - 1e-6}
    other = spheres.violation(np.eye(4), wires, own)
    assert other is None or (other["body"] != hit["body"] and other["margin_m"] == 1.0)
    from pen_art.planning.scene import MaterialScene

    scene = MaterialScene()
    assert len(wire_arrays(scene)[2]) == 0
    scene.add([0, 0, 0], [0.01, 0, 0], 0.00075, "first")
    assert len(wire_arrays(scene)[2]) == 1
    scene.add([0, 0, 0.3], [0.01, 0, 0.3], 0.00075, "second")
    assert np.allclose(wire_arrays(scene)[0], [[0, 0, 0], [0, 0, 0.3]])
