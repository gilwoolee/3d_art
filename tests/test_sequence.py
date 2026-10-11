from pen_art.planning.sequence import search_order


def test_order_search_retains_robot_configuration_in_state():
    # The same built set is reached through different orders. Only one robot
    # configuration admits the final return. A material-only cache is invalid.
    def transition(state, action, attempt):
        return {"built": state["built"] | {action}, "q": action}, None

    result = search_order(
        ["a", "b"],
        {"built": frozenset(), "q": None},
        lambda state, action: True,
        transition,
        lambda state: (
            (state, None) if state["q"] == "a" else (None, {"phase": "RETURN", "status": "NO_PATH"})
        ),
    )
    assert result["status"] == "FOUND"
    assert result["order"] == ["b", "a"]


def test_actual_material_collision_causes_order_repair(tmp_path):
    import numpy as np

    from pen_art.planning.collision import RobotCollisionModel
    from pen_art.planning.scene import MaterialScene
    from pen_art.robot import SerialArm

    path = tmp_path / "probe.urdf"
    path.write_text("""<robot name="probe"><link name="base"/>
      <link name="flange"><collision><origin xyz="1 0 0"/>
        <geometry><sphere radius="0.01"/></geometry></collision></link>
      <joint name="j" type="revolute"><parent link="base"/><child link="flange"/>
        <axis xyz="0 0 1"/><limit lower="-2" upper="2" velocity="1" effort="1"/>
      </joint></robot>""")
    model = RobotCollisionModel(SerialArm(path))

    # A scripted arm crossing is obstructed after inserting a brace. Installing
    # the brace last preserves the arm path; both scenes use real FCL capsules.
    def transition(state, action, attempt):
        if action == "crossing":
            result = model.check_edge([-0.3], [0.3], material=state, margin_m=0.001)
            if result["status"] != "CERTIFIED_MODELED_EDGE":
                return None, result
        next_scene = state.copy()
        if action == "brace":
            next_scene.add([1, -0.1, 0], [1, 0.1, 0], 0.01, "brace")
        return next_scene, None

    initial = MaterialScene()
    result = search_order(
        ["brace", "crossing"], initial, lambda *args: True, transition, lambda state: (state, None)
    )
    assert result["status"] == "FOUND"
    assert result["order"] == ["crossing", "brace"]
    assert result["failures"][0]["status"] == "COLLISION"
    assert result["failures"][0]["pairs"] == [{"first": "flange:0", "second": "brace"}]
    assert initial.segments == []
    assert np.isfinite(result["state"].segments[0]["radius_m"])
