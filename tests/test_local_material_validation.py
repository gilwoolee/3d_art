"""Reduced proposal geometry never grants a full-robot or future-wire proof."""


from types import SimpleNamespace

import fcl
import numpy as np
from test_invariant_verification import setup, state

from pen_art.planning.clock_path import ClockEdge, ClockPath
from pen_art.planning.collision import Body, RobotCollisionModel
from pen_art.planning.dual_arm import MotionProposal, ScheduledMotion
from pen_art.planning.scene import MaterialScene


def pen_planner(tmp_path):
    planner, move = setup(tmp_path)
    arm = planner.arms["arm"]
    shaft, nose, tcp = np.eye(4), np.eye(4), np.eye(4)
    shaft[:3, 3], nose[:3, 3], tcp[:3, 3] = [2, 0, 0.2], [2, 0, 0.008], [2, 0, 0]
    bodies = [
        Body(
            "pen_body",
            arm.robot.tip,
            shaft,
            fcl.Box(0.04, 0.04, 0.3),
            float(np.linalg.norm([0.04, 0.04, 0.3]) / 2),
            (),
        ),
        Body("pen_nose", arm.robot.tip, nose, fcl.Sphere(0.003), 0.003, ()),
    ]
    virtual = Body("virtual_tip", arm.robot.tip, tcp, fcl.Sphere(0.001), 0.001, ())
    arm.tool = SimpleNamespace(metadata={"estimated": True}, bodies=bodies, tip_radius_m=0.001)
    arm.flange_to_tcp = tcp
    arm.collision = RobotCollisionModel(arm.robot, bodies)
    planner.tip_models["arm"] = RobotCollisionModel(arm.robot, [*bodies, virtual])
    planner.pen_models = {"arm": RobotCollisionModel(arm.robot, bodies, include_robot=False)}
    planner.pen_tip_models = {
        "arm": RobotCollisionModel(arm.robot, [*bodies, virtual], include_robot=False)
    }
    planner.settings["collision_scope"] = "pen_candidate"
    planner.transit.update(contact_radius_m=0.03, virtual_tip_radius_m=0.001)
    return planner, move


def test_pen_candidate_reuses_only_pen_proof_and_full_gate_finds_robot_collision(tmp_path):
    planner, move = pen_planner(tmp_path)
    scene = MaterialScene()
    scene.add([1, 0, -0.02], [1, 0, 0.02], 0.01, "old-bottom-layer")
    # The robot crosses old wire. The full pen remains well outside it.
    assert planner.validate_interval(state(scene), (move,), 1)[1] is None
    context = planner.path_certificate_context("arm")
    store = planner.path_certificates
    proof = store.get(context, move.proposal.payload.record(), "pen_invariant_path")
    assert proof["status"] == "CERTIFIED_PEN_MODELED_PATH"
    assert store.get(context, move.proposal.payload.record(), "invariant_path") is None
    assert planner.validate_interval(state(scene), (move,), 1)[1] is None
    assert planner.verification_stats["pen_static_intervals_reused"] > 0
    added = scene.copy()
    added.add([2, 0, 0.2], [2, 0, 0.2], 0.005, "new-pen-obstruction")
    assert planner.validate_interval(state(added), (move,), 1)[1]["status"] == "COLLISION"
    assert planner.verification_stats["pen_invariant_path_hits"] >= 2
    planner.settings["collision_scope"] = "full_robot"
    assert planner.path_certificate_context("arm") != context
    _, failure = planner.validate_interval(state(scene), (move,), 1)
    assert failure["status"] == "COLLISION"
    assert planner.verification_stats["invariant_path_hits"] == 0
    # A raw verifier cannot be reduced through a saved candidate setting.
    planner.settings["collision_scope"] = "pen_candidate"
    planner.verification_raw = True
    assert planner.collision_scope == "full_robot"
    assert planner.validate_interval(state(scene), (move,), 1)[1]["status"] == "COLLISION"


def test_raw_full_robot_reuses_only_contact_geometry_and_still_checks_old_wire(tmp_path):
    planner, _ = pen_planner(tmp_path)
    planner.verification_raw = True
    planner.optimize_contact_geometry = True
    planner.profile["environment"] = {"table_z_m": -1}
    planner.profile["world_from_build"] = np.eye(4).tolist()
    scene = MaterialScene()
    scene.add([2, 0, 0], [2, 0, 0], 0.0005, "intentional tip join")
    clock = ClockPath([
        ClockEdge(i, i + 1, np.zeros(1), np.zeros(1), "JOIN", "line", contact=np.array([2, 0, 0]))
        for i in range(2)
    ])
    motion = ScheduledMotion(MotionProposal("line", "arm", "fixed", 2, "ground", (0,), clock), 0)
    assert planner.validate_interval(state(scene), (motion,), 2)[1] is None
    assert planner.verification_stats["contact_geometry_constructed"] == 1
    assert planner.verification_stats["contact_geometry_reused"] == 1
    assert not planner.path_certificates.stats
    scene.add([1, 0, 0], [1, 0, 0], 0.01, "old robot obstruction")
    failure = planner.validate_interval(state(scene), (motion,), 2)[1]
    assert failure["status"] == "CONTACT_PATH_REJECTED"
    assert failure["evidence"][0]["scope"] == "arm"
