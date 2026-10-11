import numpy as np

from pen_art.cell import WorldArm
from pen_art.planning.collision import RobotCollisionModel
from pen_art.planning.coordinated_motion import check_pair_linear, check_pair_paths
from pen_art.planning.smooth_audit import CubicPiece
from pen_art.robot import SerialArm


def arms(tmp_path, shift=0, joint_limit=3):
    path = tmp_path / "arm.urdf"
    path.write_text(f"""<robot name="test"><link name="base"/>
      <link name="flange"><collision><origin xyz="1 0 0"/>
      <geometry><sphere radius="0.1"/></geometry></collision></link>
      <joint name="q" type="revolute"><parent link="base"/><child link="flange"/>
      <axis xyz="0 0 1"/>
      <limit lower="-{joint_limit}" upper="{joint_limit}" velocity="1" effort="1"/></joint>
      </robot>""")
    robot = SerialArm(path)
    transform = np.eye(4)
    transform[0, 3] = shift
    return RobotCollisionModel(robot), RobotCollisionModel(WorldArm(robot, transform))


def test_uncertain_clearance_is_not_a_collision_or_pass(tmp_path):
    first, second = arms(tmp_path, 0.23)
    result = check_pair_linear(first, second, [-0.1], [0.1], [0], [0], maximum_depth=0)
    assert result["status"] == "CLEARANCE_UNRESOLVED"
    assert (
        check_pair_linear(first, second, [-0.1], [0.1], [0], [0])["status"]
        == "CERTIFIED_INTER_ARM_INTERVAL"
    )


def test_cubic_excursion_hits_waiting_arm_despite_safe_endpoint_chord(tmp_path):
    first, second = arms(tmp_path)
    curve = CubicPiece([[0], [-6.4], [6.4], [-0.8]], 1)
    assert (
        check_pair_linear(first, second, [-0.8], [-0.8], [0.13], [0.13])["status"]
        == "CERTIFIED_INTER_ARM_INTERVAL"
    )
    result = check_pair_paths(
        first,
        second,
        curve.position,
        lambda _: np.array([0.13]),
        curve.deviation,
        lambda a, b: np.zeros(1),
    )
    assert result["status"] == "COLLISION"


def test_event_search_rejects_colliding_overlap_and_uses_full_motion_alternative(tmp_path):
    from pen_art.planning.dual_arm import MotionProposal, StrokeTask, search_schedule

    first, second = arms(tmp_path, joint_limit=6)
    models = (first, second)
    tasks = [StrokeTask(name, frozenset(["ground"]), frozenset([name])) for name in ("a", "b")]

    def prepare(state, task, arm):
        if (task.identity, arm) == ("a", "left"):
            yield MotionProposal("a", arm, "short", 2, "ground", (0.8,), (-0.8, 0.8))
        if (task.identity, arm) == ("b", "right"):
            yield MotionProposal("b", arm, "colliding", 2, "ground", (-0.8,), (0.8, -0.8))
            yield MotionProposal(
                "b", arm, "around", 2, "ground", (2 * np.pi - 0.8,), (0.8, 2 * np.pi - 0.8)
            )

    def validate(state, new, end):
        motions = {m.proposal.arm: m for m in state.active + new}

        def position(arm, time):
            if arm not in motions:
                return dict(state.configurations)[arm]
            motion = motions[arm]
            a, b = motion.proposal.payload
            u = (time - motion.start_s) / motion.proposal.duration_s
            return [a * (1 - u) + b * u]

        result = check_pair_linear(
            *models,
            position("left", state.time_s),
            position("left", end),
            position("right", state.time_s),
            position("right", end),
        )
        return None, None if result["status"] == "CERTIFIED_INTER_ARM_INTERVAL" else result

    result = search_schedule(
        tasks,
        {"left": (-0.8,), "right": (0.8,)},
        ["ground"],
        prepare,
        validate,
        finish=lambda state: (state, None),
        start_offsets_s=(),
    )
    assert result["status"] == "FOUND"
    assert result["state"].time_s == 2
    assert any(m.proposal.variant == "around" for m in result["state"].history)
    assert any(f["status"] == "COLLISION" for f in result["failures"])
