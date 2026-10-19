"""A tiny one-arm cell shared by the tests."""


from collections import Counter
from time import monotonic
from types import SimpleNamespace

import fcl
import numpy as np

from pen_art.planning.cell_planner import CellPlanner
from pen_art.planning.clock_path import ClockEdge, ClockPath
from pen_art.planning.collision import RobotCollisionModel, placed
from pen_art.planning.dual_arm import MotionProposal, ScheduledMotion, ScheduleState
from pen_art.planning.path_certificates import PathCertificateStore
from pen_art.robot import SerialArm


def setup(tmp_path):
    urdf = tmp_path / "arm.urdf"
    urdf.write_text('''<robot name="one"><link name="base"/><link name="tip">
      <collision><origin xyz="1 0 0"/><geometry><sphere radius="0.02"/></geometry></collision>
      </link><joint name="joint" type="revolute"><parent link="base"/><child link="tip"/>
      <origin xyz="0 0 0"/><axis xyz="0 0 1"/><limit lower="-2" upper="2" velocity="1"/>
      </joint></robot>''')
    model = RobotCollisionModel(SerialArm(urdf, tip="tip"))
    planner = object.__new__(CellPlanner)
    planner.profile = {
        "world_from_build": np.eye(4).tolist(),
        "environment": {"table_size_m": [4, 4, .1], "platform_size_m": [.1, .1],
                        "table_z_m": -1},
    }
    planner.arms = {"arm": SimpleNamespace(
        collision=model, robot=model.robot, tool=SimpleNamespace(metadata={}),
        flange_to_tcp=np.eye(4),
    )}
    planner.tip_models = {"arm": model}
    planner.turntable, planner.actions = None, {}
    planner.settings = {"maximum_subdivision_depth": 12}
    planner.timing, planner.process, planner.transit = {}, {}, {}
    planner.margin, planner.deadline = .001, monotonic() + 20
    planner._path_model_contexts, planner._obstacle_context_refs = {}, {}
    planner.path_certificates = PathCertificateStore(tmp_path / "proofs")
    planner.verification_stats, planner.verification_seconds = Counter(), Counter()
    planner.verification_raw, planner.checked_intervals = False, 0
    transform = np.eye(4)
    transform[0, 3] = 4
    planner.obstacles = {"unknown": placed(fcl.Sphere(.02), transform)}
    path = ClockPath([ClockEdge(0, 1, np.array([-.2]), np.array([.2]), "TRAVEL", "line")])
    move = MotionProposal("line", "arm", "fixed", 1, "ground", (.2,), path)
    return planner, ScheduledMotion(move, 0)


def state(scene):
    return ScheduleState(0, (("arm", (-.2,)),), frozenset(), (), payload=scene)
