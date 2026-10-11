from types import SimpleNamespace
from xml.etree.ElementTree import Element

import numpy as np

from pen_art.io import read_json, write_json
from pen_art.planning import cell_replay
from pen_art.planning.clock_path import ClockEdge, ClockPath
from pen_art.planning.dual_arm import StrokeTask
from pen_art.planning.replay_checkpoint import read_checkpoint


def test_replay_resumes_active_motion_and_exact_material_and_rejects_changed_plan(
    tmp_path, monkeypatch
):
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("unchanged robot")
    robot = SimpleNamespace(
        frames=lambda _q: {},
        velocity=np.ones(1),
        lower=-np.ones(1),
        upper=np.ones(1),
        xml=Element("robot"),
    )
    arm = SimpleNamespace(robot=robot, urdf=urdf, tool=SimpleNamespace(metadata={}))

    def task(name):
        return StrokeTask(name, frozenset(["ground"]), frozenset([name]))

    seen = []
    interrupt = True

    class Planner:
        def __init__(self, *_args, **_kwargs):
            self.profile = {"same": "profile"}
            self.arms = {"left": arm, "right": arm}
            self.tasks = [task("a"), task("b")]
            self.ground = ["ground"]
            self.checked_intervals = 0
            self.deadline = float("inf")

        def validate_interval(self, state, new, end):
            seen.append((state.time_s, {m.proposal.task for m in state.active + new}))
            if interrupt and state.time_s == 1:
                return None, {"status": "TIME_BUDGET"}
            if state.time_s == 1:
                assert state.payload.segments == [
                    {
                        "name": "interval-0-1.0",
                        "start": [0.0, 0.0, 0.0],
                        "end": [1.0, 0.0, 0.0],
                        "radius_m": 0.002,
                    }
                ]
            scene = state.payload.copy()
            scene.add([state.time_s, 0, 0], [end, 0, 0], 0.002, f"interval-{state.time_s}-{end}")
            self.checked_intervals += 1
            return scene, None

    monkeypatch.setattr(cell_replay, "CellPlanner", Planner)
    monkeypatch.setattr(cell_replay, "tcp_tangent_bound", lambda *_args: 0)
    settings = {
        "search": {},
        "transit": {},
        "timing": {"velocity_fraction": 1, "joint_acceleration_rad_s2": [1]},
        "process": {"nominal_speed_m_s": 1},
    }
    write_json(tmp_path / "dual_arm_config.json", settings)
    actions = []
    for name, identity, start in (("a", "left", 0.0), ("b", "right", 1.0)):
        path = ClockPath([ClockEdge(0, 2, np.zeros(1), np.zeros(1), "DRAW", name)])
        actions.append(
            {
                "stroke_id": name,
                "arm": identity,
                "variant": "fixed",
                "start_anchor": "ground",
                "start_s": start,
                "end_s": start + 2,
                "trajectory": path.record(),
            }
        )
    plan = {
        "inputs": {},
        "initial_positions": {"left": [0.0], "right": [0.0]},
        "modeled_duration_s": 3.0,
        "actions": actions,
        "events": [],
        "limitations": [],
        "status": "PREFIX",
    }
    plan_path = tmp_path / "dual_arm_plan.json"
    write_json(plan_path, plan)
    first = cell_replay._replay_cell(tmp_path, "unused")
    assert first["status"] == "REPLAY_TIME_BUDGET"
    assert first["completed_strokes"] == 0

    interrupt = False
    seen.clear()
    resumed, state = cell_replay._replay_cell(tmp_path, "unused", return_state=True)
    assert seen == [(1.0, {"a", "b"}), (2.0, {"b"})]
    assert resumed["resumed_from_s"] == 1
    assert resumed["reused_path_checks"] == 2
    assert resumed["status"] == "CERTIFIED_PREFIX_REPLAY"
    assert resumed["completed_strokes"] == 2
    assert resumed["checked_intervals"] == 3
    assert len(state.payload.segments) == 3

    # Any change to the full plan invalidates both prefix and path reuse.
    plan["limitations"] = ["changed plan"]
    write_json(plan_path, plan)
    seen.clear()
    fresh, fresh_state = cell_replay._replay_cell(tmp_path, "unused", return_state=True)
    assert seen == [(0, {"a"}), (1.0, {"a", "b"}), (2.0, {"b"})]
    assert "resumed_from_s" not in fresh
    assert fresh["reused_path_checks"] == 0
    assert fresh_state.payload.segments == state.payload.segments

    checkpoint_path = tmp_path / "dual_arm_replay_checkpoint.json"
    corrupted = read_json(checkpoint_path)
    context = corrupted["context"]
    corrupted["material_segments"][0]["radius_m"] = 0.001
    write_json(checkpoint_path, corrupted)
    assert read_checkpoint(checkpoint_path, context) is None
