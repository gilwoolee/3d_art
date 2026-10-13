"""Compare conservative self-collision bounds on a fresh complete replay."""

import inspect
import shutil
import textwrap
from pathlib import Path
from time import monotonic

import pen_art.planning.collision as collision
from pen_art.io import read_json, sha256, write_json
from pen_art.planning.cell_replay import replay_cell

source = Path("runs/staged_checkpoint_partial_v1")
original = collision.RobotCollisionModel.check_bounded_path
code = textwrap.dedent(inspect.getsource(original))
start = code.index("            if name2 in self.motion_radii and name2 not in (obstacles or {}):")
end = code.index("            bound = float(radius @ half_delta)", start)
namespace = {}
exec(compile(code[:start] + code[end:], "<legacy-collision-bound>", "exec"), original.__globals__, namespace)
legacy = namespace[original.__name__]
results = {}
for label, implementation in [("legacy", legacy), ("relative", original)]:
    run = Path(f"runs/staged_{label}_bounds_v1")
    assert not run.exists(), f"Preserve existing evidence: {run}"
    shutil.copytree(source, run)
    (run / "offline_dual_arm_replay_checkpoint.json").unlink()
    counts = {"calls": 0, "evaluations": 0, "seconds": 0.0, "statuses": {}}

    def measured(self, *args, **kwargs):
        start = monotonic()
        result = implementation(self, *args, **kwargs)
        counts["seconds"] += monotonic() - start
        counts["calls"] += 1
        counts["evaluations"] += result["evaluations"]
        statuses = counts["statuses"]
        statuses[result["status"]] = statuses.get(result["status"], 0) + 1
        return result

    collision.RobotCollisionModel.check_bounded_path = measured
    start = monotonic()
    validation = replay_cell(run, Path("research/planning_cell/cell.yaml"), float("inf"), "offline_dual_arm_plan")
    wall = monotonic() - start
    assert validation["status"] == "CERTIFIED_GEOMETRIC_REPLAY", validation
    assert not validation.get("resumed_from_s"), validation
    results[label] = {"run": str(run), "wall_seconds": wall, "bounded_paths": counts,
                      "validation": validation["status"], "checked_intervals": validation["checked_intervals"]}
    print(label, results[label], flush=True)
collision.RobotCollisionModel.check_bounded_path = original
assert results["legacy"]["checked_intervals"] == results["relative"]["checked_intervals"]
write_json(Path("research/blondie/relative_collision_bounds.json"), {
    "status": "PASSED", "simulation_only": True, "fresh_replay_copies": True,
    "description": "Common ancestor rotations cancel only for robot self-pair separation. Static and material bounds remain unchanged.",
    "results": results,
    "inputs": {str(p): sha256(p) for p in [Path("src/pen_art/planning/collision.py"), source / "offline_dual_arm_plan.json", Path(__file__)]},
})
