"""Small cuRobo GPU installation check using its bundled Franka model, not Flexiv."""

import json
import subprocess
from pathlib import Path
from time import perf_counter

import curobo
import torch
from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
from curobo.types import GoalToolPose, Pose

count = 16
started = perf_counter()
solver = InverseKinematics(
    InverseKinematicsCfg.create(
        robot="franka.yml",
        scene_model="collision_table.yml",
        num_seeds=32,
        max_batch_size=count,
        self_collision_check=True,
        position_tolerance=0.0003,
        use_cuda_graph=True,
    )
)
positions = torch.zeros(count, 3, device="cuda", dtype=torch.float32)
positions[:, 0] = torch.linspace(0.35, 0.5, count, device="cuda")
positions[:, 2] = 0.4
quaternions = torch.zeros(count, 4, device="cuda", dtype=torch.float32)
quaternions[:, 0] = 1
goals = GoalToolPose.from_poses(
    {solver.tool_frames[0]: Pose(position=positions, quaternion=quaternions)}, num_goalset=1
)
solver.solve_pose(goals)
torch.cuda.synchronize()
setup_seconds = perf_counter() - started
started = perf_counter()
result = solver.solve_pose(goals)
torch.cuda.synchronize()
solve_seconds = perf_counter() - started
report = dict(
    curobo_version=curobo.__version__,
    curobo_revision=subprocess.check_output(
        ["git", "-C", "external/curobo", "rev-parse", "HEAD"], text=True
    ).strip(),
    torch_version=torch.__version__,
    gpu=torch.cuda.get_device_name(0),
    model="bundled franka.yml (not the Flexiv/pen)",
    targets=count,
    successes=int(result.success.sum().item()),
    maximum_position_error_m=float(result.position_error.max().item()),
    setup_and_first_solve_seconds=setup_seconds,
    warm_batch_seconds=solve_seconds,
    self_collision_check=True,
    scene="collision_table.yml",
    limits="Installation check only; no extrusion, Flexiv motion, or speedup claim.",
)
output = Path("research/blondie/curobo_smoke.json")
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps(report, indent=2), flush=True)
assert report["successes"] == count and report["maximum_position_error_m"] <= 0.0003
