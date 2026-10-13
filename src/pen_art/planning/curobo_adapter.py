"""Optional GPU IK proposals for the exact Flexiv chain and mounted pen TCP.

cuRobo proposes joint states; the existing full mesh/tool validator remains the
authority for collision, tracking, and growing material. No sphere approximation
is admitted as evidence of collision clearance.
"""

from time import perf_counter

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


class CuroboAdapter:
    def __init__(self, arm, batch_size=64, seeds=32):
        import torch
        from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg

        self.arm, self.torch = arm, torch
        self.batch_size, self.seeds = batch_size, seeds
        joints = arm.robot.names
        robot = {
            "robot_cfg": {
                "kinematics": {
                    "format_version": 2.0,
                    "urdf_path": str(arm.urdf.resolve()),
                    "asset_root_path": str(arm.urdf.parent.resolve()),
                    "base_link": arm.robot.root,
                    "tool_frames": [arm.robot.tip],
                    "collision_link_names": [],
                    "cspace": {
                        "joint_names": joints,
                        "default_joint_position": arm.initial_positions.tolist(),
                        "max_acceleration": 0.5,
                        "max_jerk": 500.0,
                        "cspace_distance_weight": [1.0] * len(joints),
                        "null_space_weight": [1.0] * len(joints),
                    },
                }
            }
        }
        started = perf_counter()
        self.solver = InverseKinematics(
            InverseKinematicsCfg.create(
                robot=robot,
                num_seeds=seeds,
                max_batch_size=batch_size,
                self_collision_check=False,
                load_collision_spheres=False,
                position_tolerance=0.00005,
                orientation_tolerance=0.001,
                use_cuda_graph=True,
            )
        )
        self.setup_seconds = perf_counter() - started
        self.batches = []

    def solve(self, world_tcp_targets, reference=None, alternatives=8):
        from curobo.types import GoalToolPose, Pose

        torch = self.torch
        targets = np.asarray(world_tcp_targets)
        flange = (
            np.linalg.inv(self.arm.robot.world_from_base)[None]
            @ targets
            @ np.linalg.inv(self.arm.flange_to_tcp)[None]
        )
        solutions = []
        for start in range(0, len(flange), self.batch_size):
            transforms = flange[start : start + self.batch_size]
            quaternions = Rotation.from_matrix(transforms[:, :3, :3]).as_quat()[:, [3, 0, 1, 2]]
            pose = Pose(
                position=torch.tensor(transforms[:, :3, 3], device="cuda", dtype=torch.float32),
                quaternion=torch.tensor(quaternions, device="cuda", dtype=torch.float32),
            )
            seed_config = None
            if reference is not None:
                seed_config = torch.tensor(reference, device="cuda", dtype=torch.float32)
                # Supplying every seed as the same posture collapses cuRobo's
                # nominal alternatives to one IK branch. Keep one continuity
                # seed and let the solver generate the remaining bounded seeds.
                seed_config = seed_config.reshape(1, 1, -1).repeat(len(transforms), 1, 1)
            began = perf_counter()
            result = self.solver.solve_pose(
                GoalToolPose.from_poses({self.arm.robot.tip: pose}, num_goalset=1),
                seed_config=seed_config,
                return_seeds=alternatives,
            )
            torch.cuda.synchronize()
            q = result.solution.detach().cpu().numpy()
            success = result.success.detach().cpu().numpy()
            accepted = 0
            for index, row in enumerate(q):
                good = []
                for candidate, ok in zip(row, success[index], strict=True):
                    if not ok:
                        continue
                    actual = self.arm.robot.forward(candidate, self.arm.flange_to_tcp)
                    goal = targets[start + index]
                    if (
                        np.linalg.norm(actual[:3, 3] - goal[:3, 3]) <= 0.00005
                        and Rotation.from_matrix(actual[:3, :3].T @ goal[:3, :3]).magnitude()
                        <= 0.001
                    ):
                        good.append(candidate)
                solutions.append(good)
                accepted += bool(good)
            self.batches.append(
                {
                    "targets": len(transforms),
                    "targets_with_solutions": accepted,
                    "seconds": perf_counter() - began,
                }
            )
        return solutions

    def transit_candidates(self, q0, q1):
        """Propose Cartesian pen-off routes; callers must certify every edge."""
        endpoints = np.array([self.arm.robot.forward(q, self.arm.flange_to_tcp) for q in (q0, q1)])
        count = max(
            3, int(np.ceil(np.linalg.norm(endpoints[1, :3, 3] - endpoints[0, :3, 3]) / 0.02)) + 1
        )
        times = np.linspace(0, 1, count)
        targets = np.repeat(np.eye(4)[None], count, axis=0)
        targets[:, :3, 3] = (1 - times[:, None]) * endpoints[0, :3, 3] + times[:, None] * endpoints[
            1, :3, 3
        ]
        targets[:, :3, :3] = Slerp([0, 1], Rotation.from_matrix(endpoints[:, :3, :3]))(
            times
        ).as_matrix()
        for reference in (q0, q1):
            layers = self.solve(targets[1:-1], reference=reference)
            path = [np.asarray(q0)]
            for layer in layers:
                if not layer:
                    break
                path.append(min(layer, key=lambda q: np.linalg.norm(q - path[-1])))
            else:
                yield np.array([*path, q1])
