"""Actual serial-chain FK in torch; geometric losses are proposal guidance only."""

import numpy as np
import torch
from torch import nn


class TorchKinematics(nn.Module):
    def __init__(self, metadata):
        super().__init__()
        self.chain = metadata["chain"]
        self.register_buffer("base", torch.tensor(metadata["world_from_base"], dtype=torch.float32))
        self.register_buffer("tcp", torch.tensor(metadata["flange_to_tcp"], dtype=torch.float32))
        self.register_buffer("lower", torch.tensor(metadata["lower"], dtype=torch.float32))
        self.register_buffer("upper", torch.tensor(metadata["upper"], dtype=torch.float32))
        self.bodies = metadata.get("bodies", [])

    def frames(self, q):
        frame = self.base.expand(*q.shape[:-1], 4, 4)
        frames = {}
        for joint in self.chain:
            origin = torch.as_tensor(joint["origin"], dtype=q.dtype, device=q.device)
            frame = frame @ origin
            if joint["index"] is not None:
                axis = torch.as_tensor(joint["axis"], dtype=q.dtype, device=q.device)
                x, y, z = axis.unbind()
                zero = x * 0
                skew = torch.stack((zero, -z, y, z, zero, -x, -y, x, zero)).reshape(3, 3)
                angle = q[..., joint["index"]][..., None, None]
                rotation = (
                    torch.eye(4, device=q.device, dtype=q.dtype).expand(*q.shape[:-1], 4, 4).clone()
                )
                rotation[..., :3, :3] = (
                    torch.eye(3, device=q.device, dtype=q.dtype)
                    + torch.sin(angle) * skew
                    + (1 - torch.cos(angle)) * (skew @ skew)
                )
                frame = frame @ rotation
            frames[joint["child"]] = frame
        return frames

    def forward(self, q):
        return (list(self.frames(q).values())[-1] @ self.tcp)[..., :3, 3]

    def collision_proxy(self, q, capsules):
        """Conservative body bounding spheres against deposited capsules.

        These enclosing spheres deliberately overestimate collision. Loss is a
        heuristic; neither low loss nor high loss establishes feasibility.
        """
        if capsules.shape[1] == 0 or not self.bodies:
            return q.sum() * 0
        frames = self.frames(q)
        losses = []
        start, end = capsules[:, None, :, :3], capsules[:, None, :, 3:6]
        radius, valid = capsules[:, None, :, 6], capsules[:, None, :, 7]
        direction = end - start
        denominator = direction.square().sum(-1).clamp_min(1e-12)
        for body in self.bodies:
            if body["link"] not in frames:
                continue
            transform = torch.as_tensor(body["origin"], device=q.device, dtype=q.dtype)
            center = (frames[body["link"]] @ transform)[..., :3, 3, None].transpose(-1, -2)
            position = ((center - start) * direction).sum(-1) / denominator
            nearest = start + position.clamp(0, 1)[..., None] * direction
            distance = torch.linalg.vector_norm(center - nearest, dim=-1)
            penetration = (body["radius"] + radius + 0.001 - distance).clamp_min(0)
            losses.append((penetration.square() * valid).mean())
        return torch.stack(losses).mean() if losses else q.sum() * 0


def metadata_from_arm(arm):
    robot = arm.robot
    return {
        "world_from_base": np.asarray(robot.world_from_base).tolist(),
        "flange_to_tcp": arm.flange_to_tcp.tolist(),
        "lower": robot.lower.tolist(),
        "upper": robot.upper.tolist(),
        "chain": [
            {
                "child": j["child"],
                "origin": j["origin"].tolist(),
                "index": j["index"],
                "axis": None if j["axis"] is None else j["axis"].tolist(),
            }
            for j in robot.joint_data
        ],
        "bodies": [
            {"link": b.link, "origin": b.origin.tolist(), "radius": b.radius}
            for b in arm.collision.bodies
        ],
    }
