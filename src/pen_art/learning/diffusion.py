"""Conditional DDPM training and DDIM proposals for continuous drawing paths.

The output is a piecewise-linear joint proposal. It is never an acceptance
certificate; callers must refine to the requested curve and validate continuously.
"""

import json
import math
from dataclasses import dataclass
from pathlib import Path
from time import perf_counter

import numpy as np
import torch
from torch import nn

from .data import REPRESENTATION, curve_samples, load_dataset, normalized_request
from .geometry import TorchKinematics


class Denoiser(nn.Module):
    def __init__(self, knots, condition_dim, hidden=256):
        super().__init__()
        self.knots = knots
        self.network = nn.Sequential(
            nn.Linear(knots * 7 + condition_dim + 32, hidden),
            nn.SiLU(),
            nn.Linear(hidden, hidden),
            nn.SiLU(),
            nn.Linear(hidden, hidden),
            nn.SiLU(),
            nn.Linear(hidden, knots * 7),
        )

    def forward(self, noisy, time, condition):
        frequency = torch.exp(torch.arange(16, device=noisy.device) * (-math.log(10000) / 15))
        angles = time[:, None] * frequency[None] * 1000
        embedding = torch.cat((angles.sin(), angles.cos()), -1)
        features = torch.cat((noisy.flatten(1), condition, embedding), -1)
        return self.network(features).reshape(-1, self.knots, 7)


def enforce_endpoints(clean, raw_condition, knots, center, scale):
    """Translate whole paths to q0 and distribute optional q1 correction smoothly."""
    offset = 3 * knots + 1
    start = (raw_condition[:, offset : offset + 7] - center) / scale
    has_start = raw_condition[:, offset + 7, None, None] > 0.5
    shifted = clean + (start - clean[:, 0])[:, None, :]
    clean = torch.where(has_start, shifted, clean)
    goal = (raw_condition[:, offset + 8 : offset + 15] - center) / scale
    has_goal = raw_condition[:, offset + 15, None, None] > 0.5
    progress = torch.linspace(0, 1, knots, device=clean.device)[None, :, None]
    corrected = clean + progress * (goal - clean[:, -1])[:, None, :]
    return torch.where(has_goal, corrected, clean)


def capsule_tensor(episodes, device):
    count = max((len(e["scene"].get("capsules", [])) for e in episodes), default=0)
    result = np.zeros((len(episodes), count, 8), dtype=np.float32)
    for i, episode in enumerate(episodes):
        for j, capsule in enumerate(episode["scene"].get("capsules", [])):
            values = (
                [*capsule["start"], *capsule["end"], capsule["radius_m"]]
                if isinstance(capsule, dict)
                else capsule
            )
            result[i, j, :7] = values
            result[i, j, 7] = 1
    return torch.as_tensor(result, device=device)


def train(
    dataset,
    output,
    *,
    steps=2000,
    batch_size=32,
    hidden=256,
    seed=17,
    device="cpu",
    kinematics=None,
    learning_rate=0.001,
    tracking_weight=0.05,
    limit_weight=0.01,
    collision_weight=0.001,
):
    """Train only train split; select best checkpoint with frozen validation loss."""
    if steps < 1 or batch_size < 1 or hidden < 8:
        raise ValueError("steps/batch size must be positive and hidden >=8")
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    progress_path = output.with_suffix(".progress.json")
    started = perf_counter()
    torch.manual_seed(seed)
    np.random.seed(seed)
    torch.set_num_threads(min(4, torch.get_num_threads()))
    dataset = Path(dataset)
    manifest, rows = load_dataset(dataset)
    train_rows = [row for row in rows if row["split"] == "train"]
    val_rows = [row for row in rows if row["split"] == "validation"]
    if not train_rows or not val_rows:
        raise ValueError("Training requires independent train and validation paths")
    knots = manifest["knots"]
    raw_conditions = np.array(
        [
            np.r_[normalized_request(row["episode"]["request"], knots), row["signature"]]
            for row in rows
        ],
        dtype=np.float32,
    )
    training_indices = [i for i, row in enumerate(rows) if row["split"] == "train"]
    q_all = np.array([row["q_path"] for row in rows], dtype=np.float32)
    q_center = q_all[training_indices].mean(axis=(0, 1))
    q_scale = np.maximum(q_all[training_indices].std(axis=(0, 1)), 0.1)
    c_center = raw_conditions[training_indices].mean(axis=0)
    c_scale = np.maximum(raw_conditions[training_indices].std(axis=0), 0.01)
    field_offset = 3 * knots + 1
    c_scale[[field_offset + 7, field_offset + 15]] = 1
    c_scale[field_offset + 16 : field_offset + 20] = 1
    c_scale[field_offset + 20 : field_offset + 48] = np.maximum(
        c_scale[field_offset + 20 : field_offset + 48], 0.1
    )
    c_scale[field_offset + 48 :] = 1
    q = torch.as_tensor((q_all - q_center) / q_scale, device=device)
    conditions = torch.as_tensor((raw_conditions - c_center) / c_scale, device=device)
    condition_center = torch.as_tensor(c_center, device=device)
    condition_scale = torch.as_tensor(c_scale, device=device)
    source = torch.as_tensor(
        np.array(
            [
                curve_samples(
                    row["episode"]["request"].get(
                        "source_reference_points_m", row["episode"]["request"]["source_points_m"]
                    ),
                    knots,
                )
                for row in rows
            ]
        ),
        dtype=torch.float32,
        device=device,
    )
    center = torch.as_tensor(q_center, device=device)
    scale = torch.as_tensor(q_scale, device=device)
    capsules = capsule_tensor([row["episode"] for row in rows], device)
    geometry = TorchKinematics(kinematics).to(device) if kinematics is not None else None
    model = Denoiser(knots, conditions.shape[1], hidden).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=learning_rate, weight_decay=0.0001)
    # Cosine cumulative noise schedule reaches nearly pure noise at its terminal step.
    clock = torch.linspace(0, 1, 101, device=device)
    cumulative = torch.cos((clock + 0.008) / 1.008 * math.pi / 2).square()
    alphas = (cumulative[1:] / cumulative[0]).clamp(min=1e-5, max=0.9999)
    train_idx = torch.tensor(training_indices, device=device)
    val_idx = torch.tensor(
        [i for i, row in enumerate(rows) if row["split"] == "validation"], device=device
    )
    generator = torch.Generator(device=device).manual_seed(seed + 1)
    val_t = torch.linspace(0, len(alphas) - 1, len(val_idx), device=device).long()
    val_noise = torch.randn(q[val_idx].shape, device=device, generator=generator)
    best, history, best_state = float("inf"), [], None
    for step in range(steps):
        model.train()
        idx = train_idx[
            torch.randint(len(train_idx), (min(batch_size, len(train_idx)),), device=device)
        ]
        t = torch.randint(len(alphas), (len(idx),), device=device)
        a = alphas[t, None, None]
        noisy = a.sqrt() * q[idx] + (1 - a).sqrt() * torch.randn_like(q[idx])
        # Optional boundaries and orientation can be omitted by the construction planner.
        # Drop complete fields (including presence masks), never only their numeric values.
        train_condition = conditions[idx].clone()
        raw_train = train_condition * condition_scale + condition_center
        offset = 3 * knots + 1
        for first, last in (
            (offset, offset + 8),
            (offset + 8, offset + 16),
            (offset + 20, offset + 48),
        ):
            mask = torch.rand(len(idx), device=device) < 0.2
            raw_train[mask, first:last] = 0
        train_condition = (raw_train - condition_center) / condition_scale
        clean = model(noisy, t.float() / len(alphas), train_condition)
        clean = enforce_endpoints(clean, raw_train, knots, center, scale)
        denoising = (clean - q[idx]).square().mean()
        loss = denoising
        track = limits = collision = clean.sum() * 0
        if geometry is not None:
            physical = clean * scale + center
            # Exact FK at knots and midpoints; acceptance checks the full path separately.
            sample_q = torch.cat((physical, (physical[:, 1:] + physical[:, :-1]) / 2), dim=1)
            target_xyz = torch.cat((source[idx], (source[idx, 1:] + source[idx, :-1]) / 2), dim=1)
            track = ((geometry(sample_q) - target_xyz) / 0.01).square().mean()
            limits = (
                (geometry.lower - physical).clamp_min(0).square()
                + (physical - geometry.upper).clamp_min(0).square()
            ).mean()
            collision = geometry.collision_proxy(sample_q, capsules[idx])
            loss = loss + tracking_weight * track + limit_weight * limits
            loss = loss + collision_weight * collision
        if not torch.isfinite(loss):
            raise ValueError("Non-finite training loss; checkpoint was not written")
        optimizer.zero_grad(set_to_none=True)
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
        optimizer.step()
        if step == 0 or (step + 1) % max(1, steps // 20) == 0:
            model.eval()
            with torch.no_grad():
                va = alphas[val_t, None, None]
                predicted = model(
                    va.sqrt() * q[val_idx] + (1 - va).sqrt() * val_noise,
                    val_t.float() / len(alphas),
                    conditions[val_idx],
                )
                raw_val = conditions[val_idx] * condition_scale + condition_center
                predicted = enforce_endpoints(predicted, raw_val, knots, center, scale)
                val_loss = float((predicted - q[val_idx]).square().mean())
            history.append(
                {
                    "step": step + 1,
                    "denoising_loss": float(denoising.detach()),
                    "validation_loss": val_loss,
                    "tracking_loss": float(track.detach()),
                    "joint_limit_loss": float(limits.detach()),
                    "collision_proxy_loss": float(collision.detach()),
                }
            )
            progress_path.write_text(
                json.dumps(
                    {
                        "status": "RUNNING",
                        "step": step + 1,
                        "steps": steps,
                        "elapsed_seconds": perf_counter() - started,
                        **history[-1],
                    },
                    indent=2,
                )
                + "\n"
            )
            if val_loss < best:
                best = val_loss
                best_state = {
                    key: tensor.detach().cpu().clone() for key, tensor in model.state_dict().items()
                }
    checkpoint = {
        "schema_version": 1,
        "representation": REPRESENTATION,
        "condition_curve": "source_reference_points_m",
        "context_hash": manifest["context_hash"],
        "dataset_hash": manifest["dataset_hash"],
        "probe_q": manifest["probe_q"],
        "knots": knots,
        "hidden": hidden,
        "condition_dim": conditions.shape[1],
        "model": best_state,
        "q_center": q_center.tolist(),
        "q_scale": q_scale.tolist(),
        "condition_center": c_center.tolist(),
        "condition_scale": c_scale.tolist(),
        "alphas": alphas.cpu().tolist(),
        "seed": seed,
        "kinematics": kinematics,
        "training_steps": steps,
        "optional_condition_dropout": 0.2,
        "training_ids": manifest["split_members"]["train"],
        "training_source_objects": sorted(
            {r["episode"]["request"]["source_object"] for r in train_rows}
        ),
        "training_families": sorted({r["episode"]["request"]["family_id"] for r in train_rows}),
        "validation_ids": manifest["split_members"]["validation"],
        "test_ids": manifest["split_members"]["test"],
        "loss_weights": {
            "tracking": tracking_weight,
            "limits": limit_weight,
            "collision_proxy": collision_weight,
        },
    }
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp")
    torch.save(checkpoint, temporary)
    temporary.replace(output)
    report = {
        "schema_version": 1,
        "dataset_hash": manifest["dataset_hash"],
        "seed": seed,
        "steps": steps,
        "training_seconds": perf_counter() - started,
        "best_validation_loss": best,
        "history": history,
        "split_counts": manifest["valid_counts"],
        "acceptance": "Training evidence only; no held-out planning gain established",
    }
    output.with_suffix(".training.json").write_text(json.dumps(report, indent=2) + "\n")
    progress_path.write_text(json.dumps({"status": "COMPLETE", **report}, indent=2) + "\n")
    return report


@dataclass
class GeneratedProposal:
    q_path: np.ndarray
    source_points_m: np.ndarray
    metadata: dict


class MotionDiffusionProvider:
    """Context-bound generator; the caller supplies exact collision signatures."""

    def __init__(self, checkpoint_path, context_hash, *, signature_fn=None, device="cpu"):
        self.checkpoint_path = str(checkpoint_path)
        if str(device).startswith("cpu"):
            # These small dense batches otherwise pay a large server-wide thread-pool cost.
            torch.set_num_threads(min(4, torch.get_num_threads()))
        checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
        if (
            checkpoint.get("schema_version") != 1
            or checkpoint.get("representation") != REPRESENTATION
        ):
            raise ValueError("Unsupported diffusion checkpoint")
        if context_hash != checkpoint["context_hash"]:
            raise ValueError("Diffusion checkpoint model/static context mismatch")
        numeric_fields = (
            "q_center",
            "q_scale",
            "condition_center",
            "condition_scale",
            "alphas",
            "probe_q",
        )
        if any(not np.isfinite(checkpoint[field]).all() for field in numeric_fields):
            raise ValueError("Non-finite checkpoint statistics")
        if np.shape(checkpoint["q_center"]) != (7,) or np.shape(checkpoint["q_scale"]) != (7,):
            raise ValueError("Invalid checkpoint joint dimensions")
        if np.any(np.asarray(checkpoint["q_scale"]) <= 0) or np.any(
            np.asarray(checkpoint["condition_scale"]) <= 0
        ):
            raise ValueError("Invalid checkpoint normalization")
        alphas = np.asarray(checkpoint["alphas"])
        if (
            alphas.ndim != 1
            or len(alphas) < 2
            or np.any(alphas <= 0)
            or np.any(alphas >= 1)
            or np.any(np.diff(alphas) >= 0)
        ):
            raise ValueError("Invalid checkpoint diffusion schedule")
        if any(not torch.isfinite(value).all() for value in checkpoint["model"].values()):
            raise ValueError("Non-finite checkpoint parameters")
        self.checkpoint, self.signature_fn, self.device = checkpoint, signature_fn, device
        self.model = Denoiser(
            checkpoint["knots"], checkpoint["condition_dim"], checkpoint["hidden"]
        ).to(device)
        self.model.load_state_dict(checkpoint["model"])
        self.model.eval()
        self.probe_q = np.asarray(checkpoint["probe_q"])
        self.stats = {}

    def propose(self, request, scene, count=4, *, seed=17, steps=20):
        if count < 1 or steps < 1:
            raise ValueError("count and steps must be positive")
        if hasattr(request, "as_dict"):
            request = request.as_dict()
        checkpoint, device = self.checkpoint, self.device
        if request.get("context_hash") not in (None, "", checkpoint["context_hash"]):
            raise ValueError("Drawing request model/static context mismatch")
        start = perf_counter()
        if self.signature_fn is None:
            raise ValueError("Collision signature callback is required")
        signature = np.asarray(
            self.signature_fn({"request": request, "scene": scene}, self.probe_q)
        )
        if signature.shape != (len(self.probe_q),) or not np.isin(signature, (0, 1)).all():
            raise ValueError("Invalid scene collision signature")
        encoding_seconds = perf_counter() - start
        condition_request = dict(request)
        if checkpoint.get("condition_curve", "source_points_m") == "source_points_m":
            condition_request.pop("source_reference_points_m", None)
        raw = np.r_[normalized_request(condition_request, checkpoint["knots"]), signature]
        condition = torch.as_tensor(
            (raw - checkpoint["condition_center"]) / np.asarray(checkpoint["condition_scale"]),
            dtype=torch.float32,
            device=device,
        ).expand(count, -1)
        center = torch.tensor(checkpoint["q_center"], dtype=torch.float32, device=device)
        scale = torch.tensor(checkpoint["q_scale"], dtype=torch.float32, device=device)
        alphas = torch.tensor(checkpoint["alphas"], device=device)
        generator = torch.Generator(device=device).manual_seed(seed)
        value = torch.randn((count, checkpoint["knots"], 7), generator=generator, device=device)
        schedule = np.unique(np.linspace(len(alphas) - 1, 0, min(steps, len(alphas)), dtype=int))[
            ::-1
        ]
        inference_start = perf_counter()
        with torch.no_grad():
            for i, time in enumerate(schedule):
                t = torch.full((count,), float(time) / len(alphas), device=device)
                clean = self.model(value, t, condition)
                raw_batch = torch.as_tensor(raw, dtype=torch.float32, device=device).expand(
                    count, -1
                )
                clean = enforce_endpoints(clean, raw_batch, checkpoint["knots"], center, scale)
                if i == len(schedule) - 1:
                    value = clean
                else:
                    noise = (value - alphas[time].sqrt() * clean) / (1 - alphas[time]).sqrt()
                    next_alpha = alphas[schedule[i + 1]]
                    value = next_alpha.sqrt() * clean + (1 - next_alpha).sqrt() * noise
            paths = (value * scale + center).cpu().numpy()
        self.stats = {
            "encoding_seconds": encoding_seconds,
            "generation_seconds": perf_counter() - inference_start,
            "proposal_count": count,
            "seed": seed,
        }
        source = curve_samples(
            condition_request.get(
                "source_reference_points_m", condition_request["source_points_m"]
            ),
            checkpoint["knots"],
        )
        return [
            GeneratedProposal(
                path,
                source.copy(),
                {
                    "kind": "diffusion",
                    "checkpoint": self.checkpoint_path,
                    "dataset_hash": checkpoint["dataset_hash"],
                    "context_hash": checkpoint["context_hash"],
                    "seed": seed,
                    "candidate_index": index,
                    "requires_validation": True,
                    "representation": REPRESENTATION,
                    **self.stats,
                },
            )
            for index, path in enumerate(paths)
            if np.isfinite(path).all()
        ]
