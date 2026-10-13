"""Held-out proposal measurements with an independent validator callback."""

from time import perf_counter

import numpy as np

from .data import digest, load_dataset


def evaluate(provider, dataset, *, split="test", count=4, validator=None):
    manifest, rows = load_dataset(dataset)
    if manifest["dataset_hash"] != provider.checkpoint["dataset_hash"]:
        raise ValueError("Evaluation dataset differs from checkpoint frozen dataset")
    rows = [row for row in rows if row["split"] == split]
    if not rows:
        raise ValueError(f"No {split} motions")
    training = set(provider.checkpoint["training_ids"])
    if any(row["id"] in training for row in rows) and split != "train":
        raise ValueError("Evaluation leaked training episodes")
    results = []
    for index, row in enumerate(rows):
        episode = row["episode"]
        started = perf_counter()
        proposals = provider.propose(episode["request"], episode["scene"], count, seed=1000 + index)
        rejection, verified = {}, False
        distances = []
        for proposal in proposals:
            distances.append(float(np.sqrt(((proposal.q_path - row["q_path"]) ** 2).mean())))
            if validator is not None:
                status = validator(episode, proposal.q_path, proposal.source_points_m)
                status = (
                    status
                    if isinstance(status, str)
                    else ("VERIFIED_VALID" if status is True else "UNRESOLVED")
                )
                rejection[status] = rejection.get(status, 0) + 1
                if status == "VERIFIED_VALID":
                    verified = True
                    break
        results.append(
            {
                "id": row["id"],
                "verified": verified if validator else None,
                "joint_rmse_best_rad": min(distances, default=None),
                "elapsed_seconds": perf_counter() - started,
                "reasons": rejection,
                **provider.stats,
            }
        )
    return {
        "schema_version": 1,
        "dataset_hash": manifest["dataset_hash"],
        "split": split,
        "request_count": len(rows),
        "has_independent_validator": validator is not None,
        "verified_count": sum(r["verified"] is True for r in results) if validator else None,
        "results": results,
        "report_id": digest(results),
        "scope": "Generated proposals only; paired full-pipeline baseline required for benefit",
    }
