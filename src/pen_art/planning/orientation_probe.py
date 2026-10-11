"""Compare one recorded wrist-conflict pose with a fixed inward pen tilt.

This is a static diagnostic. It never edits the saved drawing catalogue or
claims that the alternative orientation admits a complete drawing trajectory.
"""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from ..cell import load_cell
from ..io import artifact, read_json, sha256, write_json
from ..kinematics import inverse_candidates
from .cell_motion import pen_rotation
from .coordinated_motion import check_pair_linear


def probe_orientation(run, profile_path, tilt_deg=30.0):
    run = Path(run)
    source = run / "allocation_motions.json"
    raw = source.read_bytes()
    history = json.loads(raw)
    witness = next(
        (
            r["witness"]
            for r in reversed(history.get("refinements", []))
            if r.get("witness", {}).get("phase") == "INTER_ARM"
            and len(r["witness"].get("positions", [])) == 2
        ),
        None,
    )
    if witness is None:
        raise ValueError("A recorded two-arm conflict pose is required")
    profile, arms, obstacles = load_cell(profile_path)
    if profile != read_json(run / "cell_config.json") or len(arms) != 2:
        raise ValueError("Use the same two-arm cell as the recorded conflict")
    cell = read_json(run / "cell_manifest.json")
    for document in (
        cell,
        history,
        read_json(run / "allocation_problem.json"),
        read_json(run / "cell_allocation.json"),
    ):
        for name, digest in document["inputs"].items():
            if sha256(run / name) != digest:
                raise ValueError(f"Stale pose-diagnostic input: {name}")
    for identity, arm in arms.items():
        if sha256(arm.urdf) != sha256(run / f"{identity}_robot.urdf"):
            raise ValueError(f"Live robot changed: {identity}")
        saved = next(row for row in cell["arms"] if row["id"] == identity)
        if arm.tool.metadata != saved["tool_model"]:
            raise ValueError(f"Live tool changed: {identity}")
    old = [np.asarray(q) for q in witness["positions"]]
    new, rows, old_flange, new_flange = [], [], [], []
    for (identity, arm), q in zip(arms.items(), old, strict=True):
        target = arm.robot.forward(q, arm.flange_to_tcp)
        point = target[:3, 3].copy()
        target[:3, :3] = pen_rotation(arm, profile["world_from_build"], tilt_deg=tilt_deg)
        solutions = inverse_candidates(arm.robot, target, arm.flange_to_tcp, [q])
        row = {"arm": identity, "target_tcp": target.tolist(), "ik_found": bool(solutions)}
        if solutions:
            value = np.asarray(solutions[0]["positions"])
            new.append(value)
            old_flange.append(arm.robot.forward(q)[:3, 3])
            new_flange.append(arm.robot.forward(value)[:3, 3])
            row.update(
                positions=value.tolist(),
                tcp_displacement_m=float(
                    np.linalg.norm(arm.robot.forward(value, arm.flange_to_tcp)[:3, 3] - point)
                ),
                static_contacts=arm.collision.check(
                    value, obstacles, profile["environment"]["clearance_m"]
                ),
            )
        rows.append(row)
    models = [arm.collision for arm in arms.values()]
    margin = profile["environment"]["clearance_m"]
    result = artifact(
        "pen_orientation_probe",
        inputs={"cell_manifest.json": sha256(run / "cell_manifest.json")},
        simulation_only=True,
        executable=False,
        tilt_deg=tilt_deg,
        scope="One recorded pose; available arm/tool bodies only. Full drawing, pen process "
        "and physical clearance remain unvalidated. Saved motions are unchanged.",
        source_checkpoint_sha256=hashlib.sha256(raw).hexdigest(),
        original_witness=witness,
        arms=rows,
        original_pair=check_pair_linear(*models, old[0], old[0], old[1], old[1], margin_m=margin),
    )
    if len(new) == 2:
        result.update(
            tilted_pair=check_pair_linear(*models, new[0], new[0], new[1], new[1], margin_m=margin),
            flange_separation_before_m=float(np.linalg.norm(old_flange[0] - old_flange[1])),
            flange_separation_after_m=float(np.linalg.norm(new_flange[0] - new_flange[1])),
        )
    write_json(run / "pen_orientation_probe.json", result)
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=Path("configs/cell/two_flexiv.yaml"))
    parser.add_argument("--tilt-deg", type=float, default=30.0)
    args = parser.parse_args()
    result = probe_orientation(args.run, args.config, args.tilt_deg)
    print(result.get("tilted_pair", {}).get("status", "NO_PAIRED_IK"))
