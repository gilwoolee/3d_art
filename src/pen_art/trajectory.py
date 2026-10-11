"""Export exact offline motion channels without contacting robot hardware.

Polynomial coefficients use normalized segment time u=(t-start)/(end-start).
Velocity and acceleration coefficients already include the physical time scale.
This export preserves the saved path; it does not resample or smooth it.
"""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from .cell import WorldArm
from .io import artifact, read_json, sha256, write_json
from .planning.clock_path import ClockPath
from .robot import SerialArm


def export_trajectory(run, plan_name="offline_dual_arm_plan"):
    run = Path(run)
    if Path(plan_name).name != plan_name:
        raise ValueError("Plan name must be a local artifact stem")
    name = plan_name + ".json"
    raw_plan = (run / name).read_bytes()
    plan = json.loads(raw_plan)
    if not plan.get("full_schedule"):
        raise ValueError("A full saved schedule is required for trajectory export")
    for source, digest in plan["inputs"].items():
        if sha256(run / source) != digest:
            raise ValueError(f"Stale trajectory source: {source}")
    inputs = {name: hashlib.sha256(raw_plan).hexdigest()}
    for source, digest in read_json(run / "cell_manifest.json")["inputs"].items():
        if sha256(run / source) != digest:
            raise ValueError(f"Stale cell snapshot: {source}")
    config = read_json(run / "cell_config.json")
    inputs["cell_config.json"] = sha256(run / "cell_config.json")
    channels, paths = {}, {}
    duration = float(plan["modeled_duration_s"])
    if not np.isfinite(duration) or duration <= 0:
        raise ValueError("A finite positive job duration is required")
    for spec in config["arms"]:
        identity = spec["id"]
        robot_name, manifest_name = f"{identity}_robot.urdf", f"{identity}_robot.json"
        inputs.update({n: sha256(run / n) for n in (robot_name, manifest_name)})
        robot = WorldArm(SerialArm(run / robot_name), spec["world_from_base"])
        if robot.names != read_json(run / manifest_name)["joint_names"]:
            raise ValueError("Snapshot joint names differ from its manifest")
        q = np.asarray(plan["initial_positions"][identity], dtype=float)
        offset = np.asarray(spec["flange_to_tcp"])
        segments, actions, cursor = [], [], 0.0

        def append(
            piece, start, end, kind, stroke, *, segments=segments, robot=robot, offset=offset
        ):
            segments.append(
                {
                    "start_s": start,
                    "end_s": end,
                    "kind": kind,
                    "stroke_id": stroke,
                    "position_rad": [p.coef.tolist() for p in piece.polynomials],
                    "velocity_rad_s": [
                        (p.deriv() / (end - start)).coef.tolist() for p in piece.polynomials
                    ],
                    "acceleration_rad_s2": [
                        (p.deriv(2) / (end - start) ** 2).coef.tolist() for p in piece.polynomials
                    ],
                    "tcp_start_world": robot.forward(piece.position(0), offset).tolist(),
                    "tcp_end_world": robot.forward(piece.position(1), offset).tolist(),
                }
            )

        def hold(q, start, end, append=append):
            from numpy.polynomial import Polynomial

            from .planning.smooth_audit import PolynomialPiece

            if end > start:
                append(PolynomialPiece([Polynomial([v]) for v in q]), start, end, "WAIT", None)

        for action in sorted(
            (a for a in plan["actions"] if a["arm"] == identity), key=lambda a: a["start_s"]
        ):
            path = ClockPath.from_record(action["trajectory"])
            start, end = action["start_s"], action["end_s"]
            if start < cursor - 1e-9 or abs(end - start - path.duration_s) > 1e-8:
                raise ValueError("Overlapping or inconsistent arm channel")
            if not np.allclose(q, path.edges[0].q0, atol=1e-8, rtol=0):
                raise ValueError("Discontinuous arm channel")
            hold(q, cursor, start)
            for edge in path.edges:
                append(
                    edge.piece(), start + edge.start_s, start + edge.end_s, edge.kind, edge.stroke
                )
            actions.append(action)
            q, cursor = path.edges[-1].q1, end
        if cursor > duration + 1e-8:
            raise ValueError("Trajectory exceeds declared duration")
        hold(q, cursor, duration)
        channels[identity] = {
            "joint_names": robot.names,
            "world_from_base": spec["world_from_base"],
            "flange_to_tcp": spec["flange_to_tcp"],
            "segments": segments,
        }
        paths[identity] = {"joint_names": robot.names, "actions": actions}
    if set(plan["initial_positions"]) != set(channels) or any(
        a["arm"] not in channels for a in plan["actions"]
    ):
        raise ValueError("Plan channels differ from the cell snapshot")
    replay_name = plan_name.removesuffix("_plan") + "_validation.json"
    raw_replay = (run / replay_name).read_bytes() if (run / replay_name).is_file() else None
    replay = json.loads(raw_replay) if raw_replay is not None else None
    if replay and replay.get("source_plan_sha256") == inputs[name]:
        inputs[replay_name] = hashlib.sha256(raw_replay).hexdigest()
        replay_status = replay["status"]
    else:
        replay_status = "NOT_CHECKED_OR_STALE"
    metadata = {
        "simulation_only": True,
        "executable": False,
        "inputs": inputs,
        "replay_status": replay_status,
        "modeled_duration_s": duration,
        "limitations": [
            "No robot or pen commands are sent by this export",
            "Robot, tool, process and controller timing require physical calibration",
            "A geometric replay certificate is not physical fabrication validation",
            "Acceleration can be discontinuous at segment boundaries; jerk is not certified",
        ],
    }
    write_json(run / "joint_path.json", artifact("joint_path", **metadata, channels=paths))
    result = artifact(
        "trajectory",
        **metadata,
        channels=channels,
        pen_events=plan.get("events", []),
        coefficient_convention="Ascending powers of u=(job_time-start_s)/(end_s-start_s)",
        boundary_convention="Use following segment at internal boundaries; final endpoint included",
    )
    write_json(run / "trajectory.json", result)
    write_json(
        run / "trajectory_manifest.json",
        artifact(
            "trajectory_manifest",
            **{
                **metadata,
                "inputs": {
                    **inputs,
                    "trajectory.json": sha256(run / "trajectory.json"),
                    "joint_path.json": sha256(run / "joint_path.json"),
                },
            },
            source_plan=name,
            source_plan_sha256=inputs[name],
            channels=list(channels),
        ),
    )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--plan-name", default="offline_dual_arm_plan")
    args = parser.parse_args()
    result = export_trajectory(args.run, args.plan_name)
    print(result["replay_status"], result["modeled_duration_s"])


if __name__ == "__main__":
    main()
