"""Layered redundant-arm IK for drawing strokes, with sampled tracking checks.

Outputs remain simulation previews until tool collisions, pen-off transits,
physical pen calibration and coupled construction search have been validated.
"""

import argparse
from pathlib import Path
from time import monotonic

import numpy as np

from ..curves import flatten_curve, transform_points
from ..io import artifact, read_json, sha256, stage_update, write_json
from ..kinematics import inverse_candidates, pose_error, rigid_transform
from ..robot import SerialArm


def search_layers(targets, solve, edge_check, initial_seeds, beam_width=3, deadline=np.inf):
    """Retain alternate configurations, then backtrace the lowest-cost full path.

    Each target is solved using every retained previous configuration. Endpoint
    reachability alone is insufficient: edge_check must accept each connection.
    """
    if beam_width < 1:
        raise ValueError("beam_width must be positive")
    layers = []
    seeds = [np.asarray(seed) for seed in initial_seeds]
    for index, target in enumerate(targets):
        if monotonic() >= deadline:
            return {"status": "TIME_BUDGET", "failed_sample": index}
        candidates = solve(target, seeds)
        nodes = []
        for q in candidates:
            q = np.asarray(q)
            if index == 0:
                cost = min(float(np.linalg.norm(q - seed)) for seed in seeds)
                nodes.append({"q": q, "cost": cost, "parent": None, "edge_error": 0.0})
                continue
            parents = []
            for parent_index, previous in enumerate(layers[-1]):
                error = edge_check(previous["q"], q, targets[index - 1], target)
                if error is not None:
                    cost = previous["cost"] + float(np.linalg.norm(q - previous["q"]))
                    parents.append((cost, parent_index, error))
            if parents:
                cost, parent, error = min(parents)
                nodes.append({"q": q, "cost": cost, "parent": parent, "edge_error": error})
        nodes.sort(key=lambda node: node["cost"])
        distinct = []
        for node in nodes:
            if not any(np.linalg.norm(node["q"] - other["q"]) < 1e-3 for other in distinct):
                distinct.append(node)
        if not distinct:
            return {"status": "NO_CONNECTED_IK_PATH", "failed_sample": index}
        layers.append(distinct[:beam_width])
        seeds = [node["q"] for node in layers[-1]]
    if not layers:
        raise ValueError("At least one target pose is required")
    def backtrace(index):
        selected = []
        for layer in reversed(layers):
            node = layer[index]
            selected.append(node)
            index = node["parent"]
        selected.reverse()
        return {
            "positions": [node["q"].tolist() for node in selected],
            "maximum_sampled_interpolation_error_m": max(node["edge_error"] for node in selected),
        }

    alternatives = [backtrace(index) for index in range(len(layers[-1]))]
    return {
        "status": "SAMPLED_DRAWING_PATH",
        **alternatives[0],
        "path_alternatives": alternatives,
        "endpoint_alternatives": [node["q"].tolist() for node in layers[-1]],
        "retained_states": sum(len(layer) for layer in layers),
    }


def tracking_edge(robot, offset, q0, q1, p0, p1, tolerance_m, tolerance_rad, maximum_step):
    """Sample joint interpolation; reject branch jumps and excessive TCP drift."""
    if np.max(np.abs(q1 - q0)) > maximum_step:
        return None
    worst = 0.0
    for alpha in (0.25, 0.5, 0.75):
        target = p0.copy()
        target[:3, 3] = (1 - alpha) * p0[:3, 3] + alpha * p1[:3, 3]
        actual = robot.forward((1 - alpha) * q0 + alpha * q1, offset)
        error = pose_error(actual, target)
        distance = float(np.linalg.norm(error[:3]))
        if distance > tolerance_m or np.linalg.norm(error[3:]) > tolerance_rad:
            return None
        worst = max(worst, distance)
    return worst


def build_preview(run, maximum_seconds=300, beam_width=2, resume=False):
    if maximum_seconds <= 0:
        raise ValueError("maximum_seconds must be positive")
    run = Path(run)
    construction = read_json(run / "construction_plan.json")
    placement = read_json(run / "placement_search.json")
    robot = SerialArm(run / "robot.urdf")
    robot_manifest = read_json(run / "robot_manifest.json")
    for filename, expected in placement["inputs"].items():
        if sha256(run / filename) != expected:
            raise ValueError("Placement is stale; rerun placement before motion planning")
    if construction["provenance"]["strokes_sha256"] != sha256(run / "pen_strokes.json"):
        raise ValueError("Construction preview is stale")
    offset = rigid_transform(placement["assumptions"]["flange_to_tcp"])
    transform = rigid_transform(placement["selected"]["build_to_robot_base"])
    rotation = np.asarray(placement["assumptions"]["tcp_rotation"])
    initial = np.array(robot_manifest["initial_positions"])
    seeds = [initial]
    probes = [sample for sample in placement["selected"]["samples"] if sample["ik_found"]]
    deadline = monotonic() + maximum_seconds
    output = artifact(
        "drawing_motion_preview",
        frame="robot_base",
        units="m",
        angle_units="rad",
        status="RUNNING",
        simulation_only=True,
        collision_checked=False,
        transits_generated=False,
        continuous_tracking_proven=False,
        pen_feasible=None,
        construction_order_coupled=False,
        robot_variant=robot_manifest["variant"],
        flange_to_tcp=offset.tolist(),
        build_to_robot_base=transform.tolist(),
        inputs={
            name: sha256(run / name)
            for name in ["construction_plan.json", "placement_search.json", "robot.urdf"]
        },
        settings={
            "maximum_chord_m": 0.003,
            "flatten_tolerance_m": 0.0001,
            "ik_position_tolerance_m": 0.00005,
            "ik_orientation_tolerance_rad": 0.001,
            "interpolation_tolerance_m": 0.0003,
            "interpolation_tolerance_rad": 0.003,
            "maximum_joint_step_rad": 0.2,
            "beam_width": beam_width,
            "maximum_seconds": maximum_seconds,
        },
        strokes=[],
        requested_strokes=len(construction["actions"]),
        limitations=[
            "Sampled tracking does not prove a continuous error bound",
            "No collision, pen process or inter-stroke transit validation",
            "Alternatives retained within each stroke; construction order not backtracked",
            "Robot variant and mounted pen transform remain unconfirmed",
        ],
    )
    if resume and (run / "drawing_motion.json").exists():
        previous = read_json(run / "drawing_motion.json")
        old_settings = {k: v for k, v in previous["settings"].items() if k != "maximum_seconds"}
        new_settings = {k: v for k, v in output["settings"].items() if k != "maximum_seconds"}
        if previous["inputs"] != output["inputs"] or old_settings != new_settings:
            raise ValueError("Motion resume inputs or search settings changed")
        output["strokes"] = previous["strokes"]
        if output["strokes"] and output["strokes"][-1]["status"] == "TIME_BUDGET":
            output["strokes"].pop()
        for row in reversed(output["strokes"]):
            if row["status"] == "SAMPLED_DRAWING_PATH":
                seeds = [np.array(q) for q in row["endpoint_alternatives"]]
                break
    stage_update(run, "motion", "running", message="Searching per-stroke joint paths")
    settings = output["settings"]
    for stroke in construction["actions"][len(output["strokes"]) :]:
        if monotonic() >= deadline:
            output["status"] = "TIME_BUDGET"
            break
        points, _ = flatten_curve(
            stroke, settings["flatten_tolerance_m"], settings["maximum_chord_m"]
        )
        points = transform_points(points, transform)
        targets = np.repeat(np.eye(4)[None], len(points), axis=0)
        targets[:, :3, :3] = rotation
        targets[:, :3, 3] = points
        nearest = sorted(
            probes, key=lambda sample: np.linalg.norm(np.array(sample["tcp_m"]) - points[0])
        )[:2]
        initial_seeds = seeds + [np.array(sample["positions"]) for sample in nearest]

        def solve(target, previous):
            return [
                solution["positions"]
                for solution in inverse_candidates(
                    robot,
                    target,
                    offset,
                    previous,
                    position_tolerance=settings["ik_position_tolerance_m"],
                    orientation_tolerance=settings["ik_orientation_tolerance_rad"],
                    maximum_evaluations=60,
                )
            ]

        def edge_check(q0, q1, p0, p1):
            return tracking_edge(
                robot,
                offset,
                q0,
                q1,
                p0,
                p1,
                settings["interpolation_tolerance_m"],
                settings["interpolation_tolerance_rad"],
                settings["maximum_joint_step_rad"],
            )

        result = search_layers(targets, solve, edge_check, initial_seeds, beam_width, deadline)
        result.update(
            id=stroke["id"],
            source_stroke=stroke["source_stroke"],
            tcp_targets_m=points.tolist(),
            tcp_rotation=rotation.tolist(),
        )
        output["strokes"].append(result)
        if result["status"] == "SAMPLED_DRAWING_PATH":
            seeds = [np.array(q) for q in result["endpoint_alternatives"]]
        write_json(run / "drawing_motion.json", output)
        print(
            f"{len(output['strokes'])}/{output['requested_strokes']}: {result['status']}",
            flush=True,
        )
        if result["status"] == "TIME_BUDGET":
            output["status"] = "TIME_BUDGET"
            break
    else:
        output["status"] = "SAMPLED_DRAWING_PATHS"
    output["successful_strokes"] = sum(
        s["status"] == "SAMPLED_DRAWING_PATH" for s in output["strokes"]
    )
    if (
        output["status"] == "SAMPLED_DRAWING_PATHS"
        and output["successful_strokes"] != output["requested_strokes"]
    ):
        output["status"] = "INCOMPLETE_DRAWING_PATHS"
    write_json(run / "drawing_motion.json", output)
    stage_update(
        run,
        "motion",
        "partial",
        artifact="drawing_motion.json",
        message=f"{output['successful_strokes']}/{output['requested_strokes']} strokes "
        "have sampled IK paths; collision and transit planning pending",
    )
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, default=Path("runs/palace_15cm"))
    parser.add_argument("--maximum-seconds", type=float, default=300)
    parser.add_argument("--beam-width", type=int, default=2)
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    build_preview(args.run, args.maximum_seconds, args.beam_width, args.resume)
