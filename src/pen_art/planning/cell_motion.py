"""World-frame drawing alternatives for independently placed serial arms.

These are drawing-path candidates, not coordinated schedules: transits, material
and inter-arm validation belong to selected-motion realization and shared replay.
"""

import argparse
import shutil
from collections import Counter
from pathlib import Path
from time import monotonic

import numpy as np
from scipy.spatial.transform import Rotation

from ..cell import load_cell
from ..curves import flatten_curve, transform_points
from ..io import artifact, read_json, sha256, stage_update, write_json
from ..kinematics import inverse_candidates
from ..strokes import orient_stroke
from .curve_motion import search_layers, tracking_edge

DEFAULT_SETTINGS = {
    "maximum_chord_m": 0.003,
    "flatten_tolerance_m": 0.0001,
    "ik_position_tolerance_m": 0.00005,
    "ik_orientation_tolerance_rad": 0.001,
    "interpolation_tolerance_m": 0.0003,
    "interpolation_tolerance_rad": 0.003,
    "maximum_joint_step_rad": 0.2,
    "beam_width": 2,
    "maximum_subdivision_depth": 12,
}


def pen_rotation(arm, world_from_build, roll=0.0, tilt_deg=0.0):
    """Fixed inward tip axis, keeping the pen body on its robot's side.

    Zero tilt keeps the nozzle vertical in the shared world, even on a tilted base.
    Horizontal bases preserve their existing downward orientation exactly.
    The policy is a pose prescription, not a clearance or process certificate.
    """
    if not np.isfinite(tilt_deg) or not 0 <= tilt_deg < 90:
        raise ValueError("Drawing pen tilt must be finite and in [0, 90) degrees")
    base = arm.robot.world_from_base
    down = base[:3, :3] @ Rotation.from_rotvec([0, 0, roll]).as_matrix() @ np.diag([-1, 1, -1])
    if tilt_deg == 0 and np.allclose(base[:3, 2], [0, 0, 1], atol=1e-12, rtol=0):
        return down
    inward = np.asarray(world_from_build)[:2, 3] - base[:2, 3]
    length = np.linalg.norm(inward)
    if length < 1e-9 and tilt_deg != 0:
        raise ValueError("Inward pen tilt requires a base offset from the build center")
    angle = np.deg2rad(tilt_deg)
    z = (
        np.array([0.0, 0.0, -1.0])
        if tilt_deg == 0
        else np.r_[np.sin(angle) * inward / length, -np.cos(angle)]
    )
    reference = base[:3, :3] @ np.diag([-1, 1, -1])
    x = reference[:, 0] - np.dot(reference[:, 0], z) * z
    if np.linalg.norm(x) < 1e-9:
        x = reference[:, 1] - np.dot(reference[:, 1], z) * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z]) @ Rotation.from_rotvec([0, 0, -roll]).as_matrix()


def anchored_strokes(stroke, available):
    """Re-root closed contours, or reverse open contours, at existing anchors."""
    if stroke["closed"]:
        for index, visit in enumerate(stroke["junction_visits"][:-1]):
            if visit["node"] in available:
                for reverse in (False, True):
                    yield orient_stroke(stroke, reverse=reverse, start_visit=index)
    else:
        if stroke["start"] in available:
            yield orient_stroke(stroke)
        if stroke["end"] in available:
            yield orient_stroke(stroke, reverse=True)


def drawing_alternatives(
    arm,
    stroke,
    available,
    world_from_build,
    obstacles,
    *,
    deadline,
    seeds=None,
    settings=None,
    roll_angles=(0.0, np.pi),
    clearance_m=0.001,
    pen_tilt_deg=0.0,
):
    """Yield actual IK paths, including every retained endpoint backtrace.

    Collision checks here cover self and static environment only. Every path is
    rechecked against the state-dependent material and other arm when scheduled.
    Pen-axis roll is a search variable; optional inward tilt is fixed per cell.
    """
    settings = {**DEFAULT_SETTINGS, **(settings or {})}
    seeds = [arm.initial_positions] if seeds is None else seeds
    for oriented in anchored_strokes(stroke, available):
        points, _ = flatten_curve(
            oriented, settings["flatten_tolerance_m"], settings["maximum_chord_m"]
        )
        points = transform_points(points, world_from_build)
        for roll_index, roll in enumerate(roll_angles):
            if monotonic() >= deadline:
                return
            rotation = pen_rotation(arm, world_from_build, roll, pen_tilt_deg)
            targets = np.repeat(np.eye(4)[None], len(points), axis=0)
            targets[:, :3, :3], targets[:, :3, 3] = rotation, points

            def solve(target, previous):
                if monotonic() >= deadline:
                    return []
                solutions = inverse_candidates(
                    arm.robot,
                    target,
                    arm.flange_to_tcp,
                    previous,
                    position_tolerance=settings["ik_position_tolerance_m"],
                    orientation_tolerance=settings["ik_orientation_tolerance_rad"],
                    maximum_evaluations=60,
                )
                return [
                    s["positions"]
                    for s in solutions
                    if not arm.collision.check(s["positions"], obstacles, clearance_m)
                ]

            def edge_check(q0, q1, p0, p1):
                error = tracking_edge(
                    arm.robot,
                    arm.flange_to_tcp,
                    q0,
                    q1,
                    p0,
                    p1,
                    settings["interpolation_tolerance_m"],
                    settings["interpolation_tolerance_rad"],
                    settings["maximum_joint_step_rad"],
                )
                if error is None:
                    return None
                check = arm.collision.check_edge(
                    q0,
                    q1,
                    obstacles,
                    clearance_m,
                    settings["maximum_subdivision_depth"],
                    deadline=deadline,
                )
                return error if check["status"] == "CERTIFIED_MODELED_EDGE" else None

            result = search_layers(
                targets,
                solve,
                edge_check,
                seeds,
                settings["beam_width"],
                deadline,
            )
            base = {
                "arm": arm.identity,
                "stroke_id": stroke["id"],
                "start_anchor": oriented["start"],
                "end_anchor": oriented["end"],
                "reverse": oriented["reverse"],
                "start_visit": oriented["start_visit"],
                "roll_rad": float(roll),
                "pen_tilt_deg": float(pen_tilt_deg),
                "tcp_rotation": rotation.tolist(),
                "tcp_targets_m": points.tolist(),
            }
            if result["status"] != "SAMPLED_DRAWING_PATH":
                yield {**base, **result}
                continue
            for index, path in enumerate(result["path_alternatives"]):
                yield {
                    **base,
                    **path,
                    "status": "SAMPLED_DRAWING_PATH",
                    "variant": (
                        f"anchor-{oriented['start_visit']}-reverse-{int(oriented['reverse'])}"
                        f"-roll-{roll_index}-branch-{index}"
                    ),
                    "static_collision_status": "CERTIFIED_MODELED_PATH",
                    "tracking_status": "SAMPLED_ONLY",
                }


def build_cell_variants(
    run,
    profile_path,
    *,
    maximum_seconds=300,
    per_arm_stroke=2,
    resume=False,
):
    """Persist an interleaved, bounded set of candidates for the real source graph."""
    run = Path(run)
    if maximum_seconds <= 0 or per_arm_stroke < 1:
        raise ValueError("Positive time and per-arm candidate budgets are required")
    profile, arms, obstacles = load_cell(profile_path)
    cell = read_json(run / "cell_manifest.json")
    if profile != read_json(run / "cell_config.json"):
        raise ValueError("Cell profile changed; regenerate the setup before planning")
    for name, expected in cell["inputs"].items():
        if sha256(run / name) != expected:
            raise ValueError(f"Cell setup input changed: {name}")
    # Ensure the files used by load_cell are exactly the self-contained snapshot.
    for identity, arm in arms.items():
        for suffix, path in (("robot.urdf", arm.urdf), ("robot.json", arm.manifest_path)):
            if sha256(path) != sha256(run / f"{identity}_{suffix}"):
                raise ValueError(f"Live robot input changed for {identity}")
        saved = next(row for row in cell["arms"] if row["id"] == identity)
        if arm.tool.metadata != saved["tool_model"]:
            raise ValueError(f"Live tool input changed for {identity}")
    source = Path(read_json(run / "run.json")["cell_source_run"])
    if (run / "cell_source_bundle.json").is_file():
        bundle = read_json(run / "cell_source_bundle.json")
        for name, expected in bundle["inputs"].items():
            if sha256(run / name) != expected:
                raise ValueError(f"Cell source snapshot changed: {name}")
        for name, expected in bundle["source_sha256"].items():
            if sha256(source / name) != expected:
                raise ValueError(f"Source geometry changed: {name}; create a new cell run")
    else:
        for name in ("construction_plan.json", "stroke_graph.json", "pen_strokes.json"):
            if resume and (run / name).exists() and sha256(source / name) != sha256(run / name):
                raise ValueError("Source geometry changed; cannot resume motion candidates")
            shutil.copy2(source / name, run / name)
    inputs = {
        name: sha256(run / name)
        for name in (
            "cell_manifest.json",
            "construction_plan.json",
            "stroke_graph.json",
            "pen_strokes.json",
        )
    }
    construction = read_json(run / "construction_plan.json")
    graph = read_json(run / "stroke_graph.json")
    members = Counter(member for action in construction["actions"] for member in action["members"])
    if members != Counter(edge["id"] for edge in graph["edges"]):
        raise ValueError(
            "Construction does not cover the selected graph exactly once; "
            "prepare the matching source variant before generating motion"
        )
    path = run / "cell_drawing_variants.json"
    output = artifact(
        "cell_drawing_variants",
        status="RUNNING",
        simulation_only=True,
        executable=False,
        frame="world",
        units="m",
        angle_units="rad",
        inputs=inputs,
        settings=DEFAULT_SETTINGS,
        candidate_limit_per_arm_stroke=per_arm_stroke,
        variants=[],
        attempts=[],
        groups=[],
        requested_groups=sum(
            len(set(stroke.get("allowed_arms", list(arms))) & arms.keys())
            for stroke in construction["actions"]
        ),
        limitations=[
            "Bounded candidate set, not an exhaustive proof of reachability",
            "Self/static arm checks pass; current material and other arm are not checked here",
            "Tracking is sampled; pen body/mount checks require supplied geometry",
            "No approach, withdrawal, pen-off transit or common-clock schedule yet",
        ],
    )
    if resume and path.exists():
        previous = read_json(path)
        if (
            previous["inputs"] != inputs
            or previous["settings"] != DEFAULT_SETTINGS
            or previous["candidate_limit_per_arm_stroke"] != per_arm_stroke
        ):
            raise ValueError("Motion-candidate inputs/settings changed; cannot resume")
        if (
            previous["status"] == "BOUNDED_CANDIDATES"
            and len(previous["groups"]) == output["requested_groups"]
        ):
            return previous
        output.update({key: previous[key] for key in ("variants", "attempts", "groups")})
    deadline = monotonic() + maximum_seconds
    stage_update(
        run,
        "cell",
        "running",
        artifact="cell_drawing_variants.json",
        message="Building the fixed arm/action drawing catalogue",
    )
    seeds = {identity: [arm.initial_positions] for identity, arm in arms.items()}
    for stroke in construction["actions"]:
        for identity, arm in arms.items():
            if identity not in stroke.get("allowed_arms", list(arms)):
                continue
            if [stroke["id"], identity] in output["groups"]:
                continue
            if monotonic() >= deadline:
                break
            available = {visit["node"] for visit in stroke["junction_visits"]}
            existing = {
                row["variant"]
                for row in output["variants"]
                if row["arm"] == identity and row["stroke_id"] == stroke["id"]
            }
            accepted = len(existing)
            if accepted >= per_arm_stroke:
                output["groups"].append([stroke["id"], identity])
                continue
            for candidate in drawing_alternatives(
                arm,
                stroke,
                available,
                np.asarray(profile["world_from_build"]),
                obstacles,
                deadline=deadline,
                seeds=seeds[identity],
                clearance_m=profile["environment"]["clearance_m"],
                pen_tilt_deg=profile.get("drawing_pen_tilt_deg", 0.0),
            ):
                if candidate["status"] == "SAMPLED_DRAWING_PATH":
                    if candidate["variant"] in existing:
                        continue
                    output["variants"].append(candidate)
                    seeds[identity] = [np.array(candidate["positions"][-1])]
                    accepted += 1
                else:
                    output["attempts"].append(candidate)
                write_json(path, output)
                if accepted >= per_arm_stroke:
                    break
            if monotonic() < deadline:
                output["groups"].append([stroke["id"], identity])
            print(f"{stroke['id']} / {identity}: {accepted} candidates", flush=True)
            write_json(path, output)
            stage_update(
                run,
                "cell",
                "running",
                artifact="cell_drawing_variants.json",
                message=f"{len(output['groups'])} / {output['requested_groups']} arm/action "
                f"groups checked; {len(output['variants'])} fixed drawing paths",
            )
    expected_groups = output["requested_groups"]
    output["status"] = (
        "BOUNDED_CANDIDATES" if len(output["groups"]) == expected_groups else "TIME_BUDGET"
    )
    output["requested_groups"] = expected_groups
    write_json(path, output)
    stage_update(
        run,
        "cell",
        "partial",
        artifact="cell_drawing_variants.json",
        message=f"{len(output['variants'])} arm-specific drawing alternatives; "
        "coordination pending",
    )
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, default=Path("runs/palace_15cm_dual"))
    parser.add_argument("--config", type=Path, default=Path("configs/cell/two_flexiv.yaml"))
    parser.add_argument("--maximum-seconds", type=float, default=300)
    parser.add_argument("--per-arm-stroke", type=int, default=2)
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    build_cell_variants(
        args.run,
        args.config,
        maximum_seconds=args.maximum_seconds,
        per_arm_stroke=args.per_arm_stroke,
        resume=args.resume,
    )
