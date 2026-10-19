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
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from ..cell import load_cell
from ..curves import flatten_curve, transform_points
from ..io import artifact, read_json, sha256, stage_update, write_json
from ..kinematics import geometric_jacobian, inverse_candidates, pose_error
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


def pen_rotation(arm, world_from_build, roll=0.0, tilt_deg=0.0, lean_azimuth_deg=None):
    """Fixed inward tip axis, keeping the pen body on its robot's side.

    With ``lean_azimuth_deg`` the pen body leans toward that world azimuth
    instead, so the arm can draw the side of the object facing away from it.

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
    if lean_azimuth_deg is not None:
        heading = np.deg2rad(lean_azimuth_deg)
        inward, length = -np.array([np.cos(heading), np.sin(heading)]), 1.0
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


ORIENTATION_POLICY_VERSION = "fixed-tcp-per-draw-v5"
DEFAULT_PEN_TILTS_DEG = (70.0, 80.0)


def fixed_rotation_families(
    arm,
    world_from_build,
    tilt_degrees=DEFAULT_PEN_TILTS_DEG,
    roll_angles=(0.0, np.pi),
    *,
    lean_azimuth_deg=None,
):
    """Choose a complete TCP frame before drawing, then keep it unchanged."""
    for tilt in dict.fromkeys(float(value) for value in tilt_degrees):
        for roll in roll_angles:
            yield {
                "orientation_policy": "fixed",
                "fixed_orientation": True,
                "pen_tilt_deg": tilt,
                "roll_rad": float(roll),
                "rotations": pen_rotation(arm, world_from_build, roll, tilt, lean_azimuth_deg),
            }


def radial_pen_rotations(arm, points, world_from_build, tilt_deg=70.0, roll=0.0):
    """Aim TCP +Z inward/down; the axial pen body extends outward/up along -Z.

    Tilt is measured from vertical. It is a search preference, not a geometric
    clearance certificate. At a point on the central axis, use the robot side
    as the deterministic radial fallback. Near-horizontal poses use world up
    to fix roll without the rapid twist caused by projecting world X near a
    horizontal nozzle axis. Near-vertical poses retain a world-X reference.
    """
    if not np.isfinite(tilt_deg) or not 0 <= tilt_deg < 90:
        raise ValueError("Drawing pen tilt must be finite and in [0, 90) degrees")
    points = np.asarray(points, float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("Radial orientation needs finite Cartesian points")
    center = np.asarray(world_from_build)[:3, 3]
    radial = points[:, :2] - center[:2]
    lengths = np.linalg.norm(radial, axis=1)
    fallback = arm.robot.world_from_base[:2, 3] - center[:2]
    if np.linalg.norm(fallback) < 1e-9:
        fallback = np.array([1.0, 0.0])
    fallback /= np.linalg.norm(fallback)
    radial = np.divide(
        radial, lengths[:, None], out=np.zeros_like(radial), where=lengths[:, None] > 1e-9
    )
    radial[lengths <= 1e-9] = fallback
    angle = np.deg2rad(tilt_deg)
    z = np.column_stack([-np.sin(angle) * radial, np.full(len(points), -np.cos(angle))])
    reference = np.array([0.0, 0.0, 1.0] if tilt_deg >= 45 else [1.0, 0.0, 0.0])
    x = reference - z * (z @ reference)[:, None]
    x /= np.linalg.norm(x, axis=1)[:, None]
    frames = np.stack([x, np.cross(z, x), z], axis=2)
    return frames @ Rotation.from_rotvec([0, 0, -roll]).as_matrix()


def outward_rotation_families(
    arm, points, world_from_build, tilt_degrees=DEFAULT_PEN_TILTS_DEG, roll_angles=(0.0, np.pi)
):
    """Whole-path preferred families, preserving the caller's tilt ordering."""
    for tilt in dict.fromkeys(float(value) for value in tilt_degrees):
        for roll in roll_angles:
            yield {
                "orientation_policy": "outward_radial",
                "pen_tilt_deg": tilt,
                "roll_rad": float(roll),
                "rotations": radial_pen_rotations(arm, points, world_from_build, tilt, roll),
            }


def _preferred_position_edge(robot, offset, q0, q1, p0, p1, tolerance_m, maximum_step):
    """Radial targets are preferences; retain sampled position/branch checks.

    Later CellPlanner checks continuous source tracking and the full tool body
    against static and progressively deposited material, with orientation free.
    """
    if np.max(np.abs(q1 - q0)) > maximum_step:
        return None
    worst = 0.0
    for alpha in (0.25, 0.5, 0.75):
        target = p0.copy()
        target[:3, 3] = (1 - alpha) * p0[:3, 3] + alpha * p1[:3, 3]
        distance = np.linalg.norm(
            pose_error(robot.forward((1 - alpha) * q0 + alpha * q1, offset), target)[:3]
        )
        if distance > tolerance_m:
            return None
        worst = max(worst, float(distance))
    return worst


def inverse_axis_candidates(arm, target, seeds, position_tolerance=0.00005, axis_tolerance=0.001):
    """Preserve the nozzle direction while leaving rotation about it free.

    Axial pen geometry does not require a fixed roll. Fixing all three rotation
    coordinates can unnecessarily exhaust wrist travel around a closed wall.
    Returned states still require full robot/tool collision and path checks.
    """
    robot, offset = arm.robot, arm.flange_to_tcp
    accepted = []
    weights = np.array([1 / position_tolerance] * 3 + [1 / axis_tolerance] * 3)
    for seed in seeds:
        seed = np.clip(np.asarray(seed), robot.lower + 1e-9, robot.upper - 1e-9)

        def residual(q, reference=seed):
            tcp = robot.forward(q, offset)
            error = np.r_[tcp[:3, 3] - target[:3, 3], tcp[:3, 2] - target[:3, 2]]
            return np.r_[error * weights, 1e-4 * (q - reference)]

        def jacobian(q):
            geometric = geometric_jacobian(robot, q, offset)
            axis = robot.forward(q, offset)[:3, 2]
            axis_jacobian = np.cross(geometric[3:].T, axis).T
            return np.vstack(
                [
                    np.vstack([geometric[:3], axis_jacobian]) * weights[:, None],
                    1e-4 * np.eye(len(q)),
                ]
            )

        result = least_squares(
            residual,
            seed,
            jac=jacobian,
            bounds=(robot.lower, robot.upper),
            max_nfev=60,
            xtol=1e-9,
            ftol=1e-9,
            gtol=1e-8,
        )
        tcp = robot.forward(result.x, offset)
        if (
            np.linalg.norm(tcp[:3, 3] - target[:3, 3]) <= position_tolerance
            and np.linalg.norm(tcp[:3, 2] - target[:3, 2]) <= axis_tolerance
            and not any(np.linalg.norm(result.x - q) < 1e-3 for q in accepted)
        ):
            accepted.append(result.x)
    return accepted


def anchored_strokes(stroke, available):
    """Re-root closed contours, or reverse open contours, at existing anchors."""
    if stroke["closed"]:
        for index, visit in enumerate(stroke["junction_visits"][:-1]):
            if visit["node"] in available:
                for reverse in (False, True):
                    yield orient_stroke(stroke, reverse=reverse, start_visit=index)
    else:
        upward_first = stroke["segments"][0][0][2] > stroke["segments"][-1][-1][2]
        for reverse in (upward_first, not upward_first):
            start = stroke["end"] if reverse else stroke["start"]
            if start in available:
                yield orient_stroke(stroke, reverse=reverse)


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
    pen_tilt_deg=70.0,
    pen_tilt_candidates_deg=None,
    orientation_policy="fixed",
    ik_backend=None,
    rotations=None,
):
    """Yield actual IK paths, including every retained endpoint backtrace.

    Collision checks here cover self and static environment only. Every path is
    rechecked against the state-dependent material and other arm when scheduled.
    Default fixed orientation selects a 70/80-degree frame before the action and
    keeps its tilt, azimuth and roll unchanged throughout the whole path.
    Feasibility checks never split the source into separate pen actions.
    """
    settings = {**DEFAULT_SETTINGS, **(settings or {})}
    seeds = [arm.initial_positions] if seeds is None else seeds
    for oriented in anchored_strokes(stroke, available):
        points, _ = flatten_curve(
            oriented, settings["flatten_tolerance_m"], settings["maximum_chord_m"]
        )
        points = transform_points(points, world_from_build)
        # A stroke may name its own tilts, best first (detail drawn after the
        # frame knows which tilts clear it); otherwise the profile's apply.
        tilts = (
            stroke["pen_tilt_candidates_deg"]
            if stroke.get("pen_tilt_candidates_deg")
            else pen_tilt_candidates_deg
            if pen_tilt_candidates_deg is not None
            else tuple(dict.fromkeys([pen_tilt_deg, *DEFAULT_PEN_TILTS_DEG]))
        )
        if rotations is None and orientation_policy in {"fixed", "fixed_axis"}:
            orientations = list(fixed_rotation_families(arm, world_from_build, tilts, roll_angles))
        elif rotations is None and orientation_policy == "outward_radial":
            orientations = list(
                outward_rotation_families(arm, points, world_from_build, tilts, roll_angles)
            )
        elif rotations is None:
            orientations = [
                {
                    "rotations": pen_rotation(arm, world_from_build, roll, pen_tilt_deg),
                    "roll_rad": roll,
                    "pen_tilt_deg": pen_tilt_deg,
                    "orientation_policy": "fixed_axis",
                }
                for roll in roll_angles
            ]
        else:
            orientations = []
            for descriptor in rotations:
                if isinstance(descriptor, dict):
                    descriptor = dict(descriptor)
                    if descriptor.get("orientation_policy") == "outward_radial":
                        descriptor["rotations"] = radial_pen_rotations(
                            arm,
                            points,
                            world_from_build,
                            descriptor["pen_tilt_deg"],
                            descriptor.get("roll_rad", 0.0),
                        )
                    orientations.append(descriptor)
                else:
                    orientations.append(
                        {
                            "rotations": np.asarray(descriptor),
                            "roll_rad": 0.0,
                            "pen_tilt_deg": pen_tilt_deg,
                            "orientation_policy": "fixed_axis",
                        }
                    )
        for roll_index, descriptor in enumerate(orientations):
            if monotonic() >= deadline:
                return
            rotation = np.asarray(descriptor["rotations"])
            if orientation_policy in {"fixed", "fixed_axis"} and rotation.ndim != 2:
                raise ValueError("Fixed drawing orientation requires one constant TCP rotation")
            diagnostics = {
                "ik_calls": 0,
                "ik_solutions": 0,
                "static_rejections": 0,
                "tracking_rejections": 0,
                "clearance_rejections": 0,
            }
            varying = rotation.ndim == 3
            targets = np.repeat(np.eye(4)[None], len(points), axis=0)
            targets[:, :3, :3], targets[:, :3, 3] = rotation, points
            gpu_solutions = (
                ik_backend.solve(targets, reference=seeds[0]) if ik_backend is not None else None
            )
            gpu_by_target = (
                {
                    target.tobytes(): rows
                    for target, rows in zip(targets, gpu_solutions, strict=True)
                }
                if gpu_solutions is not None
                else None
            )

            def solve(
                target,
                previous,
                gpu_cache=gpu_by_target,
                diagnostics=diagnostics,
                varying_orientation=varying,
            ):
                if monotonic() >= deadline:
                    return []
                diagnostics["ik_calls"] += 1
                if gpu_cache is not None:
                    positions = gpu_cache[target.tobytes()]
                elif settings.get("free_roll", False) and varying_orientation:
                    positions = inverse_axis_candidates(
                        arm,
                        target,
                        previous,
                        settings["ik_position_tolerance_m"],
                        settings["ik_orientation_tolerance_rad"],
                    )
                else:
                    positions = [
                        s["positions"]
                        for s in inverse_candidates(
                            arm.robot,
                            target,
                            arm.flange_to_tcp,
                            previous,
                            position_tolerance=settings["ik_position_tolerance_m"],
                            orientation_tolerance=settings["ik_orientation_tolerance_rad"],
                            maximum_evaluations=60,
                        )
                    ]
                diagnostics["ik_solutions"] += len(positions)
                diagnostics["last_target_ik_solutions"] = len(positions)
                accepted = []
                for position in positions:
                    contacts = arm.collision.check(position, obstacles, clearance_m)
                    if contacts:
                        diagnostics["static_rejections"] += 1
                        diagnostics["last_static_contacts"] = contacts
                    else:
                        accepted.append(position)
                diagnostics["last_target_collision_free_solutions"] = len(accepted)
                return accepted

            def edge_check(q0, q1, p0, p1, varying_orientation=varying, diagnostics=diagnostics):
                edge_tracker = _preferred_position_edge if varying_orientation else tracking_edge
                error = edge_tracker(
                    arm.robot,
                    arm.flange_to_tcp,
                    q0,
                    q1,
                    p0,
                    p1,
                    settings["interpolation_tolerance_m"],
                    *([] if varying_orientation else [settings["interpolation_tolerance_rad"]]),
                    settings["maximum_joint_step_rad"],
                )
                if error is None:
                    diagnostics["tracking_rejections"] += 1
                    diagnostics["last_tracking_failure"] = {
                        "maximum_joint_step_rad": float(np.max(np.abs(np.asarray(q1) - q0))),
                        "from_q": np.asarray(q0).tolist(),
                        "to_q": np.asarray(q1).tolist(),
                        "from_tcp_m": p0[:3, 3].tolist(),
                        "to_tcp_m": p1[:3, 3].tolist(),
                    }
                    return None
                check = arm.collision.check_edge(
                    q0,
                    q1,
                    obstacles,
                    clearance_m,
                    settings["maximum_subdivision_depth"],
                    deadline=deadline,
                )
                if check["status"] != "CERTIFIED_MODELED_EDGE":
                    diagnostics["clearance_rejections"] += 1
                    diagnostics["last_clearance_failure"] = check
                    return None
                return error

            result = search_layers(
                targets,
                solve,
                edge_check,
                seeds,
                settings["beam_width"],
                deadline,
            )
            if gpu_by_target is not None and result["status"] != "SAMPLED_DRAWING_PATH":
                # Local seeded IK restores continuity when independent batched
                # solutions choose different redundancy branches.
                result = search_layers(
                    targets,
                    lambda target, previous: solve(target, previous, None),
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
                "roll_rad": float(descriptor.get("roll_rad", 0.0)),
                "pen_tilt_deg": float(descriptor.get("pen_tilt_deg", pen_tilt_deg)),
                "orientation_policy": descriptor.get("orientation_policy", "fixed_axis"),
                "fixed_orientation": not varying,
                "tcp_rotation": (rotation[0] if varying else rotation).tolist(),
                **(
                    {"tcp_rotations": rotation.tolist(), "free_tool_orientation": True}
                    if varying
                    else {"free_tool_orientation": False}
                ),
                "tcp_targets_m": points.tolist(),
                "sampling_diagnostics": diagnostics,
                "free_roll": bool(settings.get("free_roll", False) and varying),
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
                pen_tilt_deg=profile.get("drawing_pen_tilt_deg", 70.0),
                pen_tilt_candidates_deg=profile.get("drawing_pen_tilt_candidates_deg"),
                orientation_policy=profile.get("drawing_orientation_policy", "fixed"),
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
