"""Coupled order repair using complete approach/draw/withdraw robot transitions."""

from pathlib import Path
from time import monotonic

import fcl
import numpy as np
import yaml

from ..io import artifact, read_json, sha256, stage_update, write_json
from ..kinematics import inverse_candidates, rigid_transform
from ..robot import SerialArm
from .collision import Body, RobotCollisionModel, work_surfaces
from .rrt import connect
from .scene import MaterialScene
from .sequence import search_order
from .tool import load_tool, require_matching_tool
from .transit import (
    completed_prefix,
    contact_validator,
    find_departure,
    local_contact_surfaces,
    sampled_operation,
)


def plan_sequence(
    run,
    transit_config=Path("configs/transit.yaml"),
    collision_config=Path("configs/collision.yaml"),
    maximum_seconds=300,
    resume=False,
):
    run = Path(run)
    settings = yaml.safe_load(Path(transit_config).read_text())
    collision = yaml.safe_load(Path(collision_config).read_text())
    positive = [maximum_seconds] + [
        settings[key]
        for key in (
            "withdraw_distance_m",
            "parking_clearance_m",
            "lift_sample_m",
            "preview_joint_step_rad",
            "virtual_tip_radius_m",
            "contact_radius_m",
            "lift_position_tolerance_m",
            "lift_orientation_tolerance_rad",
            "rrt_step_rad",
            "rrt_maximum_nodes",
            "rrt_maximum_seconds",
        )
    ]
    if not np.isfinite(positive).all() or min(positive) <= 0:
        raise ValueError("Transit distances, radius, resolution and budget must be positive")
    state_budget = settings.get("order_search_maximum_states", 2000)
    if not isinstance(state_budget, int) or isinstance(state_budget, bool) or state_budget < 1:
        raise ValueError("The order-search state budget must be a positive integer")
    directions = np.asarray(settings["departure_directions"], dtype=float)
    if (
        directions.ndim != 2
        or directions.shape[1] != 3
        or not len(directions)
        or not np.isfinite(directions).all()
        or np.any(np.linalg.norm(directions, axis=1) == 0)
    ):
        raise ValueError("Departure directions must be finite nonzero 3D vectors")
    motion = read_json(run / "drawing_motion.json")
    audit = read_json(run / "drawing_collisions.json")
    if audit["config"] != collision:
        raise ValueError("Drawing collision settings changed; rerun the audit")
    if audit["source_motion_sha256"] != sha256(run / "drawing_motion.json"):
        raise ValueError("Drawing collision audit is stale")
    if motion["status"] != "SAMPLED_DRAWING_PATHS":
        raise ValueError("Complete per-stroke drawing paths are required")
    if audit["counts"]["CERTIFIED_MODELED_PATH"] != len(motion["strokes"]) or audit[
        "tracking_counts"
    ]["BOUNDED"] != len(motion["strokes"]):
        raise ValueError("Drawing paths must pass the current collision and tracking audit")
    for filename, expected in motion["inputs"].items():
        if sha256(run / filename) != expected:
            raise ValueError(f"Stale drawing input: {filename}")
    robot = SerialArm(run / "robot.urdf")
    offset = rigid_transform(motion["flange_to_tcp"])
    tool = load_tool(collision.get("tool_geometry"), Path(collision_config).parent, robot, offset)
    require_matching_tool(tool, audit)
    tip_radius = tool.tip_radius_m or settings["virtual_tip_radius_m"]
    arm = RobotCollisionModel(robot, tool.bodies)
    tip = Body(
        "virtual_tip",
        robot.tip,
        offset,
        fcl.Sphere(tip_radius),
        tip_radius,
    )
    traveling_arm = RobotCollisionModel(robot, [*tool.bodies, tip])
    placement = read_json(run / "placement_search.json")
    selected = placement["selected"]
    asset = read_json(run / "asset_manifest.json")
    size = np.subtract(asset["bounds_m"][1], asset["bounds_m"][0])
    width, depth = size[:2] + collision["platform_padding_m"]
    obstacles = work_surfaces(
        selected["center_xy_m"],
        selected["height_above_table_m"],
        width,
        depth,
        placement["assumptions"]["table_z_m"],
        collision["table_size_m"],
    )
    deadline = monotonic() + maximum_seconds
    reference = np.array(read_json(run / "robot_manifest.json")["initial_positions"])
    output = artifact(
        "sequence_motion_preview",
        status="RUNNING",
        frame="robot_base",
        units="m",
        angle_units="rad",
        simulation_only=True,
        settings=settings,
        collision_settings=collision,
        tool_model=tool.metadata,
        effective_tip_radius_m=tip_radius,
        flange_to_tcp=offset.tolist(),
        inputs={
            name: sha256(run / name)
            for name in [
                "drawing_motion.json",
                "drawing_collisions.json",
                "robot.urdf",
                "robot_manifest.json",
                "construction_plan.json",
                "placement_search.json",
                "asset_manifest.json",
                "fabrication_config.json",
            ]
        },
        operations=[],
        completed_strokes=0,
        requested_strokes=len(motion["strokes"]),
        failures=[],
        material_segments=0,
        limitations=[
            "Provisional robot variant, mount transform, work-cell and lift distance",
            "Configured non-contact pen/mount geometry is checked; tip remains a sphere"
            if tool.bodies
            else "Pen body and mount are not modeled; free travel checks an idealized tip sphere",
            "Drawing does not validate tip contact; lifts use a provisional local contact ball",
            "Order search uses fixed drawing paths; drawing-pose alternatives remain unsearched",
            "Material is rigid capsules, including a conservative full depositing segment",
            "No extrusion timing, cooling, physical tracking or structural acceptance",
            "Robot must be placed at the planned parking pose before this preview starts",
        ],
    )
    actions = {row["id"]: row for row in read_json(run / "construction_plan.json")["actions"]}
    complete_material = MaterialScene()
    for stroke in motion["strokes"]:
        radius = actions[stroke["id"]]["diameter_m"] / 2 + motion["settings"]["flatten_tolerance_m"]
        for piece, (a, b) in enumerate(
            zip(stroke["tcp_targets_m"][:-1], stroke["tcp_targets_m"][1:], strict=True)
        ):
            complete_material.add(a, b, radius, f"material/{stroke['id']}/{piece}")
    parking = np.eye(4)
    parking[:3, :3] = placement["assumptions"]["tcp_rotation"]
    center = np.mean(asset["bounds_m"], axis=0)
    center[2] = asset["bounds_m"][1][2] + settings["parking_clearance_m"]
    build_to_robot = np.array(selected["build_to_robot_base"])
    parking[:3, 3] = (build_to_robot @ np.r_[center, 1])[:3]
    solutions = inverse_candidates(
        robot, parking, offset, [reference, np.array(motion["strokes"][0]["positions"][0])]
    )
    safe = [
        row
        for row in solutions
        if not traveling_arm.check(
            row["positions"], obstacles, collision["clearance_m"], complete_material
        )
    ]
    if not safe:
        output["status"] = "NO_PARKING_POSE_FOUND"
        output["failures"].append(
            {"phase": "PARKING", "target": parking.tolist(), "ik_solutions": len(solutions)}
        )
        write_json(run / "sequence_motion.json", output)
        return output
    initial = np.array(
        min(safe, key=lambda row: np.linalg.norm(np.array(row["positions"]) - reference))[
            "positions"
        ]
    )
    output["parking_pose"] = {
        "positions": initial.tolist(),
        "tcp_pose": parking.tolist(),
        "checked_against_complete_material": True,
    }
    graph = read_json(run / "stroke_graph.json")
    construction = read_json(run / "construction_plan.json")
    if construction["provenance"]["graph_sha256"] != sha256(run / "stroke_graph.json"):
        raise ValueError("Construction graph changed; rebuild the construction preview")
    output["inputs"]["stroke_graph.json"] = sha256(run / "stroke_graph.json")
    output["algorithm"] = "robot_transition_backtracking_from_regression_order"
    output["planner_revision"] = 2
    output["search"] = {"status": "RUNNING", "attempts": 0, "backtracks": 0}
    by_id = {stroke["id"]: stroke for stroke in motion["strokes"]}
    order = list(by_id)
    ground = frozenset(node["id"] for node in graph["nodes"] if node["ground"])
    vertices = {
        identity: frozenset(v["node"] for v in action["junction_visits"])
        for identity, action in actions.items()
    }

    def add_stroke(scene, identity):
        stroke = by_id[identity]
        radius = actions[identity]["diameter_m"] / 2 + motion["settings"]["flatten_tolerance_m"]
        for piece, (a, b) in enumerate(
            zip(stroke["tcp_targets_m"][:-1], stroke["tcp_targets_m"][1:], strict=True)
        ):
            scene.add(a, b, radius, f"material/{identity}/{piece}")
        return radius

    def state_key(state, identity):
        # The order identifies the deposited geometry; q distinguishes the robot
        # configuration. This cache reuses only verified saved prefix transitions.
        return state["order"], state["q"].tobytes(), identity

    seed_operations = []
    if resume and (run / "sequence_motion.json").is_file():
        previous = read_json(run / "sequence_motion.json")
        require_matching_tool(tool, previous)
        if (
            any(output["inputs"].get(k) != v for k, v in previous["inputs"].items())
            or not set(output["inputs"]) - {"stroke_graph.json"} <= set(previous["inputs"])
            or previous["settings"] != settings
            or previous["collision_settings"] != collision
        ):
            raise ValueError("Sequence resume inputs or settings changed")
        seed_operations, _ = completed_prefix(previous["operations"], order, allow_reordered=True)
        initial = np.array(previous.get("parking_pose", output["parking_pose"])["positions"])
        if traveling_arm.check(initial, obstacles, collision["clearance_m"], complete_material):
            raise ValueError("Saved parking pose is no longer valid")
        output["parking_pose"] = previous.get("parking_pose", output["parking_pose"])

    root = {
        "q": initial,
        "material": MaterialScene(),
        "order": (),
        "nodes": ground,
        "operations": [],
    }
    seed_cache = {}
    state = root
    for start in range(0, len(seed_operations), 4):
        group = seed_operations[start : start + 4]
        identity = group[0]["stroke_id"]
        if actions[identity]["start"] not in state["nodes"]:
            raise ValueError("Saved prefix starts a stroke without a material anchor")
        if not np.allclose(state["q"], group[0]["positions"][0], atol=1e-9, rtol=0):
            raise ValueError("Saved prefix does not start at its parking pose")
        scene = state["material"].copy()
        add_stroke(scene, identity)
        next_state = {
            "q": np.array(group[-1]["positions"][-1]),
            "material": scene,
            "order": (*state["order"], identity),
            "nodes": state["nodes"] | vertices[identity],
            "operations": state["operations"] + group,
        }
        seed_cache[state_key(state, identity)] = next_state
        state = next_state
    output["resumed_from_stroke"] = len(state["order"])

    def valid_edge(scene, a, b):
        return (
            traveling_arm.check_edge(
                a,
                b,
                obstacles,
                collision["clearance_m"],
                collision["maximum_subdivision_depth"],
                scene,
                deadline=deadline,
            )["status"]
            == "CERTIFIED_MODELED_EDGE"
        )

    def lift_validator(scene, q):
        contact = robot.forward(q, offset)[:3, 3]
        top = placement["assumptions"]["table_z_m"] + selected["height_above_table_m"]
        surfaces = local_contact_surfaces(
            contact[2], placement["assumptions"]["table_z_m"], top, settings["contact_radius_m"]
        )
        return contact_validator(
            robot,
            arm,
            traveling_arm,
            offset,
            scene,
            obstacles,
            contact,
            settings["contact_radius_m"],
            tip_radius,
            collision["clearance_m"],
            collision["maximum_subdivision_depth"],
            surfaces,
            deadline=deadline,
        )

    def transit(scene, a, b, seed):
        return connect(
            a,
            b,
            robot.lower,
            robot.upper,
            lambda q0, q1: valid_edge(scene, q0, q1),
            seed=seed,
            step_rad=settings["rrt_step_rad"],
            maximum_nodes=settings["rrt_maximum_nodes"],
            maximum_seconds=min(
                settings["rrt_maximum_seconds"], max(0.001, deadline - monotonic())
            ),
        )

    def transition(state, identity, attempt):
        cached = seed_cache.get(state_key(state, identity))
        if cached is not None:
            return cached, None
        stroke = by_id[identity]
        before = state["material"]
        start, end = np.array(stroke["positions"][0]), np.array(stroke["positions"][-1])
        start_validator = lift_validator(before, start)
        approach_attempts = []
        # A clear departure is insufficient if the arm cannot reach its lifted
        # endpoint. Try each direction together with its connecting transit.
        for direction_index, direction in enumerate(settings["departure_directions"]):
            if monotonic() >= deadline:
                return None, {"stroke_id": identity, "phase": "APPROACH", "status": "TIME_BUDGET"}
            local_settings = {**settings, "departure_directions": [direction]}
            lift = find_departure(robot, offset, start, local_settings, start_validator, deadline)
            approach_attempts.extend(lift["attempts"])
            detail = lift["attempts"][-1]
            if lift["status"] != "SAMPLED_DRAWING_PATH":
                continue
            approach = list(reversed(lift["positions"]))
            travel = transit(
                before, state["q"], approach[0], settings["seed"] + attempt * 100 + direction_index
            )
            detail["travel_status"] = travel["status"]
            if travel["status"] == "CONNECTED":
                break
        else:
            return None, {
                "stroke_id": identity,
                "phase": "APPROACH_AND_TRAVEL",
                "status": "NO_CONNECTED_APPROACH",
                "attempts": approach_attempts,
            }

        scene = before.copy()
        radius = actions[identity]["diameter_m"] / 2 + motion["settings"]["flatten_tolerance_m"]
        points, q = stroke["tcp_targets_m"], stroke["positions"]
        for piece in range(len(points) - 1):
            if monotonic() >= deadline:
                return None, {"stroke_id": identity, "phase": "DRAW", "status": "TIME_BUDGET"}
            scene.add(points[piece], points[piece + 1], radius, f"material/{identity}/{piece}")
            check = arm.check_edge(
                q[piece],
                q[piece + 1],
                obstacles,
                collision["clearance_m"],
                collision["maximum_subdivision_depth"],
                scene,
                deadline=deadline,
            )
            if check["status"] != "CERTIFIED_MODELED_EDGE":
                return None, {"stroke_id": identity, "phase": "DRAW", "piece": piece, **check}
        withdraw = find_departure(
            robot, offset, end, settings, lift_validator(scene, end), deadline
        )
        if withdraw["status"] != "SAMPLED_DRAWING_PATH":
            return None, {"stroke_id": identity, "phase": "WITHDRAW", **withdraw}
        operations = []

        def append(kind, positions, **extra):
            operations.append(
                sampled_operation(
                    kind,
                    positions,
                    robot,
                    offset,
                    settings["preview_joint_step_rad"],
                    id=f"operation-{len(state['operations']) + len(operations):04}",
                    stroke_id=identity,
                    completed_strokes=len(state["order"]) + (kind == "WITHDRAW"),
                    **extra,
                )
            )

        append(
            "TRAVEL",
            travel["positions"],
            method=travel["method"],
            checked_edges=travel["checked_edges"],
        )
        append(
            "APPROACH",
            approach,
            departure_direction=lift["direction"],
            departure_attempts=approach_attempts,
        )
        append("DRAW", q, material_radius_m=radius)
        append(
            "WITHDRAW",
            withdraw["positions"],
            departure_direction=withdraw["direction"],
            departure_attempts=withdraw["attempts"],
        )
        return {
            "q": np.array(withdraw["positions"][-1]),
            "material": scene,
            "order": (*state["order"], identity),
            "nodes": state["nodes"] | vertices[identity],
            "operations": state["operations"] + operations,
        }, None

    def finish(state):
        result = transit(state["material"], state["q"], initial, settings["seed"] + 10000)
        if result["status"] != "CONNECTED":
            return None, {"phase": "RETURN", **result}
        operation = sampled_operation(
            "RETURN",
            result["positions"],
            robot,
            offset,
            settings["preview_joint_step_rad"],
            id=f"operation-{len(state['operations']):04}",
            stroke_id=None,
            completed_strokes=len(state["order"]),
            method=result["method"],
        )
        return {**state, "q": initial, "operations": state["operations"] + [operation]}, None

    def save(state, order, attempts, backtracks):
        output["operations"] = state["operations"]
        output["completed_strokes"] = len(order)
        output["material_segments"] = len(state["material"].segments)
        output["drawing_order"] = list(order)
        output["search"].update(attempts=attempts, backtracks=backtracks)
        output["total_tcp_length_m"] = sum(op["tcp_length_m"] for op in state["operations"])
        write_json(run / "sequence_motion.json", output)

    def observe(state, order, attempts, backtracks):
        save(state, order, attempts, backtracks)
        print(f"{len(order)}/{len(by_id)}: connected prefix; {backtracks} backtracks", flush=True)

    last_report = monotonic()

    def progress(order, attempts, backtracks, failure):
        nonlocal last_report
        output["search"].update(attempts=attempts, backtracks=backtracks, current_depth=len(order))
        if failure:
            output["search"]["last_rejection"] = {
                key: failure.get(key) for key in ("action", "phase", "status")
            }
        if monotonic() - last_report >= 5:
            write_json(run / "sequence_motion.json", output)
            stage_update(
                run,
                "motion",
                "running",
                message=(
                    f"Order search: {output['completed_strokes']} complete strokes; "
                    f"{attempts} transitions tried; {backtracks} backtracks"
                ),
            )
            last_report = monotonic()

    stage_update(
        run, "motion", "running", message="Searching construction orders and robot transits"
    )
    result = search_order(
        order,
        root,
        lambda state, identity: actions[identity]["start"] in state["nodes"],
        transition,
        finish,
        maximum_states=state_budget,
        deadline=deadline,
        observe=observe,
        progress=progress,
    )
    output["status"] = (
        "GEOMETRIC_SEQUENCE_PREVIEW" if result["status"] == "FOUND" else result["status"]
    )
    output["search"]["status"] = result["status"]
    output["search_failures"] = result["failures"]
    output["failures"] = (
        [result["last_failure"]] if result["status"] != "FOUND" and result["last_failure"] else []
    )
    save(result["state"], result["order"], result["attempts"], result["backtracks"])
    stage_update(
        run,
        "motion",
        "partial",
        artifact="sequence_motion.json",
        message=f"{output['completed_strokes']}/{len(by_id)} strokes; "
        f"{result['backtracks']} order backtracks; full pen and physical timing pending",
    )
    return output
