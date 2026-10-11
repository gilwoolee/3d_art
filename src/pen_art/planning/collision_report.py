"""Audit drawing paths against URDF self-collision and static work surfaces.

The interval certificate applies to piecewise linear JOINT interpolation, with
the stated numerical clearance reserve. Tip contact, growing material,
transits, controller interpolation, and physical tracking remain separate.
"""

import argparse
import hashlib
import json
from pathlib import Path
from time import monotonic

import numpy as np
import yaml

from ..io import artifact, read_json, sha256, stage_update, write_json
from ..robot import SerialArm
from .collision import RobotCollisionModel, work_surfaces
from .tool import load_tool
from .tracking import tracking_bounds


def audit_drawing_paths(run, config_path=Path("configs/collision.yaml"), maximum_seconds=300):
    run, config_path = Path(run), Path(config_path)
    config = yaml.safe_load(config_path.read_text())
    if maximum_seconds <= 0:
        raise ValueError("maximum_seconds must be positive")
    # Atomic source read: a running IK process may replace its JSON concurrently.
    raw = (run / "drawing_motion.json").read_bytes()
    motion = json.loads(raw)
    for filename, expected in motion["inputs"].items():
        if sha256(run / filename) != expected:
            raise ValueError(f"Motion input is stale: {filename}")
    placement = read_json(run / "placement_search.json")
    asset = read_json(run / "asset_manifest.json")
    robot = SerialArm(run / "robot.urdf")
    tool = load_tool(
        config.get("tool_geometry"), config_path.parent, robot, motion["flange_to_tcp"]
    )
    model = RobotCollisionModel(robot, tool.bodies)
    selected = placement["selected"]
    size = np.subtract(asset["bounds_m"][1], asset["bounds_m"][0])
    width, depth = size[:2] + config["platform_padding_m"]
    obstacles = work_surfaces(
        selected["center_xy_m"],
        selected["height_above_table_m"],
        width,
        depth,
        placement["assumptions"]["table_z_m"],
        config["table_size_m"],
    )
    output = artifact(
        "drawing_collision_audit",
        status="RUNNING",
        frame="robot_base",
        units="m",
        angle_units="rad",
        simulation_only=True,
        source_motion_sha256=hashlib.sha256(raw).hexdigest(),
        inputs={**motion["inputs"], "asset_manifest.json": sha256(run / "asset_manifest.json")},
        config_sha256=sha256(config_path),
        config=config,
        tool_model=tool.metadata,
        interpolation="piecewise_linear_joint_positions",
        tracking_reference="linear Cartesian segments with constant target orientation",
        model_scope=["nonadjacent_URDF_links", "table_box", "raised_platform_box"]
        + (["configured_noncontact_pen_and_mount"] if tool.bodies else []),
        mesh_model="filled_per_link_convex_hulls",
        excluded_mount_links=sorted(model.mount_links),
        numerical_reserve_m=1e-6,
        platform={
            "center_xy_m": selected["center_xy_m"],
            "height_m": selected["height_above_table_m"],
            "width_m": float(width),
            "depth_m": float(depth),
        },
        requested_strokes=motion["requested_strokes"],
        strokes=[],
        limitations=[
            "Exact robot variant and work-cell dimensions remain unconfirmed",
            "Contact tip and growing sculpture are not modeled in this drawing audit",
            "Cable and filament are included only if explicitly supplied as tool bodies"
            if tool.bodies
            else "Pen, mount, cable and filament are not modeled",
            "Adjacent robot links and the fixed mounting assembly/table pair are excluded",
            "No approach, withdrawal, inter-stroke travel or controller tracking validation",
            "Certificate uses rigid meshes and linear joint interpolation only",
            "Per-link convex hulls conservatively fill concave regions of URDF meshes",
        ],
    )
    deadline = monotonic() + maximum_seconds
    for path in motion["strokes"]:
        if monotonic() >= deadline:
            break
        if path["status"] != "SAMPLED_DRAWING_PATH":
            continue
        q = path["positions"]
        targets = np.repeat(np.eye(4)[None], len(q), axis=0)
        targets[:, :3, 3] = path["tcp_targets_m"]
        targets[:, :3, :3] = path["tcp_rotation"]
        row = {
            "id": path["id"],
            "source_stroke": path["source_stroke"],
            "status": "CERTIFIED_MODELED_PATH",
            "edges_checked": 0,
            "midpoint_evaluations": 0,
            "witnesses": [],
            "tracking_status": "BOUNDED",
            "position_error_bound_m": 0.0,
            "orientation_error_bound_rad": 0.0,
        }
        for index in range(max(1, len(q) - 1)):
            if monotonic() >= deadline:
                row["status"] = "TIME_BUDGET"
                break
            result = model.check_edge(
                q[index],
                q[min(index + 1, len(q) - 1)],
                obstacles,
                config["clearance_m"],
                config["maximum_subdivision_depth"],
                deadline=deadline,
            )
            row["edges_checked"] += 1
            row["midpoint_evaluations"] += result["evaluations"]
            bounds = tracking_bounds(
                robot,
                motion["flange_to_tcp"],
                q[index],
                q[min(index + 1, len(q) - 1)],
                targets[index],
                targets[min(index + 1, len(q) - 1)],
            )
            for key, value in bounds.items():
                row[key] = max(row[key], value)
            if (
                bounds["position_error_bound_m"] > motion["settings"]["interpolation_tolerance_m"]
                or bounds["orientation_error_bound_rad"]
                > motion["settings"]["interpolation_tolerance_rad"]
            ):
                row["tracking_status"] = "BOUND_EXCEEDS_TOLERANCE"
            if result["status"] != "CERTIFIED_MODELED_EDGE":
                row["status"] = result["status"]
                row["witnesses"].append({"edge_index": index, **result})
                break
        if row["edges_checked"] != max(1, len(q) - 1):
            row["tracking_status"] = "PARTIAL_CHECK"
        output["strokes"].append(row)
        write_json(run / "drawing_collisions.json", output)
        print(
            f"{len(output['strokes'])}/{motion['requested_strokes']}: {row['status']}", flush=True
        )
    rows = output["strokes"]
    output["counts"] = {
        status: sum(row["status"] == status for row in rows)
        for status in ["CERTIFIED_MODELED_PATH", "COLLISION", "CLEARANCE_UNRESOLVED", "TIME_BUDGET"]
    }
    output["tracking_counts"] = {
        status: sum(row["tracking_status"] == status for row in rows)
        for status in ["BOUNDED", "BOUND_EXCEEDS_TOLERANCE", "PARTIAL_CHECK"]
    }
    output["status"] = (
        "COMPLETE_ARM_STATIC_AUDIT"
        if len(rows) == motion["requested_strokes"] and not output["counts"]["TIME_BUDGET"]
        else "PARTIAL_ARM_STATIC_AUDIT"
    )
    write_json(run / "drawing_collisions.json", output)
    if (run / "run.json").is_file() and output["status"] == "COMPLETE_ARM_STATIC_AUDIT":
        stage_update(
            run,
            "motion",
            "partial",
            artifact="drawing_motion.json",
            collision_artifact="drawing_collisions.json",
            message=f"{len(rows)} drawing paths audited for modeled arm/static clearance; "
            "drawing contact, growing material and transits remain",
        )
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, default=Path("runs/palace_15cm"))
    parser.add_argument("--config", type=Path, default=Path("configs/collision.yaml"))
    parser.add_argument("--maximum-seconds", type=float, default=300)
    args = parser.parse_args()
    audit_drawing_paths(args.run, args.config, args.maximum_seconds)
