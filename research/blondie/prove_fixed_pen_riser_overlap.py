"""Conservative all-azimuth barrel/riser overlap proof at existing zigzag valleys."""

import hashlib
import json
from pathlib import Path

import fcl
import numpy as np

from pen_art.cell import load_cell
from pen_art.curves import transform_points
from pen_art.io import read_json, sha256, write_json
from pen_art.planning.collision import placed

output = Path("runs/fixed_pen_orientation_proof_v1")
output.mkdir(parents=True, exist_ok=True)
rows = []
for obj in ("cube", "bottle"):
    run = Path(f"runs/{obj}_fixed70_phase1_v1")
    profile_path = run / "profile.yaml"
    profile, arms, obstacles = load_cell(profile_path)
    arm = arms["arm_1"]
    old_profile, _, _ = load_cell(Path(f"runs/{obj}_taper70_phase1_v1/profile.yaml"))
    geometry = {k: profile[k] for k in ("world_from_build", "environment")}
    geometry["tool_geometry"] = profile["arms"][0]["tool_geometry"]
    old_geometry = {k: old_profile[k] for k in ("world_from_build", "environment")}
    old_geometry["tool_geometry"] = old_profile["arms"][0]["tool_geometry"]
    assert geometry == old_geometry
    geometry_hash = hashlib.sha256(json.dumps(geometry, sort_keys=True).encode()).hexdigest()
    action = next(
        a
        for a in read_json(run / "construction_plan.json")["actions"]
        if a.get("parent_curve", "").startswith("zigzag-")
    )
    endpoints = np.unique(np.array([p for s in action["segments"] for p in (s[0], s[-1])]), axis=0)
    local = endpoints[np.isclose(endpoints[:, 2], 0.0, atol=1e-10)]
    points = transform_points(local, profile["world_from_build"])
    riser = profile["environment"]["build_riser"]
    center = np.array(profile["world_from_build"])[:3, 3]
    barrel = next(b for b in arm.collision.bodies if b.name == "tool/pen_body")
    tcp_to_body = np.linalg.inv(arm.flange_to_tcp) @ barrel.origin
    radius = float(barrel.geometry.radius)
    length = float(barrel.geometry.lz)
    near = -tcp_to_body[2, 3] - length / 2
    longitudinal = near + 0.0001
    radial = radius - 0.0001
    step = 2 * np.pi / 3600
    angles = np.arange(3600) * step
    unit = np.column_stack([np.cos(angles), np.sin(angles)])
    for tilt in (70.0, 80.0):
        t = np.deg2rad(tilt)
        horizontal = longitudinal * np.sin(t) + radial * np.cos(t)
        below = longitudinal * np.cos(t) - radial * np.sin(t)
        point_radius = float(np.hypot(longitudinal, radial))
        rotational_displacement = float(2 * point_radius * np.sin(0.003 / 2))
        total_displacement = 0.0003 + rotational_displacement
        probes = points[None, :, :2] - center[:2] + horizontal * unit[:, None, :]
        if riser["type"] == "cylinder":
            inside = riser["radius_m"] - np.linalg.norm(probes, axis=2)
        else:
            inside = np.min(np.array(riser["size_m"]) / 2 - np.abs(probes), axis=2)
        best = np.max(inside, axis=1)
        ai = int(np.argmin(best))
        vi = int(np.argmax(inside[ai]))
        lower = float(best[ai] - horizontal * step / 2 - 1e-12)
        # Signed distance to a box/circle is 1-Lipschitz. A probe traces a circle
        # with speed H, so the nearest azimuth-grid witness loses at most H*dtheta/2.
        # Positive lower bound proves some existing valley has the probe inside
        # the riser at every azimuth, including between tested directions.
        body_axis = np.r_[np.sin(t) * unit[ai], np.cos(t)]
        tcp = np.eye(4)
        z = -body_axis
        x = np.array([0.0, 0.0, 1.0])
        x -= x @ z * z
        x /= np.linalg.norm(x)
        tcp[:3, :3] = np.column_stack([x, np.cross(z, x), z])
        tcp[:3, 3] = points[vi]
        shape = placed(barrel.geometry, tcp @ tcp_to_body)
        result = fcl.CollisionResult()
        overlap = fcl.collide(
            shape,
            obstacles["build_riser"],
            fcl.CollisionRequest(enable_contact=True, num_max_contacts=8),
            result,
        )
        actual_probe = np.r_[center[:2] + probes[ai, vi], points[vi, 2] + below]
        rows.append(
            {
                "object": obj,
                "profile_sha256": sha256(profile_path),
                "source_sha256": sha256(run / "construction_plan.json"),
                "section": action["id"],
                "profile_path": str(profile_path),
                "riser_geometry": riser,
                "world_from_build": profile["world_from_build"],
                "valley_count": len(points),
                "tilt_deg": tilt,
                "pen_riser_geometry_sha256": geometry_hash,
                "same_pen_riser_geometry_as_prior_radial_profile": True,
                "fixed_profile_orientation_policy": profile["drawing_orientation_policy"],
                "orientation_tolerance_rad": 0.003,
                "tracking_tolerance_m": 0.0003,
                "rotation_displacement_bound_m": rotational_displacement,
                "combined_pose_displacement_bound_m": total_displacement,
                "barrel_radius_m": radius,
                "barrel_length_m": length,
                "barrel_front_setback_m": near,
                "interior_probe_axis_distance_from_tcp_m": longitudinal,
                "interior_probe_radial_distance_m": radial,
                "probe_height_relative_to_tip_m": below,
                "probe_horizontal_offset_m": horizontal,
                "azimuth_grid_step_deg": 0.1,
                "lipschitz_loss_m": horizontal * step / 2,
                "all_azimuth_interior_footprint_bound_m": lower,
                "footprint_bound_after_tracking_and_rotation_m": lower - total_displacement,
                "vertical_overlap_after_tracking_and_rotation_m": float(
                    -below - total_displacement
                ),
                "probe_inside_riser_height": bool(-0.02 < below < 0),
                "all_azimuth_all_roll_barrel_collision_proved": bool(
                    lower > total_displacement
                    and -0.02 + total_displacement < below < -total_displacement
                ),
                "proof": (
                    "For each azimuth choose one existing bottom valley. Probe lies strictly "
                    "inside cylindrical barrel, independently of roll. Convex-footprint signed "
                    "clearance is 1-Lipschitz; H*dtheta/2 bridges all azimuth grid gaps. "
                    "The pose-error displacement bound includes 0.3mm TCP tracking and "
                    "0.003rad rotation. The displaced probe remains below top and above "
                    "bottom of solid riser, and inside its footprint. Thus every constant "
                    "orientation at this tilt collides at some required valley."
                ),
                "representative": {
                    "body_outward_azimuth_deg": float(np.degrees(angles[ai])),
                    "source_valley_m": local[vi].tolist(),
                    "tcp_world_m": points[vi].tolist(),
                    "interior_probe_world_m": actual_probe.tolist(),
                    "fcl_collision": bool(overlap),
                },
            }
        )
write_json(
    output / "proof.json",
    {
        "script_sha256": sha256(Path(__file__)),
        "scope": (
            "Geometry obstruction for the current solid object-footprint riser, "
            "actual barrel, and complete first zigzag; no robot reach assumptions."
        ),
        "rows": rows,
    },
)
for row in rows:
    print(
        row["object"],
        row["tilt_deg"],
        "PROVED",
        row["all_azimuth_all_roll_barrel_collision_proved"],
        "inside_bound_mm",
        row["all_azimuth_interior_footprint_bound_m"] * 1000,
        "probe_below_tip_mm",
        row["probe_height_relative_to_tip_m"] * 1000,
        "FCL",
        row["representative"]["fcl_collision"],
    )

assert all(
    row["all_azimuth_all_roll_barrel_collision_proved"] and row["representative"]["fcl_collision"]
    for row in rows
)
