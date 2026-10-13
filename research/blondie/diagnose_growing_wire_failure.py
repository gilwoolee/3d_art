"""Reconstruct the first descending-leg collision without a clearance margin."""

import json
from pathlib import Path
from time import monotonic

import fcl
import numpy as np

from pen_art.cell import load_cell
from pen_art.io import read_json, sha256, write_json
from pen_art.planning.cell_planner import tracked_drawing_path
from pen_art.planning.collision import placed
from pen_art.planning.scene import MaterialScene

profile, arms, _ = load_cell(Path("runs/growing_wire_failure_diagnosis_v1/old_cell_turntable.yaml"))
arm = arms["arm_1"]
body = next(b for b in arm.collision.bodies if b.name == "tool/nozzle_shank")
rows = []
for obj in ["cube", "bottle"]:
    saved = read_json(
        Path("runs/motion_cache_catalogue_v1/adaptation_coverage_candidates")
        / f"{obj}_continuous_phase1_v1_section-00001.json"
    )
    draw = saved["candidate"]
    track = tracked_drawing_path(arm, draw, True, monotonic() + 20)
    q = track["pieces"][19].position(1)
    tcp = arm.robot.forward(q, arm.flange_to_tcp)
    shaft = placed(body.geometry, arm.robot.frames(q)[body.link] @ body.origin)
    # At this exact source knot: original first 10 subsegments have been deposited.
    source = np.asarray(draw["tcp_targets_m"])
    point = source[10]
    transform = arm.robot.frames(q)[body.link] @ body.origin
    peak_local = transform[:3, :3].T @ (source[8] - transform[:3, 3])
    outside = np.maximum(
        [
            np.linalg.norm(peak_local[:2]) - body.geometry.radius,
            abs(peak_local[2]) - body.geometry.lz / 2,
        ],
        0,
    )
    peak_distance = float(np.linalg.norm(outside))
    print(obj, "tracking_error", np.linalg.norm(tcp[:3, 3] - point))
    scene = MaterialScene()
    for j, (a, b) in enumerate(zip(source[:10], source[1:11], strict=True)):
        scene.add(a, b, 0.0005, f"ideal/{j}")
    contacts = []
    for segment, wire in zip(scene.segments, scene.objects, strict=True):
        result = fcl.CollisionResult()
        n = fcl.collide(
            shaft, wire, fcl.CollisionRequest(enable_contact=True, num_max_contacts=20), result
        )
        if n:
            contacts.append(
                {
                    "segment": segment,
                    "contacts": [
                        {
                            "depth_mm": float(c.penetration_depth) * 1000,
                            "position_m": np.asarray(c.pos).tolist(),
                            "distance_from_tcp_mm": float(
                                np.linalg.norm(np.asarray(c.pos) - tcp[:3, 3])
                            )
                            * 1000,
                        }
                        for c in result.contacts
                    ],
                }
            )
    rows.append(
        {
            "object": obj,
            "sampled_piece": 19,
            "tcp_m": tcp[:3, 3].tolist(),
            "tcp_below_peak_mm": float((source[8, 2] - tcp[2, 3]) * 1000),
            "source_point_tracking_error_m": float(np.linalg.norm(tcp[:3, 3] - point)),
            "nominal_strand_radius_mm": 0.5,
            "inflation_mm": 0,
            "clearance_margin_mm": 0,
            "actual_overlap": bool(contacts),
            "independent_peak_sphere_check": {
                "peak_center_to_cylinder_mm": peak_distance * 1000,
                "strand_radius_mm": 0.5,
                "overlap_confirmed": peak_distance < 0.0005,
            },
            "contacts": contacts,
        }
    )
write_json(
    Path("runs/growing_wire_failure_diagnosis_v1/nominal_collision.json"),
    {
        "scope": (
            "Direct collision query at an exact source knot against already deposited "
            "ideal source segments; no clearance margin or material inflation"
        ),
        "cases": rows,
        "inputs": {
            "tool_profile_sha256": sha256(
                Path("runs/growing_wire_failure_diagnosis_v1/old_cell_turntable.yaml")
            ),
            "diagnostic_script_sha256": sha256(Path(__file__)),
        },
    },
)
print(json.dumps(rows, indent=2))
