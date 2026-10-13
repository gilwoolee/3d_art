from pathlib import Path

import fcl
import numpy as np

from pen_art.cell import load_cell
from pen_art.io import read_json, sha256, write_json
from pen_art.planning.cell_motion import radial_pen_rotations
from pen_art.planning.collision import placed
from pen_art.planning.scene import MaterialScene

profile_path = Path("configs/cell/one_flexiv_straight.yaml")
profile, arms, _ = load_cell(profile_path)
arm = arms["arm_1"]
body = next(b for b in arm.collision.bodies if b.name == "tool/nozzle_shank")
rows = []
for obj in ["cube", "bottle"]:
    saved = read_json(
        Path("runs/motion_cache_catalogue_v1/adaptation_coverage_candidates")
        / f"{obj}_continuous_phase1_v1_section-00001.json"
    )
    points = np.asarray(saved["candidate"]["tcp_targets_m"])
    scene = MaterialScene()
    for i in range(10):
        scene.add(points[i], points[i + 1], 0.0005, f"ideal/{i}")
    for tilt in [10, 20, 30]:
        tcp = np.eye(4)
        tcp[:3, :3] = radial_pen_rotations(
            arm, points[10:11], np.asarray(profile["world_from_build"]), tilt
        )[0]
        tcp[:3, 3] = points[10]
        shaft = placed(body.geometry, tcp @ np.linalg.inv(arm.flange_to_tcp) @ body.origin)
        gaps = [float(fcl.distance(shaft, wire)) for wire in scene.objects]
        rows.append(
            {
                "object": obj,
                "tilt_deg": tilt,
                "min_gap_mm": min(gaps) * 1000 if min(gaps) >= 0 else None,
                "overlap": min(gaps) < 0,
                "one_mm_clearance": min(gaps) >= 0.001,
            }
        )
print(rows)
write_json(
    Path("runs/growing_wire_failure_diagnosis_v1/outward_geometry_comparison.json"),
    {
        "scope": (
            "Tool-only geometry at original first-peak descent sample; "
            "no IK/path/process certification. Negative FCL overlap sentinels are not depths."
        ),
        "profile": str(profile_path),
        "profile_sha256": sha256(profile_path),
        "script_sha256": sha256(Path(__file__)),
        "cases": rows,
    },
)
