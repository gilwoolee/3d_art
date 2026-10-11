from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest
import trimesh
import yaml

from pen_art.abstraction.comparison import (
    compare_run,
    comparison_current,
    mesh_mask,
    score_candidate,
)
from pen_art.io import read_json, sha256, write_json


def settings():
    base = Path(__file__).resolve().parents[1]
    comparison = yaml.safe_load((base / "configs/comparison.yaml").read_text())
    comparison.update(resolution=96, azimuth_deg=[22.5], elevation_deg=[45])
    fabrication = yaml.safe_load((base / "configs/fabrication.yaml").read_text())
    return comparison, fabrication


def outline(z=0.02):
    points = np.array(
        [[-0.04, -0.04, z], [0.04, -0.04, z], [0.04, 0.04, z], [-0.04, 0.04, z], [-0.04, -0.04, z]]
    )
    curves = []
    for i, (a, b) in enumerate(zip(points[:-1], points[1:], strict=True)):
        curves.append(
            {
                "id": str(i),
                "origin": "synthetic",
                "segments": [np.linspace(a, b, 4).tolist()],
                "length_m": float(np.linalg.norm(b - a)),
                "closed": False,
            }
        )
    return {"label": "outline", "method": "synthetic", "curves": curves}


def test_silhouette_preserves_openings():
    # Planar square annulus: four quads, with no triangles filling its center.
    vertices = [
        [-1, -1, 0],
        [1, -1, 0],
        [1, 1, 0],
        [-1, 1, 0],
        [-0.4, -0.4, 0],
        [0.4, -0.4, 0],
        [0.4, 0.4, 0],
        [-0.4, 0.4, 0],
    ]
    faces = []
    for i in range(4):
        j = (i + 1) % 4
        faces.extend([[i, j, 4 + j], [i, 4 + j, 4 + i]])
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    mask, _ = mesh_mask(mesh, [[-1, -1, 0], [1, 1, 0]], {"azimuth_deg": 0, "elevation_deg": 90}, 96)
    assert not mask[48, 48] and mask.any()


def test_connections_are_costed_and_grounding_translation_does_not_shift_comparison():
    config, fabrication = settings()
    candidate = outline()
    mesh = trimesh.creation.box(extents=[0.08, 0.08, 0.01])
    mesh.apply_translation([0, 0, 0.02])
    view = [{"azimuth_deg": 22.5, "elevation_deg": 45}]
    row, pictures = score_candidate(
        candidate, mesh, mesh.bounds, view, config, fabrication, keypoints=[[-0.04, -0.04, 0.02]]
    )
    assert row["material_components"] == 1
    assert row["connected_length_m"] >= row["raw_length_m"] - 1e-8
    assert row["keypoint_coverage"] == 1
    assert row["timing"]["drawing_s"] == pytest.approx(
        row["connected_length_m"] / fabrication["process"]["nominal_speed_m_s"]
    )
    assert row["timing"]["complete_schedule"] is False
    raw, connected = pictures[0]

    # Support additions may add pixels, but must not translate the original contour.
    def is_line(image):
        return np.all(np.asarray(image) == [42, 95, 73], axis=2)

    assert np.mean(is_line(connected)[is_line(raw)]) > 0.95


def test_comparison_binds_geometry_and_updates_when_another_budget_arrives(tmp_path):
    config, fabrication = settings()
    mesh = trimesh.creation.box(extents=[0.08, 0.08, 0.02])
    mesh.export(tmp_path / "geometry.glb")
    write_json(tmp_path / "run.json", {"stages": {}})
    write_json(tmp_path / "asset_manifest.json", {"bounds_m": mesh.bounds.tolist()})
    write_json(
        tmp_path / "comparison_config.json", {"comparison": config, "fabrication": fabrication}
    )
    write_json(tmp_path / "geometric_candidates.json", outline())
    result = compare_run(tmp_path)
    assert result["status"] == "COMPLETE_GEOMETRIC_COMPARISON"
    assert comparison_current(tmp_path)
    assert len(list(tmp_path.glob("compare_*.png"))) == 2
    candidate = deepcopy(outline())
    candidate["label"] = "New WIR3D result"
    write_json(tmp_path / "wir3d_20.json", candidate)
    assert not comparison_current(tmp_path)
    compare_run(tmp_path)
    assert comparison_current(tmp_path)
    assert len(read_json(tmp_path / "candidate_comparison.json")["candidates"]) == 2


def test_offline_arm_comparison_requires_matching_complete_replays(tmp_path):
    from pen_art.planning.cell_comparison import compare_cell

    single, dual = tmp_path / "single", tmp_path / "dual"
    shared = [
        "cell_manifest.json",
        "cell_drawing_variants.json",
        "construction_plan.json",
        "stroke_graph.json",
    ]
    for directory, duration, owners in (
        (single, 20, ["arm_1", "arm_1"]),
        (dual, 10, ["arm_1", "arm_2"]),
    ):
        directory.mkdir()
        for name in shared:
            write_json(directory / name, {"fixture": name})
        config = "offline_dual_arm_config.json"
        write_json(directory / config, {"process": {}, "timing": {}, "transit": {}, "search": {}})
        plan = {
            "status": "FULL_SCHEDULE_GEOMETRY_UNVERIFIED",
            "full_schedule": True,
            "requested_strokes": 2,
            "completed_strokes": 0,
            "modeled_duration_s": duration,
            "initial_positions": {"arm_1": [0], "arm_2": [0]},
            "settings_artifact": config,
            "inputs": {name: sha256(directory / name) for name in [*shared, config]},
            "actions": [
                {"arm": owner, "stroke_id": str(i), "start_s": 0, "trajectory": {"edges": []}}
                for i, owner in enumerate(owners)
            ],
        }
        write_json(directory / "offline_dual_arm_plan.json", plan)
        write_json(
            directory / "offline_dual_arm_validation.json",
            {
                "status": "CERTIFIED_GEOMETRIC_REPLAY",
                "completed_strokes": 2,
                "source_plan_sha256": sha256(directory / "offline_dual_arm_plan.json"),
            },
        )
    result = compare_cell(dual, single)
    assert result["speedup"] == 2
    assert result["results"]["single_arm"]["completed_strokes"] == 2
    assert all(sha256(dual / name) == digest for name, digest in result["inputs"].items())
    check = read_json(dual / "offline_dual_arm_validation.json")
    check.update(status="REPLAY_REJECTED", completed_strokes=1, failures=[{"time_s": 3}])
    write_json(dual / "offline_dual_arm_validation.json", check)
    result = compare_cell(dual, single)
    assert result["speedup"] is None
    assert result["results"]["dual_arm"]["validated_prefix_s"] == 3
    assert result["results"]["single_arm"]["modeled_duration_s"] == 20
    write_json(single / "stroke_graph.json", {"different": "geometry"})
    with pytest.raises(ValueError, match="Stale single_arm input"):
        compare_cell(dual, single)
