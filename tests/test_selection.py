from copy import deepcopy
from pathlib import Path

import numpy as np
import trimesh
import yaml

from pen_art.abstraction.selection import greedy_select, select_run
from pen_art.abstraction.wir3d_adapter import selected_candidate_current
from pen_art.io import read_json, write_json


def test_greedy_selects_new_features_instead_of_duplicate_strokes():
    curves = [{"id": str(i)} for i in range(3)]
    coverage = np.array([[1, 0], [1, 0], [0, 1]], bool)
    result = greedy_select(
        curves,
        coverage,
        np.array([0.5, 0.5]),
        [1, 1, 1],
        lambda ids: {"partial_seconds": len(ids), "material_components": 1},
        2,
        1e-5,
    )
    assert result["selected"] == [0, 2]
    assert result["cost"]["partial_seconds"] <= 2
    assert result["trace"][-1]["selection_score"] == 1


def test_connection_cost_can_reject_then_reconsider_a_feature_after_new_anchor():
    curves = [{"id": "feature"}, {"id": "bridge"}]

    def cost(ids):
        return {"partial_seconds": 20 if ids == [0] else len(ids), "material_components": 1}

    result = greedy_select(
        curves, np.eye(2, dtype=bool), np.array([0.9, 0.1]), [0.1, 1], cost, 3, 1e-5
    )
    assert result["selected"] == [1, 0]
    assert result["rejected"][0]["partial_seconds"] == 20


def test_required_curves_are_not_silently_removed_to_fit_budget():
    result = greedy_select(
        [{"id": "required", "required": True}],
        np.ones((1, 1), bool),
        np.ones(1),
        [1],
        lambda ids: {"partial_seconds": 10, "material_components": 1},
        5,
        1e-5,
    )
    assert result["status"] == "REQUIRED_CURVES_EXCEED_CONSTRAINTS"
    assert result["selected"] == []


def test_real_selection_preserves_curves_counts_connections_and_invalidates_for_new_source(
    tmp_path,
):
    root = Path(__file__).resolve().parents[1]
    comparison = yaml.safe_load((root / "configs/comparison.yaml").read_text())
    comparison.update(resolution=64, target_seconds=100)
    selection = yaml.safe_load((root / "configs/selection.yaml").read_text())
    selection.update(azimuth_deg=[0], elevation_deg=[30], overhead_reserve_s=10, maximum_seconds=10)
    fabrication = yaml.safe_load((root / "configs/fabrication.yaml").read_text())
    points = np.array(
        [
            [-0.04, -0.04, 0.02],
            [0.04, -0.04, 0.02],
            [0.04, 0.04, 0.02],
            [-0.04, 0.04, 0.02],
            [-0.04, -0.04, 0.02],
        ]
    )
    curves = [
        {
            "id": str(i),
            "origin": "synthetic",
            "segments": [np.linspace(a, b, 4).tolist()],
            "length_m": float(np.linalg.norm(b - a)),
        }
        for i, (a, b) in enumerate(zip(points[:-1], points[1:], strict=True))
    ]
    original = deepcopy(curves)
    mesh = trimesh.creation.box(extents=[0.08, 0.08, 0.02])
    mesh.apply_translation([0, 0, 0.02])
    mesh.export(tmp_path / "geometry.glb")
    write_json(tmp_path / "run.json", {"stages": {}})
    write_json(tmp_path / "asset_manifest.json", {"bounds_m": mesh.bounds.tolist()})
    write_json(tmp_path / "geometric_candidates.json", {"curves": curves, "label": "square"})
    write_json(
        tmp_path / "selection_config.json",
        {"selection": selection, "comparison": comparison, "fabrication": fabrication},
    )
    result = select_run(tmp_path)
    assert 0 < len(result["curves"]) < 4
    assert result["selection"]["cost"]["partial_seconds"] <= 90
    assert result["selection"]["cost"]["material_components"] == 1
    for curve in result["curves"]:
        assert curve["segments"] == original[int(curve["source_curve_id"])]["segments"]
    assert read_json(tmp_path / "geometric_candidates.json")["curves"] == original
    assert selected_candidate_current(tmp_path)
    write_json(tmp_path / "wir3d_20.json", {"curves": curves})
    assert not selected_candidate_current(tmp_path)
