import numpy as np
import pytest
import trimesh

from pen_art.abstraction.geometric import extract_candidates
from pen_art.curves import curve_length, fit_polyline, sample_curve


def test_corner_does_not_disappear_when_fitting_three_points():
    points = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0]])
    curve = {"segments": fit_polyline(points, 0.001)}
    assert curve_length(curve) == pytest.approx(2)
    np.testing.assert_allclose(curve["segments"][0][-1], points[1])
    np.testing.assert_allclose(curve["segments"][1][0], points[1])


def test_closed_contour_preserves_closure():
    t = np.linspace(0, np.pi * 2, 200)
    points = np.column_stack([0.03 * np.cos(t), 0.03 * np.sin(t), np.full_like(t, 0.02)])
    curve = {"segments": fit_polyline(points, 0.0005)}
    sampled = sample_curve(curve, 100)
    np.testing.assert_allclose(sampled[0], sampled[-1], atol=1e-12)
    assert np.max(np.abs(np.linalg.norm(sampled[:, :2], axis=1) - 0.03)) < 0.001


def test_geometric_extraction_uses_actual_mesh_planes(tmp_path):
    mesh = trimesh.creation.box(extents=[0.1, 0.08, 0.12])
    np.savez(tmp_path / "geometry.npz", vertices=mesh.vertices, faces=mesh.faces)
    result = extract_candidates(
        tmp_path,
        {
            "geometric_candidates": {
                "sections_per_axis": 3,
                "min_length_m": 0.01,
                "max_curves": 20,
                "fit_tolerance_m": 0.0005,
            }
        },
    )
    assert len(result["curves"]) == 9
    assert result["method"] == "geometric_sections"
    for curve in result["curves"]:
        assert curve["closed"]
        sampled = sample_curve(curve)
        np.testing.assert_allclose(sampled[:, curve["axis"]], curve["section_m"], atol=1e-8)
