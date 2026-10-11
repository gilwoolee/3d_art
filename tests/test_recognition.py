from pathlib import Path

import numpy as np
import trimesh
import yaml

from pen_art.abstraction.contours import continuous_traces, contour_run
from pen_art.abstraction.mesh_contours import normal_contours
from pen_art.abstraction.sparse import select_fixed
from pen_art.curves import sample_curve
from pen_art.io import write_json
from pen_art.recognition import surface_raster, visible_features


def panel(z, half=0.04):
    return trimesh.Trimesh(
        vertices=[[-half, -half, z], [half, -half, z], [half, half, z], [-half, half, z]],
        faces=[[0, 1, 2], [0, 2, 3]],
        process=False,
    )


def test_frontmost_surface_hides_back_geometry_and_has_no_triangulation_crease():
    front = panel(0.01)
    hidden = trimesh.creation.box(extents=[0.03, 0.03, 0.01])
    mesh = trimesh.util.concatenate([front, hidden])
    bounds = [[-0.04, -0.04, -0.01], [0.04, 0.04, 0.01]]
    view = {"azimuth_deg": 0, "elevation_deg": 90}
    depth, normals, _ = surface_raster(mesh, bounds, view, 96)
    assert np.isclose(depth[48, 48], 0.01)
    assert np.allclose(normals[48, 48], [0, 0, 1])
    features = visible_features(
        mesh, bounds, view, 96, {"normal_angle_deg": 35, "depth_jump_m": 0.002}
    )
    assert features["boundary"].any()
    assert not features["interior"].any()
    # An actually visible raised panel must introduce an interior occlusion edge.
    raised = trimesh.util.concatenate([front, panel(0.02, half=0.015)])
    features = visible_features(
        raised, bounds, view, 96, {"normal_angle_deg": 35, "depth_jump_m": 0.002}
    )
    assert features["interior"].sum() > 20


def test_cost_reduction_preserves_small_independent_feature_group():
    args = (
        [{"id": "outline"}, {"id": "interior"}],
        np.eye(2, dtype=bool),
        [0.96, 0.04],
        [1, 1],
        2,
        {"retained_coverage": 0.95, "maximum_seconds": 2},
    )
    assert select_fixed(*args)["selected"] == [0]
    assert select_fixed(*args, retention_groups=[0, 1])["selected"] == [0, 1]


def test_mesh_normal_contour_is_one_complete_sphere_equator():
    sphere = trimesh.creation.icosphere(subdivisions=3, radius=0.04)
    view = np.array([0.2, -0.3, 0.9])
    view /= np.linalg.norm(view)
    traces = normal_contours(sphere, sphere.vertex_normals, view)
    assert len(traces) == 1
    points, closed = traces[0]
    assert closed and np.allclose(points[0], points[-1])
    assert np.max(np.abs(points @ view)) < 0.0002
    assert np.allclose(np.linalg.norm(points, axis=1), 0.04, atol=0.0002)
    assert np.linalg.norm(np.diff(points, axis=0), axis=1).max() < 0.01


def test_two_pen_capacity_cannot_split_a_whole_curve_between_resources():
    curves = [{"id": str(i)} for i in range(3)]
    result = select_fixed(
        curves,
        np.eye(3, dtype=bool),
        np.ones(3) / 3,
        [6, 6, 6],
        10,
        {"retained_coverage": 0.95, "maximum_seconds": 2, "drawing_resources": 2},
    )
    # Aggregate work fits 20 seconds, but three indivisible six-second curves
    # do not fit two ten-second pens. A simple doubled-length cap misses this.
    assert len(result["selected"]) == 2
    assert all(load <= 10 for load in result["drawing_loads_s"])
    assert sorted(result["drawing_loads_s"]) == [6, 6]


def test_surface_contours_preserve_a_real_opening_and_split_depth_jumps(tmp_path):
    bars = []
    for extents, center in [
        ([0.08, 0.02, 0.01], [0, -0.03, 0.015]),
        ([0.08, 0.02, 0.01], [0, 0.03, 0.015]),
        ([0.02, 0.04, 0.01], [-0.03, 0, 0.015]),
        ([0.02, 0.04, 0.01], [0.03, 0, 0.015]),
    ]:
        bar = trimesh.creation.box(extents=extents)
        bar.apply_translation(center)
        bars.append(bar)
    mesh = trimesh.util.concatenate(bars)
    mesh.export(tmp_path / "geometry.glb")
    write_json(tmp_path / "asset_manifest.json", {"bounds_m": mesh.bounds.tolist()})
    config = yaml.safe_load(
        (Path(__file__).resolve().parents[1] / "configs/contours.yaml").read_text()
    )
    config.update(azimuth_deg=[0], elevation_deg=[90], resolution=128)
    result = contour_run(tmp_path, config)
    assert len(result["curves"]) == 2
    assert {c["feature"] for c in result["curves"]} == {"outline", "opening"}
    for curve in result["curves"]:
        points = sample_curve(curve)
        assert curve["closed"] and np.allclose(points[0], points[-1])
        assert np.allclose(points[:, 2], 0.02, atol=1e-7)
    points = np.array([[0, 0, 0], [0.001, 0, 0], [0.001, 0, 0.04], [0, 0, 0.04], [0, 0, 0]])
    traces = continuous_traces(points, 0.004)
    assert len(traces) == 2 and all(not closed for _, closed in traces)
    assert all(np.linalg.norm(np.diff(p, axis=0), axis=1).max() <= 0.004 for p, _ in traces)
