import numpy as np
import pytest
import trimesh
from PIL import Image

from pen_art.assets import normalization, prepare_asset
from pen_art.curves import transform_points
from pen_art.io import sha256


@pytest.mark.parametrize("up", ["y", "z"])
def test_transform_roundtrip_scale_and_ground(up):
    points = np.array([[-2, -1, -3], [4, 5, 1], [1, 0, 0]], dtype=float)
    source_to_wir3d, wir3d_to_build = normalization(points, up, 0.15)
    combined = wir3d_to_build @ source_to_wir3d
    physical = transform_points(points, combined)
    assert np.ptp(physical, axis=0).max() == pytest.approx(0.15)
    assert physical[:, 2].min() == pytest.approx(0)
    assert np.linalg.norm(transform_points(points, source_to_wir3d), axis=1).max() == pytest.approx(
        1
    )
    np.testing.assert_allclose(
        transform_points(physical, np.linalg.inv(combined)), points, atol=1e-12
    )
    assert np.linalg.det(combined[:3, :3]) > 0


def test_asset_preserves_texture_and_original(tmp_path):
    mesh = trimesh.creation.box()
    image = Image.new("RGB", (8, 8), (220, 70, 40))
    mesh.visual = trimesh.visual.TextureVisuals(
        uv=np.full((len(mesh.vertices), 2), 0.5),
        material=trimesh.visual.material.PBRMaterial(baseColorTexture=image),
    )
    source = tmp_path / "source.glb"
    mesh.export(source)
    original_hash = sha256(source)
    run = tmp_path / "run"
    run.mkdir()
    result = prepare_asset(
        source, run, {"source_up": "y", "max_extent_m": 0.15, "geometry_faces": 1000}
    )
    assert sha256(source) == original_hash
    reference = trimesh.load_mesh(run / "normalized.glb", process=False)
    assert reference.visual.material.baseColorTexture is not None
    assert result["texture"]["has_base_color"] is True
    assert result["source_triangles"] == 12
    obj = trimesh.load_mesh(run / "wir3d_bundle/model.obj", process=False)
    assert obj.visual.uv is not None
    assert obj.visual.material.image is not None
    assert np.linalg.norm(obj.vertices, axis=1).max() == pytest.approx(1)
