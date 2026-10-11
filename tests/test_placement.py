import numpy as np

from pen_art.curves import transform_points
from pen_art.placement import build_transform


def test_platform_transform_preserves_size_and_build_plane():
    bounds = np.array([[-0.12, -0.04, 0], [0.08, 0.16, 0.12]])
    matrix = build_transform(bounds, [0.5, -0.15], 0.2)
    moved = transform_points(bounds, matrix)
    np.testing.assert_allclose(moved.mean(axis=0)[:2], [0.5, -0.15])
    np.testing.assert_allclose(moved[1] - moved[0], bounds[1] - bounds[0])
    assert moved[0, 2] == 0.2
