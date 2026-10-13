"""Rebuild a mesh as a closed solid without features too thin to print as wire.

The mesh is sampled on a grid, filled from the outside in, opened to remove
thin parts, and extracted again as a surface. The result is closed by
construction. It is a deliberate simplification of the source shape, recorded
with its measured deviation; it is not a repair that preserves detail.
"""

import fast_simplification
import numpy as np
import trimesh
from scipy import ndimage
from scipy.spatial import cKDTree
from skimage.measure import marching_cubes


def solidify(mesh, minimum_feature_m, pitch_m=0.001, maximum_faces=200_000):
    """Return ``(solid, report)`` for ``mesh`` standing on the build surface.

    Parts thinner than ``minimum_feature_m`` are removed, as are holes and
    seams narrower than one grid cell. The solid is seated at the source's
    lowest height.
    """
    if not (np.isfinite(minimum_feature_m) and np.isfinite(pitch_m)) or pitch_m <= 0:
        raise ValueError("Solidify needs a finite feature size and a positive grid pitch")
    if minimum_feature_m < 2 * pitch_m:
        raise ValueError("Minimum feature must span at least two grid cells")
    radius = minimum_feature_m / 2 / pitch_m
    pad = int(np.ceil(radius)) + 3
    grid = mesh.voxelized(pitch_m)
    surface = np.pad(np.asarray(grid.matrix, dtype=bool), pad)
    # Everything the outside cannot reach is solid, whatever the face winding.
    labels, _ = ndimage.label(~surface)
    solid = labels != labels[0, 0, 0]
    filled = int(solid.sum()) - int(surface.sum())
    if filled <= 0:
        raise ValueError(
            "The surface encloses no volume at this grid pitch; it has an opening wider "
            "than one cell"
        )
    # The bed supports whatever rests on it, so a low base is not a thin part:
    # continue the footprint below the bed while judging thickness.
    below = pad - int(np.ceil(radius)) - 1
    footprint = solid[:, :, pad : pad + 2].any(axis=2)
    solid[:, :, below:pad] = footprint[:, :, None]
    # Opening: erode by the feature radius, then grow back by the same amount.
    eroded = ndimage.distance_transform_edt(solid) > radius
    opened = ndimage.distance_transform_edt(~eroded) <= radius
    opened[:, :, :pad] = solid[:, :, :pad] = False
    # Keep what stands on the build surface; a detached remnant cannot be printed.
    bodies, count = ndimage.label(opened)
    floor = np.unique(bodies[:, :, pad : pad + 2])
    kept = np.isin(bodies, floor[floor > 0])
    if not kept.any():
        raise ValueError("Nothing thicker than the minimum feature stands on the build surface")
    field = ndimage.gaussian_filter(kept.astype(np.float32), 1.0)
    vertices, faces, _, _ = marching_cubes(field, 0.5)
    origin = np.asarray(grid.transform)[:3, 3] - pad * pitch_m
    vertices = vertices * pitch_m + origin
    if len(faces) > maximum_faces:
        vertices, faces = fast_simplification.simplify(
            vertices, faces, target_count=maximum_faces
        )
    result = trimesh.Trimesh(vertices, faces, process=True)
    result.merge_vertices()
    trimesh.repair.fix_normals(result)
    result.apply_translation([0, 0, mesh.bounds[0, 2] - result.bounds[0, 2]])
    if not result.is_watertight:
        raise ValueError("Solidified surface is not closed")
    # How far the new surface lies from the source surface, by dense sampling.
    samples, _ = trimesh.sample.sample_surface(result, 5000, seed=0)
    source, _ = trimesh.sample.sample_surface(mesh, 400_000, seed=0)
    removed, _ = cKDTree(source).query(samples)
    return result, {
        "method": "voxel_fill_open_marching_cubes",
        "pitch_m": pitch_m,
        "minimum_feature_m": minimum_feature_m,
        "source_faces": int(len(mesh.faces)),
        "faces": int(len(result.faces)),
        "enclosed_volume_m3": float(solid.sum() * pitch_m**3),
        "volume_m3": float(result.volume),
        "detached_bodies_removed": int(count - (floor > 0).sum()),
        "bounds_m": result.bounds.tolist(),
        "source_bounds_m": mesh.bounds.tolist(),
        "surface_shift_m": {
            "median": float(np.median(removed)),
            "p95": float(np.percentile(removed, 95)),
            "maximum": float(removed.max()),
        },
    }
