"""Visible geometric evidence for sparse sketches, without object-specific labels.

Orthographic depth/normal rasterization exposes interior creases and occlusions
that an outer silhouette alone cannot distinguish. This is a shape proxy, not
a semantic recognition model. Texture is still handled by upstream WIR3D.
"""

import numpy as np
from PIL import Image
from scipy.ndimage import binary_erosion, distance_transform_edt, gaussian_filter
from scipy.spatial import cKDTree

from .abstraction.comparison import polylines, projection, stroke_mask, validation_views


def surface_raster(mesh, bounds, view, resolution):
    """Return frontmost depth and face normal at pixel centers, double-sided.

    Rasterize bounded batches of triangle bounding boxes. Barycentric depth
    decides visibility, so hidden creases never enter the edge target.
    """
    pixels, mpp = projection(mesh.vertices, bounds, view, resolution)
    az, el = np.deg2rad([view["azimuth_deg"], view["elevation_deg"]])
    toward = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    z = (mesh.vertices - np.asarray(bounds).mean(axis=0)) @ toward
    depth = np.full(resolution * resolution, -np.inf)
    face_id = np.full(resolution * resolution, -1, dtype=int)
    for start in range(0, len(mesh.faces), 2048):
        faces = mesh.faces[start : start + 2048]
        tri = pixels[faces]
        lo = np.maximum(0, np.ceil(tri.min(axis=1) - 0.5)).astype(int)
        hi = np.minimum(resolution - 1, np.floor(tri.max(axis=1) - 0.5)).astype(int)
        size = np.maximum(0, hi - lo + 1)
        count = np.prod(size, axis=1)
        owners = np.repeat(np.arange(len(faces)), count)
        if not len(owners):
            continue
        offset = np.arange(count.sum()) - np.repeat(np.cumsum(count) - count, count)
        x = lo[owners, 0] + offset % size[owners, 0]
        y = lo[owners, 1] + offset // size[owners, 0]
        a, b, c = (tri[owners, i] for i in range(3))
        den = (b[:, 1] - c[:, 1]) * (a[:, 0] - c[:, 0]) + (c[:, 0] - b[:, 0]) * (a[:, 1] - c[:, 1])
        valid = np.abs(den) > 1e-12
        safe = np.where(valid, den, 1)
        u = (
            (b[:, 1] - c[:, 1]) * (x + 0.5 - c[:, 0]) + (c[:, 0] - b[:, 0]) * (y + 0.5 - c[:, 1])
        ) / safe
        v = (
            (c[:, 1] - a[:, 1]) * (x + 0.5 - c[:, 0]) + (a[:, 0] - c[:, 0]) * (y + 0.5 - c[:, 1])
        ) / safe
        valid &= (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9)
        owners, x, y, u, v = (item[valid] for item in (owners, x, y, u, v))
        if not len(owners):
            continue
        indices = y * resolution + x
        ztri = z[faces[owners]]
        values = u * ztri[:, 0] + v * ztri[:, 1] + (1 - u - v) * ztri[:, 2]
        previous = depth[indices].copy()
        np.maximum.at(depth, indices, values)
        visible = (values >= depth[indices] - 1e-12) & (values > previous)
        face_id[indices[visible]] = start + owners[visible]
    occupied = face_id >= 0
    normals = np.zeros((resolution * resolution, 3))
    normals[occupied] = mesh.face_normals[face_id[occupied]]
    return depth.reshape(resolution, resolution), normals.reshape(resolution, resolution, 3), mpp


def visible_features(mesh, bounds, view, resolution, settings):
    depth, normals, mpp = surface_raster(mesh, bounds, view, resolution)
    mask = np.isfinite(depth)
    boundary = mask & ~binary_erosion(mask)
    # Suppress sub-strand ornament and triangulation noise in image space.
    sigma = settings.get("smoothing_m", 0) / mpp
    if sigma > 0:
        az, el = np.deg2rad([view["azimuth_deg"], view["elevation_deg"]])
        toward = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
        normals *= np.where(normals @ toward < 0, -1, 1)[..., None]
        weight = np.maximum(gaussian_filter(mask.astype(float), sigma), 1e-12)
        depth = gaussian_filter(np.where(mask, depth, 0), sigma) / weight
        normals = gaussian_filter(normals, (sigma, sigma, 0)) / weight[..., None]
        normals /= np.maximum(np.linalg.norm(normals, axis=-1, keepdims=True), 1e-12)
    interior = np.zeros_like(mask)
    threshold = np.cos(np.deg2rad(settings["normal_angle_deg"]))
    for axis in (0, 1):
        first = (slice(None, -1), slice(None)) if axis == 0 else (slice(None), slice(None, -1))
        second = (slice(1, None), slice(None)) if axis == 0 else (slice(None), slice(1, None))
        valid = mask[first] & mask[second]
        # Ignore normal sign: double-sided triangles may have inconsistent winding.
        crease = np.abs(np.sum(normals[first] * normals[second], axis=-1)) < threshold
        gap = np.zeros_like(valid)
        gap[valid] = np.abs(depth[first][valid] - depth[second][valid]) > settings["depth_jump_m"]
        edge = valid & (crease | gap)
        interior[first] |= edge
        interior[second] |= edge
    # Keep outline and interior evidence in separate channels with no double credit.
    interior &= distance_transform_edt(~boundary) > 1.5
    return {"mask": mask, "boundary": boundary, "interior": interior, "mpp": mpp}


def edge_coverage(lines, bounds, view, resolution, diameter, features, tolerance_m):
    rendered = stroke_mask(lines, bounds, view, resolution, diameter)
    distance = (
        distance_transform_edt(~rendered) if rendered.any() else np.full(rendered.shape, np.inf)
    )
    covered = distance <= tolerance_m / features["mpp"]
    return {
        key + "_coverage": float(covered[features[key]].mean()) if features[key].any() else None
        for key in ("boundary", "interior")
    }, rendered


def feature_preview(features, rendered):
    """Gray outline, blue interior evidence, green selected strokes."""
    pixels = np.full((*rendered.shape, 3), [249, 250, 246], dtype=np.uint8)
    pixels[features["mask"]] = [237, 238, 233]
    pixels[features["boundary"]] = [151, 157, 147]
    pixels[features["interior"]] = [83, 117, 160]
    pixels[rendered] = [35, 103, 66]
    return Image.fromarray(pixels)


def feature_coverage(curves, mesh, bounds, comparison, selection, fabrication, keypoints):
    """Weight each view/channel independently; balance pixels over occupied tiles."""
    config = selection["visible_edges"]
    weights_config = config["weights"]
    if any(not np.isfinite(v) or v < 0 for v in weights_config.values()):
        raise ValueError("Feature weights must be finite and nonnegative")
    if (
        not np.isfinite(
            [config["normal_angle_deg"], config["depth_jump_m"], config.get("smoothing_m", 0)]
        ).all()
        or not 0 < config["normal_angle_deg"] < 90
        or config["depth_jump_m"] <= 0
        or config.get("smoothing_m", 0) < 0
    ):
        raise ValueError("Visible-edge thresholds must be positive, angle below 90 degrees")
    tiles = config["spatial_tiles"]
    if not isinstance(tiles, int) or not 1 <= tiles <= 16:
        raise ValueError("spatial_tiles must be an integer from 1 to 16")
    views = validation_views(
        {**comparison, **{key: selection[key] for key in ("azimuth_deg", "elevation_deg")}}
    )
    lines = polylines(curves, comparison)
    blocks, weights, groups, descriptions = [], [], [], []
    resolution = comparison["resolution"]
    for view in views:
        features = visible_features(mesh, bounds, view, resolution, config)
        distances = []
        for line in lines:
            rendered = stroke_mask(
                [line], bounds, view, resolution, fabrication["graph"]["strand_diameter_m"]
            )
            distances.append(
                distance_transform_edt(~rendered)
                if rendered.any()
                else np.full(rendered.shape, np.inf)
            )
        for channel in ("boundary", "interior"):
            target = features[channel]
            if not target.any() or weights_config[channel] == 0:
                continue
            y, x = np.nonzero(target)
            tile_ids = (y * tiles // resolution) * tiles + x * tiles // resolution
            _, counts = np.unique(tile_ids, return_counts=True)
            frequency = np.bincount(tile_ids, minlength=tiles * tiles)
            importance = 1 / frequency[tile_ids] / len(counts)
            blocks.append(
                np.array(
                    [
                        distance[target] <= comparison["boundary_tolerance_m"] / features["mpp"]
                        for distance in distances
                    ]
                )
            )
            weights.append(importance * weights_config[channel] / len(views))
            groups.extend([len(descriptions)] * len(y))
            descriptions.append({**view, "channel": channel, "pixels": len(y)})
    if keypoints is not None and len(keypoints) and weights_config["keypoints"]:
        blocks.append(
            np.array(
                [
                    cKDTree(line).query(keypoints)[0] <= comparison["keypoint_tolerance_m"]
                    for line in lines
                ]
            )
        )
        weights.append(np.full(len(keypoints), weights_config["keypoints"] / len(keypoints)))
        groups.extend([len(descriptions)] * len(keypoints))
        descriptions.append({"channel": "keypoints", "pixels": len(keypoints)})
    if not weights or sum(w.sum() for w in weights) <= 0:
        raise ValueError("No positively weighted visible features")
    importance = np.concatenate(weights)
    return (
        np.concatenate(blocks, axis=1),
        importance / importance.sum(),
        lines,
        (np.array(groups), descriptions),
    )
