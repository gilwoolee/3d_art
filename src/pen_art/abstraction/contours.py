"""Trace whole apparent contours and openings, then lift them to the mesh.

Every output point comes from a visible surface sample. Depth discontinuities
split a trace instead of introducing a fictitious bridge through empty space.
This is a geometric supplement to WIR3D, not semantic recognition.
"""

from collections import defaultdict

import numpy as np
import trimesh
from scipy.ndimage import gaussian_filter, gaussian_filter1d

from ..curves import curve_length, fit_polyline
from ..io import artifact, read_json, sha256, write_json
from ..recognition import surface_raster


def boundary_loops(mask):
    """Trace oriented pixel-cell boundaries, retaining holes as separate loops."""
    edges = set()
    height, width = mask.shape
    for y, x in zip(*np.nonzero(mask), strict=True):
        if y == 0 or not mask[y - 1, x]:
            edges.add(((x, y), (x + 1, y)))
        if x == width - 1 or not mask[y, x + 1]:
            edges.add(((x + 1, y), (x + 1, y + 1)))
        if y == height - 1 or not mask[y + 1, x]:
            edges.add(((x + 1, y + 1), (x, y + 1)))
        if x == 0 or not mask[y, x - 1]:
            edges.add(((x, y + 1), (x, y)))
    outgoing = defaultdict(set)
    for first, second in edges:
        outgoing[first].add(second)
    directions = {(1, 0): 0, (0, 1): 1, (-1, 0): 2, (0, -1): 3}
    loops = []
    while edges:
        first, second = min(edges)
        points = [first]
        previous, current = first, second
        while True:
            edges.remove((previous, current))
            outgoing[previous].remove(current)
            points.append(current)
            if current == first:
                break
            heading = directions[tuple(np.subtract(current, previous))]
            # Right turns keep diagonally touching components separate.
            following = min(
                outgoing[current],
                key=lambda p: {1: 0, 0: 1, 3: 2, 2: 3}[
                    (directions[tuple(np.subtract(p, current))] - heading) % 4
                ],
            )
            previous, current = current, following
        loops.append(np.asarray(points, dtype=float))
    return loops


def continuous_traces(points, maximum_gap):
    """Split a closed lifted loop at every 3D depth jump, preserving its seam."""
    points = np.asarray(points)
    jumps = np.flatnonzero(np.linalg.norm(np.diff(points, axis=0), axis=1) > maximum_gap)
    if not len(jumps):
        return [(points, True)]
    pieces = (
        np.split(points[:-1], jumps[:-1] + 1)
        if jumps[-1] == len(points) - 2
        else np.split(points[:-1], jumps + 1)
    )
    # The original loop seam is arbitrary; merge its two ends when connected.
    if jumps[-1] != len(points) - 2 and len(pieces) > 1:
        pieces = [np.concatenate([pieces[-1], pieces[0]])] + pieces[1:-1]
    return [(part, False) for part in pieces if len(part) > 1]


def smooth_trace(points, closed, spacing, smoothing, maximum_shift):
    lengths = np.r_[0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    keep = np.r_[True, np.diff(lengths) > 1e-10]
    points, lengths = points[keep], lengths[keep]
    if len(points) < 2 or lengths[-1] < spacing:
        return points
    count = max(3, int(np.ceil(lengths[-1] / spacing)))
    samples = np.linspace(0, lengths[-1], count, endpoint=not closed)
    sampled = np.column_stack([np.interp(samples, lengths, points[:, i]) for i in range(3)])
    smoothed = gaussian_filter1d(
        sampled, smoothing / spacing, axis=0, mode="wrap" if closed else "nearest"
    )
    shift = smoothed - sampled
    shift *= np.minimum(1, maximum_shift / np.maximum(np.linalg.norm(shift, axis=1), 1e-12))[
        :, None
    ]
    smoothed = sampled + shift
    if closed:
        smoothed = np.vstack([smoothed, smoothed[0]])
    else:
        smoothed[[0, -1]] = points[[0, -1]]
    return smoothed


def contour_run(run, settings):
    if settings.get("method") == "mesh":
        from .mesh_contours import mesh_contour_run

        return mesh_contour_run(run, settings)
    if settings.get("method", "image") != "image":
        raise ValueError("Unknown contour method")
    positive = (
        "surface_sample_radius_m",
        "maximum_depth_jump_m",
        "minimum_length_m",
        "sample_spacing_m",
        "smoothing_m",
        "maximum_shift_m",
        "fit_tolerance_m",
    )
    if any(not np.isfinite(settings[k]) or settings[k] <= 0 for k in positive):
        raise ValueError("Contour distances must be finite and positive")
    if not np.isfinite(settings["mask_smoothing_m"]) or settings["mask_smoothing_m"] < 0:
        raise ValueError("Mask smoothing must be finite and nonnegative")
    if not isinstance(settings["resolution"], int) or not 32 <= settings["resolution"] <= 1024:
        raise ValueError("Contour resolution must be an integer from 32 to 1024")
    if any(
        not settings[k] or not np.isfinite(settings[k]).all()
        for k in ("azimuth_deg", "elevation_deg")
    ):
        raise ValueError("Contour views must be nonempty and finite")
    mesh = trimesh.load(run / "geometry.glb", force="mesh", process=False)
    bounds = np.asarray(read_json(run / "asset_manifest.json")["bounds_m"])
    resolution = settings["resolution"]
    curves, diagnostics = [], []
    for azimuth in settings["azimuth_deg"]:
        for elevation in settings["elevation_deg"]:
            view = {"azimuth_deg": azimuth, "elevation_deg": elevation}
            depth, _, mpp = surface_raster(mesh, bounds, view, resolution)
            original = np.isfinite(depth)
            mask = gaussian_filter(original.astype(float), settings["mask_smoothing_m"] / mpp) > 0.5
            az, el = np.deg2rad([azimuth, elevation])
            right = np.array([-np.sin(az), np.cos(az), 0])
            up = np.array([-np.sin(el) * np.cos(az), -np.sin(el) * np.sin(az), np.cos(el)])
            toward = np.cross(right, up)
            loops = boundary_loops(mask)
            start_count = len(curves)
            for loop_index, pixels in enumerate(loops):
                if len(pixels) * mpp < settings["minimum_length_m"]:
                    continue
                x, y = np.clip(np.floor(pixels).astype(int), 0, resolution - 1).T
                # Pixel-edge samples can hit a background face through a tiny
                # raster gap. Choose the foremost actual surface sample in a
                # bounded neighborhood, retaining its real pixel coordinates.
                radius = max(1, int(np.ceil(settings["surface_sample_radius_m"] / mpp)))
                offsets = np.array(
                    [
                        (dy, dx)
                        for dy in range(-radius, radius + 1)
                        for dx in range(-radius, radius + 1)
                        if dy * dy + dx * dx <= radius * radius
                    ]
                )
                ys = np.clip(y[:, None] + offsets[None, :, 0], 0, resolution - 1)
                xs = np.clip(x[:, None] + offsets[None, :, 1], 0, resolution - 1)
                neighborhood = depth[ys, xs]
                front_depth = neighborhood.max(axis=1)
                # Prefer the nearest pixel on the foremost surface. Floating
                # point depth ties must not make a planar trace jump between
                # opposite ends of its search neighborhood.
                near_front = neighborhood >= front_depth[:, None] - 1e-7
                pixel_distance = np.sum(offsets * offsets, axis=1)
                front = np.argmin(np.where(near_front, pixel_distance[None, :], np.inf), axis=1)
                y, x = ys[np.arange(len(y)), front], xs[np.arange(len(x)), front]
                if not np.isfinite(depth[y, x]).all():
                    raise ValueError("Smoothed outline is too far from a visible surface sample")
                surface = bounds.mean(axis=0) + (
                    ((x + 0.5 - resolution / 2) * mpp)[:, None] * right
                    - ((y + 0.5 - resolution / 2) * mpp)[:, None] * up
                    + depth[y, x, None] * toward
                )
                # Pixel boundary orientation distinguishes outside loops and holes.
                area = np.sum(pixels[:-1, 0] * pixels[1:, 1] - pixels[1:, 0] * pixels[:-1, 1])
                for points, closed in continuous_traces(surface, settings["maximum_depth_jump_m"]):
                    if (
                        np.linalg.norm(np.diff(points, axis=0), axis=1).sum()
                        < settings["minimum_length_m"]
                    ):
                        continue
                    points = smooth_trace(
                        points,
                        closed,
                        settings["sample_spacing_m"],
                        settings["smoothing_m"],
                        settings["maximum_shift_m"],
                    )
                    segments = fit_polyline(points, settings["fit_tolerance_m"])
                    if not segments:
                        continue
                    curve = {
                        "id": f"surface-contour-{len(curves):04d}",
                        "origin": "surface_contour",
                        "role": "candidate",
                        "segments": segments,
                        "closed": closed,
                        "required": False,
                        "diameter_m": 0.001,
                        "feature": "opening" if area < 0 else "outline",
                        "view": view,
                        "source_loop": loop_index,
                    }
                    curve["length_m"] = curve_length(curve)
                    if curve["length_m"] >= settings["minimum_length_m"]:
                        curves.append(curve)
            diagnostics.append({**view, "loops": len(loops), "curves": len(curves) - start_count})
    result = artifact(
        "curve_candidates",
        method="surface_contours",
        frame="build",
        units="m",
        label="Surface outlines and openings",
        curves=curves,
        total_length_m=sum(c["length_m"] for c in curves),
        settings=settings,
        views=diagnostics,
        inputs={name: sha256(run / name) for name in ("geometry.glb", "asset_manifest.json")},
        limitations=[
            "View-dependent geometric contours, not semantic recognition",
            "Depth jumps produce separate curves; no bridge is invented",
            "Smoothing moves surface samples within the configured tolerance",
            "Connectivity, physical support, and robot motion require downstream validation",
        ],
    )
    write_json(run / "surface_contours.json", result)
    return result
