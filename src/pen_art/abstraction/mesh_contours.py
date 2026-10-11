"""Connected apparent contours from a normal field on the actual mesh.

March triangle edges where the normal becomes perpendicular to a fixed view.
Trace through shared mesh edges before fitting or selection; image occlusion
never cuts a source contour into disconnected fabrication strokes.
"""

from collections import defaultdict

import numpy as np
import trimesh
from scipy.sparse import csr_matrix
from scipy.spatial import cKDTree

from ..curves import curve_length, fit_polyline
from ..io import artifact, read_json, sha256, write_json
from ..recognition import surface_raster
from .comparison import projection
from .contours import smooth_trace


def trace_edges(points, edges):
    """Visit every graph edge once, continuing smoothly through nonmanifold nodes."""
    adjacency = defaultdict(set)
    remaining = set()
    for a, b in edges:
        a, b = int(a), int(b)
        if a == b:
            continue
        adjacency[a].add(b)
        adjacency[b].add(a)
        remaining.add(tuple(sorted((a, b))))
    pairing = {}
    for node, neighbors in adjacency.items():
        pool = set(neighbors)
        while len(pool) > 1:
            pairs = [(a, b) for a in sorted(pool) for b in sorted(pool) if a < b]
            vectors = {a: points[a] - points[node] for a in pool}
            vectors = {a: v / max(np.linalg.norm(v), 1e-12) for a, v in vectors.items()}
            a, b = min(pairs, key=lambda pair: (vectors[pair[0]] @ vectors[pair[1]], pair))
            pairing[node, a], pairing[node, b] = b, a
            pool.difference_update((a, b))
    starts = sorted(
        (node, n) for node, ns in adjacency.items() for n in ns if (node, n) not in pairing
    )
    traces = []
    while remaining:
        while starts and tuple(sorted(starts[-1])) not in remaining:
            starts.pop()
        start, next_node = starts.pop() if starts else min(remaining)
        sequence = [start]
        previous, current = start, next_node
        while tuple(sorted((previous, current))) in remaining:
            remaining.remove(tuple(sorted((previous, current))))
            sequence.append(current)
            following = pairing.get((current, previous))
            if following is None:
                break
            previous, current = current, following
        traces.append((points[sequence], sequence[0] == sequence[-1]))
    return traces


def normal_contours(mesh, normals, toward):
    values = normals @ toward
    values[np.abs(values) < 1e-10] = 1e-10
    edges = mesh.edges_unique
    crossing = values[edges[:, 0]] * values[edges[:, 1]] < 0
    ids = mesh.edges_unique_inverse.reshape(-1, 3)
    selected = crossing[ids]
    valid = selected.sum(axis=1) == 2
    links = ids[valid][selected[valid]].reshape(-1, 2)
    a, b = edges.T
    fraction = values[a] / np.where(crossing, values[a] - values[b], 1)
    points = mesh.vertices[a] + fraction[:, None] * (mesh.vertices[b] - mesh.vertices[a])
    return trace_edges(points, links)


def mesh_contour_run(run, settings):
    positive = (
        "normal_smoothing_m",
        "minimum_length_m",
        "sample_spacing_m",
        "smoothing_m",
        "maximum_shift_m",
        "fit_tolerance_m",
        "visibility_tolerance_m",
    )
    if any(not np.isfinite(settings[k]) or settings[k] <= 0 for k in positive):
        raise ValueError("Mesh contour distances must be finite and positive")
    if not 0 <= settings["minimum_visible_fraction"] <= 1:
        raise ValueError("Visible fraction must lie in [0,1]")
    if not isinstance(settings["resolution"], int) or not 32 <= settings["resolution"] <= 1024:
        raise ValueError("Visibility resolution must be an integer from 32 to 1024")
    if any(
        not settings[k] or not np.isfinite(settings[k]).all()
        for k in ("azimuth_deg", "elevation_deg")
    ):
        raise ValueError("Mesh contour views must be nonempty and finite")
    mesh = trimesh.load(run / "geometry.glb", force="mesh", process=False)
    bounds = np.asarray(read_json(run / "asset_manifest.json")["bounds_m"])
    if not mesh.is_winding_consistent:
        raise ValueError("Mesh contour extraction requires consistent face winding")
    tree = cKDTree(mesh.vertices)
    distance = tree.sparse_distance_matrix(
        tree, settings["normal_smoothing_m"], output_type="coo_matrix"
    )
    weights = np.exp(-0.5 * (2 * distance.data / settings["normal_smoothing_m"]) ** 2)
    kernel = csr_matrix((weights, (distance.row, distance.col)), shape=distance.shape)
    normals = kernel @ mesh.vertex_normals
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    curves, diagnostics = [], []
    for azimuth in settings["azimuth_deg"]:
        for elevation in settings["elevation_deg"]:
            view = {"azimuth_deg": azimuth, "elevation_deg": elevation}
            az, el = np.deg2rad([azimuth, elevation])
            toward = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
            traces = normal_contours(mesh, normals, toward)
            depth, _, _ = surface_raster(mesh, bounds, view, settings["resolution"])
            count = len(curves)
            for points, closed in traces:
                length = np.linalg.norm(np.diff(points, axis=0), axis=1).sum()
                if length < settings["minimum_length_m"]:
                    continue
                points = smooth_trace(
                    points,
                    closed,
                    settings["sample_spacing_m"],
                    settings["smoothing_m"],
                    settings["maximum_shift_m"],
                )
                pixels, _ = projection(points, bounds, view, settings["resolution"])
                x, y = np.clip(pixels.astype(int), 0, settings["resolution"] - 1).T
                z = (points - bounds.mean(axis=0)) @ toward
                visible = z >= depth[y, x] - settings["visibility_tolerance_m"]
                visible &= np.isfinite(depth[y, x])
                fraction = float(visible.mean())
                if fraction < settings["minimum_visible_fraction"]:
                    continue
                curve = {
                    "id": f"mesh-contour-{len(curves):04d}",
                    "origin": "mesh_contour",
                    "role": "candidate",
                    "segments": fit_polyline(points, settings["fit_tolerance_m"]),
                    "closed": closed,
                    "required": False,
                    "diameter_m": 0.001,
                    "view": view,
                    "visible_fraction": fraction,
                }
                if not curve["segments"]:
                    continue
                curve["length_m"] = curve_length(curve)
                if curve["length_m"] >= settings["minimum_length_m"]:
                    curves.append(curve)
            diagnostics.append({**view, "traces": len(traces), "retained": len(curves) - count})
    result = artifact(
        "curve_candidates",
        method="mesh_contours",
        frame="build",
        units="m",
        label="Connected mesh contours",
        curves=curves,
        total_length_m=sum(c["length_m"] for c in curves),
        settings=settings,
        views=diagnostics,
        closed_contours=sum(c["closed"] for c in curves),
        inputs={name: sha256(run / name) for name in ("geometry.glb", "asset_manifest.json")},
        limitations=[
            "Apparent contours of a smoothed normal field, not semantic features",
            "Whole contours can include hidden portions; visibility filters whole curves only",
            "Nonmanifold junctions use local tangent continuity, not semantic topology",
            "Recognition, connectivity and fabrication remain to be validated",
        ],
    )
    write_json(run / "surface_contours.json", result)
    return result
