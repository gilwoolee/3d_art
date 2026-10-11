"""Compare fixed-view shape evidence and connected drawing cost across candidates.

These are reproducible geometric proxies, not learned semantic or human
recognition scores. Printing time remains a partial, assumption-based estimate.
"""

from pathlib import Path

import numpy as np
import trimesh
from PIL import Image, ImageDraw
from scipy.ndimage import binary_erosion, distance_transform_edt
from scipy.spatial import cKDTree

from ..curves import flatten_curve, transform_points
from ..graph import build_graph
from ..io import artifact, read_json, sha256, stage_update, write_json
from ..strokes import pen_strokes
from .wir3d_adapter import available_budgets, selected_candidate_current


def validation_views(settings):
    if not isinstance(settings["resolution"], int) or not 32 <= settings["resolution"] <= 1024:
        raise ValueError("Comparison resolution must be an integer from 32 to 1024")
    for key in (
        "boundary_tolerance_m",
        "keypoint_tolerance_m",
        "sampling_chord_m",
        "flatten_tolerance_m",
        "target_seconds",
    ):
        if not np.isfinite(settings[key]) or settings[key] <= 0:
            raise ValueError(f"{key} must be finite and positive")
    weights = np.array([settings["boundary_weight"], settings["keypoint_weight"]])
    if not np.isfinite(weights).all() or np.any(weights < 0) or weights.sum() <= 0:
        raise ValueError("Comparison weights must be nonnegative with a positive sum")
    views = [(az, el) for el in settings["elevation_deg"] for az in settings["azimuth_deg"]]
    if not views or not np.isfinite(views).all():
        raise ValueError("At least one finite validation view is required")
    return [{"azimuth_deg": float(a), "elevation_deg": float(e)} for a, e in views]


def projection(points, bounds, view, resolution):
    """Common orthographic camera; scale never adapts to an individual candidate."""
    bounds = np.asarray(bounds)
    center = bounds.mean(axis=0)
    span = float(np.linalg.norm(bounds[1] - bounds[0]) * 1.1)
    if span <= 0:
        raise ValueError("Reference bounds must have positive extent")
    az, el = np.deg2rad([view["azimuth_deg"], view["elevation_deg"]])
    right = np.array([-np.sin(az), np.cos(az), 0])
    up = np.array([-np.sin(el) * np.cos(az), -np.sin(el) * np.sin(az), np.cos(el)])
    xy = np.asarray(points) - center
    pixels = np.column_stack([xy @ right, -(xy @ up)]) * resolution / span + resolution / 2
    return pixels, span / resolution


def mesh_mask(mesh, bounds, view, resolution):
    pixels, meters_per_pixel = projection(mesh.vertices, bounds, view, resolution)
    image = Image.new("1", (resolution, resolution))
    drawing = ImageDraw.Draw(image)
    # Union of projected triangles gives the opaque silhouette, including holes.
    # No depth sort is required for a binary silhouette.
    for face in mesh.faces:
        drawing.polygon([tuple(p) for p in pixels[face]], fill=1)
    return np.asarray(image, dtype=bool), meters_per_pixel


def stroke_mask(polylines, bounds, view, resolution, diameter):
    image = Image.new("1", (resolution, resolution))
    drawing = ImageDraw.Draw(image)
    for points in polylines:
        pixels, meters_per_pixel = projection(points, bounds, view, resolution)
        drawing.line(
            [tuple(p) for p in pixels], fill=1, width=max(1, round(diameter / meters_per_pixel))
        )
    return np.asarray(image, dtype=bool)


def compare_mask(reference, strokes, tolerance_pixels):
    boundary = reference & ~binary_erosion(reference)
    if not boundary.any():
        raise ValueError("Reference silhouette is empty")
    distance = distance_transform_edt(~strokes) if strokes.any() else np.full(strokes.shape, np.inf)
    outside = strokes & (distance_transform_edt(~reference) > tolerance_pixels)
    return {
        "boundary_coverage": float(np.mean(distance[boundary] <= tolerance_pixels)),
        "outside_fraction": float(outside.sum() / max(1, strokes.sum())),
    }


def preview(reference, strokes):
    pixels = np.full((*reference.shape, 3), [249, 250, 246], dtype=np.uint8)
    pixels[reference] = [230, 234, 226]
    pixels[reference & ~binary_erosion(reference)] = [160, 171, 159]
    pixels[strokes] = [42, 95, 73]
    return Image.fromarray(pixels)


def polylines(curves, settings):
    return [
        flatten_curve(c, settings["flatten_tolerance_m"], settings["sampling_chord_m"])[0]
        for c in curves
    ]


def score_candidate(
    candidate, mesh, bounds, views, settings, fabrication, keypoints=None, visible_edges=None
):
    graph = build_graph(candidate, fabrication["graph"])
    strokes = pen_strokes(graph)
    raw_lines = polylines(candidate["curves"], settings)
    inverse = np.linalg.inv(np.asarray(graph["candidate_to_build"]))
    # Compare before/after connection in the original asset frame. Grounding the
    # fabrication graph translates both the reference and sculpture together.
    connected = polylines(strokes["curves"], settings)
    connected = [transform_points(points, inverse) for points in connected]
    view_scores, pictures = [], []
    for view in views:
        mask, mpp = mesh_mask(mesh, bounds, view, settings["resolution"])
        raw_mask = stroke_mask(
            raw_lines,
            bounds,
            view,
            settings["resolution"],
            fabrication["graph"]["strand_diameter_m"],
        )
        connected_mask = stroke_mask(
            connected,
            bounds,
            view,
            settings["resolution"],
            fabrication["graph"]["strand_diameter_m"],
        )
        view_scores.append(
            {
                **view,
                "raw": compare_mask(mask, raw_mask, settings["boundary_tolerance_m"] / mpp),
                "connected": compare_mask(
                    mask, connected_mask, settings["boundary_tolerance_m"] / mpp
                ),
            }
        )
        if visible_edges:
            from ..recognition import edge_coverage, feature_preview, visible_features

            features = visible_features(mesh, bounds, view, settings["resolution"], visible_edges)
            raw_scores, raw_mask = edge_coverage(
                raw_lines,
                bounds,
                view,
                settings["resolution"],
                fabrication["graph"]["strand_diameter_m"],
                features,
                settings["boundary_tolerance_m"],
            )
            connected_scores, connected_mask = edge_coverage(
                connected,
                bounds,
                view,
                settings["resolution"],
                fabrication["graph"]["strand_diameter_m"],
                features,
                settings["boundary_tolerance_m"],
            )
            view_scores[-1]["raw"]["interior_coverage"] = raw_scores["interior_coverage"]
            view_scores[-1]["connected"]["interior_coverage"] = connected_scores[
                "interior_coverage"
            ]
            pictures.append(
                (feature_preview(features, raw_mask), feature_preview(features, connected_mask))
            )
        else:
            pictures.append((preview(mask, raw_mask), preview(mask, connected_mask)))
    point_coverage = None
    if keypoints is not None and len(keypoints):
        distances = cKDTree(np.concatenate(raw_lines)).query(keypoints)[0]
        point_coverage = float(np.mean(distances <= settings["keypoint_tolerance_m"]))
    coverage = float(np.mean([row["raw"]["boundary_coverage"] for row in view_scores]))
    numerator = settings["boundary_weight"] * coverage
    denominator = settings["boundary_weight"]
    if point_coverage is not None:
        numerator += settings["keypoint_weight"] * point_coverage
        denominator += settings["keypoint_weight"]
    if denominator <= 0:
        raise ValueError("Available metrics must have a positive total comparison weight")
    interiors = [v["raw"].get("interior_coverage") for v in view_scores]
    interiors = [v for v in interiors if v is not None]
    interior_coverage = float(np.mean(interiors)) if interiors else None
    if visible_edges:
        channel_weights = visible_edges["weights"]
        scores = {"boundary": coverage, "interior": interior_coverage, "keypoints": point_coverage}
        denominator = sum(channel_weights[k] for k, v in scores.items() if v is not None)
        numerator = sum(channel_weights[k] * v for k, v in scores.items() if v is not None)
    process = fabrication["process"]
    speed = process["nominal_speed_m_s"]
    if not np.isfinite(speed) or speed <= 0:
        raise ValueError("Comparison requires a positive assumed drawing speed")
    lag = [process["start_lag_s"], process["stop_lag_s"]]
    if not np.isfinite(lag).all() or min(lag) < 0:
        raise ValueError("Feed lags must be finite nonnegative assumptions")
    raw_length = sum(np.linalg.norm(np.diff(line, axis=0), axis=1).sum() for line in raw_lines)
    length = sum(c["length_m"] for c in strokes["curves"])
    drawing_s = length / speed
    # Deliberate join/cooling, free travel and acceleration need the final plan.
    feed_overhead = len(strokes["curves"]) * sum(lag)
    partial_s = drawing_s + feed_overhead
    result = {
        "label": candidate["label"],
        "method": candidate["method"],
        "curve_count": len(candidate["curves"]),
        "pen_stroke_count": len(strokes["curves"]),
        "raw_length_m": float(raw_length),
        "connected_length_m": float(length),
        "net_length_change_m": float(length - raw_length),
        "added_length_m": graph["metrics"]["added_length_m"],
        "material_components": graph["metrics"]["material_components"],
        "boundary_coverage": coverage,
        "worst_view_boundary_coverage": min(v["raw"]["boundary_coverage"] for v in view_scores),
        "keypoint_coverage": point_coverage,
        "interior_coverage": interior_coverage,
        "shape_proxy_score": numerator / denominator,
        "views": view_scores,
        "timing": {
            "assumed_speed_m_s": speed,
            "drawing_s": drawing_s,
            "feed_start_stop_s": feed_overhead,
            "partial_estimate_s": partial_s,
            "remaining_budget_s": settings["target_seconds"] - partial_s,
            "within_partial_budget": partial_s <= settings["target_seconds"],
            "complete_schedule": False,
        },
    }
    return result, pictures


def rank_candidates(rows):
    """Keep the quality/connected-length Pareto frontier, then rank by shape proxy."""
    for row in rows:
        row["pareto"] = not any(
            other["shape_proxy_score"] >= row["shape_proxy_score"]
            and other["connected_length_m"] <= row["connected_length_m"]
            and (
                other["shape_proxy_score"] > row["shape_proxy_score"]
                or other["connected_length_m"] < row["connected_length_m"]
            )
            for other in rows
            if other is not row
        )
    return sorted(rows, key=lambda row: (-row["shape_proxy_score"], row["connected_length_m"]))


def compare_run(run):
    run = Path(run)
    snapshot = read_json(run / "comparison_config.json")
    settings, fabrication = snapshot["comparison"], snapshot["fabrication"]
    views = validation_views(settings)
    mesh = trimesh.load(run / "geometry.glb", force="mesh", process=False)
    asset = read_json(run / "asset_manifest.json")
    keys = run / "wir3d_keypoints.json"
    keypoints = read_json(keys)["points"] if keys.exists() else None
    sources = candidate_files(run)
    if not sources:
        raise ValueError("No candidate curves are available to compare")
    inputs = ["geometry.glb", "asset_manifest.json", "comparison_config.json", *sources]
    if keys.exists():
        inputs.append(keys.name)
    report = artifact(
        "candidate_comparison",
        status="RUNNING",
        simulation_only=True,
        settings=settings,
        inputs={name: sha256(run / name) for name in inputs},
        candidates=[],
        limitations=[
            "Shape score is geometric; texture semantics and human recognition are not tested",
            "Silhouettes use the simplified geometry copy and orthographic validation views",
            "Timing omits travel, joins, cooling and acceleration; constant speed is assumed",
            "Proposed connections still require structure, deposition and robot access checks",
        ],
    )
    stage_update(
        run, "comparison", "running", message="Comparing fixed-view shapes and drawing cost"
    )
    for name in sources:
        candidate = read_json(run / name)
        result, pictures = score_candidate(
            candidate,
            mesh,
            asset["bounds_m"],
            views,
            settings,
            fabrication,
            keypoints,
            snapshot.get("visible_edges"),
        )
        result["source_file"] = name
        result["id"] = Path(name).stem
        for i, (raw, connected) in enumerate(pictures):
            for kind, picture in (("raw", raw), ("connected", connected)):
                filename = f"compare_{Path(name).stem}_{i:02d}_{kind}.png"
                picture.save(run / filename)
                result["views"][i][f"{kind}_preview"] = filename
        report["candidates"].append(result)
        write_json(run / "candidate_comparison.json", report)
    report["candidates"] = rank_candidates(report["candidates"])
    eligible = [
        c
        for c in report["candidates"]
        if c["material_components"] == 1 and c["timing"]["within_partial_budget"]
    ]
    report["provisional_choice"] = eligible[0]["source_file"] if eligible else None
    report["status"] = "COMPLETE_GEOMETRIC_COMPARISON"
    write_json(run / "candidate_comparison.json", report)
    stage_update(
        run,
        "comparison",
        "complete",
        artifact="candidate_comparison.json",
        message="Geometric comparison and partial timing; recognition and fabrication unvalidated",
    )
    return report


def comparison_current(run):
    run = Path(run)
    path = run / "candidate_comparison.json"
    if not path.is_file():
        return False
    report = read_json(path)
    expected = set(candidate_files(run))
    return (
        report["status"] == "COMPLETE_GEOMETRIC_COMPARISON"
        and expected == {c["source_file"] for c in report["candidates"]}
        and all(
            (run / name).is_file() and sha256(run / name) == value
            for name, value in report["inputs"].items()
        )
        and (
            (run / "wir3d_keypoints.json").exists() == ("wir3d_keypoints.json" in report["inputs"])
        )
    )


def refresh_comparison(run):
    """Retry incomplete comparisons, including after a transient import failure."""
    if (run / "comparison_config.json").is_file() and not comparison_current(run):
        return compare_run(run)
    return None


def candidate_files(run):
    files = ["geometric_candidates.json"] if (run / "geometric_candidates.json").is_file() else []
    if (run / "surface_contours.json").is_file():
        files.append("surface_contours.json")
    files += [f"wir3d_{b}.json" for b in available_budgets(run)]
    if selected_candidate_current(run):
        files.append("selected_candidates.json")
    return files
