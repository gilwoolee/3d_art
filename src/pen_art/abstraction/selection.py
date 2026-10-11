"""Greedy feature coverage with explicit cost of automatic physical connections.

Selects existing complete curves without altering their geometry. Every trial
is reconnected and costed. The heuristic is bounded, not globally optimal, and
its printing-time model remains provisional until the real pen is calibrated.
"""

from copy import deepcopy
from pathlib import Path
from time import monotonic

import numpy as np
import trimesh
from scipy.ndimage import binary_erosion, distance_transform_edt
from scipy.spatial import cKDTree

from ..graph import build_graph
from ..io import artifact, read_json, sha256, stage_update, write_json
from ..strokes import pen_strokes
from .comparison import mesh_mask, polylines, stroke_mask, validation_views
from .wir3d_adapter import available_budgets, selected_candidate_current


def raw_sources(run):
    files = ["geometric_candidates.json"] if (run / "geometric_candidates.json").is_file() else []
    if (run / "surface_contours.json").is_file():
        files.append("surface_contours.json")
    return files + [f"wir3d_{b}.json" for b in available_budgets(run)]


def feature_coverage(curves, mesh, bounds, comparison, selection, fabrication, keypoints):
    settings = {
        **comparison,
        "azimuth_deg": selection["azimuth_deg"],
        "elevation_deg": selection["elevation_deg"],
    }
    views = validation_views(settings)
    lines = polylines(curves, comparison)
    blocks, weights = [], []
    for view in views:
        reference, mpp = mesh_mask(mesh, bounds, view, comparison["resolution"])
        boundary = reference & ~binary_erosion(reference)
        if not boundary.any():
            raise ValueError("Selection reference silhouette is empty")
        covered = []
        for line in lines:
            rendered = stroke_mask(
                [line],
                bounds,
                view,
                comparison["resolution"],
                fabrication["graph"]["strand_diameter_m"],
            )
            distances = (
                distance_transform_edt(~rendered)
                if rendered.any()
                else np.full(rendered.shape, np.inf)
            )
            covered.append(distances[boundary] <= comparison["boundary_tolerance_m"] / mpp)
        blocks.append(np.array(covered))
        weights.append(
            np.full(boundary.sum(), comparison["boundary_weight"] / (len(views) * boundary.sum()))
        )
    if keypoints is not None and len(keypoints):
        blocks.append(
            np.array(
                [
                    cKDTree(points).query(keypoints)[0] <= comparison["keypoint_tolerance_m"]
                    for points in lines
                ]
            )
        )
        weights.append(np.full(len(keypoints), comparison["keypoint_weight"] / len(keypoints)))
    importance = np.concatenate(weights)
    if importance.sum() <= 0:
        raise ValueError("Selection needs a positive weight on available features")
    return np.concatenate(blocks, axis=1), importance / importance.sum(), lines


def connected_cost(curves, fabrication):
    graph = build_graph({"curves": curves, "method": "budgeted_selection"}, fabrication["graph"])
    strokes = pen_strokes(graph)
    process = fabrication["process"]
    length = strokes["total_length_m"]
    seconds = length / process["nominal_speed_m_s"] + len(strokes["curves"]) * (
        process["start_lag_s"] + process["stop_lag_s"]
    )
    return {
        "connected_length_m": length,
        "partial_seconds": seconds,
        "material_components": graph["metrics"]["material_components"],
        "pen_stroke_count": len(strokes["curves"]),
        "connector_support_length_m": graph["metrics"]["added_length_m"],
    }


def greedy_select(
    curves,
    coverage,
    weights,
    costs,
    evaluate,
    budget_s,
    minimum_gain,
    deadline=float("inf"),
    observe=None,
):
    """Lazy greedy gain/cost ordering; each accepted set passes its connected cost.

    Rejected additions are reconsidered after another acceptance, because a new
    anchor can reduce their connection cost. Required curves are never dropped.
    """
    selected = [i for i, curve in enumerate(curves) if curve.get("required")]
    covered = np.any(coverage[selected], axis=0) if selected else np.zeros(len(weights), bool)
    trace, rejected, attempts = [], [], int(bool(selected))
    current = evaluate(selected) if selected else None
    if current and (current["partial_seconds"] > budget_s or current["material_components"] != 1):
        return {
            "status": "REQUIRED_CURVES_EXCEED_CONSTRAINTS",
            "selected": [],
            "trace": [],
            "attempts": 1,
            "rejected": [],
            "cost": current,
        }
    while len(selected) < len(curves):
        if monotonic() >= deadline:
            status = "TIME_BUDGET"
            break
        remaining = [i for i in range(len(curves)) if i not in selected]
        gains = coverage[:, ~covered] @ weights[~covered]
        ranked = sorted(remaining, key=lambda i: (-gains[i] / max(costs[i], 1e-9), curves[i]["id"]))
        accepted = False
        for index in ranked:
            if gains[index] < minimum_gain:
                continue
            if monotonic() >= deadline:
                break
            trial = evaluate([*selected, index])
            attempts += 1
            if trial["partial_seconds"] > budget_s or trial["material_components"] != 1:
                if len(rejected) < 100:
                    rejected.append(
                        {"curve_id": curves[index]["id"], "selected_count": len(selected), **trial}
                    )
                continue
            selected.append(index)
            covered |= coverage[index]
            current = trial
            trace.append(
                {
                    "curve_id": curves[index]["id"],
                    "selected_count": len(selected),
                    "marginal_gain": float(gains[index]),
                    "selection_score": float(weights[covered].sum()),
                    **trial,
                }
            )
            if observe:
                observe(selected, trace, attempts)
            accepted = True
            break
        if not accepted:
            status = "TIME_BUDGET" if monotonic() >= deadline else "NO_FURTHER_BUDGETED_GAIN"
            break
    else:
        status = "ALL_CURVES_SELECTED"
    return {
        "status": status,
        "selected": selected,
        "trace": trace,
        "attempts": attempts,
        "rejected": rejected,
        "cost": current,
    }


def select_run(run):
    run = Path(run)
    snapshot = read_json(run / "selection_config.json")
    settings, comparison, fabrication = (
        snapshot[key] for key in ("selection", "comparison", "fabrication")
    )
    for key in ("maximum_seconds", "minimum_marginal_gain"):
        if not np.isfinite(settings[key]) or settings[key] <= 0:
            raise ValueError(f"Selection {key} must be finite and positive")
    reserve = settings["overhead_reserve_s"]
    if not np.isfinite(reserve) or not 0 <= reserve < comparison["target_seconds"]:
        raise ValueError("Selection overhead reserve must leave a positive drawing budget")
    process = fabrication["process"]
    speed = process["nominal_speed_m_s"]
    lags = [process["start_lag_s"], process["stop_lag_s"]]
    if not np.isfinite([speed, *lags]).all() or speed <= 0 or min(lags) < 0:
        raise ValueError("Selection requires positive speed and nonnegative feed delays")
    budget = comparison["target_seconds"] - reserve
    deadline = monotonic() + settings["maximum_seconds"]
    sources = raw_sources(run)
    if not sources:
        raise ValueError("No source curves are available for selection")
    inputs = ["geometry.glb", "asset_manifest.json", "selection_config.json", *sources]
    keypoints = None
    if (run / "wir3d_keypoints.json").is_file():
        inputs.append("wir3d_keypoints.json")
        keypoints = read_json(run / inputs[-1])["points"]
    fingerprints = {name: sha256(run / name) for name in inputs}
    curves = []
    for name in sources:
        for curve in read_json(run / name)["curves"]:
            row = deepcopy(curve)
            row["source_curve_id"], row["source_file"] = row["id"], name
            row["id"] = f"{Path(name).stem}/{row['id']}"
            curves.append(row)
    if not curves:
        raise ValueError("Source candidates contain no curves")
    mesh = trimesh.load(run / "geometry.glb", force="mesh", process=False)
    bounds = read_json(run / "asset_manifest.json")["bounds_m"]
    stage_update(
        run, "selection", "running", message="Selecting shape features within a drawing budget"
    )
    coverage, weights, lines = feature_coverage(
        curves, mesh, bounds, comparison, settings, fabrication, keypoints
    )
    costs = [
        np.linalg.norm(np.diff(line, axis=0), axis=1).sum() / speed + sum(lags) for line in lines
    ]

    def checkpoint(selected, trace, attempts):
        write_json(
            run / "selection_progress.json",
            artifact(
                "selection_progress",
                status="RUNNING",
                inputs=fingerprints,
                selected_count=len(selected),
                trace=trace,
                attempts=attempts,
            ),
        )

    result = greedy_select(
        curves,
        coverage,
        weights,
        costs,
        lambda indices: connected_cost([curves[i] for i in indices], fabrication),
        budget,
        settings["minimum_marginal_gain"],
        deadline,
        checkpoint,
    )
    chosen = [curves[i] for i in result["selected"]]
    selected = artifact(
        "curve_candidates",
        method="budgeted_selection",
        frame="build",
        units="m",
        label=f"Budgeted selection · {len(chosen)} curves",
        curves=chosen,
        inputs=fingerprints,
        total_length_m=sum(c["length_m"] for c in chosen),
        selection={**result, "budget_seconds": budget, "overhead_reserve_s": reserve},
        limitations=[
            "Selection uses geometric coverage, not verified human recognition",
            "Time uses uncalibrated assumptions; reserved overhead is not a guarantee",
            "Connections, hot-strand support and arm access require validation",
        ],
    )
    write_json(run / "selected_candidates.json", selected)
    write_json(
        run / "selection_progress.json",
        artifact(
            "selection_progress",
            status=result["status"],
            inputs=fingerprints,
            selected_count=len(chosen),
            trace=result["trace"],
            attempts=result["attempts"],
        ),
    )
    stage_update(
        run,
        "selection",
        "complete" if chosen else "partial",
        artifact="selected_candidates.json",
        curve_count=len(chosen),
        message="Budgeted geometric candidate; recognition and fabrication unvalidated",
    )
    return selected


def refresh_selection(run):
    if (run / "selection_config.json").is_file() and not selected_candidate_current(
        run, require_curves=False
    ):
        if (
            read_json(run / "selection_config.json")["selection"].get("algorithm")
            == "offline_sparse"
        ):
            from .sparse import sparse_run

            return sparse_run(run, settings=read_json(run / "selection_config.json")["selection"])
        return select_run(run)
    return None
