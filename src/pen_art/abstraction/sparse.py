"""Bounded offline selection over fixed, whole source curves.

Maximize multiview geometric coverage under a raw-curve drawing allowance,
then minimize cost while retaining most of the achieved score. Connections are
evaluated downstream, never treated as free or guaranteed by this model.
"""

import argparse
from copy import deepcopy
from math import ceil
from pathlib import Path

import numpy as np
import trimesh
import yaml
from ortools.sat.python import cp_model

from ..curves import curve_coordinate_bounds
from ..io import artifact, read_json, sha256, stage_update, write_json
from ..recognition import edge_coverage, feature_preview, visible_features
from ..recognition import feature_coverage as visible_feature_coverage
from .comparison import (
    compare_mask,
    mesh_mask,
    polylines,
    preview,
    stroke_mask,
    validation_views,
)
from .selection import feature_coverage, raw_sources


def select_fixed(curves, coverage, weights, costs_s, budget_s, settings, retention_groups=None):
    """Two bounded solves, with no candidate generation or connection search."""
    coverage = np.asarray(coverage, dtype=bool)
    weights, costs_s = np.asarray(weights), np.asarray(costs_s)
    if coverage.shape != (len(curves), len(weights)) or not len(curves):
        raise ValueError("Coverage dimensions must match nonempty source curves")
    if (
        not np.isfinite(weights).all()
        or np.any(weights < 0)
        or weights.sum() <= 0
        or not np.isfinite(costs_s).all()
        or np.any(costs_s < 0)
        or not np.isfinite(budget_s)
        or budget_s <= 0
    ):
        raise ValueError("Coverage weights and drawing costs must be finite and nonnegative")
    retained, seconds = settings["retained_coverage"], settings["maximum_seconds"]
    resources = settings.get("drawing_resources", 1)
    if type(resources) is not int or resources not in (1, 2):
        raise ValueError("drawing_resources must be one or two")
    if not 0 < retained <= 1 or not np.isfinite(seconds) or seconds <= 0:
        raise ValueError("Retention must be in (0,1] and solver time finite and positive")
    # Merge features with identical covering sets, preserving their total weight.
    patterns, inverse = np.unique(coverage.T, axis=0, return_inverse=True)
    importance = np.bincount(inverse, weights=weights)
    integers = np.where(importance > 0, np.maximum(1, np.rint(importance * 1_000_000)), 0).astype(
        int
    )
    model = cp_model.CpModel()
    chosen = [model.new_bool_var(f"curve/{i}") for i in range(len(curves))]
    covered = []
    for i, pattern in enumerate(patterns):
        variable = model.new_bool_var(f"coverage/{i}")
        providers = [chosen[j] for j in np.flatnonzero(pattern)]
        if providers:
            model.add_max_equality(variable, providers)
        else:
            model.add(variable == 0)
        covered.append(variable)
    cost_ms = [ceil(float(c) * 1000) for c in costs_s]
    cost = sum(c * v for c, v in zip(cost_ms, chosen, strict=True))
    score = sum(int(w) * v for w, v in zip(integers, covered, strict=True))
    capacity = int(budget_s * 1000)
    model.add(cost <= capacity * resources)
    assigned = (
        [
            [model.new_bool_var(f"resource/{r}/curve/{i}") for i in range(len(curves))]
            for r in range(resources)
        ]
        if resources > 1
        else [chosen]
    )
    if resources > 1:
        for i, variable in enumerate(chosen):
            model.add(sum(row[i] for row in assigned) == variable)
        for row in assigned:
            model.add(sum(c * v for c, v in zip(cost_ms, row, strict=True)) <= capacity)
    for curve, variable in zip(curves, chosen, strict=True):
        if curve.get("required"):
            model.add(variable == 1)
    # Supply a deterministic feasible start. Large candidate pools otherwise
    # spend a short offline budget discovering poor long-curve combinations.
    initial = {i for i, curve in enumerate(curves) if curve.get("required")}
    remaining = [capacity] * resources
    initial_assignment = {}
    for i in sorted(initial, key=lambda i: -cost_ms[i]):
        resource = max(range(resources), key=lambda r: remaining[r])
        initial_assignment[i] = resource
        remaining[resource] -= cost_ms[i]
    initial_coverage = (
        coverage[list(initial)].any(axis=0) if initial else np.zeros(len(weights), bool)
    )
    if min(remaining) >= 0:
        while True:
            eligible = [
                i for i in range(len(curves)) if i not in initial and cost_ms[i] <= max(remaining)
            ]
            if not eligible:
                break
            gains = coverage[eligible][:, ~initial_coverage] @ weights[~initial_coverage]
            best = max(
                range(len(eligible)),
                key=lambda j: (gains[j] / max(cost_ms[eligible[j]], 1), -eligible[j]),
            )
            if gains[best] <= 0:
                break
            index = eligible[best]
            resource = max(range(resources), key=lambda r: remaining[r])
            initial_assignment[index] = resource
            initial.add(index)
            initial_coverage |= coverage[index]
            remaining[resource] -= cost_ms[index]
        for i, variable in enumerate(chosen):
            model.add_hint(variable, int(i in initial))
        for pattern, variable in zip(patterns, covered, strict=True):
            model.add_hint(variable, int(any(pattern[i] for i in initial)))
        if resources > 1:
            for r, row in enumerate(assigned):
                for i, variable in enumerate(row):
                    model.add_hint(variable, int(initial_assignment.get(i) == r))
    model.maximize(score)
    solver = cp_model.CpSolver()
    solver.parameters.num_search_workers = 1
    solver.parameters.random_seed = 0
    solver.parameters.max_time_in_seconds = seconds * 0.6
    first = solver.solve(model)
    phases = [
        {
            "objective": "maximize_coverage",
            "status": solver.status_name(first),
            "seconds": solver.wall_time,
        }
    ]
    if first not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        return {"status": solver.status_name(first), "selected": [], "phases": phases}
    indices = [i for i, v in enumerate(chosen) if solver.boolean_value(v)]
    resource_indices = [
        [i for i, v in enumerate(row) if solver.boolean_value(v)] for row in assigned
    ]
    achieved = solver.value(score)
    phases[0].update(score=achieved, bound=solver.best_objective_bound)
    model.add(score >= ceil(achieved * retained))
    # Preserve the achieved evidence in each camera/channel, so cost reduction
    # cannot discard an entire weak view to save a little global score.
    if retention_groups is not None:
        retention_groups = np.asarray(retention_groups)
        if retention_groups.shape != weights.shape:
            raise ValueError("Retention groups must match feature weights")
        for group in np.unique(retention_groups):
            importance = np.bincount(
                inverse,
                weights=weights * (retention_groups == group),
                minlength=len(patterns),
            )
            integers = np.rint(importance * 1_000_000).astype(int)
            group_score = sum(int(w) * v for w, v in zip(integers, covered, strict=True))
            first_score = sum(
                int(w) * solver.value(v) for w, v in zip(integers, covered, strict=True)
            )
            model.add(group_score >= ceil(first_score * retained))
    model.minimize(cost)
    model.clear_hints()
    for variable in chosen:
        model.add_hint(variable, solver.value(variable))
    if resources > 1:
        for row in assigned:
            for variable in row:
                model.add_hint(variable, solver.value(variable))
    solver.parameters.max_time_in_seconds = seconds * 0.4
    second = solver.solve(model)
    phases.append(
        {
            "objective": "minimize_raw_drawing_time",
            "status": solver.status_name(second),
            "seconds": solver.wall_time,
        }
    )
    if second in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        indices = [i for i, v in enumerate(chosen) if solver.boolean_value(v)]
        resource_indices = [
            [i for i, v in enumerate(row) if solver.boolean_value(v)] for row in assigned
        ]
    actual = float(weights[np.any(coverage[indices], axis=0)].sum()) if indices else 0.0
    return {
        "status": "SELECTED" if indices else "NO_COVERAGE_WITHIN_ALLOWANCE",
        "selected": indices,
        "phases": phases,
        "coverage_score": actual,
        "raw_curve_seconds": float(costs_s[indices].sum()),
        "coverage_retention": retained,
        "drawing_resources": resources,
        "drawing_loads_s": [float(costs_s[row].sum()) for row in resource_indices],
        "resource_curve_ids": [[curves[i]["id"] for i in row] for row in resource_indices],
    }


def sparse_run(run, settings_path=None, *, settings=None):
    run = Path(run)
    root = Path(__file__).resolve().parents[3]
    settings_path = settings_path or root / "configs/sparse.yaml"
    settings = (
        deepcopy(settings)
        if settings is not None
        else yaml.safe_load(Path(settings_path).read_text())
    )
    comparison = yaml.safe_load((root / "configs/comparison.yaml").read_text())
    fabrication = read_json(run / "fabrication_config.json")
    write_json(
        run / "selection_config.json",
        {"selection": settings, "comparison": comparison, "fabrication": fabrication},
    )
    sources = raw_sources(run)
    active_sources = settings.get("candidate_sources", sources)
    if (
        not isinstance(active_sources, list)
        or not active_sources
        or len(set(active_sources)) != len(active_sources)
        or not set(active_sources) <= set(sources)
    ):
        raise ValueError("candidate_sources must list unique available source files")
    section_filter = settings.get("major_horizontal_sections")
    if section_filter is not None and (
        not np.isfinite(section_filter) or not 0 < section_filter <= 1
    ):
        raise ValueError("major_horizontal_sections must be an extent fraction in (0,1]")
    bounds = np.asarray(read_json(run / "asset_manifest.json")["bounds_m"])
    curves = []
    for name in active_sources:
        for original in read_json(run / name)["curves"]:
            if original.get("origin") == "geometric_section" and section_filter is not None:
                extent = np.diff(curve_coordinate_bounds(original), axis=0)[0]
                if (
                    original.get("axis") != 2
                    or not original.get("closed")
                    or np.linalg.norm(extent[:2])
                    < section_filter * np.linalg.norm((bounds[1] - bounds[0])[:2])
                ):
                    continue
            curve = deepcopy(original)
            curve.update(
                source_curve_id=curve["id"], source_file=name, id=f"{Path(name).stem}/{curve['id']}"
            )
            curves.append(curve)
    if not curves:
        raise ValueError("No source curves available for offline selection")
    inputs = [
        "geometry.glb",
        "asset_manifest.json",
        "selection_config.json",
        "fabrication_config.json",
        *sources,
    ]
    keypoints = None
    if (run / "wir3d_keypoints.json").is_file():
        inputs.append("wir3d_keypoints.json")
        keypoints = read_json(run / inputs[-1])["points"]
    mesh = trimesh.load(run / "geometry.glb", force="mesh", process=False)
    stage_update(run, "selection", "running", message="Two bounded fixed-candidate solves")
    groups, descriptions = None, []
    if settings.get("visible_edges"):
        coverage, weights, lines, (groups, descriptions) = visible_feature_coverage(
            curves, mesh, bounds, comparison, settings, fabrication, keypoints
        )
    else:
        coverage, weights, lines = feature_coverage(
            curves, mesh, bounds, comparison, settings, fabrication, keypoints
        )
    process = fabrication["process"]
    speed = process["nominal_speed_m_s"]
    if speed <= 0 or not np.isfinite(speed):
        raise ValueError("Drawing speed must be finite and positive")
    costs = [
        np.linalg.norm(np.diff(line, axis=0), axis=1).sum() / speed
        + process["start_lag_s"]
        + process["stop_lag_s"]
        for line in lines
    ]
    allowance = (
        comparison["target_seconds"]
        - settings["overhead_reserve_s"]
        - settings["support_reserve_s"]
    )
    result = select_fixed(curves, coverage, weights, costs, allowance, settings, groups)
    selected = [curves[i] for i in result["selected"]]
    if groups is not None:
        for index, description in enumerate(descriptions):
            subset = groups == index
            available = coverage[:, subset].any(axis=0)
            retained = coverage[result["selected"]][:, subset].any(axis=0)
            mass = weights[subset]
            description.update(
                available_coverage=float(mass[available].sum() / mass.sum()),
                selected_coverage=float(mass[retained].sum() / mass.sum()),
            )
    document = artifact(
        "curve_candidates",
        method="budgeted_selection",
        frame="build",
        units="m",
        label=f"Offline sparse selection · {len(selected)} curves",
        curves=selected,
        inputs={name: sha256(run / name) for name in inputs},
        total_length_m=sum(c["length_m"] for c in selected),
        selection={
            **result,
            "algorithm": "fixed_curve_max_coverage_then_min_cost",
            "budget_seconds": allowance,
            "overhead_reserve_s": settings["overhead_reserve_s"],
            "support_reserve_s": settings["support_reserve_s"],
            "trace": [],
            "source_curve_count": len(curves),
            "candidate_sources": active_sources,
            "target_seconds": comparison["target_seconds"],
            "feature_channels": descriptions,
        },
        limitations=[
            "Geometric coverage is not human recognition",
            "Raw-curve time only; actual support and motion are audited downstream",
            "Retention is relative to the achieved score, not a guaranteed optimum",
        ],
    )
    # Held-out camera angles use no solver feedback and do not trigger another solve.
    evidence = []
    selected_lines = polylines(selected, comparison)
    for i, view in enumerate(validation_views(comparison)):
        mask, mpp = mesh_mask(mesh, bounds, view, comparison["resolution"])
        rendered = stroke_mask(
            selected_lines,
            bounds,
            view,
            comparison["resolution"],
            fabrication["graph"]["strand_diameter_m"],
        )
        picture = f"sparse_view_{i}.png"
        row = {
            **view,
            **compare_mask(mask, rendered, comparison["boundary_tolerance_m"] / mpp),
            "all_candidates_boundary_coverage": compare_mask(
                mask,
                stroke_mask(
                    lines,
                    bounds,
                    view,
                    comparison["resolution"],
                    fabrication["graph"]["strand_diameter_m"],
                ),
                comparison["boundary_tolerance_m"] / mpp,
            )["boundary_coverage"],
            "image": picture,
        }
        if settings.get("visible_edges"):
            features = visible_features(
                mesh, bounds, view, comparison["resolution"], settings["visible_edges"]
            )
            scores, rendered = edge_coverage(
                selected_lines,
                bounds,
                view,
                comparison["resolution"],
                fabrication["graph"]["strand_diameter_m"],
                features,
                comparison["boundary_tolerance_m"],
            )
            baseline, _ = edge_coverage(
                lines,
                bounds,
                view,
                comparison["resolution"],
                fabrication["graph"]["strand_diameter_m"],
                features,
                comparison["boundary_tolerance_m"],
            )
            row.update(
                interior_coverage=scores["interior_coverage"],
                all_candidates_interior_coverage=baseline["interior_coverage"],
            )
            feature_preview(features, rendered).save(run / picture)
        else:
            preview(mask, rendered).save(run / picture)
        evidence.append(row)
    document["selection"]["held_out_views"] = evidence
    write_json(run / "selected_candidates.json", document)
    stage_update(
        run,
        "selection",
        "complete" if selected else "failed",
        artifact="selected_candidates.json",
        curve_count=len(selected),
        message="Fixed-candidate selection; recognition and fabrication unverified",
    )
    return document


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    args = parser.parse_args()
    output = sparse_run(args.run)
    print(output["label"], output["selection"])
