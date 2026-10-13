"""Attach genuine imported style curves without moving their source geometry."""

from copy import deepcopy

import numpy as np

from ..curves import curve_coordinate_bounds, curve_interval, curve_length
from ..graph import straight_edge
from ..io import artifact
from ..provenance import shared_source_bindings


def select_printable_style(curves, *, bed_z=0.0):
    """Select whole candidates above the bed, with no fabrication-time budget."""
    selected, excluded = [], []
    for curve in curves:
        low, _ = curve_coordinate_bounds(curve)
        if low[2] < bed_z - 1e-6:
            excluded.append(
                {"id": curve["id"], "reason": "BELOW_BUILD_SURFACE", "minimum_z_m": float(low[2])}
            )
        else:
            selected.append(curve)
    if not selected:
        raise ValueError("No complete WIR3D curves remain above the build surface")
    return selected, artifact(
        "style_selection",
        method="whole_curves_above_bed",
        selected=[row["id"] for row in selected],
        excluded=excluded,
        fabrication_time_budget_s=None,
    )


def split_shared_geometry(curves, framework):
    """Partition exact straight overlaps in source parameter space."""
    remaining, shared = [], []
    for curve in curves:
        overlaps = []
        for index, values in enumerate(curve["segments"]):
            control = np.asarray(values)
            start, axis = control[0], control[-1] - control[0]
            squared = float(axis @ axis)
            if squared < 1e-20:
                continue
            projection = (control - start) @ axis / squared
            if np.max(np.linalg.norm(control - start - projection[:, None] * axis, axis=1)) > 1e-10:
                continue
            p = projection
            polynomial = np.array(
                [
                    p[0],
                    3 * (p[1] - p[0]),
                    3 * (p[2] - 2 * p[1] + p[0]),
                    p[3] - 3 * p[2] + 3 * p[1] - p[0],
                ]
            )
            for edge in framework:
                points = np.asarray(edge["segments"])
                ends = points.reshape(-1, 3)
                fractions = (ends - start) @ axis / squared
                if np.max(np.linalg.norm(ends - start - fractions[:, None] * axis, axis=1)) > 1e-10:
                    continue
                lo, hi = sorted((fractions[0], fractions[-1]))
                if hi - lo < 1e-12:
                    continue
                cuts = [0.0, 1.0]
                for boundary in (lo, hi):
                    coefficients = polynomial.copy()
                    coefficients[0] -= boundary
                    cuts.extend(
                        float(root.real)
                        for root in np.polynomial.polynomial.polyroots(coefficients)
                        if abs(root.imag) < 1e-9 and 1e-10 < root.real < 1 - 1e-10
                    )
                cuts = sorted(set(cuts))
                for a, b in zip(cuts[:-1], cuts[1:], strict=True):
                    middle = np.polynomial.polynomial.polyval((a + b) / 2, polynomial)
                    if lo - 1e-10 <= middle <= hi + 1e-10:
                        overlaps.append((index + a, index + b, edge["id"]))
        if not overlaps:
            remaining.append(curve)
            continue
        cuts = sorted(
            {0.0, float(len(curve["segments"])), *(x for a, b, _ in overlaps for x in (a, b))}
        )
        for part, (a, b) in enumerate(zip(cuts[:-1], cuts[1:], strict=True)):
            owner = next(
                (key for lo, hi, key in overlaps if lo - 1e-10 <= (a + b) / 2 <= hi + 1e-10), None
            )
            if owner:
                shared.append(
                    {
                        "source": curve["id"],
                        "source_interval": [a, b],
                        "existing_framework_member": owner,
                    }
                )
            else:
                remaining.append(
                    {
                        **deepcopy(curve),
                        "id": f"{curve['id']}-part{part}",
                        "parent_curve": curve["id"],
                        "source_interval": [a, b],
                        "segments": curve_interval(curve, a, b),
                        "shared_geometry_remainder": True,
                    }
                )
    return remaining, shared


def attach_style(framework, curves, maximum_connector_m=0.035):
    graph = deepcopy(framework)
    nodes = {row["id"]: row for row in graph["nodes"]}
    edges = {row["id"]: row for row in graph["edges"]}
    cuts = {key: {0.0: row["start"], 1.0: row["end"]} for key, row in edges.items()}
    selected_sources = [row["id"] for row in curves]
    required_sources = [
        {"parent_curve": row["id"], "source_interval": [0, len(row["segments"])]} for row in curves
    ]
    aliases = shared_source_bindings(framework)
    curves, shared = split_shared_geometry(curves, list(edges.values()))
    added, artistic, attachments = [], [], []
    last_base_order = max(row["order_hint"] for row in edges.values()) + 1

    def new_node(point):
        identity = f"hybrid-n{len(nodes):05d}"
        nodes[identity] = {
            "id": identity,
            "position": point.tolist(),
            "ground": bool(point[2] <= 1e-7),
        }
        return identity

    def nearest(point):
        candidates = []
        for key, row in edges.items():
            control = np.asarray(row["segments"])
            a, b = control[0, 0], control[-1, -1]
            # This adapter is for the straight members produced by WirePrint.
            if control.shape != (1, 4, 3) or not np.allclose(
                control[0], np.linspace(a, b, 4), atol=1e-10, rtol=0
            ):
                raise ValueError("Framework attachment requires straight source members")
            fraction = float(np.clip(np.dot(point - a, b - a) / np.dot(b - a, b - a), 0, 1))
            position = a + fraction * (b - a)
            candidates.append((float(np.linalg.norm(point - position)), key, fraction, position))
        return min(candidates, key=lambda row: row[0])

    for index, curve in enumerate(curves):
        controls = np.asarray(curve["segments"])
        if not np.isfinite(controls).all():
            raise ValueError("Style curves must contain finite source geometry")
        duplicate = next(
            (
                key
                for key, edge in edges.items()
                if controls.shape == np.asarray(edge["segments"]).shape
                and (
                    np.allclose(controls, edge["segments"], atol=1e-10, rtol=0)
                    or np.allclose(
                        controls, np.asarray(edge["segments"])[::-1, ::-1], atol=1e-10, rtol=0
                    )
                )
            ),
            None,
        )
        if duplicate is not None:
            shared.append(
                {
                    "source": curve["id"],
                    "existing_framework_member": duplicate,
                    "source_interval": [0, len(controls)],
                }
            )
            continue
        endpoints = []
        for end, point in enumerate((controls[0, 0], controls[-1, -1])):
            distance, key, fraction, position = nearest(point)
            row = edges[key]
            # Reuse nearby cuts but retain the artistic endpoint exactly by
            # recording the remaining gap as connecting material.
            for t, identity in cuts[key].items():
                existing = np.asarray(nodes[identity]["position"])
                if np.linalg.norm(existing - position) <= 0.00002:
                    fraction, position = t, existing
                    break
            distance = float(np.linalg.norm(point - position))
            if curve.get("shared_geometry_remainder") and distance > 0.00002:
                # The other endpoint meets the shared base. This remainder is
                # a supported cantilever; adding a coincident connector would
                # deposit its source geometry twice.
                endpoints.append(new_node(point))
                attachments.append(
                    {
                        "source": curve["parent_curve"],
                        "end": end,
                        "attachment_model": "free_endpoint",
                        "connector_length_m": 0.0,
                    }
                )
                continue
            if distance > maximum_connector_m:
                raise ValueError(
                    f"Style source {curve['id']} endpoint {end} needs {distance:.6g} m support"
                )
            anchor = cuts[key].get(fraction)
            if anchor is None:
                anchor = new_node(position)
                cuts[key][fraction] = anchor
            # An endpoint already inside the existing strand is a fused joint,
            # not a new micron-long beam. Keep its curve control points intact;
            # record the small mechanical-node offset explicitly.
            fused = distance <= 0.00002
            tip = anchor if fused else new_node(point)
            endpoints.append(tip)
            if tip != anchor:
                support = straight_edge(
                    anchor, tip, nodes, f"style-support-{index}-{end}", "support", row["diameter_m"]
                )
                support.update(
                    parent_curve=support["id"],
                    source_interval=[0, 1],
                    stage="framework",
                    origin="explicit_style_attachment",
                    order_hint=last_base_order + index / max(1, len(curves)),
                )
                added.append(support)
            attachments.append(
                {
                    "source": curve["id"],
                    "end": end,
                    "framework_member": key,
                    "framework_parameter": fraction,
                    "connector_length_m": 0.0 if fused else distance,
                    "endpoint_offset_m": distance if fused else 0.0,
                    "attachment_model": "fused_overlap" if fused else "explicit_connector",
                }
            )
        artistic.append(
            {
                **deepcopy(curve),
                "start": endpoints[0],
                "end": endpoints[1],
                "parent_curve": curve.get("parent_curve", curve["id"]),
                "source_interval": curve.get("source_interval", [0, len(controls)]),
                "stage": "artistic",
                "role": "artistic",
                "required": True,
                "order_hint": index,
            }
        )

    split, descendants = [], {}
    for key, row in edges.items():
        ordered = sorted(cuts[key])
        lo, hi = row["source_interval"]
        for i, (a, b) in enumerate(zip(ordered[:-1], ordered[1:], strict=True)):
            split.append(
                {
                    **row,
                    "id": f"{key}-part{i}",
                    "start": cuts[key][a],
                    "end": cuts[key][b],
                    "segments": curve_interval(row, a, b),
                    "source_interval": [lo + a * (hi - lo), lo + b * (hi - lo)],
                }
            )
            # Shared intervals retain their complete physical ownership when
            # attachments divide a framework member into multiple pieces.
            split[-1].pop("shared_sources", None)
            descendants.setdefault(key, []).append(split[-1]["id"])
    for alias in aliases:
        alias["members"] = [part for key in alias["members"] for part in descendants[key]]
    aliases.extend(
        {
            "parent_curve": row["source"],
            "source_interval": row["source_interval"],
            "stage": "artistic",
            "members": descendants[row["existing_framework_member"]],
        }
        for row in shared
    )
    for edge in [*split, *added, *artistic]:
        edge["length_m"] = curve_length(edge)
    graph.update(
        nodes=list(nodes.values()),
        edges=split + added + artistic,
        method="wireprint_with_imported_wir3d",
        graph_revision="style_attachments_v1",
        source_aliases=aliases,
        required_artistic_sources=required_sources,
    )
    return graph, artifact(
        "style_attachments",
        attachments=attachments,
        shared_geometry=shared,
        attachment_member_source="framework_graph.json",
        selected_curves=selected_sources,
        selection_policy="all_selected_curves_preserve_source_geometry",
        added_supports=[row["id"] for row in added],
        source_geometry_modified=False,
    )
