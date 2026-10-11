"""Assign intact structural groups to two arm-side regions or shared work.

This preserves cubic geometry and material connectivity. Region ownership is a
planning constraint, not a collision certificate or permission to start at an
unprinted cut point.
"""

import argparse
from collections import Counter
from copy import deepcopy
from pathlib import Path

import networkx as nx
import numpy as np

from ..construction import construction_preview
from ..curves import curve_coordinate_bounds, flatten_curve
from ..graph import topology
from ..io import artifact, read_json, sha256, stage_update, write_json
from ..strokes import pen_strokes


def structural_units(graph, arms, normal, center, seam_width_m):
    """Keep contours, cycle blocks and possible level rims on one resource.

    A level band is conservatively grouped, not declared to be a closed or
    mechanically sound ring. No material or new cut junctions are introduced.
    """
    groups = nx.Graph()
    edges = {e["id"]: e for e in graph["edges"]}
    groups.add_nodes_from(edges)
    strokes = pen_strokes(graph)["curves"]
    for stroke in strokes:
        nx.add_path(groups, stroke["members"])
    cycles = []
    for vertices in nx.biconnected_components(nx.Graph(topology(graph))):
        members = [
            e["id"] for e in edges.values() if e["start"] in vertices and e["end"] in vertices
        ]
        if len(members) >= len(vertices):
            nx.add_path(groups, members)
            cycles.append(members)
    level = []
    for stroke in strokes:
        bounds = curve_coordinate_bounds(stroke)
        tolerance = 3 * stroke["diameter_m"]
        if stroke["role"] == "artistic" and bounds[1, 2] - bounds[0, 2] < tolerance:
            level.append((stroke, bounds, tolerance))
    bands = []
    for i, (first, a, tol) in enumerate(level):
        for second, b, other_tol in level[i + 1 :]:
            if max(a[0, 2], b[0, 2]) - min(a[1, 2], b[1, 2]) <= min(tol, other_tol):
                nx.add_path(groups, [first["members"][0], second["members"][0]])
                bands.append([first["id"], second["id"]])
    result = deepcopy(graph)
    by_id = {e["id"]: e for e in result["edges"]}
    units = []
    for index, members in enumerate(sorted(nx.connected_components(groups), key=min)):
        projected = {"segments": []}
        for key in members:
            p = np.asarray(edges[key]["segments"])
            values = (p - center) @ normal
            controls = np.zeros((*values.shape, 3))
            controls[..., 0] = values
            projected["segments"].extend(controls.tolist())
        bounds = curve_coordinate_bounds(projected)[:, 0]
        zone = (
            arms[0]["id"]
            if bounds[1] < -seam_width_m / 2
            else (arms[1]["id"] if bounds[0] > seam_width_m / 2 else "seam")
        )
        identity = f"structural-unit-{index:03}"
        for key in members:
            by_id[key].update(
                work_zone=zone,
                allocation_group=identity,
                allowed_arms=[zone] if zone != "seam" else [a["id"] for a in arms],
            )
        units.append(
            {
                "id": identity,
                "members": sorted(members),
                "work_zone": zone,
                "length_m": sum(edges[k]["length_m"] for k in members),
            }
        )
    result["partition"] = {
        "normal_build": normal.tolist(),
        "center_build": center.tolist(),
        "seam_width_m": seam_width_m,
        "join_nodes": [],
        "split_method": "whole contours and structural groups; weighted-median region boundary",
        "preserve_structural_units": True,
        "structural_units": units,
        "cycle_blocks_preserved": len(cycles),
        "potential_level_band_pairs": bands,
        "policy": "Whole side units may overlap; shared units reserve both arms; no new cuts",
    }
    return result


def partition_graph(graph, cell, seam_width_m=0.02):
    """Use the robot base baseline as the normal of a centered vertical split."""
    if not np.isfinite(seam_width_m) or seam_width_m <= 0:
        raise ValueError("The shared seam width must be positive")
    arms = cell["arms"]
    if len(arms) != 2:
        raise ValueError("A geometric half split requires two arms")
    transform = np.asarray(cell["world_from_build"])
    bases = [np.asarray(a["world_from_base"])[:3, 3] for a in arms]
    normal = transform[:3, :3].T @ (bases[1] - bases[0])
    normal[2] = 0
    if np.linalg.norm(normal) < 1e-8:
        raise ValueError("The arm bases need horizontal separation in the build frame")
    normal /= np.linalg.norm(normal)
    points = np.array(
        [
            p
            for edge in graph["edges"]
            if edge["role"] == "artistic"
            for piece in edge["segments"]
            for p in piece
        ]
    )
    if not len(points):
        points = np.array([n["position"] for n in graph["nodes"]])
    center = (points.min(axis=0) + points.max(axis=0)) / 2
    # One weighted-median calculation balances drawing length without searching
    # alternative partitions. Base/support material is part of the workload.
    projections, weights = [], []
    for edge in graph["edges"]:
        samples, _ = flatten_curve(edge)
        projections.extend(((samples[:-1] + samples[1:]) / 2) @ normal)
        weights.extend(np.linalg.norm(np.diff(samples, axis=0), axis=1))
    order = np.argsort(projections)
    cumulative = np.cumsum(np.asarray(weights)[order])
    median = np.asarray(projections)[order[np.searchsorted(cumulative, cumulative[-1] / 2)]]
    center += normal * (median - center @ normal)
    return structural_units(graph, arms, normal, center, seam_width_m)


def partition_run(run, seam_width_m=0.02):
    run = Path(run)
    graph = partition_graph(
        read_json(run / "stroke_graph.json"), read_json(run / "cell_config.json"), seam_width_m
    )
    strokes = pen_strokes(graph)
    write_json(run / "partitioned_graph.json", graph)
    # Ownership labels do not change geometry. Preserve an already established
    # whole-stroke order instead of running another bounded search that can lose it.
    existing = run / "construction_plan.json"
    construction = read_json(existing) if existing.is_file() else None
    reuse = construction is not None and (
        construction.get("inputs", {}).get("stroke_graph.json") == sha256(run / "stroke_graph.json")
        and Counter(m for a in construction["actions"] for m in a["members"])
        == Counter(e["id"] for e in graph["edges"])
    )
    if reuse:
        members = {e["id"]: e for e in graph["edges"]}
        for action in construction["actions"]:
            metadata = members[action["members"][0]]
            if any(
                members[m]["allocation_group"] != metadata["allocation_group"]
                for m in action["members"]
            ):
                raise ValueError("A saved contour crosses structural ownership groups")
            action.update(
                {k: metadata[k] for k in ("work_zone", "allowed_arms", "allocation_group")}
            )
        construction["inputs"] = {
            "partitioned_graph.json": sha256(run / "partitioned_graph.json"),
            "construction_plan.json": sha256(existing),
        }
    else:
        construction = construction_preview(graph, strokes, timeout_s=15, allow_splitting=False)
    zones = []
    for name in [a["id"] for a in read_json(run / "cell_config.json")["arms"]] + ["seam"]:
        selected = [c for c in strokes["curves"] if c["work_zone"] == name]
        zones.append(
            {
                "id": name,
                "stroke_count": len(selected),
                "length_m": sum(c["length_m"] for c in selected),
            }
        )
    report = artifact(
        "geometric_partition",
        status="STRUCTURE_PRESERVING_PARTITION_PREVIEW",
        simulation_only=True,
        executable=False,
        frame="build",
        units="m",
        inputs={name: sha256(run / name) for name in ("stroke_graph.json", "cell_config.json")},
        **graph["partition"],
        zones=zones,
        curves=strokes["curves"],
        source_length_m=read_json(run / "stroke_graph.json")["metrics"]["total_length_m"],
        partitioned_length_m=strokes["total_length_m"],
        construction_status=construction["status"],
        construction_actions=len(construction["actions"]),
        construction_reused=reuse,
        limitations=[
            "Region separation does not certify full-arm clearance",
            "Whole contours and cycle groups remain intact; grouping does not prove stiffness",
            "Stiffness, thermal joins, tool approach and trajectories remain unverified",
        ],
    )
    write_json(run / "partitioned_pen_strokes.json", strokes)
    write_json(run / "partitioned_construction.json", construction)
    write_json(run / "geometric_partition.json", report)
    stage_update(
        run,
        "partition",
        "partial",
        artifact="geometric_partition.json",
        message="Whole structural units assigned to arm-side regions or shared work",
    )
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--seam-width-m", type=float, default=0.02)
    args = parser.parse_args()
    result = partition_run(args.run, args.seam_width_m)
    print(result["status"], result["zones"], result["construction_status"])
