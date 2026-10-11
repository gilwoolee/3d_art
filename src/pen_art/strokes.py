"""Continuous pen strokes, distinct from members in the material junction graph."""

from collections import defaultdict
from copy import deepcopy

import numpy as np

from .io import artifact


def pen_strokes(graph):
    """Join consecutive pieces of a source contour without inventing material.

    Junctions remain explicit for ordering, welds and structural analysis. Merely
    crossing one does not command a feed stop. Later planning may split a stroke
    at a real junction if access, cooling or available anchors require it.
    """
    parents = defaultdict(list)
    independent = []
    for edge in graph["edges"]:
        if edge.get("parent_curve") and edge["role"] == "artistic":
            parents[(edge["parent_curve"], edge.get("work_zone"))].append(edge)
        else:
            independent.append([edge])
    groups = []
    for members in parents.values():
        members.sort(key=lambda e: e["source_interval"][0])
        current = []
        for member in members:
            if current and (
                current[-1]["end"] != member["start"]
                or abs(current[-1]["source_interval"][1] - member["source_interval"][0]) > 1e-7
                or not np.allclose(
                    current[-1]["segments"][-1][-1], member["segments"][0][0], atol=1e-10
                )
            ):
                groups.append(current)
                current = []
            current.append(member)
        if current:
            groups.append(current)
    groups.extend(independent)
    strokes = []
    for index, members in enumerate(groups):
        length, segment_count = 0.0, 0
        visits = [{"node": members[0]["start"], "distance_m": 0.0, "segment_boundary": 0}]
        for member in members:
            length += member["length_m"]
            segment_count += len(member["segments"])
            visits.append(
                {"node": member["end"], "distance_m": length, "segment_boundary": segment_count}
            )
        strokes.append(
            {
                "id": f"draw-{index:04}",
                "role": members[0]["role"],
                "parent_curve": members[0].get("parent_curve"),
                "start": members[0]["start"],
                "end": members[-1]["end"],
                "closed": members[0]["start"] == members[-1]["end"],
                "segments": [piece for member in members for piece in member["segments"]],
                "length_m": length,
                "diameter_m": members[0]["diameter_m"],
                "members": [member["id"] for member in members],
                "junction_visits": visits,
            }
        )
        if "work_zone" in members[0]:
            strokes[-1]["work_zone"] = members[0]["work_zone"]
            strokes[-1]["allowed_arms"] = members[0]["allowed_arms"]
            if "allocation_group" in members[0]:
                strokes[-1]["allocation_group"] = members[0]["allocation_group"]
    member_ids = [member for stroke in strokes for member in stroke["members"]]
    if len(member_ids) != len(set(member_ids)) or set(member_ids) != {
        e["id"] for e in graph["edges"]
    }:
        raise ValueError("Every material member must belong to exactly one proposed pen stroke")
    return artifact(
        "pen_strokes",
        frame="build",
        units="m",
        method="continuous_source_contours",
        curves=strokes,
        total_length_m=sum(s["length_m"] for s in strokes),
        material_members=len(member_ids),
        proposed_starts=len(strokes),
        status="proposed_before_construction_search",
        limitations=[
            "Physical junctions do not automatically require extrusion stops",
            "Construction search may rotate a closed stroke or split at a real anchor",
            "Cooling pauses and robot clearance have not yet been planned",
        ],
    )


def orient_stroke(stroke, reverse=False, start_visit=0):
    """Re-root a closed contour at a real junction, preserving its material exactly."""
    result = deepcopy(stroke)
    if start_visit:
        if not stroke["closed"] or not 0 <= start_visit < len(stroke["members"]):
            raise ValueError("Only closed strokes can start at another junction")
        boundary = stroke["junction_visits"][start_visit]["segment_boundary"]
        offset = stroke["junction_visits"][start_visit]["distance_m"]
        result["segments"] = result["segments"][boundary:] + result["segments"][:boundary]
        result["members"] = result["members"][start_visit:] + result["members"][:start_visit]
        visits = stroke["junction_visits"][:-1]
        result["junction_visits"] = []
        for i in list(range(start_visit, len(visits))) + list(range(start_visit)):
            visit = deepcopy(visits[i])
            visit["distance_m"] = (visit["distance_m"] - offset) % stroke["length_m"]
            visit["segment_boundary"] = (visit["segment_boundary"] - boundary) % len(
                stroke["segments"]
            )
            result["junction_visits"].append(visit)
        first = result["junction_visits"][0]
        result["junction_visits"].append(
            {
                "node": first["node"],
                "distance_m": stroke["length_m"],
                "segment_boundary": len(stroke["segments"]),
            }
        )
        result["start"] = result["end"] = first["node"]
    if reverse:
        result["segments"] = [segment[::-1] for segment in result["segments"][::-1]]
        result["members"].reverse()
        result["start"], result["end"] = result["end"], result["start"]
        result["junction_visits"] = [
            {
                "node": visit["node"],
                "distance_m": result["length_m"] - visit["distance_m"],
                "segment_boundary": len(result["segments"]) - visit["segment_boundary"],
            }
            for visit in result["junction_visits"][::-1]
        ]
    result["reverse"] = reverse
    result["start_visit"] = start_visit
    return result
