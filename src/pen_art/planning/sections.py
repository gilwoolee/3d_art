"""Source-preserving construction sections shared by planning and replay."""

from copy import deepcopy

import numpy as np

from ..curves import curve_interval, curve_length
from ..io import artifact
from ..provenance import shared_source_bindings


def split_member_for_restart(graph, member_id, fraction):
    """Split one cubic exactly, creating a stop and subsequent real rejoin.

    The new node is material deposited by the first section, not an external
    support. Ordinary prefix stiffness and cooled-anchor checks remain required.
    Source aliases retain both physical pieces before receiving completion credit.
    """
    if not np.isfinite(fraction) or not 0 < fraction < 1:
        raise ValueError("Restart fraction must lie strictly inside a member")
    result = deepcopy(graph)
    matches = [edge for edge in result["edges"] if edge["id"] == member_id]
    if len(matches) != 1 or len(matches[0]["segments"]) != 1:
        raise ValueError("Restart splitting requires one uniquely identified cubic member")
    edge = matches[0]
    node_id = f"{member_id}-restart-{fraction:.12g}"
    member_ids = [node_id + "-before", node_id + "-after"]
    if node_id in {node["id"] for node in result["nodes"]} or set(member_ids) & {
        item["id"] for item in result["edges"]
    }:
        raise ValueError("Restart boundary already exists")
    left, right = curve_interval(edge, 0, fraction), curve_interval(edge, fraction, 1)
    result["nodes"].append(
        {
            "id": node_id,
            "position": left[-1][-1],
            # The split must not introduce an independent substrate anchor.
            "ground": False,
            "origin": "deposited_restart_point",
        }
    )
    lo, hi = edge.get("source_interval", [0.0, 1.0])
    middle = lo + fraction * (hi - lo)
    pieces = []
    for index, (segments, start, end, interval) in enumerate(
        [
            (left, edge["start"], node_id, [lo, middle]),
            (right, node_id, edge["end"], [middle, hi]),
        ]
    ):
        piece = {
            **edge,
            "id": member_ids[index],
            "start": start,
            "end": end,
            "segments": segments,
            "parent_curve": edge.get("parent_curve", member_id),
            "source_interval": interval,
        }
        piece.pop("shared_sources", None)
        piece["length_m"] = curve_length(piece)
        if index == 0:
            piece["stop_after"] = True
        pieces.append(piece)
    bindings = shared_source_bindings(graph)
    for binding in bindings:
        binding["members"] = [
            replacement
            for original in binding["members"]
            for replacement in (member_ids if original == member_id else [original])
        ]
    result["source_aliases"] = bindings
    index = result["edges"].index(edge)
    result["edges"][index : index + 1] = pieces
    result.setdefault("restart_boundaries", []).append(
        {"member": member_id, "fraction": fraction, "node": node_id, "pieces": member_ids}
    )
    return result


def section_actions(graph, *, reference=False, group_sources=True):
    """Keep a connected source curve together without inventing anchors.

    Material junctions, original zigzag vertices and cubic subdivision points
    remain bookkeeping boundaries, not pen stops. Only a real discontinuity or
    an explicit restart splits the ordinary source grouping.
    """
    nodes = {node["id"]: node for node in graph["nodes"]}
    groups = [[edge] for edge in graph["edges"]]
    if group_sources:
        by_source = {}
        for edge in graph["edges"]:
            by_source.setdefault(edge.get("parent_curve", edge["id"]), []).append(edge)
        groups = []
        for source_edges in by_source.values():
            current = []
            for edge in sorted(source_edges, key=lambda row: row.get("source_interval", [0])[0]):
                if current and (
                    current[-1]["end"] != edge["start"]
                    or current[-1].get("stop_after", False)
                    or not np.allclose(
                        current[-1]["segments"][-1][-1],
                        edge["segments"][0][0],
                        atol=1e-10,
                        rtol=0,
                    )
                    or current[-1].get("stage") != edge.get("stage")
                    or current[-1]["diameter_m"] != edge["diameter_m"]
                ):
                    groups.append(current)
                    current = []
                current.append(edge)
            groups.append(current)
    actions, previous_sources = [], {}
    for index, group in enumerate(groups):
        edge = group[0]
        row = deepcopy(edge)
        # Shared sources belong to a graph-level binding that may span several
        # members/sections. Copying only the first member's aliases loses or
        # prematurely credits the remainder; export audits the complete binding.
        row.pop("shared_sources", None)
        row.update(
            id=f"section-{index:05}",
            members=[member["id"] for member in group],
            end=group[-1]["end"],
            segments=[segment for member in group for segment in member["segments"]],
            parent_curve=edge.get("parent_curve", edge["id"]),
            source_interval=[
                edge.get("source_interval", [0.0])[0],
                group[-1].get("source_interval", [0.0, len(edge["segments"])])[1],
            ],
            # Fused endpoint adjustments may leave tiny gaps in the original
            # source parameter while the physical curve remains connected.
            # Keep exact intervals; the envelope alone would invent coverage.
            source_intervals=[
                member.get("source_interval", [0.0, len(member["segments"])]) for member in group
            ],
            stage="framework" if reference else edge.get("stage", "artistic"),
            closed=edge["start"] == group[-1]["end"],
            length_m=sum(curve_length(member) for member in group),
            depends_on=[],
        )
        visits = [{"node": row["start"], "distance_m": 0.0, "segment_boundary": 0}]
        distance, boundary = 0.0, 0
        for member in group:
            distance += curve_length(member)
            boundary += len(member["segments"])
            visits.append(
                {"node": member["end"], "distance_m": distance, "segment_boundary": boundary}
            )
        row["junction_visits"] = visits
        previous = previous_sources.get(row["parent_curve"])
        if previous is not None:
            row["separation_before"] = (
                "explicit_restart"
                if previous.get("stop_after", False)
                else "disconnected_nodes"
                if previous["end"] != edge["start"]
                else "disconnected_geometry"
                if not np.allclose(
                    previous["segments"][-1][-1], edge["segments"][0][0], atol=1e-10, rtol=0
                )
                else "stage_change"
                if previous.get("stage") != edge.get("stage")
                else "strand_diameter_change"
                if previous["diameter_m"] != edge["diameter_m"]
                else "explicit_member_grouping"
            )
        previous_sources[row["parent_curve"]] = group[-1]
        row["priority"] = edge.get(
            "order_hint", max(nodes[row[k]]["position"][2] for k in ("start", "end"))
        )
        actions.append(row)
    return artifact(
        "construction_plan",
        actions=actions,
        method="connected_source_sections" if group_sources else "independent_source_sections",
        status="UNPLANNED",
        simulation_only=True,
    )


def coverage_audit(actions, expected):
    """Check multiplicity in source parameter space, independently of action order."""

    def intervals(rows):
        result = {}
        for row in rows:
            for lo, hi in row.get("source_intervals", [row["source_interval"]]):
                if not np.isfinite([lo, hi]).all() or hi <= lo:
                    raise ValueError("Source intervals must be finite and increasing")
                result.setdefault(row["parent_curve"], []).append((float(lo), float(hi)))
        return {key: sorted(value) for key, value in result.items()}

    actual, target = intervals(actions), intervals(expected)
    if actual.keys() != target.keys():
        raise ValueError("Source coverage has missing or unexpected curves")
    for source, spans in actual.items():
        for (_, end), (start, _) in zip(spans[:-1], spans[1:], strict=True):
            if start < end - 1e-10:
                raise ValueError(f"Overlapping source deposition: {source}")

        # Compare union boundaries, allowing a source interval to be split further.
        def union(parts):
            merged = []
            for lo, hi in parts:
                if merged and abs(lo - merged[-1][1]) <= 1e-10:
                    merged[-1] = (merged[-1][0], hi)
                else:
                    merged.append((lo, hi))
            return merged

        a, b = union(spans), union(target[source])
        if len(a) != len(b) or not np.allclose(a, b, atol=1e-10, rtol=0):
            raise ValueError(f"Source coverage gap or extension: {source}")
    return {"status": "EXACT_INTERVAL_COVERAGE", "sources": len(target), "sections": len(actions)}


def replay_sections(actions, graph, *, cooling_s=2.0):
    """Verify staged anchor readiness and once-only membership on a saved timeline."""
    from .clock_path import ClockPath
    from .process_clock import PROCESS_CLOCK_POLICY, action_ready_times

    if not np.isfinite(cooling_s) or cooling_s < 0:
        raise ValueError("Cooling duration must be finite and nonnegative")
    ready = {node["id"]: 0.0 for node in graph["nodes"] if node["ground"]}
    framework = {row["id"] for row in actions if row["stage"] == "framework"}
    done, members = set(), set()
    previous_end = 0.0
    framework_ready = 0.0
    for row in actions:
        if (
            not np.isfinite([row["start_s"], row["end_s"]]).all()
            or row["start_s"] < 0
            or row["end_s"] <= row["start_s"]
        ):
            raise ValueError("Section times must be finite, positive intervals")
        if row["id"] in done or members.intersection(row["members"]):
            raise ValueError("A construction section was deposited twice")
        if row["stage"] == "artistic" and not framework <= done:
            raise ValueError("Artistic deposition precedes framework completion")
        if row["stage"] == "artistic" and row["start_s"] < framework_ready - 1e-9:
            raise ValueError("Artistic deposition precedes framework cooling")
        if row["start_s"] < previous_end - 1e-9:
            raise ValueError("Single-arm sections overlap on the clock")
        # A section chained to its predecessor continues with the pen down, so
        # its joint must exist but need not have cooled; the pairing itself is
        # checked on the saved clocks by the replay.
        chained = bool(row.get("trajectory", {}).get("chained_in"))
        available = ready.get(row["start_anchor"], float("inf"))
        if not np.isfinite(available) or (not chained and available > row["start_s"] + 1e-9):
            raise ValueError("Section starts on absent or still-cooling material")
        visits = {visit["node"] for visit in row.get("junction_visits", [])}
        anchors = visits if row.get("closed") else {row["start"], row["end"]}
        if row["start_anchor"] not in anchors:
            raise ValueError("Start anchor does not belong to the section")
        done.add(row["id"])
        members.update(row["members"])
        previous_end = row["end_s"]
        path = ClockPath.from_record(row["trajectory"]) if "trajectory" in row else None
        if path is not None and path.metadata.get("process_clock_policy") == PROCESS_CLOCK_POLICY:
            events = path.metadata.get("deposition_events", [])
            expected = list(row.get("junction_visits", []))
            start_visit = path.metadata.get("source_start_visit", 0)
            if start_visit:
                unique = expected[:-1]
                expected = unique[start_visit:] + unique[:start_visit] + [unique[start_visit]]
            if path.metadata.get("source_reverse", False):
                expected = expected[::-1]
            draw_times = [edge.end_s for edge in path.edges if edge.kind == "DRAW"]
            draw_times += [edge.start_s for edge in path.edges if edge.kind == "DRAW"]
            if len(events) != len(expected):
                raise ValueError("Material clock omits source junctions")
            for event, visit in zip(events, expected, strict=True):
                if (
                    event["node"] != visit["node"]
                    or not any(abs(event["deposited_at_s"] - time) < 1e-8 for time in draw_times)
                    or abs(event["ready_at_s"] - event["deposited_at_s"] - cooling_s) > 1e-8
                ):
                    raise ValueError("Invalid source deposition/readiness clock")
            relative = action_ready_times(path, row, cooling_s)
            if row["stage"] == "framework":
                framework_ready = max(framework_ready, row["start_s"] + max(relative.values()))
            for key, time in relative.items():
                ready[key] = min(ready.get(key, float("inf")), row["start_s"] + time)
        else:
            if row["stage"] == "framework":
                framework_ready = max(framework_ready, row["end_s"] + cooling_s)
            for key in visits | {row["start"], row["end"]}:
                ready[key] = min(ready.get(key, float("inf")), row["end_s"] + cooling_s)
    return {"status": "STAGED_ANCHORS_VERIFIED", "completed_sections": len(done)}
