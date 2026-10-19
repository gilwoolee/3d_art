"""Source-preserving construction sections shared by planning and replay, and
the audits that every source interval is drawn exactly once."""

from collections import Counter
from copy import deepcopy

import numpy as np

from ..curves import curve_length
from ..io import artifact


def section_actions(graph):
    """Keep a connected source curve together without inventing anchors.

    Material junctions, original zigzag vertices and cubic subdivision points
    remain bookkeeping boundaries, not pen stops. Only a real discontinuity
    splits the ordinary source grouping.
    """
    nodes = {node["id"]: node for node in graph["nodes"]}
    by_source = {}
    for edge in graph["edges"]:
        by_source.setdefault(edge.get("parent_curve", edge["id"]), []).append(edge)
    groups = []
    for source_edges in by_source.values():
        current = []
        for edge in sorted(source_edges, key=lambda row: row.get("source_interval", [0])[0]):
            if current and (
                current[-1]["end"] != edge["start"]
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
    actions = []
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
            stage=edge.get("stage", "artistic"),
            closed=edge["start"] == group[-1]["end"],
            length_m=sum(curve_length(member) for member in group),
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
        row["priority"] = edge.get(
            "order_hint", max(nodes[row[k]]["position"][2] for k in ("start", "end"))
        )
        actions.append(row)
    return artifact(
        "construction_plan",
        actions=actions,
        method="connected_source_sections",
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
        path = ClockPath.from_record(row["trajectory"])
        if path.metadata.get("process_clock_policy") != PROCESS_CLOCK_POLICY:
            raise ValueError("Section clock has no material readiness")
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
        relative = action_ready_times(path)
        if row["stage"] == "framework":
            framework_ready = max(framework_ready, row["start_s"] + max(relative.values()))
        for key, time in relative.items():
            ready[key] = min(ready.get(key, float("inf")), row["start_s"] + time)
    return {"status": "STAGED_ANCHORS_VERIFIED", "completed_sections": len(done)}


def shared_source_bindings(graph):
    """Return one credit per source interval that several members share.

    A shared interval is credited only after all its members are deposited.
    """
    grouped = {}
    for edge in graph["edges"]:
        for alias in edge.get("shared_sources", []):
            key = (alias["parent_curve"], *alias["source_interval"])
            binding = grouped.setdefault(
                key,
                {
                    "parent_curve": alias["parent_curve"],
                    "source_interval": alias["source_interval"],
                    "stage": edge.get("stage", "framework"),
                    "members": [],
                },
            )
            binding["members"].append(edge["id"])
    return list(grouped.values())


def audit_source_provenance(actions, graph):
    """Check primary and shared source coverage against the bound material graph."""
    edges = graph["edges"]
    expected_members = {edge["id"] for edge in edges}
    if len(expected_members) != len(edges):
        raise ValueError("Duplicate graph member IDs")
    owners = {}
    counts = Counter()
    for index, action in enumerate(actions):
        for member in action["members"]:
            counts[member] += 1
            owners[member] = (index, action)
    if set(counts) != expected_members or any(count != 1 for count in counts.values()):
        raise ValueError("Source provenance requires every physical member exactly once")
    bindings = shared_source_bindings(graph)
    credits = []
    for binding in bindings:
        members = binding["members"]
        if not members or len(set(members)) != len(members) or not set(members) <= expected_members:
            raise ValueError("Shared source refers to missing or duplicate physical members")
        _, last = max((owners[member] for member in members), key=lambda item: item[0])
        credit = {**binding, "credited_after_section": last["id"]}
        if "end_s" in last:
            credit["deposition_complete_s"] = last["end_s"]
        credits.append(credit)
    expected = [
        {
            "parent_curve": edge.get("parent_curve", edge["id"]),
            "source_interval": edge.get("source_interval", [0, len(edge["segments"])]),
        }
        for edge in edges
    ]
    coverage = coverage_audit([*actions, *bindings], [*expected, *bindings])
    return {
        "status": "SOURCE_PROVENANCE_VERIFIED",
        "coverage": {
            "status": coverage["status"],
            "sources": coverage["sources"],
            "represented_intervals": coverage["sections"],
        },
        "physical_sections": len(actions),
        "physical_members": len(expected_members),
        "shared_intervals": credits,
    }
