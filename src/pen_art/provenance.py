"""Source intervals represented by shared physical material."""

from collections import Counter
from copy import deepcopy


def shared_source_bindings(graph):
    """Return one credit per source interval, including legacy member annotations.

    A shared interval can require several physical members after splitting.
    It is credited only after all those members have been deposited.
    """
    if "source_aliases" in graph:
        return deepcopy(graph["source_aliases"])
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
    from .planning.sections import coverage_audit

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
    artistic = graph.get("required_artistic_sources")
    if artistic is not None:
        coverage_audit(
            [row for row in actions if row["stage"] == "artistic"]
            + [row for row in bindings if row["stage"] == "artistic"],
            artistic,
        )
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
