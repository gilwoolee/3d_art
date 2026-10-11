"""Bounded backward search for a grounded, continuous-pen construction preview.

This is the geometry portion of regression planning. It does not substitute for
coupled arm-path search, calibrated deposition, or intermediate stiffness checks.
"""

from copy import deepcopy
from time import monotonic

import networkx as nx

from .graph import topology
from .io import artifact
from .strokes import orient_stroke


def prefix_grounded(graph, member_ids):
    nodes = {node["id"]: node for node in graph["nodes"]}
    return all(
        any(nodes[node]["ground"] for node in component)
        for component in nx.connected_components(topology(graph, member_ids))
    )


def split_stroke(stroke, visit_index):
    """Split at an actual material junction, retaining all members and geometry."""
    visit = stroke["junction_visits"][visit_index]
    boundary, distance = visit["segment_boundary"], visit["distance_m"]
    if not 0 < visit_index < len(stroke["members"]):
        raise ValueError("Split must be at an interior physical junction")
    pieces = []
    for side in (0, 1):
        piece = deepcopy(stroke)
        piece["id"] = f"{stroke['id']}:split-{visit_index}-{side}"
        piece["closed"] = False
        if side == 0:
            piece["segments"] = piece["segments"][:boundary]
            piece["members"] = piece["members"][:visit_index]
            piece["junction_visits"] = piece["junction_visits"][: visit_index + 1]
            piece["end"] = visit["node"]
            piece["length_m"] = distance
        else:
            piece["segments"] = piece["segments"][boundary:]
            piece["members"] = piece["members"][visit_index:]
            piece["junction_visits"] = [
                {
                    "node": v["node"],
                    "distance_m": v["distance_m"] - distance,
                    "segment_boundary": v["segment_boundary"] - boundary,
                }
                for v in piece["junction_visits"][visit_index:]
            ]
            piece["start"] = visit["node"]
            piece["length_m"] -= distance
        pieces.append(piece)
    return pieces


def start_options(stroke, available, nodes, limit, allow_splitting=True):
    if stroke["start"] in available:
        yield [orient_stroke(stroke)]
    if stroke["end"] in available and stroke["start"] != stroke["end"]:
        yield [orient_stroke(stroke, reverse=True)]
    interior = [
        i for i, v in enumerate(stroke["junction_visits"][1:-1], 1) if v["node"] in available
    ]
    interior.sort(key=lambda i: (nodes[stroke["junction_visits"][i]["node"]]["position"][2], i))
    for index in interior[:limit]:
        if stroke["closed"]:
            yield [orient_stroke(stroke, start_visit=index)]
        elif allow_splitting:
            first, second = split_stroke(stroke, index)
            # Draw away from the shared anchor; return with feed off for the other half.
            yield [orient_stroke(first, reverse=True), second]


def construction_preview(
    graph, strokes, maximum_states=2000, timeout_s=20.0, *, allow_splitting=True
):
    nodes = {node["id"]: node for node in graph["nodes"]}
    by_id = {stroke["id"]: stroke for stroke in strokes["curves"]}
    full = frozenset(by_id)
    member_sets = {key: set(stroke["members"]) for key, stroke in by_id.items()}
    grounded = {key for key, node in nodes.items() if node["ground"]}
    deadline = monotonic() + timeout_s
    states, failed = 0, set()
    base = artifact(
        "construction_plan",
        algorithm="bounded_regression_geometry_preview",
        frame="build",
        units="m",
        simulation_only=True,
        robot_feasible=None,
        pen_feasible=None,
        stiffness_feasible=None,
        limitations=[
            "Valid load-path topology does not establish stiffness or hot-strand feasibility",
            "No robot path, tool collision, transit, or calibrated timing has been validated",
            "Geometry-only cache keys must be extended with robot/process state in coupled search",
        ],
    )
    if len(full) > 500:
        return {
            **base,
            "status": "NEEDS_SIMPLIFICATION",
            "actions": [],
            "message": "Simplify to at most 500 proposed strokes before regression search",
        }
    if not graph["checks"]["material_connected"]:
        return {**base, "status": "DISCONNECTED_MATERIAL", "actions": []}

    def members_for(remaining):
        return set().union(*(member_sets[key] for key in remaining))

    def search(remaining):
        nonlocal states
        if not remaining:
            return []
        if states >= maximum_states or monotonic() >= deadline:
            raise TimeoutError
        if remaining in failed:
            return None
        states += 1
        candidates = sorted(
            remaining,
            key=lambda key: (
                max(nodes[v["node"]]["position"][2] for v in by_id[key]["junction_visits"]),
                key,
            ),
            reverse=True,
        )
        for identity in candidates:
            prefix = remaining - {identity}
            members = members_for(prefix)
            if not prefix_grounded(graph, members):
                continue
            available = set(topology(graph, members).nodes) | grounded
            options = list(
                start_options(
                    by_id[identity], available, nodes, limit=6, allow_splitting=allow_splitting
                )
            )
            if not options:
                continue
            earlier = search(prefix)
            if earlier is not None:
                # All these orientations have the same earlier geometry. Coupled robot
                # search must retain/revisit alternatives when a motion path fails.
                group = options[0]
                for action in group:
                    action["source_stroke"] = identity
                    action["alternative_starts"] = [option[0]["start"] for option in options]
                return earlier + group
        failed.add(remaining)
        return None

    try:
        actions = search(full)
    except TimeoutError:
        actions = None
    if actions is None:
        return {
            **base,
            "status": "NO_PLAN_FOUND_WITHIN_BUDGET",
            "states": states,
            "actions": [],
            "message": "Revise anchors, split options or the search budget",
        }
    printed, births, replay = set(), {}, []
    for index, action in enumerate(actions):
        if action["start"] not in grounded and action["start"] not in births:
            raise ValueError("Construction replay found an unavailable starting anchor")
        if printed.intersection(action["members"]):
            raise ValueError("Construction replay would print a member twice")
        action["index"] = index
        action["depends_on"] = [] if action["start"] in grounded else [births[action["start"]]]
        action["existing_join_nodes"] = sorted(
            {v["node"] for v in action["junction_visits"] if v["node"] in births}
        )
        printed.update(action["members"])
        if not prefix_grounded(graph, printed):
            raise ValueError("Construction replay contains a floating component")
        for visit in action["junction_visits"]:
            births.setdefault(visit["node"], index)
        replay.append(
            {"after_action": index, "printed_members": len(printed), "grounded_topology": True}
        )
    if printed != {edge["id"] for edge in graph["edges"]}:
        raise ValueError("Construction replay does not cover the whole material graph")
    return {
        **base,
        "status": "GEOMETRY_PREVIEW",
        "states": states,
        "actions": actions,
        "prefixes": replay,
        "total_length_m": sum(a["length_m"] for a in actions),
        "continuous_strokes": len(actions),
        "all_members_covered_once": True,
    }
