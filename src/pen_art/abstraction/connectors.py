"""Connector wire on the bed, so the frame can be drawn in as few trails as possible."""

from copy import deepcopy

import networkx as nx
import numpy as np
from shapely.geometry import Point, Polygon

from ..graph import topology


def _trails(graph):
    """Fewest pen-down trails that draw every piece once, and the odd nodes.

    Euler's rule: a connected frame is one trail exactly when at most two of
    its nodes have an odd number of pieces; in general it takes half the
    number of odd nodes.
    """
    network = topology(graph)
    odd = sum(1 for _, degree in network.degree() if degree % 2)
    trails = sum(
        max(1, sum(1 for node in part if network.degree(node) % 2) // 2)
        for part in nx.connected_components(network)
    )
    return trails, odd


def _straight_pieces(points):
    return [
        [a.tolist(), (a + (b - a) / 3).tolist(), (a + 2 * (b - a) / 3).tolist(), b.tolist()]
        for a, b in zip(points[:-1], points[1:], strict=True)
    ]


def _bowed_arc(points, outline, bow_m):
    """The outline between two nodes, pulled inside it by up to ``bow_m``.

    Each point moves toward the nearest point of the outline shrunk by
    ``bow_m``, by a share that is zero at both nodes and full halfway, so the
    connector leaves and joins the outline at the nodes and runs clear of it
    between. Following the shrunk outline keeps the path from folding back on
    itself at convex corners.
    """
    inner = outline.buffer(-bow_m, join_style=2).exterior
    lengths = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    s = np.r_[np.arange(0.0, lengths[-1], 0.001), lengths[-1]]
    dense = np.column_stack([np.interp(s, lengths, points[:, k]) for k in range(3)])
    result = dense.copy()
    for i in range(1, len(dense) - 1):
        target = inner.interpolate(inner.project(Point(*dense[i, :2])))
        share = np.sin(np.pi * s[i] / s[-1])
        result[i, :2] += share * (np.array([target.x, target.y]) - dense[i, :2])
    # Points pulled toward a corner of the shrunk outline crowd together; keep
    # them at least half a millimetre apart so no wire piece is a stub.
    kept = [result[0]]
    for point in result[1:-1]:
        if np.linalg.norm(point - kept[-1]) >= 0.0005:
            kept.append(point)
    if len(kept) > 1 and np.linalg.norm(result[-1] - kept[-1]) < 0.0005:
        kept.pop()
    return np.vstack([*kept, result[-1]])


def eulerize_frame(graph, budget_m, bow_m=0.003):
    """Add connector wire so the frame can be drawn in as few trails as possible.

    Each base line ends on the bed outline at a node where an odd number of
    pieces meet. Neighbouring ends along the outline are paired by a
    minimum-weight perfect matching and joined by connector wire bowed just
    inside the outline, so a connector never lies on other wire. The bed
    (outline, base lines and connectors) then becomes one closed trail from
    the node where the walls start, ordered by Hierholzer's algorithm, and is
    drawn as one stroke. Odd nodes above the bed are left as they are and
    reported.
    """
    graph = deepcopy(graph)
    nodes = {node["id"]: np.asarray(node["position"], float) for node in graph["nodes"]}
    trails_before, odd_before = _trails(graph)
    network = topology(graph)
    odd = {node for node, degree in network.degree() if degree % 2}
    diameter = graph["generation"]["strand_diameter_m"]

    def on_bed(node):
        return nodes[node][2] <= 1e-9

    bed_edges = [e for e in graph["edges"] if on_bed(e["start"]) and on_bed(e["end"])]
    walls = {
        node
        for e in graph["edges"]
        if not (on_bed(e["start"]) and on_bed(e["end"]))
        for node in (e["start"], e["end"])
        if on_bed(node)
    }
    left, connectors = [], []
    rings = {}
    for edge in bed_edges:
        if edge.get("parent_curve", "").startswith("contour-"):
            rings.setdefault(edge["parent_curve"], []).append(edge)
    for edges in rings.values():
        edges = sorted(edges, key=lambda e: e.get("source_interval", [0])[0])
        order = [edges[0]["start"]] + [e["end"] for e in edges]
        if order[0] != order[-1]:
            continue
        ring = set(order)
        points = [nodes[order[0]]]
        index = {order[0]: 0}
        for edge in edges:
            for segment in edge["segments"]:
                points.append(np.asarray(segment[-1], float))
            index.setdefault(edge["end"], len(points) - 1)
        points = np.asarray(points)
        outline = Polygon(points[:, :2])
        if outline.buffer(-bow_m).is_empty:
            continue  # Too small to fit a connector inside its outline.
        attached = {
            node
            for e in bed_edges
            if e["parent_curve"] not in rings or e["start"] not in ring or e["end"] not in ring
            for node in (e["start"], e["end"])
            if node in ring
        }
        junctions = [node for node in order[:-1] if node in attached]
        pairs = nx.Graph()
        for a, b in zip(junctions, junctions[1:] + junctions[:1], strict=True):
            if a in odd and b in odd and a not in walls and b not in walls and a != b:
                i, j = index[a], index[b]
                arc = points[i : j + 1] if i < j else np.vstack([points[i:-1], points[: j + 1]])
                length = float(np.linalg.norm(np.diff(arc, axis=0), axis=1).sum())
                pairs.add_edge(a, b, weight=length, arc=arc)
        matched = nx.min_weight_matching(pairs)
        for a, b in matched:
            arc = pairs[a][b]["arc"]
            if not np.allclose(arc[0], nodes[a]):
                a, b = b, a
            connectors.append((a, b, _bowed_arc(arc, outline, bow_m)))
        paired = {node for pair in matched for node in pair}
        left += [
            {"node": node, "reason": "no free neighbour on its outline"}
            for node in junctions
            if node in odd and node not in walls and node not in paired
        ]
    added = sum(float(np.linalg.norm(np.diff(c[2], axis=0), axis=1).sum()) for c in connectors)
    total = sum(edge["length_m"] for edge in graph["edges"])
    if connectors and total + added > budget_m:
        left += [{"node": n, "reason": "wire budget"} for a, b, _ in connectors for n in (a, b)]
        connectors, added = [], 0.0
    hint = min(e.get("order_hint", 0) for e in bed_edges) if bed_edges else 0
    for number, (a, b, path) in enumerate(connectors):
        # A chain of straight members, like every other wire, through new
        # nodes on the bed.
        ids = [a]
        for k, point in enumerate(path[1:-1]):
            ids.append(f"euler-{number:03}-n{k:03}")
            graph["nodes"].append({"id": ids[-1], "position": point.tolist(), "ground": True})
            nodes[ids[-1]] = np.asarray(point, float)
        ids.append(b)
        for k, (start, end) in enumerate(zip(ids[:-1], ids[1:], strict=True)):
            piece = path[k : k + 2]
            graph["edges"].append(
                {
                    "id": f"euler-{number:03}-m{k:03}",
                    "start": start,
                    "end": end,
                    "role": "framework",
                    "origin": "euler_connector",
                    "required": True,
                    "diameter_m": diameter,
                    "segments": _straight_pieces(piece),
                    "length_m": float(np.linalg.norm(piece[1] - piece[0])),
                    "process_status": "not_checked",
                    "parent_curve": f"euler-{number:03}",
                    "source_interval": [k, k + 1],
                    "stage": "framework",
                    "order_hint": hint,
                }
            )
    left += [
        {"node": node, "reason": "above the bed"}
        for node in sorted(odd)
        if not on_bed(node) and node != max(odd, key=lambda n: nodes[n][2])
    ]
    _order_bed_trail(graph, walls & odd, on_bed)
    trails_after, odd_after = _trails(graph)
    graph["generation"]["eulerization"] = {
        "odd_nodes_before": odd_before,
        "odd_nodes_after": odd_after,
        "minimum_trails_before": trails_before,
        "minimum_trails_after": trails_after,
        "connectors": len(connectors),
        "added_m": added,
        "left_odd": left,
        "method": "minimum-weight matching of odd outline nodes; Hierholzer order of the bed",
    }
    return graph


def _order_bed_trail(graph, starts, on_bed):
    """Draw the whole bed as one closed trail ending where the walls start."""
    bed = [e for e in graph["edges"] if on_bed(e["start"]) and on_bed(e["end"])]
    network = nx.MultiGraph()
    for edge in bed:
        network.add_edge(edge["start"], edge["end"], key=edge["id"])
    if (
        len(starts) != 1
        or not network.number_of_edges()
        or not nx.is_connected(network)
        or not nx.is_eulerian(network)
    ):
        return
    by_id = {edge["id"]: edge for edge in bed}
    hint = min(edge.get("order_hint", 0) for edge in bed)
    position = 0
    for a, _, key in nx.eulerian_circuit(network, source=next(iter(starts)), keys=True):
        edge = by_id[key]
        if edge["start"] != a:
            edge["start"], edge["end"] = edge["end"], edge["start"]
            edge["segments"] = [list(reversed(piece)) for piece in reversed(edge["segments"])]
        edge["parent_curve"] = "base-trail"
        edge["source_interval"] = [position, position + len(edge["segments"])]
        edge["order_hint"] = hint
        position += len(edge["segments"])
