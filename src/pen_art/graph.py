"""The wire graph: its topology, straight members, connectors and bed anchors."""

from itertools import combinations

import networkx as nx
import numpy as np
from scipy.spatial import ConvexHull, QhullError, cKDTree


def topology(graph, edge_ids=None):
    result = nx.MultiGraph()
    for edge in graph["edges"]:
        if edge_ids is None or edge["id"] in edge_ids:
            result.add_edge(edge["start"], edge["end"], key=edge["id"])
    return result


def straight_edge(start, end, nodes, identity, role, diameter):
    a, b = np.array(nodes[start]["position"]), np.array(nodes[end]["position"])
    return {
        "id": identity,
        "start": start,
        "end": end,
        "role": role,
        "origin": "automatic_connection",
        "required": True,
        "diameter_m": diameter,
        "segments": [
            [a.tolist(), (a + (b - a) / 3).tolist(), (a + 2 * (b - a) / 3).tolist(), b.tolist()]
        ],
        "length_m": float(np.linalg.norm(b - a)),
        "process_status": "not_checked",
    }


def connect_components(nodes, edges, settings):
    graph = {"nodes": list(nodes.values()), "edges": edges}
    components = sorted(nx.connected_components(topology(graph)), key=lambda c: min(c))
    candidates = nx.Graph()
    candidates.add_nodes_from(range(len(components)))
    for i, component in enumerate(components):
        ids_a = sorted(component)
        points_a = np.array([nodes[node]["position"] for node in ids_a])
        tree = cKDTree(points_a)
        for j in range(i + 1, len(components)):
            ids_b = sorted(components[j])
            points_b = np.array([nodes[node]["position"] for node in ids_b])
            distances, indices = tree.query(points_b)
            best = int(distances.argmin())
            length = float(distances[best])
            if length <= settings["maximum_connector_m"]:
                candidates.add_edge(
                    i, j, weight=length, start=ids_a[int(indices[best])], end=ids_b[best]
                )
    for index, (_, _, data) in enumerate(nx.minimum_spanning_edges(candidates, data=True)):
        edges.append(
            straight_edge(
                data["start"],
                data["end"],
                nodes,
                f"connector-{index:04}",
                "connector",
                settings["strand_diameter_m"],
            )
        )
    return len(components)


def base_anchors(nodes, ordered, ground_tolerance):
    """Choose three spread-out low contacts, rather than covering the whole bed."""
    low = [key for key in ordered if abs(nodes[key]["position"][2]) <= ground_tolerance]
    if len(low) < 3:
        low = ordered[: max(3, len(ordered) // 5)]
    if len(low) <= 3:
        return low
    points = np.array([nodes[key]["position"][:2] for key in low])
    try:
        hull = ConvexHull(points).vertices
        if len(hull) > 24:
            hull = hull[np.linspace(0, len(hull) - 1, 24).astype(int)]
        center = points[hull].mean(0)
        choices = []
        for triangle in combinations(hull, 3):
            a, b, c = points[list(triangle)]
            matrix = np.column_stack([b - a, c - a])
            area = abs(np.linalg.det(matrix))
            if area < 1e-12:
                continue
            weights = np.linalg.solve(matrix, center - a)
            inside = min(*weights, 1 - weights.sum()) >= -1e-10
            choices.append((inside, area, triangle))
        if choices:
            return [low[i] for i in max(choices, key=lambda item: item[:2])[2]]
    except QhullError:
        pass
    distance = np.linalg.norm(points - points[0], axis=1)
    return [low[0], low[int(distance.argmax())]]
