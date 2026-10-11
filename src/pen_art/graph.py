"""Physical 3D junctions and automatic material connections for cubic curves.

This stage proposes geometry. Connectivity is verified independently of support,
pen process feasibility, and robot accessibility.
"""

from copy import deepcopy
from itertools import combinations

import networkx as nx
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import ConvexHull, QhullError, cKDTree

from .curves import bezier_at, curve_coordinate_bounds, curve_interval, curve_length, flatten_curve
from .io import artifact


def topology(graph, edge_ids=None):
    result = nx.MultiGraph()
    for edge in graph["edges"]:
        if edge_ids is None or edge["id"] in edge_ids:
            result.add_edge(edge["start"], edge["end"], key=edge["id"])
    return result


def point_at(curve, parameter):
    index = min(int(parameter), len(curve["segments"]) - 1)
    return bezier_at(curve["segments"][index], parameter - index)


def closest_segment_pairs(a, b, c, d):
    """Exact closest points on pairs of finite straight segments, vectorized."""
    u, v, w = b - a, d - c, a - c
    uu, vv, uv = (u * u).sum(1), (v * v).sum(1), (u * v).sum(1)
    uw, vw = (u * w).sum(1), (v * w).sum(1)
    uu, vv = np.maximum(uu, 1e-24), np.maximum(vv, 1e-24)
    determinant = uu * vv - uv * uv
    s = (uv * vw - vv * uw) / np.maximum(determinant, 1e-30)
    t = (uu * vw - uv * uw) / np.maximum(determinant, 1e-30)
    valid = (determinant > 1e-20) & (s >= 0) & (s <= 1) & (t >= 0) & (t <= 1)
    candidates = [
        (np.zeros_like(s), np.clip(vw / vv, 0, 1)),
        (np.ones_like(s), np.clip((vw + uv) / vv, 0, 1)),
        (np.clip(-uw / uu, 0, 1), np.zeros_like(s)),
        (np.clip((uv - uw) / uu, 0, 1), np.ones_like(s)),
        (s, t),
    ]
    distances = np.stack(
        [np.linalg.norm(w + si[:, None] * u - ti[:, None] * v, axis=1) for si, ti in candidates]
    )
    distances[-1, ~valid] = np.inf
    best = distances.argmin(0)
    columns = np.arange(len(a))
    return (
        distances[best, columns],
        np.stack([p[0] for p in candidates])[best, columns],
        np.stack([p[1] for p in candidates])[best, columns],
    )


def detect_contacts(curves, settings):
    tolerance = settings["junction_tolerance_m"]
    points_a, points_b, owners, params_a, params_b, arc_a, arc_b = [], [], [], [], [], [], []
    lengths = []
    for index, curve in enumerate(curves):
        points, parameters = flatten_curve(
            curve, settings["flatten_tolerance_m"], settings["sampling_chord_m"]
        )
        arc = np.r_[0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
        lengths.append(arc[-1])
        points_a.extend(points[:-1])
        points_b.extend(points[1:])
        owners.extend([index] * (len(points) - 1))
        params_a.extend(parameters[:-1])
        params_b.extend(parameters[1:])
        arc_a.extend(arc[:-1])
        arc_b.extend(arc[1:])
    a, b = np.array(points_a), np.array(points_b)
    owner, low, high = np.array(owners), np.array(params_a), np.array(params_b)
    arc_a, arc_b = np.array(arc_a), np.array(arc_b)
    radius = np.linalg.norm(b - a, axis=1).max() + tolerance + 2 * settings["flatten_tolerance_m"]
    pairs = cKDTree((a + b) / 2).query_pairs(radius, output_type="ndarray")
    if not len(pairs):
        return []
    i, j = pairs.T
    # Exclude neighboring portions of the same stroke, including a loop's seam.
    same = owner[i] == owner[j]
    gap = np.maximum(arc_a[j] - arc_b[i], arc_a[i] - arc_b[j])
    seam_gap = (
        np.array(lengths)[owner[i]]
        - np.maximum(arc_b[j], arc_b[i])
        + np.minimum(arc_a[i], arc_a[j])
    )
    keep = ~same | ((gap > 4 * tolerance) & (seam_gap > 4 * tolerance))
    i, j = i[keep], j[keep]
    distance, s, t = closest_segment_pairs(a[i], b[i], a[j], b[j])
    direction_a, direction_b = b[i] - a[i], b[j] - a[j]
    cosine = np.abs((direction_a * direction_b).sum(1)) / np.maximum(
        np.linalg.norm(direction_a, axis=1) * np.linalg.norm(direction_b, axis=1), 1e-24
    )
    parameter_a, parameter_b = low[i] + s * (high[i] - low[i]), low[j] + t * (high[j] - low[j])
    endpoints = np.array([len(c["segments"]) for c in curves])
    endpoint = (
        (parameter_a < 1e-8)
        | (parameter_b < 1e-8)
        | (parameter_a > endpoints[owner[i]] - 1e-8)
        | (parameter_b > endpoints[owner[j]] - 1e-8)
    )
    keep = (distance <= tolerance + 2 * settings["flatten_tolerance_m"]) & (
        endpoint | (cosine < np.cos(np.deg2rad(settings["minimum_crossing_angle_deg"])))
    )
    contacts = []
    for k in np.flatnonzero(keep)[np.argsort(distance[keep])]:
        ci, cj = int(owner[i[k]]), int(owner[j[k]])
        bounds = ([low[i[k]], low[j[k]]], [high[i[k]], high[j[k]]])
        initial = np.clip([parameter_a[k], parameter_b[k]], bounds[0], bounds[1])
        solved = least_squares(
            lambda p, ci=ci, cj=cj: point_at(curves[ci], p[0]) - point_at(curves[cj], p[1]),
            initial,
            bounds=bounds,
            gtol=1e-12,
            xtol=1e-10,
            ftol=1e-10,
            max_nfev=25,
        )
        p, q = point_at(curves[ci], solved.x[0]), point_at(curves[cj], solved.x[1])
        if np.linalg.norm(p - q) <= tolerance:
            contacts.append((ci, float(solved.x[0]), cj, float(solved.x[1]), p, q))
    return contacts


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


def ground_disconnected_components(nodes, edges, diameter):
    """Join remaining islands through printed feet and on-bed base lines.

    The short-connector limit still applies to free-space connections. A more
    distant island receives an upward support and a supported line on the bed,
    rather than silently remaining a separately assembled object.
    """
    components = list(nx.connected_components(topology({"edges": edges})))
    if len(components) < 2:
        return []
    grounded = [c for c in components if any(nodes[n]["ground"] for n in c)]
    if not grounded:
        raise ValueError("A base contact is required before connecting islands")
    root = max(grounded, key=len)
    feet = [n for n in root if nodes[n]["ground"]]
    added = []
    for component in sorted(components, key=lambda c: min(c)):
        if component is root:
            continue
        contacts = [n for n in component if nodes[n]["ground"]]
        if contacts:
            foot, target = min(
                ((a, b) for a in contacts for b in feet),
                key=lambda pair: np.linalg.norm(
                    np.array(nodes[pair[0]]["position"]) - nodes[pair[1]]["position"]
                ),
            )
        else:
            anchor = min(component, key=lambda n: (nodes[n]["position"][2], n))
            position = nodes[anchor]["position"]
            foot = f"island-foot-{len(nodes):05}"
            nodes[foot] = {
                "id": foot,
                "position": [*position[:2], 0.0],
                "ground": True,
                "join_status": "proposed",
            }
            support = straight_edge(
                foot, anchor, nodes, f"island-support-{len(edges):04}", "support", diameter
            )
            if support["length_m"] > 1e-8:
                edges.append(support)
                added.append(support["id"])
            target = min(
                feet,
                key=lambda n: np.linalg.norm(
                    np.array(nodes[n]["position"]) - nodes[foot]["position"]
                ),
            )
        base = straight_edge(target, foot, nodes, f"island-base-{len(edges):04}", "base", diameter)
        if base["length_m"] <= 1e-8:
            # Coincident feet are one physical junction, not a zero-length edge.
            for edge in edges:
                for end in ("start", "end"):
                    if edge[end] == foot:
                        edge[end] = target
            nodes.pop(foot, None)
            feet.extend(n for n in contacts if n != foot)
        else:
            edges.append(base)
            added.append(base["id"])
            feet.extend(contacts or [foot])
    return added


def build_graph(candidate, settings):
    curves = deepcopy(candidate["curves"])
    if not curves:
        raise ValueError("Cannot construct a graph from an empty curve set")
    for curve in curves:
        if not curve["segments"] or not np.isfinite(np.asarray(curve["segments"])).all():
            raise ValueError("All input curves must contain finite cubic pieces")
    tolerance = settings["junction_tolerance_m"]
    nodes, references, cuts = {}, {}, [[] for _ in curves]

    def node_for(position, points):
        # Representatives stay fixed, preventing transitive snapping from drifting.
        for identity, node in nodes.items():
            if max(np.linalg.norm(np.array(node["position"]) - p) for p in points) <= tolerance:
                references[identity].extend(points)
                return identity
        identity = f"junction-{len(nodes):05}"
        nodes[identity] = {
            "id": identity,
            "position": np.asarray(position).tolist(),
            "ground": False,
            "join_status": "proposed",
        }
        references[identity] = list(points)
        return identity

    for index, curve in enumerate(curves):
        for parameter in (0.0, float(len(curve["segments"]))):
            point = point_at(curve, parameter)
            cuts[index].append((parameter, node_for(point, [point])))
    contacts = detect_contacts(curves, settings)
    for ci, ti, cj, tj, p, q in contacts:
        node = node_for((p + q) / 2, [p, q])
        cuts[ci].append((ti, node))
        cuts[cj].append((tj, node))
    edges = []
    max_snap = 0.0
    for index, curve in enumerate(curves):
        ordered = sorted(cuts[index])
        polyline, parameters = flatten_curve(curve)
        arc = np.r_[0, np.cumsum(np.linalg.norm(np.diff(polyline, axis=0), axis=1))]
        cleaned = []
        for parameter, node in ordered:
            close_repeat = (
                cleaned
                and node == cleaned[-1][1]
                and (
                    np.interp(parameter, parameters, arc)
                    - np.interp(cleaned[-1][0], parameters, arc)
                    <= 2 * tolerance
                )
            )
            if cleaned and (abs(parameter - cleaned[-1][0]) < 1e-7 or close_repeat):
                # Keep a closed curve's final endpoint, even though its node repeats.
                if parameter == len(curve["segments"]) and cleaned[-1][0] < parameter - 1e-7:
                    cleaned.append((parameter, node))
                continue
            cleaned.append((parameter, node))
        for part, ((low, start), (high, end)) in enumerate(zip(cleaned, cleaned[1:], strict=False)):
            segments = np.asarray(curve_interval(curve, low, high))
            delta_start = np.array(nodes[start]["position"]) - segments[0, 0]
            delta_end = np.array(nodes[end]["position"]) - segments[-1, -1]
            max_snap = max(max_snap, np.linalg.norm(delta_start), np.linalg.norm(delta_end))
            segments[0, :2] += delta_start
            segments[-1, 2:] += delta_end
            edge = {
                "id": f"{curve['id']}:part-{part:04}",
                "parent_curve": curve["id"],
                "source_interval": [low, high],
                "start": start,
                "end": end,
                "role": "artistic",
                "origin": curve["origin"],
                "required": False,
                "diameter_m": settings["strand_diameter_m"],
                "segments": segments.tolist(),
                "closed": start == end,
                "process_status": "not_checked",
            }
            edge["length_m"] = curve_length(edge)
            if edge["length_m"] > 1e-8:
                edges.append(edge)
    used = {node for edge in edges for node in (edge["start"], edge["end"])}
    nodes = {key: value for key, value in nodes.items() if key in used}
    original_components = nx.number_connected_components(topology({"edges": edges}))
    # Fit/snap operations can put an interior extremum below the original bed.
    # Place the whole drawing above the bed, preserving its shape, before adding
    # physical feet. Record this rigid placement rather than trimming a contour.
    minimum_z = min(curve_coordinate_bounds(edge)[0, 2] for edge in edges)
    lift = max(0.0, -float(minimum_z))
    placement = np.eye(4)
    placement[2, 3] = lift
    if lift:
        for node in nodes.values():
            node["position"][2] += lift
        for edge in edges:
            segments = np.asarray(edge["segments"])
            segments[:, :, 2] += lift
            edge["segments"] = segments.tolist()
    # Lowest existing points offer generic base candidates; no model category labels.
    sorted_nodes = sorted(nodes, key=lambda key: (nodes[key]["position"][2], key))
    ground = [node for node in sorted_nodes if abs(nodes[node]["position"][2]) <= 1e-10]
    for key in ground:
        nodes[key]["ground"] = True
    selected = base_anchors(nodes, sorted_nodes, settings["ground_tolerance_m"])
    base = []
    for identity in selected:
        original = nodes[identity]["position"]
        foot_position = [original[0], original[1], 0.0]
        if np.linalg.norm(np.array(original) - foot_position) <= 1e-10:
            foot = identity
        else:
            foot = f"base-junction-{len(nodes):05}"
            nodes[foot] = {
                "id": foot,
                "position": foot_position,
                "ground": True,
                "join_status": "proposed",
            }
            edges.append(
                straight_edge(
                    foot,
                    identity,
                    nodes,
                    f"support-{len(edges):04}",
                    "support",
                    settings["strand_diameter_m"],
                )
            )
        nodes[foot]["ground"] = True
        base.append(foot)
        if foot not in ground:
            ground.append(foot)
    # A real base network joins contacts; the build surface is never a graph edge.
    if len(base) >= 2:
        points = np.array([nodes[node]["position"][:2] for node in base])
        try:
            hull = ConvexHull(points).vertices.tolist()
            pairs = list(zip(hull, hull[1:] + hull[:1], strict=True))
            inside = sorted(set(range(len(base))) - set(hull))
            for i in inside:
                j = min(hull, key=lambda j: np.linalg.norm(points[i] - points[j]))
                pairs.append((i, j))
        except QhullError:
            pairs = list(zip(range(len(base) - 1), range(1, len(base)), strict=True))
        for index, (i, j) in enumerate(pairs):
            if np.linalg.norm(points[i] - points[j]) > 1e-8:
                edges.append(
                    straight_edge(
                        base[i],
                        base[j],
                        nodes,
                        f"base-{index:04}",
                        "base",
                        settings["strand_diameter_m"],
                    )
                )
    connect_components(nodes, edges, settings)
    island_supports = ground_disconnected_components(nodes, edges, settings["strand_diameter_m"])
    graph = artifact(
        "stroke_graph",
        frame="build",
        units="m",
        source_method=candidate["method"],
        simulation_only=True,
        nodes=list(nodes.values()),
        edges=edges,
        physical_connections="proposed_welds",
        candidate_to_build=placement.tolist(),
        settings=settings,
    )
    components = list(nx.connected_components(topology(graph)))
    graph["metrics"] = {
        "input_curves": len(curves),
        "material_members": len(edges),
        "junctions": len(nodes),
        "input_components": original_components,
        "material_components": len(components),
        "ground_contacts": sum(node["ground"] for node in nodes.values()),
        "detected_contacts": len(contacts),
        "maximum_snap_m": float(max_snap),
        "total_length_m": sum(edge["length_m"] for edge in edges),
        "added_length_m": sum(e["length_m"] for e in edges if e["role"] != "artistic"),
    }
    graph["diagnostics"] = []
    if island_supports:
        graph["diagnostics"].append(
            {
                "code": "ISLANDS_JOINED_THROUGH_BASE",
                "message": "Distant components joined automatically with vertical supports and "
                "on-bed base lines; deposition feasibility still needs validation",
                "added_edges": island_supports,
            }
        )
    if len(components) != 1:
        graph["diagnostics"].append(
            {
                "code": "DISCONNECTED_MATERIAL",
                "message": "Increase connector search range or revise the abstraction",
                "components": [sorted(component) for component in components],
            }
        )
    if max_snap > tolerance + 1e-9:
        raise ValueError("Junction snapping exceeded the configured geometric tolerance")
    graph["checks"] = {
        "material_connected": len(components) == 1,
        "has_ground_contacts": bool(ground),
        "process_feasible": None,
        "structure_feasible": None,
        "robot_feasible": None,
    }
    return graph
