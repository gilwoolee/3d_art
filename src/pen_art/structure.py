"""Linear 3D Euler–Bernoulli frame screening for cooled, welded strands.

Element equations follow the standard beam stiffness/equivalent-load formulation:
https://interactivetextbooks.citg.tudelft.nl/mechanics/slender_structures/EB_K_Feq.html
Rigid welds and small deflections are explicit modeling assumptions, not calibration.
"""

import warnings
from copy import deepcopy

import networkx as nx
import numpy as np
from scipy.linalg import block_diag
from scipy.sparse import coo_matrix, diags
from scipy.sparse.linalg import MatrixRankWarning, splu
from scipy.spatial import ConvexHull, QhullError

from .curves import flatten_curve
from .graph import straight_edge, topology
from .io import artifact


def beam_element(a, b, diameter, material):
    delta = np.asarray(b) - a
    length = np.linalg.norm(delta)
    if length < 1e-9:
        raise ValueError("A beam element must have nonzero length")
    x = delta / length
    reference = np.eye(3)[np.argmin(np.abs(x))]
    y = np.cross(reference, x)
    y /= np.linalg.norm(y)
    rotation = np.array([x, y, np.cross(x, y)])
    transform = block_diag(rotation, rotation, rotation, rotation)
    area, inertia = np.pi * diameter**2 / 4, np.pi * diameter**4 / 64
    elastic = material["young_modulus_pa"]
    shear = elastic / (2 * (1 + material["poisson_ratio"]))
    k = np.zeros((12, 12))
    k[np.ix_([0, 6], [0, 6])] = elastic * area / length * np.array([[1, -1], [-1, 1]])
    k[np.ix_([3, 9], [3, 9])] = shear * 2 * inertia / length * np.array([[1, -1], [-1, 1]])
    bending = (
        elastic
        * inertia
        / length**3
        * np.array(
            [
                [12, 6 * length, -12, 6 * length],
                [6 * length, 4 * length**2, -6 * length, 2 * length**2],
                [-12, -6 * length, 12, -6 * length],
                [6 * length, 2 * length**2, -6 * length, 4 * length**2],
            ]
        )
    )
    k[np.ix_([1, 5, 7, 11], [1, 5, 7, 11])] = bending
    signs = np.diag([1, -1, 1, -1])
    k[np.ix_([2, 4, 8, 10], [2, 4, 8, 10])] = signs @ bending @ signs
    weight = rotation @ np.array([0.0, 0.0, -9.81 * material["density_kg_m3"] * area])
    load = np.zeros(12)
    load[:3] = load[6:9] = weight * length / 2
    load[4], load[5] = -weight[2] * length**2 / 12, weight[1] * length**2 / 12
    load[10:12] = -load[4:6]
    return k, transform, load, area, inertia, length


def analyze_structure(graph, material, edge_ids=None, nodal_loads=None, fixed_translations=None):
    if material["young_modulus_pa"] <= 0 or not -1 < material["poisson_ratio"] < 0.5:
        raise ValueError("Invalid isotropic elastic material parameters")
    if material["density_kg_m3"] < 0 or material["element_length_m"] <= 0:
        raise ValueError("Density must be nonnegative and element length positive")
    edges = [e for e in graph["edges"] if edge_ids is None or e["id"] in edge_ids]
    support_graph = topology(graph, edge_ids)
    nodes = {node["id"]: node for node in graph["nodes"]}
    unsupported = [
        sorted(c)
        for c in nx.connected_components(support_graph)
        if not any(nodes[node]["ground"] for node in c)
    ]
    base = artifact(
        "structural_screen",
        model="linear_3d_frame",
        calibrated=material["calibrated"],
        physical_status="unverified",
        boundary_condition="clamped_print_bed",
        limitations=[
            "Cooled material only",
            "Rigid welds assumed",
            "No nozzle forces or thermal distortion",
            "Linear small deflections",
        ],
        settings=material,
        point_translation_constraints=sorted(fixed_translations or []),
    )
    if not edges:
        return {
            **base,
            "modeled_pass": True,
            "maximum_displacement_m": 0.0,
            "edges": [],
            "nodes": [],
        }
    if unsupported:
        return {
            **base,
            "modeled_pass": False,
            "failure": "UNSUPPORTED_COMPONENT",
            "unsupported_components": unsupported,
            "edges": [],
            "nodes": [],
        }
    used = sorted(support_graph.nodes)
    indices = {key: index for index, key in enumerate(used)}
    positions = [nodes[key]["position"] for key in used]
    fixed = [indices[key] for key in used if nodes[key]["ground"]]
    elements, chains = [], {}
    for edge in edges:
        points, _ = flatten_curve(edge, tolerance=0.000025, max_chord=material["element_length_m"])
        chain = [indices[edge["start"]]]
        for point in points[1:-1]:
            chain.append(len(positions))
            positions.append(point)
        chain.append(indices[edge["end"]])
        chains[edge["id"]] = chain
        for a, b in zip(chain, chain[1:], strict=False):
            if np.linalg.norm(np.asarray(positions[a]) - positions[b]) > 1e-9:
                elements.append((a, b, edge))
    positions = np.asarray(positions)
    count = 6 * len(positions)
    rows, cols, data = [], [], []
    force = np.zeros(count)
    records = []
    total_mass = 0.0
    mass_center = np.zeros(3)
    for a, b, edge in elements:
        k, transform, local_load, area, inertia, length = beam_element(
            positions[a], positions[b], edge["diameter_m"], material
        )
        dofs = np.r_[np.arange(6 * a, 6 * a + 6), np.arange(6 * b, 6 * b + 6)]
        global_k = transform.T @ k @ transform
        rows.extend(np.repeat(dofs, 12))
        cols.extend(np.tile(dofs, 12))
        data.extend(global_k.ravel())
        force[dofs] += transform.T @ local_load
        records.append((edge, dofs, k, transform, local_load, area, inertia, length))
        mass = material["density_kg_m3"] * area * length
        total_mass += mass
        mass_center += mass * (positions[a] + positions[b]) / 2
    if nodal_loads:
        for key, load in nodal_loads.items():
            force[6 * indices[key] : 6 * indices[key] + 6] += np.asarray(load)
    stiffness = coo_matrix((data, (rows, cols)), shape=(count, count)).tocsc()
    free = np.ones(count, dtype=bool)
    for index in fixed:
        free[6 * index : 6 * index + 6] = False
    for key in fixed_translations or []:
        if key not in indices:
            raise ValueError(f"Translation constraint refers to absent material node: {key}")
        # Ideal point contact holds position; it does not clamp rotation or create a weld.
        index = indices[key]
        free[6 * index : 6 * index + 3] = False
    displacement = np.zeros(count)
    try:
        with warnings.catch_warnings():
            warnings.simplefilter("error", MatrixRankWarning)
            reduced = stiffness[free][:, free]
            if free.any():
                # Equilibrate mixed translation/rotation units and disparate element sizes.
                diagonal = reduced.diagonal()
                if np.any(diagonal <= 0):
                    raise ValueError("Unconstrained structural degree of freedom")
                scaling = 1 / np.sqrt(diagonal)
                balanced = (diags(scaling) @ reduced @ diags(scaling)).tocsc()
                factor = splu(balanced)
                solution = scaling * factor.solve(scaling * force[free])
                for _ in range(3):
                    error = force[free] - reduced @ solution
                    if np.linalg.norm(error) <= 1e-8 * max(np.linalg.norm(force[free]), 1e-12):
                        break
                    candidate = solution + scaling * factor.solve(scaling * error)
                    if np.linalg.norm(force[free] - reduced @ candidate) >= np.linalg.norm(error):
                        break
                    solution = candidate
                displacement[free] = solution
        if not np.isfinite(displacement).all():
            raise ValueError("Nonfinite structural solution")
    except (MatrixRankWarning, ValueError, RuntimeError):
        return {
            **base,
            "modeled_pass": False,
            "failure": "SINGULAR_STRUCTURE",
            "edges": [],
            "nodes": [],
        }
    residual = stiffness @ displacement - force
    relative_residual = np.linalg.norm(residual[free]) / max(np.linalg.norm(force[free]), 1e-12)
    if relative_residual >= 1e-5:
        # Never rank braces or draw a deformation field from an unreliable solve.
        return {
            **base,
            "modeled_pass": False,
            "failure": "ILL_CONDITIONED_STRUCTURE",
            "relative_residual": float(relative_residual),
            "minimum_element_length_m": min(record[-1] for record in records),
            "message": "Resolve tiny or degenerate junction spans before structural analysis",
            "edges": [],
            "nodes": [],
        }
    translational = displacement.reshape(-1, 6)[:, :3]
    maximum = float(np.linalg.norm(translational, axis=1).max())
    stresses = {edge["id"]: 0.0 for edge in edges}
    for edge, dofs, k, transform, load, area, inertia, _ in records:
        forces = (k @ transform @ displacement[dofs] - load).reshape(2, 6)
        normal_stress = np.abs(forces[:, 0]) / area + np.linalg.norm(forces[:, 4:6], axis=1) * edge[
            "diameter_m"
        ] / (2 * inertia)
        torsion_stress = np.abs(forces[:, 3]) * edge["diameter_m"] / (4 * inertia)
        equivalent = np.sqrt(normal_stress**2 + 3 * torsion_stress**2)
        stresses[edge["id"]] = max(stresses[edge["id"]], float(equivalent.max()))
    maximum_stress = max(stresses.values())
    scale = max(np.ptp(positions, axis=0).max(), 1e-9)
    linear_valid = maximum / scale <= material["maximum_linear_deflection_ratio"]
    center = mass_center / max(total_mass, 1e-24)
    rest_stable = False
    try:
        hull = ConvexHull(positions[fixed, :2])
        rest_stable = bool(
            np.all(hull.equations[:, :2] @ center[:2] + hull.equations[:, 2] <= 1e-9)
        )
    except QhullError:
        pass
    # Resting stability is a separate necessary tipping screen, not an adhesive-bed check.
    return {
        **base,
        "modeled_pass": bool(
            linear_valid
            and relative_residual < 1e-5
            and maximum <= material["maximum_displacement_m"]
            and maximum_stress <= material["yield_stress_pa"]
        ),
        "maximum_displacement_m": maximum,
        "maximum_stress_pa": maximum_stress,
        "linear_model_valid": bool(linear_valid),
        "relative_residual": float(relative_residual),
        "resting_com_inside_contacts": rest_stable,
        "mass_kg": float(total_mass),
        "center_of_mass_m": center.tolist(),
        "nodes": [
            {"id": key, "displacement_m": translational[index].tolist()}
            for key, index in indices.items()
        ],
        "edges": [
            {
                "id": edge["id"],
                "maximum_stress_pa": stresses[edge["id"]],
                "maximum_displacement_m": float(
                    np.linalg.norm(translational[chains[edge["id"]]], axis=1).max()
                ),
                "positions_m": positions[chains[edge["id"]]].tolist(),
                "displacements_m": translational[chains[edge["id"]]].tolist(),
            }
            for edge in edges
        ],
    }


def repair_structure(graph, settings, material):
    """Try real diagonal members and retain only improvements in modeled stiffness."""
    graph = deepcopy(graph)
    report = analyze_structure(graph, material)
    history = []
    nodes = {node["id"]: node for node in graph["nodes"]}
    for _ in range(settings["maximum_braces"]):
        if report["modeled_pass"] or not report.get("nodes"):
            break
        deformation = {n["id"]: np.linalg.norm(n["displacement_m"]) for n in report["nodes"]}
        high = sorted(deformation, key=deformation.get, reverse=True)[:8]
        connected = topology(graph)
        options = []
        for a in high:
            for b in nodes:
                if a == b or connected.has_edge(a, b):
                    continue
                length = np.linalg.norm(np.array(nodes[a]["position"]) - nodes[b]["position"])
                if 3 * settings["strand_diameter_m"] < length <= settings["maximum_brace_m"]:
                    score = (deformation[a] - deformation.get(b, 0)) / length
                    if score > 0:
                        options.append((float(score), a, b))
        best = None
        for _, a, b in sorted(options, reverse=True)[:8]:
            next_index = sum(e["role"] == "brace" for e in graph["edges"])
            brace = straight_edge(
                a, b, nodes, f"brace-{next_index:04}", "brace", settings["strand_diameter_m"]
            )
            trial = {**graph, "edges": [*graph["edges"], brace]}
            result = analyze_structure(trial, material)
            if (
                result.get("maximum_displacement_m", np.inf)
                < report["maximum_displacement_m"] * 0.98
            ):
                if (
                    best is None
                    or result["maximum_displacement_m"] < best[1]["maximum_displacement_m"]
                ):
                    best = trial, result, brace
        if best is None:
            break
        graph, report, brace = best
        history.append(
            {"added": brace["id"], "maximum_displacement_m": report["maximum_displacement_m"]}
        )
    graph["checks"]["structure_feasible"] = (
        report["modeled_pass"] if material["calibrated"] else None
    )
    graph["metrics"]["material_members"] = len(graph["edges"])
    graph["metrics"]["total_length_m"] = sum(e["length_m"] for e in graph["edges"])
    graph["metrics"]["added_length_m"] = sum(
        e["length_m"] for e in graph["edges"] if e["role"] != "artistic"
    )
    report["repair_history"] = history
    return graph, report
