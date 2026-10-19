"""The frame step: a mesh scaled into the build frame, then its wire frame.

``pen-art frame`` writes ``runs/NAME/frame_source/``: the scaled mesh
(``normalized.glb``, ``geometry.glb``, ``asset_manifest.json``), the settings
(``fabrication_config.json``), the size settled on (``wire_budget.json``), the
frame (``framework_graph.json``, ``stroke_graph.json``), its strokes
(``construction_plan.json``) and whether it stands (``structural_screen.json``).
"""

import shutil

import numpy as np
import trimesh

from .curves import transform_points
from .io import artifact, read_json, sha256, write_json
from .planning.sections import section_actions
from .structure import analyze_structure

GEOMETRY_FACES = 80000


def normalization(vertices, source_up, max_extent_m):
    """(source to unit sphere, unit sphere to build frame): the longest side
    ``max_extent_m``, standing on the build surface with z up."""
    if source_up not in {"y", "z"}:
        raise ValueError("source_up must be explicitly y or z")
    if not np.isfinite(max_extent_m) or max_extent_m <= 0:
        raise ValueError("max_extent_m must be finite and positive")
    vertices = np.asarray(vertices, dtype=float)
    if not np.isfinite(vertices).all() or not len(vertices):
        raise ValueError("Mesh vertices must be finite and nonempty")
    source_to_y = np.eye(4)
    if source_up == "z":
        source_to_y[:3, :3] = [[1, 0, 0], [0, 0, 1], [0, -1, 0]]
    points = transform_points(vertices, source_to_y)
    center = (points.min(axis=0) + points.max(axis=0)) / 2
    radius = np.linalg.norm(points - center, axis=1).max()
    extent = np.ptp(points, axis=0).max()
    if radius < 1e-12 or extent < 1e-12:
        raise ValueError("Mesh has zero spatial extent")
    to_unit = np.eye(4)
    to_unit[:3, :3] /= radius
    to_unit[:3, 3] = -center / radius
    source_to_unit = to_unit @ source_to_y
    unit = transform_points(vertices, source_to_unit)
    scale = max_extent_m * radius / extent
    unit_to_build = np.eye(4)
    unit_to_build[:3, :3] = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]]) * scale
    unit_to_build[2, 3] = -unit[:, 1].min() * scale
    return source_to_unit, unit_to_build


def prepare_mesh(mesh, source, source_up, max_extent_m):
    """Scale ``mesh`` into the build frame in the folder ``source``."""
    source.mkdir(parents=True, exist_ok=True)
    scene = trimesh.load_scene(mesh, process=False)
    # Bake scene-node transforms before normalization; keep material boundaries.
    meshes = list(scene.dump())
    if not meshes or any(not isinstance(m, trimesh.Trimesh) for m in meshes):
        raise ValueError("Input must contain triangle meshes")
    vertices = np.concatenate([np.asarray(part.vertices) for part in meshes])
    source_to_unit, unit_to_build = normalization(vertices, source_up, max_extent_m)
    source_to_build = unit_to_build @ source_to_unit
    if not sum(len(part.faces) for part in meshes):
        raise ValueError("Input must contain triangle faces")
    reference = scene.copy()
    reference.apply_transform(source_to_build)
    reference.export(source / "normalized.glb")
    # Geometry only, for the viewer and the robot cell: concatenating textured
    # parts could silently drop their UVs.
    faces, offset = [], 0
    for part in meshes:
        faces.append(np.asarray(part.faces) + offset)
        offset += len(part.vertices)
    physical = transform_points(vertices, source_to_build)
    geometry = trimesh.Trimesh(vertices=physical, faces=np.concatenate(faces), process=True)
    geometry.update_faces(geometry.nondegenerate_faces())
    geometry.remove_unreferenced_vertices()
    if len(geometry.faces) > GEOMETRY_FACES:
        geometry = geometry.simplify_quadric_decimation(face_count=GEOMETRY_FACES)
    geometry.export(source / "geometry.glb")
    write_json(
        source / "asset_manifest.json",
        artifact(
            "asset_manifest",
            source={"name": mesh.name, "sha256": sha256(mesh)},
            units="m",
            source_up=source_up,
            max_extent_m=max_extent_m,
            source_to_build=source_to_build.tolist(),
            artifacts={"reference": "normalized.glb", "geometry": "geometry.glb"},
        ),
    )
    return source


def prepare_mesh_within_wire(mesh, source, source_up, max_extent_m, fabrication):
    """Prepare a mesh, shrunk until its wireframe fits ``wireprint.maximum_wire_m``.

    Wire grows with about the square of the size, so each try scales the
    longest side by the square root of the overshoot. The size settled on and
    the wire at each try are written to ``wire_budget.json``.
    """
    from .abstraction.connectors import eulerize_frame
    from .abstraction.wireprint import connect_framework_islands, generate

    budget = fabrication.get("wireprint", {}).get("maximum_wire_m")
    extent, tries = max_extent_m, []
    while True:
        prepare_mesh(mesh, source, source_up, extent)
        write_json(source / "fabrication_config.json", fabrication)
        if budget is None:
            break
        # Measured with its connector wire, so the frame that fits keeps them.
        graph = eulerize_frame(connect_framework_islands(generate(source)), float("inf"))
        wire = sum(edge["length_m"] for edge in graph["edges"])
        tries.append({"max_extent_m": extent, "wire_m": wire})
        if wire <= budget or len(tries) == 6:
            break
        shutil.rmtree(source)
        extent *= 0.99 * (budget / wire) ** 0.5
    write_json(
        source / "wire_budget.json",
        {
            "requested_max_extent_m": max_extent_m,
            "max_extent_m": extent,
            "maximum_wire_m": budget,
            "within_budget": budget is None or tries[-1]["wire_m"] <= budget,
            "tries": tries,
        },
    )
    return source


def _sag(screen):
    return {
        "modeled_pass": screen["modeled_pass"],
        "maximum_displacement_m": screen.get("maximum_displacement_m"),
    }


def prepare_framework(source):
    """Generate the frame in ``source``, with a heavier strand if it would not stand."""
    from .abstraction.connectors import eulerize_frame
    from .abstraction.wireprint import connect_framework_islands, generate, reinforce_base

    fabrication = read_json(source / "fabrication_config.json")
    material = fabrication["structure"]

    def framework(**settings):
        graph = eulerize_frame(
            connect_framework_islands(generate(source, **settings)),
            fabrication.get("wireprint", {}).get("maximum_wire_m", float("inf")),
        )
        screen = analyze_structure(graph, material)
        if not screen["modeled_pass"] and "maximum_displacement_m" in screen:
            graph = reinforce_base(graph)
            screen = analyze_structure(graph, material)
        return graph, screen

    graph, screen = framework()
    # A frame that still fails is regenerated with each heavier strand the pen
    # can lay, lightest first. The strand diameter is also the wire's collision
    # size. Nothing passing leaves the original for the planner to reject.
    tried = [{"strand_diameter_m": graph["generation"]["strand_diameter_m"], **_sag(screen)}]
    for diameter in fabrication.get("wireprint", {}).get("heavier_strands_m", []):
        if screen["modeled_pass"]:
            break
        heavier, screen = framework(diameter=diameter)
        tried.append({"strand_diameter_m": diameter, **_sag(screen)})
        if screen["modeled_pass"]:
            graph = heavier
    if len(tried) > 1:
        if not screen["modeled_pass"]:
            graph, _ = framework()
        graph["strand_escalation"] = tried
    write_json(source / "framework_graph.json", graph)
    write_strokes(source, graph, section_actions(graph), material)


def write_strokes(folder, graph, construction, material):
    """Write the stroke graph, its strokes and the structural screen; return the screen."""
    write_json(folder / "stroke_graph.json", graph)
    write_json(folder / "construction_plan.json", construction)
    screen = analyze_structure(graph, material)
    write_json(folder / "structural_screen.json", screen)
    return screen
