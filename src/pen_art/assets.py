"""Prepare a physical build-frame mesh and a separate WIR3D unit-sphere asset."""

from pathlib import Path

import numpy as np
import trimesh
from trimesh.exchange.obj import export_obj

from .curves import transform_points
from .io import artifact, sha256, write_json


def normalization(vertices, source_up, max_extent_m):
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
    source_to_wir3d = to_unit @ source_to_y
    unit = transform_points(vertices, source_to_wir3d)
    scale = max_extent_m * radius / extent
    wir3d_to_build = np.eye(4)
    wir3d_to_build[:3, :3] = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]]) * scale
    wir3d_to_build[2, 3] = -unit[:, 1].min() * scale
    return source_to_wir3d, wir3d_to_build


def prepare_asset(source: Path, run: Path, config: dict) -> dict:
    scene = trimesh.load_scene(source, process=False)
    # Bake scene-node transforms before normalization; keep material boundaries.
    meshes = list(scene.dump())
    if not meshes or any(not isinstance(m, trimesh.Trimesh) for m in meshes):
        raise ValueError("Input must contain triangle meshes")
    # Keep reference materials separate. Concatenation is geometry-only: joining
    # TextureVisuals can silently discard UVs or choose just one material.
    vertices = np.concatenate([np.asarray(mesh.vertices) for mesh in meshes])
    source_to_wir3d, wir3d_to_build = normalization(
        vertices, config["source_up"], config["max_extent_m"]
    )
    source_to_build = wir3d_to_build @ source_to_wir3d
    source_bounds = np.array([vertices.min(axis=0), vertices.max(axis=0)])
    source_faces = sum(len(mesh.faces) for mesh in meshes)
    source_vertices = len(vertices)
    if not source_faces:
        raise ValueError("Input must contain triangle faces")
    materials = [getattr(mesh.visual, "material", None) for mesh in meshes]
    uv_parts = [getattr(mesh.visual, "uv", None) is not None for mesh in meshes]
    color_parts = [
        getattr(material, "baseColorTexture", None) is not None
        or getattr(material, "image", None) is not None
        for material in materials
    ]
    # WIR3D's current wrapper consumes a single OBJ texture/material. Supporting
    # scenes in geometric mode must not imply that a texture atlas was baked.
    wir3d_available = len(meshes) == 1 and not isinstance(
        materials[0], trimesh.visual.material.MultiMaterial
    )
    wir3d_reason = (
        None
        if wir3d_available
        else (
            "Multi-mesh or multi-material scenes require texture atlas baking before "
            "the current WIR3D adapter can consume them; geometric contours are available."
        )
    )
    bundle = run / "wir3d_bundle"
    bundle.mkdir(parents=True, exist_ok=True)
    if wir3d_available:
        mesh = meshes[0].copy()
        mesh.apply_transform(source_to_wir3d)
        material = materials[0]
        if material is not None and hasattr(material, "to_simple"):
            mesh.visual.material = material.to_simple()
        obj, resources = export_obj(
            mesh,
            include_normals=True,
            include_color=False,
            return_texture=True,
            mtl_name="model.mtl",
            digits=10,
        )
        (bundle / "model.obj").write_text(obj)
        for name, value in (resources or {}).items():
            destination = bundle / Path(name).name
            destination.write_bytes(value.encode() if isinstance(value, str) else value)
    else:
        # A previous preparation in this directory must not leave a usable stale
        # single-material entry point behind.
        (bundle / "model.obj").unlink(missing_ok=True)

    reference = scene.copy()
    reference.apply_transform(source_to_build)
    reference.export(run / "normalized.glb")
    physical = transform_points(vertices, source_to_build)
    faces = []
    offset = 0
    for mesh in meshes:
        faces.append(np.asarray(mesh.faces) + offset)
        offset += len(mesh.vertices)
    geometry = trimesh.Trimesh(vertices=physical, faces=np.concatenate(faces), process=True)
    geometry.update_faces(geometry.nondegenerate_faces())
    geometry.remove_unreferenced_vertices()
    diagnostics = {
        "watertight": bool(geometry.is_watertight),
        "winding_consistent": bool(geometry.is_winding_consistent),
        "component_count": int(geometry.body_count),
        "processed_vertices": len(geometry.vertices),
    }
    if len(geometry.faces) > config["geometry_faces"]:
        geometry = geometry.simplify_quadric_decimation(face_count=config["geometry_faces"])
    geometry.export(run / "geometry.glb")
    np.savez_compressed(run / "geometry.npz", vertices=geometry.vertices, faces=geometry.faces)
    manifest = artifact(
        "asset_manifest",
        source={"name": source.name, "sha256": sha256(source)},
        units="m",
        build_up="z",
        source_up=config["source_up"],
        source_vertices=source_vertices,
        source_triangles=source_faces,
        geometry_triangles=len(geometry.faces),
        source_bounds=source_bounds.tolist(),
        bounds_m=[physical.min(axis=0).tolist(), physical.max(axis=0).tolist()],
        source_mesh_instances=len(meshes),
        max_extent_m=config["max_extent_m"],
        texture={
            "has_uv": all(uv_parts),
            "has_base_color": all(color_parts),
            "parts_with_uv": sum(uv_parts),
            "parts_with_base_color": sum(color_parts),
            "wir3d_available": wir3d_available,
            "wir3d_unavailable_reason": wir3d_reason,
            "reference": "Full resolution scene with original materials and node transforms",
            "wir3d": (
                "UV and base-color texture preserved in OBJ/MTL"
                if wir3d_available and all(uv_parts) and all(color_parts)
                else "Geometry OBJ available; no complete UV/base-color texture"
                if wir3d_available
                else "Unavailable: texture atlas baking required"
            ),
        },
        transforms={
            "source_to_wir3d": source_to_wir3d.tolist(),
            "wir3d_to_build": wir3d_to_build.tolist(),
            "source_to_build": source_to_build.tolist(),
            "build_to_source": np.linalg.inv(source_to_build).tolist(),
        },
        diagnostics=diagnostics,
        artifacts={
            "reference": "normalized.glb",
            "geometry": "geometry.glb",
            "wir3d_input": "wir3d_bundle/model.obj" if wir3d_available else None,
        },
    )
    write_json(run / "asset_manifest.json", manifest)
    return manifest
