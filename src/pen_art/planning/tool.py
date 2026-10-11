"""Load explicit flange-frame pen/mount geometry; never infer product dimensions."""

import hashlib
import io
import json
from dataclasses import dataclass
from pathlib import Path
from xml.etree.ElementTree import Element, SubElement

import fcl
import numpy as np
import trimesh

from ..kinematics import rigid_transform
from .collision import Body, collision_geometry


@dataclass
class ToolModel:
    bodies: list
    tip_radius_m: float | None
    metadata: dict


def positive(value, shape, label):
    values = np.asarray(value, dtype=float)
    if values.shape != shape or not np.isfinite(values).all() or np.any(values <= 0):
        raise ValueError(f"{label} must contain finite positive dimensions")
    return values


def load_tool(profile, config_directory, robot, flange_to_tcp):
    if profile is None:
        return ToolModel([], None, {"configured": False, "visual_bodies": []})
    if not isinstance(profile, dict):
        raise ValueError("Tool profile must be a mapping or null")
    if profile.get("schema_version") != 1 or profile.get("units") != "m":
        raise ValueError("Tool profile requires schema_version 1 and explicit meter units")
    tcp = rigid_transform(profile["flange_to_tcp"], "tool flange-to-TCP")
    if not np.allclose(tcp, flange_to_tcp, atol=1e-9, rtol=0):
        raise ValueError("Tool profile TCP differs from the motion-path TCP")
    tip_radius = float(positive(profile["tip_radius_m"], (), "Tip radius"))
    if not isinstance(profile.get("bodies"), list) or not profile["bodies"]:
        raise ValueError("A configured tool needs non-contact pen/mount geometry")
    names, bodies, visuals, mesh_hashes = set(), [], [], {}
    links = {robot.root, *(j["child"] for j in robot.joint_data)}
    for spec in profile["bodies"]:
        name = spec["name"]
        if not isinstance(name, str) or not name or name in names:
            raise ValueError("Tool body names must be nonempty and unique")
        names.add(name)
        origin = rigid_transform(spec["origin"], f"{name} flange-frame origin")
        touch = spec.get("allowed_touch_links", [])
        if not isinstance(touch, list) or any(not isinstance(link, str) for link in touch):
            raise ValueError("Tool contact exemptions must be a list of robot link names")
        touch = tuple(touch)
        if any(link not in links for link in touch):
            raise ValueError("Tool contact exemptions must name real robot links")
        shape = spec["shape"]
        kind = shape["type"]
        element = Element("geometry")
        visual = {"name": name, "origin": origin.tolist(), "type": kind}
        if kind == "box":
            size = positive(shape["size_m"], (3,), "Box size")
            SubElement(element, "box", size=" ".join(map(str, size)))
            visual["size_m"] = size.tolist()
        elif kind in {"sphere", "cylinder"}:
            radius = float(positive(shape["radius_m"], (), "Body radius"))
            attributes = {"radius": str(radius)}
            visual["radius_m"] = radius
            if kind == "cylinder":
                length = float(positive(shape["length_m"], (), "Cylinder length"))
                attributes["length"] = str(length)
                visual["length_m"] = length
            SubElement(element, kind, **attributes)
        elif kind == "mesh":
            path = (Path(config_directory) / shape["path"]).resolve()
            scale = positive(shape.get("scale", [1, 1, 1]), (3,), "Mesh scale")
            raw = path.read_bytes()
            mesh = trimesh.load_mesh(
                io.BytesIO(raw), file_type=path.suffix.lstrip("."), process=False
            )
            if (
                not isinstance(mesh, trimesh.Trimesh)
                or len(mesh.vertices) < 4
                or not np.isfinite(mesh.vertices).all()
            ):
                raise ValueError("Tool mesh must be a finite triangle mesh")
            hull = trimesh.convex.convex_hull(mesh.vertices * scale)
            # Render the same filled convex hull used by the collision checker.
            visual["vertices"] = hull.vertices.tolist()
            visual["faces"] = hull.faces.tolist()
            mesh_hashes[name] = hashlib.sha256(raw).hexdigest()
            faces = np.column_stack([np.full(len(hull.faces), 3), hull.faces]).ravel()
            geometry = fcl.Convex(
                np.asarray(hull.vertices, dtype=np.float64), len(hull.faces), faces
            )
            radius = float(np.max(np.linalg.norm(hull.vertices, axis=1)))
        else:
            raise ValueError(f"Unsupported tool geometry: {kind}")
        if kind != "mesh":
            geometry, radius = collision_geometry(element)
        bodies.append(Body(f"tool/{name}", robot.tip, origin, geometry, radius, touch))
        visuals.append(visual)
    fingerprint = hashlib.sha256(
        json.dumps(
            {"profile": profile, "mesh_hashes": mesh_hashes},
            sort_keys=True,
            allow_nan=False,
        ).encode()
    ).hexdigest()
    return ToolModel(
        bodies,
        tip_radius,
        {
            "configured": True,
            "profile_sha256": fingerprint,
            "calibration_declared": profile.get("calibrated") is True,
            "frame": "flange",
            "flange_to_tcp": tcp.tolist(),
            "tip_radius_m": tip_radius,
            "visual_bodies": visuals,
            "mesh_hashes": mesh_hashes,
            "limitations": [
                "Contact tip is modeled separately; drawing contact is not yet validated",
                "Cable/filament geometry must be included explicitly if relevant",
            ],
        },
    )


def require_matching_tool(tool, result):
    """Reject caches/audits created for different tool geometry, including meshes."""
    stored = result.get("tool_model", {"configured": False})
    if stored.get("configured") != tool.metadata["configured"] or stored.get(
        "profile_sha256"
    ) != tool.metadata.get("profile_sha256"):
        raise ValueError("Tool geometry changed; rerun the drawing audit and sequence")
