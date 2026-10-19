"""Identities of the collision context a path proof was made in.

A saved proof is reusable only while the robot, the pen, the fixed obstacles
and the settings it was checked against are unchanged; these hashes bind
them. ``VALIDATOR_VERSION`` is part of saved hashes: changing it invalidates
every saved proof.
"""

import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from time import monotonic

import numpy as np

from .collision import RobotCollisionModel

VALIDATOR_VERSION = "motion-clearance-lipschitz-v1"


def _plain(value):
    if hasattr(value, "tolist"):  # NumPy arrays and scalars
        return value.tolist()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(f"Cannot serialize {type(value).__name__}")


def _bytes(value):
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False, default=_plain
    ).encode()


def digest(value):
    return hashlib.sha256(_bytes(value)).hexdigest()


def collision_context_hash(model, geometry_context=None):
    """Bind source meshes, kinematics and supplied extra geometry/calibration.

    ``geometry_context`` must describe any opaque FCL convex extras (tool
    metadata already includes mesh hashes). Static obstacle identities/poses are
    additionally bound per query; their replacement invalidates cached bounds.
    """
    robot = model.robot
    meshes = {}
    for mesh in robot.xml.findall(".//mesh"):
        path = Path(mesh.get("filename"))
        meshes[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    return digest(
        {
            "validator": VALIDATOR_VERSION,
            "xml": ET.tostring(robot.xml).decode(),
            "meshes": meshes,
            "world_from_base": getattr(robot, "world_from_base", np.eye(4)),
            "lower": robot.lower,
            "upper": robot.upper,
            "extra_geometry": geometry_context,
            "bodies": [
                {
                    "name": b.name,
                    "link": b.link,
                    "origin": b.origin,
                    "radius": b.radius,
                    "touch_links": b.touch_links,
                    "shape": _shape(b.geometry),
                }
                for b in model.bodies
            ],
            "pairs": [(a, b, r) for (a, b), r in model._pair_radii.items()],
        }
    )


def _shape(geometry):
    return {
        "type": type(geometry).__name__,
        **{
            name: getattr(geometry, name)
            for name in ("side", "radius", "lz")
            if hasattr(geometry, name)
        },
    }


def obstacle_context(obstacles, static_geometry=None, references=None):
    """Bind actual poses; unknown opaque shapes are reusable only in-process."""
    objects, persistent = {}, True
    for name, obj in (obstacles or {}).items():
        shape = (static_geometry or {}).get(name)
        if shape is not None and references is not None:
            original = references.setdefault(("declared", name), obj)
            if original is not obj:
                # FCL does not expose replacement geometry dimensions. The
                # caller's old shape signature cannot certify a new object.
                shape = None
        if shape is None:
            persistent = False
            if references is not None:
                references[id(obj)] = obj
            shape = {"process_identity": id(obj)}
        objects[name] = {
            "shape": shape,
            "node_type": obj.getNodeType(),
            "rotation": obj.getRotation(),
            "translation": obj.getTranslation(),
        }
    return ("static:" if persistent else "process:") + digest(objects)


class MaterialOnlyCollisionModel:
    """Recheck fresh material after an exact-path self/static proof is found.

    This wrapper grants no proof itself. Its caller must hold a certificate for
    the complete unchanged interpolant and physical/static context.
    """

    check_bounded_path = RobotCollisionModel.check_bounded_path
    check_edge = RobotCollisionModel.check_edge
    check = RobotCollisionModel.check

    def __init__(self, model):
        self.model = model

    def __getattr__(self, name):
        return getattr(self.model, name)

    def distances(self, positions, obstacles=None, *, deadline=float("inf"), only=None):
        if monotonic() >= deadline:
            raise TimeoutError("Material verification exceeded its budget")
        self.robot.frames(positions)
        yield from ()



def static_shapes(profile):
    environment = profile["environment"]
    return {
        "table": {"type": "Box", "side": environment["table_size_m"]},
        "plate": {
            "type": "Box",
            "side": [
                *environment["plate_size_m"],
                environment["plate_top_z_m"] - environment["table_z_m"],
            ],
        },
    }

