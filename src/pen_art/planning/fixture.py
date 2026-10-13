"""Provisional centered object-footprint risers, separate from the broad plate."""

import fcl
import numpy as np

from .collision import placed


def riser_geometry(spec):
    height = float(spec["height_m"])
    if not np.isfinite(height) or height <= 0:
        raise ValueError("Riser height must be finite and positive")
    if spec["type"] == "box":
        size = np.asarray(spec["size_m"], dtype=float)
        if size.shape != (2,) or not np.isfinite(size).all() or np.any(size <= 0):
            raise ValueError("Riser footprint must contain two positive dimensions")
        # Fixed obstacles remain identical at all four quarter-turn indices.
        if not np.isclose(size[0], size[1], atol=1e-12, rtol=0):
            raise ValueError("Indexed box riser must have a square footprint")
        return fcl.Box(*size, height), float(np.linalg.norm(size) / 2)
    if spec["type"] == "cylinder":
        radius = float(spec["radius_m"])
        if not np.isfinite(radius) or radius <= 0:
            raise ValueError("Riser radius must be finite and positive")
        return fcl.Cylinder(radius, height), radius
    raise ValueError("Riser must be a centered box or cylinder")


def riser_object(spec, world_from_build):
    geometry, radius = riser_geometry(spec)
    local = np.eye(4)
    local[2, 3] = -float(spec["height_m"]) / 2
    return placed(geometry, np.asarray(world_from_build) @ local), radius
