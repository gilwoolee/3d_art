"""Geometric supplemental candidates. This is explicitly not WIR3D output."""

import numpy as np
import trimesh

from ..curves import curve_length, fit_polyline
from ..io import artifact, write_json


def extract_candidates(run, config):
    data = np.load(run / "geometry.npz")
    mesh = trimesh.Trimesh(data["vertices"], data["faces"], process=False)
    settings = config["geometric_candidates"]
    candidates = []
    for axis in range(3):
        normal = np.eye(3)[axis]
        low, high = mesh.bounds[:, axis]
        for level in np.linspace(low, high, settings["sections_per_axis"] + 2)[1:-1]:
            origin = np.zeros(3)
            origin[axis] = level
            section = mesh.section(plane_origin=origin, plane_normal=normal)
            if section is None:
                continue
            for points in section.discrete:
                length = np.linalg.norm(np.diff(points, axis=0), axis=1).sum()
                if length < settings["min_length_m"]:
                    continue
                # Record source sampling density separately from Bézier piece count.
                segments = fit_polyline(points, settings["fit_tolerance_m"])
                curve = {
                    "id": f"section-{axis}-{len(candidates):04d}",
                    "role": "candidate",
                    "origin": "geometric_section",
                    "axis": axis,
                    "section_m": float(level),
                    "segments": segments,
                    "closed": bool(np.linalg.norm(points[0] - points[-1]) < 1e-7),
                    "diameter_m": 0.001,
                    "required": False,
                    "source_sample_count": len(points),
                }
                curve["length_m"] = curve_length(curve)
                # This score is a geometric coverage proxy, never labeled recognition.
                curve["geometric_score"] = float(np.sqrt(length))
                candidates.append(curve)
    candidates.sort(key=lambda c: (-c["geometric_score"], c["id"]))
    candidates = candidates[: settings["max_curves"]]
    document = artifact(
        "curve_candidates",
        units="m",
        frame="build",
        method="geometric_sections",
        label="Geometric supplemental candidates",
        curves=candidates,
        total_length_m=sum(c["length_m"] for c in candidates),
        fit_tolerance_m=settings["fit_tolerance_m"],
        limitations=[
            "No WIR3D visual optimization",
            "No physical junctions or supports",
            "No fabrication or recognition validation",
        ],
    )
    write_json(run / "geometric_candidates.json", document)
    return document
