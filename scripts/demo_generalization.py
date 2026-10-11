"""Three actual mesh inputs through the same local pipeline, with no per-object tuning.

These generated inputs probe geometry/topology, not recognition performance.
Run from the repository root with .venv/bin/python scripts/demo_generalization.py.
"""

from pathlib import Path

import numpy as np
import trimesh
from PIL import Image

from pen_art.build import build
from pen_art.io import artifact, write_json


def meshes():
    yield "torus", trimesh.creation.torus(major_radius=0.05, minor_radius=0.017)
    profile = np.array(
        [
            [0, 0],
            [0.025, 0],
            [0.035, 0.025],
            [0.028, 0.065],
            [0.015, 0.09],
            [0.022, 0.1],
            [0.019, 0.1],
            [0.012, 0.09],
            [0.025, 0.065],
            [0.032, 0.025],
            [0.022, 0.004],
            [0, 0.004],
        ]
    )
    yield "vase", trimesh.creation.revolve(profile)
    pieces = [trimesh.creation.cylinder(radius=0.012, height=0.10)]
    for a, b in [
        ([0, 0, 0.005], [0.055, 0, 0.075]),
        ([0, 0, 0.02], [-0.04, 0.035, 0.085]),
        ([0, 0, 0.03], [0, -0.045, 0.1]),
    ]:
        pieces.append(trimesh.creation.cylinder(radius=0.008, segment=np.array([a, b])))
    yield "branching", trimesh.util.concatenate(pieces)


def main():
    root = Path(__file__).resolve().parents[1]
    directory = root / "research/generalization"
    directory.mkdir(parents=True, exist_ok=True)
    rows = []
    for name, mesh in meshes():
        uv = (mesh.vertices[:, :2] - mesh.bounds[0, :2]) / np.maximum(mesh.extents[:2], 1e-9)
        texture = np.zeros((16, 16, 3), dtype=np.uint8)
        texture[:] = [194, 142, 94]
        texture[::2, ::2] = [235, 215, 168]
        mesh.visual = trimesh.visual.TextureVisuals(uv=uv, image=Image.fromarray(texture))
        source = directory / f"{name}.glb"
        mesh.export(source)
        try:
            row = build(source, root / f"runs/general_{name}_sparse", "z")
            row["status"] = "STAGES_RECORDED"
        except Exception as error:
            row = {"source": name, "status": "FAILED", "error": str(error)}
        rows.append(row)
        print(name, row, flush=True)
    write_json(
        directory / "sparse_results.json",
        artifact(
            "multi_mesh_demonstration",
            settings="configs/model.yaml; configs/fabrication.yaml",
            per_object_tuning=False,
            cases=rows,
            limitations=[
                "Procedural geometry examples, not a real-object recognition benchmark",
                "Geometric abstraction only; robot and physical validation remain open",
            ],
        ),
    )


if __name__ == "__main__":
    main()
