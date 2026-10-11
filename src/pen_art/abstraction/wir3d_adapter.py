"""Pinned WIR3D job preparation and strict output import."""

import shutil
from pathlib import Path

import numpy as np

from ..curves import curve_length, transform_points
from ..io import artifact, read_json, sha256, write_json


def available_budgets(run: Path):
    """List curve outputs without mistaking progress/keypoint JSON for a budget."""
    return sorted(
        int(path.stem.removeprefix("wir3d_"))
        for path in run.glob("wir3d_*.json")
        if path.stem.removeprefix("wir3d_").isdigit()
    )


def selected_candidate_current(run: Path, require_curves=True):
    path = run / "selected_candidates.json"
    if not path.is_file():
        return False
    selected = read_json(path)
    inputs = selected.get("inputs", {})
    expected = {f"wir3d_{b}.json" for b in available_budgets(run)}
    if (run / "geometric_candidates.json").is_file():
        expected.add("geometric_candidates.json")
    if (run / "surface_contours.json").is_file():
        expected.add("surface_contours.json")
    actual = {
        name
        for name in inputs
        if name in {"geometric_candidates.json", "surface_contours.json"}
        or (
            name.startswith("wir3d_")
            and name.removeprefix("wir3d_").removesuffix(".json").isdigit()
        )
    }
    return (
        (bool(selected["curves"]) or not require_curves)
        and {"geometry.glb", "asset_manifest.json", "selection_config.json"} <= inputs.keys()
        and bool(expected)
        and expected == actual
        and all(
            (run / name).is_file() and sha256(run / name) == value for name, value in inputs.items()
        )
        and ((run / "wir3d_keypoints.json").is_file() == ("wir3d_keypoints.json" in inputs))
    )


def prepare_job(run: Path, config: dict):
    manifest = read_json(run / "asset_manifest.json")
    texture = manifest.get("texture", {})
    if texture.get("wir3d_available") is False:
        raise ValueError(texture.get("wir3d_unavailable_reason") or "WIR3D input unavailable")
    settings = config["wir3d"]
    job = artifact(
        "wir3d_job",
        repository="https://github.com/threedle/wir3d.git",
        revision=settings["revision"],
        seed=config["seed"],
        budgets=settings["budgets"],
        geometry_iterations=settings["geometry_iterations"],
        semantic_iterations=settings["semantic_iterations"],
        keypoints=settings["keypoints"],
        input_sha256=sha256(run / "wir3d_bundle/model.obj"),
        resource_sha256={
            path.name: sha256(path)
            for path in sorted((run / "wir3d_bundle").iterdir())
            if path.suffix.lower() in {".obj", ".mtl", ".png", ".jpg", ".jpeg"}
        },
        normalization="Unit sphere, Y up; import through wir3d_to_build transform",
    )
    write_json(run / "wir3d_bundle/job.json", job)
    write_json(run / "wir3d_bundle/asset_manifest.json", read_json(run / "asset_manifest.json"))
    shutil.copyfile(Path(__file__).with_name("wir3d_runner.py"), run / "wir3d_bundle/run_job.py")
    return job


def import_curves(run: Path, checkpoint_json: Path, budget: int):
    raw = read_json(checkpoint_json)
    job = read_json(run / "wir3d_bundle/job.json")
    if budget not in job["budgets"] or raw.get("budget") != budget:
        raise ValueError("WIR3D output budget differs from the requested job budget")
    if raw.get("input_sha256") != job["input_sha256"]:
        raise ValueError("WIR3D output belongs to a different input mesh")
    if raw.get("revision") != job["revision"]:
        raise ValueError("WIR3D revision differs from this job")
    points = np.asarray(raw["control_points"], dtype=float)
    if (
        points.ndim != 3
        or points.shape[1:] != (4, 3)
        or len(points) == 0
        or not np.isfinite(points).all()
    ):
        raise ValueError("WIR3D output must be a finite N x 4 x 3 cubic control-point array")
    matrix = read_json(run / "asset_manifest.json")["transforms"]["wir3d_to_build"]
    physical = transform_points(points, matrix)
    curves = []
    for i, control in enumerate(physical):
        curve = {
            "id": f"wir3d-{budget}-{i:04d}",
            "origin": "wir3d",
            "role": "candidate",
            "segments": [control.tolist()],
            "required": False,
            "diameter_m": 0.001,
        }
        curve["length_m"] = curve_length(curve)
        curves.append(curve)
    document = artifact(
        "curve_candidates",
        method="wir3d",
        frame="build",
        units="m",
        label=f"WIR3D · {len(curves)} curves",
        budget=budget,
        curves=curves,
        total_length_m=sum(c["length_m"] for c in curves),
        provenance=raw.get("provenance", {}),
        limitations=["Connections, support, recognition and execution remain to be validated"],
    )
    write_json(run / f"wir3d_{budget}.json", document)
    return document


def import_keypoints(run, raw):
    job = read_json(run / "wir3d_bundle/job.json")
    if any(raw.get(key) != job[key] for key in ("input_sha256", "revision")):
        raise ValueError("Keypoints belong to a different WIR3D job")
    points = np.asarray(raw["points"], dtype=float)
    if points.shape != (job["keypoints"], 3) or not np.isfinite(points).all():
        raise ValueError("Keypoints must match the configured finite N x 3 array")
    if raw.get("frame") != "wir3d":
        raise ValueError("Expected keypoints in the WIR3D frame")
    matrix = read_json(run / "asset_manifest.json")["transforms"]["wir3d_to_build"]
    result = artifact(
        "automatic_keypoints",
        frame="build",
        units="m",
        points=transform_points(points, matrix).tolist(),
        provenance=raw,
    )
    write_json(run / "wir3d_keypoints.json", result)
    return result
