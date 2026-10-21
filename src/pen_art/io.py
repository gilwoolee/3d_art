"""Versioned, atomic JSON and file writes shared by every step and the viewer."""

import hashlib
import json
import os
import tempfile
from datetime import UTC, datetime
from pathlib import Path

SCHEMA_VERSION = 1


def now() -> str:
    return datetime.now(UTC).isoformat()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path: Path) -> dict:
    return json.loads(path.read_text())


def write_json(path: Path, data: dict, *, compact=False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}.")
    try:
        with os.fdopen(fd, "w") as file:
            json.dump(data, file, indent=None if compact else 2,
                      separators=(",", ":") if compact else None, allow_nan=False)
            file.write("\n")
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def artifact(kind: str, **values) -> dict:
    return {"schema_version": SCHEMA_VERSION, "kind": kind, **values}


def stage_update(run: Path, stage: str, status: str, **values) -> None:
    path = run / "run.json"
    state = read_json(path)
    state["stages"][stage] = {"status": status, "updated_at": now(), **values}
    state["updated_at"] = now()
    write_json(path, state)


def asset_path(folder: Path, name: str) -> Path:
    """Resolve a saved input through its explicit, object-relative asset reference."""
    folder = Path(folder)
    manifest = read_json(folder / "asset_manifest.json")
    if manifest.get("schema_version") != 2:
        raise ValueError("Unsupported asset format: run `pen-art frame` again")
    object_root = (folder / manifest["object_root"]).resolve()
    path = (
        object_root / manifest["files"][name]
        if name in manifest["files"] else folder / name
    ).resolve()
    if not path.is_relative_to(object_root):
        raise ValueError(f"Asset escapes the object directory: {name}")
    return path


def reference_assets(source: Path, target: Path, additions=None) -> None:
    """Write references to shared object assets; copy no mesh or robot data."""
    source, target = Path(source), Path(target)
    manifest = read_json(source / "asset_manifest.json")
    object_root = (source / manifest["object_root"]).resolve()
    files = dict(manifest["files"])
    for name, path in (additions or {}).items():
        files[name] = str(Path(path).resolve().relative_to(object_root))
    manifest.update(files=files, object_root=os.path.relpath(object_root, target.resolve()))
    write_json(target / "asset_manifest.json", manifest)
