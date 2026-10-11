"""Versioned, atomic artifacts shared by the pipeline and visual workbench."""

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


def write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}.")
    try:
        with os.fdopen(fd, "w") as file:
            json.dump(data, file, indent=2, allow_nan=False)
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
