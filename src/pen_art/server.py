"""The read-only viewer API: planned objects, their files and pictures.

An object planned with ``pen-art`` (``runs/NAME/state.json``) opens by its
name and shows its current planning round. No route can move a robot.
"""

import hashlib
import json
import re
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from .io import read_json


def create_app(root: Path):
    root = root.resolve()
    runs = root / "runs"
    app = FastAPI(title="Pen Art viewer")

    def current_round(folder):
        """An object's current planning round, or None."""
        state = folder / "state.json"
        if not state.is_file():
            return None
        name = read_json(state).get("round")
        if not isinstance(name, str) or not re.fullmatch(r"round-[0-9]+", name):
            return None
        path = (folder / name).resolve()
        inside = path.is_relative_to(folder.resolve())
        return path if inside and (path / "run.json").is_file() else None

    def run_path(run_id):
        if not re.fullmatch(r"[A-Za-z0-9_-]+", run_id):
            raise HTTPException(404, "Run not found")
        path = (runs / run_id).resolve()
        if not path.is_relative_to(runs.resolve()):
            raise HTTPException(404, "Run not found")
        planned = current_round(path)
        if planned is None:
            raise HTTPException(404, "Run not found")
        return planned

    @app.get("/api/runs")
    def list_runs():
        """Every object with a timed plan, named by its folder."""
        rows = []
        for state in sorted(runs.glob("*/state.json")):
            folder = state.parent
            planned = current_round(folder)
            if planned is None or not (planned / "plan.json").is_file():
                continue
            saved = read_json(state)
            verdict = saved.get("verification") or saved.get("timing") or saved.get("geometry")
            minutes = saved.get("print_minutes")
            row = read_json(planned / "run.json")
            row.update(
                id=folder.name,
                name=f"{folder.name}: {verdict}"
                + (f", {minutes:.1f} min" if isinstance(minutes, (int, float)) else ""),
            )
            rows.append(row)
        return rows

    @app.get("/api/runs/{run_id}")
    def get_run(run_id: str):
        path = run_path(run_id)
        data = read_json(path / "run.json")
        data["artifacts"] = sorted(
            p.name for p in path.iterdir() if p.is_file() and p.suffix in {".json", ".png"}
        )
        # Workers update artifacts atomically between stage transitions. Let the
        # viewer notice those checkpoints without hashing large trajectory files.
        stamps = []
        for name in data["artifacts"]:
            source = path / name
            if not source.resolve().is_relative_to(path):
                continue
            try:
                stat = source.stat()
            except FileNotFoundError:
                continue
            stamps.append((name, stat.st_mtime_ns, stat.st_size))
        data["artifact_revision"] = hashlib.sha256(json.dumps(stamps).encode()).hexdigest()
        return data

    @app.get("/api/runs/{run_id}/artifacts/{filename}")
    def get_artifact(run_id: str, filename: str):
        if not re.fullmatch(r"[A-Za-z0-9_.-]+\.(json|glb|png)", filename):
            raise HTTPException(404, "Artifact not found")
        directory = run_path(run_id)
        path = (directory / filename).resolve()
        if not path.is_relative_to(directory) or not path.is_file():
            raise HTTPException(404, "Artifact not found")
        return FileResponse(path, headers={"Cache-Control": "no-cache"})

    @app.get("/api/runs/{run_id}/picture")
    def get_picture(run_id: str):
        """The object's original picture from objs/: beside its source mesh, or
        a picture in objs/ named like the mesh's folder (ghost.jpg, cute_ghost/)."""
        manifest = run_path(run_id) / "asset_manifest.json"
        name = read_json(manifest).get("source", {}).get("name") if manifest.is_file() else None
        objs = root / "objs"
        if not name or not objs.is_dir():
            raise HTTPException(404, "No picture")

        def pictures(folder):
            return sorted(
                p for p in folder.iterdir() if p.suffix.lower() in {".jpg", ".jpeg", ".png"}
            )

        for mesh in sorted(p for p in objs.rglob(name) if p.is_file()):
            beside = pictures(mesh.parent) if mesh.parent != objs else []
            if beside:
                return FileResponse(beside[0], headers={"Cache-Control": "no-cache"})
            words = {mesh.parent.name.lower(), mesh.stem.lower()}
            for picture in pictures(objs):
                stem = picture.stem.lower()
                if any(stem in word or word in stem for word in words):
                    return FileResponse(picture, headers={"Cache-Control": "no-cache"})
        raise HTTPException(404, "No picture")

    dist = root / "web/dist"
    if dist.is_dir():
        app.mount("/", StaticFiles(directory=dist, html=True), name="viewer")
    return app
