"""Read-only local visual workbench API. No route can execute robot commands."""

import argparse
import hashlib
import json
import math
import re
from pathlib import Path

import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from .io import read_json, sha256


def create_app(root: Path):
    root = root.resolve()
    runs = root / "runs"
    app = FastAPI(title="Pen Art Workbench")

    def run_path(run_id):
        if not re.fullmatch(r"[A-Za-z0-9_-]+", run_id):
            raise HTTPException(404, "Run not found")
        path = (runs / run_id).resolve()
        if not path.is_relative_to(runs.resolve()) or not (path / "run.json").is_file():
            raise HTTPException(404, "Run not found")
        return path

    def run_metadata(path):
        data = read_json(path / "run.json")
        config = path / "cell_config.json"
        if config.is_file() and config.resolve().is_relative_to(path.resolve()):
            tilt = read_json(config).get("drawing_pen_tilt_deg", 0)
            if (
                isinstance(tilt, (int, float))
                and not isinstance(tilt, bool)
                and math.isfinite(tilt)
            ):
                data["drawing_pen_tilt_deg"] = tilt
        return data

    @app.get("/api/runs")
    def list_runs():
        # ``runs/viewer_runs.json`` names the runs to offer, in order. Without
        # it every run is listed. Any run still opens by its own address.
        chosen = runs / "viewer_runs.json"
        if chosen.is_file():
            return [
                run_metadata(runs / name)
                for name in read_json(chosen)
                if isinstance(name, str)
                and re.fullmatch(r"[A-Za-z0-9_-]+", name)
                and (runs / name / "run.json").is_file()
            ]
        return [run_metadata(path.parent) for path in sorted(runs.glob("*/run.json"))]

    @app.get("/api/runs/{run_id}")
    def get_run(run_id: str):
        path = run_path(run_id)
        data = run_metadata(path)
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

    @app.get("/api/runs/{run_id}/checks/{filename}")
    def check_inputs(run_id: str, filename: str):
        directory = run_path(run_id)
        if not re.fullmatch(r"[A-Za-z0-9_.-]+\.json", filename):
            raise HTTPException(404, "Artifact not found")
        path = (directory / filename).resolve()
        if not path.is_relative_to(directory) or not path.is_file():
            raise HTTPException(404, "Artifact not found")
        raw = path.read_bytes()
        document = json.loads(raw)
        stale = []
        for name, expected in document.get("inputs", {}).items():
            source = (directory / name).resolve()
            if (
                not re.fullmatch(r"[A-Za-z0-9_.-]+(?:/[A-Za-z0-9_.-]+)*", name)
                or any(part in {".", ".."} for part in Path(name).parts)
                or not source.is_relative_to(directory)
                or not source.is_file()
                or sha256(source) != expected
            ):
                stale.append(name)
        return {
            "current": not stale,
            "stale_inputs": stale,
            "artifact_sha256": hashlib.sha256(raw).hexdigest(),
        }

    dist = root / "web/dist"
    if dist.is_dir():
        app.mount("/", StaticFiles(directory=dist, html=True), name="workbench")
    return app


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path.cwd())
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    uvicorn.run(create_app(args.root), host="127.0.0.1", port=args.port)


if __name__ == "__main__":
    main()
