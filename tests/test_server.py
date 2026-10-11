from fastapi.testclient import TestClient

from pen_art.io import sha256, write_json
from pen_art.server import create_app


def test_artifact_routes_are_read_only_and_confined(tmp_path):
    run = tmp_path / "runs/example"
    write_json(run / "run.json", {"id": "example", "stages": {}})
    write_json(run / "curve_candidates.json", {"curves": []})
    (tmp_path / "private.txt").write_text("not public")
    client = TestClient(create_app(tmp_path))
    assert client.get("/api/runs").json()[0]["id"] == "example"
    original = (run / "run.json").read_bytes()
    write_json(run / "cell_config.json", {"drawing_pen_tilt_deg": 30})
    assert client.get("/api/runs").json()[0]["drawing_pen_tilt_deg"] == 30
    assert client.get("/api/runs/example").json()["drawing_pen_tilt_deg"] == 30
    assert (run / "run.json").read_bytes() == original
    before = client.get("/api/runs/example").json()["artifact_revision"]
    write_json(run / "curve_candidates.json", {"curves": [{"id": "new"}]})
    assert client.get("/api/runs/example").json()["artifact_revision"] != before
    write_json(run / "curve_candidates.json", {"curves": []})
    assert client.get("/api/runs/example/artifacts/curve_candidates.json").json() == {"curves": []}
    assert client.get("/api/runs/example/artifacts/..%2F..%2Fprivate.txt").status_code == 404
    assert client.post("/api/runs/example").status_code == 405
    (run / "escape.json").symlink_to(tmp_path / "private.txt")
    assert client.get("/api/runs/example/artifacts/escape.json").status_code == 404


def test_provenance_checks_detect_changed_inputs_without_exposing_outside_files(tmp_path):
    run = tmp_path / "runs/example"
    write_json(run / "run.json", {"id": "example", "stages": {}})
    write_json(run / "input.json", {"version": 1})
    write_json(run / "sequence.json", {"inputs": {"input.json": sha256(run / "input.json")}})
    client = TestClient(create_app(tmp_path))
    endpoint = "/api/runs/example/checks/sequence.json"
    assert client.get(endpoint).json()["current"]
    assert client.get(endpoint).json()["artifact_sha256"] == sha256(run / "sequence.json")
    write_json(run / "input.json", {"version": 2})
    assert client.get(endpoint).json()["stale_inputs"] == ["input.json"]
    (tmp_path / "secret").write_text("not public")
    write_json(run / "sequence.json", {"inputs": {"../../secret": "anything"}})
    assert not client.get(endpoint).json()["current"]
    assert client.post(endpoint).status_code == 405
