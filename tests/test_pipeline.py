from pathlib import Path

import pytest
import yaml

from pen_art import pipeline
from pen_art.io import write_json


@pytest.fixture
def isolated_pipeline(tmp_path, monkeypatch):
    monkeypatch.setattr(pipeline, "initialize", lambda _: ({}, tmp_path / "source", tmp_path))
    monkeypatch.setattr(pipeline, "prepare", lambda *args: None)
    return tmp_path


def test_graph_rejects_explicit_stale_selection(isolated_pipeline, monkeypatch, capsys):
    monkeypatch.setattr("sys.argv", ["pipeline", "--stage", "graph", "--candidate", "selected"])
    with pytest.raises(SystemExit) as error:
        pipeline.main()
    assert error.value.code == 2
    assert "Selected curves are stale" in capsys.readouterr().err


def test_auto_graph_rejects_comparison_with_different_fabrication_assumptions(
    isolated_pipeline, monkeypatch, capsys
):
    settings = yaml.safe_load(Path("configs/fabrication.yaml").read_text())
    settings["process"]["nominal_speed_m_s"] *= 2
    write_json(isolated_pipeline / "comparison_config.json", {"fabrication": settings})
    monkeypatch.setattr("sys.argv", ["pipeline", "--stage", "graph"])
    with pytest.raises(SystemExit) as error:
        pipeline.main()
    assert error.value.code == 2
    assert "Fabrication assumptions changed" in capsys.readouterr().err
