from pathlib import Path
from types import SimpleNamespace

import pytest
from PIL import Image

from pen_art.abstraction.remote import sync, sync_curve_previews, sync_sdf_preview
from pen_art.io import write_json


def test_remote_progress_does_not_accept_another_mesh(tmp_path, monkeypatch):
    write_json(
        tmp_path / "wir3d_bundle/job.json",
        {"input_sha256": "this-mesh", "revision": "commit", "budgets": [20]},
    )
    monkeypatch.setattr(
        "pen_art.abstraction.remote.remote_json",
        lambda *args: {"input_sha256": "other-mesh", "revision": "commit", "completed": []},
    )
    with pytest.raises(ValueError, match="different WIR3D job"):
        sync(tmp_path, "example", "/job")
    assert not (tmp_path / "wir3d_progress.json").exists()


@pytest.mark.parametrize("complete", [True, False])
def test_sdf_preview_imports_latest_only_after_png_is_complete(tmp_path, monkeypatch, complete):
    calls = []

    def command(arguments, **options):
        calls.append(arguments)
        if arguments[0] == "ssh":
            return SimpleNamespace(stdout="step_00000.png\nstep_00500.png\nwrong-name.png\n")
        assert arguments[-2].endswith("/sdf/step_00500.png")
        destination = Path(arguments[-1])
        if complete:
            Image.new("RGB", (2, 2)).save(destination, format="PNG")
        else:
            destination.write_bytes(b"incomplete")
        return SimpleNamespace()

    monkeypatch.setattr("pen_art.abstraction.remote.subprocess.run", command)
    assert sync_sdf_preview(tmp_path, "example", "/job") is complete
    assert (tmp_path / "wir3d_sdf_step_00500.png").exists() is complete
    assert not (tmp_path / "wir3d_sdf_step_00500.png.tmp").exists()
    if complete:
        assert not sync_sdf_preview(tmp_path, "example", "/job")
        assert len(calls) == 3  # Listing may repeat; a verified image is not copied again.


@pytest.mark.parametrize("complete", [True, False])
def test_optimizer_preview_copies_latest_valid_stage_image(tmp_path, monkeypatch, complete):
    copies = []

    def command(arguments, **options):
        if arguments[0] == "ssh":
            return SimpleNamespace(
                stdout=(
                    "geo/monitoring_compare_renders/00000.png\n"
                    "geo/monitoring_compare_renders/00100.png\n"
                    "sem/monitoring_compare_renders/00050.png\n"
                    "../../outside.png\n"
                )
            )
        copies.append(arguments[-2])
        destination = Path(arguments[-1])
        if complete:
            Image.new("RGB", (2, 2)).save(destination, format="PNG")
        else:
            destination.write_bytes(b"unfinished")
        return SimpleNamespace()

    monkeypatch.setattr("pen_art.abstraction.remote.subprocess.run", command)
    assert sync_curve_previews(tmp_path, "example", "/job", 20, 42) is complete
    assert len(copies) == 2
    assert copies[0].endswith("geo/monitoring_compare_renders/00100.png")
    assert not list(tmp_path.glob("*.tmp"))
    assert len(list(tmp_path.glob("*.png"))) == (2 if complete else 0)
    if complete:
        assert not sync_curve_previews(tmp_path, "example", "/job", 20, 42)
        assert len(copies) == 2
