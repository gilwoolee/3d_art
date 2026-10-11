import numpy as np
import pytest

from pen_art.abstraction.wir3d_adapter import import_curves
from pen_art.abstraction.wir3d_runner import recover_sketch_gif
from pen_art.io import read_json, write_json


def test_checkpoint_provenance_and_coordinate_conversion(tmp_path):
    (tmp_path / "wir3d_bundle").mkdir()
    write_json(
        tmp_path / "wir3d_bundle/job.json",
        {"input_sha256": "mesh", "revision": "commit", "budgets": [20]},
    )
    transform = np.diag([0.1, 0.1, 0.1, 1])
    transform[:3, 3] = [0, 0, 0.05]
    write_json(
        tmp_path / "asset_manifest.json", {"transforms": {"wir3d_to_build": transform.tolist()}}
    )
    data = {
        "input_sha256": "wrong",
        "revision": "commit",
        "budget": 20,
        "control_points": [[[0, 0, 0], [0, 1, 0], [1, 1, 0], [1, 0, 0]]],
    }
    write_json(tmp_path / "result.json", data)
    with pytest.raises(ValueError, match="different input"):
        import_curves(tmp_path, tmp_path / "result.json", 20)
    data["input_sha256"] = "mesh"
    write_json(tmp_path / "result.json", data)
    import_curves(tmp_path, tmp_path / "result.json", 20)
    exported = read_json(tmp_path / "wir3d_20.json")
    np.testing.assert_allclose(exported["curves"][0]["segments"][0][-1], [0.1, 0, 0.05])
    assert exported["method"] == "wir3d"


def test_gif_recovery_rejects_unrelated_errors_and_incomplete_renders(tmp_path):
    from PIL import Image

    logs = tmp_path / "logs"
    logs.mkdir()
    log = logs / "sketch-renders.log"
    log.write_text("CUDA failure")
    with pytest.raises(RuntimeError, match="known"):
        recover_sketch_gif(tmp_path)
    log.write_text("FileNotFoundError: ./data/model/renders/freestyle_030_360.gif")
    directory = tmp_path / "renders/freestyle_030_360"
    directory.mkdir(parents=True)
    write_json(
        directory / "renderlist.json",
        {"positions": [[0, 0, 0]] * 150, "lookats": [[0, 0, 0]] * 150},
    )
    Image.new("RGB", (400, 400)).save(directory / "0000.png")
    with pytest.raises(FileNotFoundError):
        recover_sketch_gif(tmp_path)
    assert not (logs / "sketch-gif-recovery.json").exists()


def test_memory_patch_compiles_and_keeps_full_mesh(tmp_path):
    from pathlib import Path

    from pen_art.abstraction.wir3d_runner import patched_keypoints

    repository = Path("external/wir3d").resolve()
    patched_keypoints(repository, tmp_path, 42)
    source = (tmp_path / "memory_backprojection.py").read_text()
    compile(source, "memory_backprojection.py", "exec")
    keypoints = (tmp_path / "generate_keypoints_patched.py").read_text()
    compile(keypoints, "keypoints.py", "exec")
    assert "optim.step()" in source
    assert "chunk_loss.backward()" in source
    assert "foreach=False" in source
    assert "from .model_wrappers" not in source
    assert "igl.read_obj(meshpath)" in keypoints
    assert "features[faces[start:start + 32768]]" in keypoints
    assert "features[faces].mean" not in keypoints


def test_render_cache_migration_requires_unchanged_render_dependency(tmp_path):
    import hashlib

    from pen_art.abstraction.wir3d_runner import (
        WRAPPER_FILES,
        dependency_hash,
        migrate_render_cache,
        step_signature,
    )

    previous = {name: name for name in WRAPPER_FILES}
    current = {
        **previous,
        "generate_keypoints_patched.py": "new",
        "memory_backprojection.py": "new",
    }
    legacy = hashlib.sha256("".join(previous[name] for name in WRAPPER_FILES).encode()).hexdigest()
    steps = [("reference-renders", ["render"]), ("sketch-renders", ["sketch"])]
    (tmp_path / "logs").mkdir()
    for name, folder in [
        ("reference-renders", "blender_030_360"),
        ("sketch-renders", "freestyle_030_360"),
    ]:
        directory = tmp_path / "renders" / folder
        directory.mkdir(parents=True)
        (directory / "renderlist.json").write_text("{}")
        for index in range(150):
            (directory / f"{index}.png").touch()
        command = dict(steps)[name]
        (tmp_path / "logs" / f"{name}.done").write_text(step_signature(command, {}, legacy))
    current["render_sketch.py"] = "changed"
    migrate_render_cache(tmp_path, steps, {}, previous, current)
    assert (tmp_path / "logs/reference-renders.done").read_text() == step_signature(
        ["render"], {}, dependency_hash("reference-renders", current)
    )
    assert (tmp_path / "logs/sketch-renders.done").read_text() == step_signature(
        ["sketch"], {}, legacy
    )
    migration = read_json(tmp_path / "logs/cache-signature-migration.json")
    assert [item["step"] for item in migration] == ["reference-renders"]


def test_keypoints_import_checks_frame_provenance_and_shape(tmp_path):
    from pen_art.abstraction.wir3d_adapter import import_keypoints

    write_json(
        tmp_path / "wir3d_bundle/job.json",
        {"input_sha256": "mesh", "revision": "commit", "keypoints": 2},
    )
    transform = np.diag([0.1, 0.1, 0.1, 1])
    transform[:3, 3] = [0, 0, 0.05]
    write_json(
        tmp_path / "asset_manifest.json", {"transforms": {"wir3d_to_build": transform.tolist()}}
    )
    raw = {
        "input_sha256": "mesh",
        "revision": "commit",
        "frame": "wir3d",
        "points": [[0, 0, 0], [1, 1, 1]],
    }
    result = import_keypoints(tmp_path, raw)
    np.testing.assert_allclose(result["points"], [[0, 0, 0.05], [0.1, 0.1, 0.15]])
    with pytest.raises(ValueError, match="frame"):
        import_keypoints(tmp_path, {**raw, "frame": "build"})
    with pytest.raises(ValueError, match="finite"):
        import_keypoints(tmp_path, {**raw, "points": [[0, 0, 0]]})
