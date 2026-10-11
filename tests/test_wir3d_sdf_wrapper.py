import subprocess
import sys

from pen_art.abstraction import wir3d_runner


def test_sdf_wrapper_class_is_picklable_in_main_module(tmp_path, monkeypatch):
    repository = tmp_path / "upstream"
    bundle = tmp_path / "bundle"
    (repository / "render").mkdir(parents=True)
    bundle.mkdir()
    for filename in ["render_mesh.py", "sketch_generator.py"]:
        (repository / "render" / filename).write_text("pass\n")
    (repository / "torch.py").write_text("def manual_seed(seed): pass\n")
    (repository / "sdf.py").write_text("""import os
os.environ['PYOPENGL_PLATFORM'] = 'egl'
import pickle
class GeometryDataset:
    pass
assert isinstance(pickle.loads(pickle.dumps(GeometryDataset())), GeometryDataset)
print('dataset checkpoint roundtrip passed')
""")
    monkeypatch.setattr(wir3d_runner, "patched_keypoints", lambda *args: "keypoints")
    monkeypatch.setattr(wir3d_runner, "patched_overlay", lambda *args: "overlay")
    wir3d_runner.prepare_wrappers(repository, bundle, 42)
    result = subprocess.run(
        [sys.executable, str(bundle / "seeded_sdf.py")], capture_output=True, text=True, check=True
    )
    assert "dataset checkpoint roundtrip passed" in result.stdout
