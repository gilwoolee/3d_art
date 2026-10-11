"""Queue the GPU stages after a separately launched Blender preprocessing run."""

import json
import os
import subprocess
import time
from pathlib import Path

root = Path(__file__).resolve().parent
bundle = root / "wir3d_bundle"
deadline = time.monotonic() + 6 * 3600
while time.monotonic() < deadline:
    state = json.loads((bundle / "progress.json").read_text())
    if state["status"] == "failed":
        raise SystemExit(f"Preprocessing failed: {state['step']}")
    if state["status"] == "awaiting_compute" and "sketch-renders" in state["completed"]:
        break
    time.sleep(15)
else:
    raise SystemExit("Preprocessing did not finish within six hours")

environment = dict(os.environ)
environment.update(CUDA_HOME="/usr/local/cuda-12.8", TORCH_CUDA_ARCH_LIST="12.0", MAX_JOBS="4")
environment["PATH"] = "/usr/local/cuda-12.8/bin:" + environment["PATH"]
command = [str(root / ".venv-wir3d/bin/python"), str(bundle / "run_job.py"),
           "--repo", str(root / "external/wir3d"), "--blender",
           str(root / "tools/blender-3.6.23-linux-x64/blender"), "--resume"]
raise SystemExit(subprocess.call(command, cwd=root, env=environment))
