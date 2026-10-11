"""Pull a running WIR3D job's progress and verified curve outputs over SSH."""

import argparse
import json
import re
import shlex
import subprocess
import time
from pathlib import Path

from PIL import Image

from ..io import read_json, stage_update, write_json
from .comparison import refresh_comparison
from .selection import refresh_selection
from .wir3d_adapter import import_curves, import_keypoints


def sync_previews(run, host, remote_bundle, completed):
    changed = False
    previews = {
        "reference-renders": ("blender_030_360", "reference"),
        "sketch-renders": ("freestyle_030_360", "sketch"),
    }
    for step, (directory, label) in previews.items():
        if step not in completed:
            continue
        for index in (0, 12, 25, 37, 100, 125):
            destination = run / f"wir3d_{label}_{index:04}.png"
            if destination.exists():
                continue
            remote = f"{remote_bundle}/renders/{directory}/{index:04}.png"
            temporary = destination.with_suffix(".png.tmp")
            subprocess.run(
                [
                    "scp",
                    "-o",
                    "BatchMode=yes",
                    "-o",
                    "ConnectTimeout=10",
                    "-o",
                    "ControlMaster=no",
                    "-o",
                    "ControlPath=none",
                    f"{host}:{remote}",
                    str(temporary),
                ],
                check=True,
                capture_output=True,
                timeout=60,
            )
            temporary.replace(destination)
            changed = True
    return changed


def sync_sdf_preview(run, host, remote_bundle):
    """Import the latest complete training plot, without treating it as curves."""
    folder = shlex.quote(f"{remote_bundle}/sdf")
    response = subprocess.run(
        [
            "ssh",
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout=10",
            "-o",
            "ControlMaster=no",
            "-o",
            "ControlPath=none",
            host,
            f"if test -d {folder}; then find {folder} -maxdepth 1 "
            "-type f -name 'step_*.png' -printf '%f\\n'; fi",
        ],
        text=True,
        capture_output=True,
        timeout=30,
        check=True,
    )
    names = sorted(
        name for name in response.stdout.splitlines() if re.fullmatch(r"step_\d{5}\.png", name)
    )
    if not names:
        return False
    name = names[-1]
    destination = run / f"wir3d_sdf_{name}"
    if destination.exists():
        return False
    temporary = destination.with_suffix(".png.tmp")
    subprocess.run(
        [
            "scp",
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout=10",
            "-o",
            "ControlMaster=no",
            "-o",
            "ControlPath=none",
            f"{host}:{remote_bundle}/sdf/{name}",
            str(temporary),
        ],
        capture_output=True,
        timeout=60,
        check=True,
    )
    try:
        with Image.open(temporary) as preview:
            preview.verify()
    except (OSError, SyntaxError):
        temporary.unlink(missing_ok=True)
        return False  # The remote process may still be writing this plot.
    temporary.replace(destination)
    return True


def remote_json(host, filename):
    result = subprocess.run(
        [
            "ssh",
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout=10",
            "-o",
            "ControlMaster=no",
            "-o",
            "ControlPath=none",
            host,
            "cat " + shlex.quote(filename),
        ],
        text=True,
        capture_output=True,
        timeout=30,
        check=True,
    )
    return json.loads(result.stdout)


def sync_curve_previews(run, host, remote_bundle, budget, seed):
    """Copy the newest complete optimizer comparison image from each stage."""
    folder = f"{remote_bundle}/outputs/{int(budget)}/seed{int(seed)}"
    response = subprocess.run(
        [
            "ssh",
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout=10",
            "-o",
            "ControlMaster=no",
            "-o",
            "ControlPath=none",
            host,
            f"if test -d {shlex.quote(folder)}; then find {shlex.quote(folder)} "
            "-maxdepth 3 -type f -path '*/monitoring_compare_renders/*.png' "
            "-printf '%P\\n'; fi",
        ],
        text=True,
        capture_output=True,
        timeout=30,
        check=True,
    )
    latest = {}
    for name in sorted(response.stdout.splitlines()):
        match = re.fullmatch(r"(geo|sem)/monitoring_compare_renders/(\d{5})\.png", name)
        if match:
            latest[match[1]] = (name, match[2])
    changed = False
    for phase, (name, iteration) in latest.items():
        destination = run / f"wir3d_optimization_{budget}_{phase}_{iteration}.png"
        if destination.exists():
            continue
        temporary = destination.with_suffix(".png.tmp")
        subprocess.run(
            [
                "scp",
                "-o",
                "BatchMode=yes",
                "-o",
                "ConnectTimeout=10",
                "-o",
                "ControlMaster=no",
                "-o",
                "ControlPath=none",
                f"{host}:{folder}/{name}",
                str(temporary),
            ],
            capture_output=True,
            timeout=60,
            check=True,
        )
        try:
            with Image.open(temporary) as preview:
                preview.verify()
        except (OSError, SyntaxError):
            temporary.unlink(missing_ok=True)
            continue
        temporary.replace(destination)
        changed = True
    return changed


def sync(run, host, remote_bundle):
    job = read_json(run / "wir3d_bundle/job.json")
    state = remote_json(host, f"{remote_bundle}/progress.json")
    if any(state.get(key) != job[key] for key in ("input_sha256", "revision")):
        raise ValueError("Remote progress belongs to a different WIR3D job")
    previews_changed = sync_previews(run, host, remote_bundle, state["completed"])
    for budget in job["budgets"]:
        if state["step"] == f"curves-{budget}" or f"curves-{budget}" in state["completed"]:
            previews_changed |= sync_curve_previews(run, host, remote_bundle, budget, job["seed"])
    if state["step"] == "surface-distance-field" or "surface-distance-field" in state["completed"]:
        previews_changed |= sync_sdf_preview(run, host, remote_bundle)
    if "automatic-keypoints" in state["completed"] and not (run / "wir3d_keypoints.json").exists():
        import_keypoints(run, remote_json(host, f"{remote_bundle}/keypoints.json"))
    previous = run / "wir3d_progress.json"
    if (
        previous.exists()
        and not previews_changed
        and read_json(previous) == state
        and all(
            (run / f"wir3d_{budget}.json").is_file()
            for budget in job["budgets"]
            if f"curves-{budget}" in state["completed"]
        )
    ):
        refresh_selection(run)
        refresh_comparison(run)
        return state
    write_json(run / "wir3d_progress.json", state)
    available = []
    for budget in job["budgets"]:
        if f"curves-{budget}" not in state["completed"]:
            continue
        if not (run / f"wir3d_{budget}.json").is_file():
            data = remote_json(host, f"{remote_bundle}/curves_{budget}.json")
            destination = run / "wir3d_bundle" / f"curves_{budget}.json"
            write_json(destination, data)
            import_curves(run, destination, budget)
        available.append(budget)
    stage_update(
        run,
        "wir3d",
        state["status"],
        message=f"{host}: {state['step']}",
        budgets=available,
        requested_budgets=job["budgets"],
        remote_updated_at=state["updated_at"],
        host=host,
    )
    refresh_selection(run)
    refresh_comparison(run)
    return state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, default=Path("runs/palace_15cm"))
    parser.add_argument("--host", required=True)
    parser.add_argument("--remote-bundle", required=True)
    parser.add_argument("--watch", action="store_true")
    args = parser.parse_args()
    while True:
        try:
            state = sync(args.run, args.host, args.remote_bundle)
            print(json.dumps(state), flush=True)
            if not args.watch or state["status"] in {"complete", "failed"}:
                return
        except (subprocess.SubprocessError, ValueError) as error:
            print(f"Progress sync failed: {error}", flush=True)
            if not args.watch:
                raise
        time.sleep(15)


if __name__ == "__main__":
    main()
