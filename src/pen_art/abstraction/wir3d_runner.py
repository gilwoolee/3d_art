"""Portable runner. Execute this file in the upstream WIR3D Python environment."""

import argparse
import ast
import hashlib
import importlib.util
import json
import subprocess
import sys
import sysconfig
from datetime import datetime, timezone
from pathlib import Path

WRAPPER_FILES = (
    "generate_keypoints_patched.py",
    "render_reference.py",
    "render_sketch.py",
    "seeded_sdf.py",
)


def replace_once(source, old, new):
    if source.count(old) != 1:
        raise RuntimeError("Pinned WIR3D source changed; review memory patch: " + old[:80])
    return source.replace(old, new)


def memory_backprojection(repository, bundle):
    """Chunk separable vertex features without changing per-view Adam updates."""
    source = (repository / "backto3d/feature_backprojection/backprojection.py").read_text()
    node = next(
        n
        for n in ast.parse(source).body
        if isinstance(n, ast.FunctionDef) and n.name == "features_from_renders"
    )
    source = ast.get_source_segment(source, node)
    source = replace_once(
        source,
        "vertex_features = torch.zeros(len(vertices), emb_dim).to(device)\n"
        "    vertex_features.requires_grad_()\n"
        "    optim = torch.optim.Adam([vertex_features], lr=0.1)",
        "chunk_size = 32768\n"
        "    chunks = [torch.zeros(min(chunk_size, len(vertices) - start), emb_dim,\n"
        "                          device=device, requires_grad=True)\n"
        "              for start in range(0, len(vertices), chunk_size)]\n"
        "    optim = torch.optim.Adam(chunks, lr=0.1, foreach=False)",
    )
    # Preserve the batch dimension for batch size one (upstream squeeze loses it).
    source = replace_once(
        source,
        "visible_mask = (zbuffer_samples + 0.0005 >= zimage.squeeze())",
        "visible_mask = (zbuffer_samples.reshape(len(renders), -1) + 0.0005 >= "
        "zimage.reshape(len(renders), -1))",
    )
    start = source.index("            features_per_view = get_feature_for_pixel_location(")
    end = source.index('            print(f"Epoch {epoch}: Initial loss:', start)
    source = (
        source[:start]
        + """            optim.zero_grad(set_to_none=True)
            loss = torch.zeros((), device=device)
            for chunk_index, chunk in enumerate(chunks):
                start = chunk_index * chunk_size
                end = start + len(chunk)
                features_per_view = get_feature_for_pixel_location(
                    processed_images, vertices_pixels[:, start:end], image_size=res,
                    patch_size=model.patch_size(), use_sam=isinstance(model, SAMWrapper))
                chunk_loss = torch.sum((features_per_view -
                    chunk * visible_mask[:, start:end].unsqueeze(-1)) ** 2)
                chunk_loss.backward()
                loss += chunk_loss.detach()
                del features_per_view, chunk_loss
            optim.step()

"""
        + source[end:]
    )
    source = replace_once(
        source,
        "    return vertex_features",
        "    optim.zero_grad(set_to_none=True)\n"
        "    del optim\n"
        "    return torch.cat([chunk.detach() for chunk in chunks])",
    )
    source = replace_once(
        source,
        "from .model_wrappers import SAMWrapper",
        "from backto3d.feature_backprojection.model_wrappers import SAMWrapper",
    )
    # Normals are unused in this function, and create large unnecessary tensors.
    source = "\n".join(
        line
        for line in source.splitlines()
        if not line.strip().startswith(("fnormals =", "vertexnormals ="))
    )
    header = (
        "import torch\n"
        "from backto3d.feature_backprojection.backprojection import "
        "get_feature_for_pixel_location\n"
    )
    content = header + source + "\n"
    compile(content, "memory_backprojection.py", "exec")
    (bundle / "memory_backprojection.py").write_text(content)
    return hashlib.sha256(content.encode()).hexdigest()


def wrapper_hashes(bundle):
    return {
        name: hashlib.sha256((bundle / name).read_bytes()).hexdigest()
        for name in (*WRAPPER_FILES, "memory_backprojection.py", "render_kp_overlay_patched.py")
        if (bundle / name).is_file()
    }


def dependency_hash(name, hashes):
    dependencies = {
        "reference-renders": ["render_reference.py"],
        "sketch-renders": ["render_sketch.py"],
        "surface-renders": [],
        "automatic-keypoints": [
            "generate_keypoints_patched.py",
            "memory_backprojection.py",
            "render_kp_overlay_patched.py",
            "render_reference.py",
            "render_sketch.py",
        ],
        "surface-distance-field": ["seeded_sdf.py"],
    }.get(name, list(hashes))
    return hashlib.sha256(
        json.dumps({key: hashes[key] for key in dependencies}, sort_keys=True).encode()
    ).hexdigest()


def migrate_render_cache(bundle, steps, job, previous, current):
    """Reuse legacy renders only when their old signature and dependencies match."""
    if not all(name in previous for name in WRAPPER_FILES):
        return
    legacy = hashlib.sha256("".join(previous[name] for name in WRAPPER_FILES).encode()).hexdigest()
    migrated = []
    for name, command in steps:
        if name not in {"reference-renders", "sketch-renders", "surface-renders"}:
            continue
        done = bundle / "logs" / f"{name}.done"
        if (
            done.is_file()
            and done.read_text() == step_signature(command, job, legacy)
            and dependency_hash(name, previous) == dependency_hash(name, current)
            and outputs_exist(bundle, name)
        ):
            old_signature = done.read_text()
            new_signature = step_signature(command, job, dependency_hash(name, current))
            done.write_text(new_signature)
            migrated.append({"step": name, "old": old_signature, "new": new_signature})
    if migrated:
        (bundle / "logs/cache-signature-migration.json").write_text(json.dumps(migrated, indent=2))


def patched_overlay(repository, bundle):
    source = (repository / "render_kp_overlay.py").read_text()
    source = replace_once(source, "torch.any(keypoint > 1)", "torch.any(keypoint >= 1)")
    source = replace_once(
        source,
        "torch.tensor([512, 512], device=device).float()",
        "torch.tensor([zbuffer.shape[1], zbuffer.shape[0]], device=device).float()",
    )
    content = f"import sys; sys.path.insert(0, {str(repository)!r})\n" + source
    (bundle / "render_kp_overlay_patched.py").write_text(content)
    return hashlib.sha256(content.encode()).hexdigest()


def recover_keypoint_overlay(bundle, repository, job, previous_source):
    log = (bundle / "logs/automatic-keypoints.log").read_text()
    current_source = (bundle / "generate_keypoints_patched.py").read_text()
    if (
        "Done with keypoint generation" not in log
        or "IndexError" not in log
        or "render_kp_overlay.py" not in log
        or "out of bounds" not in log
    ):
        raise RuntimeError("Recovery requires the known keypoint-overlay indexing failure")
    expected = previous_source.replace(
        "from render_kp_overlay import render_kp_overlay",
        "from render_kp_overlay_patched import render_kp_overlay",
    )
    if expected != current_source:
        raise RuntimeError("Extraction code changed; cannot reuse failed-stage keypoints")
    export_keypoints(bundle, job)  # Validate exact count and finite saved coordinates.
    command = [
        sys.executable,
        str(bundle / "render_kp_overlay_patched.py"),
        bundle.name,
        str(bundle.parent),
        "--keypoints_path",
        str(bundle / "keypoints.pt"),
        "--zbuffer",
        str(bundle / "renders/surface_030_360_zbuffer"),
        "--renderset",
        "surface_030_360",
        "--rendername",
        f"kp_{bundle.name}",
        "--overwrite",
    ]
    with (bundle / "logs/keypoint-overlay-recovery.log").open("w") as output:
        subprocess.run(command, cwd=repository, stdout=output, stderr=subprocess.STDOUT, check=True)
    from PIL import Image

    directory = bundle / "renders" / f"kp_{bundle.name}_occ"
    for index in range(150):
        with Image.open(directory / f"{index:04}.png") as picture:
            picture.load()
            if picture.size != (400, 400):
                raise ValueError("Unexpected overlay image dimensions")
    proof = {
        "input_sha256": job["input_sha256"],
        "keypoints": json.loads((bundle / "keypoints.json").read_text())["checkpoint_sha256"],
        "extraction_unchanged": True,
        "preview_count": 150,
        "original_failure_sha256": hashlib.sha256(log.encode()).hexdigest(),
    }
    (bundle / "logs/keypoint-overlay-recovery.json").write_text(json.dumps(proof, indent=2))


def patched_keypoints(repository, bundle, seed):
    """Keep the pinned checkout intact; record the exact compatibility patch."""
    source = (repository / "generate_keypoints_final.py").read_text()
    old = "img = torch.from_numpy(np.array(Image.open(imgpath))) / 255."
    new = (
        'img = torch.from_numpy(np.array(Image.open(imgpath).convert("RGB")))'
        ".permute(2, 0, 1).float() / 255."
    )
    if source.count(old) != 1:
        raise RuntimeError("Pinned keypoint loader changed; review the HWC-to-CHW patch")
    preamble = (
        f"import sys; sys.path.insert(0, {str(repository)!r})\n"
        "import random, numpy as np, torch\n"
        f"random.seed({seed}); np.random.seed({seed}); torch.manual_seed({seed})\n"
    )
    memory_hash = memory_backprojection(repository, bundle)
    source = replace_once(
        source,
        "from render_kp_overlay import render_kp_overlay",
        "from render_kp_overlay_patched import render_kp_overlay",
    )
    source = replace_once(
        source,
        "from backto3d.feature_backprojection.backprojection import features_from_renders",
        "from memory_backprojection import features_from_renders",
    )
    source = replace_once(
        source,
        "        face_centroid_features = features[faces].mean(dim=1)\n\n"
        "        dists = torch.norm(face_centroid_features - keypoints[[i]], dim=-1)\n"
        "        closest_face_idx.append(torch.argmin(dists).item())",
        "        best_distance, best_index = float('inf'), None\n"
        "        for start in range(0, len(faces), 32768):\n"
        "            values = features[faces[start:start + 32768]].mean(dim=1)\n"
        "            distances = torch.norm(values - keypoints[[i]], dim=-1)\n"
        "            distance, index = distances.min(dim=0)\n"
        "            if distance.item() < best_distance:\n"
        "                best_distance, best_index = distance.item(), start + index.item()\n"
        "        closest_face_idx.append(best_index)",
    )
    patched = (
        preamble + f"# memory_backprojection_sha256: {memory_hash}\n" + source.replace(old, new)
    )
    path = bundle / "generate_keypoints_patched.py"
    path.write_text(patched)
    return hashlib.sha256(patched.encode()).hexdigest()


def prepare_wrappers(repository, bundle, seed):
    """Use isolated dependencies; disable upstream's unpinned runtime pip installs."""
    hashes = [patched_keypoints(repository, bundle, seed), patched_overlay(repository, bundle)]
    for name, script in (
        ("render_reference", repository / "render/render_mesh.py"),
        ("render_sketch", repository / "render/sketch_generator.py"),
        ("seeded_sdf", repository / "sdf.py"),
    ):
        source = script.read_text()
        if name == "seeded_sdf":
            source = replace_once(
                source,
                "os.environ['PYOPENGL_PLATFORM'] = 'egl'",
                "os.environ['PYOPENGL_PLATFORM'] = 'osmesa'",
            )
        lines = []
        for line in source.splitlines():
            if line.lstrip().startswith(("pip.main(", "import pip")):
                line = line[: len(line) - len(line.lstrip())] + "pass  # dependencies preinstalled"
            lines.append(line)
        source = "\n".join(lines) + "\n"
        preamble = (
            "import sys\n"
            f"sys.path.insert(0, {sysconfig.get_paths()['purelib']!r})\n"
            f"sys.path.insert(0, {str(repository)!r})\n"
        )
        if name == "seeded_sdf":
            preamble += "import os; os.environ['PYOPENGL_PLATFORM'] = 'osmesa'\n"
            preamble += (
                "import random, numpy as np, torch\n"
                f"random.seed({seed}); np.random.seed({seed}); torch.manual_seed({seed})\n"
            )
        wrapper = preamble + (
            f"exec(compile({source!r}, {str(script)!r}, 'exec'), "
            f"{{'__file__': {str(script)!r}, '__name__': '__main__', '__package__': None}})\n"
        )
        if name == "seeded_sdf":
            # torch.save pickles GeometryDataset by its __main__ module name.
            # An anonymous exec dictionary makes that class unresolvable even
            # after all expensive scan preprocessing has completed.
            wrapper = preamble + (
                f"__file__ = {str(script)!r}\n"
                f"exec(compile({source!r}, __file__, 'exec'), globals())\n"
            )
        (bundle / f"{name}.py").write_text(wrapper)
        hashes.append(hashlib.sha256(wrapper.encode()).hexdigest())
    return hashlib.sha256("".join(hashes).encode()).hexdigest()


def step_signature(command, job, patch_hash):
    payload = {"command": command, "job": job, "patch_sha256": patch_hash}
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()


def progress(bundle, job, status, step, completed):
    data = {
        "status": status,
        "step": step,
        "completed": completed,
        "input_sha256": job["input_sha256"],
        "revision": job["revision"],
        "updated_at": datetime.now(timezone.utc).isoformat(),
    }
    path = bundle / "progress.json"
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(data))
    temporary.replace(path)


def export_budget(bundle, job, budget, patch_hash):
    import torch

    paths = list((bundle / "outputs" / str(budget)).glob("seed*/sem/tot_bestcurves.pt"))
    if len(paths) != 1:
        raise RuntimeError(f"Expected one final WIR3D checkpoint for budget {budget}")
    tensor = torch.load(paths[0], weights_only=True, map_location="cpu")
    output = {
        "input_sha256": job["input_sha256"],
        "revision": job["revision"],
        "budget": budget,
        "control_points": tensor.tolist(),
        "provenance": {
            "seed": job["seed"],
            "checkpoint": str(paths[0].relative_to(bundle)),
            "keypoint_patch_sha256": patch_hash,
            "torch": torch.__version__,
            "cuda": torch.version.cuda,
            "gpu": torch.cuda.get_device_name(),
        },
    }
    path = bundle / f"curves_{budget}.json"
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(output, allow_nan=False))
    temporary.replace(path)


def export_keypoints(bundle, job):
    import torch

    points = torch.load(bundle / "keypoints.pt", weights_only=True, map_location="cpu")
    if points.shape != (job["keypoints"], 3) or not bool(torch.isfinite(points).all()):
        raise ValueError("Expected the configured number of finite automatic keypoints")
    result = {
        "input_sha256": job["input_sha256"],
        "revision": job["revision"],
        "frame": "wir3d",
        "points": points.tolist(),
        "checkpoint_sha256": hashlib.sha256((bundle / "keypoints.pt").read_bytes()).hexdigest(),
    }
    path = bundle / "keypoints.json"
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(result, allow_nan=False))
    temporary.replace(path)


def outputs_exist(bundle, name):
    render_names = {
        "reference-renders": "blender_030_360",
        "sketch-renders": "freestyle_030_360",
        "surface-renders": "surface_030_360",
    }
    if name in render_names:
        directory = bundle / "renders" / render_names[name]
        return (directory / "renderlist.json").is_file() and len(
            list(directory.glob("*.png"))
        ) >= 150
    if name == "automatic-keypoints":
        return (bundle / "keypoints.pt").is_file()
    if name == "surface-distance-field":
        return (bundle / "sdf.pt").is_file()
    if name.startswith("curves-"):
        return bool(
            list((bundle / "outputs" / name.split("-")[1]).glob("seed*/sem/tot_bestcurves.pt"))
        )
    return False


def recover_sketch_gif(bundle):
    """Recover only the known optional-GIF path error after validating every view."""
    from PIL import Image

    log = bundle / "logs/sketch-renders.log"
    failure = log.read_text()
    if (
        "FileNotFoundError" not in failure
        or "./data/model/renders/freestyle_030_360.gif" not in failure
    ):
        raise RuntimeError("Recovery requires the known post-render GIF export failure")
    directory = bundle / "renders/freestyle_030_360"
    cameras = json.loads((directory / "renderlist.json").read_text())
    if len(cameras["positions"]) != 150 or len(cameras["lookats"]) != 150:
        raise RuntimeError("Cannot recover incomplete camera metadata")
    images, hashes = [], {}
    for index in range(150):
        path = directory / f"{index:04}.png"
        with Image.open(path) as image:
            image.load()
            if image.size != (400, 400):
                raise RuntimeError(f"Unexpected render dimensions: {path}")
            images.append(image.copy())
        hashes[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
    destination = bundle / "renders/freestyle_030_360.gif"
    images[0].save(
        destination,
        format="GIF",
        append_images=images[1:],
        save_all=True,
        duration=30,
        loop=0,
        disposal=0,
    )
    record = {
        "reason": "All views completed; only optional GIF export used the wrong directory",
        "original_log_sha256": hashlib.sha256(log.read_bytes()).hexdigest(),
        "renderlist_sha256": hashlib.sha256(
            (directory / "renderlist.json").read_bytes()
        ).hexdigest(),
        "image_sha256": hashes,
        "gif_sha256": hashlib.sha256(destination.read_bytes()).hexdigest(),
    }
    (bundle / "logs/sketch-gif-recovery.json").write_text(json.dumps(record, indent=2))


def preflight(repository, blender, render_only=False):
    problems = []
    modules = (
        ("numpy", "yaml", "PIL", "imageio", "igl")
        if render_only
        else (
            "torch",
            "kaolin",
            "igl",
            "pydiffvg",
            "nvdiffrast",
            "clip",
            "pytorch3d",
            "robust_loss_pytorch",
            "importlib_resources",
        )
    )
    for module in modules:
        if importlib.util.find_spec(module) is None:
            problems.append(f"Missing Python module: {module}")
    if not render_only and importlib.util.find_spec("torch"):
        import torch

        if not torch.cuda.is_available():
            problems.append("CUDA is unavailable; the upstream pipeline requires a CUDA GPU")
        elif torch.cuda.get_device_properties(0).total_memory < 23 * 1024**3:
            problems.append("The documented WIR3D setup requires at least 24 GB GPU memory")
    if not (repository / "optimize_twostage_final.py").is_file():
        problems.append("WIR3D checkout is missing")
    if not blender.is_file():
        problems.append("Blender 3.6 executable is missing")
    else:
        version = subprocess.run([str(blender), "--version"], capture_output=True, text=True)
        if version.returncode or not version.stdout.startswith("Blender 3.6."):
            problems.append("This pipeline requires Blender 3.6.x")
    return problems


def commands(bundle, repo, blender, job):
    obj = str(bundle / "model.obj")
    python = sys.executable
    # Use argv lists rather than upstream preprocess.sh's unquoted command strings.
    common = [obj, "0", "30", "3", "0", "360", "50"]
    steps = [
        (
            "reference-renders",
            [
                str(blender),
                str(repo / "render/blend_files/soft_shadow_light.blend"),
                "--background",
                "--python-exit-code",
                "1",
                "--python",
                str(bundle / "render_reference.py"),
                "--",
                obj,
                "config.json",
                "blender_030_360",
                "0",
                "30",
                "3",
                "0",
                "360",
                "50",
                "--animate",
                "--resolution",
                "400",
                "400",
                "--background-color-hex=#FFFFFFFF",
                "--radius",
                "2.2",
                "--normalize",
                "--keep_material",
                "--device",
                "CPU",
                "--overwrite",
            ],
        ),
        (
            "sketch-renders",
            [
                str(blender),
                str(repo / "render/blend_files/freestyle.blend"),
                "--background",
                "--python-exit-code",
                "1",
                "--python",
                str(bundle / "render_sketch.py"),
                "--",
                *common,
                "--rendername",
                "freestyle_030_360",
                "--outputpath",
                str(bundle),
                "--radius",
                "2.2",
                "--fov",
                "60",
                "--normalize",
                "--resolution",
                "400",
                "--overwrite",
            ],
        ),
        (
            "surface-renders",
            [
                python,
                "render_data.py",
                *common,
                "--rendername",
                "surface_030_360",
                "--radius",
                "2.2",
                "--normalize",
                "--resolution",
                "400",
                "--overwrite",
            ],
        ),
        (
            "automatic-keypoints",
            [
                python,
                str(bundle / "generate_keypoints_patched.py"),
                obj,
                "blender_030_360",
                "--npoints",
                str(job["keypoints"]),
                "--model",
                "DINO",
                "--normalize",
                "--batchsize",
                "1",
                "--centertype",
                "latent",
                "--overlaydir",
                "surface_030_360",
                "--overwrite",
            ],
        ),
        (
            "surface-distance-field",
            [
                python,
                str(bundle / "seeded_sdf.py"),
                "--objdir",
                obj,
                "--nsteps",
                "20000",
                "--res",
                "256",
                "--batch_size",
                "1024",
                "--positional",
                "--overwrite",
            ],
        ),
    ]
    for budget in job["budgets"]:
        semantic = min(job["keypoints"], max(1, budget // 3))
        steps.append(
            (
                f"curves-{budget}",
                [
                    python,
                    "optimize_twostage_final.py",
                    obj,
                    "--sem_clip_model_name",
                    "RN50x16",
                    "--sem-spatial-keypoint-loss",
                    "keypoints.pt",
                    "--sem-spatial-weight-base",
                    "0",
                    "--init_keypoint_dir_sem",
                    "keypoints.pt",
                    "--ngeocurves",
                    str(budget - semantic),
                    "--nsemcurves",
                    "1",
                    "--sem_max_init_keypoints",
                    str(semantic),
                    "--geoniters",
                    str(job["geometry_iterations"]),
                    "--semniters",
                    str(job["semantic_iterations"]),
                    "--setseed",
                    "--geoseed",
                    str(job["seed"]),
                    "--semseed",
                    str(job["seed"]),
                    "--outputdir",
                    str(bundle / "outputs" / str(budget)),
                ],
            )
        )
    return steps


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", required=True, type=Path)
    parser.add_argument("--blender", required=True, type=Path)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--recover-sketch-gif", action="store_true")
    parser.add_argument("--recover-keypoint-overlay", action="store_true")
    parser.add_argument("--through", help="Stop after a named preprocessing/optimization step")
    args = parser.parse_args()
    if (args.recover_sketch_gif or args.recover_keypoint_overlay) and not args.resume:
        parser.error("Recovery flags require --resume")
    bundle = Path(__file__).resolve().parent
    repo, blender = args.repo.resolve(), args.blender.resolve()
    job = json.loads((bundle / "job.json").read_text())
    steps = commands(bundle, repo, blender, job)
    if args.through and args.through not in [name for name, _ in steps]:
        parser.error("--through must name one of: " + ", ".join(name for name, _ in steps))
    problems = preflight(repo, blender, args.through in {"reference-renders", "sketch-renders"})
    if repo.is_dir():
        revision = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=repo, text=True
        ).strip()
        if revision != job["revision"]:
            problems.append(f"Expected WIR3D revision {job['revision']}, got {revision}")
    if hashlib.sha256((bundle / "model.obj").read_bytes()).hexdigest() != job["input_sha256"]:
        problems.append("Input mesh hash differs from job.json")
    for filename, expected in job.get("resource_sha256", {}).items():
        path = bundle / filename
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            problems.append(f"Input resource hash differs: {filename}")
    print(json.dumps({"ready": not problems, "problems": problems}, indent=2), flush=True)
    if problems:
        progress(bundle, job, "failed", "; ".join(problems), [])
        return 2
    if args.check:
        return 0
    previous_source = (
        (bundle / "generate_keypoints_patched.py").read_text()
        if (args.recover_keypoint_overlay and (bundle / "generate_keypoints_patched.py").is_file())
        else ""
    )
    previous_hashes = wrapper_hashes(bundle)
    patch_hash = prepare_wrappers(repo, bundle, job["seed"])
    current_hashes = wrapper_hashes(bundle)
    if args.resume:
        migrate_render_cache(bundle, steps, job, previous_hashes, current_hashes)
    logs = bundle / "logs"
    logs.mkdir(exist_ok=True)
    completed = []
    for name, command in steps:
        done = logs / f"{name}.done"
        signature = step_signature(command, job, dependency_hash(name, current_hashes))
        progress(bundle, job, "running", name, completed)
        try:
            if (
                name == "automatic-keypoints"
                and args.recover_keypoint_overlay
                and not done.exists()
            ):
                recover_keypoint_overlay(bundle, repo, job, previous_source)
                done.write_text(signature)
            if name == "sketch-renders" and args.recover_sketch_gif and not done.exists():
                recover_sketch_gif(bundle)
                done.write_text(signature)
            if not (
                args.resume
                and done.exists()
                and done.read_text() == signature
                and outputs_exist(bundle, name)
            ):
                print(f"Running {name}", flush=True)
                with (logs / f"{name}.log").open("w") as output:
                    subprocess.run(
                        command, cwd=repo, stdout=output, stderr=subprocess.STDOUT, check=True
                    )
                if not outputs_exist(bundle, name):
                    raise RuntimeError(f"{name} exited without its expected output files")
                done.write_text(signature)
            if name == "automatic-keypoints":
                export_keypoints(bundle, job)
            if name.startswith("curves-"):
                export_budget(bundle, job, int(name.split("-")[1]), patch_hash)
        except Exception:
            progress(bundle, job, "failed", name, completed)
            raise
        completed.append(name)
        if name == args.through and name != steps[-1][0]:
            progress(bundle, job, "awaiting_compute", f"Completed through {name}", completed)
            return 0
    progress(bundle, job, "complete", "All curve budgets exported", completed)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
