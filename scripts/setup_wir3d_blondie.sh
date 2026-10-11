#!/usr/bin/env bash
# Isolated RTX 5090 environment. Run from ~/Workspace/3d_art on Blondie.
set -euo pipefail
task_root=$(pwd)
task_python="$task_root/.venv-wir3d/bin/python"
task_uv="$HOME/.local/bin/uv"
export CUDA_HOME=/usr/local/cuda-12.8
export PATH="$task_root/.venv-wir3d/bin:$CUDA_HOME/bin:$PATH"
export TORCH_CUDA_ARCH_LIST=12.0
export MAX_JOBS=4
export CMAKE_BUILD_PARALLEL_LEVEL=4
mkdir -p external tools logs

"$task_python" - <<'PY'
import json, torch
x = torch.randn(512, 512, device='cuda')
y = x @ x
torch.cuda.synchronize()
info = {'torch': torch.__version__, 'cuda': torch.version.cuda,
        'gpu': torch.cuda.get_device_name(),
        'memory_bytes': torch.cuda.get_device_properties(0).total_memory,
        'matmul_finite': bool(y.isfinite().all())}
print(json.dumps(info, indent=2))
with open('logs/gpu_preflight.json', 'w') as f:
    json.dump(info, f, indent=2)
PY

"$task_uv" pip install --python "$task_python" \
  'numpy==1.26.4' 'setuptools<81' wheel ninja 'cmake<4' \
  'scipy==1.15.3' 'libigl==2.5.1' 'trimesh==4.6.6' \
  matplotlib scikit-learn scikit-image networkx pillow tqdm dill \
  imageio imageio-ffmpeg 'moviepy==1.0.3' 'opencv-python-headless==4.11.0.86' \
  'mesh-to-sdf==0.0.15' 'pyglet<2' \
  potpourri3d svgpathtools svgwrite cssutils torch-dct \
  ftfy regex iopath lpips einops timm pybind11 gdown 'importlib_resources==6.5.2'
"$task_uv" pip install --python "$task_python" kaolin==0.18.0 \
  -f https://nvidia-kaolin.s3.us-east-2.amazonaws.com/torch-2.8.0_cu128.html

checkout() {
  local url=$1 dest=$2 revision=$3
  if [ ! -d "$dest/.git" ]; then
    git init "$dest"
    git -C "$dest" remote add origin "$url"
    git -C "$dest" fetch --depth 1 origin "$revision"
  fi
  git -C "$dest" checkout --detach "$revision"
}
checkout https://github.com/threedle/wir3d.git external/wir3d 35171dfca522272eb113882acfaaf6de746a78ce
checkout https://github.com/facebookresearch/pytorch3d.git external/pytorch3d 33824be3cbc87a7dd1db0f6a9a9de9ac81b2d0ba
checkout https://github.com/NVlabs/nvdiffrast.git external/nvdiffrast 253ac4fcea7de5f396371124af597e6cc957bfae
checkout https://github.com/BachiLi/diffvg.git external/diffvg 85802a71fbcc72d79cb75716eb4da4392fd09532
checkout https://github.com/openai/CLIP.git external/CLIP d05afc436d78f1c48dc0dbf8e5980a9d471f35f6
checkout https://github.com/jonbarron/robust_loss_pytorch.git external/robust_loss_pytorch 0c25c59ddbd0a14e5d963ae4f3847f8f3974fdc4
git -C external/diffvg submodule update --init --recursive
for package in CLIP robust_loss_pytorch pytorch3d nvdiffrast; do
  "$task_uv" pip install --python "$task_python" --no-build-isolation "./external/$package"
done
# The upstream README uses setup.py; its unrelated Poetry metadata omits pydiffvg.
# CUDA 12.8's Thrust headers require C++17. Keep the pinned source unchanged.
"$task_python" - <<'PY'
import pathlib, shutil
source = pathlib.Path('external/diffvg')
target = pathlib.Path('external/diffvg-compat')
shutil.copytree(source, target, dirs_exist_ok=True,
                ignore=shutil.ignore_patterns('.git', 'build', 'dist', '*.egg-info'))
cmake = target / 'CMakeLists.txt'
text = cmake.read_text().replace('CMAKE_CUDA_STANDARD 11', 'CMAKE_CUDA_STANDARD 17')
text = text.replace('-std=c++11', '-std=c++17 -gencode=arch=compute_120,code=sm_120')
cmake.write_text(text)
PY
(
  cd external/diffvg-compat
  "$task_python" setup.py bdist_wheel
  "$task_uv" pip install --python "$task_python" dist/*.whl
)

if [ ! -x tools/blender-3.6.23-linux-x64/blender ]; then
  (
    cd tools
    curl -fL --retry 3 -O https://download.blender.org/release/Blender3.6/blender-3.6.23-linux-x64.tar.xz
    curl -fL --retry 3 -O https://download.blender.org/release/Blender3.6/blender-3.6.23.sha256
    grep 'blender-3.6.23-linux-x64.tar.xz$' blender-3.6.23.sha256 | sha256sum -c -
    tar -xf blender-3.6.23-linux-x64.tar.xz
  )
fi
"$task_uv" pip freeze --python "$task_python" > logs/wir3d-environment.txt
tools/blender-3.6.23-linux-x64/blender --version | head -3
"$task_python" -c 'import torch, kaolin, igl, pydiffvg, nvdiffrast, clip, pytorch3d; print("WIR3D dependencies imported")'

# Pyrender's legacy PyOpenGL pin lacks OSMesaCreateContextAttribs. OSMesa uses
# the installed Mesa CPU renderer for SDF scans when NVIDIA EGL cannot initialize.
"$task_uv" pip install --python "$task_python" --no-deps 'PyOpenGL==3.1.7'
