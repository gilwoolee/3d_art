#!/bin/bash
# Set up a fresh clone on a new Linux x86-64 machine. Run from anywhere:
#   scripts/setup_machine.sh
# Needs git, git-lfs, curl and uv (https://docs.astral.sh/uv/). Safe to re-run;
# each step is skipped when already done. Steps:
#   1. objs/ meshes and pictures from Git LFS
#   2. .venv (Python 3.12) with the planner and dev tools
#   3. external/flexiv_description at the pinned commit, then the robot cell in
#      runs/cell from configs/cell.yaml (its URDF holds this machine's
#      absolute paths, so it is made here, never copied)
#   4. Node 24 in tools/node, the viewer's packages and build
set -euo pipefail
cd "$(dirname "$0")/.."
root=$PWD

for tool in git git-lfs curl uv; do
  command -v "$tool" > /dev/null || { echo "missing: $tool" >&2; exit 1; }
done

echo "== 1. Git LFS files (objs/)"
git lfs install --local
git lfs pull

echo "== 2. Python environment"
[ -x .venv/bin/python ] || uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python -e '.[dev]' -c requirements.lock

echo "== 3. Robot description and cell"
flexiv=92ef7865d76585e6e08d291bdfe652d32f7740f4
if [ ! -d external/flexiv_description ]; then
  git clone https://github.com/flexivrobotics/flexiv_description.git external/flexiv_description
fi
git -C external/flexiv_description checkout --quiet --detach "$flexiv"
if [ ! -f runs/cell/robot.urdf ] || ! grep -q "$root/external" runs/cell/robot.urdf; then
  .venv/bin/pen-art cell
fi

echo "== 4. Node and the viewer"
node_version=v24.21.0
node_name=node-$node_version-linux-x64
if [ ! -x tools/node/bin/node ]; then
  mkdir -p tools
  (
    cd tools
    curl -fsSLO "https://nodejs.org/dist/$node_version/$node_name.tar.xz"
    curl -fsSL "https://nodejs.org/dist/$node_version/SHASUMS256.txt" | grep " $node_name.tar.xz\$" \
      | sha256sum -c -
    tar -xf "$node_name.tar.xz"
    ln -sfn "$node_name" node
  )
fi
export PATH="$root/tools/node/bin:$PATH"
(cd web && npm ci && npm run build)

mkdir -p runs
echo "Done. Next: .venv/bin/pen-art frame objs/cute_ghost/cute_ghost.glb --name ghost"
echo "Then follow the README: .venv/bin/pen-art frame objs/cute_ghost/cute_ghost.glb --name ghost"
