# Pen Art

Mesh → artistic wireframe → supported build order → one Flexiv arm with a 3D pen.
Objects fit within 15 cm per side. Pen stops, travel, and rejoining are allowed.
There is no fabrication-time target. The current algorithm work is **Caelan-style
build-order search**, starting with simple reference fixtures before Palace.

See the [active plan](docs/PLAN.md). Geometry processing, structural screening,
robot collision checks, timing, and partial-build playback exist. Integrated
build-order search and a verified complete build are still outstanding. Pen,
material, tool dimensions, and physical robot variant remain uncalibrated.

Continuing on Blondie: read the [handoff](docs/HANDOFF.md) first.

## Work on Blondie

The private source repository is [gilwoolee/3d_art](https://github.com/gilwoolee/3d_art).
The primary compute workspace is `/home/gilwoo/Workspace/3d_art` on `blondie`.
Synchronize source through Git and run new jobs on Blondie in separate output
directories. Copied historical runs remain evidence; their old absolute paths
and validation hashes are not portable resume checkpoints.

```bash
ssh blondie
cd ~/Workspace/3d_art
git pull --ff-only
.venv/bin/python scripts/generate_primitive_fixtures.py
.venv/bin/python scripts/prepare_single_arm_cell.py
```

`.venv` contains the Linux planner. `.venv-wir3d` preserves the separate WIR3D
Python/CUDA environment. Node is installed locally under `tools/node`; to rebuild
the viewer on Blondie, run `export PATH="$PWD/tools/node/bin:$PATH"`, then
`npm --prefix web ci && npm --prefix web run build`.
`research/planning_cell/cell.yaml` uses fresh, host-local
Flexiv assets with the existing single-arm placement and pen geometry. Regenerate
these assets on a different host instead of editing historical URDFs/manifests.

The read-only webviewer runs in Blondie's `pen-art-viewer` tmux session. From the
Mac, forward it to port **8766**:

```bash
ssh -N -o ExitOnForwardFailure=yes -o ControlMaster=no -o ControlPath=none \
  -L 127.0.0.1:8766:127.0.0.1:8765 blondie
```

Open <http://127.0.0.1:8766>. Existing runs and their saved partial/full candidate
playback remain inspectable, with their original verification status. The viewer
has no robot execution route. To start it manually on Blondie:

```bash
.venv/bin/python -m pen_art.server --root "$PWD" --port 8765
```

## Small planning fixtures

[Preview](research/primitive_fixtures/preview.svg) ·
[Manifest](research/primitive_fixtures/manifest.json)

| Fixture | Dimensions (mm) | Material members / reference curves |
| --- | --- | --- |
| Cube | 80 × 80 × 80 | 12 / 12 |
| Triangular prism | 80 × 60 × 80 | 9 / 9 |
| Bottle | 50 × 50 × 120 | 28 / 8 |
| Torus | 120 × 120 × 30 | 32 / 8 |

Each `research/primitive_fixtures/<name>/` contains a watertight `model_mm.stl`,
reference curves, stroke graph, pen strokes, and final-frame preflight. STL
coordinates are millimeters; graph coordinates are meters, with Z up.
These deliberately simple reference wireframes isolate build-order planning
from automatic mesh abstraction. Their structural screens assume cooled 1 mm
strands, rigid welded joints, and clamped substrate contacts. They do not certify
an intermediate build state or physical printability.

## New mesh and GPU work

For automatic geometric abstraction, use a fresh run:

```bash
.venv/bin/python -m pen_art.build --source path/to/model.stl \
  --source-up z --max-extent-m 0.15 --run runs/new_model
```

This produces geometry-stage artifacts, not a complete robot plan. WIR3D provides
the texture-aware alternative; its existing environment and results are preserved.
[cuRobo](https://nvlabs.github.io/curobo/latest/) is installed at commit
`78fd485fa82d9b9a063fb4985e371814587e666a`. Its bundled Franka GPU check passed
16/16 collision-aware IK targets; see the [result](research/blondie/curobo_smoke.json).
Repeat with `.venv/bin/python scripts/check_curobo.py`. This is an installation
check, not a Flexiv benchmark. Use cuRobo for batched IK and pen-off candidates; it
does not replace
structural checks, build-order search, or precise extrusion-path validation.
The Flexiv/pen adapter and integration into order search are pending.
[Environment](research/blondie/environment.json) ·
[Installed packages](research/blondie/requirements-linux.txt) ·
[Migration checksum check](research/blondie/migration_check.json)

## Repository contents

Git tracks source, tests, configs, documentation, reference fixtures, and research
evidence. Generated `runs/`, original Palace meshes, external checkouts, installed
environments, and tool binaries remain on disk on Blondie; a Git clone does not
download those artifacts. Nothing from the existing workspace is deleted by pulling.

For a fresh machine, install Python dependencies as below, then fetch the pinned
Flexiv description before running `scripts/prepare_single_arm_cell.py`:

```bash
git clone https://github.com/flexivrobotics/flexiv_description.git external/flexiv_description
git -C external/flexiv_description checkout --detach 92ef7865d76585e6e08d291bdfe652d32f7740f4
```

## Local fallback and history

```bash
uv venv --python 3.13
uv pip install -e '.[dev]'
npm --prefix web ci
npm --prefix web run build
.venv/bin/python -m pen_art.server
```

Detailed commands and earlier experiments are retained in the
[archived workbench history](docs/archive/2026_1011_workbench_history.md).
The [archived plan](docs/archive/2026_1010_mesh_to_robot_pen_plan.md) and
[Caelan integration review](docs/archive/2026_1011_caelan_build_order_plan.md)
are references, not active task lists.
