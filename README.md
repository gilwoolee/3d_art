# Pen Art

Mesh → WirePrint-style framework + WIR3D art → one Flexiv arm with a 3D pen
and a rotating build plate.
WirePrint contours and zigzags are the chosen framework geometry; planning
searches construction motions and order, not alternative framework designs.
Objects fit within 15 cm per side. Pen stops, travel, and rejoining are allowed.
There is no fabrication-time target. Build the framework before the artistic
additions. Use **Caelan-style order and motion search** for both stages, with
WirePrint's order as an initial suggestion. The target mechanism combines seven
arm joints with a turntable indexed to 0, 90, 180, or 270 degrees. Turn only in
90-degree steps between strokes, with the pen off and arm withdrawn/parked;
hold the plate fixed during printing. Plan both stages before execution, starting
with simple reference fixtures before Palace.

See the [active plan](docs/PLAN.md). Geometry processing, structural screening,
robot collision checks, section-level timing/export/replay, WirePrint framework
generation, and backward build-order search are implemented. Complete reference
cube, prism, bottle and torus diagnostic builds pass modeled replay. The primary
solid-STL → WirePrint cube, generated bottle hybrid, and Palace builds remain
unverified. An edge-only reference cube does not satisfy the mesh-to-print task.
Both Palace searches are stopped with an independently verified access conflict
under the provisional pen geometry; see the [plan](docs/PLAN.md) for the required resolution.
The current bottle trial uses actual preceding deposition in indexed forward
planning; its two backward trials are stopped at a verified conflict in their
conservative framework envelope. The complete bottle build remains unverified.
Indexed turntable search, quarter-turn collision checks,
timing, replay, and viewer animation are integrated. A reference cube plan passes full
modeled replay; complete indexed validation of the other required objects remains
in the [active plan](docs/PLAN.md).
Pen, material, tool dimensions, physical robot variant, and turntable
parameters remain uncalibrated.

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

Open <http://127.0.0.1:8766>, or `http://127.0.0.1:8766/?run=<run id>` for one
run. The page shows three linked views side by side: the original textured
model, the wireframe strokes, and the robot's saved motion with one set of play
controls (30× by default). It states whether the plan it plays is the one the
separate verification passed. The earlier multi-stage workbench, with its
diagnostics, is at `/?workbench=1`. To check the page against a saved run in a
headless browser:

```bash
(cd web && PATH="$PWD/../tools/node/bin:$PATH" node scripts/check-simple.mjs <run id> out.png)
```

Existing runs and their saved partial/full candidate playback remain
inspectable in the workbench, with their original verification status. The viewer
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

Run a complete reference build (planning, timing, return home, and replay):

```bash
.venv/bin/python -m pen_art.planning.section_build \
  --fixture research/primitive_fixtures/cube --run runs/cube_sections_new --gpu
```

Verified examples: [cube playback](http://127.0.0.1:8766/?run=cube_sections_v1&stage=motion),
[prism playback](http://127.0.0.1:8766/?run=prism_sections_v1&stage=motion),
[bottle playback](http://127.0.0.1:8766/?run=bottle_sections_v1&stage=motion),
and [torus playback](http://127.0.0.1:8766/?run=torus_sections_v1&stage=motion).
Use port 8765 directly on Blondie. The focused regression tests are in
`tests/test_construction_sections.py`; `scripts/check_section_interleaving.py`
also runs an actual Flexiv `A → B → A` build and rejection checks.
After that fixture is prepared, `scripts/check_shared_source.py` runs a staged
build with a coincident artistic source and verifies it adds no duplicate extrusion.
`scripts/check_wireprint_frameworks.py` checks every proposed release state on
the four generated frameworks. `scripts/check_wireprint_topology.py` exercises
nested contours, split/merge events, and rejection of an unsupported island order.
These structural checks do not establish robot feasibility.

After a full build passes independent replay, check complete 10x playback against
the running viewer:

```bash
PATH="$PWD/tools/node/bin:$PATH" npm --prefix web run check:replay -- cube_sections_v1
```

The check rejects incomplete builds, confirms the viewer serves the verified plan,
and saves a screenshot and replay receipt under `research/blondie/`. It uses local
Chrome when available, or Playwright's installed Chromium. Set
`PEN_ART_BROWSER_EXECUTABLE` for another browser location and `PEN_ART_VIEWER_URL`
for another viewer address. If needed, install Chromium with
`npm --prefix web exec -- playwright install chromium`.

## New mesh and GPU work

For automatic geometric abstraction, use a fresh run:

```bash
.venv/bin/python -m pen_art.build --source path/to/model.stl \
  --source-up z --max-extent-m 0.15 --run runs/new_model
```

This produces geometry-stage artifacts, not a complete robot plan. WIR3D provides
the texture-aware alternative; its existing environment and results are preserved.
For the new WirePrint-to-robot path, use:

```bash
.venv/bin/python -m pen_art.planning.section_build \
  --mesh path/to/model.stl --source-up z --max-extent-m 0.15 \
  --run runs/new_wireprint --gpu
```

For a prepared WIR3D job, `--source-run runs/the_source --wir3d-budget 20`
imports its completed `wir3d_bundle/curves_20.json` and adds explicit attachments.
Whole curves below the build surface are excluded with recorded reasons; selection
does not impose a fabrication-time budget. Short connectors for disconnected
framework components are recorded as added material.
Both stages must pass together; a framework prefix is not a complete hybrid build.

[cuRobo](https://nvlabs.github.io/curobo/latest/) is installed at commit
`78fd485fa82d9b9a063fb4985e371814587e666a`. Its bundled Franka GPU check passed
16/16 collision-aware IK targets; see the [result](research/blondie/curobo_smoke.json).
Repeat with `.venv/bin/python scripts/check_curobo.py`. The Flexiv/pen adapter also
passes 46/46 fixture IK targets; see [its benchmark](research/blondie/flexiv_ik_benchmark.json)
and `scripts/benchmark_flexiv_ik.py`. cuRobo supplies batched IK and pen-off candidates;
it does not replace
structural checks, build-order search, or precise extrusion-path validation.
Every accepted path still uses the existing full robot/tool and material checks.
[Environment](research/blondie/environment.json) ·
[Installed packages](research/blondie/requirements-linux.txt) ·
[Migration checksum check](research/blondie/migration_check.json)

## Cached and learned drawing motions

The indexed planner supports two modes through the same collision, tracking,
timing, and independent replay checks. Phase 1 retrieves and adapts stored motions
and reuses applicable self/static clearance bounds. Phase 2 additionally generates
diffusion proposals when cached candidates do not produce an accepted motion.
The implementation and validation checklist is in [PLAN.md](docs/PLAN.md).

The viewer's **Motion cache** collection contains historical verified three-leg
continuous-zigzag cold, warm and width-variation demonstrations. In Robot motion,
scroll to the cache inventory and click a stroke to inspect its source curve and
jump to its drawing start. These small full-replay diagnostics are separate from
the precomputed real cube/bottle seeds, which have self/static-only validity.
Older prism and learned-fallback playbacks remain historical examples.

Use a persistent library with an indexed profile:

```bash
.venv/bin/python -m pen_art.planning.section_build \
  --mesh path/to/model.stl --run runs/new_cached_build \
  --profile research/planning_cell/cell_turntable.yaml \
  --motion-library runs/motion_library --gpu
```

### Parallel planning within one object

`--workers N` plans different whole actions of one supported order in `N`
processes, then replays the result with the complete robot model on the same
number of processes. Printing itself stays sequential; only computation is
concurrent. One worker runs the identical pipeline and is the comparison mode.

```bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python -m pen_art.planning.section_build \
  --cell-source-run runs/cube_nozzle27_phase1_v1_source --run runs/new_parallel_cube \
  --profile runs/cube_nozzle27_phase1_v1/profile.yaml --progression-only --workers 8
```

`--cell-source-run` binds a fresh run to an existing generated source without
regenerating its geometry. Workers are forked CPU processes, so `--gpu` is not
accepted with `--workers`. The motion library is frozen for the run: nothing is
written to it, and new path proofs go under the run's `parallel/` directory.
Re-running the same command resumes from recorded job results; any source or
input change starts over. `parallel_build.json` holds stage times, memory and
per-action outcomes; `parallel/progress.json` shows live state.

Each action first fixes its drawing path and clock, which determine its
deposits. Later actions are then checked against those predicted deposits
while earlier checks are still running. A replaced candidate invalidates every
result computed from it. Planning uses the pen-only collision scope; only the
final ordered full-robot replay certifies a build. See [PLAN.md](docs/PLAN.md)
for measured results, and `scripts/benchmark_parallel_build.py` to repeat them.

For the current whole-zigzag workflow, every cached DRAW trajectory holds one
full TCP orientation across its complete connected action. Admission certifies
orientation continuously; adaptation and replay recheck it. The default library
is `runs/motion_library_phase1_upright27_v1`; older variable-orientation libraries
are excluded from normal retrieval. They remain available as historical data.
The pen is upright; generated descending wires are at most 27° from horizontal.
Neither tool tilt, azimuth nor roll may change within an action.

The new source runs are `runs/cube_nozzle27_phase1_v1` and
`runs/bottle_nozzle27_phase1_v1`. Geometry feasibility must pass before launching
long whole-object searches; see [PLAN.md](docs/PLAN.md). Offline cache records still
need runtime checks against the actual growing sculpture, timing and connecting
motions. Available seeds do not establish a printable full build.

Add `--diffusion-checkpoint path/to/diffusion.pt --learning-device cuda` for
Phase 2. Cache-only mode does not import or require PyTorch. Learned checkpoints
are bound to the robot/tool and cell profile used for training; generated paths
remain proposals until they pass the normal checks.

Verification now persists exact source/orientation/static/clock certificates,
reuses conservative clearance bounds for changed paths, and queries material
against the required gap. Current material is reconstructed and checked every
time. Raw independent replay remains the default; use
`replay_cell(..., verification_raw=False, reuse_material_checkpoint=False)` to
measure warm invariant reuse with fresh material.
`DrawingTeacher` and `CellSignatureOracle` accept `verification_directory=...`
for the same persistent acceleration and `verification_raw=True` for comparison.
The learned proposal context binds the physical cell; timing/process assumptions
remain separately bound by the runtime validator.

The current reproducible multi-object pipeline is:

```bash
.venv/bin/python scripts/benchmark_verification_efficiency.py \
  --cases runs/verification_efficiency_cases_upright27_v3.json \
  --profile runs/bottle_nozzle27_phase1_v1/profile.yaml \
  --output runs/new_verification_benchmark --seconds 45 --training-steps 128
.venv/bin/python scripts/benchmark_prefix_verification.py \
  --output runs/new_prefix_verification_benchmark --sections 2 --maximum-seconds 180
```

Use new output directories. The first command checks cube/bottle/torus local
paths, generates a current-tool dataset, trains one shared diffusion checkpoint,
and exercises actual learned proposals with forced cache misses. The second
compares exact cube/bottle prefix replay while rebuilding material in every pass.
See [PLAN.md](docs/PLAN.md#verification-efficiency-shared-phase-1-and-phase-2-checklist)
for measured results, rejection controls and the detailed checklist. These
benchmarks do not establish complete sculpture or physical-print acceptance.

The offline data pipeline uses saved local motions from complete or partial
builds, generates nearby stroke/scene variations, and records verified paths,
collision witnesses, and unresolved attempts separately. Choose fresh outputs:

```bash
# PyTorch is optional for cache-only planning.
uv pip install -e '.[learning]'
.venv/bin/python scripts/generate_motion_dataset.py --output runs/new_motion_data
.venv/bin/python -m pen_art.learning curate runs/new_motion_data/attempts.jsonl \
  --output runs/new_motion_data/curated \
  --profile research/planning_cell/cell_turntable.yaml \
  --fit-validator pen_art.learning_generation:validate_fitted_episode
.venv/bin/python -m pen_art.learning train runs/new_motion_data/curated \
  --output runs/new_motion_data/diffusion.pt \
  --profile research/planning_cell/cell_turntable.yaml --device cuda
.venv/bin/python scripts/benchmark_motion_reuse.py \
  --attempts runs/new_motion_data/attempts.jsonl \
  --checkpoint runs/new_motion_data/diffusion.pt \
  --output runs/new_motion_benchmark
```

Dataset partitions keep each source object and its related variants together.
`curate` also accepts a motion-library directory directly; sampled seeds from a
saved clock still require the fit validator above. To solve recorded cache/model
misses with the teacher, use
`scripts/generate_motion_dataset.py --attempts runs/motion_library --output runs/new_feedback`.
Curate the resulting attempts
together with the original dataset to preserve its existing object partitions.
The benchmark compares teacher planning, cached adaptation, and cached adaptation
plus diffusion using an independent continuous drawing validator. Complete
construction, approach/withdrawal, and home return are checked separately with
`scripts/check_motion_runtime.py --output runs/new_motion_integration`.
Generated datasets, libraries, checkpoints, and reports remain under ignored
`runs/`; the scripts, schemas, and validators are source files in this repository.

## Geometry-first planning for new objects

`--geometry-first` finds a collision-free path through the whole object without
a clock, then times it once at the end. A timing failure is reported; it never
triggers re-planning. The flag implies merged wire capsules
(`--merge-deposits-um 20`), one path piece per straight member
(`--path-pieces members`) and clock knots only at piece ends and along the
ramps (`--retiming-grid knots`). The separate full-robot verification is
unchanged.

With member-length pieces the pen is checked against printed wire as a row of
65 spheres, at both ends of every piece and at poses no more than 10 mm apart
along it, each against the wire laid up to that pose. This is a sampled check,
not a continuous proof (decided 2026-10-13); the pen against the cell's fixed
obstacles and the contact moves keep their continuous checks.
`--build-option=--path-pieces=samples` restores a piece per path sample with
the continuous pen check.

```bash
.venv/bin/python scripts/benchmark_parallel_build.py \
  --source-run runs/the_source --profile path/to/profile.yaml \
  --output runs/new_build --workers 24 --build-option=--geometry-first \
  --verification-option=--accept-geometry-proofs
```

Use 24 workers on Blondie (24 cores; 32 gave nothing). With
`--accept-geometry-proofs`, verification accepts planning's proofs of speed
limits and source coverage for the exact saved clocks and still checks
the rest in full. By default verification also accepts planning's proof that the
pen clears the cell and the preceding wire, and planning's structural screens,
and checks the arm on every edge against itself, the cell and one volume
enclosing all wire; an arm not clear of those is a rejection. Pass
`--verification-option=--full-replay` and leave the first option out for a fully
independent verification.

Each zigzag is a vertical riser from the lower contour followed by one slanted
stroke back down to it, as in WirePrint (since 2026-10-13; before that both
strokes were slanted). The riser's top is the nearest point of the contour
above, so it is exactly vertical only where the outline does not move between
layers. The Palace and branching rows below use this wireframe; the other rows
predate it, and their wireframes would now come out different.

Long runs on Blondie go in tmux. Wireframe settings live in the source run's
`fabrication_config.json` (defaults in `configs/fabrication.yaml`):

| Setting under `wireprint` | Effect |
| --- | --- |
| `minimum_feature_m` | Opt-in. Rebuilds the mesh as a closed solid without parts thinner than this (`solidified.glb`), and ends rounded tips at the last printable contour. A simplification, recorded with its deviation. |
| `layer_height` | Requested layer height. Use 0.003 for solidified shapes; from the default 0.01 the refinement overshoots to sub-millimetre layers. |
| `heavier_strands_m` | Strand diameters tried in order when the frame fails the structural screen (default 1.5 mm, a tip not yet on hand). The choice is recorded as `strand_escalation` and is also the wire's collision size. |

The structural model treats the strand as 3D pen PLA at 2.5 GPa, a typical
published value that has not been measured.

Measured on 2026-10-13 with 24 workers and the vertical-riser wireframe. Every
build listed with times is `FULL_BUILD_VERIFIED` with no timing rejection.
"Total" is planning plus the separate verification, including setup; wireframe
generation (seconds to under a minute) is not included.

| Object | Actions | Collision-free path | Timing | Verification | Total | Peak memory |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Triangular prism | 21 | 3.6 s | 1.0 s | 2.8 s | 8.1 s | 3.3 GB |
| Bottle | 31 | 3.2 s | 1.4 s | 4.8 s | 10.5 s | 4.4 GB |
| Cube | 21 | 4.4 s | 1.4 s | 3.5 s | 10.2 s | 3.6 GB |
| Narrowed cube | 21 | 2.5 s | 1.2 s | 3.6 s | 8.1 s | 3.4 GB |
| Vase, outer shell only | 33 | 2.9 s | 2.6 s | 7.6 s | 14.5 s | 4.2 GB |
| Branching shape, 8 mm feature, 1 mm strand | 151 | 5.8 s | 3.1 s | 9.0 s | 20.9 s | 6.5 GB |
| Palace, 8 mm feature | 271 | 14.6 s | 5.9 s | 14.1 s | 42.6 s | 7.9 GB |
| Torus | 15 | not planned | | | | |

The torus does not plan with member-length pieces: it rests on a tangent
circle, so its inner and outer zigzags stand on two rings a fraction of a
millimetre apart, and the pen's nose comes 0.979 mm from a neighbouring
diagonal against the 1 mm margin on every candidate. It plans and verifies in
18.5 s with `--build-option=--path-pieces=samples`. Runs:
`runs/geomfirst_gf43_*_w24`. A violation reported by the pen's spheres is
confirmed against the exact pen shape at the same pose before an action is
rejected.

What the pipeline does to be fast, where the time goes and what
limits it:
[fast planning and verification](docs/2026_1013_fast_planning_and_verification.md). Details and history:
[geometry-first planning plan](docs/2026_1012_geometry_first_planning_plan.md).

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
