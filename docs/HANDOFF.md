# Handoff — continue on Blondie

Updated 2026-10-11. Read this first, then [PLAN.md](PLAN.md). The longer design
review and old experiment logs are in `docs/archive/`.

## Start here

```bash
ssh blondie
cd /home/gilwoo/Workspace/3d_art
git pull --ff-only
git status --short
```

Private repo: <https://github.com/gilwoolee/3d_art>, branch `main`.
The initial source baseline is `55efefd`; subsequent handoff commits are on top.
Use this existing workspace: its ignored data and environments are already installed.
A fresh Git clone alone will not contain the full asset collection.

## Target and decisions

- Generalizable mesh → artistic **wireframe only** → automatic build order → robot motion.
- One Flexiv 7-DoF arm, one rigidly mounted 3D pen, TCP at the pen tip.
- Maximum 15 cm per side. Palace stays uniformly scaled to 15 × 15 × 9 cm.
- Stops, withdrawal, pen-off travel, and rejoining are normal. No manual connections.
- Find the first feasible offline plan. No fabrication-time target, two-arm work,
  broad mounting search, or repeated grinding of one blocked motion.
- Keep the existing 1 mm clearance and 0.3 mm TCP tracking requirements.
- The Rizon4 model, right-angle bracket, pen dimensions, 10° initial lean, and
  elevated platform are provisional. Physical arm variant and exact pen details
  still need confirmation/calibration. Everything is simulation-only.

## What works and what does not

Mesh import, curve extraction/selection, graphs, structural screening, Flexiv
kinematics/collision models, local stroke restarts, timing, and partial-motion
playback exist. **Integrated Caelan-style order search and a verified complete
robot build do not yet exist.** No physical sculpture has been validated.

The clean fixtures in `research/primitive_fixtures/` are the next planning inputs:

| Fixture | Dimensions (mm) | Members / reference curves |
| --- | --- | --- |
| Cube | 80 × 80 × 80 | 12 / 12 |
| Triangular prism | 80 × 60 × 80 | 9 / 9 |
| Bottle | 50 × 50 × 120 | 28 / 8 |
| Torus | 120 × 120 × 30 | 32 / 8 |

Each contains `model_mm.stl`, `stroke_graph.json`, `pen_strokes.json`, and
`build_order/preflight.json`. STL coordinates are mm; graph coordinates are m.
Their reference curves deliberately isolate planning from automatic abstraction.
All pass the **final-frame** screen under uncalibrated cooled-strand, rigid-weld,
clamped-ground assumptions. Intermediate construction states still need checks.

Historical runs are preserved for diagnosis, not mandatory starting prefixes:

| Run | Saved state at handoff |
| --- | --- |
| `runs/palace_15cm_one_arm` | `MOTION_BLOCKED`; 15/64 strokes complete; `draw-0029` stopped at samples 163→164 |
| `runs/general_torus_one_arm_base_first` | `MOTION_BLOCKED`; 11/19 strokes complete; `draw-0006` stopped at 106→107 |

The Palace final graph has 724 members and an ill-conditioned structural solve.
The older Torus graph has 91 members and fails the provisional structural model;
its predicted 29.2 mm displacement exceeds the linear model's validity range.
Neither result establishes physical impossibility. The clean Torus fixture is a
different, explicit reference graph; do not confuse their results.

## Next implementation

1. Start with the cube, then prism, then clean torus/bottle, then Palace.
2. Freeze a graph of joinable sections with exact source intervals and real
   junctions. Preflight the final structure. Support additions create an explicit
   graph revision and restart order search; never insert them silently mid-plan.
3. Implement Caelan-style backward search over `(material subset P, robot q)`.
   For a possible last section, check its predecessor structure and supported
   start; lazily generate its forward extrusion and pen-off travel to the planned
   suffix. Recurse, retaining alternative directions/modes and backtracking.
   Include initial home-to-first travel and final return home. Use forward
   stiffness reasoning as a priority, not a fixed mandatory order.
4. Add the Flexiv/pen cuRobo adapter for batched IK and pen-off candidates.
   Keep exact extrusion tracking, growing-material collision checks, and
   structural checks in the existing validation path. GPU IK success is not a
   construction or motion certificate.
5. Replay the resulting plan forward: exact source coverage, supported joins,
   release states, extrusion, travel, and home return. Then export timing and
   the complete viewer playback. Use a few meaningful tests, not a large new suite.

Useful code (all relative to `src/pen_art/`):

| Responsibility | Existing modules |
| --- | --- |
| Goal preflight / mechanics | `planning/build_order_preflight.py`, `structure.py` |
| Exact curves, intervals, starts | `curves.py`, `strokes.py`, `construction.py` |
| Search/state patterns | `planning/sequence.py`, `planning/coupled.py` |
| Robot, material, motion | `planning/cell_planner.py`, `collision.py`, `scene.py`, `rrt.py` under `planning/` |
| Rejoin/support semantics | `planning/stroke_restarts.py`, `planning/endpoint_support.py` |
| Coverage and playback export | `planning/source_coverage.py`, `planning/full_build_export.py` |

`construction_preview` is primarily a topology preview. `SingleArmBuild` follows
a fixed selection. `coupled.plan_sequence` backtracks but expects complete drawing
paths. Reuse their components; none is the new coupled regression planner.
See [the algorithm review](archive/2026_1011_caelan_build_order_plan.md) for details.

## Installed environment and commands

- `.venv/bin/python`: Linux Python 3.12 planner plus cuRobo/PyTorch.
- `.venv-wir3d/bin/python`: separate working WIR3D environment; preserve it.
- RTX 5090, 32 GB; repaired driver `595.99.02`. CUDA computation passes.
- cuRobo commit `78fd485fa82d9b9a063fb4985e371814587e666a`, current v2 API.
  Its bundled **Franka** IK check passed 16/16 targets; this is not a Flexiv benchmark.
- Node: `export PATH="$PWD/tools/node/bin:$PATH"` before npm commands.
- Fresh host-local robot/tool cell: `research/planning_cell/cell.yaml`.
  Do not resume Mac checkpoints by rewriting paths/hashes or weakening validation.

```bash
# Read-only preflight on an existing fixture:
.venv/bin/python -m pen_art.planning.build_order_preflight \
  --run research/primitive_fixtures/cube --output /tmp/cube-preflight.json

# Only if regeneration is needed; separate output preserves tracked fixtures:
.venv/bin/python scripts/generate_primitive_fixtures.py --output research/primitive_fixtures_linux
.venv/bin/python scripts/prepare_single_arm_cell.py

# Optional GPU installation check, not the new build-order planner:
.venv/bin/python scripts/check_curobo.py
```

The existing `pen_art.build` command produces automatic geometry and can call the
legacy motion pipeline. It is not an entry point for the pending Caelan adaptation.

## Viewer and assets

Only the read-only viewer was running at handoff, in tmux `pen-art-viewer`.
No new order or motion search is running. On Blondie open <http://127.0.0.1:8765>.
From the Mac, use the existing <http://127.0.0.1:8766> tunnel or recreate it:

```bash
ssh -N -o ExitOnForwardFailure=yes -o ControlMaster=no -o ControlPath=none \
  -L 127.0.0.1:8766:127.0.0.1:8765 blondie
```

If the remote server is stopped, from the project directory:

```bash
tmux new-session -d -s pen-art-viewer -c "$PWD" \
  "$PWD/.venv/bin/python -m pen_art.server --root $PWD --port 8765"
```

The motion page includes 10× playback. Partial/rejected histories are inspectable;
they are not complete builds. The server cannot send robot commands.

Assets are under the same project root: the four original Palace GLB/STL files,
all `runs/` and `research/`, `external/` dependencies/assets, and built `web/dist/`.
Blondie also retains its 7.3 GB `wir3d_bundle/`, Linux environments, and `tools/`
(including Blender and Node). They are intentionally outside Git.
The Mac environment and node_modules are not useful Linux assets; use installed
Linux dependencies instead. Remote-only data is preserved.

[Asset inventory](../research/blondie/handoff_assets.json) and
[verification report](../research/blondie/handoff_verification.json) record paths,
sizes, and SHA-256 checks. Environment details are in
[`research/blondie/`](../research/blondie/). Continue development on Blondie and
commit source changes through the private repo.
