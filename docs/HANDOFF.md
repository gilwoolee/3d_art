# Handoff

Updated 2026-10-14, branch `geometry-first-planning` (Gilwoo Lee). Read this
first. The previous handoff, about the cube/bottle work before geometry-first
planning, is in [archive/2026_1012_handoff.md](archive/2026_1012_handoff.md).

## What this is

A planner for printing wire sculptures with a 3D pen held by one robot arm.
The model is a Flexiv Rizon4 arm with the pen tip 225 mm from the flange, and
a turntable that turns in 90 degree steps between strokes. Everything is
simulation: nothing has been printed and the pen, wire and arm placement are not
measured.

A mesh from `objs/` becomes a **frame**: a WirePrint wireframe of contour loops
joined by zigzags, about 10 m of wire, at most 15 cm per side. The planner finds
the order and the arm motion, and a separate check replays the whole robot
against everything built so far. A run that passes ends `FULL_BUILD_VERIFIED`.
On top of a planned frame we add about 2 m of **detail**: a few long pen strokes
over the surface, drawn after the frame.

## Moving to a new machine

Git carries the code, docs, configs, the four meshes in `objs/` (Git LFS) and
one cell profile template. Planned runs are not in git: the `runs/` folder here
is 108 GB. A 1.6 GB archive holds the runs worth keeping.

1. **Clone** and switch to the branch:

   ```bash
   git clone git@github.com:gilwoolee/3d_art.git && cd 3d_art
   git checkout geometry-first-planning
   ```

2. **Set up** (needs git, git-lfs, curl and [uv](https://docs.astral.sh/uv/);
   Linux x86-64):

   ```bash
   scripts/setup_machine.sh
   ```

   It pulls the LFS files, makes `.venv` (Python 3.12), checks out the Flexiv
   robot description at the pinned commit into `external/`, writes the robot cell
   to `research/planning_cell/`, installs Node 24 into `tools/node` and builds
   the viewer. The cell's URDF names this machine's absolute mesh paths, so it
   is generated on each machine and never copied.

3. **Copy and unpack the runs archive.** On the old machine it is
   `/home/gilwoo/Workspace/3d_art_runs.tar.zst`; remake it with
   `scripts/pack_runs.sh`. Unpack it in the clone's root:

   ```bash
   tar --zstd -xf 3d_art_runs.tar.zst
   ```

   It holds `runs/viewer_runs.json`, all profiles (`runs/geomfirst_profiles/`),
   the frame sources and detail sources behind the results below, the verified
   frame and detail runs, and the detail reports. Without it you can regenerate
   every frame from `objs/` (see **Running**), but you lose the planned results.

4. **Check:**

   ```bash
   .venv/bin/pytest -q                                   # 25 pass
   .venv/bin/ruff check src tests scripts
   env -u DISPLAY tools/node/bin/node --test web/tests/*.test.mjs   # 4 pass
   ```

   Then plan one frame to check the planner end to end (half a minute):
   `TAG=gf102 scripts/plan_frame.sh ghost runs/geomfirst_ghost_b10_source runs/geomfirst_profiles/ghost_b10.yaml`,
   and look for `FULL_BUILD_VERIFIED` in `runs/geomfirst_gf102_progress.txt`.

**Left behind on purpose:**
- **The motion library** (`runs/motion_library_phase1_upright27_v1`, 4.2 GB).
  Geometry-first planning takes no seeds from it: 0 of 18,400 requests on the
  Palace and 0 of 10,296 on the ghost detail run. `scripts/plan_frame.sh` and
  `scripts/plan_detail.py` point at an empty one. A prism planned with an empty
  library in 12.1 s, the same as with it.
- **Every other run:** older experiments, intermediate detail rounds, and the
  cube, bottle and Palace searches from before geometry-first.
- **The WIR3D GPU environment and its checkouts:** `.venv-wir3d`, and under
  `external/` CLIP, diffvg, nvdiffrast, pytorch3d and wir3d. Also cuRobo and
  Blender. The current work uses none of them. `scripts/setup_wir3d_blondie.sh`
  rebuilds the WIR3D environment for an RTX 5090.

**Machine habits that matter:**
- **Long runs go in tmux.** A session restart kills background and `nohup`
  jobs.
- **Use 24 workers.** That machine has 24 cores and 62 GB; 32 workers gave
  nothing more. On a smaller machine, set `WORKERS=` for `plan_frame.sh` or
  `--workers` for `plan_detail.py`.
- **Headless Chrome needs `env -u DISPLAY`** when you are logged in over SSH
  with X forwarding, or WebGL fails.
- **The viewer** runs as
  `.venv/bin/python -m pen_art.server --root "$PWD" --port 8765`, in a tmux
  session (`pen-art-viewer` on the old machine). Ctrl-C ends that session.
  From a laptop:
  `ssh -N -L 127.0.0.1:8766:127.0.0.1:8765 <host>`, then open
  <http://127.0.0.1:8766>. The run list is `runs/viewer_runs.json`.
  - Panel 0 shows the object's picture from `objs/`.
  - Panel 3 shows the wire as 1 mm tubes.
  - Unticking "Show statue" shows the wire alone.
- **Never `pkill -f` a pattern that can match your own shell.**

## Where things stand

### Frames (geometry-first, all `FULL_BUILD_VERIFIED`, 24 workers)

| Object | Source | Profile | Planned run | Wire | Strokes | Print time |
|---|---|---|---|---:|---:|---:|
| Liberty, cut at the feet | `geomfirst_libertystatue_c1_source` | `liberty_statue.yaml` | `geomfirst_gf75_liberty_statue_w24` | 4.9 m | 57 | 7.1 min |
| Ghost | `geomfirst_ghost_b10_source` | `ghost_b10.yaml` | `geomfirst_gf71_ghost_w24` | 9.5 m | 39 | 10.2 min |
| Pumpkin | `geomfirst_pumpkin_b10_source` | `pumpkin_b10.yaml` | `geomfirst_gf71_pumpkin_w24` | 9.6 m | 41 | 10.9 min |
| Palace (new frame) | `geomfirst_palace2_source` | `palace2.yaml` | `geomfirst_gf101_palace2_w24` | 9.9 m | 177 | 23.1 min |
| Palace (old frame) | `geomfirst_palace_b10_source` | `palace_cylinder126.yaml` | `geomfirst_gf71_palace_w24` | 7.0 m | 139 | 16.4 min |

Sources and runs are under `runs/`, profiles under `runs/geomfirst_profiles/`.
Print time is the plan's modeled duration. Its waits and speeds are settings,
not measurements.

**The new Palace frame** (commit 544e02d) comes from a generator change. Where
a loop is too short to fit 4 slanted strokes within the 27 degree limit, the
layers are made thinner there, down to 3 mm. Each loop now carries at least 4
pairs of rising and slanted strokes (`minimum_connectors: 4` in
`configs/fabrication.yaml`).

| Palace pillars | Old frame | New frame |
|---|---:|---:|
| Layer height | 7.6 mm | 4.5 mm |
| Rising strokes per loop | 2 | 5-7 |
| Sway under 0.1 N, free-standing | 3-7 mm | 0.13-2.2 mm |
| Sway under 0.1 N, finished | 0.6-1.2 mm | 0.05-0.13 mm |

The Liberty, ghost and pumpkin frames predate this change and have not been
regenerated.

### Detail (all `FULL_BUILD_VERIFIED`; budget 2 m and 15 extra minutes)

| Object | Final run | Detail | Strokes | Added time |
|---|---|---:|---:|---:|
| Liberty | `geomfirst_gf94_liberty_detail_r14_w24` | 1.99 m | 33 | 11.5 min |
| Ghost | `geomfirst_gf98_ghost_detail_r14_w24` | 1.98 m | 28 | 8.1 min |
| Pumpkin | `geomfirst_gf98_pumpkin_detail_r24_w24` | 1.99 m | 17 | 7.7 min |
| Palace (old frame) | `geomfirst_gf100_palace_detail_r24_w24` | 1.98 m | 33 | 8.3 min |

- **The budget.** The user lowered it from 5 m to 2 m on 2026-10-14. Detail
  is on top of the frame's own 10 m.
- **The most detail within 15 minutes, at the 5 m budget:** pumpkin 3.40 m,
  ghost 2.21 m, Liberty 2.15 m.
- **Measured cost:** about 12.5 s per stroke besides drawing, and drawing at
  about 14 mm/s.
- **Reports** for each object are in `runs/geomfirst_gfNN_NAME_detail_detail.json`.
- **The Palace detail is stale:** it sits on the old frame.

The full procedure, the rules and how the generator works are in the
[detail-tours skill](../.claude/skills/detail-tours/SKILL.md). The skill uses
the word "tour" for one detail stroke: lines on one turntable face joined by
short connecting runs, starting on a frame node.

## Next work

1. **The agent detail pipeline:** [plan](2026_1014_agent_detail_pipeline_plan.md),
   phases 0-7, not started, and the user has not yet said go.
   - **Why.** Today detail follows surface creases chosen by geometry. On the
     ghost it lands on the hat brim, while the eyes, mouth and blush are
     painted texture.
   - **What the plan does.** It keeps the tool layer that makes lines drawable
     and lets an agent choose features from rendered views of the textured
     mesh.
   - **Speed-ups.** A reduced work mesh, a faster inside test and 24-worker
     line checks.
   - **Decided by the user:** drop crease detection; the pipeline is driven by
     an agent and reusable; reduce the mesh in the prepare stage.
   - **Still open:** the camera set, colour clustering on the Palace's
     photo-like texture, and whether WIR3D candidates are worth it.
2. **Palace detail on the new frame.** Offered, not answered. Run
   `scripts/detail_tours.py` on `geomfirst_palace2_source` with `palace2.yaml`
   and frame run `geomfirst_gf101_palace2_w24`. Reduce the mesh first: the
   Palace has 2.85 M faces, and generation took an hour on the old frame.
3. **Regenerate the Liberty, ghost and pumpkin frames** with the new generator,
   to check for sparse parts. Offered, not answered.
4. **Small offers not taken up:**
   - colour panel 3 by frame (green) and detail (orange);
   - rename "tour" to "detail stroke".

**Untested physical assumptions:**
- fusing detail by laying it over frame wire;
- the pen's nose shape and its 0.3 mm clearance from wire;
- overhangs of up to 8 mm;
- the drawing speed.

PLA stiffness is a published 2.5 GPa, not measured.

## Running

From the repo root, in tmux for anything long:

```bash
# Frame from a mesh: source and profile, then plan (half a minute to a few minutes)
.venv/bin/python scripts/frame_source.py ghost objs/cute_ghost/cute_ghost.glb --up y
TAG=gf102 scripts/plan_frame.sh ghost runs/geomfirst_ghost_source runs/geomfirst_profiles/ghost.yaml

# Detail: generate strokes (minutes; an hour on the Palace), then plan, drop and trim
PYTHONPATH=src .venv/bin/python scripts/detail_tours.py FRAME_SOURCE runs/geomfirst_NAME_detail_source \
  --profile PROFILE --check-with-planner --margin 0.0008 --detail 12 --minutes 40 --collect 2.5 \
  --crease-threshold 0.12 0.08 0.05 0.03 --bridge 0.03 --fine-region X0 X1 Y0 Y1 Z0 Z1
PYTHONPATH=src .venv/bin/python -u scripts/plan_detail.py runs/geomfirst_NAME_detail_source PROFILE \
  runs/geomfirst_gfNN_NAME_detail --frame-run FRAME_RUN --detail 2 --minutes 15
```

`frame_source.py` options: `--layer` (m between loops), `--feature` (thinner
parts are dropped) and `--coil-width` (narrower parts are wound as a coil).

Settings behind the current frames (each source's `fabrication_config.json`
records them):

| Frame | Mesh | Options |
|---|---|---|
| Liberty | `objs/liberty/liberty_statue_only.glb` | `--layer 0.0055 --feature 0.004 --coil-width 0.01` |
| Ghost | `objs/cute_ghost/cute_ghost.glb` | `--layer 0.01` |
| Pumpkin | `objs/pumpkin/pumpkin.glb` | `--layer 0.01` |
| Palace | `objs/palace/Meshy_AI_Palace_Reflections_1011014238_texture.glb` | `--layer 0.008 --coil-width 0.01` |

Generator settings and defaults are in `configs/fabrication.yaml` under
`wireprint`; the README's table explains each one. Add a finished run to
`runs/viewer_runs.json` to list it in the viewer.

## Rules from the user

- **No `Co-Authored-By:` trailers** in commits or pull requests.
- **Fewer than 30 tests in all.** Today there are 25 Python and 4 browser
  tests. Add assertions to existing test functions instead of adding tests.
- **24 workers.**
- **Per object:** frame wire at most 10 m; detail about 2 m and at most 15
  extra minutes.
- **The pen must not twist the wire while depositing.** The planner holds one
  pen tilt per stroke (0-60 degrees toward the robot), and one turntable face
  per stroke.
- **Report plainly and faithfully:** say what failed, what was skipped and what
  was not measured.

## Code map

| Area | Where |
|---|---|
| Frame generator (contours, zigzags, coils, layer heights) | `src/pen_art/abstraction/wireprint.py`, `solidify.py` |
| Mesh to frame source, wire budget | `src/pen_art/planning/section_build.py` (`prepare_mesh_within_wire`, `prepare_framework`), `scripts/frame_source.py` |
| Structural check (sway, support, fused contacts) | `src/pen_art/structure.py` |
| Parallel geometry-first planning | `src/pen_art/planning/parallel_build.py`, `cell_planner.py`, `cell_motion.py`, `indexed_build.py`, `transit.py` |
| Pen and wire collision | `src/pen_art/planning/pen_spheres.py`, `axis_corridor.py`, `collision.py`, `tool.py` |
| Full-robot verification | `src/pen_art/planning/cell_replay.py` |
| Benchmark driver for plan and verification | `scripts/benchmark_parallel_build.py`, `scripts/plan_frame.sh` |
| Detail harness | `scripts/detail_tours.py`, `scripts/plan_detail.py`, `.claude/skills/detail-tours/SKILL.md` |
| Viewer | `src/pen_art/server.py`, `web/src/simple.ts`, `web/src/viewer.ts` |
| Setup and transfer | `scripts/setup_machine.sh`, `scripts/pack_runs.sh` |

Measured planning speed and where the time goes:
[fast planning and verification](2026_1013_fast_planning_and_verification.md).
Geometry-first design and history:
[geometry-first planning plan](2026_1012_geometry_first_planning_plan.md) and
[stroke chaining plan](2026_1013_stroke_chaining_plan.md). `docs/PLAN.md`
describes the earlier cube and bottle work and is not the active task list.
