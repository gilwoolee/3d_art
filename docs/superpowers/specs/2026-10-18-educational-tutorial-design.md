# Pen Art as a tutorial: design

Date: 2026-10-18. Status: for the user's approval.

## Intent

Pen Art plans wire sculptures drawn in the air by a 3D pen on one robot arm,
with a turntable. The research work is finished. The repository becomes an
**educational example** that walks a learner through the full path from a
mesh to a verified print plan, in simulation.

Learners are expected to change the setup: the build volume (for example
from a 15 cm to an 8 cm cube), the pen, the robot's placement. Everything
follows from those choices; nothing is pre-built or downloaded besides what
setup fetches.

## The learner's path

The README is a tutorial in this order. Each step has: what it does and why,
one command, the expected output for the ghost, and what to try changing.

0. **Describe the cell** (`configs/cell.yaml`): build volume, pen shape,
   robot placement, turntable. One command writes the robot cell from it.
1. **Mesh to frame** (`configs/frame.yaml`): the generator slices the mesh
   into horizontal loops joined by zigzags (a vertical riser, then a slant
   back down of at most 27 degrees), within the 10 m wire budget, with a
   base grid on the bed, coils for thin parts, and a structural check that
   the frame stands.
2. **Plan the paths, geometry first**, on 24 workers. The build order comes
   from the frame. Each stroke gets one turntable face and one pen tilt and
   a collision-free arm path, checked against the cell and all wire laid so
   far. No timing yet. A stroke that cannot be drawn stops the plan and is
   named.
3. **Time it once:** speeds, cooling holds, turntable turns between strokes.
   This gives the print time. A timing failure is reported, never searched
   again.
4. **Verify:** a separate check replays the full robot against everything
   built so far. A plan that passes is `FULL_BUILD_VERIFIED`.
5. **Add detail:** generate surface strokes, plan them the same way (steps
   2-4), drop strokes that cannot be drawn, trim to about 2 m and 15 extra
   minutes.
6. **View** the result in the browser viewer.

Steps 2, 3 and 4 are three separate commands, `plan-geometry`,
`plan-timing` and `verify-full-robot` (decided by the user), so the learner
runs and inspects each stage on its own. Each reads the previous stage's
output from the run folder and writes its own.

The ghost (`objs/cute_ghost/`) is the worked example in every step. A short
"Concepts" section explains frame, stroke, turntable face, geometry-first
planning, timing and verification in plain words.

## Goals

- A learner on the target machine runs the whole tutorial from a fresh clone
  by following the README.
- Changing the build volume or the pen and rerunning the steps works: the
  ghost at 8 cm also plans to `FULL_BUILD_VERIFIED`.
- The repository holds only what the tutorial uses: single arm,
  geometry-first planning, two config files, one command per step, five
  tests.

## Non-goals

- Two arms.
- Any planning mode other than geometry first: the sequential timed planner,
  its checkpoints and resume, and benchmark comparison modes.
- The motion library: storing, retrieving and adapting earlier motions, and
  building libraries. (Measured on the ghost frame: planning takes about
  20 s with or without the old library; it only made prints faster, 10.9
  vs 14.2 min, because its motions came from the dropped timed planner.)
- Learning (diffusion) proposals, WIR3D, cuRobo, benchmarking and memory
  sampling, the viewer's old "workbench" page.
- Physical printing. Everything stays simulation.

## Anti-goals (a failure even if it runs)

- Numbers in the README that a fresh clone does not reproduce.
- Claims of physical printability, or machine-specific paths.
- A learner-facing setting that has no effect, or an effect the README does
  not explain.

## Constraints (from the user)

- Single arm, geometry-first planning only.
- Frame wire at most 10 m per object; detail about 2 m and at most 15 extra
  minutes of print time.
- The pen never twists wire while depositing: one tilt and one turntable
  face per stroke.
- Frame settings (layer height, zigzag spacing, base grid spacing) are
  physical sizes set by the wire and the pen. They do **not** scale with the
  build volume: a smaller volume gives a smaller object with fewer layers
  and less wire, the minimal print.
- Target machine: a 24-core workstation with an RTX 5090 (the planner uses
  the CPU only). Planning uses 24 workers. Long runs go in tmux.
- Keep only the 5 most critical tests.
- MIT license; meshes in `objs/` included.
- Report plainly: what passed, what failed, what was not measured.

## Current state (for the builder)

Measured on the ghost on 2026-10-18 (24 workers): frame 9.9 m of wire, 41
strokes, planning about 20 s, `FULL_BUILD_VERIFIED`, **14.2 min** of print
time without a library. Detail with the default options added 1.33 m and
6.6 min, about 12 minutes of work; those runs used the old library for the
frame strokes, so the totals without it are to be measured.

Today's pieces:
- `scripts/frame_source.py`: mesh to frame source and cell profile.
- `scripts/plan_frame.sh` → `scripts/benchmark_parallel_build.py` →
  `pen_art.planning.section_build --geometry-first --workers 24`, then
  `pen_art.planning.cell_replay` (verification).
- `scripts/detail_tours.py`, `scripts/plan_detail.py`.
- `scripts/setup_machine.sh` and `scripts/prepare_single_arm_cell.py`
  (setup and robot cell).
- `pen_art.server` and `web/` (viewer).

A full first run reads 7 configs: `configs/cell/one_flexiv_straight.yaml`,
`configs/profiles/frame_template.yaml`, `configs/fabrication.yaml`,
`configs/dual_arm.yaml`, `configs/transit.yaml`, `configs/timing.yaml`,
`configs/timing_continuous_bounded.yaml`. It imports 55 of the 105 modules
in `src/pen_art`; the other 50 (about 13,000 of 37,000 lines) never run.
The geometry-first planner currently takes frame strokes from the motion
library when one is present and otherwise falls back to its own sampler
(`cold_fallback`); that sampler becomes the only path.

## Decisions

- **Configs:** merge the 7 into `configs/cell.yaml` (build volume, pen
  geometry, robot placement, turntable) and `configs/frame.yaml` (frame
  generator, plus process settings such as speeds, cooling and strand
  diameter under `process`). The pen stays upright with the 27 degree slant
  limit; these are not tutorial settings.
- **Code removal:** two-arm support and its naming (`dual_arm.yaml`,
  `offline_dual_arm_plan.json`, `arm_1`), the motion library, every planning
  mode except geometry first, and everything else the tutorial does not
  use.
- **Tests:** keep 5; update the rule in `AGENTS.md` (today "fewer than 30").

## The builder's calls (mine, open to the user's correction)

- **Command form:** one entry point, `pen-art <step>`, with the steps
  `cell`, `frame`, `plan-geometry`, `plan-timing`, `verify-full-robot`
  (these three names decided by the user), `detail`, `view`.
- **Detail** stays one command: its rounds of planning and dropping strokes
  run the same three stages internally.
- **The 5 tests:** frame generator (slice heights), continuous collision
  check, timing sweeps exactly the checked path, verification reports a
  rejection, and a quarter turn of the turntable rejects a collision between
  clear endpoints. Browser tests are dropped.
- **README numbers** are measured from a fresh clone and written as
  "about"; repeated plans gave identical results in testing.
- **Downloads:** `scripts/setup_machine.sh` may still fetch its usual inputs
  (Git LFS meshes, the Flexiv description, Node).
- **Run names:** plain names chosen by the user (for example `runs/ghost`),
  not `geomfirst_gfNN_*_w24`.
- **This document** lives in `docs/superpowers/specs/` while the work goes
  on and is removed when the tutorial is done, to keep the repository
  clean.

## Left to the builder

- Internal module layout after the removal, and file names inside `runs/`.
- How to split today's single planning process (paths and timing together
  in `section_build`) so that timing runs as its own command on a saved
  untimed plan; verification (`cell_replay`) is already separate.
- Whether `detail` reuses the three stage commands directly.
- How `cell.yaml` values map onto the existing profile and collision model.

## Order of work

1. Single arm and geometry first only: remove two-arm code, the motion
   library, other planning modes and unused code; keep the 5 tests green;
   check the ghost still plans to `FULL_BUILD_VERIFIED` at about 14.2 min.
2. Merge the configs into `cell.yaml` and `frame.yaml`.
3. One command per step, with geometry, timing and verification split into
   `plan-geometry`, `plan-timing` and `verify-full-robot`.
4. Write the tutorial README with measured outputs.
5. From a fresh clone: run the tutorial at 15 cm and at 8 cm.
