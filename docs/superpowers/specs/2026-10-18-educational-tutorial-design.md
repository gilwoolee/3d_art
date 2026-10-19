# Pen Art as a tutorial: design

Date: 2026-10-18. Status: approved by the user on 2026-10-18.

## Intent

Pen Art plans wire sculptures drawn in the air by a 3D pen on one robot arm
(7 joints) around a fixed build plate. The research work is finished. The repository becomes an
**educational example** that walks a learner through the full path from a
mesh to a verified print plan, in simulation.

Learners are expected to change the setup: the build volume (for example
from a 15 cm to an 8 cm cube), the pen, the robot's placement. Everything
follows from those choices; nothing is pre-built or downloaded besides what
setup fetches.

## The learner's path

The README is a tutorial in this order. Each step has: what it does and why,
one command, the expected output for the ghost, and what to try changing.

0. **Describe the cell** (`configs/cell.yaml`): a printer build plate lying
   on the table, the build volume, the pen, and where the plate is relative
   to the robot. One command writes the robot cell from it.
1. **Mesh to frame** (`configs/frame.yaml`): the generator slices the mesh
   into horizontal loops joined by zigzags (a vertical riser, then a slant
   back down of at most 27 degrees), within the 10 m wire budget, with a
   base grid on the bed, coils for thin parts, and a structural check that
   the frame stands. It then makes the frame drawable in as few pen-down
   trails as possible (see "Chaining and lift-offs") and reports that
   minimum.
2. **Add detail** (decided by the user to come before planning): generate
   surface strokes that rest on the frame wire, about 2 m. Generation only;
   no planning yet. Detail is drawn after the frame. Detail is purely
   geometric (creases of the mesh's shape), so it misses features that are
   only painted in the texture, like the ghost's eyes and mouth. The
   tutorial points out this step as the place to add an AI agent that picks
   more artistic detail, for example from rendered views of the textured
   mesh (decided by the user: mentioned, not built).
3. **Plan the paths, geometry first**, on 24 workers, for frame and detail
   together. The build order comes from the frame, then the detail. Each
   stroke gets one pen orientation (tilt and lean direction) and a
   collision-free arm path, checked against the cell and all wire laid so
   far. No timing yet. A detail stroke that cannot be drawn is dropped and
   reported; a frame stroke that cannot be drawn stops the plan and is named.
   Wherever a stroke starts where the previous one ended, the planner first
   tries to **chain** it: carry on drawing from the previous stroke's final
   pose with the pen down. It lifts off only when chaining fails, and reports
   every lift-off with its reason.
4. **Time it once:** speeds, cooling holds, moves between strokes. This gives
   the print time. Detail is then trimmed to the budget: at most about 2 m
   and 15 minutes beyond the frame's own share of the timed plan, dropping
   the strokes that give the least detail per minute; the trimmed plan is
   timed again. A timing failure is reported, never searched again.
5. **Verify:** a separate check replays the full robot against everything
   built so far, once for the whole build. A plan that passes is
   `FULL_BUILD_VERIFIED`. Each stage reports the number of lift-offs next to
   the print time.
6. **View** the result in the browser viewer.

Steps 3, 4 and 5 are three separate commands, `plan-geometry`,
`plan-timing` and `verify-full-robot` (decided by the user), so the learner
runs and inspects each stage on its own. Each reads the previous stage's
output from the run folder and writes its own. Each stage runs exactly once:
what today's `plan_detail.py` hides in rounds of replanning becomes visible
as "plan-geometry dropped these strokes, plan-timing trimmed these". A
learner may skip step 2 to plan the frame alone.

**Input meshes.** The tutorial says plainly that the pipeline generally
assumes a mesh made by a text- or image-to-3D generator such as Meshy
(meshy.io), like the ghost: one object, a textured `.glb`, at whatever scale
and up axis the generator chose (step 1 scales it to the build volume and
takes the up axis as an option). Other meshes may work but are not the
target.

The ghost (`objs/cute_ghost/`) is the worked example in every step. A short
"Concepts" section explains frame, stroke, pen orientation, geometry-first
planning, timing and verification in plain words.

## Chaining and lift-offs (decided by the user)

Strokes are chained as much as possible, and lift-offs are minimized. A
lift-off means stopping the feed, withdrawing, travelling and approaching
again to rejoin the wire. The arm runs open loop, so there is no way to be
sure the pen comes back to the same pose; and every rejoin is a joint where
new wire must fuse onto wire that may have sagged. Chaining keeps the wire
continuous and avoids both.

Today the frame generator already orders strokes so that each starts where
the previous one ended, but the planner only chains when it takes a stroke
from the motion library; its own sampler never chains. Measured on the ghost
frame (41 strokes): 15 travels with the library, 44 without it (every
stroke lifted off), 10.9 and 14.2 min. Removing the library must therefore
come with chaining in the planner's own sampler.

**Fewest trails (decided by the user).** The frame is a graph: wire pieces
are edges, the points where they meet are nodes. By Euler's rule, the
fewest pen-down trails that draw every piece once is half the number of
nodes where an odd number of pieces meet. The ghost frame (1,421 pieces,
1,091 nodes) has 18 such nodes: 16 where base grid lines end on the bed
outline, plus the bottom start and top end of the walls. Its minimum is 9
trails: the walls above the bed are already one trail, the base grid adds
8. The frame step adds connector wire to make those nodes even: it pairs
odd nodes by a minimum-weight perfect matching and adds one connector per
pair, so the whole frame can be one trail from bottom to top. Connectors
are new wire, never wire printed over existing wire: on the bed they run
just inside the outline (about one wire width), and they count against the
10 m budget. Above the bed a connector is added only if it is supported and
passes the structural check; otherwise the node stays odd and is reported.
The planner's order then follows that trail. Every planning stage reports
actual lift-offs next to the minimum, for example "1 possible, 3 actual",
each with its reason. The tutorial has a short concept section on drawing
without lifting the pen.

## Goals

- A learner on the target machine runs the whole tutorial from a fresh clone
  by following the README.
- Changing the build volume or the pen and rerunning the steps works: the
  ghost at 8 cm also plans to `FULL_BUILD_VERIFIED`.
- The ghost frame's minimum is 1 trail after connectors are added (9
  before). It plans with at most as many lift-offs as it had with the
  motion library (15 travels on 41 strokes), and as close to the minimum
  as the robot allows. The README shows lift-offs against the minimum for
  every plan.
- The repository holds only what the tutorial uses: single arm,
  geometry-first planning, two config files, one command per step, five
  tests.

## Non-goals

- Two arms.
- A turntable. The current code turns the object in 90 degree steps between
  strokes; that goes, along with its timing, collision checks and test.
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
- The pen never twists wire while depositing: one pen orientation (tilt
  and lean direction) per stroke.
- No turntable (decided by the user): the build plate is fixed and the arm
  reaches around the object.
- The plate is an ordinary 3D-printer build plate on the table, like a Bambu
  Lab plate (decided by the user). The object is built directly on it.
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

- **Configs:** merge the 7 into `configs/cell.yaml` (plate size and
  position, build volume, pen geometry, robot placement) and `configs/frame.yaml` (frame
  generator, plus process settings such as speeds, cooling and strand
  diameter under `process`). The pen stays upright with the 27 degree slant
  limit; these are not tutorial settings.
- **Code removal:** two-arm support and its naming (`dual_arm.yaml`,
  `offline_dual_arm_plan.json`, `arm_1`), the motion library, every planning
  mode except geometry first, and everything else the tutorial does not
  use.
- **Tests:** keep 5; update the rule in `AGENTS.md` (today "fewer than 30").

## The builder's calls (mine, open to the user's correction)

- **The plate:** default 256 × 256 mm, the Bambu Lab X1/P1/A1 size (as far
  as I know; the A1 mini's is 180 × 180 mm), lying flat on the table at a
  position the learner measures from the robot base. Today's 19 cm
  platform 20 cm high and the 2 cm riser under the object go. Moving the
  object from 22 cm up down to table height changes what the arm must reach,
  so order-of-work step 2 checks the ghost there and picks the default plate
  position from what plans.
- **Command form:** one entry point, `pen-art <step>`, with the steps
  `cell`, `frame`, `detail`, `plan-geometry`, `plan-timing`,
  `verify-full-robot` (these three names decided by the user), `view`.
- **Dropping is a single pass.** `plan-geometry` drops every detail stroke
  it cannot draw in one pass, rather than replanning in rounds. If dropping
  a stroke can make a later one drawable, that later one may still be
  dropped; acceptable for a tutorial.
- **The 5 tests:** frame generator (slice heights), continuous collision
  check, timing sweeps exactly the checked path, verification reports a
  rejection, and the pen is checked against wire already laid. Browser
  tests are dropped.
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
- How `plan-timing` trims detail to the budget (re-time after removing
  strokes, or remove and re-time in steps) and how much of
  `scripts/plan_detail.py` it reuses.
- How `cell.yaml` values map onto the existing profile and collision model.

## Order of work

1. Single arm and geometry first only: remove two-arm code, the motion
   library, other planning modes and unused code; keep the 5 tests green;
   check the ghost still plans to `FULL_BUILD_VERIFIED` (about 14.2 min with
   today's turntable, every stroke lifting off).
2. Chaining in the planner's own sampler: try the chained continuation
   first for every stroke that starts where the previous one ended; report
   lift-offs with reasons. Check the ghost frame is `FULL_BUILD_VERIFIED`
   with at most 15 lift-offs.
3. Fewest trails: add connector wire by minimum-weight matching of odd
   nodes, order the frame along the resulting trail, and report the minimum
   next to the actual lift-offs. Check the ghost's minimum is 1 and its
   lift-offs fall below step 2's.
4. Remove the turntable, the platform and the riser; the object stands on a
   plate on the table. Frame strokes stay upright; detail strokes lean the
   pen toward the side of the object they draw on, and the arm reaches
   around. A stroke that fails the path-accuracy check from the robot's side
   retries with a finer path or another posture (today the planner turned
   the turntable instead). Check the ghost frame and detail plan to
   `FULL_BUILD_VERIFIED` with no more lift-offs than step 3.
5. Merge the configs into `cell.yaml` and `frame.yaml`.
6. One command per step, in the order cell, frame, detail, plan-geometry,
   plan-timing, verify-full-robot, view; detail generation only, with
   dropping in plan-geometry and trimming in plan-timing.
7. Write the tutorial README with measured outputs.
8. From a fresh clone: run the tutorial at 15 cm and at 8 cm.
