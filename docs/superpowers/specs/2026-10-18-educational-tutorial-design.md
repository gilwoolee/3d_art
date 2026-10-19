# Pen Art as a tutorial: design

Date: 2026-10-18. Status: for the user's approval.

## Intent

Pen Art plans wire sculptures drawn in the air by a 3D pen on one robot arm,
with a turntable. The research work is finished. The repository becomes an
**educational example** that walks a learner through the full path from a
mesh to a verified print plan, in simulation.

Learners are expected to change the setup: the build volume (for example
from a 15 cm to an 8 cm cube), the pen, the robot's placement. Everything
must follow from those choices. In particular the **motion library**, which
makes planning produce fast prints, is built by the learner for their own
setup as a first-class step, not shipped pre-built.

The learner should come away understanding the full path. The library is one
step on it, not the subject of the tutorial.

## The learner's path

The README is a tutorial in this order. Each step has: what it does and why,
one command, the expected output for the ghost, and what to try changing.

0. **Describe the cell.** One file holds what a learner changes: build
   volume, pen geometry, robot placement, turntable.
1. **Build the motion library** for that cell.
2. **Mesh to frame:** loops and zigzags within the wire budget.
3. **Plan and verify the frame.** Fast because strokes start from library
   motions.
4. **Add detail** strokes over the surface.
5. **View** the result in the browser viewer.

The ghost (`objs/cute_ghost/`) is the worked example in every step and the
test of the library: it is **never** one of the library's practice objects,
so the tutorial shows real reuse.

A short "Concepts" section explains each component in plain words: frame,
stroke, turntable face, motion library, verification.

## Goals

- A learner on the target machine runs steps 0-5 from a fresh clone by
  following the README, with no other files or downloads.
- Changing the build volume or the pen and rebuilding the library gives fast
  prints again for the new setup.
- The repository holds only what the tutorial uses: single arm, one cell
  file, one command per step, five tests.

## Non-goals

- Two arms. The repository becomes single-arm only.
- Benchmarking, memory sampling, comparison modes, and learning (diffusion)
  proposals.
- WIR3D, cuRobo and other paths the current pipeline does not use.
- Physical printing. Everything stays simulation.

## Anti-goals (a failure even if it runs)

- Planning silently uses a library built for a different cell, and is slow
  or wrong without saying so.
- The ghost's own motions end up in the library, so the example only
  replays itself.
- Numbers in the README that a fresh clone does not reproduce.
- Claims of physical printability, or hidden machine-specific paths.

## Constraints (from the user)

- Single arm. Frame wire at most 10 m per object; detail about 2 m and at
  most 15 extra minutes of print time.
- The pen never twists wire while depositing: one tilt and one turntable
  face per stroke.
- Target machine: a 24-core workstation with an RTX 5090; 8×H200 is
  available if needed. Planning uses 24 workers. Long runs go in tmux.
- Keep only the 5 most critical tests.
- MIT license; meshes in `objs/` included.
- Report plainly: what passed, what failed, what was not measured.

## Components and decisions

### Cell file (step 0)

Today the settings live in 7 configs, the only ones a first run reads:
`configs/cell/one_flexiv_straight.yaml` (robot placement, used by setup),
`configs/profiles/frame_template.yaml` (cell profile template),
`configs/fabrication.yaml` (frame generator, structure, process),
`configs/dual_arm.yaml`, `configs/transit.yaml`, `configs/timing.yaml`,
`configs/timing_continuous_bounded.yaml`.

Decided: merge them into `configs/cell.yaml` (what a learner changes: build
volume, pen, robot placement, turntable) and `configs/frame.yaml` (frame
generator settings). Anything a learner would not change stays out of
`cell.yaml`.

### Motion library (step 1)

How it works today: the library is a folder of verified robot motions for
whole strokes. For each new stroke the planner takes the stored strokes
nearest in absolute position and shape, adapts their joint paths as seeds,
and checks everything again. Records are bound to a hash of the cell
(robot, pen, collision model) and verified against the hash of their bytes.
The 24-worker planner reads the library frozen; it never writes to it.

The current library (`runs/motion_library_phase1_upright27_v1`, 2.2 GB, 80
records, not in git) was built from cube and bottle builds in earlier work.
`src/pen_art/planning/motion_cache_precompute.py` holds the code that
precomputed those records from source builds; the script that drove it was
removed.

Measured on the ghost frame (24 workers): with that library, 41 strokes, all
from the library, **10.9 min** of print time; with no library, **14.2 min**.
Planning took about 30 s either way. A sequential plan meant to write
motions into a library failed on its first stroke (`FORWARD_TIMING`) and
wrote nothing, so the library cannot be grown by planning objects as-is.

Decided:
- One command builds a library from `configs/cell.yaml`, with two sizes
  (decided by the user; both are options):
  - **quick:** a small set of short practice strokes, aiming at 15-30
    minutes on the 24-core machine, for a first run of the tutorial;
  - **full:** whole practice objects plus wide synthetic coverage, a few
    hours on 24 cores, or faster on the 8×H200 machine; for the fastest
    prints.
  The tutorial says which to choose and states the measured time, disk use
  and ghost print time of each.
- Practice material is **a set of various objects plus synthetic strokes**:
  - objects scaled to the cell's build volume, turned into frames by the
    normal frame generator and planned whole;
  - synthetic zigzags and loop pieces that fill what the objects leave out:
    low to high, centre to edge of the volume, all four turntable faces,
    slants up to the 27 degree limit.
- The ghost is not in the practice set; it is the test.
- The library records which cell it was built for. Planning with a library
  from another cell refuses and says to rebuild.
- Success: the ghost frame planned with a freshly built **full** library
  for the default 15 cm cell prints in about the time it does with today's
  library (10.9 min; 14.2 min with none). The **quick** library must print
  faster than no library; its gain is measured and stated. The same checks
  run at 8 cm against an 8 cm ghost planned without a library. The tutorial
  states the measured build time of each size on 24 cores.

### Frame, plan, detail, view (steps 2-5)

These work today and stay as they are in behaviour:
- `scripts/frame_source.py` (mesh to frame within the wire budget);
- `scripts/plan_frame.sh` → `scripts/benchmark_parallel_build.py` →
  `pen_art.planning.section_build` (24 workers) then
  `pen_art.planning.cell_replay` (full-robot verification);
- `scripts/detail_tours.py` and `scripts/plan_detail.py`;
- `pen_art.server` and `web/` (the viewer).

Decided: one command per step, replacing the current mix of scripts, and
plain "plan and verify" without benchmark reports.

### Code to remove

A full first run imports 55 of the 105 modules in `src/pen_art`. The other
50, about 13,000 of 37,000 lines, are earlier two-arm, learning, WIR3D,
comparison and benchmark code. Decided: remove two-arm support and its
naming (`dual_arm.yaml`, `offline_dual_arm_plan.json`, `arm_1`), and
everything the tutorial does not use, including the viewer's old
"workbench" page. `motion_cache_precompute.py` is the exception if the
library builder reuses it.

### Tests

Decided: keep the 5 most critical tests, and update the rule in `AGENTS.md`
(today "fewer than 30").

## The builder's calls (mine, open to the user's correction)

- **Command form:** one entry point, `pen-art <step>`, with the steps `cell`,
  `library`, `frame`, `plan`, `detail`, `view`.
- **Practice objects:** a cylinder, a cone, a box and a vase-like shape,
  generated in code (no new mesh files).
- **The 5 tests:** frame generator (slice heights), continuous collision
  check, timing sweeps exactly the checked path, verification reports the
  serial rejection, and the library refusing a different cell. Browser tests
  are dropped.
- **Run names:** plain names chosen by the user (for example `runs/ghost`),
  not `geomfirst_gfNN_*_w24`.
- **Pass mark for the full library:** at 15 cm, the ghost frame prints
  within 0.5 min of 10.9 min. At 8 cm, it saves at least 20% of print time
  against the same 8 cm ghost planned without a library. Repeated plans in
  testing gave identical results (14.23 min cold, twice), so one run per
  check; if results vary, use the median of three.
- **README numbers** are measured from a fresh clone and written as "about";
  the planner is not made deterministic as part of this work.
- **What `cell.yaml` holds:** build volume, pen geometry (length, nose and
  tip), robot placement, turntable. Process settings (speeds, cooling,
  strand diameter) and timing go in `frame.yaml` under `process`. The pen
  stays upright with the 27 degree slant limit; these are not tutorial
  settings. The library's identity covers all of `cell.yaml` and the process
  settings, so changing any of them requires a rebuild.
- **Downloads:** `scripts/setup_machine.sh` may still fetch its usual inputs
  (Git LFS meshes, the Flexiv description, Node). "No other files or
  downloads" means no pre-built runs or library.
- **This document** lives in `docs/superpowers/specs/` while the work goes
  on and is removed when the tutorial is done, to keep the repository
  clean.

### Changing the build volume

Decided by the user: frame settings do **not** scale with the build volume.
Layer height, zigzag spacing and base grid spacing are physical sizes set by
the wire and the pen. A smaller volume gives a smaller object with fewer
layers and less wire: the minimal print. The 8 cm check uses the same
`frame.yaml` as 15 cm.

## Left to the builder

- How synthetic strokes are sampled and how many; how many objects and at
  what sizes; stopping when the ghost test passes.
- Whether the library builder reuses `motion_cache_precompute.py`, the
  normal planner on the practice objects, or both.
- Internal module layout after the removal, and file names inside `runs/`.
- How the full library uses the H200s.

## Order of work

1. Single arm only; remove unused code; keep the 5 tests green.
2. Merge the configs into `cell.yaml` and `frame.yaml`.
3. Prototype the library builder early and measure it against the ghost
   at 15 cm.
4. One command per step.
5. Write the tutorial README with measured outputs.
6. From a fresh clone: run the tutorial at 15 cm and at 8 cm.
