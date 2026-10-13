# Precomputed Clearance Table Design

**Date:** 2026-10-11
**Status:** Design (conceptual; nothing here is implemented or measured)

**Implementation direction:** [PLAN.md](PLAN.md#trajectory-reuse-two-implementation-phases)
now defines two phases: first a reusable trajectory library with self/static
clearance reuse and local adaptation, then dataset generation and diffusion
proposals through the same validator. The dense clearance table below remains
background exploration, not a prerequisite for either phase. Its performance
estimates are historical hypotheses, not current measurements or acceptance.

## Purpose and Scope

This document summarizes the precomputed-roadmap collision technique from
Murray et al. and describes how the pen-printing planner could use the same
idea to avoid repeating collision work that does not change between strokes.

It covers robot self-collision, robot against fixed surroundings, and robot
against the growing printed structure. It assumes the print volume is confined
to a fixed 15 cm x 15 cm x 15 cm cube. It is a design direction, not an
implementation plan.

## Background: Motion Planning on a Chip

The source technique is "Robot Motion Planning on a Chip" (Murray, Floyd-Jones,
Qi, Sorin, Konidaris; RSS 2016) and its hardware companion "The
Microarchitecture of a Real-Time Robot Motion Planning Accelerator" (MICRO
2016). Realtime Robotics grew out of this work.

The method splits collision checking into a robot half done offline and an
obstacle half done at runtime.

| Stage | What happens |
|---|---|
| Offline | Build one roadmap for the robot in an empty world. For every edge, compute the set of workspace voxels the robot sweeps through. Compile each set into a small logic circuit. |
| Runtime | Perception produces occupied voxels. Each voxel is broadcast to every edge circuit at once. An edge whose swept set contains the voxel is marked blocked. |
| Search | Blocked edges are removed and a path is found through the rest of the roadmap. |

The papers report a plan for a 6-DOF arm in under 1 ms, about three orders of
magnitude faster than the software planners they compared against.

Why it is fast:

- No forward kinematics or geometry runs at runtime. That work is folded into
  the precomputed swept sets.
- A collision check becomes set membership on voxel address bits.
- All edges are tested in parallel, so cost scales with the number of occupied
  voxels and not with the number of edges.
- Self-collision does not depend on the scene, so it can be resolved offline
  when the roadmap is built.

Its limits are the other side of the same design: motions are restricted to the
fixed roadmap, and accuracy is restricted to the voxel resolution.

A GPU or FPGA is not required for the idea. The runtime step is a bitmask
intersection, which a CPU can do for a roadmap of modest size.

## Source Alignment

These facts about the current planner constrain the design.

- `planning/collision.py` is the authority for clearance. `check_bounded_path`
  certifies a continuous joint path by comparing each pair's distance against a
  bound on how far the bodies can move, built from per-joint motion radii. This
  is the Schwarzer-Saha-Latombe adaptive scheme.
- The same file already removes shared-ancestor joints from self-pair bounds.
- `planning/curobo_adapter.py` states the project rule: no sphere approximation
  is admitted as evidence of collision clearance.
- Strokes need a 1 mm margin, and the pen tip works at contact with the
  structure.
- `research/blondie/relative_collision_bounds.json` shows bounded-path checks
  taking about 2.5 s of an 8.4 s staged run, roughly 30%. Cutting evaluations by
  23% changed that time by only 2%.
- `research/blondie/palace_collision_profile.json` shows roughly 3 ms per
  evaluation with about 21,000 material capsules.

The time has not been split between self pairs, static surfaces and material.

## Design Principles

1. **The exact checker stays the authority.** Precomputed data may only prove
   that something is clear. Anything it cannot prove goes to the exact checker.
2. **Approximations must be one-sided.** A voxel or bubble test may report a
   false "possibly blocked", never a false "clear".
3. **Precompute only what cannot change.** Robot against itself and against
   fixed surroundings is constant. The printed structure is not.
4. **Cover continuous motion, not just samples.** A discrete set of postures
   must come with a bound that extends each result to its neighbourhood.

## Proposed Architecture

There are two precomputed parts and one growing part.

### 1. Clearance table (robot against itself and fixed surroundings)

Sample the print cube in task space: a grid of pen-tip positions, a set of pen
orientations, and a small fixed set of arm postures for each tool pose. Each
sample is one joint configuration.

For each sample, compute the distance of every self pair and every pair against
the table, platform and turntable. The pen and bracket are rigidly attached, so
pen-against-arm is part of self-collision and belongs here.

Each sample stores a certified bubble instead of a pass or fail bit. The
existing motion-radius bound turns a pair's clearance into a region of joint
space where that pair is still clear. Taking the tightest pair per joint gives
one weight vector per sample, and a configuration lies inside the bubble when
the weighted joint offset from the sample is below one.

A motion is certified for self and static collision when every interval of it
lies inside some sample's bubble. There are no stored edges; overlapping
bubbles play that role.

### 2. Robot footprint (robot against the printed structure)

Structure exists only inside the cube, so only the part of the robot inside the
cube matters. For one posture the whole robot is a fixed shape, made of two
parts that are stored differently.

| Part | Depends on | Storage |
|---|---|---|
| Flange, bracket and pen | Tool orientation only | One voxel template per sampled orientation, shifted to the tip position |
| Arm links above the flange | Full posture | Voxels inside the cube, stored per sample; expected to be empty or small for most postures |

The footprint of a motion between two samples is the union of the two
footprints, inflated by the motion bound between them.

### 3. Occupancy grid (the growing structure)

The structure is kept as an occupancy grid over the cube alongside the exact
capsules in `MaterialScene`. Each finished line is rasterized into the grid,
inflated so the grid never under-covers the real material.

## Runtime Flow

For a new line to draw:

1. Look up the tip cells the line passes through.
2. Keep the pen orientations and postures whose bubbles cover the whole line.
   This is also a candidate filter that runs before any search.
3. Intersect the motion footprint with the occupancy grid.
   - No intersection: the motion is clear of the structure.
   - Intersection: the exact checker decides.
4. Draw the line, then add it to the occupancy grid and the exact scene.

Anything the table cannot certify, including motion outside the cube, uses the
current `check_bounded_path` unchanged.

## Correctness Argument

- **Bubbles.** A body point moves no farther than the sum of joint changes
  times the motion radii. If that bound is smaller than a pair's clearance minus
  the margin, the pair cannot have closed. This is the same inequality
  `check_bounded_path` relies on, evaluated once per sample.
- **Footprints.** If the robot footprint over-covers the robot and the grid
  over-covers the structure, an empty intersection implies real separation.
- **Margin.** Both inflations must include the required margin for an empty
  intersection to certify it.
- **Rounding.** Stored weights are rounded so a bubble can only shrink.

The design therefore never certifies a motion the exact checker would reject.
It can fail to certify a motion that is in fact clear.

## Tradeoffs and Alternatives

- **Storage.** A sample is a configuration plus a weight vector. At a 1 cm tip
  grid with 30 orientations and 4 postures, that is about half a million
  samples and tens of megabytes. A 5 mm grid is several times larger. These
  counts are estimates.
- **Posture policy.** The table is finite only because the redundant arm
  freedom is restricted to a few choices per tool pose. Lines needing another
  posture fall back to online search.
- **Tip contact.** A line normally starts or ends on existing material, so its
  footprint always meets occupied voxels there. The voxel test needs the same
  exemption for joined strands that the exact checker already has.
- **Benefit ceiling.** Bounded-path checks were about 30% of the measured run,
  which caps what this can save in that workload.
- **Per-edge swept volumes, as in the papers.** Rejected for storage: at 1 mm
  resolution the estimate runs to hundreds of gigabytes. Shifting a tool
  template gives the same test.
- **Voxels as the authority.** Rejected: a grid cannot resolve a 1 mm margin
  at contact.
- **GPU sphere models.** Rejected under the existing project rule.

Two smaller changes to `check_bounded_path` are independent of this design and
cheaper: passing only uncertain pairs to child intervals, and using bounding
spheres as a lower bound before calling FCL.

## Non-Goals

- Replacing the exact FCL checker or its certificates.
- Precomputing every possible stroke or edge.
- Custom hardware or GPU acceleration.
- Transit planning outside the print cube.
- Changing the build-order or stroke search.

## Open Decisions

- **Where the 3 ms per evaluation goes.** If material queries dominate, the
  footprint and occupancy parts matter most. If self and static pairs dominate,
  the clearance table does.
- **Coverage.** What fraction of an existing build's intervals fall inside the
  bubbles at a given grid spacing.
- **Cube placement.** Whether the cube is centred on the turntable axis, so one
  table serves every turntable index.
- **Posture policy.** How many arm postures per tool pose are needed.
- **Occupancy resolution.** 1 mm gives a 150-cubed grid of about 3.4 million
  voxels; coarser grids send more cases to the exact checker.
- **Arm-link footprints.** Whether per-sample arm voxels are worth storing, or
  whether postures with a link in the cube should always use the exact checker.

## Relationship to Implementation Planning

This is not ready for an implementation plan. The first two open decisions
need measurements on an existing build: a timing split of one Palace section,
and a coverage count from a coarse table. Those results decide which of the
three parts is worth building.

## References

- Murray, Floyd-Jones, Qi, Sorin, Konidaris. "Robot Motion Planning on a Chip."
  RSS 2016. https://cs.brown.edu/people/gdk/pubs/plan_chip.pdf
- Murray, Floyd-Jones, Qi, Konidaris, Sorin. "The Microarchitecture of a
  Real-Time Robot Motion Planning Accelerator." MICRO 2016.
  https://cs.brown.edu/people/gdk/pubs/plan_chip_micro.pdf
- Murray et al. "A Programmable Architecture for Robot Motion Planning
  Acceleration." ASAP 2019. https://ieeexplore.ieee.org/document/8825107
- Schwarzer, Saha, Latombe. "Adaptive Dynamic Collision Checking for Single and
  Multiple Articulated Robots in Complex Environments." IEEE Transactions on
  Robotics, 2005.
