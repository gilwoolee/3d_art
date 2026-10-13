# Pen-against-wire collision check: design

**Date:** 2026-10-13
**Status:** Design agreed in outline (items 1-4 of the
[collision and timing research](2026_1013_collision_and_timing_research.md));
nothing here is implemented or measured. Numbers marked *estimate* are
reasoning, not results.

## Goal

Make the check of the pen and tool against printed wire take milliseconds per
action instead of about 1.1 s, and keep it a proof: no collision between
samples can be missed.

On the Palace (366 actions, 62,194 path pieces, 24 workers) the work this
replaces is about 570 s of worker time in planning: material validation 399 s,
transit validation 172 s, plus the wire-prediction share of propose (54 s).
That is roughly 25 s of the 77 s path stage.

## What is checked today, and why it is slow

For every path piece, `CellPlanner._validate_interval` does three things:

1. encloses the wire the piece deposits (`add_piece_material`, which subdivides
   the piece until a chord-plus-inflation capsule fits within 20 um);
2. calls `RobotCollisionModel.check_bounded_path` with the pen model and the
   wire scene. At the middle of the interval it asks FCL, one tool body and one
   wire at a time from Python, for the clearance, and certifies the interval if
   clearance exceeds *lever arm times joint motion* plus the 1 mm margin;
   otherwise it halves the interval;
3. for pieces that touch at a designated contact point, runs a separate contact
   validator instead of step 2.

Two things make step 2 expensive. The first-order bound (lever arm times joint
motion) is about as large as the clearance the pen actually has, so intervals
are halved again and again. And each clearance is a general convex-shape query
through a Python binding.

## Design

### 1. Bound the motion by its chord plus a second-order remainder

A point `p` rigidly attached to the tool moves along `p(u)` for `u` in
`[0, 1]` over one path piece. For any twice-differentiable curve,

    | p(u) - chord(u) |  <=  max |p''| / 8,

where `chord` is the straight segment from `p(0)` to `p(1)`. The curve
therefore lies inside the capsule around the chord with radius `max|p''| / 8`.

- `p(0)` and `p(1)` come from forward kinematics at the piece's two ends.
- `max|p''|` is bounded without sampling by the serial-chain bound the
  repository already uses for tracking and deposition
  (`timing.tcp_acceleration_bound`), fed with the piece's largest joint
  velocity and acceleration (`PolynomialPiece.derivatives`), and with the lever
  arm of the point in question in place of the tool-point offset.

With joint motion near 0.001 rad per piece, the remainder is of order
`lever * (velocity^2 + acceleration) / 8`: about a micron from the velocity
term, and up to tens of microns where the spline bends (*estimate*). The
first-order bound in use today is about a millimetre. Where the remainder is
still too large for the clearance, the piece is halved, which divides the
remainder by four; this replaces today's halving but should rarely trigger.

This makes no assumption that the pen stays upright or that the path is
straight. Orientation drift inside a piece is part of `p(u)` and is covered by
the same remainder.

### 2. Model the tool as spheres along its axis

The tool is a body of revolution about the pen axis except for the flange
adapter: a 12 mm-radius barrel 180 mm long, a nose tapering from 12 mm to
0.8 mm over 27 mm, and a 40 x 40 x 15 mm adapter block.

Represent it by spheres centred on the axis. A sphere of radius
`sqrt(R(h)^2 + (d/2)^2)` every `d` along the axis covers a profile of radius
`R(h)`; the over-estimate is about `d^2 / (8 R)`. Spacing 0.2 mm near the tip
and 4 mm on the barrel gives roughly 70 spheres with at most 0.01 mm of
over-estimate at the nose and 0.17 mm on the barrel (*estimate*; the spacing is
a tuning choice). The adapter block is covered by spheres on its bounding
cylinder. The profile is generated from `tool_geometry` in the cell profile, so
a different pen needs no code change.

By item 1, each sphere sweeps a volume inside the capsule around its centre's
chord with radius `sphere radius + remainder`. The whole question becomes:

    distance(segment of sphere k's chord, segment of wire w)
        >=  r_k + remainder_k + r_w + margin

for every sphere `k` and every nearby wire `w`.

### 3. Batched closed-form distances

Segment-to-segment distance is closed form. All pairs for one piece, or for all
pieces of an action, are evaluated as one array operation. No collision library
is involved. One action is about 170 pieces x 70 spheres x the wires within
reach; with the culling below that is on the order of a million pairs:
tens of milliseconds in numpy, about a millisecond compiled (*estimate*).

Arithmetic is double precision. A fixed rounding allowance (1e-9 m) is added to
the right-hand side.

### 4. One append-only wire index with birth order

Wire is only ever added, in the order the actions run. Each worker keeps one
uniform grid over the build volume. A wire capsule is inserted once, tagged
with the position in the deposit chain at which it appears. A check against the
material before action `k` queries the grid and ignores wires tagged `k` or
later. Nothing is copied or rebuilt per check; a later prefix only appends.

The grid cell is sized to the barrel radius plus margin, so a sphere tests the
wires in its own and neighbouring cells. The cull must use each sphere's full
inflated radius; that is the one place where a mistake silently drops a pair,
so the index is tested against brute force over all wires.

### 5. Deposited wire from the same chords

The wire a piece deposits is the tool point's path. By item 1 it lies inside
the capsule around the tool point's chord with radius
`strand radius + remainder`. That replaces the subdivision in
`add_piece_material`. If the remainder exceeds the present 20 um enclosure
tolerance the piece is halved, as now.

## What stays exactly as it is

- **Contact.** Pieces that touch at a designated contact point keep the
  existing contact validator and its exemptions. Getting contact wrong either
  fails every drawing or masks a real collision, so it is not part of this
  change.
- **The arm.** Arm links against themselves, the cell and the printed volume
  are unchanged.
- **The tool against fixed obstacles** (riser, platform, table). Unchanged at
  first; the sphere model could later serve there too.
- **Fixed-orientation and tracking checks.** Unchanged.
- **What a proof means.** The proof written by the timing job and accepted by
  verification binds the pen's geometry. The sphere profile becomes part of
  that binding, so proofs from the old checker are not accepted for the new
  one and the reverse.

## Where it plugs in

| Change | Place |
| --- | --- |
| Sphere profile from tool geometry | new, beside `planning/tool.py` |
| Chord and remainder per sphere per piece | new function taking a `PolynomialPiece` and the arm |
| Batched segment distances | new, pure numpy |
| Append-only grid with birth tags | beside `MaterialScene` in `planning/scene.py`, fed by `MaterialPrefix` |
| Use in the check | the non-contact branch of `_validate_interval` that calls `check_bounded_path` with a wire scene and a pen model |
| Deposits | `add_piece_material` |

Selected by a setting (`pen_check: spheres | fcl`), default `fcl` until the
validation below passes.

## Validation before it becomes the default

1. **Distance kernel.** Batched segment distances against a scalar reference
   and against FCL capsule-capsule distance on random pairs, including
   parallel, touching and degenerate segments.
2. **Remainder bound.** For random pieces of real plans, the largest sampled
   deviation of every sphere centre from its chord (thousands of samples per
   piece) must never exceed the bound.
3. **Profile cover.** Random points on the true tool surface must lie inside
   the sphere union.
4. **Index.** Grid queries equal brute force over all wires for random spheres
   and prefixes.
5. **Shadow mode on real builds.** Run both checkers on every piece of the
   branching shape and the Palace. Record every disagreement. The new checker
   must never certify a piece on which dense sampling with exact FCL distances
   finds clearance below the margin. Pieces the new checker rejects and the old
   one accepts are counted: they measure how much the sphere over-estimate
   costs.
6. **End to end.** Both shapes plan and verify with the new checker, with the
   same number of accepted actions.

Only item 5's "never certifies a real violation" is a correctness gate; the
others localise a failure.

## Risks

| Risk | Consequence | Mitigation |
| --- | --- | --- |
| Remainder bound derived with a wrong constant or lever arm | Certifies a collision | Validation 2 on real pieces; the bound reuses the function already trusted for tracking |
| Sphere over-estimate at the nose removes clearance the plan needs | Valid actions rejected, more re-planning | Fine spacing near the tip; validation 5 counts these; spacing is a setting |
| Grid cull radius too small | Silently skipped pairs | Validation 4 against brute force |
| Contact pieces handled by two different models | Inconsistent verdicts at corners | Contact branch untouched in this change |
| Compiled or GPU arithmetic in single precision | Margins of 0.1 mm lost to rounding | Double precision; explicit rounding allowance |

## Expected effect

*Estimates, to be replaced by measurements.* Pen-against-wire work in planning
from about 570 s of worker time on the Palace to under 30 s in numpy, less
compiled: about 20-25 s off the Palace's 77 s path stage and 6-8 s off the
branching shape's 25 s. Verification is unaffected when it accepts planning's
pen proofs, and gains the same factor under `--full-replay`.

## Order of work

1. Distance kernel, sphere profile, remainder bound, each with its test.
2. Append-only grid with birth tags, tested against brute force.
3. Shadow mode in the check job; run both shapes; study disagreements.
4. Switch the non-contact branch; then the deposits; re-run both shapes.
5. Measure; decide whether a compiled kernel is still worth it.
