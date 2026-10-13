# Pen-against-wire collision check: design

**Date:** 2026-10-13
**Status:** Partly implemented on 2026-10-13, in a simpler form decided that
day: one path piece per straight member, the pen as a row of spheres checked
at both ends of each piece and every 10 mm, each pose against the wire laid so
far. That is a sampled check, not the proof described under "Goal" and in
section 1 below (which assumed half-millimetre pieces and a 1 mm spacing rule).
Implemented: the sphere model (section 2), batched distances (section 3) with
a distance cull in place of the grid of section 4, and validation item 3 in
part. Not implemented: the grid, shadow mode, and the change to deposited wire.
Measured results are in
[fast planning and verification](2026_1013_fast_planning_and_verification.md).
The rest of this page is the original design; numbers marked *estimate* are
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

### 1. Check at the ends of each short piece

The pen moves at fixed orientation and each piece moves it about half a
millimetre. If the pen is clear by the required margin at the start and at the
end of such a piece, it is clear in between.

That is more than a likelihood. Between two checked positions a sphere centre
travels a distance `s`, so it is never further than `s / 2` from a checked
position (plus a few microns of bulge from the curved joint path). If both
checked positions have clearance of at least the 1 mm margin, the clearance in
between is at least `1 mm - s / 2`. For the median piece (0.45 mm) that is
0.77 mm. Contact is impossible as long as `s` stays under 2 mm.

Rule: check every sphere at both ends of every piece, requiring the full
margin. Where a sphere centre travels more than 1 mm within a piece (the
longest drawing pieces, approach, withdraw and travel moves), add evenly spaced
checks so that no gap exceeds 1 mm. Clearance between checks is then at least
0.5 mm everywhere. No motion bound is derived and nothing is subdivided
adaptively.

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

The whole question becomes, at each checked position:

    distance(centre of sphere k, segment of wire w)  >=  r_k + r_w + margin

for every sphere `k` and every nearby wire `w`.

### 3. Batched closed-form distances

Point-to-segment distance is closed form and simpler than segment-to-segment.
All pairs for one piece, or for all pieces of an action, are evaluated as one
array operation. No collision library is involved. One action is about 170
pieces x 70 spheres x the wires within reach; with the culling below that is on
the order of a million pairs: tens of milliseconds in numpy, about a
millisecond compiled (*estimate*).

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

### 5. Deposited wire

The wire a piece deposits is recorded as the capsule between the tool point's
two end positions with the strand radius. The existing enclosure by subdivision
(`add_piece_material`) is kept where a piece is long or strongly curved; for
the typical half-millimetre piece it already produces exactly this capsule.

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
| Sphere centres at the checked positions of a piece | new function taking a `PolynomialPiece` and the arm |
| Batched point-to-segment distances | new, pure numpy |
| Append-only grid with birth tags | beside `MaterialScene` in `planning/scene.py`, fed by `MaterialPrefix` |
| Use in the check | the non-contact branch of `_validate_interval` that calls `check_bounded_path` with a wire scene and a pen model |
| Deposits | `add_piece_material` |

Selected by a setting (`pen_check: spheres | fcl`), default `fcl` until the
validation below passes.

## Validation before it becomes the default

1. **Distance kernel.** Batched segment distances against a scalar reference
   and against FCL capsule-capsule distance on random pairs, including
   parallel, touching and degenerate segments.
2. **Spacing rule.** On real plans, the largest travel of any sphere centre
   between consecutive checked positions must not exceed 1 mm.
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
| A long or fast piece is checked too sparsely | Clearance between checks falls below 0.5 mm | The 1 mm spacing rule adds checks; validation 2 measures the largest gap on real plans |
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

1. Distance kernel, sphere profile and the spacing rule, each with its test.
2. Append-only grid with birth tags, tested against brute force.
3. Shadow mode in the check job; run both shapes; study disagreements.
4. Switch the non-contact branch; then the deposits; re-run both shapes.
5. Measure; decide whether a compiled kernel is still worth it.
