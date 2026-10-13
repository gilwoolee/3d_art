# Fast planning and verification: what the current version does, and what limits it

Recorded 2026-10-13 on branch `geometry-first-planning`. Measured on Blondie
(24 cores without hyper-threading, 62 GB RAM, RTX 5090 unused). Everything is
simulation only. History and earlier tables are in the
[geometry-first planning plan](2026_1012_geometry_first_planning_plan.md).

## Where we are

End-to-end time is planning plus the separate full-robot verification. Every
build below is `FULL_BUILD_VERIFIED`, with every action adapted from the motion
library, no adaptation failure and no timing rejection.

| Object | Actions | Start of 2026-10-13 (16 workers) | Now (24 workers) |
| --- | ---: | ---: | ---: |
| Palace, 8 mm feature | 366 | 937.0 s | 59.1 s |
| Branching shape, 8 mm feature, 1.5 mm strand | 151 | 237.2 s | 21.3 s |

The Palace by stage, as each change landed:

| Step | Workers | Setup | Collision-free path | Timing | Verification | Total |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Start of day | 16 | 159 s | 527 s | 69 s | 178 s | 937.0 s |
| Progress report at most every 5 s | 16 | 155 s | 293 s | 69 s | 179 s | 700.7 s |
| Build-order screens on workers; lighter library lookup | 16 | 67 s | 257 s | 69 s | 177 s | 573.6 s |
| 24 workers | 24 | 28 s | 204 s | 49 s | 144 s | 429.0 s |
| Newton inverse kinematics; first polynomial fixes | 24 | 27 s | 127 s | 44 s | 119 s | 320.7 s |
| Scheduler bookkeeping | 24 | 27 s | 86 s | 44 s | 120 s | 280.6 s |
| Polynomial pieces from coefficient tables | 24 | 27 s | 79 s | 39 s | 94 s | 242.2 s |
| Verification accepts planning's speed and coverage proofs | 24 | 27 s | 78 s | 43 s | 90 s | 242.5 s |
| Collision: re-check only unresolved pairs, reuse measured clearances | 24 | 27 s | 76 s | 43 s | 84 s | 234.0 s |
| No-posture-change rule (a new check, no rejections) | 24 | 27 s | 77 s | 43 s | 84 s | 235.4 s |
| Verification report keeps only the final deformed shape (1.2 GB to 8 MB) | 24 | 27 s | 77 s | 43 s | 66 s | 217.1 s |
| Verification accepts the pen proof; arm links checked against one envelope | 24 | 27 s | 76 s | 45 s | 51 s | 203.8 s |
| Verification accepts planning's structural screens; faster clock parsing | 24 | 27 s | 77 s | 45 s | 32 s | 185.4 s |
| Library coverage survey moved to a worker (it only informs the report) | 24 | 11 s | 77 s | 46 s | 32 s | 169.8 s |
| Closed-form path clock; no second tip-speed audit of drawing | 24 | 11 s | 77 s | 31 s | 33 s | 156.8 s |
| One path piece per straight member; pen checked as spheres at spaced poses | 24 | 11 s | 58 s | 21 s | 26 s | 117.4 s |
| Proof context hashed once; pre-validated library records; joint positions solved only at member ends; predicted wire reused | 24 | 11 s | 19 s | 8 s | 19 s | 59.1 s |

Thirty-two workers were tried and do not help: Palace 301 s against 281 s with
24 at the same code state, because the machine has 24 cores. Memory is not the
limit (16.9 GB peak at 32 workers, 12.9 GB at 24, of 62 GB). The 32-worker
verification stage overlapped an edit and a test run, so only the planning
stages of that run are a clean comparison (162 s against 161 s).

Twenty-four workers is the setting to use on this machine.

### Member-length path pieces (2026-10-13)

A drawing used to be cut wherever the adapted library path had a sample, about
every 0.45 mm. It is now one piece per straight member of the wireframe
(`member_drawing_path`), and a member is split only where its tool path would
leave the 0.3 mm tracking tolerance. The joint path is fitted separately
between the corners where the pen stops, so a member between two corners is
exactly a straight joint segment. The Palace went from about 170 to 52 path
edges per action; verification checks 36,808 edges instead of 59,857.

Three things had to change with it:

- **The clock.** With no knots inside a member the clock could not ramp, so
  clock knots are placed at 1, 2, 4, 8 mm of tool travel from each stop and at
  the middle of each run (`ramp_knots`). Print time fell, 8,822 s to 8,275 s on
  the Palace and 3,547 s to 3,361 s on the branching shape, because the clock is
  no longer held to a knot every half millimetre.
- **The pen check.** The pen is a row of 65 spheres generated from the tool
  geometry (`pen_spheres.py`), checked against wire at both ends of each piece
  and at poses no more than 10 mm apart. On 3,597 random wire placements the
  spheres never reported more clearance than the exact shapes; the median
  over-estimate is 0.17 mm. **This is a sampled check, not a continuous proof**,
  by decision of 2026-10-13. The pen against fixed obstacles and all contact
  moves keep their continuous checks.
- **Wire laid so far.** Each checked pose sees only the wire deposited up to
  that pose. Adding a member's whole wire before checking was tried first and
  blocked the Palace: at the start of a rising member the nose is 0.72 mm from
  that member's own not-yet-drawn wire, inside the 1 mm margin.

No action was rejected on either shape. The new pen check costs 14.6 s of
worker time on the Palace (14,320 poses) plus 4.0 s for the fixed obstacles.
The check stage is still 247 s of worker time; what remains is the contact
checks at corners, approach and withdrawal, and bookkeeping around them.
Full replay (`--full-replay`) also verifies the branching shape (39.3 s).

### Bookkeeping removed (2026-10-13, later)

A profile of every worker job showed that about a third of planning's worker
time was not planning. Four changes, the first two of which change no result:

- **Proof context hashed once.** Every proof lookup or save re-serialised and
  hashed the whole planning context (cell profile, all source actions, tool,
  settings): 5,859 calls at 0.11 s. The planner now returns the same frozen
  context object while nothing it depends on has changed, and the store keeps
  its text (`FrozenContext`). Proof keys are byte-identical. This had been
  counted as "source coverage", "timing audit" and "material validation".
- **Pre-validated library records.** A lookup parsed 3-55 MB JSON files and
  re-validated them, about two per action. The part planning reads is now kept
  validated beside the index (`index/*.selected.pkl`, bound to the original
  file's hash and this module's source) and loads in 7-25 ms.
- **Joint positions solved only at member ends.** Adaptation solved 66 points
  per action and the planner then discarded most of them. It now solves member
  ends and corners, and a member's middle only where its tool path would leave
  tolerance. This changes the joint paths slightly.
- **Predicted wire reused.** Withdrawal planning reuses the wire enclosures
  computed when the deposit was predicted, and a transit that lays no wire no
  longer copies the scene.

Palace worker time by job, before and after: propose 646 s to 173 s (lookup
370 s to 48 s, adaptation 232 s to 53 s), check 340 s to 171 s, connect 267 s
to 51 s, timing 471 s to 185 s. Print time is unchanged (8,277 s). The first
run after an edit to `motion_library.py` rebuilds the index and the record
cache once (about 40 s).

What is left of the Palace's 59 s: setup 11 s, path 19 s, timing 8 s,
export 2 s, verification 19 s.

### Vertical risers in the wireframe (2026-10-13, later still)

The generator now builds each zigzag as WirePrint does: a riser straight up
from the lower contour, then one slanted stroke back down. Before, both strokes
were slanted at about 15 degrees and nothing was vertical. This changes the
object that is planned, so the rows above and below are not like for like.

| | Slanted both ways | Vertical risers |
| --- | ---: | ---: |
| Palace: actions / members / wire | 366 / 6,879 / 24.8 m | 271 / 4,854 / 18.0 m |
| Palace: layer height | 2.12 mm (refined from 3) | 2.94 mm |
| Palace: planning and verification | 59.1 s | 43.0 s |
| Palace: print time | 8,277 s | 6,116 s |
| Branching: strand needed to pass the structural screen | 1.5 mm (1.2 mm sag at 1 mm) | 1 mm (0.10 mm sag) |
| Branching: planning and verification | 21.3 s | 19.1 s |
| Branching: print time | 3,360 s | 3,354 s |

Risers lean where the outline moves between layers: the median is 78 degrees
from horizontal on the Palace and 81 on the branching shape, with 190 of 528
and 110 of 268 within 5 degrees of vertical. The slanted strokes are now about
16-17 mm long at about 10 degrees (they were 9.5 mm at 13-18). Every steep
stroke was drawn upward in both plans. The Palace had two rejected candidates
(action 257, source coverage) that planning recovered from; the branching
shape had none.

Runs: `runs/geomfirst_gf40_{palace,branching}_w24` (current, sources
`runs/geomfirst_{palacevr,branchingvr}_f8_l3_source`),
`runs/geomfirst_gf39_*_w24` (slanted zigzags),
`runs/geomfirst_gf37_*_w24` (member pieces, before the bookkeeping changes),
`runs/geomfirst_gf33_*_w24` (a piece per path sample),
`runs/geomfirst_gf21_*_w32` (32 workers), `runs/geomfirst_gf14`-`gf26` (steps).
The stage breakdown further down was taken at the 242.2 s step.

## What the current version does

### 1. Wireframe from the mesh (`abstraction/wireprint.py`, `abstraction/solidify.py`)

- Welds coincident points before slicing, and retries an open cut a few microns
  higher or lower before calling the surface open.
- Prints only the outer shell of a cavity (inner contours born above the bed are
  omitted).
- Optional solidify (`wireprint.minimum_feature_m`): rebuilds the mesh on a 1 mm
  grid as a closed solid without parts thinner than the feature size, recorded
  with its deviation from the source.
- Ends a shape at the last contour of printable size instead of letting a
  rounded tip thin every layer.
- Stands zigzags on the nearest part of the contour beneath where contours split
  or merge, or where equal-fraction pairing gives a too-steep diagonal.
- If the frame fails the structural screen, regenerates it with the next heavier
  strand in `wireprint.heavier_strands_m` before planning starts.

These decide the action count, which is the main driver of every later stage:
the Palace went from an unbuildable 1,412 actions to 366.

### 2. Setup (`parallel_build.prepare_context`)

- Fixes one build order with a greedy search that needs a structural screen of
  every prefix of the order. The screens of the expected order are computed
  ahead on all workers (`_prescreen`); the search itself is unchanged.
- Freezes the motion library for the run.

### 3. Collision-free path, untimed (`--geometry-first`)

- A coordinator hands three kinds of job to forked worker processes:
  **propose** (find a joint path for one action), **check** (validate it
  against the wire printed before it) and **connect** (the transit from the
  previous action). Each result is bound by digest to the upstream results it
  assumed and is discarded if those change.
- Propose looks up similar stored motions in the library and adapts one to the
  new geometry: depth-first with backtracking over inverse-kinematics
  candidates, each step checked for tracking and collision. No clock is
  involved at this stage.
- Inverse kinematics tries minimum-norm Newton steps from the seed first and
  falls back to the bounded least-squares solver only when those do not
  converge inside the joint limits.
- Printed wire is represented as merged capsules (20 um budget), about fifty
  times fewer than the unmerged capsules.
- Joint polynomial pieces keep plain coefficient tables for themselves and two
  derivatives and evaluate bounds directly; subdivision reuses stationary points
  already found. Results match the previous numpy implementation to 2e-16.
- The scheduler memoises its job specifications between state changes and binds
  each job to the preceding material with one running digest.
- Live progress is written at most every 5 s. The library lookup keeps eight
  records loaded per worker and checks each record file's hash, without
  re-serializing the record to recompute a digest the file hash covers.

### 4. Timing, once, at the end

- After the whole chain is collision-free, each action is timed independently on
  the workers: time-optimal retiming on the path's own knots, with stalled parts
  retimed on a finer grid and a finer grid on numerical failure.
- Since 2026-10-13 the clock itself comes from a closed-form forward and
  backward pass (`timing.scan_clock`, `--retiming-solver scan`, the default with
  `--geometry-first`) instead of the TOPP-RA solver: each knot gets the largest
  path speed its joint-velocity, tip-speed and curvature limits allow, and the
  two passes limit how fast that speed may rise and fall under the exact
  joint-acceleration limits at both ends of each piece. As before, the result
  is then bounded over every whole interval (Bernstein coefficients of eight
  subintervals, within 1% of the true maximum) and slowed uniformly by whatever
  the bounds require, so the limits are guaranteed, not assumed. The modelled
  print is 0.6% longer on the branching shape and 0.9% on the Palace.
- The audit no longer bounds drawing tip speed a second time: the clock is
  already rescaled against a tip-tangent bound over every interval. Joint
  velocity and acceleration are still audited on the saved clock.
- A same-sweep check certifies that the timed clock visits exactly the untimed
  configurations, so the geometric proof carries over.
- A timing failure is reported (`PARALLEL_FORWARD_TIMING_REJECTED`) and never
  triggers re-planning.

### 5. Verification (`planning.cell_replay`, a separate command)

- Reloads the exported plan and re-audits it with the full robot model: source
  coverage, clock and path audits, deposit journals, the continuous full-robot
  collision replay against growing wire, and one structural screen per
  completed action. This stage is kept by decision; only its overhead is cut.
- Runs on the same number of workers; writes progress at most every 5 s and the
  large structural release states only in the final report.
- With `--accept-geometry-proofs` (decided 2026-10-13) it accepts planning's
  proofs of joint and tip speed limits and of source coverage for the exact
  saved clocks. Those audits read only the path, the arm's kinematics, the
  source curve and the process settings, and each proof is bound to all of
  them by digest. Collision, orientation, deposits and structure are still
  checked in full. Without the flag verification is fully independent, as
  before.

### 6. How densely the robot is collision-checked

The check is continuous, not sampled: at a configuration it measures every
clearance and certifies the whole interval if each clearance exceeds a bound on
how far that body can move within it; otherwise it halves the interval. On the
Palace's verification that is 62,194 edges (median 0.06 degrees of joint motion
and 0.45 mm of tip travel each), 74,824 path checks and 212,328 evaluated
configurations: 3.4 per edge, concentrated where the pen is close to wire.

Two things now keep that from costing a full-robot query each time, without
weakening the proof:

- **Only what is unresolved is re-checked.** When an interval is halved, the
  halves query only the pairs the whole interval left unresolved, usually the
  pen against nearby wire. Everything else was already certified for the whole
  interval.
- **Measured clearances carry across edges.** Clearances of the arm against
  itself and the fixed cell, measured at one configuration, certify every later
  configuration whose bounded motion from it stays inside them. Because the arm
  barely moves per edge, 75% of the Palace's path checks needed no self or
  fixed-obstacle query at all.

Self and fixed-obstacle query time in the Palace's verification fell from 156 s
to 28 s of worker time. The new checker returned the same verdict as the
previous one on 4,800 random edges (148 of them collisions). The pen against
deposited wire is checked exactly as densely as before.

## Where the Palace's 242 s goes now

Wall time, 24 workers:

| Stage | Wall | Share |
| --- | ---: | ---: |
| Setup | 27 s | 11% |
| Collision-free path | 79 s | 33% |
| Timing | 39 s | 16% |
| Assembly and export | 4 s | 2% |
| Verification | 94 s | 39% |

Worker time inside the planning stages (summed over workers, 366 actions):

| Job | Worker time | Per action | Largest parts |
| --- | ---: | ---: | --- |
| Propose | 783 s | 2.1 s | library lookup 376 s, adaptation 241 s, drawing proof 103 s, deposit prediction 55 s |
| Check | 613 s | 1.7 s | material validation 451 s, approach and withdraw 129 s |
| Connect | 344 s | 0.9 s | transit validation 173 s, source coverage 144 s |
| Time | 903 s | 2.5 s | retiming 490 s, timing audit 361 s |

Workers are 92% busy during the path stage, so the coordinator is no longer the
limit. Verification's named phases: loading and source audits 26 s, clock parse
and path checks 13 s, deposit journals 6 s, collision replay 23 s. About 25 s
falls outside those phases (process start, structural screens, writing the
final report) and has not been separated.

### 7. What verification still checks for collision

Decided 2026-10-13, and the default; `--full-replay` restores the complete
independent replay. Planning stays pen-only. Verification no longer repeats
planning's pen checks when it can show they apply:

- **The pen and tool against the cell and the wire.** Planning's check and
  transit jobs proved this for each action's untimed path against the exact
  chain of deposits before it. The timing job certifies that the saved clock
  sweeps the same configurations and writes a proof bound to that clock, the
  pen's geometry, the fixed obstacles and the chain. Verification accepts an
  action when its saved clock holds such a proof and the chain it names is
  exactly the deposits of the accepted actions before it. The first action
  that does not match, and every one after it, gets today's full check.
- **The arm.** Every edge is still checked continuously with the full robot:
  against itself (tool against arm included), and each arm link against the
  table, riser and turntable and against one upright cylinder that encloses
  every wire of the finished object. An arm that cannot be certified clear of
  itself, the cell or that cylinder is a rejection (`ARM_NOT_CLEAR`): the arm
  has no business inside the volume the object occupies.
- **How the arm check is answered.** Clearances measured at one configuration
  certify every later one whose bounded motion stays inside them. On the Palace
  61,247 of 62,193 edges (98.5%) were certified from that lookup with no new
  measurement, 1.006 evaluations per edge, 34 s of worker time in all.
- **Not re-derived for accepted actions:** deposit enclosures, pen contact
  geometry and fixed orientation of the timed clock.

On both shapes every action was accepted and none was rejected: the arm never
comes near the printed volume. The collision stage went from 19.0 s to
2.5 s on the Palace and from 7.4 s to 1.4 s on the branching shape. Without the
`--full-replay` verification runs the complete independent replay as before.
The deposit journals are still rebuilt (5 s on the Palace) for the final
material record.

Verification also accepts the structural screens planning ran to choose the
build order (`structural_screens.json`), when a saved verdict names the same
graph, material, screening code and exactly the members built so far; the
finished object is always solved again. 365 of 366 Palace screens were accepted.
Clock parsing no longer builds numpy polynomials to compare joint velocities at
edge joins. Together these took loading and source audits from 23.6 s to 7.4 s.

### 8. No posture change while the pen works

Seven joints let the arm swing its elbow with the tip still. On drawing,
approach and withdraw edges the largest joint motion may not exceed 1 degree
per millimetre of tip travel (`clock_path.posture_change`): per edge over at
least 0.1 mm, and per unbroken run of such edges over its travel plus 1 mm.
Planning rejects a candidate that breaks it and tries another; verification
rejects the plan. Travel between actions is exempt. Verified builds stay below
0.25 degrees per millimetre, and neither shape lost a candidate to the rule.

## The Palace at 217 s (now 185 s), and what under 60 s would take

| Stage | Wall | What it is |
| --- | ---: | --- |
| Setup | 26.7 s | build-order structural screens, library coverage report, cell setup |
| Collision-free path | 77.1 s | 1,699 s of worker time: propose 765 (library lookup 364, adaptation 233, drawing proof 105, deposit prediction 53), check 591 (material validation 436, approach and withdraw 123), connect 343 (transit validation 175, source coverage 143) |
| Timing | 43.2 s | 1,003 s of worker time: retiming 494, audit 366 |
| Assembly and export | 4.0 s | |
| Verification | 66.1 s (now 32.2 s) | load and source audits 23.5 (now 7.4), path checks 8.3 (now 9.3), deposit journals 6.0 (now 5.1), collision replay 19.0 (now 2.9), other 9.3 (now 7.5) |

Worker-time figures are from the 234.0 s run; the later changes did not touch
those stages. About 3,300 s of worker time and 40 s of single-core time make up
the total. On 24 cores, 60 s leaves room for roughly 1,000 s of worker time and
15 s of serial time: a threefold cut in both. Estimates, not measurements:

| Change | Palace | Branching |
| --- | ---: | ---: |
| Seed each action from the layer below instead of the library | -12 to -15 s | -4 to -5 s |
| Vectorized or compiled retiming constraints | -10 to -15 s | -4 to -6 s |
| Compiled and batched kernels for tracking, speed, clearance and kinematics | -40 to -60 s | -12 to -18 s |
| Carry deposit enclosures and orientation over by the same-sweep certificate | done: -13 s | done: -4 s |
| Reuse setup's structural screens in verification; faster clock parsing | done: -19 s | done: -4 s |
| Incremental structural screens and a deferred coverage report in setup | -12 to -18 s | -3 to -4 s |

The branching shape (71.8 s) should pass under 60 s with any two or three of
these. The Palace needs nearly all of them to land at the upper end of their
ranges, which would put it around 60-90 s; a firm 60 s also needs either more
cores or fewer actions. Time is close to linear in the action count (0.59 s per
action now), and the Palace has 366 because its layers are 2.1 mm.

Research on making collision checking and timing near-instant, with sources:
[collision and timing research](2026_1013_collision_and_timing_research.md).

## Bottlenecks and what could remove them

Estimates below are judgements from the profiles, not measurements, unless a
number is attributed to a run.

### A. Python overhead in small numeric kernels (largest remaining lever)

The profiles are dominated by interpreter overhead, not arithmetic: forward
kinematics costs about 70 us per call in numpy and is called millions of times;
tracking, speed and clearance bounds each subdivide intervals in Python. Today's
gains (adaptation down from 1,568 s to 241 s of worker time on the Palace,
polynomial tables) came
from removing such overhead without changing any check.

- **Method:** compile the kernels (forward kinematics, Jacobian, tracking bound,
  tip-speed bound, capsule distance) with numba or a small C++ extension, and
  evaluate them in batches per interval instead of per call.
- **Expected effect:** these kernels are most of check, connect, timing audit
  and the verification path audit, roughly two thirds of all worker time. A
  ten-fold kernel speed-up would plausibly halve the end-to-end time again.
- **Risk:** low for correctness if outputs are compared against the current
  code, as was done for the polynomial tables.

Other mechanisms that attack the same cost without a compiler:

- **Ask fewer questions.** The collision change of 2026-10-13 is the model:
  reuse a measured margin instead of re-measuring. The same idea applies to
  tracking and tip-speed bounds, which subdivide every edge from scratch
  although neighbouring edges are nearly identical.
- **Share forward kinematics between audits.** Tracking, orientation, tip speed
  and collision each evaluate the arm at the same configurations of the same
  edge. One evaluation per configuration, passed to all of them, removes most
  repeated calls.
- **Tighter bounds.** The tip-acceleration bound assumes the worst case for the
  whole chain. A bound from the actual Jacobian over the interval would certify
  most edges without subdividing.
- **Vectorize across intervals in numpy.** Process all pending intervals of an
  edge as one array operation instead of one Python call each. This is batching
  without a new dependency and a natural step before compiling.
- **An existing kinematics library.** Pinocchio provides compiled forward
  kinematics and Jacobians for a URDF; adopting it replaces the hottest kernel
  without writing C++.
- **Drop per-call validation in inner loops.** Each kinematics call re-validates
  its inputs and each transform is re-checked for rigidity; validating once per
  edge is enough.

### B. Verification repeats work planning already did

Verification re-derives deposits, re-audits every path and re-screens structure
that planning checked minutes earlier, by design: it is the independent check.

- **Done: accept planning's speed and coverage proofs**
  (`--accept-geometry-proofs`). All 366 Palace actions and all 151 branching
  actions matched all three proofs. The saving is smaller than the 20-40 s first
  estimated: path checks fell from 13.4 s to 9.0 s on the Palace and from 5.4 s
  to 2.0 s on the branching shape. The Palace's total did not move (242.5 s
  against 242.2 s) because planning's timing audit now covers the transit as
  well, about 4 s more; the branching shape gained 4 s. Two things had to change
  for the proofs to match: they are bound to a kinematic context that leaves out
  the collision model, and planning audits the clock exactly as saved, since a
  speed proof binds every floating-point duration and joining the transit
  shifts them.
- **Not accepted, on purpose:** planning's self and fixed-obstacle collision
  proof. Planning checks the pen only; the full robot is checked in
  verification.
- **Still open (overhead only):** loading and source audits are 26 s of mostly
  single-core time on the Palace: 40 MB of JSON read back and parsed. Binary
  arrays for clocks would remove most of it. About 25 s more falls outside the
  named phases and has not been separated.
- **Still open:** orientation and deposit enclosures are re-derived for the
  timed clock although the untimed path was checked. The same-sweep certificate
  could carry those over as it now does for source coverage.

### C. Library lookup now costs more than adaptation

Lookup is 1.0 s per action; adaptation is 0.7 s. Each request scores all 80
records and loads up to four record files of several megabytes each.

- **Method:** in a layered object, consecutive actions are near copies of each
  other one layer apart. Seed each action from the same kind of action one layer
  below in the *current* build before consulting the library. That removes most
  lookups and gives a closer seed than any library record.
- **Alternative:** keep the library but index it (precomputed geometry
  descriptors, records stored as arrays instead of JSON).

### D. Timing (16% of wall, 2.5 s of worker time per action)

Retiming builds the time-optimal problem's constraints in Python at every grid
point, then the audit bounds tip speed by subdivision.

- **Better solver:** the algorithm (TOPP-RA) is already the right one; the cost
  is constraint assembly in Python. The library's compiled backend, or
  assembling constraints for all grid points in one vectorized call, addresses
  that directly.
- **Different method:** drawing moves are straight Cartesian segments at fixed
  orientation with stops at corners. A closed-form trapezoidal or S-curve
  profile per segment, checked by the existing audit, would avoid the solver for
  most pieces. It would lose some print-time optimality on curved joint paths.

### E. Material collision checks (check and connect, about 600 s of worker time)

Each check queries FCL from Python for one robot configuration at a time against
the nearby wire capsules, and subdivides until an analytic bound proves
clearance.

- **GPU:** this is the one stage that fits a GPU: many configurations times many
  capsules, all independent. The earlier
  [GPU findings](2026_1012_gpu_validation_findings.md) reach the same
  conclusion: use the GPU to validate, not to generate candidates. Two caveats.
  The acceptance test must stay conservative, so a GPU pass can only *accept*
  intervals whose bound proves clearance and must hand the rest to the current
  checker. And workers are forked CPU processes that cannot share an initialized
  GPU, so this needs one GPU service the workers call, not a flag.
- **Without a GPU:** batching the same queries in numpy or numba (capsule against
  capsule and capsule against convex link, closed form) removes the per-call
  overhead and may capture most of the gain with far less integration work.
- **Not worthwhile:** GPU inverse kinematics. After the Newton change, inverse
  kinematics is a small part of 0.7 s per action.

### F. Structural screens are computed twice and from scratch

Setup solves one growing frame per action (366 solves), and verification solves
them again.

- **Method:** update the factorization as members are added instead of solving
  each prefix from scratch, and let verification reuse setup's screens when the
  order and graph hashes match. Worth about 10-15 s of the Palace's setup and a
  similar share of verification.

### G. More cores

Scaling from 16 to 24 workers gave 1.3 times on the Palace. The machine has 24
cores, and 32 workers gave nothing. A machine with more cores would help the
path, timing and verification stages roughly in proportion until the serial
parts (setup remainder, plan loading, export) dominate; those are about 60 s of
the Palace's 242 s today.

### What would not help

- **A trajectory optimizer for adaptation.** Adaptation has not failed on any
  of the eight objects and costs 0.7 s per action.
- **More memory.** Peak use is a fifth of what the machine has.

## Suggested order

1. Seed from the previous layer of the current build (C): small change, removes
   the largest part of propose.
2. Compile and batch the numeric kernels (A): largest total effect, touches
   check, connect, timing audit and verification together.
3. Binary plan exchange in verification, and carrying orientation and deposit
   enclosures over by the same-sweep certificate (B).
4. Vectorized or compiled-backend retiming (D).
5. Batched material collision, CPU first, GPU only if the CPU version leaves it
   the dominant cost (E).
6. Incremental structural screens (F).

Done since this list was written: accepting planning's speed and coverage
proofs in verification (B), and the collision density change (section 6).
