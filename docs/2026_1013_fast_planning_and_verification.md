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
| Palace, 8 mm feature | 366 | 937.0 s | 242.2 s |
| Branching shape, 8 mm feature, 1.5 mm strand | 151 | 237.2 s | 83.0 s |

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

Thirty-two workers were tried and do not help: Palace 301 s against 281 s with
24 at the same code state, because the machine has 24 cores. Memory is not the
limit (16.9 GB peak at 32 workers, 12.9 GB at 24, of 62 GB). The 32-worker
verification stage overlapped an edit and a test run, so only the planning
stages of that run are a clean comparison (162 s against 161 s).

Runs: `runs/geomfirst_gf23_{palace,branching}_w24` (current),
`runs/geomfirst_gf21_*_w32` (32 workers), `runs/geomfirst_gf14`-`gf22` (steps).

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

### B. Verification repeats work planning already did (39% of wall time)

Verification re-derives deposits, re-audits every path and re-screens structure
that planning checked minutes earlier, by design: it is the independent check.

- **Method 1 (overhead only):** stop round-tripping through 40 MB of JSON
  between the two commands (binary arrays for clocks), and parse clocks on the
  workers instead of the coordinator. Loading and source audits are 26 s of
  mostly single-core time.
- **Method 2 (needs a decision):** let verification accept planning's path
  certificates for audits that depend only on the untimed geometry, keeping the
  full-robot collision replay. This is the open item G5. It trades independence
  for about 20-40 s on the Palace.

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
3. Binary plan exchange and worker-side clock parsing in verification (B,
   method 1).
4. Vectorized or compiled-backend retiming (D).
5. Batched material collision, CPU first, GPU only if the CPU version leaves it
   the dominant cost (E).
6. Incremental structural screens (F).

Item B method 2 needs a decision on how independent verification must remain.
