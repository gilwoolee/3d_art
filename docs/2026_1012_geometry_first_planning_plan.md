# Geometry-first planning for new objects

2026-10-12. **Status: in progress; see results below.**

Establish a collision-free, untimed joint path through the whole object first;
time it once at the end. Alongside that: check fewer, longer path pieces, make
trajectory adaptation fail less, stop checking adapted paths twice, and widen
what the motion library covers. The target is planning a shape the library has
not seen, not re-planning the cube. Keep the single Flexiv 7-DoF arm, whole
connected drawing actions, fixed TCP orientation within DRAW, stationary
0/90/180/270-degree table indices, 1 mm clearance, 0.3 mm tracking and
0.003 rad orientation bounds. Simulation only.

This replaces the [cached swept-volume plan](2026_1012_cached_swept_volume_plan.md)
as the next step. Swept volumes are revisited only if G0 and G7 show that
arm-versus-material queries remain a large share.

## Results so far (2026-10-12, 16 workers, each verified by the unchanged full-robot replay)

| Build | Chain collision-free | Timing pass | Verification | Total | Modelled print |
| --- | ---: | ---: | ---: | ---: | ---: |
| Cube, timed first (reference) | 179.4 s | inside | about 212 s | 450.7 s | 1,917.9 s |
| Cube, untimed | 70.4 s | 103.4 s | 190.6 s | 422.8 s | 1,735.6 s |
| Cube, untimed + merged capsules | 66.7 s | 106.4 s | 179.6 s | 373.5 s | 1,735.6 s |
| Cube, + knots-only clock | 66.2 s | 23.2 s | 81.6 s | 185.2 s | 2,054.2 s |
| Cube, knots-only clock, stall fix | 66.0 s | 26.7 s | 84.3 s | 191.4 s | 1,796.1 s |
| Prism, knots-only clock, stall fix | 46.1 s | 13.4 s | 31.6 s | 98.4 s | 1,063.9 s |
| Narrowed cube, untimed + merged | 90.5 s | 106.9 s | 190.4 s | 410.1 s | not recorded |
| Prism, untimed + merged | 47.1 s | 54.5 s | 82.9 s | 195.7 s | 1,027.7 s |
| Prism, + knots-only clock | 39.8 s | 12.6 s | 31.2 s | 90.3 s | 1,401.2 s |

Every action in every untimed build was adapted from a library record on the
first attempt; none fell back to cold planning. Merged capsules cut the cube's
wire from 105,954 capsules to about 2,000 and peak memory from 15.4 GB to
8.0 GB. The knots-only clock gives one timed segment per path piece, so
verification already runs on the untimed piece count. The first knots-only runs
lengthened the modelled print by 18% and 36% because one leg per object stalled;
with stalled legs retimed on a finer grid the cost is 3.5% on both. `--geometry-first` now implies
`--merge-deposits-um 20 --retiming-grid knots`; both can be overridden.

Status: G0-G4 and G6 done; G8 done as a selectable clock; merged capsules added
(not in the original list). Open: G5 (verification still re-derives deposits and
re-proves orientation on the timed plan), G7, the waypoint-only adaptation
measurement in G6, timing-failure retry in G4, torus and bottle builds.

## Evidence

All numbers are from `runs/compute_cleanup_cube_high_v3_w16` (16 workers) and
the frozen library `runs/motion_library_phase1_upright27_v1`.

- **The 7.5-minute cube is a warm, same-object result.** 35 of 39 actions reused
  a stored trajectory unchanged; 36 stored cube trajectories came from the
  earlier `cube_nozzle27_full_v3` run. Exact reuse needs point-identical source
  geometry, context hash and table index (`motion_runtime.py:340-376`).
- **Adaptation roughly doubles per-action cost.** Adapted actions (sections 7,
  36, 37, 38) averaged about 110 worker-seconds against 56 for reused ones:
  adapt 21.6 s, build the timed trajectory 26.4 s, approach/withdraw 8.9 s extra.
  All four were adapted from near-identical cube zigzags and passed first time;
  adaptation failure rates on a different shape are unmeasured.
- **Timing sits inside the loop.** Timing audit is 539 of 2,411 planning
  worker-seconds; verification audits paths again (724 worker-seconds).
- **Timing does not change the sweep.** `prepare_local_drawing`
  (`cell_planner.py:916`) fixes the joint spline from tracking alone, and
  `timed_path` (`clock_path.py:341`) composes each cubic piece with a clock.
  Feed lags and cooling are stationary holds. Nothing currently tests this.
- **Most segments are a retiming artifact.** The retimer adds a midpoint per
  spline piece and 101 samples per drawing part (`timing.py:255-260`). A 24-corner
  zigzag has 193 source points, 384 tracked spline pieces and 3,731 timed DRAW
  edges; the stored cube actions total about 100,000 timed DRAW edges for roughly
  14,500 spline pieces (about 7x). Collision, orientation and deposit checks run
  per timed edge today. A 7x cut in pieces is not a 7x speedup: longer pieces
  subdivide more inside the continuous checker.

## Target workflow

```text
Today:    per action:  path -> timing -> collision + deposit + timing audit -> connect
Target:   per action:  path -> collision + deposit -> connect     (whole object first)
          once at end: timing -> timing audit -> holds/events -> schedule
          verify:      same-sweep guard + geometry proof + timing proof
```

A geometric path reuses `ClockPath`/`ClockEdge` with the spline parameter in
place of seconds, so `_validate_interval`, journals and material prefixes keep
working. Its DRAW edges are the tracked cubic pieces; travel, approach and
withdraw edges are the existing joint segments; holds become zero-motion edges
that keep their contact point. Metadata marks it `parameterization: geometric`.

## Existing code to reuse

- `cell_planner.py:58` `tracked_drawing_path` — spline and tracking bounds.
- `cell_planner.py:916` `prepare_local_drawing`, `:1011` `complete_local_drawing`
  — scene-free drawing, then scene-dependent approach/withdraw.
- `cell_planner.py:1277` `_validate_interval` — interval collision, deposition,
  contact and orientation checks; `deposit_only` and `journal` modes.
- `cell_planner.py:136` `add_piece_material` — deposit capsules from a polynomial
  piece at 20 um chord tolerance, independent of edge count.
- `clock_path.py:341` `timed_path` with `drawing_reuse`; `timing.py:205`
  `clock_polynomials`; `timing.py:236` `retime_operation`.
- `section_build.py:569` `check_forward_timing`; `process_clock.py:132`
  `audit_process_clock`; `turntable.audit_indexed_clock`.
- `parallel_build.py:487/572/646` `_propose`, `_check`, `_connect`.
- `motion_library.py:1225` `_adapt`, `:885` `retrieve`;
  `scripts/precompute_motion_cache.py`, `scripts/check_motion_cache_coverage.py`.
- `cell_replay.py:136` `_replay_cell_body` — separate final verification.

## Implementation checklist

- [ ] **G0 (counters done; reference builds running) — Baseline and a new-object reference.**
  Owners: `scripts/benchmark_parallel_build.py`, `planning/parallel_build.py`,
  `planning/motion_runtime.py`, `planning/cell_planner.py`.
  Commit the current working tree first (user approval needed).
  Record per worker: self/static, material and contact query time; pieces
  checked; continuous-checker evaluations. Record per action: retrieval
  candidates tried, adaptation outcome, and on failure the point index and
  reason (IK, tracking, collision, deadline). Aggregate across workers.
  Plan three references with the unchanged planning code, sequentially, in
  tmux, 16 workers, fresh run directories: the cube (warm), the cube narrowed
  to 98% in x (`runs/geomfirst_cube_narrow_v1_source`, seeds for every zigzag)
  and the triangular prism (`runs/geomfirst_prism_v1_source`, no zigzag seeds).
  These are the reference for every later claim.

- [x] **G1 — Geometric path and same-sweep guard.**
  Owners: `planning/clock_path.py`, `timing.py`, new test in
  `tests/test_timing.py`.
  Add `geometric_path(operations, arm)` producing the geometric `ClockPath`
  described above from the same operations `timed_path` receives.
  Add `same_sweep(geometric, timed)`: every timed DRAW edge must equal one
  geometric cubic piece composed with a nondecreasing clock, consecutive timed
  edges must tile each piece without gap or overlap, and every other edge must
  have identical endpoints and kind in the same order. Compare within
  1e-9 rad summed over a joint's coefficients, about a nanometre at the tool;
  a failed guard is a rejection, never a warning.
  Acceptance: the guard passes on a path with travel, holds, a corner stop and
  a reversing joint, and fails on a perturbed coefficient, a dropped edge, a
  reversed clock and a moved travel point. Saved plans hold no spline, so real
  cube actions are checked when G3 produces both forms.

- [ ] **G2 — Check geometry on geometric pieces.**
  Owners: `planning/cell_planner.py`, `planning/parallel_build.py`.
  Give `_validate_interval` a geometric mode keyed on the path metadata: skip
  `audit_indexed_clock`, cooling-before-turn and `support_ready` time tests;
  keep table-index agreement, deposition order, contact, orientation, material
  and self/static checks unchanged. Geometric mode requires one arm and raises
  otherwise; the two-arm branch stays timed.
  Deposits come from the geometric cubic pieces. The final material hash will
  differ from `31ba57ec...`; record the new reference and confirm equal capsule
  coverage against the old material within the 20 um enclosure tolerance.
  Acceptance: on the saved cube, geometric-mode validation accepts all 39
  actions and rejects the existing blocked-material probe.

- [ ] **G3 — Geometry-first planning loop.**
  Owners: `planning/parallel_build.py`, `planning/cell_planner.py`,
  `planning/section_build.py`, `planning/indexed_build.py`.
  `_propose` builds the tracked spline and geometric path, without retiming.
  `_check` adds approach/withdraw and validates geometry; it no longer calls
  `check_forward_timing`. `_connect` validates the connecting travel and source
  coverage on geometry. Planning ends with a `GEOMETRIC_CHAIN_COMPLETE` status
  and a saved geometric plan covering every action and the return home.
  Report time to that status separately from total time.

- [ ] **G4 — Timing as the final pass.**
  Owners: `planning/parallel_build.py`, `planning/section_build.py`,
  `planning/clock_path.py`, `planning/full_build_export.py`.
  For each accepted action, in parallel: `timed_path` on the saved spline,
  holds, deposition events, `check_forward_timing`, `audit_process_clock`,
  `audit_indexed_clock`, release readiness, then `same_sweep`. Assemble the
  schedule in build order. The saved plan keeps today's timed format and adds
  the geometric record, so export and the viewer are unchanged.
  A timing failure returns that action to G3 with its reason, at most three
  times, then stops with the action named; it never relaxes a limit.

- [ ] **G5 — Split final verification.**
  Owners: `planning/cell_replay.py`, `planning/parallel_build.py`.
  Verification recomputes `same_sweep` itself, replays full-robot collision and
  deposition on the geometric pieces, and audits timing on the timed edges.
  Keep verdicts, plan hashes and viewer gating; keep the raw mode fresh. Add
  piece counts and guard results to the verdict.
  Acceptance: the verifier rejects a plan whose timed edges were altered after
  the geometric proof.

- [ ] **G6 — Adaptation that fails less and checks once.**
  Owners: `planning/motion_library.py`, `planning/motion_runtime.py`.
  In `_adapt`, a failed point currently abandons the candidate. Instead back up
  over at most 8 preceding points and retry their alternative inverse-kinematics
  solutions, with a per-candidate attempt cap; raise the evaluation limit only
  on retry. Return the failing index and reason. In `candidates`, move to the
  next retrieved record after a capped failure instead of spending the
  remaining budget.
  Remove the per-step continuous self/static `check_edge` on the provisional
  linear path; keep the point check at each waypoint. The G2 check on the final
  spline is the authority. Keep the removal only if G0 counters show no rise in
  late rejections on the new-object reference.

- [ ] **G7 — Library keyed by geometry, with wider coverage.**
  Owners: `planning/motion_library.py`, `planning/motion_runtime.py`,
  `planning/motion_cache_precompute.py`, both library scripts.
  New records store the tracked spline and geometric path; exact reuse matches
  on geometry and cell context, not on timing settings. Existing records stay
  usable as adaptation seeds through `_compact_timed_seed` and are not
  rewritten. Publish a new library directory; never mutate the frozen one.
  Cover the union of all test-object wireframes: cube, narrowed cube, bottle,
  triangular prism and torus, 191 actions in 26 shape families, of which only
  two are shared between objects. Today the library seeds every cube and bottle
  zigzag and none of the prism's 14 or the torus's 12. Report coverage per
  object with `check_motion_cache_coverage.py --require-zigzags`.
  Retrieval already ranks records from any cell context by shape, so a cube
  zigzag can seed another object's zigzag; only unchanged reuse needs the same
  context, which includes the riser sized to one object's footprint. Coverage
  therefore depends on having a record of a similar family, not on re-keying.
  Palace is excluded for now: WirePrint slicing fails on its mesh with
  "Open contour at layer 1; repair the input mesh".

- [ ] **G8 — Cheaper timing pass.**
  Owners: `timing.py`.
  Replace the fixed 101 samples per drawing part with samples spaced by path
  length, keeping every spline knot. Accept only if the timing audit still
  passes every cube action with no added time-scale correction.

- [ ] **G9 — Measure and accept.**
  Owners: `scripts/benchmark_parallel_build.py`, tests under `tests/`.
  Extend existing contracts; keep the routine suite below 30 tests (28 today).
  Compare against G0 on the cube and on the new-object reference, in fresh
  runs: time to a complete geometric chain, timing pass, verification, total,
  pieces checked, adaptation success rate and peak tree PSS. Browser-check one
  separately verified result.

## Risks

| Risk | Impact | Mitigation |
| --- | --- | --- |
| Timed motion differs from the checked geometry | A collision proof would cover the wrong path | `same_sweep` in planning and again in verification; a failure rejects |
| Timing fails after the geometry is accepted | Late re-planning of an action | Return to G3 with the reason, capped retries; report count. The cube run had no timing rejections |
| Deposits change with piece boundaries | Material hash and prefix journals no longer match old runs | New reference hash; coverage comparison in G2; old runs stay historical |
| Longer pieces subdivide more in the continuous checker | Smaller speedup than the 7x piece ratio | Measure evaluations in G0 and G9 before claiming a number |
| Removing the adaptation-time collision check | Failures surface later, after more work | Keep only if late rejections do not rise on the new-object reference |
| Existing library holds timed records only | First geometry-first cube run adapts every action | Expected one-time cost; publish geometric records from the first verified run |
| Two-arm coordination needs a shared clock | Geometric mode cannot schedule concurrent arms | Single-arm only; the timed two-arm branch is left in place |
| Large uncommitted working tree | No rollback point | Commit before G0 |

## Acceptance and limits

Ship when the cube still passes full-robot verification with equal material
coverage, the new-object reference completes, and the guard and blocked-material
rejections hold. Report time to a geometric chain and total time separately, and
report adaptation failures rather than hiding them behind fallback. No change to
margins, tracking limits or orientation bounds. No diffusion training, GPU
backend, new order search or physical execution is part of this plan.

```bash
.venv/bin/pytest -q
tools/node/bin/node --test web/tests/*.test.mjs
.venv/bin/ruff check src tests
git diff --check
tools/node/bin/node web/scripts/check-playback.mjs NEW_VERIFIED_RUN_ID
```
