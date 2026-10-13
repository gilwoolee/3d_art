# WirePrint + WIR3D: implementation plan

Updated 2026-10-12. One Flexiv 7-DoF arm, one pen, one indexed turntable.
C1–C6 are complete. Planning, final verification and viewer playback are separate steps.

## Goal and fixed decisions

Build from the solid STL to a complete sculpture, then return arm/table home.
Use WirePrint contours and whole connected zigzags for the framework, followed
by attached WIR3D artwork. Build and cool the entire framework before the art.

- Single arm; objects including strand thickness fit within 15 cm per axis.
- Straight flange-centered pen mount, 225 mm flange-to-tip; provisional tapered
  nose, footprint riser and closer placement. Preserve the bound run/profile.
- Hold one full TCP orientation throughout each action, including between
  samples. Current pen is upright; framework descents are at most 27° from
  horizontal. Pen-off travel may reorient.
- Table indices: 0/90/180/270°. Before turning, stop feed, withdraw/park, satisfy
  cooling, check the quarter-turn sweep and allow settling. Table is stationary
  during drawing; coordinated arm/table motion is out of scope.
- Preserve 1 mm clearance, 0.3 mm source tracking, 0.003 rad orientation bounds,
  joint limits, progressive support/material checks and source coverage.
- Drawing ceiling: 30 mm/s under joint limits and explicit feed/cooling holds.
  Count elapsed drawing/travel toward readiness; avoid duplicate cooling waits.
- Keep connected curves whole. Supported cuts are a documented fallback for a
  demonstrated feasibility problem; a cut creates no support or cooled anchor.
  Support/design changes require a new graph. Do not silently drop artwork.

## Parallel Phase 1 planning within one cube

```text
Solid STL -> WirePrint framework (+ attached WIR3D curves)
                         |
                Support-valid action order
                         |
        PARALLEL: retrieve/adapt cached local drawing paths
                         |
        Exact predicted deposits -> per-action material prefixes
                         |
        PARALLEL: pen checks + approach/withdrawal + connecting travel
                         |
        Failure -> lazy alternative -> invalidate/recheck dependents
                         |
        Save exact schedule (planning command ends)
                         |
        Separate full-robot verification + process/support/home audits
                         |
        Viewer loads saved plan + matching verification -> Play
```

P1–P4 and P5 assembly/export are implemented in `planning/parallel_build.py`.
Later actions can use earlier proposed deposits before their checks finish;
results are bound to the exact predecessor geometry and candidate/context.
Each action also checks its own growing material. Alternatives are evaluated
when needed; every selected motion must pass its applicable collision checks.

The build command now ends at `FULL_BUILD_PLANNED`. Export writes compact viewer
JSON with exact clocks/feed events; it does not repeat source/process audits,
build collision-worker journals, write computational sidecars or run final
verification. The separate `planning.cell_replay` command checks the full robot
and updates the saved verdict. New schedules become playable only after a matching
`CERTIFIED_GEOMETRIC_REPLAY` result. The viewer animates saved data and never plans
or verifies. Existing verified artifacts and diagnostic prefixes remain intact.

- [x] Separate planning/export from final verification; no build-mode switch.
- [x] Keep full-robot verification mandatory before new full-build playback.
- [x] Validate exact cube export, separate verification and browser playback.

`runs/cube_modular_playback_v1` preserves all 39 actions, 111,584 intervals and
the original feed events/timing. Minimal export took **6.48 s**; the separate
16-worker raw full-robot verification took **216.14 s** and passed with identical
142,102-capsule final geometry. These are saved-schedule measurements, not a new
planning benchmark. Browser checks withheld Play without the matching verdict,
then started playback and sought to the full horizon without backend requests
or changing either artifact. Receipts: `modular_playback_receipt.json` and
`modular_browser_receipt.json` in that run. The API test was subsequently removed
at the user's request; the routine suite is now 24 Python + four browser tests.

The current parallel path keeps one support-valid order and searches motion
alternatives within it. General backward order search exists separately;
parallel order-search completeness is not claimed. Workers currently use CPUs.

### Measured outcomes

- **Current cube:** `runs/compute_cleanup_cube_high_v3_w16`, 39 drawing
  sequences, raw full-robot `CERTIFIED_GEOMETRIC_REPLAY`, 111,584 intervals,
  142,102 capsules and arm/table home. Cache supplied 35 retained clocks and
  four adapted seeds; no table turns or cold proposals.
- **Current compute:** 16 CPU workers, **7.5 min**, observed peak tree PSS
  **19.2 GiB**. Setup 8.5 s, planning 185.5 s, assembly 2.1 s, export/replay
  249.7 s. Modeled printing is separately **32.0 min**.
- **Matched scaling:** 4/8/12/16 workers took 16.0/10.1/8.7/7.5 min. Sixteen
  saves 53% versus four (2.13×). Source/profile/cache hashes remained unchanged;
  every run produced the identical plan and passed fresh full-robot replay.
- **Checks:** 25 Python + four browser tests, Ruff and diff checks pass. Real
  obstruction rejection, local cache-free proposal/full-robot check and full
  browser playback passed. Recovery testing was dropped by the user.

Detailed timings, historical measurements and receipts:
[parallel cube results](2026_1012_parallel_cube_results.md).

## Trajectory reuse: two implementation phases

**Phase 1 is the default:** cached whole paths and self/static bounds are reused
only under matching context or conservative adaptation bounds. Changed material
and process conditions still require current checks; cache misses plan the same
whole action. Active library: `runs/motion_library_phase1_upright27_v1`.

New objects are planned geometry first: a collision-free untimed path for the
whole object, timed once at the end. Status and results:
[geometry-first planning plan](2026_1012_geometry_first_planning_plan.md). It
supersedes the [cached swept-volume plan](2026_1012_cached_swept_volume_plan.md),
which is not scheduled.

**Phase 2 exists as a research pilot:** dataset generation and one shared
current-profile diffusion model provide proposals through the same verifier.
It is deferred for this compute cleanup. Small local tests do not establish a
full-build speedup or general learned planning quality.

## Verification efficiency: shared Phase 1 and Phase 2 checklist

V1–V6 are implemented: measured/raw comparisons, exact-path invariant proofs,
conservative self/static bounds, local material queries, cache/learned integration
and rejection tests. Pen-only proofs cannot certify full-robot clearance.
Keep raw verification as the reference; accelerated checks must preserve bounds,
progressive deposits and scoped nozzle contact. Detailed evidence is in
[HANDOFF.md](HANDOFF.md) and [clearance design](2026_1011_precomputed_clearance_table_design.md).

## Compute and data cleanup

**C1–C6 are implemented and validated.** Reduce repeated work and data duplication while preserving
exact trajectories and acceptance. Use fresh outputs and retain original run
receipts. No historical cache/run deletion is included.

Before cleanup: plan JSON 358.1 MB (185.3 MB compact); cache JSON 1.8 GB; raw replay
contact geometry setup 4,057/6,683 worker-seconds (~61%). Owners below are under
`src/pen_art/planning/`.

- [x] **C1: Local contact geometry in final replay.** Reuse conservative clipping
  without skipping fresh full arm/tool checks. Separate geometry optimization
  from proof reuse; compare clear, blocked and near-margin cases to raw replay.
  Owners: `scene.py`, `transit.py`, `cell_planner.py`.
- [x] **C2: Compact JSON and binary computational clocks.** Remove indentation
  first; then use packed arrays with a hashed manifest. Preserve every numeric
  value, polynomial, interval and process event; keep supported viewer JSON
  bound to the same plan. Owners: `full_build_export.py`, `clock_store.py`, `cell_replay.py`.
- [x] **C3: Load cache records on demand.** Use a small compatibility index and
  selected trajectories/proofs; historical material scenes remain in evidence
  records. Preserve original hashes, provenance, frozen inputs and stale/corrupt
  rejection. Owners: `motion_library.py`, `motion_runtime.py`.
- [x] **C4: Reduce worker/material copies.** Use shared immutable capsule arrays
  with per-job prefixes, bounded journal caches and isolated mutable FCL state.
  Release unused proposal data before replay; verify geometry hashes, table
  transforms and replacement invalidation. Owners: `parallel_build.py`, `material_journal.py`, `cell_replay.py`.
- [x] **C5: Reuse local drawing proofs.** Bind source/orientation/speed proofs to
  exact polynomials, durations and physical context so adding travel or shifting
  schedule time can retain them. Changed paths/context invalidate reuse; check
  current material and travel freshly. Owners: `path_certificates.py`, `section_build.py`, `cell_replay.py`.
- [x] **C6: Validate and benchmark end to end.** Complete the real blocked and
  cold-fallback checks and full browser replay. Compare one/two/four workers
  sequentially with matched frozen inputs and cache conditions; follow with
  requested 4/8/12/16 scaling under the hardened storage revision. Report actual
  total PSS peak, setup, planning, export/loading, parsing and full replay time.
  Each run must pass independent full acceptance before any speedup claim.

Validation: the 29-test critical suite and Ruff pass (310 Python tests
passed before the requested test reduction). A saved late cube action matches
fresh raw full-robot acceptance/deposits: clear 235→19 s, blocked 116→11 s.
The complete 111,584-interval clock round-trips exactly: viewer JSON 358→185 MB,
packed payload+manifest 100 MB. Cache index 1.3 MB instead of loading 1.93 GB;
reload 14 ms. These are local/storage checks, not full-build speedup claims.
Receipts: `runs/compute_cleanup_c1_v1.json`,
`runs/compute_cleanup_clock_storage_v1/receipt.json`,
`runs/compute_cleanup_cache_index_v1.json`.

The original 1/2/4 comparison passed in 52.7/28.2/16.0 min before the final
codec binding and replay-journal hardening:
`runs/compute_cleanup_cube_scaling_v2_benchmark.json`. All three exact schedules
were re-exported and freshly verified under the corrected codec:
`runs/compute_cleanup_cube_codec_v2_codec.json`.

The subsequent 4/8/12/16 comparison passed under one current revision:
`runs/compute_cleanup_cube_high_v3_benchmark.json`; the queue receipt verifies
unchanged inputs. Peak observed simultaneous tree PSS was 6.20/10.97/15.23/19.21
GiB, sampled every two seconds. Sixteen is fastest among the tested counts.
Original acceptance: `runs/compute_cleanup_completion_v2.json` (`PASSED`).
The browser-tested codec plan has the same SHA as all four current builds.
A matched 32-worker trial also passed in 454.5 s (7.6 min), peak PSS 36.86 GiB,
with identical inputs/plan: `runs/compute_cleanup_cube_32_v4_benchmark.json`.
It offers no meaningful total gain over 16: planning improved but replay slowed.
Keep 16 for this workload. No benchmark or source freeze remains.
GPU investigation was stopped for handoff;
[findings](2026_1012_gpu_validation_findings.md) are separate from C1–C6.

## Remaining acceptance

- [x] Publish/browser-check the verified complete cube on port 8765. Full 10×
  playback passed; [browser receipt](../runs/cube_parallel_trial_v2/viewer_browser_browser_replay.json).
- [ ] Complete and independently replay the 55-action bottle framework; test
  cold/precomputed/warm reuse and a nearby geometry variation with provenance.
- [ ] Verify identity of the WirePrint website bottle STL or explicitly label
  our procedural substitute. Compare the 14-minute target using achieved speed,
  wire length and all motion/hold/feed/table overhead, separately from compute.
- [ ] Complete bottle framework plus all 29 selected WIR3D curves and the cooled
  framework-to-art transition; this is separate from the framework speed test.
- [ ] Resolve the declared Palace tool-access conflict and complete its framework
  plus all 18 selected artistic sources. Preserve geometry/clearance obligations.
- [ ] Publish full accepted examples with cache provenance, timing breakdown and
  download progress. Physical printing needs measured tool/material calibration.

## Validation and operation

Retain graph/profile/source bindings, exact clocks, replay reports and browser
receipts. Timeout means unresolved, not collision. Use fresh processes after
source changes; old resumable job results bind the implementation hash. Consult
[HANDOFF.md](HANDOFF.md) for process/viewer state; do not blindly launch the old
follow-up script or its concurrent scaling sequence.

```bash
.venv/bin/pytest -q
.venv/bin/ruff check src tests
git diff --check
# Plan and save clocks in a fresh run:
.venv/bin/python -m pen_art.planning.section_build \
  --cell-source-run runs/cube_nozzle27_phase1_v1_source \
  --profile runs/cube_nozzle27_phase1_v1/profile.yaml \
  --run runs/new_cube --progression-only --workers 16 --maximum-seconds 14400
# Verify separately; this enables playback. Play never invokes this command:
.venv/bin/python -m pen_art.planning.cell_replay \
  --run runs/new_cube --config runs/cube_nozzle27_phase1_v1/profile.yaml \
  --plan-name offline_dual_arm_plan --workers 16 --maximum-seconds 1200
# The benchmark runs those two commands separately and reports their combined time:
.venv/bin/python scripts/benchmark_parallel_build.py \
  --source-run runs/cube_nozzle27_phase1_v1_source \
  --profile runs/cube_nozzle27_phase1_v1/profile.yaml \
  --output runs/new_cube_scaling --workers 1 2 4
# After publishing the full cube, check its exact run ID:
env -u DISPLAY tools/node/bin/node web/scripts/check-replay.mjs cube_parallel_trial_v2
```

Research: [Caelan extrusion](https://arxiv.org/abs/2002.02360),
[PDDLStream](https://arxiv.org/abs/1802.08705),
[GLS](https://ojs.aaai.org/index.php/ICAPS/article/view/3543),
[EIRM*](https://arxiv.org/abs/2205.08480),
[WirePrint](https://hcie.csail.mit.edu/research/wireprint/wireprint.html),
[WIR3D](https://threedle.github.io/wir3d/).
Historical designs: [algorithm review](archive/2026_1011_caelan_build_order_plan.md),
[earlier implementation](archive/2026_1010_mesh_to_robot_pen_plan.md).
