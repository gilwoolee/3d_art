# Cached swept volumes for changing printed material

2026-10-12. **Status: planned; not implemented or benchmarked.**

Finish the missing collision-reuse part of Phase 1: cache conservative volumes
swept by exact robot/tool trajectories, then test those volumes against the
current printed material. This should reduce collision work across different
objects without needing diffusion. Use CPUs first and retain the existing
checker for every query the cache cannot prove clear.
Keep the current single Flexiv 7-DoF arm, whole connected drawing actions,
fixed TCP orientation within DRAW and stationary 0/90/180/270-degree table
indices. This preserves modeled clearance, not physical fabrication qualification.

## Scope and current state

Trajectory reuse, self/static clearance bounds, exact-path certificates and
material-prefix journals already exist. `CachedCollisionModel.material_distances`
still delegates growing-material queries to the underlying geometry checker;
there is no reusable swept-volume/occupancy test. Planning uses `pen_candidate`
scope; separate final verification restores `full_robot` scope and currently
defaults to raw checks that bypass proof reuse.

Keep the modular workflow:

```text
STL -> WirePrint actions (+ WIR3D) -> supported order
     -> Phase 1 cached/adapted joint trajectories
     -> current material checks + connecting travel -> save complete schedule
     -> separate full-robot verification -> verified viewer playback

During planning and verification:
     exact path + robot/tool context -> cached continuous swept covers
     current material prefix        -> conservative occupied cells
                                     |
                          empty intersection: certified material clearance
                          overlap/miss: existing exact checker
```

This caches collision geometry, not a previous object's clear/blocked answer.
Full-robot self/static certificates and current material clearance are separate
obligations. A pen-only footprint can never certify full-robot clearance.

## Implementation checklist

- [ ] **S1 — Measure the collision workload and freeze a baseline.**
  Owners: `planning/collision.py`, `planning/cell_planner.py`,
  `planning/cell_replay.py`, `scripts/benchmark_parallel_build.py`.
  Aggregate actual worker counts/time for self, static, ordinary material and
  contact-tip queries; coordinator counters alone do not describe worker use.
  Record full geometry, margin, source/profile/cache hashes and exact clocks.
  Use 16 workers sequentially. The saved cube reference has 39 drawing actions,
  111,584 intervals and 142,102 final capsules; its separate verification took
  216.14 s, including 94.87 s collision replay. These are prior measurements,
  not a prediction of this change's speedup.

- [ ] **S2 — Construct conservative continuous robot/tool covers.**
  Owners: new `planning/swept_volume_cache.py`, `planning/collision.py`,
  `planning/clock_path.py`, `planning/smooth_audit.py`.
  For each local path interval, bound each body's geometry at an anchor pose,
  then expand by `motion_radii[body] @ joint_deviation` over the entire interval.
  Reuse the polynomial deviation bounds consumed by `check_bounded_path`;
  endpoints alone cannot cover an interior excursion. Use analytic primitive
  bounds or bounds over all imported collision vertices, including body-local
  and world transforms. Missing/unproved geometry bounds force fallback.
  Include every required robot and tool body. Apply the 1 mm margin once to the
  robot cover, plus the checker's numerical allowance; round voxel bounds outward.
  Subdivide loose covers where worthwhile, with bounded generation effort.
  Start with 4 mm cells and compare 2 mm cells; cell size is not a relaxed
  clearance requirement. Prove cover containment before admitting cache entries.

- [ ] **S3 — Persist covers by exact geometry and collision context.**
  Owners: new `planning/swept_volume_cache.py`, existing
  `planning/path_certificates.py`, `planning/motion_library.py`.
  Bind normalized joint polynomials/interpolation and local interval bounds,
  robot/tool geometry, base/TCP transforms, collision scope, grid definition,
  margin, numerical allowances and generator version. Do not key by object ID.
  Self/static proofs additionally bind collision-pair rules and static geometry.
  Store sparse packed cell blocks with checksummed manifests, atomic publication,
  read-only shared arrays and a bounded worker LRU. Corruption or changed context
  is a cache miss, never permission to skip a check.
  Absolute schedule shifts may retain geometric covers; timing proofs retain
  their duration bindings. Adapted joints need new covers unless a separately
  proved envelope contains the adaptation. First release regenerates changed
  pieces and reuses identical pieces; no arbitrary template translation or tilt.

- [ ] **S4 — Maintain conservative occupancy for the exact material prefix.**
  Owners: `planning/scene.py`, `planning/material_journal.py`,
  `planning/parallel_build.py`, new `planning/swept_volume_cache.py`.
  Rasterize the existing conservative deposit capsules, including their radii;
  marking centerlines alone under-covers material. Derive occupancy alongside
  exact capsules from the same journal and geometry-prefix identity. Later
  workers use their bound predecessor prefix, not whatever another worker has
  finished. Candidate replacement must invalidate/rebuild dependent occupancy.
  Preserve all older material and add the current interval's own growing-wire
  enclosure before querying, matching `_validate_interval`'s existing order.
  Fix the grid in world coordinates. Cover the material envelope for all four
  stationary table indices; rebuild/re-rasterize after indexed turns rather
  than rotating voxel centers approximately. Out-of-grid material forces exact
  fallback; clipping a robot cover is allowed only when all queried material is
  contained in the declared grid region. Keep moving table-sweep checks exact.

- [ ] **S5 — Use positive clearance proofs in planning and final verification.**
  Owners: `planning/collision.py`, `planning/cell_planner.py`,
  `planning/motion_runtime.py`, `planning/cell_replay.py`.
  Integrate at the material-query boundary without bypassing deposition,
  tracking, fixed orientation, timing, support or home checks. An empty
  intersection certifies only the covered bodies' material clearance; self and
  static clearance still need valid scoped proofs or fresh checks. Overlap is
  inconclusive, not a collision verdict: refine or run the existing checker.
  Initially accelerate ordinary body/material queries and leave virtual-tip
  contact checks exact through `transit.contact_validator` and
  `scene.ContactMaterialView`. Never erase a contact region for the pen body or
  arm, or exempt an entire joined strand. Connecting travel may use admitted
  covers too; uncached travel falls back normally.
  After S6 passes, make certificate-backed verification the normal separate
  final step; retain `verification_raw=True` as a diagnostic reference. Do not
  run a complete raw pass after every accelerated verification, which would
  eliminate the benefit. Keep full-robot verdicts, plan hashes and viewer gating
  unchanged; add proof-hit/fallback statistics, with no viewer mode switch.

- [ ] **S6 — Validate changing scenes, then measure the whole workflow.**
  Owners: existing `tests/test_local_material_validation.py`,
  `tests/test_material_journal.py`, `tests/test_parallel_build.py`,
  `tests/test_path_certificates.py`; the new cache module and benchmark script.
  Extend current critical contracts; keep the routine suite below 30 tests.
  Check continuous interior excursions, 0.999/1.000/1.001 mm clearance, negative
  voxel coordinates/boundaries, old wire and newly added blockers, own growing
  wire, intended tip contact versus body contact, replacement/shorter prefixes,
  quarter-turn transforms, out-of-grid material, corrupt entries and changed
  joints/tool/base/static geometry. Every accelerated CLEAR must be supported
  by its containment proof and agree with fresh exact checking on these fixtures.
  On ambiguous fixtures, fallback may remain unresolved; a coarse overlap must
  never cause an unsupported collision rejection or false clearance acceptance.
  Run raw, existing acceleration without swept covers, and acceleration with
  covers on the same saved cube in fresh runs. Measure cold cover generation,
  warm loads, occupancy updates, hit/fallback rates by body and query kind,
  collision/full-verification wall time, disk size and simultaneous tree PSS.
  Repeat with a changed material prefix using unchanged clocks to demonstrate
  cross-scene reuse, then one regenerated nearby cube variation with adapted
  trajectories to expose regeneration cost. These are cube variants, not a
  claim of many independent objects. Measure one fresh full planning/export/
  verification run and browser-check its separately verified result.

## Acceptance and limits

Ship the fast path only after continuous containment, context/prefix invalidation
and contact behavior pass the checks above. Preserve final geometry and full
acceptance for the cube; viewer playback must start no backend work. Report
warm savings together with cold precomputation and amortization across scenes.
If conservative covers cause mostly fallback, report that result and improve
cover tightness/storage before expanding the cache; do not weaken margins.

Collision-only savings address about 95 s of the estimated 419 s cube workflow:
even eliminating that whole stage would cap total speedup near 1.29x on this
measurement. Source/path/process audits need their own valid reuse to gain more.
No diffusion training, GPU/FPGA backend, dense task-space atlas, physical robot
execution or new geometry/order-search algorithm is part of this plan.

Validation commands, with fresh output directories for measured runs:

```bash
.venv/bin/pytest -q
tools/node/bin/node --test web/tests/*.test.mjs
.venv/bin/ruff check src tests
git diff --check
tools/node/bin/node web/scripts/check-playback.mjs NEW_VERIFIED_RUN_ID
```

Sources: [Motion Planning on a Chip](https://www.roboticsproceedings.org/rss12/p04.pdf),
[programmable successor](https://cs.brown.edu/people/gdk/pubs/planchip_programmable.pdf),
[earlier clearance-table design](2026_1011_precomputed_clearance_table_design.md),
[current cube measurements](2026_1012_parallel_cube_results.md).
The paper motivates precomputing robot motion independently of new obstacles;
the conservative CPU covers, exact fallback and scoped contact handling above
are our proposed implementation, not the paper's FPGA implementation.
