# Pen Art Tutorial Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn the repository into a tutorial that takes a learner from a
Meshy-style mesh to a verified single-arm print plan, with strokes chained as
much as possible, in seven commands.

**Architecture:** Cut the codebase down to the geometry-first parallel
planner, then reshape it: chaining in the planner's own sampler, a fixed
build plate instead of the turntable, two config files, and a `pen-art` CLI
whose stages (`cell`, `frame`, `detail`, `plan-geometry`, `plan-timing`,
`verify-full-robot`, `view`) each read the previous stage's output from one
run folder. Every task ends by planning the ghost and checking the verdict
and the lift-off count.

**Tech Stack:** Python 3.12 (numpy, scipy, trimesh, python-fcl, toppra,
FastAPI), TypeScript + three.js viewer built with Vite, pytest, ruff.

**Spec:** `docs/superpowers/specs/2026-10-18-educational-tutorial-design.md`

## Global Constraints

- Single arm (Flexiv Rizon4, 7 joints); geometry-first planning only; no
  turntable; no motion library.
- Chain strokes as much as possible; lift off only when chaining fails, and
  report every lift-off with its reason next to the minimum number of
  trails (Euler's rule after connector wire is added).
- Frame wire at most 10 m per object; detail about 2 m and at most 15 extra
  minutes of print time.
- One pen orientation (tilt and lean direction) per stroke: the pen never
  twists wire while depositing.
- Frame settings (layer height, zigzag spacing, base grid spacing) are
  physical sizes; they do not scale with the build volume.
- The object stands on a printer build plate on the table; default
  256 × 256 mm.
- Planning uses 24 workers; long runs go in tmux.
- Exactly 5 tests in the repository.
- Commits: no `Co-Authored-By:` trailers. On this machine commit with
  `git -c user.name="Gilwoo Lee" -c user.email="gilwoo301@gmail.com" commit`.
- Unset `PYTHONPATH` before running anything (this machine's shell sets it
  for ROS, and pytest then loads ROS plugins and fails).
- Never `pkill -f` a pattern that can match your own shell; kill by PID.
- Headless Chrome over SSH needs `env -u DISPLAY`.
- Everything is simulation; report plainly what passed, failed and was not
  measured.

## Review Focus

1. **A Meshy mesh with a different up axis or scale** (Z-up export, a model
   1000× too large): `pen-art frame` takes `--up {y,z}` and scales the
   longest side to the build volume; a wrong `--up` gives a lying-down
   frame, which the README's step 1 tells the learner to check in the
   viewer. Owned by Task 8 (acceptance step with `--up z`).
2. **Rerunning a stage over an existing run folder.** Today
   `frame_source.py` silently keeps stale work when `runs/geomfirst_NAME`
   exists. Every `pen-art` stage either rebuilds its own output folder from
   scratch or refuses with a message naming the folder; it never mixes old
   and new results. Owned by Task 8.
3. **Running stages out of order** (`plan-timing` before `plan-geometry`,
   `verify-full-robot` before `plan-timing`, `plan-geometry` after the
   frame changed): the stage stops with a message naming the stage to run
   first. Owned by Task 8.
4. **Detail with nothing drawable** (every detail stroke dropped, or no
   crease lines found): planning continues with the frame alone and reports
   0 m of detail. Owned by Task 8.
5. **A smaller build volume (8 cm)**: frame, plan and verification work with
   the same `frame.yaml`; the plate position default still reaches. Owned by
   Task 10.

The 5-test limit means these are acceptance checks (commands with expected
output) in their owning tasks, not new pytest tests.

---

### Task 0: Working environment

**Files:** none changed.

- [ ] **Step 1: Set up the repository on this machine**

Run: `unset PYTHONPATH; scripts/setup_machine.sh`
Expected: ends with `Done.`; `.venv/`, `external/flexiv_description/`,
`research/planning_cell/robot.urdf`, `tools/node/` and `web/dist/` exist.

- [ ] **Step 2: Record the baseline ghost frame (with today's code, no library)**

Run (in tmux):
```bash
unset PYTHONPATH
.venv/bin/python scripts/frame_source.py ghost objs/cute_ghost/cute_ghost.glb --up y
TAG=base scripts/plan_frame.sh ghost runs/geomfirst_ghost_source runs/geomfirst_profiles/ghost.yaml
```
Expected: last line of `runs/geomfirst_base_progress.txt` ends in
`FULL_BUILD_VERIFIED`. Count lift-offs as the number of `TRAVEL` edges in
`runs/geomfirst_base_ghost_w24/offline_dual_arm_plan.json`: expected 44 (no
library present). Write the numbers in the task log; they are the baseline
for Tasks 2 and 3.

---

### Task 1: Remove unused code, the motion library and non-geometry-first modes

**Files:**
- Delete: the 50 modules never imported by a pipeline run (traced from
  bytecode loads on 2026-10-18), all under `src/pen_art/`:
  `abstraction/{comparison,contours,geometric,mesh_contours,refine,remote,selection,sparse,wir3d_runner}.py`,
  `{backbone,build,construction,growth,learning_generation,pipeline,placement,recognition,trajectory,validation}.py`,
  `learning/` (whole package),
  `planning/{allocation_motion,allocation_repair,base_first_run,build_metrics,build_order_preflight,cell_allocation,cell_comparison,collision_report,constant_axis,construction_order,coupled,curobo_adapter,endpoint_support,forward_state,motion_cache_precompute,mounting,orientation_probe,partial_export,partition,resource_schedule,run_cell,sequence,single_arm_build,stroke_restarts}.py`.
  If a remaining module turns out to import one of them, keep the import
  target only if the import is on the geometry-first path; otherwise
  delete the importing code.
- Modify: `pyproject.toml` (remove the `pen-art` and `pen-art-serve` entry
  points, which point at deleted or replaced code, and the `learning`
  extra; Task 8 adds the new `pen-art`).
- Delete: `src/pen_art/planning/motion_library.py`, `motion_runtime.py`.
- Create: `src/pen_art/planning/collision_context.py` holding what
  `cell_planner.py` imports from them (`collision_context_hash`, `digest`,
  `obstacle_context`, `VALIDATOR_VERSION`, `static_shapes`,
  `MaterialOnlyCollisionModel`), moved unchanged.
- Modify: `section_build.py` (keep only the `--workers` geometry-first path
  of `run_build`, the `prepare_*` used by `frame_source.py`, and the helpers
  listed as needed; delete `forward`, `backward`, checkpoints, `--fixture`,
  `--mesh`, `--source-run`, `--wir3d-budget`, `--regression-only`, `--gpu`,
  `--diffusion-checkpoint`, `--learning-device`, `--motion-library`,
  `--inject-rejection`; geometry-first becomes the only behaviour, not a
  flag), `parallel_build.py` (cold mode only: delete the cache branch of
  `_propose`, `cache_exhausted`, `skip`, `experience` uses and the
  `cache`/`retained_clock` counts), `indexed_build.py` (keep `at_index` and
  the `IndexedMotions` members the parallel path uses: `drawings`,
  `structure_admissible`, `transition`, `table`, `round`; delete the rest),
  `regression.py` (keep `greedy_forward_order`), `replay_checkpoint.py`
  (keep `digest`, `IMPLEMENTATION`), `cell_replay.py` (drop resume and
  `partial_export`), `transit.py` (drop the `coupled` import).
- Delete tests: all but the five listed in Step 1; delete `web/tests/`.
- Modify: `tests/README.md`, `AGENTS.md` (test count rule: "Exactly 5
  tests"; checks list without the browser tests).

**Interfaces:**
- Produces: `pen_art.planning.collision_context` with the moved names;
  `VALIDATOR_VERSION` keeps its value so saved certificate hashes do not
  change.
- `section_build` CLI after this task: `--cell-source-run --run --profile
  --maximum-seconds --action-budget --workers --progression-only` plus the
  geometry-first tuning options (`--merge-deposits-um`, `--path-pieces`,
  `--retiming-grid`, `--retiming-solver`, `--chain-strokes`) at their
  geometry-first defaults.

- [ ] **Step 1: Keep exactly these five tests**

```
tests/test_wireprint_nozzle.py::test_slice_height_follows_the_nozzle_on_a_cube_and_a_solidified_fork
tests/test_collision.py::test_continuous_check_finds_collision_between_clear_endpoints
tests/test_timing.py::test_timing_must_sweep_exactly_the_checked_geometric_path
tests/test_parallel_build.py::test_parallel_replay_reports_the_serial_rejection
tests/test_parallel_build.py::test_coordinator_repairs_blocked_material_and_drops_stale_results
```
The last one is rewritten in Task 2 for chaining; in this task delete its
cache-mode assertions (`skip == ["parent-0-0"]`, `chain_from`) so it tests
repair and stale results in cold mode. Delete every other test function and
emptied test file.

- [ ] **Step 2: Run the five tests on the unchanged code**

Run: `.venv/bin/pytest -q`
Expected: `5 passed`.

- [ ] **Step 3: Remove the modules, the library and the modes as listed under Files**

- [ ] **Step 4: Tests and lint**

Run: `.venv/bin/pytest -q && .venv/bin/ruff check src tests scripts`
Expected: `5 passed`, `All checks passed!`.

- [ ] **Step 5: Ghost frame end to end**

Run Task 0 Step 2 with `TAG=t1` (delete `runs/geomfirst_ghost*` first).
Expected: `FULL_BUILD_VERIFIED`, 44 `TRAVEL` edges and print time equal to
the baseline (the planner's own sampler was already the only source).

- [ ] **Step 6: Commit**

```bash
git add -A && git commit -m "Keep only the geometry-first planner: remove the motion library, other modes and unused code"
```

---

### Task 2: Chaining in the planner's own sampler

**Files:**
- Modify: `src/pen_art/planning/parallel_build.py` (`ForwardSchedule._may_chain`,
  `_derive_propose_spec`, `_propose` cold branch), `indexed_build.py`
  (`IndexedMotions.drawings` accepts the chain)
- Modify: `src/pen_art/planning/full_build_export.py` (lift-off report)
- Test: `tests/test_parallel_build.py`

**Interfaces:**
- Produces: `IndexedMotions.drawings(row, available, q, index, scene, *, chain=None)`
  where `chain = {"q": list[float], "table_index": int, "action": str}` is
  the previous stroke's final joint pose; when given, the first candidates
  are continuations that start exactly at `chain["q"]` with the previous
  stroke's pen orientation, and only then the unchained candidates.
- Produces: `ForwardSchedule` proposes the chained continuation for every
  action whose start node equals the previous action's end node, in cold
  mode, before any unchained candidate.
- Produces: the exported plan and `section_build.json` carry
  `"lift_offs": [{"before": action_id, "reason": str}]`, one entry per
  action that starts with an approach rather than a chained continuation;
  reasons are `"not continuous"` (start ≠ previous end),
  `"chain blocked: <failure status>"`, or `"first stroke"`.

- [ ] **Step 1: Rewrite the coordinator test to require chaining**

Rename `test_coordinator_repairs_blocked_material_and_drops_stale_results`
to `test_coordinator_chains_continuing_strokes_and_drops_stale_results`.
Keep its repair and stale-result assertions and add, on its fixture where
action B starts at action A's end node:
```python
spec = schedule.next_propose_spec(index_of("B"))
assert spec["chain_from"]["action"] == "A"
assert spec["mode"] == "cold"
assert report["lift_offs"] == [{"before": "A", "reason": "first stroke"}]
```

- [ ] **Step 2: Run it to make sure it fails**

Run: `.venv/bin/pytest -q tests/test_parallel_build.py`
Expected: FAIL (`chain_from` missing in cold mode).

- [ ] **Step 3: Implement chained continuation in cold mode and the lift-off report**

A chained candidate is accepted by the same checks as any other (collision,
structure, path accuracy); when it fails, record the failure as the
lift-off reason and fall back to the unchained candidates.

- [ ] **Step 4: Tests pass**

Run: `.venv/bin/pytest -q && .venv/bin/ruff check src tests scripts`
Expected: `5 passed`.

- [ ] **Step 5: Ghost frame end to end**

Run Task 0 Step 2 with `TAG=t2`.
Expected: `FULL_BUILD_VERIFIED`; `lift_offs` has at most 15 entries (the
library run had 15 travels on 41 strokes); print time about 11 min or less.
Record the number and the reasons.

- [ ] **Step 6: Commit**

```bash
git add -A && git commit -m "Chain strokes in the planner's own sampler; report every lift-off"
```

---

### Task 3: Fewest trails: connectors and Euler order

**Files:**
- Modify: `src/pen_art/abstraction/wireprint.py` (new `eulerize_frame`)
- Modify: `src/pen_art/planning/section_build.py:172,296` (call it after
  `connect_framework_islands`), `src/pen_art/planning/sections.py` (frame
  strokes follow the trail)
- Modify: `src/pen_art/planning/full_build_export.py` (report the minimum)
- Test: `tests/test_wireprint_nozzle.py` (assertions added to the existing
  test; the test count stays 5)

**Interfaces:**
- Produces: `eulerize_frame(graph: dict, budget_m: float, structure: dict) -> dict`
  in `wireprint.py`. It pairs the odd-degree nodes by minimum-weight perfect
  matching (`networkx.min_weight_matching`) and adds one connector edge per
  pair, `role: "framework"`, `origin: "euler_connector"`. Weights: on the bed
  (`z == 0`), the length of a path just inside the outline, offset by one
  strand diameter, which is also the connector's shape; above the bed, the
  straight distance, and the connector is kept only if
  `analyze_structure` still passes. A connector never lies on existing
  wire. It leaves the bottom start and top end odd, and stops adding when
  the wire budget would be exceeded. It records
  `graph["generation"]["eulerization"] = {"odd_nodes_before", "odd_nodes_after",
  "minimum_trails_before", "minimum_trails_after", "added_m",
  "left_odd": [{"node", "reason"}]}`.
- Produces: the construction plan's frame strokes follow an Euler trail
  (Hierholzer) of each band, built bottom to top so a member comes after
  the members that support it; consecutive frame strokes share an end
  node wherever the trail continues.
- Produces: `minimum_trails` (from `eulerization`) next to `lift_offs` in
  `section_build.json` and the exported plan.

- [ ] **Step 1: Add assertions to the frame generator test**

In `test_slice_height_follows_the_nozzle_on_a_cube_and_a_solidified_fork`,
after the existing cube checks:
```python
graph = eulerize_frame(graph, budget_m=10.0, structure=STRUCTURE)
odd = [n for n, d in topology(graph).degree() if d % 2]
assert len(odd) <= 2
info = graph["generation"]["eulerization"]
assert info["minimum_trails_after"] == 1
assert 0 < info["added_m"] < 0.5
assert not any(on_existing_wire(e, graph) for e in graph["edges"]
               if e["origin"] == "euler_connector")
```
(`STRUCTURE` from `configs/fabrication.yaml`'s `structure`; `topology` and
`on_existing_wire` are small helpers in the test file.)

- [ ] **Step 2: Run it to make sure it fails**

Run: `.venv/bin/pytest -q tests/test_wireprint_nozzle.py`
Expected: FAIL (`eulerize_frame` not defined).

- [ ] **Step 3: Implement `eulerize_frame`, the trail order and the report**

- [ ] **Step 4: Tests pass**

Run: `.venv/bin/pytest -q && .venv/bin/ruff check src tests scripts`
Expected: `5 passed`.

- [ ] **Step 5: Ghost frame end to end**

Regenerate the ghost frame, then run Task 0 Step 2 with `TAG=t3`.
Expected: `eulerization` shows 18 odd nodes before, at most 2 after,
`minimum_trails_after` 1, added wire under 0.5 m, frame still within 10 m;
plan `FULL_BUILD_VERIFIED` with fewer lift-offs than Task 2. Record the
lift-offs and their reasons.

- [ ] **Step 6: Commit**

```bash
git add -A && git commit -m "Fewest trails: connector wire pairs odd nodes; frame order follows the Euler trail"
```

---

### Task 4: Single-arm naming

**Files:**
- Modify: `cell.py`, `cell_planner.py`, `cell_replay.py`, `full_build_export.py`,
  `dual_arm.py` → rename to `src/pen_art/planning/schedule.py`, and every
  importer; `scripts/plan_detail.py`, `scripts/benchmark_parallel_build.py`,
  `web/src/simple.ts`, `src/pen_art/server.py`, `tests/test_parallel_build.py`
- Delete: `configs/dual_arm.yaml` (move `assumed_cooling_s` and
  `maximum_subdivision_depth` into `configs/fabrication.yaml` under a new
  `planning:` key; Task 7 moves it into `frame.yaml`)

**Interfaces:**
- Produces: run artifacts `plan.json` (was `offline_dual_arm_plan.json`),
  `plan_config.json` (was `offline_dual_arm_config.json`),
  `verification.json` (was `offline_dual_arm_validation.json`),
  `verification_journals/`; artifact name `"plan"` (was `"dual_arm_plan"`);
  `cell_replay --plan-name` defaults to `plan` and `--config` has no default.
- Produces: the single arm's id is `"arm"`; pair checks
  (`check_pair_linear`, `check_pair_paths`, the `product` over arm choices)
  and arm-count guards are gone; `ScheduleState.configurations` holds one
  configuration.

- [ ] **Step 1: Rename and simplify as listed**

- [ ] **Step 2: Tests, lint, viewer build**

Run: `.venv/bin/pytest -q && .venv/bin/ruff check src tests scripts && (cd web && PATH=$PWD/../tools/node/bin:$PATH npm run build)`
Expected: `5 passed`, lint clean, Vite `built in`.

- [ ] **Step 3: Ghost frame end to end**

Run Task 0 Step 2 with `TAG=t4`.
Expected: `FULL_BUILD_VERIFIED`, `runs/geomfirst_t4_ghost_w24/plan.json`
exists, lift-offs and print time equal to Task 3's.

- [ ] **Step 4: Commit**

```bash
git add -A && git commit -m "Single arm only: rename dual-arm artifacts and remove pair checks"
```

---

### Task 5: Fixed build plate instead of the turntable

**Files:**
- Delete: `src/pen_art/planning/turntable.py`
- Modify: `indexed_build.py` (no `table_index` in candidates; `transition`
  without `INDEX_TURN`/`TABLE_SETTLE`), `parallel_build.py` (no
  `(first + ordinal) % 4`; `MaterialPrefix` without turns), `cell_planner.py`
  (one fixed `world_from_build`), `clock_path.py`, `clock_store.py`,
  `scene.py`, `process_clock.py`, `cell_replay.py`,
  `full_build_export.py`, `cell.py` (obstacles: table plus a plate box),
  `section_build.py` (`--workers` no longer requires a turntable),
  `scripts/detail_tours.py` (lean direction instead of turntable sector;
  no `SECONDS_PER_TURN`), `scripts/plan_detail.py` (no `group_faces` by
  turntable), `cell_motion.py`, `web/src/{timing,viewer,simple}.ts`
- Modify: `configs/profiles/frame_template.yaml`,
  `configs/cell/one_flexiv_straight.yaml` (plate instead of platform,
  riser and turntable)

**Interfaces:**
- Produces: `pen_rotation(arm, world_from_build, roll=0.0, tilt_deg=0.0, lean_azimuth_deg=None)`
  and `fixed_rotation_families(..., lean_azimuth_deg=None)` in
  `cell_motion.py`: with `lean_azimuth_deg`, the pen body leans toward that
  world azimuth (degrees); without it, toward the robot as today.
- Produces: detail stroke rows carry `"pen_lean_azimuth_deg": float`
  (the outward direction of the surface they draw on) instead of
  `"turntable_index"`; `IndexedMotions.drawings` passes it through.
- Produces: profile `environment` keys `table_z_m`, `table_size_m`,
  `plate_size_m: [0.256, 0.256]`, `plate_thickness_m`, `clearance_m`;
  `world_from_build` puts the build origin on the plate's top surface.
  `platform_*` and `build_riser` are gone.
- Produces: a stroke whose sampled path fails the path-accuracy audit
  (`deposition_to_source_bound_m` over `position_limit_m`) is retried with a
  finer path (double the samples, up to two retries) and then with the
  other roll (`roll_angles` 0 and π) before it is reported blocked.

- [ ] **Step 1: Remove the turntable and add the plate and lean direction as listed**

- [ ] **Step 2: Tests, lint, viewer build** (as Task 4 Step 2)

Expected: `5 passed`. `test_parallel_replay_reports_the_serial_rejection`
already runs with `turntable=None`.

- [ ] **Step 3: Choose the default plate position by planning the ghost**

Plan the ghost frame (Task 0 Step 2, `TAG=t5`) with the plate centre at the
old build position (0.45, −0.15 m from the robot base, on the table). If
it is not `FULL_BUILD_VERIFIED`, move the plate toward the robot in 2 cm
steps (at most 5 tries) and keep the first position that verifies; record
the tries.
Expected: `FULL_BUILD_VERIFIED`, lift-offs no more than Task 3's.

- [ ] **Step 4: Ghost detail end to end**

```bash
PYTHONPATH=src .venv/bin/python scripts/detail_tours.py runs/geomfirst_ghost_source runs/geomfirst_ghost_detail_source --profile runs/geomfirst_profiles/ghost.yaml --check-with-planner
PYTHONPATH=src .venv/bin/python scripts/plan_detail.py runs/geomfirst_ghost_detail_source runs/geomfirst_profiles/ghost.yaml runs/geomfirst_t5_ghost_detail --frame-run runs/geomfirst_t5_ghost_w24 --detail 2 --minutes 15
```
Expected: final line `FULL_BUILD_VERIFIED`; record detail metres, added
minutes and lift-offs.

- [ ] **Step 5: Commit**

```bash
git add -A && git commit -m "Fixed build plate instead of the turntable; detail strokes lean toward the side they draw"
```

---

### Task 5a: Detail as few long strokes (no fixed sides)

Decided by the user after Task 5: join detail lines into as few strokes as
possible. The four fixed "faces" in the detail generator are left over from
the turntable's quarter turns and go.

**Files:**
- Modify: `scripts/detail_tours.py` (moves to `src/pen_art/detail.py` in Task 8)

**Interfaces:**
- Lines are grouped by a shared pen lean: each group gets one lean azimuth
  (any direction) and one tilt that clear all its lines against the frame,
  and becomes one stroke, its lines joined end to end by connecting runs
  that lie on the surface. Line order and joining ends minimize connecting
  wire (nearest neighbour, then 2-opt). Connecting wire counts toward the
  2 m detail budget. Stroke rows carry `pen_lean_azimuth_deg`; `face` goes.
- One pen orientation per stroke still holds: a group never needs the pen
  to change lean or tilt while drawing.

- [ ] **Step 1: Replace the four faces with free lean grouping and joining**

- [ ] **Step 2: Ghost detail end to end** (as Task 5 Step 4)

Expected: `FULL_BUILD_VERIFIED`; detail lift-offs below Task 5's 13 detail
strokes; record strokes, lift-offs, detail metres and added minutes.

- [ ] **Step 3: Tests, lint; commit**

```bash
git add -A && git commit -m "Detail as few long strokes, each with its own free pen lean"
```

---

### Task 6: Two config files and `pen-art cell`

**Files:**
- Create: `configs/cell.yaml`, `configs/frame.yaml`
- Create: `src/pen_art/cell_config.py`
- Delete: `configs/cell/`, `configs/profiles/`, `configs/fabrication.yaml`,
  `configs/transit.yaml`, `configs/timing.yaml`,
  `configs/timing_continuous_bounded.yaml`,
  `scripts/prepare_single_arm_cell.py`
- Modify: every hard-coded reader of the deleted configs
  (`section_build.py:345,363`, `cell_planner.py:386-393`,
  `scripts/frame_source.py`, `scripts/detail_tours.py:299-302`),
  `scripts/setup_machine.sh`

**Interfaces:**
- `configs/cell.yaml` (learner-facing, short, commented):
  ```yaml
  build_volume_m: 0.15          # longest side of the object
  plate:
    size_m: [0.256, 0.256]      # Bambu Lab X1/P1/A1 plate
    center_from_robot_m: [x, y] # from Task 5 Step 3
  pen:
    flange_to_tip_m: 0.225
    body_radius_m: 0.012
    nose_length_m: 0.027
    tip_radius_m: 0.0005
  robot:
    model: Rizon4
  ```
- `configs/frame.yaml`: today's `fabrication.yaml` (`wireprint`, `graph`,
  `structure`, `process`) plus `timing` (from
  `timing_continuous_bounded.yaml`), `transit` (from `transit.yaml`) and
  `planning` (from Task 4).
- `pen_art.cell_config.expand_cell(cell: dict, robot_dir: Path) -> dict`
  returns the full profile (today's `frame_template.yaml` structure without
  the turntable) with the pen bodies, collision model and initial joint
  positions as fixed defaults in code.
- `pen_art.cell_config.load_frame_config(path=Path("configs/frame.yaml")) -> dict`.
- `pen-art cell` (wired in Task 8; until then
  `python -m pen_art.cell_config`) fetches nothing itself, expands
  `cell.yaml` and writes `runs/cell/cell.yaml`, `robot.urdf`,
  `robot_manifest.json`, `robot_reference.glb`.
- The build volume comes from `cell.yaml`, not from `frame_source.py`.

- [ ] **Step 1: Write the configs and `cell_config.py`; switch the readers**

- [ ] **Step 2: The expanded profile equals today's**

Run: `python -m pen_art.cell_config` then compare `runs/cell/cell.yaml`
with the Task 5 profile.
Expected: identical except for key order and paths.

- [ ] **Step 3: Tests, lint, ghost frame end to end** (as Task 4 Steps 2-3, `TAG=t6`)

Expected: same verdict, lift-offs and print time as Task 5.

- [ ] **Step 4: Commit**

```bash
git add -A && git commit -m "Two config files: cell.yaml and frame.yaml"
```

---

### Task 7: Planning split into plan-geometry, plan-timing and verify-full-robot

**Files:**
- Create: `src/pen_art/stages.py`
- Modify: `src/pen_art/planning/parallel_build.py` (stop after the
  geometric phase or start from a saved one), `full_build_export.py`
- Modify: `scripts/detail_tours.py` (detail rows marked optional, ordered
  by detail per minute, best first)

**Interfaces:**
- `plan_geometry(run: Path, workers: int = 24) -> dict` writes
  `run/geometry_plan.json`:
  `{"status", "binding", "order", "actions": [{"id", "connect_key",
  "deposit_key", "chain", "optional"}], "returned_key", "dropped":
  [{"stroke", "reason"}], "lift_offs": [...]}`. A blocked optional
  (detail) stroke is dropped in one pass and later strokes are planned
  without it; a blocked frame stroke ends with status `BLOCKED` naming it.
- `plan_timing(run: Path, workers: int = 24, detail_m: float = 2.0, minutes: float = 15.0) -> dict`
  loads `geometry_plan.json` and the pickles under `run/parallel/jobs/`,
  runs the per-action time jobs, then trims detail: removes a suffix of
  detail strokes (they are last and ordered best first) until detail
  length ≤ `detail_m` and detail time ≤ `minutes`, plans one new return
  home from the new last stroke, and exports `run/plan.json` with
  `print_minutes`, `frame_minutes`, `detail_m`, `lift_offs`, `trimmed`.
- `verify_full_robot(run: Path, workers: int = 24) -> dict` runs
  `cell_replay` on `plan.json` and writes `verification.json`; status
  `FULL_BUILD_VERIFIED` or the rejection.
- Each stage refuses to run when its input is missing or its recorded input
  hash differs, naming the stage to run first.

- [ ] **Step 1: Split the run and write `stages.py`**

- [ ] **Step 2: Same result as one run**

Run the three stages on the ghost frame (no detail).
Expected: `plan.json` equals Task 6's plan except for file names and
timestamps; `FULL_BUILD_VERIFIED`; same lift-offs.

- [ ] **Step 3: Frame plus detail**

Run detail generation, then the three stages.
Expected: `FULL_BUILD_VERIFIED`; `dropped` and `trimmed` listed; detail
≤ 2 m and ≤ 15 min; lift-offs reported. Record the numbers.

- [ ] **Step 4: Out-of-order and stale inputs** (Review Focus 3)

Run `plan-timing` on a run with no `geometry_plan.json`.
Expected: exits non-zero with `run plan-geometry first`.
Regenerate the frame, then run `plan-timing` on the old geometry plan.
Expected: exits non-zero naming `plan-geometry`.

- [ ] **Step 5: Tests, lint; commit**

```bash
git add -A && git commit -m "Split planning into plan-geometry, plan-timing and verify-full-robot"
```

---

### Task 8: The `pen-art` CLI

**Files:**
- Create: `src/pen_art/cli.py`
- Modify: `pyproject.toml` (`pen-art = "pen_art.cli:main"`; drop
  dependencies no module imports any more), `scripts/setup_machine.sh` (calls `pen-art cell`)
- Delete: `scripts/frame_source.py`, `scripts/plan_frame.sh`,
  `scripts/benchmark_parallel_build.py`, `scripts/plan_detail.py`
  (logic moved into `stages.py`/`cli.py`); `scripts/detail_tours.py` moves
  to `src/pen_art/detail.py`.

**Interfaces:**
```
pen-art cell [--cell configs/cell.yaml]                  -> runs/cell/
pen-art frame MESH --name NAME [--up y|z]                -> runs/NAME/frame/
pen-art detail NAME                                      -> runs/NAME/detail/
pen-art plan-geometry NAME [--frame-only] [--workers 24] -> runs/NAME/plan/geometry_plan.json
pen-art plan-timing NAME [--workers 24]                  -> runs/NAME/plan/plan.json
pen-art verify-full-robot NAME [--workers 24]            -> runs/NAME/plan/verification.json
pen-art view [--port 8765]
```
Each command prints one summary line, for example
`plan-timing ghost: 41 strokes, 9.9 m frame + 1.3 m detail, 7 lift-offs, 17.2 min`.
A stage whose output folder exists rebuilds it from scratch
(`plan-geometry` also removes `plan.json` and `verification.json`).

- [ ] **Step 1: Write `cli.py`, move the scripts' logic, update setup**

- [ ] **Step 2: The whole path on the ghost**

```bash
pen-art cell && pen-art frame objs/cute_ghost/cute_ghost.glb --name ghost --up y
pen-art detail ghost && pen-art plan-geometry ghost && pen-art plan-timing ghost && pen-art verify-full-robot ghost
```
Expected: `FULL_BUILD_VERIFIED`; numbers match Task 7 Step 3.

- [ ] **Step 3: Review Focus checks**

- `pen-art frame objs/cute_ghost/cute_ghost.glb --name ghostz --up z`:
  completes; the viewer shows the ghost lying down (Review Focus 1).
- `pen-art frame` twice with the same name: the second run rebuilds
  `runs/ghost/frame/` and later stages refuse until rerun (Review Focus 2).
- `pen-art detail` with `--crease-threshold 10` (nothing found), then the
  three stages: `FULL_BUILD_VERIFIED` with `0.00 m detail` (Review Focus 4).

- [ ] **Step 4: Tests, lint; commit**

```bash
git add -A && git commit -m "pen-art CLI: one command per tutorial step"
```

---

### Task 9: Viewer for the tutorial only

**Files:**
- Delete: `web/src/main.ts`, `web/src/motion-reuse.ts`, the workbench page
  and its routes in `src/pen_art/server.py` (`/?workbench=1`, check
  routes)
- Modify: `web/src/simple.ts`, `web/src/viewer.ts`, `web/src/timing.ts`,
  `web/src/entry.ts`, `web/scripts/check-simple.mjs`

**Interfaces:**
- The page lists runs from `runs/*/plan/` and opens `?run=NAME`; panels as
  today (picture, model, wire, robot); the plate replaces the turntable;
  the status line shows verdict, print time and lift-offs.

- [ ] **Step 1: Remove the workbench and update the page**

- [ ] **Step 2: Headless check on the ghost**

Run: `(cd web && PATH=$PWD/../tools/node/bin:$PATH env -u DISPLAY node scripts/check-simple.mjs ghost /tmp/ghost.png)`
Expected: JSON line with `"status"` containing `verified`; screenshot shows
the ghost on the plate.

- [ ] **Step 3: Build, tests; commit**

```bash
git add -A && git commit -m "Viewer: tutorial page only, fixed plate, lift-off count"
```

---

### Task 10: Tutorial README and fresh-clone runs

**Files:**
- Modify: `README.md` (rewrite), `AGENTS.md`, `tests/README.md`,
  `docs/images/viewer_ghost.png` (new screenshot)
- Delete: `docs/superpowers/` (design and this plan) in the final commit

**Interfaces:** none.

- [ ] **Step 1: Fresh clone at 15 cm**

In tmux: clone from the local repository into the scratchpad, run
`scripts/setup_machine.sh`, then the seven commands as the README will show.
Expected: `FULL_BUILD_VERIFIED`; record print time, lift-offs, detail and
the time each command took.

- [ ] **Step 2: Fresh clone at 8 cm** (Review Focus 5)

Set `build_volume_m: 0.08` in `configs/cell.yaml`, rerun from `pen-art cell`.
Expected: `FULL_BUILD_VERIFIED`; record the same numbers.

- [ ] **Step 3: Write the README as a tutorial**

Sections: what it is (with "Everything is simulation"); input meshes
(generated by tools like Meshy, textured `.glb`, `--up`); setup; steps 0-6,
each with what it does, the command, the measured ghost output from Steps
1-2, and what to try changing (volume to 8 cm; pen length; plate size;
`frame.yaml` layer height); a concept section "Drawing the frame in one
stroke (an Euler trail)": Euler's rule (0 or 2 odd nodes), connector wire
pairing the odd nodes, Hierholzer's algorithm band by band, a link to the
Seven Bridges of Königsberg
(https://en.wikipedia.org/wiki/Seven_Bridges_of_K%C3%B6nigsberg), and the
ghost's numbers (18 odd nodes, minimum 9 trails before connectors, 1
after, the measured lift-offs with reasons); chaining and lift-offs
explained (open-loop re-approach, joins onto sagged wire); detail as the
place to add an AI agent for artistic detail (eyes, mouth painted in the
texture); concepts; limitations; development checks; license.

- [ ] **Step 4: Update `AGENTS.md` and `tests/README.md`**

`AGENTS.md`: rules unchanged except "Exactly 5 tests" and the check
commands without browser tests; point to the README.

- [ ] **Step 5: Final checks and commit**

Run: `.venv/bin/pytest -q && .venv/bin/ruff check src tests && (cd web && PATH=$PWD/../tools/node/bin:$PATH npm run build)`
Expected: `5 passed`, lint clean, build ok.

```bash
git rm -r docs/superpowers
git add -A && git commit -m "Tutorial README; remove the design and plan documents"
```
