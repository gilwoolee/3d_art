# Handoff — continue on Blondie

Updated 2026-10-12. Read this first, then [PLAN.md](PLAN.md). The longer design
review and old experiment logs are in `docs/archive/`.

**Current workflow:** build commands stop at `FULL_BUILD_PLANNED` after writing
compact schedule JSON. Run `planning.cell_replay` separately for final full-robot
verification; only a matching complete pass enables new full-build playback.
The viewer only loads/animates saved data. Export no longer repeats audits, creates
collision journals or writes computational sidecars. Existing verified runs are
unchanged. Commands and validation checklist are in [PLAN.md](PLAN.md).
Historical timings below include the former automatic final verification pass.
Current demonstrated run: `cube_modular_playback_v1`. Exact saved cube export
took 6.48 s; separate 16-worker raw verification passed in 216.14 s with the same
39 actions, 111,584 intervals and final geometry. Browser checks passed: missing
verdict blocks playback; verified playback sends no backend requests and keeps
artifacts unchanged. `modular_playback_receipt.json` and `modular_browser_receipt.json`
are in the run. All 25 Python + four browser contracts, Ruff and the frontend
build pass. This is a saved-schedule check, not a new full planning benchmark.
The API test was subsequently removed at the user's request; the current routine
suite contains 24 Python + four browser tests. No stalled test remains running.

**32-worker benchmark finished:** `runs/compute_cleanup_cube_32_v4_benchmark.json`
and `_queue.json` passed with identical source/profile/library hashes and plan
SHA to the 4/8/12/16 comparison. Complete time 454.5 s (7.6 min), peak tree PSS
36.86 GiB. Planning improved to 158.5 s, but export/replay grew to 279.9 s;
collision replay alone grew from 93.3 to 143.4 s. Total is effectively tied with
16 workers (451.6 s), using almost twice the memory. Keep 16 for this workload.
The 32-worker controller/guard/build processes have exited. No source freeze or
GPU investigation is active.

## Parallel Phase 1 cube planning — C1–C6 complete

The first checkpoint was 2026-10-12 13:45 PDT; the C1–C6 update below is current.
PLAN.md's **Parallel Phase 1 planning within one
cube** section is the authoritative checklist and holds the measured numbers.
The final results record is [2026_1012_parallel_cube_results.md](2026_1012_parallel_cube_results.md).
The notes below supersede older worker descriptions in this file.

**C1–C6 implementation update:** C1–C6 are implemented and validated. The reduced
critical suite has 25 Python + four browser tests, all passing. The reproduced
C2 JSON/manifest binding gap has been fixed and applied, together with bounded
shared final-replay journals and separate loading/worker audit metrics.
Prepared source copies remain as evidence:
`runs/compute_cleanup_clock_store_candidate_v2.py` and
`runs/compute_cleanup_clock_tests_candidate_v2.py`. A third tested candidate,
`runs/compute_cleanup_cell_replay_candidate_v2.py`, separates loading and
worker parse/audit timings and bounds shared final-replay journals.

C6 controlled 1/2/4-worker full-cube scaling finished sequentially:
`runs/compute_cleanup_cube_scaling_v2_benchmark.json`. All passed 39/39 actions,
111,584 intervals and matching 142,102-capsule final geometry. End-to-end times
are 52.7/28.2/16.0 min; observed simultaneous tree PSS peaks are
2.14/4.18/6.92 GiB. Four workers are 3.29× faster than serial. These timings
precede the subsequent codec/journal hardening. Original receipts remain intact.

The completion runner `scripts/complete_compute_cleanup.py` finished with
`PASSED`: `runs/compute_cleanup_completion_v2.json`.
It checked source/candidate hashes, applied the
codec fix atomically and reran the 29 critical tests/Ruff/diff checks. All three
exact saved schedules passed fresh raw full-robot replay; receipt
`runs/compute_cleanup_cube_codec_v2_codec.json`. The obstruction probe and cold
base-action proposal/full-robot local check passed in
`runs/compute_cleanup_blocked_cold_v1.json`. Recovery testing was dropped by the
user; the completed logs remain historical evidence, and the future C6 runner
no longer schedules it. Full browser playback passed at 10× through the 1,917.9 s horizon,
with no errors: `runs/compute_cleanup_cube_codec_v2_w4/viewer_browser_browser_replay.json`.
Fresh codec runs:
`runs/compute_cleanup_cube_codec_v2_w{1,2,4}`. Downstream receipt:
`runs/compute_cleanup_c6_validation_v2.json`. The old waiting runner was stopped;
`runs/compute_cleanup_c6_validation_v1.json` records that hold. Inspect actual
processes and receipts before restarting anything. All C1–C6 gates are checked
in PLAN.md. Recovery testing was removed at the user's request.

**Current CPU results:** `runs/compute_cleanup_cube_high_v3_benchmark.json` and
`runs/compute_cleanup_cube_high_v3_queue.json` are complete (`all_complete`
true / `PASSED`, source/profile/library hashes unchanged). Runs were sequential.

| Workers | Complete compute | Observed peak tree PSS |
| --- | ---: | ---: |
| 4 | 16.0 min | 6.20 GiB |
| 8 | 10.1 min | 10.97 GiB |
| 12 | 8.7 min | 15.23 GiB |
| 16 | 7.5 min | 19.21 GiB |

Sixteen is fastest among tested counts: 53% less time than four (2.13×).
All runs passed 39/39, 111,584 raw full-robot intervals, matching deposits/final
material, home table and an identical plan SHA. The already-passed browser
receipt binds that same plan SHA. The scaling controller, build workers and
queue wrapper have exited for that sequence and for the 32-worker trial above.
Use `--workers 16` for this measured cube
configuration; other objects/workloads need their own evidence.

**GPU handoff:** the user stopped GPU investigation for another agent to take
over. Findings and proposals are saved in
[GPU validation findings](2026_1012_gpu_validation_findings.md). No GPU backend
code was changed. Do not resume this investigation in the current session.

The RTX 5090 is idle during CPU scaling. The existing
`curobo_adapter.py` uses cuRobo for IK proposals with collision checking
disabled; `--gpu` cannot currently accompany forked `--workers`. All 39
baseline candidates came from cache, so the main GPU opportunity is batched
kinematics/collision and trajectory audits, not just fresh IK. This requires
integration with exact material prefixes, scoped nozzle contact and continuous
bounds; no GPU speedup has been measured. Current
[cuRobo capabilities](https://github.com/NVlabs/curobo) support investigating it
after the matched CPU results. GPU implementation is outside C1–C6.

**Viewer publication update:** port 8765 has been restarted in tmux session
`pen-art-viewer`. Current builds now includes **Cube · complete 39/39 · Phase 1
parallel** (`cube_parallel_trial_v2`); the older five-action cube remains in
All runs. Open `http://localhost:8765/?run=cube_parallel_trial_v2&stage=motion`.
The bottle remains a verified 5/55 prefix. Full cube 10× browser replay passed
through the complete 1,917.9 s horizon, with no browser errors and the passed
replay badge. Receipt: `runs/cube_parallel_trial_v2/viewer_browser_browser_replay.json`.
This update supersedes the earlier reboot/viewer state below.

**Done.** P1–P4 and the assembly/replay part of P5 are implemented:
`src/pen_art/planning/parallel_build.py` plus changes in `cell_planner.py`,
`clock_path.py`, `cell_replay.py`, `motion_library.py`, `motion_runtime.py`,
`path_certificates.py`, `scene.py`, `full_build_export.py`, `section_build.py`.
`runs/cube_parallel_trial_v2` is the first complete generated cube: 39/39,
raw full-robot `CERTIFIED_GEOMETRIC_REPLAY`, 8 workers, 23.4 min end to end.
299 tests, Ruff and `git diff --check` pass.

**Remaining project work.** Bottle/artwork acceptance remains as listed in
PLAN.md; it is outside C1–C6. GPU implementation is not supported with
`--workers` and belongs to the stopped investigation handed off above.

**Previously queued validation.** The sequence is `runs/cube_parallel_followup.sh`
(16/8-worker benchmark, repair, resume, probe, then 1/2/4 concurrently; roughly
3.5 hours). It was interrupted and its existing output names cannot be reused
unchanged. On an authorized resume, use fresh output names and run scaling jobs
sequentially for a controlled comparison. The historical launch command was:

```bash
tmux new-session -d -s pen-art-parallel -c "$PWD" \
  "bash runs/cube_parallel_followup.sh > runs/cube_parallel_followup.log 2>&1"
```

Use tmux: background and `nohup` launches were killed by session restarts. Do
not edit `src/pen_art/` while a run that may be resumed is in progress; job
results are bound to a hash of the source tree. The user asked for the running
sequence to be stopped; relaunch only when directed.

**How it works.** Each action first fixes its drawing path and DRAW clock
(`prepare_local_drawing`), which determine its deposits
(`validate_interval(deposit_only=True, journal=...)`). Checks of later actions
then run against those predicted deposits while earlier checks are still
running. `ForwardSchedule` binds every result to the candidates it assumed, so
a replaced candidate invalidates its dependents. `Coordinator` dispatches to a
forked pool and records results under `<run>/parallel/`; re-running the same
command resumes. `replay_cell(workers=N)` splits the full-robot replay into
chunks whose material is rebuilt from deposit journals and tied together by a
geometry hash chain. Planning uses the pen-only scope; only the replay certifies.

**Historical process state, before the authorized C1–C6 work above.** The machine rebooted at 13:38. No planner, benchmark or
replay process is running. The two old stopped workers (cube PID 1643234,
bottle PID 1642834) were killed earlier at the user's request; their on-disk
checkpoints in `runs/cube_nozzle27_full_v3` and `runs/bottle_nozzle27_full_v1`
remain. The viewer's tmux session did not survive the reboot and has not been
restarted; see **Viewer and assets** for the command. Viewer prefixes are
unchanged (cube 5/39, bottle 5/55).

**Compute notes.** Setup is about 70 s, mostly loading and hashing the 1.8 GB
library. Each planning worker held roughly 3 GB of its own memory. The raw
full-robot replay is the dominant cost, about 90 CPU-minutes for the cube.

**Earlier work (2026-10-12):** reduce repeated collision-validation time on cube
and bottle, rather than finish the old full-build searches. Those workers,
then at 36/39 and 31/55 qualified actions, have since been killed (see above).
The current changes replace whole-scene contact clipping with local cell views,
cull distant/lower material using conservative bounds, and refine only nearby
wires. Planning uses the complete pen against material/work surfaces; final
replay always restores the full robot. Pen-only checks cannot admit qualified
motion-library records or full-robot invariant certificates. Candidate snapshots
carry a distinct scope. Benchmark fixtures/source snapshots are under
`runs/local_collision_efficiency_v1`. Matched planning-clock times before → now:
cube cold 78.75 → 16.11 s, warm 68.83 → 7.46 s; bottle cold 31.86 → 24.51 s,
warm 16.27 → 8.53 s. Separate full-robot final checks pass with identical material
hashes; both added-obstruction fixtures are rejected by candidate and full checks.
These are local late-action timings, not whole-build speedups. Main savings come
from local contact geometry, with warm reuse of deposition/orientation proofs;
removing robot checks alone makes little difference. Full suite: 282 passing tests.
See `runs/local_collision_efficiency_v1/summary.json` and PLAN for scope/details.
The viewer remains on its previously verified prefixes.

**Latest milestone:** the detailed verification-efficiency checklist V1–V6 in
PLAN is implemented for Phase 1 and Phase 2, checked across cube, bottle and
torus. Persistent exact-path certificates reuse source, fixed orientation,
self/static and clock checks; deposition envelopes are reused only for the
current exact interval. Material is always reconstructed and rechecked.
Threshold material queries and vectorized conservative atlas lookup reduce
collision work. Compact IK seeds do not alter accepted timed clocks.

Actual first-two-action replay (base + entire zigzag), with material checkpoints
disabled in every pass: cube raw/cold/warm **50.98/52.46/14.51 s**; bottle
**82.93/84.69/37.57 s**. Verdicts, material and final state digests match. See
`runs/replay_verification_efficiency_v2/benchmark.json`. Cold proof storage has
overhead; warm verification is 3.51×/2.21× faster. This is not fabrication speed.

The current-tool dataset/checkpoint and short three-object comparison are under
`runs/verification_efficiency_upright27_v5`; actual runtime neural proposals
under forced cache misses are checked in `runs/runtime_neural_efficiency_upright27_v2`.
Do not use the historical checkpoint for the new upright tool. One shared model
is bound to the common physical profile; runtime timing/process constraints are
separate and still checked. All neural seeds are projected to fixed orientation;
the previous free-axis seed flag caused their adapter to reject them.
Raw replay remains default. For a warm comparison with fresh material, use
`replay_cell(..., verification_raw=False, reuse_material_checkpoint=False)`.
Teacher/oracle acceleration uses `verification_directory=...`; explicit
`verification_raw=True` disables it. PLAN lists the reproducible CLI commands.
At that milestone, the full suite had 230 passing tests and all owned efficiency
workers had finished. The newer full-build workers below are active.

**Latest steering:** redo drawing with **one full TCP orientation per whole
connected action**, and require this of **all cached drawing trajectories**.
Tilt, azimuth and roll stay fixed during extrusion; pen-off travel may reorient.
The latest approved geometry uses descending strands at **at most 27° from
horizontal**, with a fixed upright pen (0° tilt). The generator keeps the source
asset and scale, reduces layer height while retaining ~25 mm wire spacing,
checks actual chords, and omits the final closing descent. Current cube/bottle
maxima are 25.820°/25.823°, with 39/55 whole actions.

Current execution retains the provisional tapered nose, 20 mm footprint riser,
closer cell and whole-action geometry. Profiles use `drawing_orientation_policy:
fixed`; active cache is `runs/motion_library_phase1_upright27_v1`, source runs are
`runs/{cube,bottle}_nozzle27_phase1_v1`. Keep the viewer running. Source checks:
`runs/nozzle27_validation_v1/source_validation.json`. All 39/55 framework actions pass sampled tool/fixture/growing-wire clearance
(minimum >1.85 mm). Both first whole zigzags passed continuous fixed-orientation
tracking and raw robot/growing-prefix clearance against nominal completed bases;
both are admitted to the active cache. Both bases passed from home and independent replay. Both whole-zigzag warm
probes pass full-clock checks and independent raw replay with zero cold solver
calls (`runs/{cube,bottle}_nozzle27_cache_runtime_v3`). Bounded full-build checks ended incomplete at 5/39 cube actions and 5/55 bottle
actions (`cube_nozzle27_full_v3`, `bottle_nozzle27_full_v1`). Their checkpoints
are preserved; do not equate the verified local probes with complete objects.
The bottle contact report contains an underlying `TIME_BUDGET` witness.
Timed whole-zigzag records were also promoted into the active library after
independent replay, with exact hashes retained in
`runs/nozzle27_validation_v1/timed_cache_promotion.json`.
Cache timing now uses a bounded 240 s shared action budget, limited by the caller's
global deadline, without knot-count-based realization cutoffs. This matches cold
candidate allowance. Both workers now run frozen implementation `89555bc1...`
with a 14400 s global deadline; cube retained 21 proposals and bottle 15 at
restart, which require current-code qualification before snapshot reuse.
Worker receipts are `runs/cube_nozzle27_phase1_resume_v7_receipt.json` and
`runs/bottle_nozzle27_full_v1/phase1_full_worker_v4.json`; current execution is
tracked in `runs/full_phase1_verification_reduction_v1/execution.json`.
Shared library refresh and missing whole-path families are
implemented. Indexed forward now defers the duplicate geometric drawing check
to its mandatory full timed-clock gate; conservative prospective material still
protects withdrawal planning. Strict verified-prefix snapshots, exact-pose
checks against appended material, and within-interval contact-geometry reuse
are implemented. All 252 tests, Ruff and diff checks pass; the frozen code receipt
is `runs/full_phase1_verification_reduction_v1/freeze.json`. The actual Bottle08
deferred-check A/B showed no measured speed gain, although both the clear pass
and blocked rejection were preserved. Contact-geometry reuse was added after
that comparison and still invokes fresh checks for each edge. Full-build speed
and completion remain unproven until the continued builds finish. The identical
Bottle08 local-clock warm probe now passes in 25.70 s versus 52.76 s earlier;
contact construction falls from 24 calls/25.95 s to 2 calls/2.09 s, retaining all
24 validation calls. See `runs/bottle_action08_contact_profile_v2/result.json`.
Strict cube state restoration also preserves exact native state/material parity;
five qualified actions restore in 0.95 s. These are local measurements, not
full-object acceptance. The browser
Current builds now opens independently checked **prefix playbacks**: cube 5/39
actions and bottle 5/55. Both are explicitly labeled partial,
with `CERTIFIED_PREFIX_REPLAY`, and both passed browser playback checks. See
`runs/new_partial_playback_validation_v1/summary.json`. These are not complete
builds. Run/stage selections now survive refresh; nested witness input hashes
are checked without marking these fresh exports stale.

Open `http://localhost:8766/?run=cube_nozzle27_phase1_v1&stage=motion` or replace
`cube` with `bottle`. The viewer still serves on remote port 8765; forward with
`ssh -N -L 127.0.0.1:8766:127.0.0.1:8765 blondie`. Full-build resumes use the
preserved checkpoints and active cache. The earlier 600 s continuations finished at
5/39 and 5/55. New 14400 s workers continue both full builds; their retained planning
proposals are not new independently replayed viewer results. Both prefix exports passed fresh raw replay and
browser checks; cube's next contact rejection contained `TIME_BUDGET`, not a
demonstrated collision. Consult their receipts before claiming further progress.

The earlier 70°/80° riser obstruction and 60°–30° tilt diagnostics remain
historical. Do not resume their old full-build checkpoints or infer that their
collision results apply to the new 27° framework.

Setup and local-riser-tip contact checks pass. Both setup pages render correctly;
headless browser checks require `env -u DISPLAY` because inherited SSH X forwarding
breaks EGL. Setup receipts are under `runs/taper70_setup_v1`. Previous base timings
and variable whole-zigzag certificates do not meet the corrected orientation
contract. Do not resume their old full-build checkpoints.

## Start here

```bash
ssh blondie
cd /home/gilwoo/Workspace/3d_art
git pull --ff-only
git status --short
```

Private repo: <https://github.com/gilwoolee/3d_art>, branch `main`.
The initial source baseline is `55efefd`; subsequent handoff commits are on top.
Use this existing workspace: its ignored data and environments are already installed.
A fresh Git clone alone will not contain the full asset collection.

## Target and decisions

- Generalizable mesh → WirePrint-style framework + WIR3D artistic curves →
  automatic build order → robot motion; **wireframe only**.
- WirePrint contours and zigzags are the chosen framework geometry. Do not add
  alternative framework generators or framework-design search. Planning chooses
  how to construct the supplied graph.
- Build and cool the framework first. Use one Caelan-style order-and-motion
  planner for framework and artwork, treating WirePrint's order as a suggestion
  with bounded initial effort. Plan both stages before execution so the completed
  framework does not block the details. The active checklist is in [PLAN.md](PLAN.md).
- One Flexiv 7-DoF arm, one rigidly mounted 3D pen, TCP at the pen tip, and a
  powered turntable carrying the build plate. Assume one vertical rotation axis;
  axis calibration and physical limits are not yet specified. The target state is
  `(P, q_arm[1..7], k)`, where `k` selects 0, 90, 180, or 270 degrees. Source
  geometry stays in plate coordinates. Try the four fixed orientations with the
  existing arm planner; no continuous eight-axis motion solver is required.
- Turn only in checked 90-degree steps between strokes, with the pen off and
  arm withdrawn/parked after required cooling. Hold the arm still during a turn;
  hold the plate still during arm motion and extrusion. Check the whole rotating
  sculpture/fixture sweep, legal travel directions, and settling before resuming.
  Coordinated rotation during extrusion is out of scope. Replay the sequential
  actions and both home returns. See PLAN's unchecked acceptance items.
- Maximum 15 cm per side. Palace stays uniformly scaled to 15 × 15 × 9 cm.
- Stops, withdrawal, pen-off travel, and rejoining are normal. No manual connections.
- Find the first feasible offline plan. No fabrication-time target, two-arm work,
  broad mounting search, or repeated grinding of one blocked motion.
- Keep the existing 1 mm clearance and 0.3 mm TCP tracking requirements.
- Current mount: straight along the flange centerline, TCP 225 mm from flange.
  Hold full orientation throughout each action; the active pose is upright
  (0° tilt), with descending framework strands capped at 27° from horizontal.
  The Rizon4 model, pen dimensions, and elevated platform remain provisional.
  Physical arm variant and exact pen details need calibration; everything is simulation-only.
  Old right-angle runs/caches remain historical and cannot certify the new tool.

## Cached trajectories and learned proposals

Both motion-reuse phases in PLAN.md are implemented. Phase 1 uses a persistent
motion library, local IK adaptation, and conservative self/static clearance reuse;
current material and the complete clock still receive ordinary validation.
Phase 2 trains conditional diffusion proposals from verified local paths and
feeds generated candidates through the same refinement and validation path.

Use `--motion-library runs/<library>` with an indexed `section_build` invocation.
Add `--diffusion-checkpoint runs/motion_learning_final/diffusion.pt` to enable the
learned fallback. The learning dependency is optional; cache-only planning does
not import PyTorch. README.md has the generation/curation/training commands.

Evidence lives in `runs/motion_runtime_prism_v2`,
`runs/motion_runtime_diffusion_v2`, and `runs/motion_learning_final`:
two cache-only and two hybrid full prism builds pass independent replay; the
hybrid cold build accepts one learned motion, and both modes exercise cached
backward planning. The frozen dataset has 86 verified paths, 65 witnessed
collisions, and seven unresolved attempts. The model trained for 2,000 CUDA steps
in 33.2 seconds. Keep these local motion/build diagnostics distinct from the
unfinished generated cube, bottle hybrid, and Palace acceptance below.

All 15 known-positive held-out requests are variants of one torus stroke, not
15 objects or complete builds. Caching passes all 15; the local IK teacher
passes 12. Its three failures return in 0.063–0.065 s, not at the 15 s timeout:
all four initial IK candidates collide with a synthetic arm obstacle. This is
not an exhaustive evaluation of the full construction planner.
Diffusion is never needed on those 15, so no additional test benefit
is demonstrated. A separate prism validation cache miss is solved by the learned
fallback slightly faster than teacher fallback. Simple queries favor the teacher.
The raw index-2 test remains unresolved under all three bounded methods.
Reports retain component timings, cache size, source hashes, and exact outcomes.
The viewer now has a **Motion cache** collection: `phase1_cache_cold`,
`phase1_cache_warm`, and `phase2_diffusion_cold`. Each exposes the original
verified playback plus stored-motion inventory and a selectable per-stroke
source. Display copies preserve the original plan/replay hashes.

## What works and what does not

The workbench defaults to **Current builds**: one representative each for Cube,
Bottle, and Palace. Cube and Bottle select their new 27° verified-prefix runs;
Palace selects the original blocked case. Run metadata uses the existing
`workbench_primary` flag to choose this group. **All runs** exposes the historical
experiments. An explicit older link adds just that selected case under
**Opened older run**, keeping the default list short instead of switching to the
whole archive. Switching back from All runs restores the three-object list.
`research/blondie/workbench_current_builds.json` records the display metadata
changes, and `workbench_current_browser_check.json` checks filtering, old links,
and the mobile layout. Refresh the browser to load the current frontend.

The two backward bottle workers are now stopped. A complete, independently
verified orientation bound excludes the required `wir3d-20-0014` endpoint in
their conservative full-framework envelope, with the existing 1 mm clearance
and 0.3 mm tracking allowance. This is not a proof against less conservative
actual deposition. `research/blondie/bottle_endpoint_orientation_certificate_verification.json`
binds both graphs; reproduce the check with
`.venv/bin/python research/blondie/verify_bottle_endpoint_certificate.py`.
Each stopped run has `conservative_envelope_stop.json`, preserving its previous
report, checkpoint hash, process identity, and replacement run.

`runs/bottle_hybrid_forward_turntable_v1` retains the original 273-action graph
and all 29 art sources. It uses `--progression-only`: a greedy support/stiffness
prepass followed by motion backtracking in the resulting order, with indexed
transitions and actual preceding deposition. It is not a general forward order
search. It can reuse 78 fixed-base
forward-prefix paths as proposals; it rechecks their geometry and timing and
validates them in the current material/table state. The source-hint rejection
checks are in `research/blondie/indexed_forward_source_hints_check.json`.

The new mode passed the three-action `staged_indexed_forward_v1` build, a fresh
537-interval independent replay with zero reused path checks, and full 10x
browser playback. `bottle_reference_forward_v1` now also passes: 28/28 strokes,
5,483 independently checked intervals with no reused path checks, two turns,
and both homes. Full 10x browser playback passes in 90.176 seconds without errors.
`research/blondie/indexed_forward_check.json` records this scoped acceptance
and four negative index/interlock checks. Its `supplied_order.json` records the known reference
order used for this integration; an earlier natural-order attempt stopped at
4/28 and remains in `natural_order_result.json`. Neither diagnostic replaces
the required generated bottle. Logs are `<run>.log` for the generated forward
bottle (now `_resume1.log`) and `bottle_reference_forward_v1_resume2.log` for the reference.
The natural bottle priority order failed its complete support precheck at
position 154. The new greedy prepass swaps `section-00154` before
`section-00153`, checks every one of the 273 prefixes, and preserves the first
153 actions. `forward_order_repair.json` and `forward_order_resume.json` bind
this correction and the retained 58-motion prefix. All 58 motions have now been
revalidated under the corrected order; the resume receipt records that milestone.
The current run has also passed both reordered strokes: `section-00154` ends at
`n00769`, where `section-00153` starts. The same receipt records the observed
156-action checked prefix and bound witness hashes. This is planner progress;
full independent replay of the generated bottle is still pending.
The cube's original order passes all 225 support/stiffness checks
and needs no restart for this change. The new prepass also returns exactly the
independently verified reference-bottle order; that check is recorded in
`research/blondie/indexed_forward_check.json`. The existing focused suite still
has 11 passing tests; the forward test was extended rather than adding a test file.
The current completion/browser watcher is `/tmp/watch_indexed_forward_replays.py`;
its status is `runs/indexed_forward_replays.json`. Historical status paragraphs
below describe earlier runs; check the current process argv before acting.

The three active planners now run in persistent tmux sessions:
`pen-art-cube-forward`, `pen-art-bottle-forward`, and `pen-art-cube-backward`.
`pen-art-completion-watch` watches those runs and launches the browser check only
after full replay passes. Do not stop `pen-art-viewer` or unrelated sessions.
The former tool-owned workers disappeared with intact checkpoints at cube
106/225, bottle 62/273, and a 147-action cube backward suffix. Each run's
`tool_session_recovery.json` records the verified checkpoint hash and previous
report. `live_worker.json` records the new process ID, exact command, and terminal
exit status; verify `/proc/<pid>/cmdline` as well before treating it as live.
The wrapper is `/tmp/run_pen_art_build.py`. Current logs are
`cube_stl_forward_turntable_v1_resume1.log`,
`bottle_hybrid_forward_turntable_v1_resume2.log`, and
`cube_stl_turntable_v2_resume7.log` under `runs/`. Saved paths are revalidated.
For a quick status check, `.venv/bin/python /tmp/poll_indexed_builds.py` scans
actual planner command lines and distinguishes the currently revalidated prefix
from the longer retained proposal checkpoint.

`runs/cube_stl_forward_turntable_v1` is a second actual-deposition forward trial
for the generated cube. It uses the freshly regenerated graph from
`cube_stl_turntable_v2` and considers 174 local path proposals from
`cube_wireprint_spatial_v1`. `forward_proposal_source.json` records identical
nodes, members, and construction actions; the old/new graph hash difference is
only generation metadata. Source proposal import compares the full nodes and
edges, frame, units, and schema before considering any paths. Fresh validation
is still required; 173 proposals passed the initial source-path audit, and the
tmux resume imported 184 from the newer saved source prefix. The cube's existing
backward search remains active.

Palace hardware diagnostic: `research/blondie/probe_palace_pen_clearance.py`
keeps the active profile unchanged while screening hypothetical nozzle geometry.
Narrowing the shank alone found no passing sampled orientation for the required
curve. An explicitly modeled 1 mm diameter, 40 mm long extension admits two
constant tool orientations with continuous clearance from the full framework.
These fail against a conservative enclosure of the entire current stroke,
including its future material; this does not prove a progressive drawing is
impossible. Robot reach, self-collision, deposition, and practical extension
hardware remain unverified. This is a geometry diagnostic, not an accepted pen
change or a Palace solution. See `palace_extended_tip_probe.json`; reproduce
with `--extended-tip`, followed by `--extended-tip --verify-saved`.

The user explicitly reaffirmed that the cube deliverable is **solid STL →
WirePrint framework → print**. `cube_turntable_v1` is only the sparse 12-edge
diagnostic, not acceptance of that pipeline. The primary cube run is
`cube_stl_turntable_v2`: freshly generated from the solid STL, with nine horizontal
contour levels, zigzag bracing, 436 graph members and 225 construction actions.
Its source audit confirms exactly the same member geometry as the earlier
`cube_wireprint_turntable_v1`; its complete print remains pending. The newer
candidate ordering tries outward-facing pen bodies first, without changing any
geometry or acceptance margin. Both existing searches remain intact.

The indexed planner is wired through construction search, clocks, source-relative
replay, and viewer animation. `planning/indexed_build.py` reuses the existing
regression search with an eighth state entry for the discrete plate index.
`runs/cube_turntable_v1` passes full independent replay: 12/12 sections, 3,036
intervals, 548.388 modeled seconds, both homes. Its complete 10x browser check
passed in 57.5 seconds without errors; see
`research/blondie/cube_turntable_v1_browser_replay.json` and the matching images.
`runs/turntable_integration_v2` replays that plan in a fresh cell and appends all
six legal quarter-turns; it passes 3,055 intervals and returns both mechanisms
home. It rejects unparked turns, missing settling, wrong source index, and stale
checkpoint context. Its full 10x browser check passed in 67 seconds with all six
turns observed and no errors. The earlier v1 driver
passed using a fixed-base source proposal, but v2 binds the new indexed source.
The turntable profile remains provisional and simulation-only.

Four focused turntable tests cover transforms, intermediate swept collisions,
pen/parking/cooling/settling interlocks, and optimizer deadline enforcement. The existing six construction tests
pass. Affected fixed-base tests needed only two mock planners updated with the
new explicit `turntable=None` field. The indexed checkpoint check is recorded in
`research/blondie/indexed_checkpoint_recheck.json`; source changes are always
revalidated and do not become certificates by loading a cache.

All four indexed reference diagnostics pass independent replay and full 10x
browser playback: `cube_turntable_v1`, `prism_turntable_replay_v1`,
`bottle_turntable_v1`, and `torus_turntable_v1`. The bottle uses two actual
quarter-turns; the prism includes six checked turns. Source, plan, replay,
browser hashes and both home returns are checked in
`research/blondie/indexed_reference_acceptance.json`. These remain diagnostics;
the generated solid-STL cube, hybrid bottle, and Palace are still incomplete.

Current generated runs: `cube_stl_turntable_v2`,
`bottle_hybrid_turntable_v1`, `palace_turntable_v1`.
Each has a matching `runs/<name>.log`. The generated cases keep exactly 225, 273,
and 1,218 sections; the hybrid and Palace retain all 29 and 18 art sources.
Their `indexed_source_audit.json` checks source identity, coverage, and size only.
The unchanged bottle section-00017 was revalidated at 90 degrees in
`indexed_access_bottle_v2`; the same-state 0-degree candidate failed continuous
checking. This is a bounded comparison, not proof of all-angle infeasibility.
The reusable driver has `--access-witness` for this case. Full generated-object results
remain pending. `prism_turntable_warm_v1` tests reuse of source-run motion
proposals with fresh collision/source checks; it imports no old certificates.
Latest cube and bottle workers log to their run names followed by `_resume6.log`;
the stopped original Palace worker used `_resume5.log`. Both older cube and bottle workers
stopped when FCL returned a negative distance after a no-overlap result. The bottle
was resumed first; the cube subsequently hit the same already-fixed condition.
The inconsistent result now conservatively rejects that candidate, with a
focused regression check. Each affected run preserves its retained proposal
count and previous error in `resume_fcl_disagreement.json`.
Check live process ownership before acting; historical PID snapshots are stale.
The exact-restart trials use `runs/<name>_resume2.log` and the validated
material-cell sphere prefilter. Their `resume_material_filter.json` files retain
the previous reports, checkpoint hashes, and old/new process IDs. The original
cube and bottle workers remain running with their earlier loaded code; both
Palace workers are now stopped for the access conflict below.
The prefilter uses conservative whole-body and cell spheres only to skip distant
cells; retained cells still receive exact FCL checks. Bounds refresh after new
deposition. All 124 sampled Palace queries match the prior distances, and a
complete reference bottle replay passes all 5,483 intervals and two turns.
See `research/blondie/material_sphere_filter_check.json`,
`material_filter_optimizer_check.json`, and `material_sphere_filter_replay_check.json`.
Measured gains are about 1.48x for warm queries and 1.11x for one complete pose
solve; neither is a full-build speedup. Existing tests were extended in place.
Retained indexed motions now take priority over regenerated structural guidance
and are revalidated before reuse. A process-local cache avoids repeating the
same successful check. The actual 21-motion cube suffix was recovered with
fresh checks in `runs/cube_stl_turntable_v2/checkpoint_resume_check.json`; it is
not a completed build or a forward printed prefix.

A bounded one-curve restart probe on bottle curve `wir3d-20-0009` preserves
its exact cubic as two halves, adds no external support, and permits one
quarter-turn between them. Both local motions pass fresh source, clock and
continuous-collision checks in `runs/bottle_restart_probe_v1/restart_probe.json`.
This is not a full build. A complete 274-action trial is running in
`runs/bottle_hybrid_restart_turntable_v1` with all 29 artistic curves retained.
Its graph revision and parent hashes are recorded in `restart_trial.json`.
The original 273-action bottle run remains active.

A Palace one-curve restart trial is now `runs/palace_restart_turntable_v1`
(1,219 actions, all 18 selected artistic sources retained). It splits only
`wir3d-20-0016` exactly in half, with no external support or geometry change.
`runs/palace_restart_probe_v2/restart_probe.json` revalidates both local motions
against the reached state with three strokes scheduled later. The earlier v1
probe failed a structural prerequisite: removing this curve from the otherwise
complete sculpture gave 1.009 mm modeled deflection, above the unchanged 1 mm
limit. The framework alone passes at 0.366 mm; the reached state passes at
0.377 mm. This is an order constraint, not justification to weaken stiffness.
The full trial imports the three later motions only as source-remapped
proposals requiring fresh checks; its `restart_trial.json` records the mapping.
Full build and browser acceptance remain pending.

Palace endpoint access was first investigated with bounded diagnostics for
`section-01212` / `wir3d-20-0020`, its shortest remaining artwork curve.
`research/blondie/palace_endpoint_obstructions.json` samples 1,297 tool
orientations per endpoint; none clears both the completed conservative
framework and platform, even with other artwork excluded. A continuous
orientation optimization also fails to reach the required 1 mm clearance;
see `palace_endpoint_continuous_orientation.json`. Those finite searches alone
were not an impossibility proof.

The subsequent orientation certificate covers the entire rotation domain with
161,028 verified regions. It includes 0.3 mm tool tracking, 0.35 mm framework
point displacement, and removal of the 0.35 mm planning-radius inflation.
An inscribed tool sphere remains within the required 1 mm clearance of a
framework strand or the plate/housing throughout each region. Exact binary
partition coverage and every region's distance bound are checked independently.
See `research/blondie/palace_endpoint_source_tolerance_certificate_verification.json`.
The result applies to both Palace graphs and all four table indices, under
the declared nominal strand and provisional tool geometry. It is not a claim
about different pen hardware or unmodeled physical deformation.

Both Palace workers were stopped after this verification. Their run-local
`model_access_conflict.json` files preserve the old reports, checkpoint hashes,
stopped process IDs, and the certificate reference. No artwork, geometry,
clearance, or framework-first requirement was changed. The actual pen geometry
or design constraints need resolution before these cases can be replanned.
The cube and both bottle searches continue. Reproduce the check with:

```bash
.venv/bin/python research/blondie/certify_palace_endpoint.py --source-tolerance
.venv/bin/python research/blondie/verify_palace_endpoint_certificate.py --source-tolerance
```

The current workers interleave bounded failed sampling chunks
across eligible source actions instead of exhausting every orientation for one
action first. Successful paths and untried alternatives remain lazy iterators;
framework-before-art and stiffness gates are unchanged. The focused regression
forces an unresolved first source to yield so a later source can complete the
plan. A fresh staged indexed build and independent replay pass in
`runs/staged_indexed_fair_v1`; its full 10x browser playback also passes in
20.2 seconds. See `research/blondie/indexed_fair_search_check.json`. This remains
a small integration check, not completion of the generated objects. Prior
worker identities and source hashes are in `resume_fair_source_sampling.json`.

Matching retained varying-axis corrections are now freshly prepared before
continuing recovery; a failed four-correction attempt need not lose its progress.
The current workers include this reuse adjustment.

Varying-axis candidates now reach the same bounded interior collision recovery
as fixed-axis candidates instead of returning immediately after a continuous
clearance failure. Corrected candidates must pass full preparation again; four
failed corrections remain a rejection. The controlled dispatch check is in
`research/blondie/free_axis_recovery_dispatch.json` (not physical feasibility).
All three current workers load this change. The generated cube has
reached real interior corrections for section 00216. A fresh complete reference
bottle replay using the current implementation passes all 5,483 intervals and
two indexed turns, ending at table index 0, in
`runs/bottle_turntable_current_replay_v1`; its complete 10x browser check passes in 89.965 seconds with both turns and
no errors. See `research/blondie/bottle_current_code_acceptance.json`. The one-shot watcher
`/tmp/watch_indexed_generated_replays.py` starts complete 10x browser checks
only after `FULL_BUILD_VERIFIED`; its log is
`runs/indexed_generated_replay_watcher_both_restart_trials.log`. Check its live process before
starting another watcher.

Robot/tool self-collision pairs and their relative motion bounds are now
precomputed per model. A bounded in-memory cache stores exact signed pair
clearances keyed by the joints that can change that pair, so a common rigid
rotation reuses the answer. Environment and deposited-material checks remain
uncached by this mechanism; margins are applied after retrieving distances.
Rebuild the model when robot/tool geometry or calibration changes. The actual
Flexiv-plus-pen check in `research/blondie/self_collision_cache_check.json`
agrees with the uncached implementation on 50 poses and makes repeated self
checks 2.92 times faster. This does not claim a full-planner speedup. Existing
collision tests cover common versus relative motion, changed margins, moved
obstacles, and material added after cache warmup without adding test cases.
A fresh complete reference replay with this cache passed all 5,483 intervals
and both turns, ending at table index 0; see
`research/blondie/self_collision_cache_replay_check.json`. The current workers load the cache. Their prior PIDs and retained proposal counts are
recorded in each run's `resume_self_collision_cache.json`.

Material scene copies and contact clipping now share unchanged, immutable FCL
capsules; only cut pieces get new geometry. The actual cube comparison in
`research/blondie/cube_material_reuse_check.json` retains identical clipped
capsules and reduces construction CPU time from 2.64 to 0.43 seconds over
12 contact regions. This is a local cost reduction, not a full-search speedup.
The current real-arm model check `runs/turntable_model_v5` passes all six
quarter-turns. Existing live workers retain their import-time implementation.

A Palace stall exposed an exception swallowed by python-fcl's native `noexcept`
distance callback. The callback now stops traversal and rethrows on the Python
side; collision and pose repair queries carry deadlines. The actual 21,382-strand
Palace query reports timeout in 0.137 seconds for a 0.01-second budget (native
query/setup overhead can overrun the requested time); see
`research/blondie/palace_query_budget_check.json`. This checks bounded query
behavior, not geometry feasibility. Focused tests cover deadline and interrupt
propagation without adding test cases. All three generated workers were subsequently resumed with this fix; their
`resume_free_axis_collision_repair.json` receipts preserve prior worker identities
and retained proposal counts.

Old fixed-base workers retain their loaded implementation; do not restart live
workers merely because observation takes time. The full execution goal is active.

Complete modeled cube, prism, bottle and torus builds pass independent replay, including
home return: `runs/cube_sections_v1`, `runs/prism_sections_v1`, and
`runs/bottle_sections_v1`, plus `runs/torus_sections_v1`. Bottle covers 28 sections in
745.13 modeled seconds; torus covers 32 sections in 1,036.77 seconds. A real
interleaved-source case passes in `runs/interleaved_sections_v1`, with negative
coverage, cooling, stage, fabricated-junction and pen-event checks. No physical sculpture has been
validated. The remaining acceptance checklist and evidence are in `PLAN.md`.

New entry point: `python -m pen_art.planning.section_build`. It supports reference
fixtures, generated WirePrint sources, optional cuRobo IK, and backward search.
Generated graphs pass structural preflight on all four fixtures; complete generated
framework/hybrid and Palace robot builds remain unverified. The WIR3D bottle job
in `runs/bottle_hybrid_source/wir3d_bundle/` completed and exported 29 curves, all
retained. The 273-section hybrid passes the provisional final-frame screen at
0.0671 mm displacement and its source-coverage audit. Robot planning is active in
`runs/bottle_hybrid_v1`; its current log is `runs/bottle_hybrid_v1_reporter_fixed.log`. Evidence:
`research/blondie/bottle_source_provenance.json`.
`research/blondie/final_graph_preflights.json` now binds all seven final test
graphs to passing structural/source-coverage audits and strand-inclusive size
bounds below 15 cm. These checks do not replace construction-prefix or motion
validation; the material assumptions remain uncalibrated.
The current cube run `runs/cube_wireprint_spatial_v1` splits zigzags into legs at
real apex/valley joins and prefers upward deposition. Earlier whole-band and
paired-leg attempts preserve clearance failures. Radial-axis proposals failed
standalone path checks and have been removed. Adjacent source contours now align
their seams; earlier trials accidentally connected different square corners.
`runs/palace_hybrid_v7` uses the full-resolution normalized mesh, crossing repair,
explicit component connectors, four bed posts, and whole-style-curve selection
above the bed. Its corrected contour seams and individual zigzag-leg sections
produce 8,950 members and 1,218 sections. The provisional final-frame screen passes
at 0.999 mm maximum deflection. The earlier 0.892 mm value belongs to v5, preserved
in `research/blondie/palace_framework_preflight.json`; it does not describe v7.
Robot planning remains in progress. Both search directions now use explicit
stacks to support frameworks larger than Python's recursion limit; the current
Palace forward worker logs to `runs/palace_hybrid_spatial_v1.log`. The real staged
regression passed again with this implementation:
`research/blondie/staged_regression_iterative.json`.
Independent backward Palace trials are `runs/palace_hybrid_backward_spatial_v1`
and the older `runs/palace_hybrid_backward_diverse_v1`. Their matching `.log` files
record progress. They use the same physical graph
and starts with artistic additions against the completed framework. Feasibility
remains unverified. Forward stiffness guidance is bounded so motion search gets
time. Accepted repair reports are stored by content hash; superseded working
reports are not trusted as evidence for retained motion clocks.
The prior cube and Palace workers were superseded after verifying a GPU adapter
bug: repeating the reference posture across all seeds collapsed eight nominal
solutions to one distinct IK configuration. Supplying one reference allows
cuRobo to generate the remaining seeds; a four-target check yields 7--8 distinct
configurations, and all 46 fixture targets still pass. See
`research/blondie/flexiv_seed_diversity.json` and `flexiv_ik_diverse_benchmark.json`.
The current workers use this fix, collision-checked travel through home when
needed, and bounded repair at failed interior collision intervals. Every repair
still requires ordinary continuous validation. Restart receipts preserve the
earlier searches and verify unchanged physical inputs.
`research/blondie/cube_interior_collision_repair.json` records an accepted
interior collision correction in the generated-cube search, without claiming
full-build completion. A live helper, `/tmp/watch_generated_build_replays.py`,
launches 10x browser checks only after `FULL_BUILD_VERIFIED`; inspect its process
and `runs/generated_replay_watcher.log` before launching duplicate replay jobs.
The updated code passes a complete staged three-section build, independent
replay, and 10x browser playback in `runs/staged_shared_diverse_v1`; evidence is
`research/blondie/staged_shared_diverse.json` and
`staged_shared_diverse_v1_browser_replay.json`. This is a small regression case,
not completion of the generated-object work.
New forward workers save accepted motion proposals in `forward_witnesses/` and
an atomic `forward_checkpoint.json`. Re-run the same command/run to revalidate
and reuse these proposals; source/configuration changes invalidate the index.
Reuse still checks source geometry, current material, support and stiffness;
final return and full independent replay remain mandatory. This is a saved
prefix, not a complete search-frontier checkpoint. The older Palace workers
predate this feature. `research/blondie/section_checkpoint.json` records
a successful partial-prefix restart and damaged-witness/input rejection checks.
Checkpoint revalidation now retains the longest saved index until further
progress is made. `research/blondie/checkpoint_retention.json` records a real
interruption after one motion, preservation of all three saved proposals, and
complete independent replay after the next resume.
Its 545 intervals and the deliberately rejected-candidate regression's 555
intervals also pass full 10x browser playback. Receipts:
`research/blondie/checkpoint_retention_browser_browser_replay.json` and
`research/blondie/replay_rejection_browser_browser_replay.json`.
New backward workers save section-and-transit proposals in `backward_witnesses/`
and `backward_checkpoint.json`. Re-run the same backward command to try them;
each reuse is tied to the predecessor material set and destination configuration,
and rechecks source coverage, support and the complete motion. The index retains
alternative proposals, not a saved search frontier or reusable acceptance.
`research/blondie/backward_checkpoint.json` records an interrupted real suffix
restart and a complete three-motion recovery without new drawing generation.
The resulting 582-interval independent replay and full 10x browser pass. The
test also rejects damaged witnesses, changed input bindings and a different
destination configuration. Workers started before this change do not acquire
backward checkpoints retroactively. `research/blondie/backward_checkpoint_reload.json`
also verifies recovery in a fresh interpreter, malformed-index rejection, a fresh
582-interval replay and complete 10x browser playback. Six existing section tests pass.
Spatial material queries now preserve whole capsules in bounded cells, using
exact body-to-cell distances to cull irrelevant groups. The full staged replay
and 10x browser check pass in `runs/staged_spatial_bounds_v1`; all 166 segments
of a saved Palace drawing also pass, excluding its approach/withdrawal/travel.
Evidence: `research/blondie/material_queries.json` and `palace_collision_profile.json`.
Additional full cube and Palace forward trials run in `runs/cube_wireprint_spatial_v1`
and `runs/palace_hybrid_spatial_v1`. They retain unchanged geometry and use the
new query accelerator and forward checkpoints. `/tmp/run_spatial_build.py` launches
their browser checks only after full verification. The older Palace forward worker
was retired after the replacement matched its nine-section accepted prefix; its
results remain in `runs/palace_hybrid_diverse_v1`. The older generated-cube worker
stopped at 90/225 sections with a duplicate `arm` argument in rejection reporting.
Its terminal receipt is `runs/cube_wireprint_diverse_v1/terminal_failure.json`.
The reporter now retains the full validator evidence under `validation` and
continues to another motion candidate. Six focused section tests pass.
`research/blondie/replay_rejection_e2e.json` also records full real planning and
replay after one explicitly test-only forced rejection; its runner is preserved
in `research/blondie/check_replay_rejection_e2e.py`.
The cube, bottle, and short-restart Palace runs were restarted with that fix;
their logs are `runs/<run-id>_reporter_fixed.log`. Revalidate processes before
starting another trial; a launch receipt alone does not prove a live worker.
An exact restart split is now supported by `planning/sections.py`: one cubic is
replaced by two exact subcurves, with a deposited rejoin node and no new ground
anchor. Grouping respects the stop and shared-source credit waits for both pieces.
The four-section staged case passes complete independent replay (946 intervals)
and 10x browser playback; see `research/blondie/restart_split_e2e.json` and
`scripts/check_restart_sections.py`. Restart-location selection is still explicit.
`runs/palace_hybrid_restart_v1` tests a split at 1/8 of the problematic early
zigzag leg, prompted by a sampled posture jump. Its independent geometry and
source audit is `research/blondie/palace_restart_split.json`; full robot feasibility
remains pending. `runs/palace_hybrid_restart_order_v1` changes only construction
priorities so the split pieces stay adjacent in the original order. Its identical
physical graph and unchanged section payloads are audited in
`research/blondie/palace_restart_order.json`; imported motions are revalidated.
`runs/palace_hybrid_short_restarts_v1` tests six additional deposited junctions
across the two difficult legs. It keeps every source curve and original node,
uses 1,224 sections, and passes final and early-release structural screens.
`research/blondie/palace_short_restarts.json` binds the exact-geometry audit;
complete robot feasibility is still pending. The earlier finite orientation and
conservative lookahead probes did not yield an accepted path. See
`research/blondie/palace_path_diagnostics.json` before repeating those probes.
`runs/palace_hybrid_neighbor_order_v1` preserves the original graph and tries
section 5 before section 4, which appeared as an approach obstacle. The eight
unchanged preceding motion proposals are revalidated. This is an order candidate,
not a completed pair or build; preparation and geometry/source bindings are in
`research/blondie/prepare_palace_neighbor_order.py` and `palace_neighbor_order.json`.
The optional `minimum_anchor_spacing` generator setting addresses very close
anchors on unequal contours and leaves the closing diagonal unprinted. The
separate `palace_hybrid_spaced_source` graph retains the original contour shape
within 0.013 mm and all 18 selected WIR3D curves, with 605 sections; it fails the
unchanged final deflection screen at 1.256 mm and is not a robot build candidate.
See `research/blondie/palace_spaced_framework.json`. The reproducible preparation
script `prepare_palace_spaced_framework.py` also accepts
`--framework-diameter-mm 1.2` for an explicit thicker-framework revision. That
revision passes the unchanged final screen at 0.857 mm displacement and was
tried as `runs/palace_hybrid_spaced_1200um_v1` (603 sections). Its receipt is
`research/blondie/palace_spaced_framework_1200um.json`; its log uses the run ID.
Preserve the rejected thin revision as well. The thicker revision still needs
all construction-state, robot, source-coverage, and full-replay acceptance checks.
A matching backward search is `runs/palace_hybrid_spaced_1200um_backward_v1`;
it tests artistic access with the whole revised framework present. Its geometry
hashes match the forward trial. `research/blondie/palace_spaced_source_audit.json`
also confirms identical mesh and imported WIR3D bytes and all 110 contours at
their original ten slice levels.
The spaced forward order is structurally rejected at `section-00524` after six
validated motions: that component connector is ordered before either of its
endpoints exists. This cannot be repaired by retrying robot configurations.
The planner now leaves such an order immediately; the real six-motion recheck
is in `research/blondie/fixed_order_rejection.json`. The old forward worker was
stopped with a checkpoint and an `invariant_order_stop.json` receipt.
`research/blondie/prepare_palace_supported_order.py` is constructing a separate
anchor-ready order on the identical graph, screening every provisional release.
Its progress is in `research/blondie/palace_supported_order.json`; only a complete
order may be passed to robot planning, and it does not establish robot feasibility.
The preparer accepts `--resume --maximum-seconds 21600` after a bounded attempt
ends; matching graph, construction and material hashes are required. The live
`/tmp/continue_supported_order.py` controller waits for the current preparer to
finish, resumes a time-limited attempt, and starts the full
`palace_hybrid_supported_order_v1` robot build only after a complete structural
order exists. Inspect `runs/supported_order_continuation.json`, its matching log,
and the actual processes before starting a duplicate. Full verification triggers
the repository 10x browser checker through `/tmp/run_verified_build.py`.
The exact-artwork restart variant is prepared and its backward robot search has
started in `runs/palace_hybrid_art_restarts_v1` (885 sections). Preparation is
reproducible with `research/blondie/prepare_palace_art_restarts.py`; the bound
geometry audit is `research/blondie/palace_art_restarts.json`. Every non-artistic
member is unchanged; artistic subcurves retain their original source intervals,
with no new material or ground anchors. Its final-frame screen passes at 0.857 mm.
That screen does not certify individual restart support or any robot motion.
The launch receipt and log are `worker_launch.json` in the run and
`runs/palace_hybrid_art_restarts_v1.log`; inspect the actual process before reuse.
The motivation is a finite access diagnostic: section 602 admits 834 clear tool
orientations and 54 reachable clear starting poses, but the sampled constant-axis
set has no survivor at its third curve point. Evidence is in
`runs/palace_spaced_seed_probe_v1/seed_probe.json` and
`runs/palace_spaced_dense_constant_v1/comparison.json`. Do not infer impossibility
or a complete section from either starting-pose or constant-axis results.
Both finite access probes have finished; the starting-pose trial rejected all
eight attempted seeds. `research/blondie/palace_dense_access_probes.json` binds
the reports and archived reproduction scripts.

Newly started searches compact their intermediate structural cache entries by
omitting only `nodes` and `edges`; all scalar decisions/reasons remain, and the
full-graph entry still retains geometry. The real forward/backward check in
`research/blondie/compact_structure_cache.json` passes independent replay at
545/590 intervals. Both runs also pass full 10x browser playback in
`compact_cache_forward_browser_browser_replay.json` and
`compact_cache_backward_browser_browser_replay.json`. The reproduction driver is
`research/blondie/check_compact_structure_cache.py`; its report binds the exact
executed snapshot and current planning code. `structural_cache_size.json`
measures serialized report size only, not actual peak-memory or runtime savings.
Beam-element reuse is now separately bounded at 32,768 entries in `structure.py`.
Keys include both endpoints, diameter, elastic modulus, Poisson ratio, and density;
cached arrays are read-only. Full assembly/loads/supports/solves remain uncached.
`research/blondie/check_structure_element_cache.py` compares ten real structural
cases with caching enabled/disabled. Every output field matches, including full
Palace and partial Palace screens plus changed geometry/material/load/support cases.
The fresh `staged_beam_cache_v1` build passes 590 replay intervals and complete 10x
browser playback. `structure_element_cache_e2e.json` binds its byte-identical plan
and validation report to the pre-cache baseline; `structure_element_cache.json`
records local screen timings without claiming a full-planning speedup.
Polynomial bounds in `timing.py` now cache stationary roots by polynomial class,
coefficients, domain, and window, without changing the extrema calculation or
acceptance tolerances. `research/blondie/polynomial_root_cache.json` records 3,280
exact range comparisons and identical full 590-interval replay results with and
without caching. `runs/staged_root_cache_cached_v3` also passes complete 10x browser
playback; see `root_cache_browser_browser_replay.json`. Nine existing timing and
smooth-audit tests pass. The v2 diagnostic's copied run ID caused the viewer to
select another run; its browser timeout remains recorded in that run's
`diagnostic_failure.json`. The reproduction driver fixes copied metadata, and the
browser checker now rejects mismatched run IDs and wrong browser selection.
Already-running workers retain their loaded code; these timings do not establish
a speedup for a complete generated build.

The dense restart checks now isolate a specific artistic self-occlusion: at the
failed point of `section-00602-part-5`, 32 sampled tool poses clear the completed
framework, but `section-00602-part-2` blocks every one. No sampled pose clears all
other artwork there. `research/blondie/palace_restart_access.json` binds four
diagnostics and exact driver snapshots; `palace_art_blockers.json` has the material
witnesses. This is order guidance, not an infeasibility or robot-access proof.
Later checks at the next suffix expose a headroom distinction. At 1.1 mm, sampled
orientations suggest parts 1 and 3 block each other's starts; the active solver
nonetheless finds a part-1 pose with 1.094 mm clearance against the same material.
It passes the unchanged 1 mm requirement. Repeating part 1 at 1 mm finds eight
sampled start orientations. `research/blondie/palace_suffix_access.json` binds both
thresholds, the exact-pose check, and archived diagnostic drivers. Do not promote
the 1.1 mm screen to an infeasibility claim. An optional 883-section trial in
`runs/palace_hybrid_grouped_restart_v1` joins source parts 1--3, preserving all
geometry and source coverage while removing two restart boundaries. Its
predecessor structural screen passes; full robot validation remains pending.
Preparation and bindings: `research/blondie/palace_grouped_restart.json`.
`runs/palace_hybrid_art_order_v1` tests giving the blocker last priority. Its graph
and every other action field are unchanged; removing that section leaves a passing
0.8571 mm predecessor screen. Preparation is reproducible through
`research/blondie/prepare_palace_art_order.py`, with bindings in
`research/blondie/palace_art_order.json`. Its full backward robot search has started;
check `worker_launch.json`, its log, and the actual process before another launch.
The additional `runs/bottle_hybrid_backward_v1` trial checks
artistic suffixes against the completed bottle framework. Consult the refreshed
`runs/current_goal_workers.json` and `research/blondie/generated_build_progress.json`
for process hints and progress; neither snapshot replaces live-process checks.
Shared-source provenance now survives grouping and attachment splitting. Export
and replay credit a shared interval only after all its physical members complete.
`research/blondie/staged_shared_regression.json` records a real three-section build
with shared detail and no extra extrusion; `palace_source_provenance.json` checks
the actual Palace sources without changing its running physical geometry.
Browser evidence: `research/blondie/section_viewer_check.json` covers the curve,
graph, construction and motion stages; `cube_browser_replay.json` records 10x playback.
New browser checks no longer require a temporary script: run
`npm --prefix web run check:replay -- <run-id>` with the local Node on PATH and
the viewer running. `web/scripts/check-replay.mjs` requires a fully verified build,
matches served artifacts against disk, and checks the whole 10x playback horizon.
`research/blondie/browser_replay_check.json` records the complete CLI regression
and both full-cell and completed-structure captures.
Existing live wrappers still use their original temporary browser runner.

The clean fixtures in `research/primitive_fixtures/` are the next planning inputs:

| Fixture | Dimensions (mm) | Members / reference curves |
| --- | --- | --- |
| Cube | 80 × 80 × 80 | 12 / 12 |
| Triangular prism | 80 × 60 × 80 | 9 / 9 |
| Bottle | 50 × 50 × 120 | 28 / 8 |
| Torus | 120 × 120 × 30 | 32 / 8 |

Each contains `model_mm.stl`, `stroke_graph.json`, `pen_strokes.json`, and
`build_order/preflight.json`. STL coordinates are mm; graph coordinates are m.
Their reference curves deliberately isolate planning from automatic abstraction.
All have complete replay with intermediate release checks under uncalibrated
cooled-strand, rigid-weld, clamped-ground assumptions. Complete automatically
generated builds remain a separate acceptance requirement.

Historical runs are preserved for diagnosis, not mandatory starting prefixes:

| Run | Saved state at handoff |
| --- | --- |
| `runs/palace_15cm_one_arm` | `MOTION_BLOCKED`; 15/64 strokes complete; `draw-0029` stopped at samples 163→164 |
| `runs/general_torus_one_arm_base_first` | `MOTION_BLOCKED`; 11/19 strokes complete; `draw-0006` stopped at 106→107 |

The Palace final graph has 724 members and an ill-conditioned structural solve.
The older Torus graph has 91 members and fails the provisional structural model;
its predicted 29.2 mm displacement exceeds the linear model's validity range.
Neither result establishes physical impossibility. The clean Torus fixture is a
different, explicit reference graph; do not confuse their results.

## Remaining implementation and validation

1. Finish the solid-STL-derived cube, generated hybrid bottle, and Palace robot
   builds. The sparse reference fixtures remain diagnostics; their success does
   not replace the generated-object acceptance cases.
2. Preserve the audited graph/source bindings in `final_graph_preflights.json`.
   Generation, attachment, exact coverage and final structural preflights pass
   for the prepared graphs. Revalidate those bindings for any revision. Support
   additions create an explicit graph revision and restart order search; never
   insert them silently mid-plan.
3. The `(P, q_arm, k)` search is now in `planning/indexed_build.py`, reusing
   `planning/regression.py`. Retain its alternatives, source intervals, stage
   precedence, checked quarter-turns, and both home returns while finishing the
   larger cases. Bounded failed samples do not prove infeasibility. Pose solvers
   now check deadlines inside optimizer evaluations so one sample cannot consume
   the whole search budget. The resumed Palace worker logs to
   `runs/palace_turntable_v1_resume.log`; its stop/resume receipt is in the run.
4. The Flexiv/pen cuRobo adapter supplies batched IK and pen-off candidates.
   Keep exact extrusion tracking, growing-material collision checks, and
   structural checks in the existing validation path. GPU IK success is not a
   construction or motion certificate.
5. Independently replay each saved timed plan: exact source coverage, supported
   joins, release states, extrusion, travel, and home return. Complete its 10x
   viewer playback. Keep the four focused turntable checks and existing core
   tests; do not grow a broad new suite.

Useful code (all relative to `src/pen_art/`):

| Responsibility | Existing modules |
| --- | --- |
| Goal preflight / mechanics | `planning/build_order_preflight.py`, `structure.py` |
| Exact curves, intervals, starts | `curves.py`, `strokes.py`, `construction.py` |
| Search/state patterns | `planning/sequence.py`, `planning/coupled.py` |
| Robot, material, motion | `planning/cell_planner.py`, `collision.py`, `scene.py`, `rrt.py` under `planning/` |
| Rejoin/support semantics | `planning/stroke_restarts.py`, `planning/endpoint_support.py` |
| Coverage and playback export | `planning/source_coverage.py`, `planning/full_build_export.py` |

`construction_preview` is primarily a topology preview. `SingleArmBuild` follows
a fixed selection. `coupled.plan_sequence` backtracks but expects complete drawing
paths. Reuse their components; none is the new coupled regression planner.
See [the algorithm review](archive/2026_1011_caelan_build_order_plan.md) for details.

## Installed environment and commands

- `.venv/bin/python`: Linux Python 3.12 planner plus cuRobo/PyTorch.
- `.venv-wir3d/bin/python`: separate working WIR3D environment; preserve it.
- RTX 5090, 32 GB; repaired driver `595.99.02`. CUDA computation passes.
- cuRobo commit `78fd485fa82d9b9a063fb4985e371814587e666a`, current v2 API.
  Its bundled Franka IK check passed 16/16 targets. The new Flexiv benchmark
  passes 46/46 fixture targets: `research/blondie/flexiv_ik_benchmark.json`.
- Node: `export PATH="$PWD/tools/node/bin:$PATH"` before npm commands.
- Fresh host-local robot/tool cell: `research/planning_cell/cell.yaml`.
  Do not resume Mac checkpoints by rewriting paths/hashes or weakening validation.

```bash
# Read-only preflight on an existing fixture:
.venv/bin/python -m pen_art.planning.build_order_preflight \
  --run research/primitive_fixtures/cube --output /tmp/cube-preflight.json

# Only if regeneration is needed; separate output preserves tracked fixtures:
.venv/bin/python scripts/generate_primitive_fixtures.py --output research/primitive_fixtures_linux
.venv/bin/python scripts/prepare_single_arm_cell.py

# Complete reference build, using GPU IK proposals and independent replay:
.venv/bin/python -m pen_art.planning.section_build \
  --fixture research/primitive_fixtures/cube --run runs/cube_sections_v2 --gpu

# Real interleaved-source regression and negative replay checks:
.venv/bin/python scripts/check_section_interleaving.py

# After check_shared_source.py; choose a fresh output run:
.venv/bin/python scripts/check_section_checkpoints.py --run runs/checkpoint_check_v2
```

The existing `pen_art.build` command produces automatic geometry and can call the
legacy motion pipeline. It is not an entry point for the pending Caelan adaptation.

## Viewer and assets

The read-only viewer runs in tmux `pen-art-viewer`. Inspect current process handles
before launching duplicate planning jobs. Current planner sessions are
`pen-art-cube-forward`, `pen-art-cube-backward`, and `pen-art-bottle-forward`;
Palace is stopped at the documented model-access conflict. Each current run's
`live_worker.json` identifies its worker, but verify the process command line
before treating it as live. `runs/current_goal_workers.json` and
`research/blondie/generated_build_progress.json` are historical snapshots.
On Blondie open <http://127.0.0.1:8765/> for the three current objects.
From the Mac, use the existing <http://127.0.0.1:8766> tunnel or recreate it:

```bash
ssh -N -o ExitOnForwardFailure=yes -o ControlMaster=no -o ControlPath=none \
  -L 127.0.0.1:8766:127.0.0.1:8765 blondie
```

If the remote server is stopped, from the project directory:

```bash
tmux new-session -d -s pen-art-viewer -c "$PWD" \
  "$PWD/.venv/bin/python -m pen_art.server --root $PWD --port 8765"
```

The motion page includes 10× playback. Partial/rejected histories are inspectable;
they are not complete builds. The server cannot send robot commands.

Assets are under the same project root: the four original Palace GLB/STL files,
all `runs/` and `research/`, `external/` dependencies/assets, and built `web/dist/`.
Blondie also retains its 7.3 GB `wir3d_bundle/`, Linux environments, and `tools/`
(including Blender and Node). They are intentionally outside Git.
The Mac environment and node_modules are not useful Linux assets; use installed
Linux dependencies instead. Remote-only data is preserved.

[Asset inventory](../research/blondie/handoff_assets.json) and
[verification report](../research/blondie/handoff_verification.json) record paths,
sizes, and SHA-256 checks. Environment details are in
[`research/blondie/`](../research/blondie/). Continue development on Blondie and
commit source changes through the private repo.


## Continuous actions and Phase 1 (2026-10-11 update)

Current implementation uses whole connected WirePrint contours/zigzags, 30 mm/s
as the provisional draw ceiling, internal vertex cooling/feed events, and one
readiness clock. Phase 1 is the indexed default. Cache reuse keeps exact accepted
interpolants for matching requests, adapts changed shapes, prefers cached table
indices, and bounds proposal effort across all roots/indices. Per-leg runs above
are historical evidence; new execution must not automatically split every leg.

Validation: 50 focused tests and the three-leg real-cell integration pass. See
`runs/continuous_zigzag_integration_v3/acceptance.json`: cold 33.06 s, warm 7.64 s,
width +2% 12.72 s (planning plus independent replay), one action/three members,
one approach/withdrawal and one internal hold. Warm backward reuse also passes.
All three published `phase1_connected_zigzag_{cold,warm,variation}` playbacks pass
10x browser verification and show execution-time breakdowns. These are small
integration diagnostics, not full-object or physical-print qualification.

Full generated builds are paused under `runs/cube_continuous_phase1_v1`
(17 actions/436 members) and `runs/bottle_continuous_phase1_v1` (25 framework
actions). The two remaining older full-object searches are also stopped; consult
the pause receipt above. Do not infer completion from a prepared
local motion. Full-object cold/warm and 14-minute acceptance are still pending.
The website bottle STL was not found; `research/blondie/wireprint_bottle_asset_search.json`
records the primary-source search and identifies our bottle as a procedural
fixture. Do not describe it as the exact paper asset.

The source grouping audit and member-prefix structural audit are under
`runs/continuous_actions_a_v1`. Keep the 1 mm clearance and 0.3 mm tracking limits.
A small integration exposed a floating-point readiness comparison defect, now
fixed with 1 ns absolute timestamp tolerance while keeping IDs/order exact.
Do not restore the old duplicate cooling gaps or per-root cache budget resets.

Offline whole-path cache preparation now has a separate CLI:
`scripts/precompute_motion_cache.py`. It admits only independently checked
`SELF_STATIC_ONLY` records. Runtime still adapts to the exact source and checks
current/progressive material, timing, cooling and connecting motions; those seed
records cannot inherit a fully validated motion's clock shortcut.
`scripts/check_motion_cache_coverage.py` reports availability for actual actions,
directions and all four stationary table indices; `--require-zigzags` makes missing
families explicit. The original active library had two base contours and no
whole-zigzag records. The actual cube requires 26-leg zigzags; the bottle has
14-, 12-, 10- and 8-leg families. Precomputation and bounded reuse probes are the
current task, not full-object search or Phase 2 training.

The default library is now populated: ten new whole-zigzag seeds (two postures
for each of the five families) plus the two existing contour records. All new
seeds use stationary table index 0; coverage at other indices is not claimed.
[`runs/motion_cache_catalogue_v1/promotion.json`](../runs/motion_cache_catalogue_v1/promotion.json)
records preserved/added IDs and imported self/static clearance samples.
The generator supports `--publish-library` and idempotent activation of an
existing catalogue with `--catalogue`. Keep compatible-family availability,
actual source adaptation and full material/process acceptance distinct.
The 20-action bounded audit validates five exact-source reuses; 15 changed
sources remain unresolved within three seconds each. The local cube probe
retrieves a full cached zigzag but runs out of repair budget with growing-material
clearance unresolved. These are recorded in `adaptation_coverage_20_v1.json` under
the catalogue and `runs/whole_zigzag_cache_runtime_active_v1/result.json`. The
active-library probe records the exact parent seed and zero cold-solver calls.
Do not describe
the seeds as accepted full print motions or claim all 20 actions adapt successfully.
Coverage reporting now fingerprints each stored trajectory once per pass and
reuses parsed descriptors; it does not repeatedly serialize the 23.5 MB contour
clock for every request. The full bottle coverage report retains all 8,752
start/direction/index requests and takes 4.10 s in
`runs/motion_cache_coverage_performance_v2/metrics.json`; repeat reporting reuses
the snapshot. This is reporting overhead, separate from motion validation.
