# Parallel Phase 1 cube planning — results

Recorded 2026-10-12. Simulation only; no physical print. The first run and
subsequent compute-cleanup measurements are recorded here. The implementation checklist
is in [PLAN.md](PLAN.md); continuation notes are in [HANDOFF.md](HANDOFF.md).

## Headline

Under the final C1–C6 implementation, **16 CPU workers completed the cube in
7.5 minutes**, versus 16.0 minutes with four: **53% less compute time, 2.13×
faster**. Sixteen is fastest among tested counts, with a 19.21 GiB observed
peak process-tree PSS. Modeled printing remains separately 32.0 min.

| Workers | Setup | Planning | Assembly | Export + replay | Command total | Peak tree PSS |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 4 | 8.4 s | 456.9 s | 2.1 s | 489.7 s | 962.5 s (16.0 min) | 6.20 GiB |
| 8 | 8.6 s | 278.4 s | 2.1 s | 311.8 s | 606.2 s (10.1 min) | 10.97 GiB |
| 12 | 8.6 s | 232.8 s | 2.1 s | 271.9 s | 519.7 s (8.7 min) | 15.23 GiB |
| 16 | 8.5 s | 185.5 s | 2.1 s | 249.7 s | 451.6 s (7.5 min) | 19.21 GiB |
| 32 | 8.4 s | 158.5 s | 2.1 s | 279.9 s | 454.5 s (7.6 min) | 36.86 GiB |

Runs were sequential with matched frozen cache, profile, source and budgets.
Memory is the maximum observed simultaneous process-tree PSS sampled every
two seconds. Every run independently passed 39 drawing sequences, 111,584 raw
full-robot intervals, source/process/support/home audits and the same selected
deposits/final material. All plan SHAs are
`a7fdaf9ebb7960fc844992474bf1ec0dad874f4b53d2d8fffd5de90f9281d2f1`, matching
the hardened-codec plan already checked through complete browser playback.
The queue verified source/profile/library hashes unchanged after the sequence.

Evidence: `runs/compute_cleanup_cube_high_v3_benchmark.json` and
`runs/compute_cleanup_cube_high_v3_queue.json` (`PASSED`). Command totals include
startup/teardown and exit polling in addition to the internal phase timers.
No benchmark remains running. All C1–C6 gates passed; recovery testing was
removed from scope by the user. GPU investigation stopped for handoff:
[findings](2026_1012_gpu_validation_findings.md). No GPU speedup was measured.

Sixteen-worker replay: 2.0 s loading, 0.03 s context setup, 40.3 s source and
structural audits, 59.3 s parsing/path checks, 0.4 s schedule checks, 19.8 s
deposit journals and 93.3 s collision replay. Parsing/path checks include 42.0
summed worker-seconds parsing and 724.0 auditing; these are not wall times.
Export and surrounding bookkeeping account for the rest of the combined phase.

The separately requested 32-worker run used identical source/profile/library
hashes and produced the same plan SHA; its guard passed. Evidence:
`runs/compute_cleanup_cube_32_v4_benchmark.json` and `_queue.json`. It saved 27 s
in planning but added 30 s in export/replay. Collision replay took 143.4 s,
versus 93.3 s with 16. Total differs by only 3 s in these single trials; there
is no meaningful speed gain, while observed peak PSS almost doubles. Sixteen
is the better measured balance for this cube on the 24-logical-CPU host.

## What export and full replay do

**Current implementation:** these are now separate commands. Planning saves the
exact schedule, the independent `planning.cell_replay` command verifies the full
robot, and the viewer only animates a saved schedule with its matching verdict.
New full schedules cannot play before verification passes. The table below is
the historical combined phase, before this separation.

Saved-schedule check `runs/cube_modular_playback_v1`: export **6.48 s**, separate
16-worker raw verification **216.14 s**. Exact actions, clocks, events and timing
match the original cube; verification passed 39 actions/111,584 intervals and
the identical final material. Browser playback made no backend requests and
left both artifacts unchanged; missing verification withheld the Play controls.
This check did not rerun planning, and is not a new end-to-end speedup result.
Receipts: `modular_playback_receipt.json` and `modular_browser_receipt.json`.

At 16 workers the combined phase is 249.7 s. It packages the complete clocks,
source/process metadata and feed events, writes compact viewer JSON and packed
arrays, releases planning-only objects, and reopens the saved plan for fresh
full-robot validation. Replay is computation, not real-time animation.

| Part | Wall time |
| --- | ---: |
| Export and other overhead (residual, not an isolated export timer) | 34.9 s |
| Load the packed plan | 2.0 s |
| Source, structural and stage audits | 40.3 s |
| Parse clocks; audit tracking, joint limits, speed, acceleration and process | 59.3 s |
| Derive deposit journals for parallel material prefixes | 19.8 s |
| Full robot/tool/static/growing-material collision replay | 93.3 s |

The export residual includes small context/schedule checks and bookkeeping.
Raw replay explicitly disables invariant-proof reuse. Planning checks pen-only
scope; replay restores full robot geometry. It audits 111,584 intervals, not
just 39 path endpoints, and derives conservative deposits again during collision
validation. Journals let workers begin later intervals with the exact preceding
material; they grant no collision proof. Each worker still builds its own FCL
objects/index and checks its assigned continuous paths, including their new wire.

At 16 workers, replay interval validation contains 300.7 summed worker-seconds
for deposition enclosures and 257.6 for fixed orientation, within 1,235.4 total
interval-validation worker-seconds. These nested times cannot be added as wall
times. Clock parsing takes only 42.0 summed worker-seconds; path audits take
724.0. Large-file reading is therefore a smaller cost than geometric/dynamic
proofs. The old contact-geometry bottleneck was removed by C1 (0.01 summed
worker-seconds here).

Increasing workers also reduces replay chunks: the current rule targets 871
edges per chunk at 16 workers and 435 at 32, roughly 153 versus 271 action-edge
chunks. More chunks mean more material-prefix/setup work. Together with 32
workers competing on 24 logical CPUs and larger memory demand, this is a likely
cause of the collision-stage regression; its exact contribution is not isolated.

Further reuse would require accepting source/speed/orientation/deposition proofs
bound to unchanged clocks and contexts while still checking full robot material
and travel freshly. That is different from the deliberately fresh raw benchmark.
Changing validation policy or chunk sizing requires a separate controlled result;
the 32-worker trial changed neither implementation nor acceptance policy.

## Earlier controlled baseline, before final storage hardening

After C1–C5 cleanup, the controlled 1/2/4-worker comparison passed full-robot
verification for every run. Four workers completed in **16.0 minutes**, versus
52.7 minutes with one worker: **70% less compute time, 3.29× faster**.

| Workers | Setup | Planning | Assembly | Export + replay | Command total | Peak tree PSS |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 8.7 s | 1,623.3 s | 2.1 s | 1,524.2 s | 3,161.9 s | 2.14 GiB |
| 2 | 8.6 s | 829.7 s | 2.0 s | 843.1 s | 1,689.1 s | 4.18 GiB |
| 4 | 8.6 s | 457.4 s | 2.1 s | 488.4 s | 961.3 s | 6.92 GiB |

Runs were sequential, CPU only, with the same frozen cache and 240 s action
budget. Memory is the maximum observed simultaneous process-tree PSS sampled
every two seconds. Each passed 39/39 actions and 111,584 full-robot intervals;
the exact plan hash, selected deposits and final material match across runs.
Evidence: `runs/compute_cleanup_cube_scaling_v2_benchmark.json`.

These timings precede the subsequent codec binding fix and shared final-replay
journal hardening. All three exact schedules passed fresh raw full-robot replay
under that revision (`runs/compute_cleanup_cube_codec_v2_codec.json`). A real
obstruction probe rejected the blocked path, and a cache-free base-action
proposal passed pen and full-robot local checks
(`runs/compute_cleanup_blocked_cold_v1.json`). Full 10× browser playback passed
without errors; original acceptance
receipt `runs/compute_cleanup_completion_v2.json` is `PASSED`.
The subsequent current-revision comparison is recorded above. Do not combine
the old serial timing and new sixteen-worker timing into a matched speedup.

## First complete build (before compute cleanup)

The generated solid-STL cube framework was planned and independently verified
end to end in **23.4 minutes** on 8 worker processes.

- Run: `runs/cube_parallel_trial_v2`
- Status: `FULL_BUILD_VERIFIED`, 39/39 whole actions
- Final gate: raw full-robot replay, `CERTIFIED_GEOMETRIC_REPLAY`,
  111,584 checked intervals, arm and turntable home, no table turns needed
- Modeled print duration of the resulting plan: 1,917.9 s (32.0 min)

This is the first complete generated-cube build under the current contract:
whole connected actions, 27° maximum descents, one fixed upright pen
orientation per action, 1 mm clearance, 0.3 mm tracking.

## The 23.4-minute run

Conditions: 8 workers, CPU only, frozen motion library
`runs/motion_library_phase1_upright27_v1`, 240 s budget per planning job, one
numerical thread per process, fresh run directory, cold path certificates.

| Phase | Wall time |
| --- | ---: |
| Setup: planner and library load | 69.7 s |
| Parallel planning, 39 actions | 281.3 s |
| Assembly of the selected paths | 0.6 s |
| Export and full-robot replay | 1,051.5 s |
| **Total** | **1,403.1 s (23.4 min)** |

Planning outcome:

- 118 jobs (39 proposals, 39 checks, 40 connections including the return
  home); all accepted. No rejection, repair, stale result or lost worker.
- All 39 candidates came from the cache: 35 retained exact clocks and 4
  adapted seeds. The cold fallback was never needed.
- Every action's validated deposit matched its predicted deposit within
  1e-9 m.
- Workers peaked at about 25.1 GB in total (proportional set size).

Summed worker time by planning stage, to show where the work is:

| Stage | Worker-seconds | Largest parts |
| --- | ---: | --- |
| Propose | 464 | predicted deposits 166, seed retrieval/adaptation 101, tracking and clock 83 |
| Check | 1,040 | material validation 527, timing audit 434 |
| Connect | 268 | source coverage 163, transit validation 68 |

The export-and-replay phase of this run used the first version of the parallel
replay, in which structural audits and clock parsing were still serial.

## Replay-only measurements on the same plan

After that run the replay's audits were parallelized and long actions were
split into edge-boundary chunks. Only the replay was re-run, on the identical
plan, with 16 workers.

| Replay variant | Total | Collision phase | Verdict |
| --- | ---: | ---: | --- |
| One job per action | 664 s | 542 s | Certified |
| 256-edge chunks | 581 s | 458 s | Certified |

Both report 111,584 intervals and the same final material: 142,102 capsules,
geometry hash `31ba57ec525f250bce09265bf31b60a97437e7237143df7e215611f0b8ede8d5`.
The other phases of the chunked run were 40 s loading and structural audits,
60 s clock parsing and path audits, and 20 s deposit journals.

The collision phase costs roughly 90 CPU-minutes. That figure is an estimate
from the 16-worker run, not a serial measurement.

A later change that sizes chunks by worker count has not been measured.

## Serial versus parallel replay

On the previously certified 5-action cube prefix, replayed raw with the full
robot:

| Workers | Wall time | Verdict | Intervals |
| --- | ---: | --- | ---: |
| 1 | 190 s | `CERTIFIED_PREFIX_REPLAY` | 12,029 |
| 4 | 92 s | `CERTIFIED_PREFIX_REPLAY` | 12,029 |

Another job was running on the machine at the time, so the ratio is rough.
Evidence: `runs/replay_parallel_check_v1/comparison.json`.

## Limits of the first run

- **Historical baseline limits.** No 16-worker end-to-end run completed. Before
  the controlled comparison above, the only
  serial reference is the earlier sequential worker, which had 36/39
  pen-candidate actions after more than two hours of planning with the replay
  still outstanding. It used a GPU and older code, so it is context, not a
  baseline.
- **Repair is untested on the real cell.** No job failed, so candidate
  replacement, downstream invalidation, the cold fallback and the
  replay-failure feedback loop ran only in unit tests.
- **Historical scope:** the real-cell blocked-material probe had not run yet.
  Subsequent recovery testing was dropped by the user and is not a C6 gate.
- **Publication update:** the run is now a Current build on port 8765 as
  **Cube · complete 39/39 · Phase 1 parallel**. Full 10× browser playback passed
  through the 1,917.9 s horizon with no errors; receipt:
  `runs/cube_parallel_trial_v2/viewer_browser_browser_replay.json`.
- **Cube only.** The bottle has not been attempted with this pipeline.

Three attempts at the benchmark sequence were interrupted, twice by session
restarts and once by a machine reboot at 13:38. The incomplete
`runs/cube_parallel_high_v1_w16` directory is from the last attempt and is not
evidence of anything.

## Checks

Current critical suite: 25 Python and four browser tests pass, with Ruff and
`git diff --check`. The first run used the earlier 299-test suite.

## Evidence files

| File under `runs/cube_parallel_trial_v2/` | Contents |
| --- | --- |
| `parallel_build.json` | Phase times, job outcomes, memory, per-action selections |
| `section_build.first_8_workers.json` | Build report from the 23.4-minute run |
| `offline_dual_arm_plan.json` | The verified 39-action plan |
| `offline_dual_arm_validation.first_8_workers.json` | Replay from the 23.4-minute run |
| `offline_dual_arm_validation.per_action_16_workers.json` | 16-worker replay, one job per action |
| `offline_dual_arm_validation.json` | 16-worker replay, 256-edge chunks |

## Reproduce

Use a new run directory, and launch in tmux so a session restart cannot kill it.

```bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python -m pen_art.planning.section_build \
  --cell-source-run runs/cube_nozzle27_phase1_v1_source --run runs/new_parallel_cube \
  --profile runs/cube_nozzle27_phase1_v1/profile.yaml --progression-only --workers 8
```

Current validation uses `scripts/complete_compute_cleanup.py`; inspect
`runs/compute_cleanup_completion_v2.json` before launching anything.
The old `runs/cube_parallel_followup.sh` is historical and must not be relaunched.
