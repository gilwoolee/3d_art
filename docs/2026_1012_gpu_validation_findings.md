# GPU validation beyond IK

Recorded 2026-10-12. Findings from current code, complete cube runs and cuRobo
source/papers. GPU validation is a proposed extension, not implemented or timed.
The CPU scaling comparison continues unchanged; recovery testing is out of scope.
Investigation stopped at the user's request for another agent to take over.
Read [HANDOFF.md](HANDOFF.md) for live CPU benchmark ownership. Keep
`src/pen_art/` unchanged until that matched comparison finishes. The CPU sequence
subsequently finished; its final results are in
[parallel cube results](2026_1012_parallel_cube_results.md). The findings below
retain the eight-worker profiling basis used during this investigation.

## Finding

The useful GPU target is **validation of cached trajectories**, not generating
more IK candidates. Batch forward kinematics, tracking/orientation/speed audits
and robot/material distance queries. Accept intervals directly when conservative
bounds prove clearance; refine or use the current CPU checker for uncertain ones.
Running GPU screening followed by every existing CPU check would duplicate work
on the already-clear cube and offer little benefit.

The RTX 5090 has about 32 GB VRAM and was idle during CPU benchmarking. Our
`planning/curobo_adapter.py` only supplies IK candidates; its collision geometry
and self-collision checks are disabled. The CLI currently disallows combining
`--gpu` with forked `--workers`. This needs integration, not another flag.

## Where the measured time goes

Current complete cube totals: 4 workers **962.5 s**, 8 workers **606.2 s**,
12 workers **519.7 s**. Each passed 39 drawing actions and 111,584 raw full-robot
intervals with the same final material. Sixteen workers are still running at
this checkpoint. Every candidate came from cache: 35 retained clocks, four
adapted seeds, no cold proposals.

Eight-worker measurements:

| Operation | Time | Measurement scope |
| --- | ---: | --- |
| Planning | 278.4 s | Wall time |
| Export and independent replay | 311.8 s | Wall time, includes export/bookkeeping |
| Replay loading and source/structural audits | 47.8 s | Wall time; excludes path audits |
| Replay clock parsing and path audits | 81.2 s | Wall time |
| Replay deposit journals | 29.1 s | Wall time |
| Replay collision phase | 118.4 s | Wall time |
| Planning material validation | 524.5 s | Summed worker elapsed time |
| Planning timing audits | 442.1 s | Summed worker elapsed time |
| Replay path audits alone | 557.3 s | Summed worker elapsed time |

Inside replay interval validation, 832.9 summed worker-seconds include 224.4 s
constructing deposition enclosures and 190.0 s checking fixed orientation.
These are nested costs; do not add them to the interval total. The remaining
418.6 s includes collision work and other interval bookkeeping, not a separately
measured distance-kernel cost. Contact geometry setup is already negligible
after C1: 0.009 summed worker-seconds in this replay.

Evidence: `runs/compute_cleanup_cube_high_v3_w8/{parallel_build,
offline_dual_arm_validation}.json`; totals are reported by
`runs/compute_cleanup_cube_high_v3.log`. Full PSS/comparison receipt is published
when the sequence ends. These data do not measure any GPU speedup.

## Concrete acceleration targets

| Target | What changes | Existing owner |
| --- | --- | --- |
| Batched FK and path audits | Evaluate polynomial states, link/TCP poses and Jacobians in tiles; share results between tracking, orientation, speed and collision checks | `clock_path.py`, `tracking.py`, `cell_replay.py` |
| Batched continuous clearance | Process many unresolved intervals together; evaluate midpoint geometry and conservative motion bounds, then subdivide only uncertain intervals | `collision.py:check_bounded_path`, `cell_planner.py:validate_interval` |
| Persistent material queries | Upload capsule geometry once, reuse its spatial index and select each query's exact deposited prefix | `material_journal.py`, `scene.py`, `parallel_build.py` |
| Shared deposition enclosures | Derive the selected path's conservative deposits once, bind them to exact geometry/context and reuse checked enclosures across stages | `cell_planner.py`, `path_certificates.py` |

The first target is useful even before replacing collision geometry: FK is
repeated in path audits, fixed-orientation checks, deposition and collision.
Polynomial joint limits and speed bounds must remain continuous checks; batched
samples alone do not establish them. Bernstein coefficient bounds and subdivision
are a possible GPU formulation, requiring equivalence/error validation.

## Use cuRobo selectively

cuRobo V2 provides standalone [forward kinematics](https://github.com/NVlabs/curobo/blob/main/curobo/kinematics.py)
and [robot collision checking](https://github.com/NVlabs/curobo/blob/main/curobo/collision_checking.py)
APIs. We can use these kernels on existing paths without switching to MotionGen
or changing the action order.

The installed implementation has three world SDF backends: cuboids, meshes and
voxels ([registry](https://github.com/NVlabs/curobo/blob/main/curobo/_src/geom/data/registry.py)).
A `Capsule` scene type does not imply a native capsule distance kernel. Our
142,102 deposited capsules therefore need a representation decision:

- **Preferred investigation:** a custom Warp capsule-distance query with a
  conservative spatial index and prefix/birth mask. Keep the existing capsule
  geometry and avoid constructing one FCL object per capsule in every worker.
- **Mesh alternative:** grouped conservative wire meshes with BVHs; qualify
  tessellation, containment and signed-distance behavior before clearance use.
- **Voxel alternative:** conservative occupancy/distance bounds with explicit
  voxelization/interpolation errors. Sparse thin wires must survive conversion;
  a coarse interpolated SDF is not automatically a 1 mm clearance certificate.

Do not turn every tiny deposited capsule into an independent cuRobo cuboid.
The current [world kernel](https://github.com/NVlabs/curobo/blob/main/curobo/_src/geom/collision/wp_collision_kernel.py)
launches sphere/obstacle pairs; a huge obstacle list can overwhelm the gain.
Conservative merging of near-collinear deposits is another possible reduction,
but must enclose the original volume rather than replace it with source lines.

## Preserve continuous clearance

Our CPU checker already uses the right acceptance pattern:

```text
clearance lower bound at an interval's midpoint
    > 1 mm + bound on motion over the interval + numerical allowance
```

For a GPU version, bound the original CAD geometry with verified enclosing
shapes, inflate those shapes by the interval motion bound and clearance margin,
and query the current material prefix. A proved clear inflated enclosure can
accept the whole interval. For self collision, account for both bodies' motion
or reuse the existing relative-motion bounds. Uncertain intervals subdivide or
fall back to exact CPU geometry; overlap of outer approximations is not proof
of a collision between the original shapes.

Ordinary cuRobo sphere fitting is approximate; coverage metrics do not prove
containment ([sphere fitting](https://github.com/NVlabs/curobo/blob/main/docs/reference/sphere_fitting.rst)).
Use certified outer covers for the arm and pen. The tapered nose needs a tighter
representation so unnecessary overlap does not send most drawing back to CPU.
Import only justified collision-pair exclusions from our existing model.

The [original cuRobo paper, section 3.4](https://arxiv.org/html/2310.17274v2#S3.SS4)
explicitly assumes linear sphere motion between waypoints. The installed V2
[sweep kernel](https://github.com/NVlabs/curobo/blob/main/curobo/_src/geom/collision/wp_sweep_collision_kernel.py)
also interpolates sphere positions and has a finite sweep-step budget. These
operations alone do not certify our curved joint polynomials. Keep the joint
envelope proof, including excursions between endpoints. Similarly, cuRobo's
activated collision cost is not an unrestricted signed-distance lower bound;
its zero-cost result needs a correctly inflated query and justified error bounds.

## Growing material and nozzle contact

Each query binds the selected candidate, exact predecessor deposits, table
index and progress within its own action. A capsule's birth/progress tag excludes
future material and includes the material the current CPU interval checks.
Candidate changes invalidate queries/results that used the replaced geometry.
Keep earlier layers represented; spatial bounds can prove them irrelevant.

The robot and pen housing see the full material. Only the tip receives the
scoped join-ball/work-surface exemption used by `transit.contact_validator`.
Do not mask a whole wire, layer or pen body because its tip is intentionally
touching. A GPU voxel/mesh representation must support this distinction or use
CPU fallback for the affected tip checks. Indexed table turns need their own
swept bounds; stationary drawing-prefix queries do not certify a turn.

## Execution and expected benefit

```text
CPU action coordinator + exact cached polynomials/material journals
                     |
       One persistent GPU process, initialized without CUDA fork
                     |
       Batched FK + interval bounds + current-prefix distances
                     |
        proved clear -> accept and retain the exact proof binding
        uncertain   -> GPU subdivision or targeted CPU checks
                     |
       Existing source/process/support/home acceptance and playback
```

Keep clocks, geometry, spatial indices and query buffers resident. Batch across
actions and intervals; transfer compact requests/results instead of a full scene
per query. A fixed tile pool can support CUDA graph reuse where shapes permit.
Reuse qualified self/static proofs from Phase 1; check changed material freshly.

Accelerating only replay collision cannot produce an order-of-magnitude overall
gain: even eliminating that entire 118.4 s phase would improve the 606.2 s run
by at most 1.24×. Hypothetically making the entire replay parsing/path-audit and collision
phases 10× faster would reduce it to about 427 s (1.42×), before transfer/fallback
overheads. These are arithmetic scenarios, not forecasts. Larger gains require
using GPU numerics during planning checks and deposition as well as replay.

The next useful measurement is on the same saved cube: report FK/audit time,
material-index construction, query throughput, refinement depth, CPU fallback
fraction and complete wall time including GPU startup/transfers. Compare to
fresh CPU results on clear, blocked, near-margin, changed-prefix/contact and
curved-path cases. Any false-clear invalidates acceptance. Reuse/extend the
existing critical contracts; no recovery test or new broad test suite is needed.

Open questions: verified arm/nose covers; capsule index versus bounded voxel
cost; floating-point error budgets; handling clipped tip contact; and how much
of real drawing still needs CPU refinement. These decide achievable speedup.
