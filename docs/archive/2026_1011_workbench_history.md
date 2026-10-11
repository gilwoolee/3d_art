# Pen Art

Textured mesh → artistic 3D curves → connected wireframe → one or two Flexiv arms with independent 3D pens. Surface filling and finishing are outside this pipeline.

Current priority: finish and replay the **15 cm Palace with one robot arm** end to
end, with no fabrication-time target or overall planning timeout. The active run
is `runs/palace_15cm_one_arm`, using
`configs/cell/one_flexiv_right_angle_lean.yaml`. It has one Flexiv model and one
pen; the second robot is absent. The original 15 × 15 × 9 cm geometry and
automatic supports are preserved. The pen retains the provisional right-angle
bracket and uses 10° inward lean as its initial orientation. One-arm motion
preparation can adjust the pen orientation to clear deposited material while
preserving the Cartesian stroke. Earlier two-arm and parked-arm comparisons are
stopped with their artifacts preserved.

```bash
MPLCONFIGDIR=/tmp/3d-art-mpl .venv/bin/python -m pen_art.planning.run_cell \
  --source-run runs/palace_15cm_two_pen_selection \
  --output runs/palace_15cm_one_arm \
  --config configs/cell/one_flexiv_right_angle_lean.yaml \
  --timing-config configs/timing_continuous_bounded.yaml \
  --resource-mode single --no-time-limit --maximum-allocation-repairs 64
```

`--no-time-limit` removes the overall and drawing-catalogue wall-clock limits;
it retains the fixed catalogue and bounded failure-repair loop. The webpage
shows one robot and provides full-build playback when a schedule is generated.
Simulation does not establish physical fabrication without measured pen and
robot calibration.

All 64 one-arm drawing paths have been generated. Their 6,082 saved poses
preserve the original tip targets and fixed 10° lean
(`research/palace_one_arm_catalogue_complete.json`). Complete-motion planning,
including approach, withdrawal, travel and growing-material replay, follows
path timing. This is not yet a complete validated robot build.
The repair first seeks a clear pen pose and then solves the arm's IK. Every
accepted repair must pass continuous tip tracking, growing-material clearance,
retiming and replay. The required 1 mm clearance is unchanged.
The generic `pen_art.build` entry point also accepts `--resource-mode single
--no-time-limit` with this one-arm cell profile; it skips the two-arm partition.

Earlier two-arm development results follow. The former
30-minute fabrication target is advisory; a longer complete build is acceptable.
Offline planning has a separate one-hour resumable allowance. The vertical-pen
run generated all 64 motions but failed shared-clock collision replay. The
right-angle branch, `runs/palace_15cm_right_angle_cell`, models a physical 90° bracket:
the nozzle stays vertical while each wrist is offset outward. The pen and bracket
dimensions are provisional. Its latest bounded batch stopped at a pen-body/post
obstruction after 8 of 64 motions; no successful complete replay has been
established. Its complete 128-path
catalogue preserves the original wireframe and vertical tip orientation. A
numerical timing failure is now isolated to its specific fixed mode, recorded
in `allocation_timing.json`, and excluded from allocation while other eligible
modes remain available. Resumption preserves timing results under matching
inputs. This does not establish that a failed timing mode is physically impossible.
127 of 128 modes are timed; the unresolved arm-1 mode has an eligible arm-2
alternative, retaining all 64 construction actions. Recorded nozzle/post
obstructions now produce conditional stroke-order constraints that are preserved
through final retiming. In **05 Two-arm setup → Material ordering constraints**,
blue highlights the stroke that needs access and bronze the blocking stroke that
must come later. The original Palace wireframe remains visible in gray.

The previous two-arm continuation was `runs/palace_15cm_right_angle_lean_cell`, using
`configs/cell/two_flexiv_right_angle_lean.yaml`. It preserves the same tool,
bases, 15 cm wireframe and clearance, with one fixed 10° inward nozzle lean.
That lean cleared the exact blocked start pose in a separate diagnostic
(`research/palace_right_angle_material_lean_probe.json`); complete drawing,
transit and shared-clock validation remain necessary. The vertical run is kept
intact. This is one fixed-catalogue continuation, not an open-ended angle search.
All 128 drawing paths are now generated. Their 12,164 saved IK samples retain
the original TCP targets and the configured 10° nozzle lean; the source mesh,
stroke graph and construction actions are unchanged. The run is timing those
paths before allocation and complete-motion validation.

Shared-clock replay now saves the exact material capsules at verified event
boundaries. Repeating the replay command resumes that prefix only when the full
plan, inputs, live collision meshes, tool geometry, settings, software and
checking implementation match. Changed schedules start from zero; a partially
checked interval is never reused. The workbench reports the resumed job time.
One focused interruption/resumption regression passes, together with a
two-process check of the first two real Palace actions (438 checked intervals).
The two earlier comparison replays were already running before this change and
have been left uninterrupted. The 10° run resumed all 51 cached drawing modes
in a new process so its future replay can use these checkpoints.

A fixed base-mount study used the same Palace, tool and 12 stroke points per arm.
Upright bases gave 24/24 clear individual poses and 5/6 clear paired poses;
45° yaw gave the same counts; 45° inward base tilt gave 17/24 and 2/6.
The minimum best-solution joint margins were 10.2%, 10.1% and 2.8% of joint range.
This favors upright bases for this placement, not a universal mounting optimum.
Open **05 Two-arm setup → Base mounting assessment** in the right-angle run to
compare the actual arm configurations. Static samples do not certify paths,
growing-material clearance, the mounting wedge, gravity or payload dynamics.

Repeating the same three-option assessment with the active 10° pen lean gives
24/24 individual and 6/6 paired poses for both upright and 45° yaw mounting.
The 45° inward base tilt gives 7/24 individual and 0/6 paired poses. The
assessment now uses the configured pen lean explicitly. Keep upright bases for
this placement; the current-run comparison is available in the same workbench
view and in `runs/palace_15cm_right_angle_lean_cell/cell_mounting_assessment.json`.

The earlier 30° inward pen-tilt run and its matched one-arm baseline continue as
separate comparisons. Its provisional 31.5-minute allocation estimate is not yet
a validated build time. Both comparison runs now contain all 64 prepared motions
and complete timed candidates: 44.8 minutes for two arms, 71.6 minutes for one.
Both are undergoing full replay; these are not yet validated durations or an
achieved speedup. The workbench shows each run's current checkpoint.
Approach/withdrawal planning now refines its numerical samples only after a
tracking-bound rejection. The exact previously failing one-arm approach passed
with the original clearance and tracking limits; both jobs resumed their saved
allocations and motions after loading this fix.
Material checks also use the actual link's distance to the box enclosing all
deposited strands as a conservative early clearance test, including the full
interval motion allowance. Overlap or insufficient clearance still invokes the
original capsule checks. The active Palace stroke prepared in 17 seconds with
this change; the 12 existing scene/collision tests pass. Both jobs resumed with
their allocations and all 33 two-arm / 21 one-arm saved motions intact. This
component result does not establish a successful full-build replay.

Implementation follows [the active plan](docs/PLAN.md). The workbench shows the Palace Reflections model, geometric candidates, automatic curve selection, connected geometry, and proposed continuous pen strokes. A backward-search construction preview is also available. Per-stroke IK, continuous arm/static clearance checks, and a growing-material travel preview are implemented. Order repair can backtrack after robot-transition failures. Measured pen geometry, a successful complete shared-clock replay, and physical fabrication remain outstanding.

## Local workbench

```bash
uv venv --python 3.13
uv pip install -e '.[dev]'
npm --prefix web ci
npm --prefix web run build
.venv/bin/python -m pen_art.server
```

Open <http://127.0.0.1:8765>. The server reads artifacts from `runs/`; it has no robot execution route. It refreshes job progress automatically. Run the pipeline first if no artifact exists:

To play a prepared robot motion, select **Palace wireframe · right-angle pen ·
one active arm** in the model-run menu, open **06 Robot motion**, and click **Play robot motion** above
the 3D view. Use **Assigned motion** in the right panel to choose a stroke and
**Motion time** to scrub. This player previews individual prepared motions;
it does not imply that the complete build has passed replay.
When the complete one-arm schedule is available, **07 Pen timing** opens it with
**Play full one-arm build** above the viewport. Rejected schedules remain inspectable and
are labeled with their replay status.
Progress refresh now waits for the current stage to finish loading, so large
trajectory files can open while the planner continues updating checkpoints.
The full tilted candidate was checked at its final frame and restarted in the
browser; both arms advanced on the same clock with independent feed states.
The shared timeline groups contiguous numerical intervals into motion phases.
Click a phase to seek to its start; gray intervals are waits with the pen feed
off. The original motion polynomials and pen events remain unchanged.

The same fixed-catalogue planner can prepare a matched one-arm baseline. Use
the same source, cell and timing profile with `--resource-mode single` and a
separate output directory. The second robot remains in the cell as a parked
collision obstacle. For example:

```bash
.venv/bin/python -m pen_art.planning.run_cell \
  --source-run runs/palace_15cm_two_pen_selection \
  --output runs/palace_15cm_tilted_single_cell \
  --config configs/cell/two_flexiv_tilted.yaml \
  --timing-config configs/timing_continuous_bounded.yaml \
  --resource-mode single --maximum-seconds 3600
```

The matching Palace baseline is now running in
`runs/palace_15cm_tilted_single_cell`. Its geometry, cell, fixed catalogue and
process configuration match the tilted two-arm run byte for byte; both jobs
have separate one-hour resumable budgets. The webpage selects **One arm
baseline** for this run and displays its prepared-motion checkpoint. A complete
replay-passed baseline has not yet been produced. Do not compare allocation-only
estimates with a completed build as an achieved speedup.

Once both offline runs have saved full schedules, compare them with:

```bash
.venv/bin/python -m pen_art.planning.cell_comparison \
  --run runs/palace_15cm_tilted_cell \
  --single-run runs/palace_15cm_tilted_single_cell
```

This checks matching geometry, cell, process, timing and initial poses. It saves
evidence snapshots alongside the two-arm run for the webpage. Speedup is reported
only when both complete schedules have matching successful replay reports;
failed or unchecked runs remain an incomplete comparison.

```bash
.venv/bin/python -m pen_art.pipeline --config configs/palace.yaml --stage abstraction
```

The current config fits the model within a 15 cm cube, preserving proportions (15 × 15 × 9 cm). The 20 cm comparison is retained through `configs/palace_20cm.yaml`; the earlier development run is retained through `configs/palace_legacy15cm.yaml`. WIR3D optimizes the same unit-scale model for either size; verified outputs are transformed to the selected physical scale on import.

The source GLB is preserved. The run stores a full textured reference, an 80,000-face processing copy, physical coordinate transforms, and geometric cross-section curves. The geometric baseline is labeled separately from WIR3D. The current importer requires one mesh/material group and rejects unsupported multi-material inputs explicitly.

The Palace feature-selection experiment is `runs/palace_15cm_features`. Its
Candidate curves view shows the original model beside the selected curves and
compares outline, interior-edge, and keypoint coverage. Blue pixels in the
excluded-view previews mark interior features; green pixels mark strokes.
Selection is automatic and uses the same settings across models.

To refine an existing mesh/curve bundle into a new run without repeating WIR3D:

```bash
.venv/bin/python -m pen_art.abstraction.refine \
  --source-run runs/palace_15cm --output runs/palace_features_new
```

The command preserves source curves, connects them, adds start supports, and
generates the comparison and whole-contour order. It does not generate robot
motions. That earlier Palace candidate needs about 24.7 minutes for drawing and
feed delays alone; its structural solve is unresolved. A complete 30-minute
fabrication result remains unvalidated.

The Candidate curves page now compares the full textured source and strokes in
two synchronized viewports. Camera angle, physical scale and zoom are shared.
Turn off **Compare with original** to return to the single viewport.

An experimental surface-contour route traces visible outlines and openings back
onto the mesh. It splits at depth jumps, bounds smoothing, and supplies whole
traces to the same fixed-budget selector. A deterministic feasible starting
selection prevents the short solver budget from starting with poor long curves.

```bash
.venv/bin/python -m pen_art.abstraction.refine \
  --source-run runs/palace_15cm_features --output runs/new_contour_review \
  --contours --curves-only
```

`configs/contours.yaml` supplies the same settings for any mesh. The saved
`palace_15cm_contours` experiment selects 69 curves and 3.39 m, with a 20-minute
raw drawing/feed estimate. Its excluded-view outline coverage is higher, but
the columns and openings remain fragmented. It is explicitly marked **Shape
needs revision**, with no connected structure or robot plan. These proxy scores
are not a recognition acceptance test. The earlier Palace and robot results are
preserved; this experiment has not replaced them as an accepted design.

The subsequent **two-pen shape budget** experiment is
`runs/palace_15cm_two_pen_selection`. It traces connected apparent contours on
the mesh (`mesh_contours.py`) instead of splitting image contours at occlusions.
Selection uses those complete curves plus broad horizontal section rings. The
generation and selection cameras agree; validation cameras remain separate.
No Palace-specific feature labels or hand-edited connections are used.

```bash
.venv/bin/python -m pen_art.abstraction.refine \
  --source-run runs/palace_15cm_features --output runs/new_two_pen_shape \
  --contour-config configs/mesh_contours.yaml \
  --selection-config configs/selection_mesh_contours_two_arms.yaml --curves-only
# Continue with precisely the saved curves, without running selection again:
.venv/bin/python -m pen_art.abstraction.refine \
  --output runs/new_two_pen_shape --structure-only
```

The selector now packs whole curves into one or two provisional drawing
budgets. It does not split a long contour between pens. In the current Palace
example, 33 selected curves total 6.44 m, with 36.3 minutes of aggregate drawing
and feed work divided into approximately 18.2 minutes per pen. This uses the
second pen's capacity to retain more of the dome, bands, arches and columns.
The webpage shows aggregate work and per-pen loads separately. These are raw
drawing budgets, not final arm assignments or fabrication times. Connections, support, travel, cooling and collisions are handled downstream. Human
recognition and physical fabrication remain unvalidated.

Run the downstream two-arm stages for an existing supported selection with:

```bash
MPLCONFIGDIR=/tmp/3d-art-mpl .venv/bin/python -m pen_art.planning.run_cell \
  --source-run runs/palace_15cm_two_pen_selection \
  --output runs/palace_15cm_bounded_clock_cell \
  --timing-config configs/timing_continuous_bounded.yaml \
  --maximum-seconds 3600 --catalogue-seconds 300
```

This uses the exact saved supported geometry. It builds at most one drawing
mode per arm/action, solves matched one-/two-arm allocations, constructs the
selected full motions, and checks the shared-clock replay. Existing catalogue
entries can be resumed without regeneration. It stops with a recorded partial
result if a stage cannot finish within its budget. A fully prepared selection
also survives replay/return errors on resume when its input hashes still match;
resuming does not choose a new allocation and discard those motions. It does not
silently change the sculpture or expand the candidate set. Each generated visual stage appears
under the new run in the workbench. `cell_pipeline_result.json` records progress
and input hashes. Simulation output does not authorize hardware execution.
Restoration retains saved repair constraints and replay history before decoding
any motion. A deliberate stop records interrupted/partial status for motion and
replay, so a stopped job is not displayed as still running.

Within that budget, at most two allocation repairs exclude an observed failed
incoming motion together with its exact completed-material prefix. The same
motion remains eligible under other prefixes; the planner does not globally
reject its stroke. Current failure evidence is reused on resume. Structural
region previews also reuse a matching saved construction order, avoiding a
redundant search that could report failure for already ordered geometry.

Shared-clock replay can also add a non-overlap constraint for a conflicting pair,
park an idle arm, or order a motion before the stroke whose deposited material
obstructs it. These bounded repairs preserve the fixed drawing paths. Material
ordering cannot override required anchors or cooling; a contradiction remains
an unsuccessful plan, and each revised schedule must pass replay.

Replay reports two distinct phases: continuous motion-limit checks and the
shared-clock collision/material replay. Its current-plan artifact is written
before checking paths, with completed action/interval counts displayed in the
workbench. Exhausting the compute allowance during either phase leaves
verification incomplete; it does not label the object physically infeasible.
The speed-bound subdivision itself observes the same planning deadline.
Within one repair batch, unchanged paths reuse their completed kinematic-limit
checks. The reuse key includes the exact trajectory, arm, robot file, cell
profile, settings and source hashes. Schedule continuity, pen events, anchors,
inter-arm clearance and growing material are checked again for each schedule.
No replay success is cached across schedule changes or process restarts.

The original catalogue uses vertical pens. A separate fixed inward-tilt policy
is available in `configs/cell/two_flexiv_tilted.yaml`: it points each pen toward
the build center while moving its wrist toward its own base. It preserves zero
tilt as the original behavior and changes no sculpture geometry. The separate
`runs/palace_15cm_tilted_cell` run completed all 128 arm/action groups (64
construction actions available on each arm) with this policy and is preparing
the selected full motions. The initial allocation estimates 31.2 minutes for
two arms and 59.1 minutes for arm 1; full constructed-path timing and shared-clock
replay remain pending.
The workbench shows the configured angle and each candidate's actual tilt;
candidate/waypoint selection and the reference toggle survive live updates.
The run selector appends the configured inward tilt to distinguish orientation
experiments without renaming or modifying their planning artifacts.
The allocation view isolates the wireframe by default; its full-cell control
and collision-witness controls restore the robot bodies for inspection.

The original vertical-pen run finished its bounded pass with
`SCHEDULE_REPLAY_REJECTED`: 64 motions were prepared, but the seventh replay
still found wrist interference. Its 35.1-minute schedule is retained as rejected
evidence, not as an executable result. No additional vertical-pen search is
running.

```bash
MPLCONFIGDIR=/tmp/3d-art-mpl .venv/bin/python -m pen_art.planning.run_cell \
  --source-run runs/palace_15cm_two_pen_selection \
  --output runs/palace_15cm_tilted_cell \
  --config configs/cell/two_flexiv_tilted.yaml \
  --timing-config configs/timing_continuous_bounded.yaml \
  --maximum-seconds 3600 --catalogue-seconds 600 --maximum-allocation-repairs 2
```

Do not launch a second writer into a run directory while its planner is active.
The command resumes a stopped matching catalogue; it does not change the selected
Palace graph or expand the number of alternatives per arm/action.

To compare one recorded conflict pose with that orientation in the workbench:

```bash
.venv/bin/python -m pen_art.planning.orientation_probe \
  --run runs/palace_15cm_bounded_clock_cell --tilt-deg 30
```

The **Pen-angle diagnostic** compares static robot poses at the same tip positions;
it does not replace the active catalogue or establish fabrication feasibility.

For a new mesh, the local geometric route can run both stages in one command:

```bash
MPLCONFIGDIR=/tmp/3d-art-mpl .venv/bin/python -m pen_art.build \
  --source path/to/model.glb --source-up z --max-extent-m 0.15 \
  --run runs/model_strokes --cell-output runs/model_cell \
  --timing-config configs/timing_continuous_bounded.yaml \
  --planning-seconds 600
```

The source run preserves mesh, curve selection, connections and supports. The
separate cell run shows fixed drawing paths, allocation, prepared motions and
replay as each becomes available. Omit `--cell-output` to stop after geometry.
`--source-up` must match the input asset. This route uses geometric contours;
texture-aware WIR3D results continue through the saved-run command above.

The continuous clock keeps motion through numerical samples while checking
acceleration over each cubic interval. The current 15 cm Palace allocation
estimates 27.6 minutes for two pens and 53.6 minutes for one, using the same
7.43 m wireframe and assumed 3 mm/s deposition limit. These are allocations
with estimated transfers; complete motion preparation and replay remain
separate checks. The workbench displays the latest saved planner checkpoint
and refreshes when artifacts change.

A saved full schedule can be exported without contacting hardware:

```bash
MPLCONFIGDIR=/tmp/3d-art-mpl .venv/bin/python -m pen_art.trajectory \
  --run runs/palace_15cm_bounded_clock_cell
```

This requires `offline_dual_arm_plan.json`; partial motion preparation is not
enough. `trajectory.json` contains per-arm joint position, velocity and
acceleration polynomials, explicit waits, TCP endpoint transforms and the shared
pen events. Evaluate coefficients at normalized segment time; do not linearly
interpolate the endpoint poses. `joint_path.json` preserves source paths and
`trajectory_manifest.json` binds the package to the plan and matching replay.
The workbench offers these downloads only for a matching manifest. Rejected
replay stays rejected in the export; all packages remain simulation-only.

`python -m pen_art.validation --run runs/NAME` creates a hash-bound validation
summary. The Validation page separates modeled geometric results, uncalibrated
pen/structure assumptions and physical work that has not been tested. The
fixed-catalogue driver generates this summary when a batch returns.

## WIR3D on Blondie

Remote project directory: `/home/gilwoo/Workspace/3d_art`. GPU dependencies live in `.venv-wir3d`, separate from the local workbench and any future robot controller. The RTX 5090 uses PyTorch 2.8.0/CUDA 12.8 and Kaolin 0.18.0, following the [PyTorch Blackwell support](https://pytorch.org/blog/pytorch-2-7/) and [Kaolin compatibility matrix](https://kaolin.readthedocs.io/en/stable/notes/installation.html).

`scripts/setup_wir3d_blondie.sh` records pinned source dependencies and installs Blender 3.6.23 inside that project. It expects the isolated Python environment and PyTorch to exist. It makes no system driver changes.

Upload `runs/palace_15cm/wir3d_bundle/` to the remote project. Then, on Blondie:

```bash
cd ~/Workspace/3d_art
export CUDA_HOME=/usr/local/cuda-12.8
export PATH="$CUDA_HOME/bin:$PATH"
export TORCH_CUDA_ARCH_LIST=12.0
export MAX_JOBS=4
.venv-wir3d/bin/python wir3d_bundle/run_job.py \
  --repo external/wir3d \
  --blender tools/blender-3.6.23-linux-x64/blender --resume
```

`--check` performs preflight checks. `--through sketch-renders` runs only the first two rendering steps. Blender 3.6 reference rendering uses CPU; CUDA optimization uses the RTX 5090. Runtime wrappers disable upstream's unpinned package installation, correct the automatic keypoint loader's HWC/CHW mismatch, and seed keypoint/SDF processing. The original upstream checkout stays unchanged. Patch hashes, seeds, input hashes, and environment details accompany the results; bitwise reproducibility across hardware is not claimed.

Pull live progress and automatically import completed budgets from the Mac:

```bash
.venv/bin/python -m pen_art.abstraction.remote \
  --run runs/palace_15cm --host blondie \
  --remote-bundle /home/gilwoo/Workspace/3d_art/wir3d_bundle --watch
```

The Candidate curves page also shows the newest complete geometry/semantic
optimizer comparison image for each started budget. These images are unfinished
diagnostics, separate from the verified final 3D curve exports. Partial PNG writes
are retried without replacing a previously imported preview.

The runner exports each completed budget separately and records failures. Resume markers depend on the job and adapter configuration and require expected outputs to exist. A single budget can also be imported with `pen_art.pipeline --stage import --budget 20 --curves PATH`. Curve files are checked for provenance and converted back into the physical build frame.

## Flexiv reference

The setup preview uses a provisional Rizon4 model from the manufacturer's
[description repository](https://github.com/flexivrobotics/flexiv_description/tree/humble-v1),
pinned at `92ef7865d76585e6e08d291bdfe652d32f7740f4`. The physical variant is not confirmed.
The exporter expands the URDF without requiring ROS, preserves joint limits, and exports a
static model preview. Forward kinematics accepts an explicit flange-to-TCP transform;
none is assumed to be measured yet.

```bash
.venv/bin/python -m pen_art.robot --variant Rizon4 --output runs/palace_15cm
```

This produces `robot.urdf`, `robot_manifest.json`, and `robot_reference.glb`. Robot motion,
collision checking, pen geometry, and control remain separate outstanding work.

## Automatic curve selection

```bash
.venv/bin/python -m pen_art.pipeline --config configs/palace_15cm.yaml --stage select
.venv/bin/python -m pen_art.pipeline --config configs/palace_15cm.yaml --stage graph --candidate selected
```

`abstraction/selection.py` chooses intact source curves from available geometric
and WIR3D sets. It scores additional silhouette coverage and automatic keypoint
coverage on fixed selection views, then checks each proposed addition's connected
graph and partial drawing time. Rejected curves can be reconsidered when another
curve supplies a cheaper connection. Required curves are retained or reported as
incompatible with the budget. This bounded greedy search is not a global optimum.

`configs/selection.yaml` reserves six minutes of the 30-minute target for overhead.
The remaining 24 minutes constrain connected drawing length and assumed feed
start/stop time. Neither the speed nor the overhead reserve is calibrated.
`selected_candidates.json` records accepted additions, costs, source IDs and
input hashes. Separate held-out views in `candidate_comparison.json` compare the
selected set with its sources. The webpage exposes the selection steps and both
raw and connected projections. The remote watcher refreshes selection/comparison
as WIR3D budgets arrive; existing construction plans are not overwritten.

The independent `palace_15cm_selected` experiment currently retains 14 of 64
geometric curves: 20 connected pen strokes, 4.25 m, and a **23.9-minute partial
estimate at an assumed 3 mm/s**. Its held-out outline coverage falls from 57% to
38%; automatic feature coverage stays at 67%. This is a development candidate,
not proof of recognizability or a complete sub-30-minute fabrication plan. The
denser original run remains available for comparison.

All 20 selected drawing paths passed the modeled arm/static-surface and continuous
TCP tracking audits. Sequence planning also completed all 20 strokes and the
return to parking: 81 operations with continuous joint positions across operation
boundaries. This remains a geometric simulation preview with an unmeasured pen,
not an executable or physically validated fabrication program.

## Pen strokes and automatic connections

```bash
.venv/bin/python -m pen_art.pipeline --stage graph --candidate geometric
# After WIR3D import, use --candidate wir3d_20 (or 40 / 60).
```

This generates `stroke_graph.json`, `pen_strokes.json`, `structural_screen.json`, and
`process_screen.json`. Real 3D contacts become graph junctions. Permanent base
members and connectors join the material without treating the bed as a connection.
The whole drawing is lifted if necessary to keep fitted curves above the bed;
the rigid placement is recorded and applied to the reference overlay.

**A graph member is not a pen start.** Consecutive pieces of each source contour
are joined into continuous drawing strokes, with intermediate junctions retained.
A closed contour can be started at an existing junction in either direction. Later
construction planning can introduce deliberate joins or cooling pauses as needed.
The current geometric baseline has 70 proposed pen strokes, rather than a separate
start/stop at each of its roughly 1,900 material members. It remains much too dense
to be the preferred artistic abstraction.

`configs/fabrication.yaml` explicitly leaves unmeasured pen limits unknown.
Unknown free-span, curvature, downward drawing, and cooling capabilities request
calibration; they do not automatically reject a stroke. Explicit provisional values
are advisory until calibrated. Bed penetration and missing starting anchors are
separate geometric/construction constraints. The current screen assumes potential
anchors are already present and therefore is not a construction-order proof.

Start calibration with a bed line and join, an upright strand, a bridge, and a curved
stroke, including the pen's stop/start response. Measure drawing speed, diameter,
cooling holds, and failures. Prefer a constant pen setting per job initially.
The workbench's speed slider is a what-if calculation: drawing length divided by
assumed speed excludes joins, travel, cooling, and startup. The 30-minute target
must eventually use the complete calibrated schedule.

The optional cooled-frame analysis is a linear beam approximation with assumed
material and weld stiffness. Analytic cantilever cases are checked. Singular or
poorly conditioned graphs produce an unresolved-analysis diagnostic, never a
fabrication-failure claim or an unreliable deformation visualization. Physical
pen tests take priority over refining a dense geometric baseline's beam model.

## Construction preview

```bash
.venv/bin/python -m pen_art.pipeline --stage construction
```

`construction_plan.json` records a grounded geometric ordering of continuous pen
strokes. The search works backward, can move a closed contour’s starting point to
an existing junction, and splits open contours at real anchors when needed. Forward
replay checks available starts, grounded prefix topology, and exactly-once member
coverage. The workbench provides step controls and schematic playback. This stage
is labeled partial: intermediate stiffness, calibrated deposition, robot paths,
collisions, and travel still need to be integrated into the search.

## Validation

### Offline two-arm allocation

For a refined run with automatic start supports, prepare the matching graph and
construction order together. The source run and earlier robot experiments are
preserved:

```bash
.venv/bin/python -m pen_art.cell \
  --source-run runs/palace_15cm_features --source-variant backbone \
  --output runs/palace_15cm_features_dual
.venv/bin/python -m pen_art.planning.cell_motion \
  --run runs/palace_15cm_features_dual --per-arm-stroke 1 --maximum-seconds 180
```

`--source-variant backbone` binds the supported graph, complete contour order,
pen strokes, and corresponding process/structure screens in one verified
snapshot. Every construction member must occur exactly once in that graph.
Motion generation checks this snapshot instead of copying the earlier graph
over the added supports. Resume an unfinished fixed catalogue with `--resume`
only after the preceding command exits.

For these supported inputs, allocation finishes the automatically added starting
scaffold before the main contours. Both arms may work during either phase.
The refined Palace has 106 fixed drawing paths for 53 actions, with provisional
support-first allocation estimates of 25.9 minutes for two arms and 50.2 minutes
for one. These estimates are not collision-validated fabrication times.
Full-motion preparation stopped after 35 of 53 actions when the approach
planner exhausted its time budget. Shared-clock replay has not run for this
schedule; the result does not establish an impossible approach or a printable
Palace.

```bash
MPLCONFIGDIR=/tmp/3d-art-mpl .venv/bin/python -m pen_art.planning.cell_allocation --maximum-seconds 15
MPLCONFIGDIR=/tmp/3d-art-mpl .venv/bin/python -m pen_art.planning.allocation_motion --maximum-seconds 180
```

The first command schedules the existing motion catalogue with CP-SAT. It assigns
each stroke exactly once, chooses arm sequences, preserves alternative support
anchors and cooling constraints, includes sequence-dependent transfer estimates,
and minimizes makespan. It writes `allocation_problem.json` and
`cell_allocation.json`; the **Two-arm setup** page displays colored stroke
ownership, selectable timelines, and matched one-/two-arm estimates.

The second command constructs only the selected motions, reusing completed
preparations on an unchanged input. It records `allocation_motions.json` and
stops at a concrete selected-motion failure. If all motions and returns are
prepared, it replaces the transfer estimates with actual path clocks, preserves
the selected arm order, and runs independent shared-clock replay. Allocation
alone does not establish robot collision clearance, intermediate stiffness or
physical pen feasibility. It does not restart the earlier beam/candidate search.

Replay uses a bounded refinement budget (default six) for concrete moving/moving
exclusions and explicit idle-arm parking, retaining the prepared drawing paths. The page can
play each prepared full motion and show the recorded joint poses of a clearance
conflict.

The current fixed-catalogue solve assigns all 20 Palace strokes: estimated finish
24.2 minutes with two arms and 47.5 minutes with one. Both results are feasible
incumbents from 15-second solves, not proven optima or measured fabrication times.
All 20 selected motions were prepared. The latest bounded repair reached a
35.7-minute schedule and still failed inter-arm clearance. No complete coordinated
schedule is certified yet.

The separate `palace_15cm_wir3d20_dual` run uses the actual WIR3D output: 29 curves,
2.22 m after automatic connections, and 62 construction actions. Its fixed-catalogue
estimates are 15.7 minutes with two arms and 28.5 minutes with one. All 62 selected
full motions were prepared; a 21.9-minute retimed schedule still failed replay.
These estimates and rejected schedules are not demonstrated fabrication times.

### Structural work regions

```bash
MPLCONFIGDIR=/tmp/3d-art-mpl .venv/bin/python -m pen_art.planning.partition --run runs/palace_15cm_wir3d20_dual
```

This generic graph pass groups whole source contours, coupled cycle blocks and
possible level rims before assigning arm-side regions. It adds no cuts and does
not stretch the sculpture. Shared groups keep one arm owner and reserve both
resources. The **Structural work regions** view under **Two-arm setup** shows the
groups, lengths and unresolved construction status. The current model has no
unsplit construction order within the bounded search; support topology alone
does not establish a standable intermediate structure.

Synthetic ring, arch and branching-graph checks are recorded in
`research/partition_generalization.json`. These check geometry preservation and
grouping, not general mesh-to-physical fabrication. That broader requirement and
calibrated structural/pen validation remain open.

### Timed preview

`--stage timing` retimes a complete sequence and adds a **Pen timing** page with
an operation timeline, tip-speed chart, robot playback and feed-command events.
It reports unknown overhead and short-stroke response conflicts separately.
See [the timing contract](docs/timing.md) for the command, motion law and limits.
Cubic smoothing requires a new geometry audit; the original joint-linear path
certificates do not make the timed preview executable.

```bash
.venv/bin/ruff check src tests
.venv/bin/pytest -q
npm --prefix web run build
```

Tests cover asset preservation and transforms, curve geometry and bed placement,
WIR3D provenance, true 3D connectivity, continuous stroke grouping, closed-contour
starts, analytic beam solutions, measured versus unknown pen limits, and read-only
artifact boundaries. Browser inspection verifies rendered artifacts and controls.
These checks do not establish physical feasibility. The exact Flexiv variant,
pen/mount geometry, controls, and measured deposition behavior remain required.

### Provisional platform placement

Run `.venv/bin/python -m pen_art.placement` to search the table-mounted arm's
build-platform XY position and height using `configs/placement.yaml`. The search
checks 27 bounding-volume poses per placement, ranks sampled IK coverage then
normalized joint-limit margin, and checks voxel-selected curve samples at the
highest-ranked placement. `placement_search.json` retains every candidate,
per-sample joint solutions, input hashes, and assumptions. The Robot motion panel
reports the result. The sculpture size stays fixed; a physical riser raises its
build plane without adding printed supports.

This is a finite-grid, sampled reach study, not a globally optimal or executable
placement. The provisional Rizon4 model, vertical pen orientation and 16 cm axial
flange-to-tip offset need hardware confirmation. The downstream motion stages add continuous IK paths, arm/table/platform checks,
and a growing-material transit preview. Pen and mount geometry remain pending;
the reach ranking itself does not include collision cost.

WIR3D memory compatibility: the generated keypoint wrapper chunks vertex-feature
Adam updates and latent nearest-face matching while retaining the full source
mesh. `scripts/check_wir3d_memory_math.py` checks exact update equivalence in the
Torch environment. Generated wrapper hashes and per-stage dependency signatures
are recorded; legacy render cache migration requires matching prior signatures
and unchanged render dependencies. This avoids rerendering when only the
keypoint implementation changes.

The SDF wrapper executes its upstream definitions in the real module namespace,
so PyTorch can pickle `GeometryDataset` when caching the expensive scan result.
The regression test exercises that class lookup; earlier isolated `exec`
namespaces failed at checkpoint saving after preprocessing.

### Per-stroke arm-motion preview

After `--stage construction` and the placement search, run:

```bash
.venv/bin/python -m pen_art.planning.curve_motion --run runs/palace_15cm
```

`drawing_motion.json` stores bounded layered IK results for the directed drawing
strokes. The search retains alternate joint configurations, rejects large joint
jumps, and checks TCP tracking at quarter points of each joint interpolation.
The Robot motion panel can select and play individual paths using the same URDF
kinematics. The blue line represents the assumed flange-to-tip axis.

These paths do not include inter-stroke travel, tool/environment collisions,
extrusion scheduling or physical process acceptance. Sampling alone does not
prove a continuous error bound. The separate interval audit below adds conservative continuous tracking bounds.
The coupled order-and-motion planner remains unfinished; this preview exposes
its per-stroke building block.

### Arm and static-surface collision audit

```bash
.venv/bin/python -m pen_art.pipeline --stage collisions
```

This writes `drawing_collisions.json` and adds per-stroke results to the motion
panel. The audit checks filled convex hulls of nonadjacent URDF links and the table/platform geometry
from `configs/collision.yaml`. It bounds point motion over each joint interval,
subdividing until clearance is established, a collision is found, or the depth
budget is exhausted. An unresolved clearance is not reported as a collision or
a pass. The UI rejects audits whose motion-file hash no longer matches.

The same report bounds continuous TCP position and orientation error relative
to each linear Cartesian reference segment using the serial chain's second
derivative bounds. This is separate from the original curve's discretization
error and from the real controller's tracking error. A conservative bound above
the threshold is inconclusive, not proof that the physical pen path fails.

With the current null tool profile, this is an arm/static-scene audit. A measured
pen and mount profile can be included (see below). Drawing contact, deposited
strands, inter-stroke travel and the controller's real interpolation are separate
from this audit. Table dimensions, robot variant and TCP are
provisional. The fixed mounting assembly may contact the table; moving links may
not. No physical robot commands are issued.


### Construction and travel through growing material

```bash
.venv/bin/python -m pen_art.pipeline --stage sequence --maximum-seconds 600
# Resume after a time limit with unchanged input artifacts and settings:
.venv/bin/python -m pen_art.planning.transit --resume --maximum-seconds 600
```

`sequence_motion.json` joins the existing drawing paths with explicit pen-off
travel, approach, withdrawal, and return to a planned parking pose. The Robot
motion panel's **View construction + travel** control animates these operations,
the growing deposited structure, and a purple local contact region. It reports
partial results separately from complete geometric previews. This is schematic
playback, without calibrated timestamps or hardware commands.

The planner adds capsule geometry as material is deposited and uses bounded
bidirectional RRT-Connect for free travel. Cartesian departures try an upward
path and eight oblique alternatives while retaining fixed pen orientation.
The same collision checks apply to every direction. Arm/static/material checks
remain active at joins; only the idealized tip has a provisional 4 mm local
contact allowance. A regression test shows a vertical departure blocked by an
overhead strand and a valid oblique exit around it. Direction choices and search
attempts are saved with each approach/withdrawal. A timeout or exhausted set of
directions is a search result, not proof that the object is physically impossible.

The parking pose is checked against the complete modeled sculpture. The robot
must already be there before this preview begins; an arbitrary real starting
pose is not covered. Pen-body/mount clearance and contact during drawing remain
unvalidated. These paths use the provisional Rizon4 and TCP and do not establish
deposition stability, weld quality, or the 30-minute target. The current dense
geometric baseline is still a development example; sparse WIR3D curves are the
intended design input.


During distance-field training, the remote importer also retrieves the latest
complete diagnostic PNG. The Candidate curves page shows its iteration number
and learned/reference comparisons separately from the curve output. Partially
written images are retried on a later sync. A rendered training plot does not
mean WIR3D's final curve optimization has completed.

Near a build surface, the local tip-contact allowance applies only when that
surface intersects the same endpoint contact ball and the bounded swept tip is
entirely inside it. It does not disable arm collision checks or exempt the rest
of a departure path. Saved unsuccessful direction attempts contain collision or
unresolved-clearance witnesses for diagnosis.


### Coupled order repair

The sequence command now uses `planning/sequence.py` and `planning/coupled.py`
to try complete approach/draw/withdraw transitions and backtrack into earlier
order choices. A new stroke is eligible only when its start node is grounded or
already present in the current material graph. Each branch retains its own robot
configuration and material scene; rejected branches do not leave phantom
material behind. The backward geometry plan supplies the preferred order.

Approach directions are considered together with their pen-off transit, so a
clear lift whose endpoint is unreachable from the previous pose does not end the
search prematurely. Resume can reuse verified complete prefixes while still
backtracking before them if necessary. The output stores the selected drawing
order, attempts, backtracks, and rejected-transition diagnostics. Search budgets
remain explicit; failure to find a plan is not proof of physical impossibility.

This implementation repairs order around the existing drawing configurations.
It does not yet regenerate all drawing-pose alternatives or evaluate calibrated
hot-strand behavior. Measured non-contact pen geometry is supported, but has not
been supplied for the current run. Tests include a real FCL material
collision that forces an order change and a case where only a different final
robot configuration admits the return motion.


### Measured pen and mount collision model

Both drawing auditing and sequence search now accept explicit flange-frame
boxes, spheres, cylinders and convex mesh bodies in `tool_geometry`. These bodies
remain collision-active at joins. Only the separate tip approximation receives
the local joining allowance. The motion webpage reports whether geometry was
supplied and renders the same bodies used by the checker.

See [the tool profile contract](docs/tool_geometry.md) for fields, coordinate
frames, contact exclusions and audit commands. A fingerprint includes mesh
contents, so changing a CAD file invalidates saved sequence reuse. The Palace
run remains arm-only until measured geometry and a confirmed TCP are available.

The renderer has a geometry/transform test in addition to the Python collision
checks. With Node 24 (or a version supporting `stripTypeScriptTypes`), run
`cd web` then `npm test`; `npm run build` also checks TypeScript.


### Compare shape and connected drawing cost

```bash
.venv/bin/python -m pen_art.pipeline --stage compare
```

This snapshots `configs/comparison.yaml` and the fabrication assumptions, then
evaluates available candidates from eight fixed orthographic views.
`candidate_comparison.json` records mean/worst-view outline coverage, proximity
to automatic semantic keypoints, raw and connected drawing length, and a partial
time estimate. The Candidate curves page shows a comparison table and paired
original/connected projection previews. Green strokes are overlaid on a gray
silhouette from the geometry copy. The full textured reference stays available
in the main viewer.

The score is a geometric proxy, not a human-recognition or texture-semantic
evaluation. The Pareto frontier keeps alternatives that trade shape evidence
against connected length. A provisional choice must be connected and fit the
partial time budget under the assumed speed. Timing includes proposed strokes
and feed start/stop overhead; it omits travel, deliberate joins, cooling,
acceleration and any later braces. Positive remaining time does not prove a
30-minute build. The current 64-curve baseline estimates 72.6 minutes at 3 mm/s
even before those omissions, so it is not provisionally selected.

After a comparison has been configured, `--stage graph --candidate auto` uses
its current provisional choice. It rejects stale comparisons or an empty
choice; specify `--candidate geometric` only for the development baseline. The
remote importer refreshes the comparison when a new WIR3D budget arrives and
retries incomplete comparisons. It does not replace the current construction
graph or a running motion job.

### Two-arm development cell

```bash
.venv/bin/python -m pen_art.cell \
  --config configs/cell/two_flexiv.yaml \
  --source-run runs/palace_15cm_selected \
  --output runs/palace_15cm_dual
```

Select **Palace Reflections · two-arm development → Two-arm setup** in the
workbench. Both 7-DoF arms use independent world/base and flange/tip transforms.
The reference remains uniformly scaled to 15 cm. The current 1.2 m opposing-base
placement is a development assumption, not a measured or optimized installation.
The manifest records arm/static and inter-arm checks at the shown poses only.

`planning/coordinated_motion.py` checks inter-arm collision continuously over
joint-linear paths or curved paths supplied with joint displacement bounds.
It distinguishes collision witnesses, unresolved clearance and time-budget
limits. This component does not yet assign strokes or produce a complete
two-arm schedule. The single-arm artifacts remain separate.

Generate bounded drawing alternatives for each arm (the source geometry and
cell inputs are fingerprinted):

```bash
.venv/bin/python -m pen_art.planning.cell_motion \
  --run runs/palace_15cm_dual --config configs/cell/two_flexiv.yaml \
  --maximum-seconds 300 --per-arm-stroke 2
# Resume only after the preceding process has ended:
.venv/bin/python -m pen_art.planning.cell_motion \
  --run runs/palace_15cm_dual --config configs/cell/two_flexiv.yaml \
  --maximum-seconds 300 --per-arm-stroke 2 --resume
```

The **Two-arm setup** view can select an arm/stroke candidate and replay its
waypoints. These candidates include arm/self and static-environment checks,
but are not coordinated trajectories. The integrated planner now connects them
with travel, approach, joining and withdrawal. It searches arm ownership and
start times against both arms and the growing material:

```bash
.venv/bin/python -m pen_art.planning.cell_planner --maximum-seconds 600
# Run after the planner has finished, against the exact saved output:
.venv/bin/python -m pen_art.planning.cell_replay --maximum-seconds 300
```

`dual_arm_plan.json` records the checked prefix or complete schedule, shared-clock
joint paths and arm-addressed feed commands. `dual_arm_validation.json` replays
that serialized output from a fresh scene. A partial prefix is not a complete
sculpture. The workbench can switch between candidate inspection and shared-clock
playback, with one timeline and feed state per arm.

The current bounded search favors completing a feasible schedule before reducing
its makespan; it retains four checked successors per state. Its quintic time law
preserves the checked joint-linear paths and stops at geometric waypoints. The
pen must be calibrated for those stops or the paths must be blended and checked
again. Clear staged start poses are a job precondition; the approach from an
external home pose is not included. Missing pen/mount geometry and hot-strand
behavior remain explicit limitations, so these outputs cannot command hardware.

## Object-independent entry point

The design principle is **recognizable shape, then buildable structural units,
then parallel work allocation**. A spatial half-plane guides arm ownership; it
does not cut contours. Preserve loops and construction-critical arches until a
process/stiffness model explicitly permits subdividing them. Connectivity alone
does not prove either a stable finished object or a printable growing structure.

The local geometry path accepts a mesh without object-specific settings:

```bash
.venv/bin/python -m pen_art.build --source /path/to/model.glb \
  --source-up y --run runs/my_model --max-extent-m 0.15
```

It normalizes without stretching, extracts geometric contours, selects a sparse
subset with two bounded offline solves, connects them, selects automatic start
supports with one bounded CP-SAT solve, verifies an
intact-contour construction order, screens the cold structure, and proposes
whole-unit arm regions. Inspect its stages at http://localhost:8765. This entry
point stops before robot motion and uses geometric contours, not texture-aware
WIR3D optimization. Multiple transformed mesh/material parts are supported and
their textured reference is preserved; multi-material WIR3D still needs atlas baking.

To continue through the single-arm restart planner and complete replay:

```bash
MPLCONFIGDIR=/tmp/3d-art-mpl .venv/bin/python -m pen_art.build \
  --source /path/to/model.glb --source-up y --max-extent-m 0.15 \
  --run runs/my_model --cell-output runs/my_model_one_arm \
  --cell-config configs/cell/one_flexiv_right_angle_lean.yaml \
  --resource-mode single --no-time-limit --single-arm-restarts
```

The size is a 15 cm maximum bounding-box dimension, not a 15 cubic-centimeter
volume. Smaller requested dimensions are supported; normalization is uniform.
This route preserves source strokes, permits supported pen-off restarts, and can
add an automatically planned endpoint post when a release needs reinforcement.
It resumes from bound checkpoints and exports a full schedule only after all
selected strokes are prepared, then performs growing-material collision replay.
It is an implemented route, not yet a demonstrated end-to-end generalization
result. Fixed-catalogue eligibility, reachability, nozzle access, and structural
screens can still stop a model with an explicit diagnostic. Physical pen limits
remain uncalibrated; fitting the size box alone does not prove printability.

For an existing WIR3D run, use `python -m pen_art.backbone --run RUN_DIR` after
import/graph generation. The same solver operates on either abstraction.
It selects endpoint posts and dependencies, minimizing estimated support length;
it does not certify deposition or stiffness. Baseline artifacts remain available.

`scripts/demo_generalization.py` runs the identical local settings on textured
torus, vase, and branching meshes. Sparse results are in
`research/generalization/sparse_results.json`; the earlier all-curve comparison
is retained in `results.json`. Each has a separate workbench run. The shared
selector keeps 10/46 torus curves, 10/52 vase curves and 24/50 branching curves.
Connected drawing plus feed estimates are 18.9, 17.0 and 14.7 minutes at an
assumed 3 mm/s; full travel, joins and cooling are still outstanding.

These are mesh-to-order demonstrations, not recognition or fabrication benchmarks.
All sparse runs produce intact orders but exceed the uncalibrated cold-frame
deflection limit. Mean outline coverage on eight excluded views is 87.5%, 45.4%
and 81.3%, respectively. The vase demonstrates why a selection-view score is not
enough to establish recognition. No result has a validated physical build.

`configs/sparse.yaml` fixes the two-pass coverage/cost objective, 95% retention of
the first pass's achieved score, and separate time allowances. It generates no
new candidates. Add `--all-curves` to a new run for the dense comparison. To use
the same selector on an existing geometric/WIR3D candidate pool, run
`python -m pen_art.abstraction.sparse --run RUN_DIR`; regenerate graph and later
stages afterward. The webpage shows selected 3D curves, solve status, time
allowances, and projected outlines from camera angles excluded from selection.


### One-arm supported stroke restart trial

`MPLCONFIGDIR=/tmp/3d-art-mpl .venv/bin/python scripts/try_stroke_restarts.py`
tries automatic breaks on the blocked Palace `draw-0023`, using the 12 prepared
motions as its printed-material prefix. It preserves the source contour: adjacent
chunks share the same TCP point. Posture jumps propose breakpoints; a cooled,
clamped-base beam model screens the partial structure with the pen removed.
Accepted chunks use the existing arm, pen, material and 1 mm clearance checks.
Each chunk stops feed, holds at the endpoint for the provisional 2 s cooling time,
withdraws, travels with feed off, and approaches the same point in its new posture.
The next chunk restarts feed and continues along the source contour.

Results go to `runs/palace_15cm_one_arm/stroke_restarts/`. `trial.json` distinguishes
partial accepted chunks from a complete contour; `trajectory.json` contains only
accepted chunks. This trial does not replace the full-build allocation or certify
physical fabrication. Cooling time, rigid welds, material stiffness and pen geometry
remain uncalibrated. A failed release screen prohibits that proposed break under
these assumptions; it does not prove that all possible break locations fail.

The restart harness also accepts `--run`, `--stroke`, and `--profile`, provided the
run contains a matching posture proposal and a prepared material prefix. Use
`--verify-inputs` to check those bindings without planning. `--resume` restores
accepted chunks only when the input hashes match; after a drawing-clearance
failure, it proposes one additional split at the nearest interior source vertex.
It still requires a fresh release screen, withdrawal, approach and drawing check.

Latest Palace restart trial (2026-10-11): 12 accepted chunks and 11 verified
same-point restarts cover 152.8 mm of the 272.5 mm sampled contour, with a modeled
clock of 291.7 s. This includes the previously abrupt 2.84 rad joint change at
source vertex 105, now performed with feed off after withdrawal. Source geometry
and the one-arm cell are unchanged. The next proposed release at vertex 155
predicts 1.162 mm maximum deflection against the provisional 1 mm limit. Moving
that break earlier automatically found vertex 132, which passed both support and
motion checks. Vertices 133–155 failed the release screen. This is a completed
trial with a verified partial trajectory, not a complete contour or Palace build.
`restart_audit.json` records input hashes, contiguous source coverage, feed-state
checks and same-point errors below 1 micrometer. The four existing smooth-motion
audit tests and lint passed; physical validation is still outstanding.

Open the current one-arm restart playback directly at
<http://localhost:8765/?run=palace_15cm_one_arm&stage=motion> and click
**Play restart trial · 10×**. The speed selector offers 1×, 5×, 10× and 20×,
and the time slider supports seeking. This plays the saved partial contour with
11 restarts (about 29 seconds at 10×), with the earlier printed supports shown.
It does not play a complete Palace build. New restart runs publish
`stroke_restart_preview.json` for this viewer automatically.

### Complete one-arm build continuation

The full-build continuation is separate from the small restart demonstration:

```bash
MPLCONFIGDIR=/tmp/3d-art-mpl .venv/bin/python -u -m pen_art.planning.single_arm_build --run runs/palace_15cm_one_arm
```

It restores the 12 earlier source motions and the accepted contour fragments,
adds the checked automatic base-post motion, and continues the original 64-stroke
allocation. The post braces the last supported contour endpoint; source contours
are unchanged. `full_build/state.json` records source-stroke completion separately
from fragments and added supports, with input hashes and exact incoming joint
poses checked on resume. Individual accepted motion clocks remain separate files.
A stopped checkpoint is evidence of partial progress, not a full build.

Only after all 64 source strokes are covered does the continuation export the
complete `offline_dual_arm_plan.json` (the existing format name also supports one
arm), plan the final return, and invoke the existing continuous replay. The replay
checks clock limits, feed events, material timing and collision clearance from the
empty base. The full-build player must remain labeled unverified until that replay
passes. Pen cooling, weld behavior and tool dimensions still require calibration.
