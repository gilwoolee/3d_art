# Educational codebase simplification plan

Date: 2026-10-20
Status: implemented and validated; run outputs removed as requested.
Reviewed baseline: `21be118` on `main`.

## Progress

- Comparison run: `simplify_before` passed every stage; output
  `/tmp/pen_art_simplify_before.log`. It contained 1,863 files, 384,289,427 bytes
  and ten normalized meshes.
- Original Ruff and viewer build: passed. Vite reports its existing bundle-size
  warning.
- Layout: implemented; `simplify_layout` passed every stage with identical
  graphs, selected strokes, timed action trajectories and summaries. New paths,
  LFS attributes, shell activation and browser playback checked. Fresh setup is
  deferred to the final changed snapshot.
- Stage interfaces: `simplify_interfaces` passed every stage with identical
  graphs, selected strokes, timed action trajectories and summaries. Repeated
  direct frame-only stages left no worker processes running.
- Single-arm data and shared assets: implemented together as one saved-data
  migration. `simplify_final` passed every stage, preserving construction,
  selected/dropped/spare strokes and all timed action trajectories. The
  frame-generation manifest digest changed with the asset-reference schema;
  comparison checked those hashes against their actual frame inputs before
  comparing the geometry.
- Final ghost: 12.6574761821 simulated print minutes, 15 lift-offs, 1.3927637719 m
  of detail, `FULL_BUILD_VERIFIED`. The verification digest matches the current
  plan and its collision scope is `full_robot`.
- Storage, including retained retry diagnostics and proof caches: 1,863 files /
  384,289,427 bytes before; 1,799 files / 220,718,896 bytes after (42.6% fewer
  bytes). Normalized mesh copies fell from ten to one; the geometry mesh,
  robot visual and robot URDF also each have one copy per object. All archived
  and selected cell snapshot input hashes and source references validate.
- Fresh setup: `./setup.sh` passed in the isolated changed-file snapshot
  `/tmp/pen-art-fresh-5m3cbor0/checkout`, including LFS assets, Flexiv revision
  `92ef7865d76585e6e08d291bdfe652d32f7740f4`, Node `v24.21.0` with checksum
  verification, and the viewer build. Its frame-only README pipeline also
  reached `FULL_BUILD_VERIFIED`; old root directory names are absent there.
- Boundary checks: missing/out-of-order stages, changed profiles, old asset /
  cell / plan / saved-state formats, escaping asset references and a tampered
  `plan_config.json` are rejected. Verification passed after restoring the
  tampered scratch input. The no-detail CLI case reports that the frame can
  be planned alone. Repeated final-format replays leave no worker processes.
- Browser: frame-only and full-detail results show the reference picture,
  textured model, wireframe and full robot cell; playback advances, the
  matching verified status is displayed, and no JavaScript exceptions were
  observed. Final screenshots: `/tmp/pen_art_final_viewer.png` and
  `/tmp/pen_art_final_viewer_cell.png`.
- Final Ruff, shell syntax, whitespace and viewer build checks pass. The viewer
  retains its existing bundle-size warning. No tests or dependencies were added.
- Existing runs were retained throughout validation. After validation, the user
  requested removal of all runs and publication of the changes: the entire
  ignored repository `runs/` directory (about 4.5 GB) was removed. Validation
  summaries and temporary logs remain recorded here; old downloads are retained.
  Unsupported old run formats require regeneration.

Validation outputs are in `/tmp/pen_art_simplify_{before,layout,interfaces,final}.log`,
`/tmp/pen_art_simplify_final_comparison.json`,
`/tmp/pen_art_final_frameonly.log`, `/tmp/pen_art_fresh_final_setup.log`,
`/tmp/pen_art_fresh_pipeline.log` and `/tmp/pen_art_no_detail.log`.
No runtime speedup is established: stage wall times were recorded, but some
validation ran concurrently. Physical printing, material behavior and robot
calibration were not measured; all verification is simulation.

## Outcome

Make the repository easy to explore from the README: examples, settings,
Python code and the viewer should have clear homes. Each command should lead
to an ordinary stage function, and each run should expose its useful results
without making learners navigate worker files and copied planning rounds.

Start with the folder layout. Change stage interfaces and saved data in
separate batches so regressions can be traced to one kind of change.

## Evidence and constraints

- `scripts/` contains only `setup_machine.sh`; `docs/` contained only a viewer
  screenshot before this plan. `objs/` holds the ghost mesh and picture.
- `tools/` and `external/` are ignored downloads: Node and the pinned Flexiv
  description. They are not additional source projects maintained here.
- `web/` has two TypeScript implementation files, CSS and HTML. Keeping the
  viewer separate is useful; a frontend rewrite is unnecessary.
- Geometry planning crosses `cli.py`, `stages.py`, an internal command-line
  program in `planning/section_build.py`, and the parallel planner. Detail
  likewise has a public parser and an internal parser.
- `cell.load_cell()` rejects anything other than one arm, but returns an arm
  dictionary. Arm collections continue through planning, saved plans and the
  viewer.
- `cell.prepare_cell()` and `detail.write_detail_source()` copy asset bundles
  into planning rounds. At review time, `runs/ghost` contained 1,863 files
  totaling about 384 MB, including ten `normalized.glb` files.
- `cell_drawing_variants.json` has an empty `variants` list; its input hashes
  are the useful part of the artifact.

Preserve one arm, no turntable, geometry-first planning, 24 workers, stroke
chaining, one pen orientation per stroke, the wire/detail budgets, and the
independent full-robot replay. Timing must never select or remove strokes.
Keep input-change detection, atomic writes, worker-result invalidation,
collision bounds and verification-to-plan binding. All results are simulation.

Do not flatten algorithms into giant files, introduce a generic pipeline
framework, replace the viewer stack, add dependencies, or restore tests.
Do not delete existing runs or downloads, rewrite Git history, commit or push
as part of merely documenting this plan.

## 1. Capture a comparison run

Before implementation, run the README pipeline with a new object name such as
`simplify_before`; preserve `runs/ghost`. Use 24 workers and tmux for long work.
Save the printed summaries, stage times, selected/dropped/spare strokes, frame
and detail lengths, lift-offs and verification verdict. Record file counts,
total bytes and repeated mesh assets separately from execution time.

The saved ghost currently reports about 12.7 print minutes, 15 lift-offs,
1.39 m of detail and `FULL_BUILD_VERIFIED`. Treat these as historical reference
values, not a substitute for the fresh baseline. Do not rerun unchanged checks
without a concrete reason.

## 2. Simplify the root layout and first-run instructions

Target layout, alongside existing package metadata and license files:

```text
setup.sh
configs/
examples/ghost/
  cute_ghost.glb
  reference.jpg
  preview.png
src/pen_art/
viewer/
docs/                     # This implementation plan

# Generated and ignored:
.cache/
  flexiv_description/
  node/                   # May point to the versioned Node directory
.venv/
runs/
```

Move `scripts/setup_machine.sh` to `setup.sh`, `web/` to `viewer/`, the ghost
inputs to `examples/ghost/`, and the README screenshot to that example's
`preview.png`. Keep the existing mesh filename so source manifests still
identify it. Update the setup script's repository-root calculation after the
move; its current `dirname/..` would otherwise select the parent repository.

Update all path consumers together: `README.md`, `AGENTS.md`, `.gitignore`,
`.gitattributes`, `src/pen_art/cell.py` and `src/pen_art/server.py`. Match the
example image directly beside its mesh instead of relying on partial filename
matches. Retain LFS for the mesh and existing LFS-managed reference image;
avoid accidentally converting the ordinary README screenshot to an LFS pointer.

Have setup install pinned downloads under `.cache/`, retaining the Flexiv
revision and Node checksum checks. Rebuild the generated cell when its asset
paths change: its URDF contains absolute paths. Leave old downloads and runs
in place; existing artifacts can still refer to them. No automatic cleanup or
permanent legacy-path adapter is needed.

Show `source .venv/bin/activate` after setup in the README, then use `pen-art`
consistently. Keep the short descriptions of each stage and update the code
map and development commands. Retain this plan in `docs/`.

Acceptance: setup succeeds in a fresh checkout of the changed files; the ghost
inputs are real LFS objects, the new viewer builds and opens, and the README
commands work from a fresh shell. Existing run data is untouched. Tracked
runtime references use the new paths; historical discussion in this plan is
allowed to mention the old ones.

## 3. Give each stage a direct Python interface

Use existing modules rather than adding another orchestration layer:

- In `detail.py`, extract a `generate_detail(...)` function from `main()`.
  The public CLI owns argument parsing; the function returns a meaningful
  result, including the no-detail case, instead of requiring exit-code parsing.
- In `planning/section_build.py`, expose a function that prepares and runs one
  geometry or timing round. Reuse `SectionBuild` and `run_parallel_forward`.
- Reuse `planning/cell_replay.py:replay_cell()` as the verification entry point.
- Update `cli.py` and `stages.py` to use these interfaces and explicit results.
  Retain the public commands and their concise output and failure reporting.

Assess the subprocess boundaries against actual worker cleanup. The planner
and replay use forked pools, module-level worker state and nonblocking pool
shutdown; direct calls must not leave workers or state behind. Keep a thin
process wrapper where isolation remains necessary. Remove duplicated internal
argument parsers once they have no remaining callers. Do not introduce a
generic job runner just to share a few calls.

Acceptance: a reader can follow each public command to its stage function;
failures propagate explicitly, logs remain available, and repeated stages
neither reuse stale state nor leave workers running. Ghost behavior matches
the baseline.

## 4. Finish the single-arm model and remove the empty artifact

Change `cell.py` and `cell_template.yaml` to represent a single arm directly.
Update `detail.PlannerCheck`, `planning/cell_planner.py`, `section_build.py`,
`parallel_build.py`, `cell_replay.py`, and the viewer's manifest and playback
readers together. Replace arm dictionaries, duplicate-ID checks and loops
whose only supported length is one. Keep useful robot/link identities and
all kinematic, tool and collision checks.

Fold the hashes from `cell_drawing_variants.json` into the existing cell
manifest. Update its readers in `cell_planner.py` and the plan export bindings
in `parallel_build.py`, then remove the empty artifact's writer and consumers.
Construct hashes without a manifest hashing itself.

Version changed artifact contracts explicitly and reject unsupported old
formats with regeneration instructions. Existing folders remain untouched;
new validation runs use fresh names. Do not silently reinterpret an old
verification verdict or build a compatibility framework for old experiments.

Acceptance: configuration, planner and viewer agree on one arm; no consumer
requires `cell_drawing_variants.json`; changing a bound input still invalidates
the result; a new ghost plan and playback match the baseline.

## 5. Make saved results navigable and share immutable assets

Refactor the existing ownership in `frame.py`, `detail.py`, `cell.py` and
`stages.py`. Store normalized mesh assets once per object. Keep frame/detail
results and the selected geometry/timing/verification results discoverable;
place round attempts, worker payloads, journals and proof caches under an
internal `.work/` directory. Retain journals and proofs that final replay
actually reads: internal does not mean disposable.

Replace whole-directory copies in `detail.write_detail_source()` and repeated
asset copies in `cell.prepare_cell()` with explicit references to shared
assets plus the graph and selection data that actually change per round.
Prefer object-relative paths and existing manifests over symlink-dependent
layouts or a new asset database. Keep one authoritative current result pointer.

Trace every changed reference through planning, replay and `server.py` into
`viewer/src/`. The viewer should resolve the selected result without knowing
the retry directory layout. Preserve server path-containment checks. Rebuild
input and proof bindings from the actual referenced files; moving a file must
never make a stale proof appear valid.

Acceptance: one normalized mesh per object, clear stage results, no duplicate
asset bundle per retry, and no loss of diagnostics needed to explain a rejected
stroke. Measure the resulting count and size reduction against step 1; do not
claim a runtime speedup without measuring it. Full replay and viewer playback
must still pass.

## Validation and completion

No new test suite. Before moving paths, use the current required checks:

```bash
unset PYTHONPATH
.venv/bin/ruff check src scripts
(cd web && PATH="$PWD/../tools/node/bin:$PATH" npm run build)
```

After moving paths, use:

```bash
unset PYTHONPATH
.venv/bin/ruff check src
(cd viewer && PATH="$PWD/../.cache/node/bin:$PATH" npm run build)
```

Run long pipelines in a dedicated tmux session. For example, start
`tmux new-session -s pen-art-simplify`, then run the following inside it after
setup. Use a distinct run name for each changed batch; before the layout move,
substitute `objs/cute_ghost/cute_ghost.glb` for the example path.

```bash
unset PYTHONPATH
source .venv/bin/activate
set -euo pipefail
run_name=simplify_layout
pen-art cell
pen-art frame examples/ghost/cute_ghost.glb --name "$run_name"
pen-art detail "$run_name"
pen-art plan-geometry "$run_name" --workers 24
pen-art plan-timing "$run_name" --workers 24
pen-art verify-full-robot "$run_name" --workers 24
pen-art view --port 8799
```

Capture command output and compare it with the baseline after each behavior
batch. Compare graph/plan content after accounting for intentional path/schema
changes; do not compare only the final verdict. Investigate changed stroke
choices, lift-offs, wire lengths or print times. Worker-dependent counters and
wall times can vary and must be reported separately.

Check frame-only planning after changing stage interfaces and saved outputs.
Exercise missing/out-of-order inputs and changed-input rejection on scratch
runs, plus the unsupported-schema path when introduced. Verify the example
picture, textured model, wireframe, robot playback and displayed verdict in a
browser; headless Chrome over SSH needs `env -u DISPLAY`.

Finish with one fresh-setup check of the final layout. Report what passed,
what changed and what was not measured, including the artifact-size comparison.
No unresolved product decision blocks preparation; process isolation and exact
artifact references are implementation choices to settle from their current
consumers and validation evidence.
