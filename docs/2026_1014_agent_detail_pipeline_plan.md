# Agent Detail Pipeline Plan

**Date:** 2026-10-14
**Status:** Planning

A reusable pipeline that adds about 2 m of detail on top of a planned WirePrint
frame, drawn after the frame, within 15 extra minutes of print time. Tools make
lines drawable and plannable; an agent (Claude Code, Codex, or a headless vision
model) decides which features make the object recognisable and which lines
express them.

## Motivation

- **Detail today is chosen by geometry, not meaning.** `scripts/detail_tours.py`
  ranks slice creases by length x depth. On the ghost the detail landed on the
  hat brim; the eyes, mouth and blush are painted texture, invisible to creases.
  The only semantic part is `liberty_features` (`scripts/detail_tours.py:190`),
  hand-written for one statue.
- **Creases are not worth keeping** (decided 2026-10-14). On AI-generated meshes
  thresholds of 0.03-0.12 mm pick up surface noise far below a 1 mm wire.
- **Generation is slow.** Measured on the ghost (6.5 min, one core): planner
  check 183 s, laying lines on the surface 108 s, joining 42 s, tilt checks 24 s,
  creases 19 s. The Palace took an hour: its mesh has 2.85 M faces and a 300-point
  signed-distance query takes 10.2 s (ghost 0.6 s).
- **The tool layer works.** Liberty, ghost, pumpkin and Palace each planned and
  passed full-robot verification at about 2 m of detail and 7.7-11.5 extra
  minutes (`.claude/skills/detail-tours/SKILL.md`, results table).

**Target:** `scripts/detail_pipeline.py <stage> <name>` runs fixed stages that
read and write files in `runs/detail/<name>/`. The agent acts only in **choose**
and **review**, through a documented `choice.json`. The same flow works for any
mesh in `objs/` and any agent that can read images.

**Non-goals:** changing the frame generator or the motion planner (beyond what
the tool layer already uses); differentiable curve optimisation; drawing
undersides out of the tilted pen's reach.

**Budget and rules carried over** (from the skill): detail rests one wire outside
the surface and is fused where it touches; the tip may touch existing wire, the
nose keeps 0.3 mm; the pen tilts 0-60 degrees toward the robot; one turntable
face per stroke; tours start on a frame node; measured cost about 12.5 s per tour
plus drawing at about 14 mm/s; budget 2 m and 15 minutes per object.

## Existing Shared Modules to Reuse

- `scripts/detail_tours.py:92` `Surface` (`push_out`, `lay`): laying lines one
  wire outside the surface. Will move to the library and use the work mesh.
- `scripts/detail_tours.py:268` `dip`: bend lines onto nearby frame wire.
- `scripts/detail_tours.py:327` `detail_profile`: tilts, 0.3 mm nose clearance,
  `planning_checks_robot`, 15 mm back-away.
- `scripts/detail_tours.py:345` `cubic_pieces`, `:366` `PlannerCheck` (`check`,
  `in_and_out`): the planner's own pen screen, drawing sampler, posture limit,
  approach and withdrawal per line, with cut suggestions.
- `scripts/detail_tours.py:539` `main`: face assignment (`sector_of`), tilt clear
  sets (`clear_sets`, `nose_hits`), joining (`bridge`), peel ordering
  (`allowed`), `tilt_profile`, `supported`, graph build with frame cuts and
  `fused_turns`, budget fill. These become library functions.
- `scripts/detail_tours.py:1108` `tilt_hints`: copy per-tour tilts to actions.
- `scripts/plan_detail.py:42` `drop_tours`, `:66` `group_faces`, `:96`
  `planned_seconds`, `:108` `main`: plan rounds with drops, replay rejections,
  budget trim, detail-only minutes.
- `scripts/frame_source.py:23`: frame source and profile for a new mesh.
- `src/pen_art/cell.py:297` `prepare_cell`,
  `src/pen_art/planning/section_build.py:134` `prepare_candidates`: the check
  cell used by `PlannerCheck`.
- `src/pen_art/planning/sections.py:81` `section_actions`,
  `src/pen_art/planning/section_build.py:33` `write_section_artifacts`,
  `src/pen_art/structure.py:148` `analyze_structure`: writing and screening the
  detail source.
- `src/pen_art/planning/turntable.py:195` `build_transform`: world pose per
  turntable position (cameras and checks).
- `src/pen_art/abstraction/wir3d_adapter.py:82` `import_curves`: optional WIR3D
  candidates in the build frame.
- `src/pen_art/server.py:92` `get_picture` and `web/src/simple.ts:34` panel 0:
  the original picture for an object, used as the agent's reference.
- `scripts/plan_frame.sh`, `scripts/benchmark_parallel_build.py`: frame planning
  and each plan round.

## Phased Breakdown

### Phase 0: Library and driver skeleton

- **Problem:** the tool layer is one 1126-line script with creases and Liberty
  code mixed in; nothing can call its pieces separately or test them.
- **Solution:** create `src/pen_art/detail/` with `geometry.py` (resample,
  smooth, dense, nose radius, `Surface`), `drawable.py` (face, tilt, dip,
  `PlannerCheck`, cutting), `tours.py` (join, order, budget, graph build),
  `plan.py` (from `plan_detail.py`), `contracts.py` (JSON schemas and
  validators). Add `scripts/detail_pipeline.py` with subcommands and a work
  folder `runs/detail/<name>/` holding `pipeline.json` (inputs, stage status,
  hashes). Remove `slice_creases`, `liberty_features`, `candidate_lines` and the
  crease options. Keep `scripts/detail_tours.py` and `scripts/plan_detail.py` as
  thin wrappers for one release, then delete.
- **Impact:** each stage is a function with file inputs and outputs; the
  existing verified runs remain reproducible from their recorded sources.

### Phase 1: Prepare (work mesh, views, check cell)

- **Problem:** distance queries on full meshes dominate run time; there is no
  shared camera set for the agent's images and for sketch projection.
- **Solution:**
  - Work mesh: quadric-reduce the frame source's `normalized.glb` to the fewest
    faces whose two-way sampled distance to the original is within 0.3 mm (99th
    percentile 0.2 mm). Measured: Palace 2.85 M to 300 k faces (max 0.29 mm),
    ghost 233 k to 30 k (max 0.39 mm, so it gets more faces), 2 s each.
  - Colour: each reduced face takes the texture colour at the nearest point of
    the full textured mesh (UV interpolation, base-colour texture).
  - Views: 8 cameras, one per turntable face (the face looking at the robot) at
    20 degrees elevation, the four diagonals at 20 degrees, plus top. Per view:
    `view_<k>.png` (colour render of the work mesh, 768 px) and `view_<k>.npz`
    (pixel to face index and surface point, by ray casting).
  - Detail profile and check cell (from `detail_profile`, `prepare_cell`,
    `prepare_candidates`), the frame wire, and `prepare.json` (error, face
    counts, camera matrices, picture path).
- **Impact:** every later stage uses one fast mesh and one camera set.

### Phase 2: Fast, parallel tool layer

- **Problem:** `push_out` uses ray-cast signed distance; all per-line work runs
  on one core.
- **Solution:** inside/outside from the closest face's normal
  (`dot(p - closest, n) < 0` means inside); per-line laying, tilt sets and
  `PlannerCheck` in a forked process pool (24 workers), each worker building its
  own planner once.
- **Impact:** a ghost round in under a minute and a Palace round in a few minutes
  (to be measured; Phase 7 records the numbers).

### Phase 3: Propose (candidates and sheets)

- **Problem:** candidates must cover what makes objects recognisable, including
  painted features, and must arrive already drawable.
- **Solution:**
  - **Colour regions:** cluster face colours (Lab, k-means, k chosen by
    silhouette score in 4-12), smooth labels over face adjacency (majority, 3
    passes), drop regions below 20 mm^2; region boundaries become polylines
    (chain shared edges, smooth, drop under 8 mm).
  - **Silhouettes:** for each view, edges between front- and back-facing faces
    that are visible in the pixel map; chained and smoothed.
  - **WIR3D (optional, `--wir3d run_dir`):** `import_curves` lines.
  - **Agent sketches** enter later, in realize.
  - Every candidate goes through the tool layer (lay, dip, face split, tilt
    runs, `PlannerCheck` with cutting) and is stored with its drawable pieces.
  - `candidates.json`: id, source, polyline (build frame, m), pieces (points,
    turntable face, tilts that work), length, estimated seconds, drawable
    fraction, and the views where it is visible.
  - `sheet_<k>.png`: the view render faded, each drawable candidate drawn and
    labelled with its id, colour by source; undrawable parts dashed.
- **Impact:** a numbered, plannable candidate set the agent can choose from by
  looking.

### Phase 4: Choose (agent contract)

- **Problem:** the agent needs a stable contract and a rubric that work for any
  agent.
- **Solution:**
  - `choice.json` schema (`contracts.py`): object, one-line description,
    features in priority order, each with name, why, candidate ids in order, and
    sketches (view id plus normalised 2D points), plus the budget. Validator:
    known ids, sketches inside the image, at least one feature.
  - `detail_pipeline.py choose <name>`: prints what the agent should look at
    (picture, sheets, `candidates.json` summary) and validates the file the
    agent writes.
  - Optional `--model <id>`: send the picture, sheets and rubric to a vision
    model through the Anthropic API (default `claude-opus-5-5`), write and
    validate `choice.json`. Key from `ANTHROPIC_API_KEY`; off by default.
- **Impact:** semantic decisions sit in one file any agent can write.

### Phase 5: Realize (choice to detail source)

- **Problem:** turn the agent's features and sketches into tours in its
  priority order, within budget.
- **Solution:**
  - Sketch projection: map each 2D point through `view_<k>.npz` to a surface
    point, resample at 1 mm, and send it through the tool layer like any
    candidate.
  - Tours: join pieces of the same feature on the same face (connecting runs up
    to 30 mm), then order by peeling (`allowed`), then group faces.
  - Fill the budget feature by feature in priority order, estimated with the
    measured cost (12.5 s per tour, 14 mm/s).
  - Each tour carries `detail_feature`; write the detail source with the
    existing graph build, `tilt_hints` and the structure screen.
- **Impact:** a detail source whose every tour traces back to an agent decision.

### Phase 6: Plan and report

- **Problem:** the agent needs results in its own terms to review them.
- **Solution:**
  - `plan` stage: the existing round loop (drops, replay rejections, trim by
    planned seconds), refilling a dropped tour with the next candidate of the
    same feature before dropping the feature.
  - `report.json`: per feature, chosen metres, planned metres, tours kept and
    dropped with reasons; totals for detail metres, added and detail-only
    minutes, and the final run.
  - `result_<k>.png`: the planned wire (frame grey, detail coloured by feature)
    from the same cameras.
  - Add the final run to `runs/viewer_runs.json`.
- **Impact:** the agent can see which features survived.

### Phase 7: Review loop, agent integration, and the four objects

- **Problem:** the loop and the instructions must be usable by other agents.
- **Solution:**
  - `review` stage: shows report, result images and picture; the agent edits
    `choice.json`; the driver reruns realize and plan; at most 3 rounds.
  - Rewrite `.claude/skills/detail-tours/SKILL.md` as the agent loop with the
    rubric:
    - front first;
    - features that are not already in the silhouette;
    - long lines over short ones;
    - name every feature before choosing lines;
    - sketch only where no candidate fits.
  - Add `AGENTS.md` pointing Codex to the same file.
  - Run the ghost with Claude as the agent and compare it with
    `geomfirst_gf98_ghost_detail_r14_w24`, then the pumpkin, Liberty, and the
    Palace on its new frame (`geomfirst_gf101_palace2_w24`).
- **Impact:** a documented, repeatable pipeline with measured results on four
  objects.

## Risk Assessment

| Risk | Impact | Mitigation |
| --- | --- | --- |
| Colour regions are noisy on baked or photo textures (Palace) | Too many short candidates; sheets unreadable | Region smoothing and minimum area; cap candidates per view at 60 by drawable length; the agent can sketch instead |
| Painted features sit where the pen cannot reach (undersides, recesses) | A chosen feature plans to nothing | Candidates carry drawable fraction; the sheet dashes undrawable parts; report says which features failed and why |
| Quadric reduction loses UVs or thin features | Colour or small shapes wrong | Colour is taken from the full mesh, not the reduced one; error bound 0.3 mm keeps anything a wire can show |
| Agent choices vary between runs or agents | Inconsistent results | Fixed rubric; feature names and reasons stored; review compares against the picture; headless mode logs the model id |
| Planner drops many chosen tours | Detail falls short of 2 m | Refill from the same feature's next candidate; report per feature; the agent adds alternatives in review |
| Parallel workers multiply memory (each builds a planner) | Out of memory on 62 GB | Workers share forked state; cap at 24; measure RSS in Phase 2 |
| Test budget (29 of 30 used: 25 Python, 4 browser) | No room for a new test function | Fold assertions into an existing test function (see 0.4); do not add test functions |
| Headless model call needs network and a key | Unattended runs fail | Optional flag; the file contract works without it |

## Detailed Checklist

### Phase 0: Library and driver skeleton

- [ ] **0.1** Create `src/pen_art/detail/__init__.py`, `geometry.py`,
  `drawable.py`, `tours.py`, `plan.py`, `contracts.py`; move code from
  `scripts/detail_tours.py` (`Surface`, `dip`, `detail_profile`,
  `cubic_pieces`, `PlannerCheck`, face, tilt, join, order, budget, graph build,
  `tilt_hints`) and `scripts/plan_detail.py` (`drop_tours`, `group_faces`,
  `planned_seconds`, round loop).
  - Acceptance: rebuilding the ghost detail source from
    `runs/geomfirst_ghost_detail4_source/detail_tours.json` settings gives the
    same tours (same count and lengths within 1 mm).
- [ ] **0.2** Delete `slice_creases`, `liberty_features`, `candidate_lines`,
  `--crease-threshold`, `--fine-region`, `--liberty`, `--min-line`.
- [ ] **0.3** `scripts/detail_pipeline.py` with subcommands `prepare`,
  `propose`, `choose`, `realize`, `plan`, `review`, `run` (all non-agent stages
  in order), and `runs/detail/<name>/pipeline.json` (frame source, frame run,
  base profile, picture, stage status, input hashes; a stage refuses to run on
  stale inputs).
- [ ] **0.4** Fold pipeline assertions into
  `tests/test_wireprint_nozzle.py::test_slice_height_follows_the_nozzle_on_a_cube_and_a_solidified_fork`
  (no new test functions; total stays 29). Each phase below adds its assertions
  there.

### Phase 1: Prepare

- [ ] **1.1** `geometry.work_mesh(mesh, tolerance=0.0003)`: search face counts
  (halve until error exceeds tolerance, then bisect), report max and 99th
  percentile error.
- [ ] **1.2** `geometry.transfer_colour(full_scene, reduced)`: per reduced face,
  base-colour texture at the nearest point on the full mesh (barycentric UV).
- [ ] **1.3** `views.cameras(profile, bounds)`: 8 cameras as in Phase 1; save
  intrinsics and extrinsics in `prepare.json`.
- [ ] **1.4** `views.render(work_mesh, camera)`: colour image and pixel map
  (face id, surface point) by ray casting the work mesh.
- [ ] **1.5** Write the detail profile, check cell and frame wire into the work
  folder; `prepare.json` records everything.
- [ ] **1.6** Test (in 0.4's function): a two-colour sphere reduces within
  0.3 mm and keeps both colours on the right faces.

### Phase 2: Fast, parallel tool layer

- [ ] **2.1** `Surface.push_out`: closest point plus face-normal sign, replacing
  `trimesh.proximity.signed_distance`.
- [ ] **2.2** `drawable.make_drawable(lines, workers=24)`: per-line laying, dip,
  face split, tilt runs and `PlannerCheck` with cutting in a fork pool; each
  worker builds its planner and frame scenes once.
- [ ] **2.3** Record per-stage seconds (existing timers) and worker peak RSS in
  `propose.json`.
  - Acceptance: ghost propose under 60 s; Palace under 5 min.

### Phase 3: Propose

- [ ] **3.1** `candidates.colour_boundaries(work_mesh)`: Lab k-means (k by
  silhouette score, 4-12), adjacency smoothing, 20 mm^2 minimum region,
  boundary chaining, 8 mm minimum line.
- [ ] **3.2** `candidates.silhouettes(work_mesh, cameras, pixel_maps)`:
  visible front/back edges per view, chained.
- [ ] **3.3** `candidates.wir3d(run_dir)` behind `--wir3d`.
- [ ] **3.4** Write `candidates.json` (fields as in Phase 3), sorted by drawable
  length; cap 60 per view in sheets.
- [ ] **3.5** `sheets.draw(view, candidates)`: `sheet_<k>.png` with the faded
  view, numbered lines and a legend.
- [ ] **3.6** Test: on the two-colour sphere, the colour boundary candidate is
  the equator within 1 mm.

### Phase 4: Choose

- [ ] **4.1** `contracts.CHOICE_SCHEMA` and `validate_choice(choice, candidates,
  cameras)`.
- [ ] **4.2** `choose` subcommand: list the picture, sheets and a short
  candidate table; validate an existing `choice.json`.
- [ ] **4.3** `--model`: Anthropic Messages API call with the picture, sheets
  and rubric (`claude-opus-5-5` by default); parse, validate, write; record the
  model id in `choice.json`.
- [ ] **4.4** Test: the validator rejects unknown ids and out-of-image sketch
  points.

### Phase 5: Realize

- [ ] **5.1** `views.project_sketch(points, camera, pixel_map)`: 2D to surface,
  1 mm resampling; then `make_drawable`.
- [ ] **5.2** `tours.build(choice, candidates)`: join per feature and face
  (30 mm runs), peel order, face grouping, budget fill in feature priority with
  the measured cost model, `detail_feature` on every tour.
- [ ] **5.3** Write the detail source (graph build, `tilt_hints`, structure
  screen) and `realize.json` (chosen metres per feature, estimated minutes).
- [ ] **5.4** Test: a sketch drawn across the sphere's front view lands on the
  surface within 0.5 mm of the expected arc.

### Phase 6: Plan and report

- [ ] **6.1** `plan.run(...)`: round loop from `plan_detail.py`, plus refill of
  dropped tours from the same feature's next candidate.
- [ ] **6.2** `report.json` per feature and totals (Phase 6 fields).
- [ ] **6.3** `result_<k>.png` from the plan's deposited wire.
- [ ] **6.4** Add the final run to `runs/viewer_runs.json` with a descriptive
  name.

### Phase 7: Review loop, integration, objects

- [ ] **7.1** `review` subcommand: show report, results and picture; rerun
  realize and plan after the agent edits `choice.json`; stop after 3 rounds.
- [ ] **7.2** Rewrite `.claude/skills/detail-tours/SKILL.md` (stages, contracts,
  rubric, commands, results).
- [ ] **7.3** Add `AGENTS.md` (Codex) pointing to the skill file and contracts.
- [ ] **7.4** Run ghost, pumpkin, Liberty, Palace (new frame); record detail
  metres, added minutes, features kept, and stage times in the skill's results
  table; compare ghost with `geomfirst_gf98_ghost_detail_r14_w24`.
- [ ] **7.5** Delete `scripts/detail_tours.py` and `scripts/plan_detail.py`
  wrappers once 7.4 passes.

## Open Questions

- Camera set: 8 views as above, or views derived from the picture's angle?
  Assumed: fixed 8, with the turntable-0 robot-facing view as "front".
- Colour clusters on photo-textured models (Palace): k-means may need a
  texture-gradient method instead; decided after 3.1 is measured on the Palace.
- Whether WIR3D candidates are worth their GPU hour per object; assumed
  optional and off by default.

## Validation Commands

- `.venv/bin/ruff check src tests scripts`
- `.venv/bin/pytest -q` (25 functions; must stay 25)
- `env -u DISPLAY tools/node/bin/node --test web/tests/*.test.mjs` (4)
- `PYTHONPATH=src .venv/bin/python scripts/detail_pipeline.py run ghost` then
  `choose`, `realize`, `plan`, `review` as the skill describes
