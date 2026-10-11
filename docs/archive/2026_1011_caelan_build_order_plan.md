# Build order before trajectory refinement

**Date:** 2026-10-11  
**Status:** Current-code review and full-graph preflight completed. Coupled regression integration pending.

## Decision

Use one Flexiv arm and one pen. Preserve the Palace wire geometry and uniform scale.
Allow extrusion stops, withdrawal, pen-off repositioning, return, and rejoining.
The immediate output is a complete, defensible construction order. Detailed timing
and viewer export follow a feasible order. Existing runs remain evidence, not a
mandatory prefix for the new offline plan.

## What the papers already solve

- [Caelan Garrett et al., robotic spatial extrusion](https://arxiv.org/html/2002.02360):
  sequence and robot-motion search on a fixed grounded element graph. Regression
  searches backward from the complete structure and terminal robot configuration.
  Each removal checks predecessor stiffness, the corresponding forward extrusion,
  and pen-off transit to the already planned suffix. Forward stiffness reasoning
  guides the search; expensive motion checks are deferred. This is the relevant
  foundation, including stops between elements. Its assumptions include straight
  elements and fixed orientation within each element; our curved sections require
  an explicit adapter. Its completeness result does not transfer automatically to
  our finite candidates and bounded search.
- [RL toolpath planner](https://arxiv.org/html/2408.09198v1): graph sequencing with
  deformation and printer-head collision objectives. Section 7.5 explicitly leaves
  global arm collision feasibility unresolved. It is optional future sequencing
  guidance, not a substitute for the robot-motion feasibility interface.
- [WIR3D](https://arxiv.org/abs/2505.04813) supplies visually meaningful curves,
  including texture cues. It is upstream of construction feasibility.
- [Fabricable 3D Wire Art](https://kenji-tojo.github.io/publications/fab3dwire/)
  targets wire bending with generated jigs. Its fabrication constraints do not
  directly describe molten-plastic extrusion.
- [View-independent wire art](https://github.com/THLsword/3DWireArt) addresses
  wire geometry through manifold fitting.
- [WirePrint](https://hcie.csail.mit.edu/research/wireprint/wireprint.html)
  provides a concrete contour/zigzag extrusion process, including pauses and
  cooling. Its process data must not be treated as calibration of our pen.

## Current implementation gaps

| Evidence | Consequence | Change |
| --- | --- | --- |
| `construction.construction_preview` checks grounded topology, then chooses the first start option | A connected order can still be too flexible or inaccessible | Retain direction/start choices; evaluate stiffness and motion inside order search |
| `SingleArmBuild.run_remaining` walks a fixed `selection` and stops on failure | Local repairs cannot undo an earlier obstructing support/order choice | Backtrack order and extrusion mode instead of repeatedly repairing one fixed sequence |
| `endpoint_support.plan_endpoint_support` inserts a post after a prefix fails release | A useful post can block a future stroke; this happened on Torus | Freeze explicit supports before a search; additions create a new graph revision and trigger replanning |
| Original source contours remain the main action units | A long contour can demand many incompatible robot configurations | Use independently joinable sections with exact source-interval provenance |
| Palace repair introduced a 1.686 rad adjacent joint jump | Valid isolated poses produced invalid interpolation | End the extrusion section where needed and plan a real pen-off transition |
| Base-first fixed the Torus diagonal obstruction | Useful initialization, not a general sequencing solution | Keep base-first as a preference, not an unconditional scaffold-first constraint |
| `coupled.plan_sequence` has backtracking but requires fixed complete drawing paths | Alternatives that need a different local extrusion mode are unavailable | Reuse collision/transit primitives; replace the fixed drawing-path prerequisite |

These are implementation and modeling gaps. They do not show that Caelan's
formulation fails on our problem. We have not run a faithful full adaptation of
that formulation on these inputs.

## New prerequisite evidence

Reproducible reports are in each run's `build_order/preflight.json`.
They screen the original fixed `stroke_graph.json`, not just its accepted prefix.

| Input | Elements | Full-goal result |
| --- | --- | --- |
| `general_torus_one_arm_base_first` | 91 | Configured model rejects it: maximum displacement 29.207 mm versus 1 mm; small-deflection assumption also fails |
| `palace_15cm_one_arm` | 724 | Structural solve unresolved: relative residual 0.001546 versus required less than 0.00001 |

The current material is explicitly unmeasured: elastic modulus 1 GPa, rigid
welds, clamped ground contacts and cooled strands. Torus's predicted magnitude
is outside the linear model's validity range, not an accurate physical sag
prediction. These results require model/graph investigation, not relaxed limits.
Palace includes a 6.78 micrometre graph member; this is a diagnostic lead, not a
proven explanation for its numerical failure.

The already augmented full graphs were also screened during review. Existing
prefix supports did not resolve Torus's final-graph rejection or Palace's
numerical uncertainty. No source geometry or tolerances were changed.

## Algorithm and interfaces

The planning state is `(P, q)`: the material subset and the robot configuration
at the boundary with the already planned suffix. Start from the full fixed
graph and the requested terminal/home configuration. For a proposed last
element, form its predecessor material subset and require a supported start.
Validate the predecessor structure, a forward extrusion, then transit through
the post-extrusion scene. Recurse backward; at the empty graph, connect the home
configuration to the first extrusion. Backtrack on a failed order or mode.

For this project:

1. **Freeze elements and supports.** Retain source IDs, exact curve intervals,
   ground contacts, diameter, and weld identities. Begin with existing material
   junctions. Add exact within-curve cuts for restarts when needed. A cut does not
   make a floating point an anchor or imply a rigid support. Do not force original
   whole-stroke `depends_on` choices into the new search; derive available starts
   from actual predecessor material.
2. **Preflight the goal.** An invalid or numerically unresolved final graph is
   returned to modeling/support design before order search. Splitting an edge
   alone must not alter its final mechanical behavior.
3. **Build a stiffness guide.** Use grounded forward additions and the existing
   beam model to propose a structurally sensible priority. The guide is a
   preference; geometric failures can change the eventual order.
4. **Search backward with lazy feasibility.** Cheap connectivity/stiffness checks
   precede robot solving. Request a local extrusion mode only for the selected
   branch. Pen-off transitions are expected, not penalized as failures. Stop at
   the first complete feasible order; no makespan optimization or broad mounting
   search.
5. **Handle failure at the right level.** No continuous drawing solution means
   try a legal section boundary, another mode or another order. A sampled motion
   failure means unresolved under that effort, not proven geometric impossibility.
   No structural order may trigger a finite support revision; record added
   geometry and restart the fixed-graph search. Never silently insert a post into
   an already approved graph/order.
6. **Replay forward.** Verify every source interval is deposited once, starts
   exist when used, release states pass the structural model, extrusion and
   transit scenes are correct, and home return exists. Then synchronize timing,
   cooling and pen commands using existing exporters.

Only a state-independent structural result may be cached by material subset.
Robot feasibility depends on configuration, mode, tool, obstacles and material.
Keep unknown/retryable motion outcomes distinct from structural rejection.
Do not equate a preview order with a robot-validated order.

## Modules to reuse

- `structure.py`: beam model and structural diagnostics; numerical reliability
  must be resolved before using it as the search oracle.
- `curves.py`, `strokes.py`: exact curve restrictions, source intervals and direction.
- `planning/sequence.py`: existing backtracking/state separation patterns.
- `planning/cell_planner.py`, `collision.py`, `scene.py`, `rrt.py`: collision,
  approach/withdrawal, transit and local deposition checks.
- `planning/stroke_restarts.py`: release semantics and existing rejoin behavior.
- `planning/endpoint_support.py`: support proposal ingredients, moved outside
  fixed-graph order search.
- `planning/full_build_export.py`, `source_coverage.py`: complete coverage, return
  and replay after a plan is found.

## Implementation phases and acceptance

### 1. Input and model audit

- [x] Compare current call paths with the paper and observed failures.
- [x] Add `planning/build_order_preflight.py` and save full-goal diagnostics.
- [ ] Resolve Palace's numerical structural model without deforming the source.
- [ ] Validate Torus's structural representation and propose explicit supports
  if its configured final structure still fails.
- [ ] Freeze a source-mapped element graph; confirm exact coverage and unchanged scale.

### 2. Coupled regression order

- [ ] Add a backward `(P, q)` search with lazy local extrusion and transit callbacks.
- [ ] Keep alternate directions and restart modes available; include initial and
  terminal transit in search, not as a later surprise.
- [ ] Add forward stiffness priorities and bounded retry diagnostics.
- [ ] Emit ordered element/source intervals, chosen modes, pen-off transitions,
  structural evidence, graph/input hashes and unresolved outcomes.

### 3. Palace and Torus order demonstrations

- [ ] Run from the empty base, independent of historical accepted prefixes.
- [ ] First obtain complete orders with explicit confidence/scope labels.
- [ ] Connect resulting section actions to existing timing/replay/viewer adapters.
- [ ] Validate one full Palace and one Torus build without human curve joining.

## Focused validation

Use existing scenes and narrowly test the failure classes: an obstructing support
must change the chosen order; a robot configuration change must create a pen-off
transition; a structurally invalid goal must be rejected before motion work.
Avoid expanding unrelated tests or optimizing runtime before these work.

```bash
.venv/bin/ruff check src/pen_art/planning/build_order_preflight.py
.venv/bin/python -m pen_art.planning.build_order_preflight --run runs/palace_15cm_one_arm
.venv/bin/python -m pen_art.planning.build_order_preflight --run runs/general_torus_one_arm_base_first
```

## Remaining risks

Pen temperature, feed delay, welding, cooling, and safe unsupported deposition
remain uncalibrated. A 15 cm bounding box does not itself establish printability.
Numerical structural failure is not physical failure. Curved-element and hot-pen
extensions are ours; do not attribute their guarantees to the cited paper.
