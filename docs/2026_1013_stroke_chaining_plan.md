# Stroke Chaining Plan

**Date:** 2026-10-13
**Status:** Planning. Phases 0 and 1 are being implemented first; nothing below
is measured unless it says so.

## Motivation

Print time, not planning time. On the branching shape (151 strokes, 30.4 min)
drawing is 453 s; the rest is the pen lifting, moving and waiting between
strokes. Every stroke is its own unit: travel, approach, feed start, join
dwell, draw, feed stop, cooling hold, withdraw. Per stroke that is about 3 s of
lift cycle and 3.5 s of waits.

Chaining means: when the next stroke starts where this one ended, keep the pen
down and carry on. A chained join removes the withdraw, travel and approach,
the feed start lag and join dwell of the next stroke, and the end hold of this
one (a riser top keeps a 1 s corner hold).

Decided 2026-10-13: chain strokes, with the build order changed to "zigzag then
its contour, layer by layer" (version 1). Not chosen: climbing one loop several
layers ahead (version 2), which leaves wire up to 48 mm above unfinished
neighbours and may block the pen.

### Measured so far (`scratchpad/reorder_check.py`, structure only)

| | Order | Joins sharing a node | Structural steps failing |
| --- | --- | ---: | ---: |
| Branching, 150 joins | today | 65 | 0 of 151 |
| | version 1 | 122 | 0 of 151 |
| Palace, 270 joins | today | 30 | 0 of 271 |
| | version 1 | 157 | 0 of 271 |

Estimated print time with version 1, at about 6 s per chained join: branching
30.4 to about 18 min, Palace 56.0 to about 41 min.

**Correction to those counts.** They count a contour's two joins separately. A
closed contour ends where it starts, so it can take over from the zigzag below
*and* hand over to the zigzag above only if those two meet it at the same
node. Today they do not: a zigzag ends on the top of its last riser, and the
next zigzag starts one spacing further along, above the first riser. Phase 0
fixes this in the wireframe. Until it is measured, the counts above are an
upper bound.

### Non-goals

- Version 2 ordering.
- Changing any wait duration (they are placeholders; a fan is planned).
- Curved (spline) approach and withdraw.
- Chaining across a table turn.

## Existing Code to Reuse

- `abstraction/wireprint.py` `anchors()` (zigzag feet and tops) and
  `align_loop()` (where a contour's seam sits): Phase 0.
- `planning/regression.py:15` `greedy_forward_order`: sorts by
  `row["priority"]`, requires an available anchor and a passing structural
  screen. Phase 1 changes only the priorities it starts from.
- `planning/sections.py:171` sets `order_hint`, the source of `priority`.
- `planning/motion_library.py` `DrawingRequest.q_start` and the `fixed`
  endpoint handling in `_adapt`: a stroke can already be adapted from a fixed
  starting joint position.
- `planning/parallel_build.py` `ForwardSchedule`: jobs are keyed by the results
  they read, so a result that changes invalidates its dependents. Phase 2 adds
  one more dependency.
- `planning/cell_planner.py` `complete_local_drawing` builds the operation
  list (`TRAVEL`, `APPROACH`, `PEN_START_WAIT`, `JOIN`, draw, `PEN_STOP_WAIT`,
  `WAIT`, `WITHDRAW`): Phase 3 omits the parts a chained join does not need.
- `planning/clock_path.py` `split_drawing_operations`: the corner hold
  (`PEN_STOP_WAIT`, `VERTEX_COOL`, `PEN_START_WAIT`) is reused for a join at a
  riser top.
- `planning/process_clock.py` `audit_process_clock`, `planning/turntable.py`
  `audit_indexed_clock`: the feed state machines Phase 4 extends.

## Phased Breakdown

### Phase 0: Each zigzag starts where the previous one ended

- **Problem:** a contour cannot chain on both sides, because the zigzag below
  ends at one node and the zigzag above starts at another.
- **Solution:** in the generator, number a zigzag's feet starting from the
  point where the zigzag beneath its lower contour put its last riser top,
  instead of from the contour's seam. With equal counts the risers stay
  aligned across layers. Where outlines differ (nearest-point pairing) the
  first top is placed at that point instead.
- **Impact:** a contour started at the top of the last riser below ends at the
  first foot of the zigzag above. Changes every wireframe; all eight objects
  are regenerated and re-verified.

### Phase 1: Zigzag then its contour, layer by layer

- **Problem:** the order draws all of a layer's zigzags, then all its contours.
- **Solution:** set priorities so that within a layer each upper loop's zigzags
  are immediately followed by its contour, and a layer begins with the loop
  standing on the contour just finished. `greedy_forward_order` and its
  structural screen are unchanged.
- **Impact:** more consecutive strokes share a node. No print-time change yet.
  Measured gate: count joins where stroke k's end node equals stroke k+1's
  start node (contours counted once, with one shared node for both sides).

### Phase 2: Propose a chained stroke from its predecessor's end

- **Problem:** strokes are adapted independently, so even where they meet, the
  arm's posture differs (median 25 degrees on the branching shape).
- **Solution:** for a chainable join, the proposal of stroke k+1 reads the
  accepted proposal of stroke k and adapts with `q_start` fixed to its final
  joint position, the same pen orientation and the same table position. A
  closed contour is started at the shared node. If that adaptation fails, the
  stroke is proposed as today and the join is not chained.
- **Impact:** proposals along a chain run one after another instead of in
  parallel. Chains break at every unchained join, so the depth is the chain
  length, not the stroke count. Planning time will rise; to be measured.

### Phase 3: Build chained clocks

- **Problem:** every stroke's clock begins with travel and approach and ends
  with a hold and a withdraw.
- **Solution:** `complete_local_drawing` takes two flags. *Chained in*: no
  `TRAVEL`, `APPROACH`, `PEN_START_WAIT`, `JOIN`; the clock starts with the
  first drawing edge. *Chained out*: no `WAIT` and `WITHDRAW`; after a zigzag
  (free riser top) the corner hold stays, after a contour nothing is added.
  The connecting transit of a chained stroke is empty. Each stroke stays its
  own clock, proof and deposit unit.
- **Impact:** the print-time gain. A stroke's collision check no longer covers
  an approach or withdraw it does not have.

### Phase 4: Audits and verification accept chained clocks

- **Problem:** the process audits assume feed off at both ends of every
  stroke and a withdraw after cooling.
- **Solution:** record `chained_from` and `chained_to` in the clock's
  metadata. `audit_process_clock` and `audit_indexed_clock` start with feed on
  for a chained-in clock and allow feed on at the end of a chained-out one,
  and require the neighbouring stroke named in the metadata to exist, be
  adjacent in time and meet at the same joint position. Verification
  (`cell_replay`) checks the same pairing on the saved plan before accepting
  any proof.
- **Impact:** a chained plan verifies; an inconsistent pairing is a rejection.

### Phase 5: Measure and document

Both shapes, then all eight objects: print time, joins chained, rejections,
planning and verification time, tip acceleration. Update the README table and
the fast-planning document.

## Risk Assessment

| Risk | Impact | Mitigation |
| --- | --- | --- |
| Fixed-start adaptation fails often | Few joins chain | Fall back to an unchained stroke; report the chained share |
| Serial proposals along long chains slow planning | The single-loop trunk of the branching shape is one long chain | Measure; if needed cap chain length so planning stays parallel |
| A rejected stroke invalidates the strokes chained after it | Re-planning cascades | Job keys already carry dependencies; a chained stroke falls back to unchained after one rejection |
| The pen stays down across a join where the wire is still soft | Physical, not modelled | Riser tops keep their hold; contour-to-zigzag joins are on supported wire; needs a real test |
| Feed never stops along a long chain | Physical, not modelled | Corner holds still stop and restart the feed at every riser top |
| Phase 0 changes all wireframes again | Results before it are not comparable | Regenerate and re-verify all eight; old sources kept |
| Audits loosened incorrectly | A bad clock passes | Feed-on at an end is accepted only with a matching neighbour; a test for a mismatched pair |

## Detailed Checklist

### Phase 0

- [ ] **0.1** `wireprint.py` `anchors()`: start foot numbering at the previous
  zigzag's last top on the lower contour (stored per loop while building
  `bridges`); nearest-point mode starts the tops there.
- [ ] **0.2** Record in the generated graph where each zigzag starts and ends.
- [ ] **0.3** Existing wireframe test: risers on the cube still vertical and
  aligned across layers; each zigzag's first node equals the previous
  zigzag's last node on the shared contour.
- [ ] **0.4** Regenerate all eight sources; structural screen passes.

### Phase 1

- [ ] **1.1** Priorities for the paired order where actions get their
  `priority` (`sections.py`), enabled with `--geometry-first`.
- [ ] **1.2** Count chainable joins (shared end and start node) on all eight.
- [ ] **1.3** Plan and verify all eight with the new order, unchained.

### Phase 2

- [ ] **2.1** `ForwardSchedule`: for a chainable join, the propose job of k+1
  reads the candidate of k (its key in the job key); unchained fallback after
  a failure or rejection.
- [ ] **2.2** `_propose`: fixed `q_start`, pen orientation and table position
  from the predecessor; closed contour started at the shared node.
- [ ] **2.3** Report chained and fallen-back joins per build.

### Phase 3

- [ ] **3.1** `complete_local_drawing(chained_in, chained_out)` and the corner
  hold at a riser-top join.
- [ ] **3.2** `_check` and `_connect`: empty transit for a chained stroke;
  joint positions at the join equal to 1e-9 rad.
- [ ] **3.3** `_time`: timing of clocks without approach or withdraw.

### Phase 4

- [ ] **4.1** `audit_process_clock`, `audit_indexed_clock`: chained feed state.
- [ ] **4.2** `cell_replay`: pairing check on the saved plan.
- [ ] **4.3** Test (folded into an existing one): a chained pair passes; a
  chained-out clock without its partner is rejected.

### Phase 5

- [ ] **5.1** Runs, README table, fast-planning document, this plan's status.

## Validation Commands

- `.venv/bin/pytest -q` (stay under 30 tests including the 4 browser tests)
- `.venv/bin/ruff check src tests`
- `tools/node/bin/node --test web/tests/*.test.mjs`
- `scripts/benchmark_parallel_build.py --source-run … --profile … --workers 24
  --build-option=--geometry-first --verification-option=--accept-geometry-proofs`
- `scratchpad/print_time.py <run>` for the print-time breakdown and sampled tip
  acceleration.
