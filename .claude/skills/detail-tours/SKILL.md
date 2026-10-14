---
name: detail-tours
description: Add surface detail (folds, hems, face, crown, torch, book) to an already generated WirePrint frame as a few long, motion-plannable strokes drawn after the frame, then hand the result to the motion planner. Use when asked to add artistic detail on top of a wireframe, to regenerate the Liberty detail tours, or to keep added detail within a print-time budget.
---

# Detail tours on a WirePrint frame

The frame is generated as-is (`prepare_framework`) and planned as before. This
skill adds the detail: lines that make the object recognisable, laid on top of
the frame and drawn after it. The goal is detail the motion planner can plan;
the planner plans the frame and then the detail strokes.

## Rules the detail must follow

These come from the user and from the planner. Do not relax them silently.

- **Overlay, not clearance.** Detail lies on the frame like a coil turn on the
  turn below: 1 mm (one wire) outside the mesh surface. Touching points are
  fused (recorded in `fused_turns`); no struts. The planner already omits the
  pen tip from material collision while drawing; only the nose behind the tip
  (estimated tapered nose, 3 mm and up) keeps `clearance_m` (1 mm), and the
  nose must stay above the bed.
- **Drawn after the frame**, with the pen tilted. The pen body leans toward the
  robot; the turntable turns in 90 degree steps. One stroke keeps one turntable
  position, so a stroke stays on the face that looks at the robot. The tilt may
  change along a stroke (user, 2026-10-13), but the planner today holds one
  tilt per stroke (`drawing_orientation_policy: fixed`), so prefer strokes
  that keep one tilt.
- **Print time.** Added detail must cost at most 15 minutes. Strokes dominate
  (about 6 s each, measured on planned runs: pen start/stop waits, approach,
  withdraw, travel), drawing is about 20 mm/s on curvy paths, and a turntable
  step about 8 s. So detail must be a few long continuous strokes, not many
  short ones. Aim for at most ~80 strokes.
- **Wire.** The detail is extra wire on top of the frame's 10 m budget; the
  user asked for about 5 m.
- **Every stroke starts on a built node.** The planner's order search only
  starts a stroke at a node already built; fused contacts do not count. Each
  tour therefore begins exactly on a frame wire at a node it shares with the
  frame.
- **The structure check must pass** with fused contacts counted as
  connections (`src/pen_art/structure.py`).

## Steps

1. **Frame source.** Use a frame that already planned and verified, as
   generated. For the cut Liberty statue that is
   `runs/geomfirst_libertystatue_c1_source` (coiled raised arm; verified as
   `geomfirst_gf75_liberty_statue_w24`). The no-coil frame
   `geomfirst_libertynocoil_art12_source` does not plan: a 7 mm zigzag at the
   arm/head fork has no path (`NO_CONNECTED_IK_PATH`).
2. **Generate the detail** (about 2 minutes):

   ```bash
   PYTHONPATH=src .venv/bin/python scripts/detail_tours.py \
     runs/geomfirst_libertystatue_c1_source runs/geomfirst_libertytours_c1_source \
     --liberty --fine-region -11 2 -15 15 95 111
   ```

   It prints line, stroke and contact counts, the estimated added print time,
   the tilt mix and the structural screen. It writes the target as a source
   run (`stroke_graph.json`, `construction_plan.json`,
   `structural_screen.json`) plus `detail_tours.json` (per tour: length,
   turntable index, contacts, tilt per 1 mm sample, source lines).
   Knobs: `--wire` (line budget; about 0.55x survives), `--max-strokes`,
   `--crease-threshold` (mm; lower finds subtler creases), `--min-line`,
   `--bridge` (longest connecting run), `--fine-region` (mm box for a finer
   pass; Liberty face shown above), `--liberty` (crown rays, crown band, torch
   flame and balcony, book outline).
3. **Check it before planning.**
   - Estimated added time under 15 minutes; structure `pass True`.
   - Look at it: add the target to `runs/viewer_runs.json`, then in the viewer
     (tmux `pen-art-viewer`, port 8765) untick "Show statue" in panel 2.
     Screenshots from this machine need `env -u DISPLAY` (SSH X forwarding
     breaks headless WebGL).
4. **Plan.** Use a profile that allows tilts, for Liberty
   `runs/geomfirst_profiles/liberty_detail.yaml` (`liberty_statue.yaml` with
   `drawing_pen_tilt_candidates_deg: [0, 60, 45, 30, 15]`). In tmux:

   ```bash
   TAG=gfNN ./runs/geomfirst_one.sh liberty_tours \
     runs/geomfirst_libertytours_c1_source runs/geomfirst_profiles/liberty_detail.yaml
   ```

   Output: `runs/geomfirst_gfNN_liberty_tours_w24`. Progress line in
   `runs/geomfirst_gfNN_progress.txt`. If it stops with
   `PARALLEL_FORWARD_INCOMPLETE`, find the blocked stroke from
   `parallel_build.json` (`blocked`) and its failure counts; check whether it
   is a frame stroke (order position below the frame's stroke count) or a
   detail tour.

## How the generator works

1. Candidate lines: slice creases (slice along x, y, z every 0.75 mm; a crease
   is where the outline bulges out of or dents into the chord to neighbours
   1.5 mm away; chained slice to slice), a finer pass in the face box, and
   designed Liberty features. Chosen by importance (length x crease depth),
   skipping lines that mostly duplicate chosen ones.
2. Each line is pushed 1 mm outside the surface and cut where it turns to
   another face, or where no tilt (0-60) clears the frame and the bed.
3. Pieces on one face are joined end to end through connecting runs (up to
   22 mm, laid on the surface) when every point of the run has a clear tilt.
4. Tours are ordered by peeling from the end: the last tour must clear every
   other with some tilt at every point. When none can, the conflicting points
   of the tour that conflicts least are cut out.
5. Each tour starts on a shared frame node, overhangs at most 8 mm past its
   last contact, and is fused to the frame wherever it touches (every 4 mm at
   most). A tilt profile per tour changes as little as possible.

## Known gaps

- The planner stops at the first stroke it cannot plan; it cannot yet drop a
  detail tour and continue. The user wants it to plan most of the detail and
  drop the rest.
- Tilt that varies within a stroke is not supported by the planner.
- Connecting runs cross the surface between unrelated lines and read as
  scribbles. Routing them along frame wire (overlay) would hide them.
- The pose check models the nose only, not the barrel, arm or turntable.
- Physical assumptions are unmeasured: nose shape, fusing by overlay, 20 mm/s,
  unsupported overhangs up to 8 mm.
