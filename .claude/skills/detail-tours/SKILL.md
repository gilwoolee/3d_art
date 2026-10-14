---
name: detail-tours
description: Add about 2 m of surface detail (folds, hems, face, ribs, crown, torch, book) on top of a planned WirePrint frame as a few long strokes the robot draws after the frame, within 15 extra minutes of print time, and get the motion planner to plan it, dropping what it cannot draw. Use for any object in objs/ (Liberty, ghost, pumpkin, or a new mesh) when asked to add detail to a wireframe or rerun the detail harness.
---

# Detail tours on a WirePrint frame

The frame is generated and planned as before. Detail is extra wire laid on
top of it after it is built: a few long continuous strokes ("tours") that
follow the object's creases. An agent generates the tours to be plannable;
the motion planner then plans frame and tours, and the harness drops the
tours the planner (or the final full-robot check) rejects and trims the rest
to the budget.

## Budget and rules

- **Budget per object: about 2 m of detail and at most 15 minutes of added
  print time** (planned time, compared with the frame's own planned run; the
  user lowered the length from 5 m on 2026-10-14 after the timing below). The frame
  keeps its own 10 m wire budget; detail is on top of it.
- **Overlay.** Detail lies one wire (1 mm) outside the surface, resting on the
  frame wire like a coil turn on the turn below, and is bent onto frame wire
  that passes within 3.5 mm. Touching points are fused (`fused_turns`); there
  are no struts. The structural check counts fused points as connections.
- **The pen tip may touch existing wire.** The planner leaves the tip out of
  wire collision. The tapered nose right behind it keeps 0.3 mm
  (`environment.tool_clearance_m: {tapered_nose: 0.0003}`, used by drawing,
  approach and withdrawal checks); the barrel, arm and everything else keep
  `clearance_m` (1 mm).
- **Pose.** The pen body leans toward the robot, 0-60 degrees. The turntable
  turns in 90 degree steps, so one stroke stays on the face that looks at the
  robot. The planner holds one tilt per stroke; each tour carries its own
  tilt candidates (`pen_tilt_candidates_deg`), and such strokes skip the
  upright motion library.
- **Start on a built node.** Each tour starts exactly on a frame wire, at a
  node it shares with the frame; the planner only starts strokes there.
- **Print time is mostly strokes.** Measured per detail tour: about 20 s
  besides drawing (travel into a tilted pose, approach, withdrawal, waits),
  drawing about 10 mm/s, a 90 degree turntable step about 12 s. 5 m in 15
  minutes therefore needs about 20 tours averaging 250 mm; with the 40-60 mm
  tours the checks leave today, 15 minutes holds roughly 1.5-3 m.

## Steps

All commands from the repo root. Long runs in tmux (session restarts kill
background jobs); 24 workers.

1. **Frame.** Use a frame that already planned to `FULL_BUILD_VERIFIED`:

   | Object | Frame source | Profile | Frame run |
   |---|---|---|---|
   | Liberty (cut statue) | `runs/geomfirst_libertystatue_c1_source` | `runs/geomfirst_profiles/liberty_statue.yaml` | `runs/geomfirst_gf75_liberty_statue_w24` |
   | Ghost | `runs/geomfirst_ghost_b10_source` | `runs/geomfirst_profiles/ghost_b10.yaml` | `runs/geomfirst_gf71_ghost_w24` |
   | Pumpkin | `runs/geomfirst_pumpkin_b10_source` | `runs/geomfirst_profiles/pumpkin_b10.yaml` | `runs/geomfirst_gf71_pumpkin_w24` |

   For a new mesh:

   ```bash
   PYTHONPATH=src .venv/bin/python scripts/frame_source.py NAME objs/NAME/NAME.glb --up y
   TAG=gfNN ./runs/geomfirst_one.sh NAME runs/geomfirst_NAME_source runs/geomfirst_profiles/NAME.yaml
   ```

   The frame must reach `FULL_BUILD_VERIFIED` (see `runs/geomfirst_gfNN_progress.txt`).
   If it stops, change `--layer`, `--feature` or `--coil-width` before adding
   detail; detail never fixes a frame.

2. **Generate the tours** (5-10 minutes):

   ```bash
   PYTHONPATH=src .venv/bin/python scripts/detail_tours.py FRAME_SOURCE runs/geomfirst_NAME_detail_source \
     --profile PROFILE --check-with-planner --margin 0.0008 --detail 12 --minutes 40 --collect 2.0
   ```

   `--check-with-planner` builds a cell from the detail profile and asks the
   planner's own samplers, at each tour's turntable position, whether the
   whole arm can draw it: pen screen against the frame, drawing path (IK,
   self and fixed obstacles), the posture limit, an approach and a
   withdrawal. A failing tour is cut at the failing point (or trimmed at the
   end it cannot reach) and the pieces are checked again; each kept tour
   carries the tilts that worked. Generate well above the budget: the planner check and
   the planning rounds both remove tours, and the plan step trims to 2 m /
   15 min at the end. Read the "length by stage" line to see where length is
   lost (faces the pen cannot reach, no clear tilt, arm cannot draw it, not
   touching the frame). The pool of distinct crease lines is the real cap on
   a small or smooth object. For Liberty add `--liberty --fine-region -11 2 -15 15 95 111`
   (crown rays, crown band, torch, book; finer creases on the face). For
   another object a `--fine-region` box (mm) around a small detailed part
   (a face) helps.

3. **Plan, drop, trim** (30-90 minutes):

   ```bash
   PYTHONPATH=src .venv/bin/python -u scripts/plan_detail.py runs/geomfirst_NAME_detail_source PROFILE \
     runs/geomfirst_gfNN_NAME_detail --frame-run FRAME_RUN --detail 2 --minutes 15
   ```

   It writes a detail profile beside PROFILE (tilts and nose clearance;
   robot paths in profiles are relative to their folder) and plans round after round. A round drops the tours that
   planning blocked on, or that the full-robot replay rejected (planning
   checks the pen; replay checks the whole arm, e.g. self-collision at a
   tilted pose). Once everything plans, it trims the tours with the least
   detail per planned second until the detail is at most 2 m and the added
   time at most 15 minutes. A blocked frame stroke stops the loop.
   Detail tours carry their turntable position; the planner tries it first,
   so the drawing order keeps the face grouping.

4. **Check.** `runs/geomfirst_gfNN_NAME_detail_detail.json`: final run,
   `detail_m`, `added_minutes`, every dropped tour with its round and reason.
   The final run's status must be `FULL_BUILD_VERIFIED`. Add the final run to
   `runs/viewer_runs.json`; in the viewer (tmux `pen-art-viewer`, port 8765)
   untick "Show statue" to see wire only. Screenshots on this machine need
   `env -u DISPLAY` (SSH X forwarding breaks headless WebGL).

5. **Report** per object: detail metres and added minutes from the plan,
   tours kept and dropped (and why), the frame's own time, and a screenshot.

## How the generator works (`scripts/detail_tours.py`)

1. Candidate lines: slice creases along x, y and z at several depths (a
   crease is where a slice outline bulges out of or dents into the chord to
   its neighbours 1.5 mm away, chained slice to slice), a finer pass inside
   `--fine-region`, and designed Liberty features with `--liberty`.
2. In order of importance (length x crease depth), skipping near-duplicates:
   lay the line 1 mm outside the surface, bend it onto nearby frame wire, cut
   it where it turns to another face, and keep the stretches where some tilt
   clears the frame and the bed. Stop once 1.6x `--detail` is reachable.
3. Join stretches on one face end to end through connecting runs of up to
   22 mm on the surface.
4. Order the tours by peeling from the end: the last tour's nose must clear
   every other tour and its own wire more than 4 mm away. When nothing can go
   last, cut the conflicting points out of the tour that conflicts least.
5. Start each tour on a contact (ending uphill when either way works),
   overhang at most 8 mm, drop tours that never touch the frame. With
   `--check-with-planner`, check each tour with the planner and cut failing
   ones into drawable pieces. Keep the longest tours within `--detail` and
   `--minutes`, in an order that stays on one turntable face as long as it
   can.
6. Write the source run: frame members split at contacts, tour members,
   fused contacts, per-tour tilt candidates, `detail_tours.json`.

## Known gaps

- Tilted poses sit near arm singularities: approaches need fine sampling
  (down to 0.5 mm steps) and some tours fail the posture-change limit.
- A tour that ends going downhill can trap its own last wire under the nose
  when backing out.

- The drop loop replans everything each round (about 40-60 s); many failing
  tours make it slow.
- Planning checks only the pen; the whole arm is checked at the end, so arm
  self-collision at tilted poses costs a full round per tour.
- Connecting runs cross the surface and can read as scribbles; routing them
  along frame wire would hide them.
- Tilt that changes along a stroke is not supported by the planner.
- Undersides (surfaces facing down) are out of reach from above.
- Physical assumptions are unmeasured: nose shape, fusing by overlay, 20 mm/s,
  overhangs up to 8 mm, 0.3 mm nose clearance.
