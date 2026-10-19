# Pen Art

Plans wire sculptures drawn in the air with a 3D pen held by a robot arm.

A mesh becomes a **frame**: a WirePrint-style wireframe of outline loops joined
by zigzags, at most 10 m of wire and 15 cm per side. A few long **detail**
strokes (about 2 m) are added over the surface. The planner finds the drawing
order and the arm motion for a Flexiv Rizon4 arm and a turntable that turns in
90 degree steps between strokes. A separate check then replays the whole robot
against the wire built so far; a plan that passes ends `FULL_BUILD_VERIFIED`.

Everything is simulation. Nothing has been printed, and the pen, wire and arm
placement are not measured.

![The viewer on the ghost: picture, textured model, frame (green) and detail
(orange), and the robot drawing it](docs/images/viewer_ghost.png)

## Results

All plans are `FULL_BUILD_VERIFIED`. Print time is the plan's modeled duration;
its waits and speeds are settings, not measurements.

| Object | Wire: frame + detail | Strokes | Print time |
| --- | ---: | ---: | ---: |
| Liberty, cut at the feet | 4.9 m + 1.99 m | 90 | 18.6 min |
| Ghost | 9.5 m + 1.98 m | 66 | 18.2 min |
| Pumpkin | 9.6 m + 1.99 m | 57 | 18.5 min |
| Palace | 7.0 m + 1.98 m | 172 | 24.6 min |
| Palace, thinner layers on the pillars (frame only) | 9.9 m | 177 | 23.1 min |

The meshes are in `objs/`. The planned runs are attached to the
[v1.0 release](https://github.com/gilwoolee/3d_art/releases/tag/v1.0).

## Setup

Linux x86-64 with git, git-lfs, curl and [uv](https://docs.astral.sh/uv/):

```bash
git clone https://github.com/gilwoolee/3d_art.git && cd 3d_art
scripts/setup_machine.sh
```

This pulls the meshes, makes `.venv`, checks out the Flexiv robot description
into `external/`, writes the robot cell, installs Node into `tools/node` and
builds the viewer.

Then download the release files and unpack them in the repository root:

```bash
base=https://github.com/gilwoolee/3d_art/releases/download/v1.0
curl -LO $base/pen_art_final_runs.tar.zst       # the planned results, to view
curl -LO $base/pen_art_motion_library.tar.zst   # needed to plan new frames
tar --zstd -xf pen_art_final_runs.tar.zst
tar --zstd -xf pen_art_motion_library.tar.zst
```

Frame strokes are adapted from the motion library. Without it they are planned
from scratch and print slower (ghost: 13.0 min instead of 10.1).

## Usage

**View the results.** Start the viewer and open <http://127.0.0.1:8765>:

```bash
.venv/bin/python -m pen_art.server --root "$PWD" --port 8765
```

It shows the textured model, the wire, and the robot's motion side by side
with one set of play controls. The run list comes from `runs/viewer_runs.json`;
any run also opens directly at `/?run=<run id>`.

**Plan a frame from a mesh.** Planning takes half a minute to a few minutes:

```bash
.venv/bin/python scripts/frame_source.py ghost objs/cute_ghost/cute_ghost.glb --up y
TAG=gf1 scripts/plan_frame.sh ghost runs/geomfirst_ghost_source runs/geomfirst_profiles/ghost.yaml
```

`frame_source.py` writes the frame and its cell profile. Its options are
`--layer` (m between loops), `--feature` (thinner parts are dropped) and
`--coil-width` (narrower parts are wound as a coil). `plan_frame.sh` uses 24
workers (set `WORKERS=` to change it) and writes `runs/geomfirst_gf1_ghost_w24`; the
last line of `runs/geomfirst_gf1_progress.txt` ends in `FULL_BUILD_VERIFIED`
when the plan passed. Run long jobs in tmux.

**Add detail.** Generate detail strokes over a frame, then plan them. Strokes
the planner cannot draw are dropped and the rest are trimmed to the budget:

```bash
PYTHONPATH=src .venv/bin/python scripts/detail_tours.py runs/geomfirst_ghost_source runs/geomfirst_ghost_detail_source \
  --profile runs/geomfirst_profiles/ghost.yaml --check-with-planner
PYTHONPATH=src .venv/bin/python scripts/plan_detail.py runs/geomfirst_ghost_detail_source \
  runs/geomfirst_profiles/ghost.yaml runs/geomfirst_gf2_ghost_detail \
  --frame-run runs/geomfirst_gf1_ghost_w24 --detail 2 --minutes 15
```

The final run is `runs/geomfirst_gf2_ghost_detail_rNN_w24`, after NN rounds of
dropping. On the ghost these two commands take about 12 minutes and give
1.33 m of detail and 6.6 more minutes of printing; the Palace takes over an
hour. The results above used tuned options, which the
[detail-tours guide](.claude/skills/detail-tours/SKILL.md) lists with the full
procedure.

## How the frame is built

- **Loops and zigzags.** Horizontal outline loops, one per layer, joined by
  zigzags: a vertical riser, then one slanted stroke back down, as in WirePrint.
- **Wire budget.** A mesh is shrunk until its frame fits in 10 m of wire.
- **No steep slants.** Slanted strokes stay within 27 degrees of horizontal.
  Where a loop is too short for that, its layers are made thinner.
- **Base.** Each outline on the bed is filled with a crossed grid of lines,
  drawn first.
- **Coil towers.** A part too narrow for loops, such as a raised arm, is wound
  as one climbing strand whose turns touch.
- **No loops in mid-air.** An outline with nothing beneath it is left out.
- **No twisting.** Each stroke keeps one pen tilt and one turntable face, and
  a stroke that starts where the last one ended is drawn without lifting the pen.

## Settings

Defaults are in `configs/fabrication.yaml` under `wireprint`. Each frame source
records its own in `fabrication_config.json`.

| Setting | Effect |
| --- | --- |
| `layer_height` | Height between loops. Taller layers use less wire and show less detail. |
| `minimum_feature_m` | Rebuilds the mesh without parts thinner than this. |
| `maximum_wire_m` | Wire budget; a mesh is shrunk to fit it. |
| `base_line_spacing` | Spacing of the grid on the bed. |
| `coil_maximum_width`, `coil_pitch` | When a narrow part becomes a coil, and how far it climbs per turn. |
| `heavier_strands_m` | Thicker strands tried in turn when the frame fails the structural check. |

## Limitations

These are assumptions, not measurements:
- the pen's shape and its 0.3 mm clearance from the wire;
- the drawing speed and cooling waits;
- PLA stiffness (a published 2.5 GPa);
- that detail laid over frame wire fuses to it, and overhangs of up to 8 mm.

## Development

```bash
.venv/bin/pytest -q
.venv/bin/ruff check src tests scripts
env -u DISPLAY tools/node/bin/node --test web/tests/*.test.mjs
(cd web && PATH=$PWD/../tools/node/bin:$PATH npm run build)
```

Code map and design notes: [docs/HANDOFF.md](docs/HANDOFF.md),
[fast planning and verification](docs/2026_1013_fast_planning_and_verification.md).

## License

[MIT](LICENSE), including the meshes and pictures in `objs/`. The third-party
projects that the setup script clones into `external/` (Flexiv description) are
not part of this repository and keep their own licenses.
