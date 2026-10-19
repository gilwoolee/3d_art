# Pen Art

Plan a wire sculpture that a robot arm draws in the air with a 3D pen, from a
mesh to a plan checked against the whole robot. Everything is simulation.

![The viewer on the ghost](docs/images/viewer_ghost.png)

## What it does

A 3D pen pushes out plastic that hardens in a second, so a robot holding one
can draw a wire frame in mid-air, layer on layer, as in
[WirePrint](https://dl.acm.org/doi/10.1145/2642918.2647359). This repository
turns a mesh into a frame that stands, draws it in as few pen-down strokes as
possible, and finds arm motions that never hit the robot, the plate or the
wire already drawn.

## Setup

Linux x86-64 with git, git-lfs, curl and [uv](https://docs.astral.sh/uv/), on a
machine with many cores (planning uses 24 workers; pass `--workers` for fewer).
Unset `PYTHONPATH` first if your shell sets it.

```bash
git clone https://github.com/gilwoolee/3d_art.git && cd 3d_art
scripts/setup_machine.sh
```

Setup pulls the mesh, makes `.venv`, fetches the Flexiv robot description into
`external/`, builds the robot cell and builds the viewer.

## Steps

Each step is one command, run in order; results go in `runs/NAME/`. A step run
out of order, or after its inputs changed, stops and names the step to run.
Use `--help` on any command for its options.

**`pen-art cell`** builds the robot cell from `configs/cell.yaml`: a Flexiv
Rizon 4 arm, a 256 × 256 mm build plate on a 10 cm stand in front of it, the
pen, and the largest object size (15 cm). Run it again after editing the file.

**`pen-art frame objs/cute_ghost/cute_ghost.glb --name ghost`** slices the mesh
into horizontal loops joined by zigzags, with a crossed grid on the plate, and
scales the object so the frame needs at most 10 m of wire. It adds short
connector wire so the frame can be drawn in one stroke, and checks that it
stands.

**`pen-art detail ghost`** (optional) finds long strokes along the creases of
the mesh, drawn over the frame with one pen orientation each. It keeps up to
2 m and 15 added minutes of them, plus spare strokes for the next step.

**`pen-art plan-geometry ghost`** finds a collision-free arm path for every
stroke, in build order, without timing. It keeps the pen down from one stroke
to the next where it can, to keep lift-offs few. A detail stroke the arm cannot
draw is replaced by a spare.

**`pen-art plan-timing ghost`** times the paths once: drawing speed, cooling
holds, and the arm's speed and acceleration limits. It gives the print time
and never adds or removes strokes.

**`pen-art verify-full-robot ghost`** replays the saved plan with the whole
robot against the cell and all the wire built so far. Only a plan that passes
(`FULL_BUILD_VERIFIED`) counts.

**`pen-art view`** serves the viewer at http://127.0.0.1:8765: the object's
picture, the textured model, the frame (green) and detail (orange), and the
robot drawing it.

## Development

```bash
.venv/bin/pytest -q
.venv/bin/ruff check src tests scripts
(cd web && PATH=$PWD/../tools/node/bin:$PATH npm run build)
```

Settings are in `configs/`. The pen, wire and arm limits there are
assumptions, not measurements.

## License

[MIT](LICENSE), including the mesh and picture in `objs/`. The Flexiv robot
description cloned into `external/` keeps its own license.
