# Pen Art

Plan a wire sculpture that a robot arm draws in the air with a 3D pen, from a
mesh to a plan checked against the whole robot. Everything is simulation.

![The viewer on the ghost](docs/images/viewer_ghost.png)

## Setup

Linux x86-64 with git, git-lfs, curl and [uv](https://docs.astral.sh/uv/).
Unset `PYTHONPATH` first if your shell sets it.

```bash
git clone https://github.com/gilwoolee/3d_art.git && cd 3d_art
scripts/setup_machine.sh
```

## Run

```bash
.venv/bin/pen-art cell                                    # robot cell from configs/cell.yaml
.venv/bin/pen-art frame objs/cute_ghost/cute_ghost.glb --name ghost
.venv/bin/pen-art detail ghost                            # optional
.venv/bin/pen-art plan-geometry ghost
.venv/bin/pen-art plan-timing ghost
.venv/bin/pen-art verify-full-robot ghost
.venv/bin/pen-art view                                    # http://127.0.0.1:8765
```

Results go in `runs/ghost/`. Settings are in `configs/`. Use `--help` on any
command for its options.

## Development

```bash
.venv/bin/pytest -q
.venv/bin/ruff check src tests scripts
(cd web && PATH=$PWD/../tools/node/bin:$PATH npm run build)
```

## License

[MIT](LICENSE), including the mesh and picture in `objs/`. The Flexiv robot
description cloned into `external/` keeps its own license.
