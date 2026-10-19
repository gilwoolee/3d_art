"""Build a WirePrint frame source for a mesh, in the cell from ``pen-art cell``.

    .venv/bin/python scripts/frame_source.py NAME MESH [--up y] [--layer M]
        [--feature M] [--coil-width M] [--profile runs/cell/cell.yaml]

Writes ``runs/geomfirst_NAME_source``: the frame within the wire budget, the
mesh scaled so its longest side is the cell's build volume. Plan it with
``TAG=gfNN scripts/plan_frame.sh NAME SOURCE runs/cell/cell.yaml``.
"""

import argparse
from pathlib import Path

import yaml

from pen_art.cell_config import CELL_DIR, load_frame_config
from pen_art.io import read_json
from pen_art.planning.section_build import prepare_framework, prepare_mesh_within_wire


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("name")
    parser.add_argument("mesh", type=Path)
    parser.add_argument("--up", default="y", help="mesh axis that points up")
    parser.add_argument("--layer", type=float, default=0.0097, help="m between outline loops")
    parser.add_argument("--feature", type=float, default=0.008, help="m; thinner parts dropped")
    parser.add_argument("--coil-width", type=float, help="m; parts this narrow are wound")
    parser.add_argument("--profile", type=Path, default=CELL_DIR / "cell.yaml")
    args = parser.parse_args()

    config = load_frame_config()
    config["wireprint"].update(layer_height=args.layer, minimum_feature_m=args.feature)
    if args.coil_width is not None:
        config["wireprint"]["coil_maximum_width"] = args.coil_width
    extent = yaml.safe_load(args.profile.read_text())["build_volume_m"]
    run = Path(f"runs/geomfirst_{args.name}")
    source = prepare_mesh_within_wire(args.mesh, run, args.up, extent, config)
    prepare_framework(source, run, args.profile)

    graph = read_json(source / "stroke_graph.json")
    screen = read_json(source / "structural_screen.json")
    wire = sum(e["length_m"] for e in graph["edges"])
    print(f"{source}: {wire:.2f} m frame, structure pass {screen['modeled_pass']}")


if __name__ == "__main__":
    main()
