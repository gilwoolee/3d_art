"""Build a WirePrint frame source and its cell profile for a new mesh.

    .venv/bin/python scripts/frame_source.py NAME MESH [--up y] [--layer 0.0097]
        [--feature 0.008] [--coil-width M] [--template PROFILE]

The default template is the cell all current frames were planned in
(``configs/profiles/frame_template.yaml``); only the riser radius changes.

Writes ``runs/geomfirst_NAME_source`` (the frame within the 10 m wire budget,
longest side up to 150 mm) and ``runs/geomfirst_profiles/NAME.yaml``: the
template profile with the build riser sized to the frame's footprint. Plan
the frame with ``TAG=gfNN scripts/plan_frame.sh NAME SOURCE PROFILE`` and keep
the run as the detail harness's ``--frame-run``.
"""

import argparse
from pathlib import Path

import numpy as np
import yaml

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
    parser.add_argument(
        "--template", type=Path, default=Path("configs/profiles/frame_template.yaml")
    )
    args = parser.parse_args()

    config = yaml.safe_load(Path("configs/fabrication.yaml").read_text())
    config["wireprint"].update(layer_height=args.layer, minimum_feature_m=args.feature)
    if args.coil_width is not None:
        config["wireprint"]["coil_maximum_width"] = args.coil_width
    run = Path(f"runs/geomfirst_{args.name}")
    source = prepare_mesh_within_wire(args.mesh, run, args.up, 0.15, config)
    profile = yaml.safe_load(args.template.read_text())
    prepare_framework(source, run, args.template)

    graph = read_json(source / "stroke_graph.json")
    ground = np.array([n["position"] for n in graph["nodes"] if n["ground"]])
    radius = float(np.linalg.norm(ground[:, :2], axis=1).max()) + 0.003
    profile["environment"]["build_riser"]["radius_m"] = round(radius, 3)
    out = Path(f"runs/geomfirst_profiles/{args.name}.yaml")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(yaml.safe_dump(profile, sort_keys=False))
    screen = read_json(source / "structural_screen.json")
    wire = sum(e["length_m"] for e in graph["edges"])
    print(f"{source}: {wire:.2f} m frame, structure pass {screen['modeled_pass']}")
    print(f"{out}: build riser radius {radius * 1000:.0f} mm")


if __name__ == "__main__":
    main()
