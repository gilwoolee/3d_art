"""pen-art: one command per step of the tutorial.

    pen-art cell                         the robot cell from configs/cell.yaml
    pen-art frame MESH --name NAME       a mesh into a frame, in runs/NAME/
    pen-art detail NAME                  detail strokes over the frame
    pen-art plan-geometry NAME           collision-free paths, untimed
    pen-art plan-timing NAME             speeds and holds: the print time
    pen-art verify-full-robot NAME       replay the whole robot
    pen-art view                         the browser viewer

Each command prints one summary line. Everything is simulation.
"""

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

import yaml

from .cell_config import CELL_CONFIG, CELL_DIR, load_frame_config, prepare_cell_model
from .io import read_json

RUNS = Path("runs")
PROFILE = CELL_DIR / "cell.yaml"


def _folder(name):
    folder = RUNS / name
    if not folder.is_dir():
        raise SystemExit(f"no object {name!r} in {RUNS}/: run `pen-art frame MESH --name {name}`")
    return folder


def _cell():
    if not PROFILE.exists():
        raise SystemExit(f"no robot cell in {CELL_DIR}/: run `pen-art cell`")
    return PROFILE


def cell(args):
    target = prepare_cell_model(args.cell, CELL_DIR)
    profile = yaml.safe_load(target.read_text())
    plate = profile["environment"]
    print(
        f"cell: {profile['robot_model']}, plate {plate['plate_size_m'][0] * 1000:g} x "
        f"{plate['plate_size_m'][1] * 1000:g} mm with its top {plate['plate_top_z_m'] * 100:g} cm "
        f"up, objects up to {profile['build_volume_m'] * 100:g} cm -> {target}"
    )


def frame(args):
    from .planning.section_build import prepare_framework, prepare_mesh_within_wire

    profile = _cell()
    folder = RUNS / args.name
    if folder.exists():
        shutil.rmtree(folder)  # A new frame invalidates everything planned from the old one.
    folder.mkdir(parents=True)
    config = load_frame_config()
    wireprint = config["wireprint"]
    if args.layer is not None:
        wireprint["layer_height"] = args.layer
    if args.feature is not None:
        wireprint["minimum_feature_m"] = args.feature
    if args.coil_width is not None:
        wireprint["coil_maximum_width"] = args.coil_width
    extent = yaml.safe_load(profile.read_text())["build_volume_m"]
    try:
        source = prepare_mesh_within_wire(args.mesh, folder / "frame", args.up, extent, config)
        prepare_framework(source, folder / "frame", profile)
    except ValueError as error:
        raise SystemExit(
            f"frame {args.name}: {error}. Is the mesh's up axis right (--up y or --up z)?"
        ) from None
    graph = read_json(source / "stroke_graph.json")
    wire = sum(edge["length_m"] for edge in graph["edges"])
    euler = graph["generation"].get("eulerization", {})
    size = read_json(source / "wire_budget.json")["max_extent_m"]
    stands = read_json(source / "structural_screen.json")["modeled_pass"]
    if wire < 1.0:
        print("  very little wire: is the mesh's up axis right (--up y or --up z)?")
    print(
        f"frame {args.name}: {wire:.1f} m of wire, {size * 1000:.0f} mm tall at most, "
        f"{len(graph['edges'])} wire pieces, stands: {'yes' if stands else 'NO'}, "
        f"fewest trails {euler.get('minimum_trails_after', '?')} "
        f"(was {euler.get('minimum_trails_before', '?')} before connector wire)"
    )


def detail(args):
    folder = _folder(args.name)
    for old in [folder / "detail", folder / "state.json", *folder.glob("round-*")]:
        if old.is_dir():
            shutil.rmtree(old)
        elif old.exists():
            old.unlink()
    command = [
        sys.executable, "-m", "pen_art.detail", str(folder / "frame_source"),
        str(folder / "detail"), "--profile", str(_cell()), "--check-with-planner",
    ]
    with open(folder / "detail.log", "w") as log:
        result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=False)
    if result.returncode:
        raise SystemExit(f"detail failed; see {folder / 'detail.log'}")
    tours = read_json(folder / "detail" / "detail_tours.json")["tours"]
    print(
        f"detail {args.name}: {sum(t['length_m'] for t in tours):.2f} m in {len(tours)} strokes "
        "(plan-geometry drops what the arm cannot draw; plan-timing trims to the budget)"
    )


def _lift_offs(state):
    lifts = state.get("lift_offs") or []
    minimum = state.get("minimum_trails")
    text = f"{len(lifts)} lift-offs"
    return text + (f" (fewest possible for the frame: {minimum})" if minimum else "")


def plan_geometry(args):
    from .stages import plan_geometry as stage

    state = stage(_folder(args.name), _cell(), args.workers, frame_only=args.frame_only)
    dropped = len(state.get("dropped", []))
    print(
        f"plan-geometry {args.name}: {state['geometry']}, {_lift_offs(state)}"
        + (f", dropped {dropped} detail strokes the arm cannot draw" if dropped else "")
    )


def plan_timing(args):
    from .stages import plan_timing as stage

    state = stage(_folder(args.name), args.workers, args.detail, args.minutes)
    line = f"plan-timing {args.name}: {state['timing']}"
    if state.get("print_minutes"):
        line += f", {state['print_minutes']:.1f} min of printing, {_lift_offs(state)}"
    if state.get("detail_m") is not None:
        line += f", detail {state['detail_m']:.2f} m adding {state['detail_minutes']:.1f} min"
    if state.get("trimmed"):
        line += f", trimmed {len(state['trimmed'])} detail strokes to fit"
    print(line)


def verify_full_robot(args):
    from .stages import verify_full_robot as stage

    state = stage(_folder(args.name), args.workers)
    print(f"verify-full-robot {args.name}: {state['verification']} (simulation only)")


def view(args):
    import uvicorn

    from .server import create_app

    print(f"viewer: http://127.0.0.1:{args.port}")
    uvicorn.run(create_app(Path.cwd()), host="127.0.0.1", port=args.port)


def main(argv=None):
    parser = argparse.ArgumentParser(prog="pen-art", description=__doc__.splitlines()[0])
    steps = parser.add_subparsers(dest="step", required=True)

    step = steps.add_parser("cell", help="build the robot cell from configs/cell.yaml")
    step.add_argument("--cell", type=Path, default=CELL_CONFIG)
    step.set_defaults(run=cell)

    step = steps.add_parser("frame", help="turn a mesh into a frame")
    step.add_argument("mesh", type=Path)
    step.add_argument("--name", required=True, help="the object's folder in runs/")
    step.add_argument("--up", choices=("y", "z"), default="y", help="the mesh axis that points up")
    step.add_argument("--layer", type=float, help="m between loops (frame.yaml otherwise)")
    step.add_argument("--feature", type=float, help="m; drop parts thinner than this")
    step.add_argument("--coil-width", type=float, help="m; wind parts this narrow as a coil")
    step.set_defaults(run=frame)

    step = steps.add_parser("detail", help="add detail strokes over the frame")
    step.add_argument("name")
    step.set_defaults(run=detail)

    for name, function, text in (
        ("plan-geometry", plan_geometry, "collision-free paths for every stroke, untimed"),
        ("plan-timing", plan_timing, "time the paths: speeds, holds, print time"),
        ("verify-full-robot", verify_full_robot, "replay the whole robot against the wire"),
    ):
        step = steps.add_parser(name, help=text)
        step.add_argument("name")
        step.add_argument("--workers", type=int, default=24)
        step.set_defaults(run=function)
        if name == "plan-geometry":
            step.add_argument("--frame-only", action="store_true", help="leave the detail out")
        if name == "plan-timing":
            step.add_argument("--detail", type=float, default=2.0, help="m of detail at most")
            step.add_argument("--minutes", type=float, default=15.0, help="added minutes at most")

    step = steps.add_parser("view", help="open the browser viewer")
    step.add_argument("--port", type=int, default=8765)
    step.set_defaults(run=view)

    args = parser.parse_args(argv)
    try:
        args.run(args)
    except Exception as error:
        from .stages import StageOrderError

        if isinstance(error, StageOrderError):
            raise SystemExit(str(error)) from None
        raise


if __name__ == "__main__":
    main()
