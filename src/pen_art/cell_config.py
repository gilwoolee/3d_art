"""The two configuration files, and the robot cell built from ``configs/cell.yaml``.

``configs/cell.yaml`` holds what a learner changes: the build volume, the
plate and the pen. Everything else about the cell (the robot's posture, the
collision model of the pen mount, parking) is fixed in ``cell_template.yaml``
next to this module. ``configs/frame.yaml`` holds the frame generator and the
process, timing and motion settings.
"""

from copy import deepcopy
from pathlib import Path

import yaml

TEMPLATE = Path(__file__).with_name("cell_template.yaml")
FRAME_CONFIG = Path("configs/frame.yaml")
CELL_CONFIG = Path("configs/cell.yaml")
CELL_DIR = Path("runs/cell")
# The pen mount between the flange and the pen body, and the gap from the end
# of the tapered nose to the tip; fixed parts of the modelled pen.
ADAPTER_M, TIP_GAP_M = 0.015, 0.003


def load_frame_config(path=FRAME_CONFIG):
    return yaml.safe_load(Path(path).read_text())


def expand_cell(cell, robot_dir="."):
    """The full cell profile for ``cell`` (the contents of ``configs/cell.yaml``).

    ``robot_dir`` is where the robot files are, relative to the profile.
    """
    profile = deepcopy(yaml.safe_load(TEMPLATE.read_text()))
    plate, pen = cell["plate"], cell["pen"]
    profile["build_volume_m"] = float(cell["build_volume_m"])
    profile["robot_model"] = cell["robot"]["model"]
    origin = (*plate["center_m"], plate["top_z_m"])
    for row, value in zip(profile["world_from_build"][:3], origin, strict=True):
        row[3] = float(value)
    profile["environment"]["plate_size_m"] = [float(v) for v in plate["size_m"]]
    profile["environment"]["plate_top_z_m"] = float(plate["top_z_m"])
    arm = profile["arms"][0]
    robot_dir = Path(robot_dir)
    arm.update(
        robot_urdf=str(robot_dir / "robot.urdf"),
        robot_manifest=str(robot_dir / "robot_manifest.json"),
        robot_visual=str(robot_dir / "robot_reference.glb"),
    )
    length, nose = float(pen["length_m"]), float(pen["nose_length_m"])
    body = length - ADAPTER_M - nose - TIP_GAP_M
    if body <= 0:
        raise ValueError("The pen is too short for its mount and nose")
    tool = arm["tool_geometry"]
    for transform in (arm["flange_to_tcp"], tool["flange_to_tcp"]):
        if isinstance(transform[0], list):
            transform[2][3] = length
        else:
            transform[11] = length
    tool["tip_radius_m"] = float(pen["tip_radius_m"])
    bodies = {b["name"]: b for b in tool["bodies"]}
    bodies["pen_body"]["shape"].update(radius_m=float(pen["body_radius_m"]), length_m=body)
    bodies["pen_body"]["origin"][2][3] = ADAPTER_M + body / 2
    bodies["tapered_nose"]["shape"].update(
        bottom_radius_m=float(pen["body_radius_m"]), length_m=nose
    )
    bodies["tapered_nose"]["origin"][2][3] = ADAPTER_M + body + nose / 2
    profile["setup_description"] = (
        f"One Flexiv {cell['robot']['model']} with a straight {length * 1000:g} mm pen; "
        f"a {plate['size_m'][0] * 1000:g} x {plate['size_m'][1] * 1000:g} mm plate with its "
        f"top {plate['top_z_m'] * 1000:g} mm above the table at "
        f"({plate['center_m'][0]:g}, {plate['center_m'][1]:g}) m. Pen, plate placement and "
        "process are unmeasured."
    )
    return profile


def prepare_cell_model(
    cell_path=CELL_CONFIG, output=CELL_DIR, repository="external/flexiv_description"
):
    """Build the robot model and write ``output/cell.yaml``, the profile planning uses."""
    from .cell import load_cell
    from .robot import prepare_flexiv

    cell = yaml.safe_load(Path(cell_path).read_text())
    output = Path(output)
    prepare_flexiv(repository, output, variant=cell["robot"]["model"])
    target = output / "cell.yaml"
    target.write_text(yaml.safe_dump(expand_cell(cell), sort_keys=False))
    _, arms, _ = load_cell(target)
    if len(arms) != 1 or len(next(iter(arms.values())).robot.names) != 7:
        raise ValueError("Expected one seven-joint arm")
    return target
