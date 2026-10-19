"""The two configuration files, and the robot cell built from ``configs/cell.yaml``.

``configs/cell.yaml`` holds what a learner changes: the build volume, the
plate and the pen. Everything else about the cell (the robot's posture and the
collision model of the pen mount) is fixed in ``cell_template.yaml`` next to
this module. ``configs/frame.yaml`` holds the frame generator and the process,
timing and motion settings. A planning round gets its own copy of the cell
(``prepare_cell``), which the planner checks has not changed.
"""

import re
import shutil
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from .io import artifact, now, read_json, sha256, write_json
from .kinematics import rigid_transform
from .planning.collision import RobotCollisionModel, work_surfaces
from .planning.tool import load_tool
from .robot import SerialArm

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
    return profile


def prepare_cell_model(
    cell_path=CELL_CONFIG, output=CELL_DIR, repository="external/flexiv_description"
):
    """Build the robot model and write ``output/cell.yaml``, the profile planning uses."""
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


class WorldArm:
    """Reuse serial kinematics with a rigid base placement in the world."""

    def __init__(self, robot, world_from_base):
        self.local = robot
        self.world_from_base = rigid_transform(world_from_base, "world-from-base")

    def __getattr__(self, name):
        return getattr(self.local, name)

    def frames(self, positions):
        return {
            name: self.world_from_base @ frame
            for name, frame in self.local.frames(positions).items()
        }

    def forward(self, positions, flange_to_tcp=None):
        frame = self.frames(positions)[self.tip]
        return frame if flange_to_tcp is None else frame @ np.asarray(flange_to_tcp)


@dataclass
class ArmInstance:
    identity: str
    robot: WorldArm
    collision: RobotCollisionModel
    flange_to_tcp: np.ndarray
    initial_positions: np.ndarray
    tool: object
    urdf: Path
    manifest_path: Path
    visual_path: Path


def load_cell(path):
    path = Path(path).resolve()
    profile = yaml.safe_load(path.read_text())
    if profile.get("schema_version") != 1 or profile.get("units") != "m":
        raise ValueError("Cell requires schema_version 1 and meter units")
    if profile.get("simulation_only") is not True:
        raise ValueError("Cell planning currently supports simulation only")
    tilt = profile.get("drawing_pen_tilt_deg", 0.0)
    if (
        isinstance(tilt, bool)
        or not isinstance(tilt, (int, float))
        or not np.isfinite(tilt)
        or not 0 <= tilt < 90
    ):
        raise ValueError("Drawing pen tilt must be finite and in [0, 90) degrees")
    specs = profile.get("arms")
    if not isinstance(specs, list) or len(specs) != 1:
        raise ValueError("Cell requires one arm")
    arms = {}
    for spec in specs:
        identity = spec.get("id")
        if not isinstance(identity, str) or not re.fullmatch(r"[a-zA-Z0-9_-]+", identity):
            raise ValueError("Arm IDs must be nonempty artifact-safe names")
        if identity in arms:
            raise ValueError("Duplicate arm ID")
        urdf = (path.parent / spec["robot_urdf"]).resolve()
        manifest_path = (path.parent / spec["robot_manifest"]).resolve()
        visual_path = (path.parent / spec["robot_visual"]).resolve()
        manifest = read_json(manifest_path)
        if manifest["urdf_sha256"] != sha256(urdf):
            raise ValueError(f"Robot manifest is stale: {identity}")
        robot = WorldArm(SerialArm(urdf), spec["world_from_base"])
        if len(robot.names) != 7 or robot.names != manifest["joint_names"]:
            raise ValueError("Cell arm requires seven matching joints")
        offset = rigid_transform(spec["flange_to_tcp"], "flange-to-TCP")
        initial = np.asarray(spec.get("initial_positions", manifest["initial_positions"]), float)
        robot.frames(initial)
        tool = load_tool(spec["tool_geometry"], robot, offset)
        arms[identity] = ArmInstance(
            identity,
            robot,
            RobotCollisionModel(robot, tool.bodies),
            offset,
            initial,
            tool,
            urdf,
            manifest_path,
            visual_path,
        )
    world_from_build = rigid_transform(profile["world_from_build"], "world-from-build")
    # The object stands on a build plate on a stand on the table. The plate
    # and its stand are one solid block from the table up to the plate top.
    if not np.allclose(world_from_build[:3, 2], [0, 0, 1], atol=1e-9, rtol=0):
        raise ValueError("The build plate must be horizontal")
    environment = profile["environment"]
    values = [
        environment["table_z_m"],
        *environment["table_size_m"],
        *environment["plate_size_m"],
        environment["plate_top_z_m"],
        environment["clearance_m"],
    ]
    if not np.isfinite(values).all() or environment["clearance_m"] < 0:
        raise ValueError("Invalid cell environment dimensions or clearance")
    if not np.isclose(world_from_build[2, 3], environment["plate_top_z_m"]):
        raise ValueError("The build origin must lie on the top of the plate")
    if environment["plate_top_z_m"] <= environment["table_z_m"]:
        raise ValueError("The plate top must be above the table")
    obstacles = work_surfaces(
        world_from_build[:2, 3],
        environment["plate_top_z_m"] - environment["table_z_m"],
        *environment["plate_size_m"],
        environment["table_z_m"],
        environment["table_size_m"],
    )
    return profile, arms, obstacles


# What a planning round reads from the frame or detail it plans.
SOURCE_FILES = (
    "asset_manifest.json", "normalized.glb", "geometry.glb",
    "stroke_graph.json", "construction_plan.json", "fabrication_config.json",
)


def prepare_cell(profile_path, output, source_run):
    """A planning round in ``output``: the strokes of ``source_run`` and a copy of the cell."""
    profile_path, output, source_run = Path(profile_path), Path(output), Path(source_run)
    profile, arms, obstacles = load_cell(profile_path)
    output.mkdir(parents=True, exist_ok=True)
    write_json(
        output / "run.json",
        {
            "id": output.name,
            "name": output.name,
            "created_at": now(),
            "updated_at": now(),
            "cell_source_run": str(source_run.resolve()),
            "stages": {},
        },
    )
    for name in SOURCE_FILES:
        shutil.copy2(source_run / name, output / name)
    write_json(output / "cell_config.json", profile)
    files = ("asset_manifest.json", "normalized.glb", "geometry.glb", "cell_config.json")
    inputs = {name: sha256(output / name) for name in files}
    arm_rows = []
    for identity, arm in arms.items():
        resources = {}
        for suffix, origin in (
            ("robot.urdf", arm.urdf),
            ("robot.json", arm.manifest_path),
            ("robot.glb", arm.visual_path),
        ):
            filename = f"{identity}_{suffix}"
            shutil.copy2(origin, output / filename)
            inputs[filename] = sha256(output / filename)
            resources[suffix] = filename
        arm_rows.append(
            {
                "id": identity,
                "world_from_base": arm.robot.world_from_base.tolist(),
                "flange_to_tcp": arm.flange_to_tcp.tolist(),
                "initial_positions": arm.initial_positions.tolist(),
                "initial_tcp_world": arm.robot.forward(
                    arm.initial_positions, arm.flange_to_tcp
                ).tolist(),
                "robot_manifest": resources["robot.json"],
                "robot_visual": resources["robot.glb"],
                "robot_urdf": resources["robot.urdf"],
                "tool_model": arm.tool.metadata,
                "initial_static_contacts": arm.collision.check(
                    arm.initial_positions, obstacles, profile["environment"]["clearance_m"]
                ),
            }
        )
    result = artifact(
        "robot_cell_preview",
        status="CELL_SETUP_PREVIEW",
        simulation_only=True,
        executable=False,
        frame="world",
        units="m",
        arms=arm_rows,
        inputs=inputs,
        world_from_build=profile["world_from_build"],
        environment=profile["environment"],
        placement_confirmed=profile.get("placement_confirmed", False),
        limitations=[
            "Base placement, robot variants and pen offsets are provisional",
            "This is a setup view, not a coordinated trajectory or fabrication schedule",
        ],
    )
    write_json(output / "cell_manifest.json", result)
    # The planner checks that none of these changed since.
    names = ("cell_manifest.json", "construction_plan.json", "stroke_graph.json")
    write_json(
        output / "cell_drawing_variants.json",
        artifact(
            "cell_drawing_variants",
            inputs={name: sha256(output / name) for name in names},
            variants=[],
            status="LAZY_SECTION_CANDIDATES",
        ),
    )
    return result
