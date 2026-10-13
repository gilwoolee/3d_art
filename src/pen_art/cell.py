"""Explicit shared-world model for independently configured seven-axis arms."""

import argparse
import re
import shutil
from collections import Counter
from dataclasses import dataclass
from os.path import relpath
from pathlib import Path

import numpy as np
import yaml

from .io import artifact, now, read_json, sha256, stage_update, write_json
from .kinematics import rigid_transform
from .planning.collision import RobotCollisionModel, work_surfaces
from .planning.coordinated_motion import check_pair_linear
from .planning.fixture import riser_object
from .planning.tool import load_tool
from .robot import SerialArm


class WorldArm:
    """Reuse serial kinematics with a rigid base placement in the shared world."""

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
    if not isinstance(specs, list) or len(specs) not in (1, 2):
        raise ValueError("Cell requires one or two arms")
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
        tool = load_tool(spec.get("tool_geometry"), path.parent, robot, offset)
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
    # The current table/platform model assumes Z-up horizontal work surfaces.
    if not np.allclose(world_from_build[:3, 2], [0, 0, 1], atol=1e-9, rtol=0):
        raise ValueError("Current cell requires a horizontal build platform")
    environment = profile["environment"]
    values = [
        environment["table_z_m"],
        *environment["table_size_m"],
        *environment["platform_size_m"],
        environment["clearance_m"],
    ]
    if not np.isfinite(values).all() or environment["clearance_m"] < 0:
        raise ValueError("Invalid cell environment dimensions or clearance")
    plate_top = float(environment.get("platform_top_z_m", world_from_build[2, 3]))
    obstacles = work_surfaces(
        world_from_build[:2, 3],
        plate_top - environment["table_z_m"],
        *environment["platform_size_m"],
        environment["table_z_m"],
        environment["table_size_m"],
    )
    if "turntable" in profile:
        from .planning.turntable import IndexedTurntable

        turntable = IndexedTurntable.from_profile(profile)
        obstacles.pop("platform")
        obstacles.update(turntable.obstacles())
    elif "build_riser" in environment:
        riser = environment["build_riser"]
        if not np.isclose(world_from_build[2, 3] - plate_top, riser["height_m"]):
            raise ValueError("Riser must bridge plate top and build surface")
        obstacles["build_riser"] = riser_object(riser, world_from_build)[0]
    return profile, arms, obstacles


def prepare_object_profile(source, output, *, footprint, width_m):
    """Write a fresh, source-bound profile with a centered object-footprint riser."""
    source, output = Path(source).resolve(), Path(output).resolve()
    if output.exists():
        raise ValueError("Use a fresh object profile path")
    profile = yaml.safe_load(source.read_text())
    for arm in profile["arms"]:
        for key in ("robot_urdf", "robot_manifest", "robot_visual"):
            arm[key] = relpath((source.parent / arm[key]).resolve(), output.parent)
        for body in (arm.get("tool_geometry") or {}).get("bodies", []):
            shape = body["shape"]
            if shape["type"] == "mesh":
                shape["path"] = relpath((source.parent / shape["path"]).resolve(), output.parent)
    riser = {"type": footprint, "height_m": 0.020, "calibrated": False}
    if footprint == "box":
        riser["size_m"] = [width_m, width_m]
    elif footprint == "cylinder":
        riser["radius_m"] = width_m / 2
    else:
        raise ValueError("Object footprint must be box or cylinder")
    riser_object(riser, profile["world_from_build"])
    profile["environment"]["build_riser"] = riser
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(profile, sort_keys=False))
    return profile


def sync_source_visuals(output, source_run, include_graph=True):
    """Carry upstream visual results into a cell run without replacing inputs.

    Preserve the source stage's status and provenance. Robot motion and timing
    are deliberately excluded: single-arm results are not two-arm validation.
    """
    output, source_run = Path(output), Path(source_run)
    pending = [
        "geometric_candidates.json",
        "surface_contours.json",
        "selected_candidates.json",
        "abstraction_review.json",
        "candidate_comparison.json",
        "stroke_graph.json",
        "pen_strokes.json",
        "construction_plan.json",
        "structural_screen.json",
        "process_screen.json",
        "fabrication_config.json",
        "style_selection.json",
        "style_attachments.json",
        "framework_graph.json",
    ]
    graph_files = {
        "stroke_graph.json",
        "pen_strokes.json",
        "construction_plan.json",
        "structural_screen.json",
        "process_screen.json",
    }
    if not include_graph:
        pending = [name for name in pending if name not in graph_files]
    pending.extend(
        path.name for path in source_run.glob("wir3d_*") if path.suffix in {".json", ".png"}
    )
    copied = set()

    def references(value):
        if isinstance(value, dict):
            for key, item in value.items():
                if key == "inputs" and isinstance(item, dict):
                    yield from item
                yield from references(item)
        elif isinstance(value, list):
            for item in value:
                yield from references(item)
        elif isinstance(value, str) and Path(value).name == value:
            if Path(value).suffix in {".json", ".png"}:
                yield value

    while pending:
        name = pending.pop()
        if name in copied or Path(name).name != name:
            continue
        source = source_run / name
        if not source.is_file():
            continue
        destination = output / name
        if destination.exists() and sha256(destination) != sha256(source):
            raise ValueError(f"Cell input differs from source; refusing to replace {name}")
        if not destination.exists():
            shutil.copy2(source, destination)
        copied.add(name)
        if source.suffix == ".json":
            pending.extend(references(read_json(source)))
    source = read_json(source_run / "run.json")
    for key in ("geometric", "wir3d", "selection", "comparison", "graph", "construction"):
        stage = source["stages"].get(key)
        frozen_wir3d = key == "wir3d" and any(
            name.startswith("wir3d_") and name.endswith(".json") for name in copied
        )
        if stage and (stage.get("artifact") in copied or frozen_wir3d):
            values = {k: v for k, v in stage.items() if k not in {"status", "updated_at"}}
            values["source_run"] = source["id"]
            stage_update(output, key, stage["status"], **values)
    return sorted(copied)


def snapshot_backbone(output, source_run):
    """Export one internally consistent supported graph/order for robot planning."""
    mapping = {
        "stroke_graph.json": "backbone_graph.json",
        "pen_strokes.json": "backbone_pen_strokes.json",
        "construction_plan.json": "backbone_construction.json",
        "structural_screen.json": "backbone_structure.json",
        "process_screen.json": "backbone_process.json",
    }
    source_hashes = {name: sha256(source_run / name) for name in mapping.values()}
    graph = read_json(source_run / mapping["stroke_graph.json"])
    construction = read_json(source_run / mapping["construction_plan.json"])
    members = Counter(member for action in construction["actions"] for member in action["members"])
    if members != Counter(edge["id"] for edge in graph["edges"]):
        raise ValueError("Supported construction must cover its exact graph members once")
    for target, source in mapping.items():
        document = read_json(source_run / source)
        if target == "construction_plan.json":
            # The canonical graph has identical bytes; only rename this local input binding.
            document["inputs"] = {"stroke_graph.json": sha256(output / "stroke_graph.json")}
        destination = output / target
        if destination.exists() and read_json(destination) != document:
            raise ValueError(f"Cell supported input changed: {target}; use a new run")
        write_json(destination, document)
    write_json(
        output / "cell_source_bundle.json",
        artifact(
            "cell_source_bundle",
            variant="backbone",
            source_run=str(source_run.resolve()),
            source_sha256=source_hashes,
            inputs={name: sha256(output / name) for name in mapping},
        ),
    )
    stage_update(
        output,
        "graph",
        "complete",
        artifact="stroke_graph.json",
        source="selected",
        message="Supported graph with intact source contours",
    )
    stage_update(
        output,
        "construction",
        "partial",
        artifact="construction_plan.json",
        message="Supported whole-contour order; physical validation pending",
    )


def prepare_cell(profile_path, output, source_run, source_variant="original"):
    """Write a separate setup preview; no paths or concurrency are asserted."""
    profile_path, output, source_run = Path(profile_path), Path(output), Path(source_run)
    if source_variant not in {"original", "backbone"}:
        raise ValueError("Source variant must be original or backbone")
    if output.resolve() == source_run.resolve():
        raise ValueError("Create a separate run for the two-arm experiment")
    profile, arms, obstacles = load_cell(profile_path)
    output.mkdir(parents=True, exist_ok=True)
    source = read_json(source_run / "run.json")
    if (output / "run.json").exists():
        previous = read_json(output / "run.json")
        if previous.get("cell_source_run") != str(source_run.resolve()):
            raise ValueError("Output belongs to a different experiment; choose a new run directory")
        if previous.get("source_sha256") != source["source_sha256"]:
            raise ValueError("Source asset changed; choose a new run directory")
        if previous.get("cell_source_variant", "original") != source_variant:
            raise ValueError("Source variant changed; choose a new run directory")
    if not (output / "run.json").exists():
        write_json(
            output / "run.json",
            {
                "id": output.name,
                "name": f"{source['name']} · {'one-arm' if len(arms) == 1 else 'two-arm'}",
                "created_at": now(),
                "updated_at": now(),
                "source_sha256": source["source_sha256"],
                "cell_source_run": str(source_run.resolve()),
                "cell_source_variant": source_variant,
                "stages": {
                    name: {"status": "pending"}
                    for name in (
                        "asset",
                        "geometric",
                        "wir3d",
                        "graph",
                        "construction",
                        "motion",
                        "timing",
                        "validation",
                        "cell",
                    )
                },
            },
        )
    files = ("asset_manifest.json", "normalized.glb", "geometry.glb")
    for name in files:
        shutil.copy2(source_run / name, output / name)
    stage_update(output, "asset", "complete", artifact="asset_manifest.json")
    sync_source_visuals(output, source_run, include_graph=source_variant == "original")
    if source_variant == "backbone":
        snapshot_backbone(output, source_run)
    write_json(output / "cell_config.json", profile)
    inputs = {name: sha256(output / name) for name in (*files, "cell_config.json")}
    if source_variant == "backbone":
        inputs["cell_source_bundle.json"] = sha256(output / "cell_source_bundle.json")
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
    pair_check = None
    if len(arms) == 2:
        a, b = arms.values()
        pair_check = check_pair_linear(
            a.collision,
            b.collision,
            a.initial_positions,
            a.initial_positions,
            b.initial_positions,
            b.initial_positions,
            margin_m=profile["environment"]["clearance_m"],
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
        **({"turntable": profile["turntable"]} if "turntable" in profile else {}),
        placement_confirmed=profile.get("placement_confirmed", False),
        initial_inter_arm_check=pair_check,
        limitations=[
            "Base placement, robot variants and pen offsets are provisional",
            "This is a setup view, not a coordinated trajectory or fabrication schedule",
            "Pen/mount geometry is absent unless explicitly supplied",
        ],
    )
    write_json(output / "cell_manifest.json", result)
    stage_update(
        output,
        "cell",
        "partial",
        artifact="cell_manifest.json",
        message=f"{len(arms)}-arm world setup; motion planning pending",
    )
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("configs/cell/two_flexiv.yaml"))
    parser.add_argument("--output", type=Path, default=Path("runs/palace_15cm_dual"))
    parser.add_argument("--source-run", type=Path, default=Path("runs/palace_15cm_selected"))
    parser.add_argument("--source-variant", choices=["original", "backbone"], default="original")
    args = parser.parse_args()
    prepare_cell(args.config, args.output, args.source_run, args.source_variant)
