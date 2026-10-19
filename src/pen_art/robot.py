"""Expand pinned Flexiv descriptions and evaluate a URDF's serial kinematics."""

import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
import xacro
import yaml
from scipy.spatial.transform import Rotation

from .io import artifact, sha256, write_json

FLEXIV_REVISION = "92ef7865d76585e6e08d291bdfe652d32f7740f4"
FLEXIV_VARIANTS = {"Rizon4", "Rizon4s", "Rizon4M", "Rizon4R", "Rizon10", "Rizon10s"}


def origin_matrix(element):
    result = np.eye(4)
    if element is not None:
        result[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
        result[:3, :3] = Rotation.from_euler(
            "xyz", np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
        ).as_matrix()
    return result


class SerialArm:
    def __init__(self, urdf, tip="flange"):
        self.xml = ET.parse(urdf).getroot()
        by_child = {joint.find("child").get("link"): joint for joint in self.xml.findall("joint")}
        chain, link = [], tip
        while link in by_child:
            joint = by_child[link]
            chain.append(joint)
            link = joint.find("parent").get("link")
            if len(chain) > len(by_child):
                raise ValueError("URDF joint graph contains a cycle")
        self.root = link
        self.tip = tip
        self.chain = list(reversed(chain))
        self.active = [joint for joint in self.chain if joint.get("type") != "fixed"]
        if not self.chain or any(joint.get("type") != "revolute" for joint in self.active):
            raise ValueError("Expected a serial chain with bounded revolute joints")
        self.names = [joint.get("name") for joint in self.active]
        self.lower = np.array([float(j.find("limit").get("lower")) for j in self.active])
        self.upper = np.array([float(j.find("limit").get("upper")) for j in self.active])
        self.velocity = np.array([float(j.find("limit").get("velocity")) for j in self.active])
        self.joint_data = []
        index = 0
        for joint in self.chain:
            active = joint.get("type") != "fixed"
            axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ") if active else None
            if active:
                axis = axis / np.linalg.norm(axis)
                x, y, z = axis
                skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
            else:
                skew = None
            self.joint_data.append(
                {
                    "parent": joint.find("parent").get("link"),
                    "child": joint.find("child").get("link"),
                    "origin": origin_matrix(joint.find("origin")),
                    "axis": axis,
                    "skew": skew,
                    "index": index if active else None,
                }
            )
            index += int(active)

    def frames(self, positions):
        q = np.asarray(positions, dtype=float)
        if q.shape != (len(self.active),) or not np.isfinite(q).all():
            raise ValueError("Expected finite joint positions matching the arm DOF")
        if np.any(q < self.lower) or np.any(q > self.upper):
            raise ValueError("Joint positions exceed URDF limits")
        frames, current = {self.root: np.eye(4)}, np.eye(4)
        for joint in self.joint_data:
            current = current @ joint["origin"]
            if joint["index"] is not None:
                angle = q[joint["index"]]
                skew = joint["skew"]
                rotation = np.eye(4)
                rotation[:3, :3] += np.sin(angle) * skew + (1 - np.cos(angle)) * (skew @ skew)
                current = current @ rotation
            frames[joint["child"]] = current.copy()
        return frames

    def forward(self, positions, flange_to_tcp=None):
        flange = self.frames(positions)[self.tip]
        return flange if flange_to_tcp is None else flange @ np.asarray(flange_to_tcp)


def prepare_flexiv(repository, output, variant="Rizon4"):
    """Rizon4 is a simulation reference until the physical variant is confirmed."""
    repository, output = Path(repository).resolve(), Path(output).resolve()
    if variant not in FLEXIV_VARIANTS:
        raise ValueError(f"Unsupported Flexiv variant: {variant}")
    revision = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=repository, text=True
    ).strip()
    if revision != FLEXIV_REVISION:
        raise ValueError("Flexiv description revision differs from the pinned version")
    output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as temporary:
        package = Path(temporary)
        shutil.copytree(repository / "config", package / "config")
        for source in (repository / "urdf").rglob("*.xacro"):
            destination = package / source.relative_to(repository)
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_text(
                source.read_text().replace("$(find flexiv_description)", str(package))
            )
        doc = xacro.process_file(
            str(package / "urdf/rizon.urdf.xacro"), mappings={"rizon_type": variant}
        )
        urdf = (
            doc.toprettyxml(indent="  ")
            .replace("package://flexiv_description", str(repository))
            .replace(str(package), str(repository))
        )
    urdf_path = output / "robot.urdf"
    urdf_path.write_text(urdf)
    robot = SerialArm(urdf_path)
    if len(robot.names) != 7:
        raise ValueError("The selected robot must have seven active joints")
    initial = yaml.safe_load((repository / "config/initial_positions.yaml").read_text())[
        "initial_positions"
    ]
    q = [initial[name] for name in robot.names]
    frames = robot.frames(q)
    scene = trimesh.Scene()
    visual_nodes = []
    for link in robot.xml.findall("link"):
        name = link.get("name")
        if name not in frames:
            continue
        for index, visual in enumerate(link.findall("visual")):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            geometry = trimesh.load_mesh(mesh.get("filename"))
            geometry.apply_scale(np.fromstring(mesh.get("scale", "1 1 1"), sep=" "))
            visual_nodes.append(
                {
                    "name": f"{name}-{index}",
                    "link": name,
                    "origin": origin_matrix(visual.find("origin")).tolist(),
                }
            )
            scene.add_geometry(
                geometry,
                node_name=f"{name}-{index}",
                transform=frames[name] @ origin_matrix(visual.find("origin")),
            )
    scene.export(output / "robot_reference.glb")
    manifest = artifact(
        "robot_reference",
        manufacturer="Flexiv",
        variant=variant,
        simulation_only=True,
        variant_confirmed=False,
        units="m",
        angle_units="rad",
        frame="robot_base",
        revision=revision,
        urdf_sha256=sha256(urdf_path),
        kinematic_root=robot.root,
        kinematic_tip=robot.tip,
        kinematic_chain=[
            {
                "parent": joint.find("parent").get("link"),
                "child": joint.find("child").get("link"),
                "origin": origin_matrix(joint.find("origin")).tolist(),
                "axis": np.fromstring(joint.find("axis").get("xyz"), sep=" ").tolist()
                if joint.get("type") != "fixed"
                else None,
                "index": robot.names.index(joint.get("name"))
                if joint.get("type") != "fixed"
                else None,
            }
            for joint in robot.chain
        ],
        visual_nodes=visual_nodes,
        joint_names=robot.names,
        lower=robot.lower.tolist(),
        upper=robot.upper.tolist(),
        velocity=robot.velocity.tolist(),
        initial_positions=q,
        flange_pose=robot.forward(q).tolist(),
        missing=[
            "Confirm physical arm variant",
            "Measured flange-to-pen-tip transform",
            "Pen and mount collision geometry",
            "Pen control and calibration",
        ],
    )
    write_json(output / "robot_manifest.json", manifest)
    return manifest
