"""Prepare one Flexiv arm with a provisional straight flange-centered pen."""

import argparse
from pathlib import Path

import yaml

from pen_art.cell import load_cell
from pen_art.robot import prepare_flexiv


def prepare(output, profile, repository):
    output = Path(output)
    config = yaml.safe_load(Path(profile).read_text())
    if len(config["arms"]) != 1:
        raise ValueError("Expected the single-arm reference cell")
    prepare_flexiv(repository, output, variant="Rizon4")
    arm = config["arms"][0]
    arm.update(
        robot_urdf="robot.urdf",
        robot_manifest="robot_manifest.json",
        robot_visual="robot_reference.glb",
    )
    target = output / "cell.yaml"
    target.write_text(yaml.safe_dump(config, sort_keys=False))
    _, arms, _ = load_cell(target)
    assert len(arms) == 1 and len(next(iter(arms.values())).robot.names) == 7
    print(f"Single-arm cell and collision meshes loaded: {target}")
    return target


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("research/planning_cell"))
    parser.add_argument(
        "--profile", type=Path, default=Path("configs/cell/one_flexiv_straight.yaml")
    )
    parser.add_argument("--repository", type=Path, default=Path("external/flexiv_description"))
    args = parser.parse_args()
    prepare(args.output, args.profile, args.repository)
