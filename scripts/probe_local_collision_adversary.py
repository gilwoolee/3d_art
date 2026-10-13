"""Confirm actual pen overlap at each benchmark obstruction's interior pose."""

import argparse
from pathlib import Path

from pen_art.io import read_json, sha256, write_json
from pen_art.planning.clock_path import ClockPath
from pen_art.planning.replay_checkpoint import IMPLEMENTATION
from pen_art.planning.scene import MaterialScene
from pen_art.planning.section_build import SectionBuild


def probe(fixture_path):
    fixture = read_json(fixture_path)
    build = SectionBuild(Path(fixture["run"]), Path(fixture["profile"]), 120)
    arm = build.arm
    edges = [edge for edge in ClockPath.from_record(fixture["clock"]).edges if edge.kind == "DRAW"]
    edge = edges[len(edges) // 2]
    time_s = (edge.start_s + edge.end_s) / 2
    q = edge.position(time_s)
    body = arm.tool.bodies[-1]
    point = (arm.robot.frames(q)[body.link] @ body.origin)[:3, 3]
    scene = MaterialScene()
    scene.add(
        point - [0.004, 0, 0], point + [0.004, 0, 0], 0.003, "explicit-interior-pen-obstruction"
    )
    hits = arm.collision.check(q, {}, margin_m=0.001, material=scene)
    pairs = [pair for pair in hits if pair.get("second") == "explicit-interior-pen-obstruction"]
    if not pairs or not any(pair["distance_m"] <= 0 for pair in pairs):
        raise ValueError(f"Injected fixture does not prove interior pen overlap: {hits}")
    result = {"status": "COLLISION", "pairs": hits}
    return {
        "status": "WITNESSED_INTERIOR_PEN_COLLISION",
        "section": fixture["section"],
        "fixture_sha256": sha256(fixture_path),
        "implementation": IMPLEMENTATION,
        "time_s": time_s,
        "positions": q.tolist(),
        "obstruction_center_m": point.tolist(),
        "result": result,
        "scope": "Direct full-model query at the exact injected interior pose. "
        "Continuous validators separately reject before this pose at the clearance boundary.",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, required=True)
    args = parser.parse_args()
    rows = [probe(args.directory / f"{name}_fixture.json") for name in ("cube", "bottle")]
    write_json(
        args.directory / "adversary_overlap.json",
        {"status": "COMPLETE", "results": rows, "script_sha256": sha256(Path(__file__))},
    )


if __name__ == "__main__":
    main()
