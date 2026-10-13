"""Compare complete-tree and spatial-cell FCL queries on a saved Palace path."""

import argparse
from pathlib import Path
from time import monotonic

from pen_art.io import read_json, sha256, write_json
from pen_art.planning.collision import placed
from pen_art.planning.section_build import SectionBuild


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, default=Path("runs/palace_hybrid_backward_diverse_v1"))
    parser.add_argument("--section", default="section-01210")
    parser.add_argument(
        "--output", type=Path, default=Path("research/blondie/material_queries.json")
    )
    args = parser.parse_args()
    profile = Path("research/planning_cell/cell.yaml")
    build = SectionBuild(args.run, profile, 300)
    scene = build.scene(set(build.actions) - {args.section})
    repair_path = args.run / "axis_repairs" / f"arm_1_{args.section}.json"
    repair_bytes = repair_path.read_bytes()
    repair = read_json(repair_path)
    objects = []
    for q in repair["positions"][::3]:
        frames = build.arm.robot.frames(q)
        objects.extend(
            placed(body.geometry, frames[body.link] @ body.origin) for body in build.arm.tool.bodies
        )
    scene.manager.update()
    started = monotonic()
    expected = [scene._nearest_in_manager(obj, scene.manager) for obj in objects]
    legacy_seconds = monotonic() - started
    measured = {}
    for label in ("cold", "warm"):
        started = monotonic()
        actual = [scene.nearest(obj) for obj in objects]
        seconds = monotonic() - started
        errors = []
        for first, second in zip(expected, actual, strict=True):
            a, b = first["distance_m"], second["distance_m"]
            assert (a <= 0) == (b <= 0), (first, second)
            errors.append(abs(a - b))
        assert max(errors) <= 1e-7, max(errors)
        measured[label] = {"seconds": seconds, "maximum_distance_difference_m": max(errors)}
    assert repair_path.read_bytes() == repair_bytes, "Saved path changed during benchmark"
    inputs = [
        args.run / "stroke_graph.json",
        args.run / "construction_plan.json",
        repair_path,
        profile,
        Path("src/pen_art/planning/scene.py"),
        Path("src/pen_art/planning/collision.py"),
        Path(__file__),
    ]
    result = {
        "status": "PASSED",
        "scope": "Sampled exact distance comparison; not full path or build acceptance",
        "material_capsules": len(scene.objects),
        "query_count": len(objects),
        "legacy_seconds": legacy_seconds,
        "spatial": measured,
        "inputs": {str(path): sha256(path) for path in inputs},
    }
    write_json(args.output, result)
    print(result, flush=True)


if __name__ == "__main__":
    main()
