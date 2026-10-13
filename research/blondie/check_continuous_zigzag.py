"""Bounded whole-action Phase 1 integration on a synthetic three-leg zigzag.

This small source is an integration fixture, not cube/bottle acceptance or a
physical extrusion claim. The real arm, pen, plate and ordinary replay are used.
"""

import argparse
import subprocess
import sys
from collections import Counter
from pathlib import Path

import trimesh
import yaml

from pen_art.assets import prepare_asset
from pen_art.graph import straight_edge
from pen_art.io import artifact, now, read_json, sha256, write_json
from pen_art.planning.section_build import write_section_artifacts
from pen_art.planning.sections import section_actions


def prepare_source(root, name, width):
    source = root / name
    source.mkdir(parents=True)
    mesh = trimesh.creation.box(extents=[0.04, 0.04, 0.01])
    mesh.apply_translation([0, 0, 0.005])
    path = source / "context_mm.stl"
    mesh.export(path)
    write_json(
        source / "run.json",
        {
            "id": source.name,
            "name": "Synthetic three-leg connected zigzag",
            "created_at": now(),
            "updated_at": now(),
            "stages": {},
            "source_geometry_kind": "synthetic_integration_fixture",
            "source_sha256": sha256(path),
        },
    )
    prepare_asset(path, source, {"source_up": "z", "max_extent_m": 0.04, "geometry_faces": 20000})
    points = [
        [-width / 2, -0.03, 0],
        [-width / 6, -0.03, 0.008],
        [width / 6, -0.03, 0.008],
        [width / 2, -0.03, 0],
    ]
    nodes = {
        str(i): {"id": str(i), "position": p, "ground": p[2] == 0} for i, p in enumerate(points)
    }
    edges = []
    for index in range(3):
        edge = straight_edge(str(index), str(index + 1), nodes, f"leg-{index}", "framework", 0.001)
        edge.update(
            parent_curve="zigzag-integration",
            source_interval=[index, index + 1],
            stage="framework",
            origin="wireprint",
            order_hint=0,
        )
        edges.append(edge)
    graph = artifact(
        "stroke_graph",
        nodes=list(nodes.values()),
        edges=edges,
        source_method="synthetic_continuous_zigzag",
        simulation_only=True,
    )
    config = yaml.safe_load(Path("configs/fabrication.yaml").read_text())
    write_json(source / "fabrication_config.json", config)
    construction = section_actions(graph)
    assert len(construction["actions"]) == 1
    write_json(source / "construction_plan.json", construction)
    write_json(source / "stroke_graph.json", graph)
    write_section_artifacts(source, graph, construction)
    return source


def audit_cases(root, report):
    """Check the complete saved operation sequence, not a local cache sample."""
    audits = []
    for case in report["runs"]:
        run = Path(case["run"])
        plan = read_json(run / "offline_dual_arm_plan.json")
        replay = read_json(run / "offline_dual_arm_validation.json")
        actions = [row for row in plan["actions"] if not row.get("auxiliary")]
        assert len(actions) == 1 and len(actions[0]["members"]) == 3
        assert replay["status"] == "CERTIFIED_GEOMETRIC_REPLAY"
        edges = [edge for row in plan["actions"] for edge in row["trajectory"]["edges"]]
        seconds = Counter()
        operations = Counter()
        previous = None
        for edge in edges:
            seconds[edge["kind"]] += edge["end_s"] - edge["start_s"]
            if edge["kind"] != previous:
                operations[edge["kind"]] += 1
            previous = edge["kind"]
        assert operations["APPROACH"] == operations["WITHDRAW"] == 1
        assert operations["VERTEX_COOL"] == 1
        gap = plan["modeled_duration_s"] - sum(seconds.values())
        assert abs(gap) < 1e-8
        audits.append(
            {
                "run": str(run),
                "replay": replay["status"],
                "actions": len(actions),
                "members": 3,
                "approaches": 1,
                "withdrawals": 1,
                "internal_cooling_holds": 1,
                "timing_s": dict(seconds),
                "modeled_duration_s": plan["modeled_duration_s"],
                "unaccounted_seconds": gap,
            }
        )
    result = {
        "status": "VERIFIED",
        "scope": "synthetic_three_leg_integration_only",
        "cases": audits,
    }
    write_json(root / "acceptance.json", result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output", type=Path, default=Path("runs/continuous_zigzag_integration_v1")
    )
    parser.add_argument("--maximum-seconds", type=float, default=75)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    base = prepare_source(args.output, "source", 0.03)
    variation = prepare_source(args.output, "variation_source", 0.0306)
    result = subprocess.run(
        [
            sys.executable,
            "scripts/check_motion_runtime.py",
            "--source-build",
            str(base),
            "--variation-source-build",
            str(variation),
            "--output",
            str(args.output / "builds"),
            "--maximum-seconds",
            str(args.maximum_seconds),
        ],
        check=False,
    )
    report = {
        "scope": "synthetic_three_leg_integration_only",
        "returncode": result.returncode,
        "source": str(base),
        "variation_source": str(variation),
    }
    integration = args.output / "builds/integration.json"
    if integration.exists():
        report.update(read_json(integration))
        report["acceptance"] = audit_cases(args.output, report)
    else:
        report["status"] = "INCOMPLETE"
    report["scope"] = "synthetic_three_leg_integration_only"
    write_json(args.output / "result.json", report)
    raise SystemExit(result.returncode)


if __name__ == "__main__":
    main()
