"""Report seed availability for real connected actions without launching search."""

import argparse
import json
from pathlib import Path

from pen_art.curves import flatten_curve, transform_points
from pen_art.io import read_json, sha256, write_json
from pen_art.planning.motion_library import DrawingRequest, MotionLibrary
from pen_art.planning.motion_runtime import load_context_for_run
from pen_art.planning.turntable import IndexedTurntable


def coverage(library, source_builds, profile_path):
    requests, inputs, action_kinds = {}, {}, {}
    for source in source_builds:
        config = load_context_for_run(source, profile_path)
        table = IndexedTurntable.from_profile(config["profile"])
        context = config["context_hash"]
        inputs.update(config["inputs"])
        inputs[str(source / "construction_plan.json")] = sha256(source / "construction_plan.json")
        for action in read_json(source / "construction_plan.json")["actions"]:
            parent = action.get("parent_curve", "")
            kind = next((k for k in ("zigzag", "contour") if parent.startswith(k + "-")), "unknown")
            identity = f"{source.name}/{action['id']}"
            action_kinds[identity] = kind
            points, _ = flatten_curve(action, 0.00005, 0.003)
            for index in range(4):
                for reverse in (False, True):
                    requests[f"{identity}/index-{index}/reverse-{int(reverse)}"] = DrawingRequest(
                        source_points_m=transform_points(
                            points[::-1] if reverse else points, table.build_transform(index)
                        ),
                        table_index=index,
                        path_kind=kind,
                        context_hash=context,
                        source_object=source.name,
                        family_id=parent,
                    )
    report = library.coverage(requests)
    by_action = {}
    for row in report["requests"]:
        identity = row["id"].rsplit("/index-", 1)[0]
        by_action.setdefault(identity, []).append(row)
    missing = [
        identity
        for identity, rows in by_action.items()
        if action_kinds[identity] == "zigzag"
        and not any(row["exact_source_seeds"] or row["compatible_family_seeds"] for row in rows)
    ]
    report.update(
        status="READY_FOR_REUSE_CHECKS" if not missing else "MISSING_WHOLE_ZIGZAG_SEEDS",
        inputs=inputs,
        action_count=len(by_action),
        zigzag_actions=sum(kind == "zigzag" for kind in action_kinds.values()),
        zigzag_actions_without_seeds=missing,
        interpretation=(
            "Seed availability only. Every requested path still needs adaptation, "
            "current-material/process validation and full replay."
        ),
    )
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-build", type=Path, action="append", required=True)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument(
        "--profile", type=Path, default=Path("research/planning_cell/cell_turntable.yaml")
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--require-zigzags", action="store_true")
    args = parser.parse_args()
    if not (args.library / "motions").is_dir():
        parser.error("--library must name an existing motion library")
    report = coverage(MotionLibrary(args.library), args.source_build, args.profile)
    if args.output:
        write_json(args.output, report)
    print(
        json.dumps(
            {
                key: report[key]
                for key in (
                    "status",
                    "record_count",
                    "counts",
                    "action_count",
                    "zigzag_actions",
                    "zigzag_actions_without_seeds",
                )
            },
            indent=2,
        )
    )
    return int(args.require_zigzags and bool(report["zigzag_actions_without_seeds"]))


if __name__ == "__main__":
    raise SystemExit(main())
