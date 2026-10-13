"""Exercise an exact source split through real robot planning and independent replay.

Prepare the small staged source with check_shared_source.py first. Use a fresh
--run. Browser playback can then be checked on the resulting run separately.
"""

import argparse
from pathlib import Path

from pen_art.cell import prepare_cell
from pen_art.io import read_json, sha256, write_json
from pen_art.planning.section_build import (
    SectionBuild,
    prepare_candidates,
    write_section_artifacts,
)
from pen_art.planning.sections import section_actions, split_member_for_restart
from pen_art.provenance import audit_source_provenance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-run", type=Path, default=Path("runs/staged_shared_regression_v1"))
    parser.add_argument("--run", type=Path, default=Path("runs/staged_restart_split_v1"))
    parser.add_argument(
        "--report", type=Path, default=Path("research/blondie/restart_sections.json")
    )
    args = parser.parse_args()
    source, run = args.source_run, args.run
    assert not run.exists(), "Choose a fresh --run; existing evidence is preserved"
    profile = Path("research/planning_cell/cell.yaml")
    prepare_cell(profile, run, source)
    original = read_json(source / "stroke_graph.json")
    graph = split_member_for_restart(original, "edge0", 0.5)
    construction = section_actions(graph)
    assert len(construction["actions"]) == 4
    write_json(run / "construction_plan.json", construction)
    write_section_artifacts(run, graph, construction)
    prepare_candidates(run)
    SectionBuild(run, profile, 600, gpu=True).run_build()
    assert read_json(run / "section_build.json")["status"] == "FULL_BUILD_VERIFIED"
    validation = read_json(run / "offline_dual_arm_validation.json")
    assert validation["status"] == "CERTIFIED_GEOMETRIC_REPLAY"
    plan = read_json(run / "offline_dual_arm_plan.json")
    assert len([row for row in plan["events"] if row["command"] == "FEED_ON"]) == 4
    write_json(
        args.report,
        {
            "status": "PASSED",
            "run": str(run),
            "sections": 4,
            "source_provenance": audit_source_provenance(construction["actions"], graph),
            "replay": validation["status"],
            "checked_intervals": validation["checked_intervals"],
            "simulation_only": True,
            "inputs": {
                str(path): sha256(path)
                for path in [
                    source / "stroke_graph.json",
                    run / "stroke_graph.json",
                    run / "construction_plan.json",
                    run / "offline_dual_arm_plan.json",
                    run / "offline_dual_arm_validation.json",
                    Path("src/pen_art/planning/sections.py"),
                ]
            },
        },
    )
    print("Exact source split: complete robot build and independent replay PASSED", flush=True)


if __name__ == "__main__":
    main()
