"""Check complete indexed replay or compare access to an unchanged source stroke.

Source joint paths are proposals, not reused collision certificates. The access
comparison checks a bounded fixed-plate attempt against a retained indexed motion;
it does not prove that every possible fixed-plate motion is infeasible.
"""

import argparse
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
from shutil import copy2
from time import monotonic

import fcl
import numpy as np

from pen_art.cell import prepare_cell
from pen_art.io import read_json, sha256, stage_update, write_json
from pen_art.planning.cell_replay import replay_cell
from pen_art.planning.clock_path import ClockPath
from pen_art.planning.collision import placed
from pen_art.planning.dual_arm import ScheduledMotion
from pen_art.planning.indexed_build import IndexedMotions, at_index
from pen_art.planning.replay_checkpoint import IMPLEMENTATION, read_checkpoint
from pen_art.planning.scene import MaterialScene
from pen_art.planning.section_build import SectionBuild, combine_clocks, prepare_candidates
from pen_art.planning.turntable import (
    IndexedTurntable,
    audit_indexed_clock,
    index_angle,
    prepare_profile,
)


def model_checks(build, table):
    scene = build.scene(set(build.actions))
    arm = replace(build.arm, collision=build.planner.tip_models[build.arm.identity])
    checks = []
    current = 0
    for target in (1, 2, 3, 2, 1, 0):
        result = table.check_parked_turn(
            scene,
            current,
            target,
            arm,
            build.initial,
            build.planner.obstacles,
            margin_m=build.planner.margin,
            deadline=build.planner.deadline,
        )
        checks.append({"from_index": current, "to_index": target, **result})
        print(current, target, result["status"], flush=True)
        scene = table.reposition(scene, index_angle(current), index_angle(target))
        current = target
    original = build.scene(set(build.actions))
    for before, after in zip(original.segments, scene.segments, strict=True):
        for key in ("start", "end"):
            np.testing.assert_allclose(before[key], after[key], atol=1e-12)
    assert all(row["status"] == "CERTIFIED_TURNTABLE_INTERVAL" for row in checks)
    return checks


def swept_collision_rejection(table):
    """A real FCL collision halfway through a turn with both endpoints clear."""
    local = np.array([[0.05, 0, 0.10], [0.05, 0, 0.11]])
    transform = table.transform_at_angle(0)
    points = local @ transform[:3, :3].T + transform[:3, 3]
    scene = MaterialScene()
    scene.add(*points, 0.001, "sweep-probe")
    pose = table.transform_at_angle(np.pi / 4).copy()
    pose[:3, 3] += pose[:3, :3] @ np.array([0.05, 0, 0.105])
    obstacle = placed(fcl.Sphere(0.002), pose)
    assert scene.nearest(obstacle)["distance_m"] > 0.001
    assert table.reposition(scene, 0, np.pi / 2).nearest(obstacle)["distance_m"] > 0.001
    result = table.check_quarter_turn(scene, 0, 1, {"obstacle": obstacle})
    assert result["status"] == "COLLISION" and result["fraction"] == 0.5
    return result


def access_comparison(build, witness):
    """Compare the same source and material state without inventing obstacles."""
    relative = Path("indexed_witnesses") / witness.name
    (build.run / relative).parent.mkdir(parents=True, exist_ok=True)
    copy2(witness, build.run / relative)
    reference = {"file": str(relative), "sha256": sha256(witness)}
    saved = read_json(witness)
    context, row = saved["context"], build.actions[saved["section"]]
    model = IndexedMotions(build)
    available = build.state(context["predecessor"], build.initial).ready_nodes()
    restored = model.restore(reference, row, context, available)
    assert restored is not None, "Indexed witness failed fresh tracking/collision validation"
    indexed_index = restored[0][-1]
    assert indexed_index != 0, "Access comparison needs a nonzero drawing index"
    print(f"Revalidated drawing at index {indexed_index}", flush=True)

    scene = model.scene(context["predecessor"], 0)
    state = build.state(context["predecessor"], build.initial, scene)
    candidate = next(
        model.drawings(row, available, np.asarray(context["terminal"][:7]), 0, scene), None
    )
    fixed = {"status": "NO_SAMPLED_CANDIDATE"}
    if candidate is not None:
        deadline = build.planner.deadline
        try:
            build.planner.deadline = min(deadline, monotonic() + 60)
            move = build.planner.prepare_drawing(state, build.arm.identity, candidate, None)
        finally:
            build.planner.deadline = deadline
        if move is None:
            fixed = {"status": "UNCERTIFIED_DRAWING", "failures": build.planner.attempts}
        else:
            move = replace(move, payload=at_index(move.payload, 0))
            _, failure = build.planner.validate_interval(
                state, (ScheduledMotion(move, 0),), move.duration_s
            )
            fixed = {
                "status": "REJECTED_REPLAY" if failure else "VALID_DRAWING",
                "failure": failure,
            }
    passed = fixed["status"] in {"UNCERTIFIED_DRAWING", "REJECTED_REPLAY"}
    return {
        "status": "INDEXED_ACCESS_WITNESS_PASSED" if passed else "ACCESS_COMPARISON_INCONCLUSIVE",
        "source_section": row["id"],
        "predecessor": context["predecessor"],
        "index_zero": fixed,
        "indexed_drawing_index": indexed_index,
        "indexed_motion": "REVALIDATED_COMPLETE_MOTION_WITH_RETURN",
        "witness": reference,
        "indexed_access_improvement_verified": passed,
    }


def full_replay(build, source, table):
    run = build.run
    original = read_json(source / "offline_dual_arm_plan.json")
    previous = read_json(source / "offline_dual_arm_validation.json")
    assert previous["status"] == "CERTIFIED_GEOMETRIC_REPLAY"
    assert previous["source_plan_sha256"] == sha256(source / "offline_dual_arm_plan.json")
    plan = deepcopy(original)
    for row in plan["actions"]:
        path = ClockPath.from_record(row["trajectory"])
        if "turntable" not in original:
            path = at_index(path, 0)
            if not row.get("auxiliary", False):
                row["table_index"] = 0
                path.metadata["drawing_table_index"] = 0
        row["trajectory"] = path.record()
    scene = build.scene(set(build.actions))
    model = IndexedMotions(build)
    paths = []
    current = 0
    for target in (1, 2, 3, 2, 1, 0):
        path = model.transition(scene, build.initial, current, build.initial, target)
        assert path is not False and path is not None
        paths.append(path)
        scene = table.reposition(scene, index_angle(current), index_angle(target))
        current = target
    turns = combine_clocks(*paths)
    clock = plan["modeled_duration_s"] + build.cooling
    plan["actions"].append(
        {
            "arm": build.arm.identity,
            "stroke_id": "indexed-inspection-return",
            "variant": "six-checked-quarter-turns",
            "start_anchor": "parking",
            "start_s": clock,
            "end_s": clock + turns.duration_s,
            "trajectory": turns.record(),
            "auxiliary": True,
        }
    )
    plan.update(
        status="FULL_SECTION_SCHEDULE_REPLAY_PENDING",
        initial_table_index=0,
        turntable=build.planner.profile["turntable"],
        modeled_duration_s=clock + turns.duration_s,
    )
    settings = original.get("settings_artifact", "offline_dual_arm_config.json")
    write_json(run / settings, read_json(source / settings))
    for name in original["inputs"]:
        if not (run / name).is_file():
            write_json(run / name, read_json(source / name))
    plan["inputs"] = {name: sha256(run / name) for name in original["inputs"]}
    write_json(run / "offline_dual_arm_plan.json", plan)
    result = replay_cell(run, build.profile, float("inf"), "offline_dual_arm_plan")
    assert result["status"] == "CERTIFIED_GEOMETRIC_REPLAY", result.get("failures")
    assert result["completed_strokes"] == len(build.actions)
    build.best = list(build.actions)
    build.report("FULL_BUILD_VERIFIED", replay_status=result["status"])

    rejections = {}
    bad = ClockPath.from_record(turns.record())
    first = next(i for i, edge in enumerate(bad.edges) if edge.kind == "INDEX_TURN")
    changed = list(bad.edges)
    changed[first] = replace(changed[first], q0=build.initial + 0.01, q1=build.initial + 0.01)
    try:
        audit_indexed_clock(table, ClockPath(changed), build.initial, build.cooling)
    except ValueError as error:
        rejections["unparked_turn"] = str(error)
    changed = [edge for edge in bad.edges if edge.kind != "TABLE_SETTLE"]
    try:
        audit_indexed_clock(table, ClockPath(changed), build.initial, build.cooling)
    except ValueError as error:
        rejections["missing_settle"] = str(error)
    wrong = deepcopy(plan)
    next(row for row in wrong["actions"] if not row.get("auxiliary", False))["table_index"] = 1
    write_json(run / "wrong_index_plan.json", wrong)
    try:
        replay_cell(run, build.profile, 60, "wrong_index_plan")
    except ValueError as error:
        rejections["wrong_source_index"] = str(error)
    checkpoint = run / "offline_dual_arm_replay_checkpoint.json"
    assert read_checkpoint(checkpoint, "changed-table-calibration") is None
    rejections["stale_checkpoint"] = "rejected after context change"
    assert set(rejections) == {
        "unparked_turn",
        "missing_settle",
        "wrong_source_index",
        "stale_checkpoint",
    }
    stage_update(run, "validation", "complete", artifact="offline_dual_arm_validation.json")
    return {"replay_status": result["status"], "rejections": rejections, "turns": 6}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--model-only", action="store_true")
    mode.add_argument("--access-witness", type=Path)
    parser.add_argument("--gpu", action="store_true", help="Use cuRobo for the access comparison")
    parser.add_argument("--source-build", type=Path, default=Path("runs/cube_sections_v1"))
    parser.add_argument("--run", type=Path, default=Path("runs/turntable_integration_v1"))
    args = parser.parse_args()
    sources = [
        Path("src/pen_art/planning/turntable.py"),
        Path("src/pen_art/planning/indexed_build.py"),
        Path(__file__),
    ]
    source_hashes = {str(path): sha256(path) for path in sources}
    run = args.run
    assert not run.exists(), "Use a fresh run to retain prior evidence"
    profile = run / "turntable_profile.yaml"
    prepare_profile(Path("research/planning_cell/cell.yaml"), profile)
    prepare_cell(profile, run, args.source_build)
    prepare_candidates(run)
    build = SectionBuild(run, profile, 3600, gpu=args.gpu)
    table = IndexedTurntable.from_profile(build.planner.profile)
    assert read_json(run / "cell_manifest.json")["turntable"] == build.planner.profile["turntable"]
    checks = [] if args.access_witness else model_checks(build, table)
    rejection = swept_collision_rejection(table)
    integration = (
        access_comparison(build, args.access_witness)
        if args.access_witness
        else {}
        if args.model_only
        else full_replay(build, args.source_build, table)
    )
    report = {
        "status": "MODEL_TURN_CHECKS_PASSED" if args.model_only else "INDEXED_BUILD_REPLAY_PASSED",
        "scope": "Six swept quarter-turns with the complete sculpture and parked real arm."
        if args.model_only
        else "Complete empty-plate replay, six quarter-turns, both homes, rejection cases.",
        "simulation_only": True,
        "loaded_implementation": IMPLEMENTATION,
        "full_build_verified": not args.model_only and not args.access_witness,
        "indexed_access_improvement_verified": False,
        "checks": checks,
        **({"source_geometry_round_trip": True} if not args.access_witness else {}),
        "intermediate_collision_rejection": rejection,
        **integration,
        "inputs": source_hashes
        | {
            str(p): sha256(p)
            for p in (
                profile,
                run / "cell_manifest.json",
                run / "stroke_graph.json",
            )
        },
    }
    if args.access_witness:
        report["scope"] = (
            "Same source stroke and predecessor material; bounded fixed-plate attempt "
            "versus revalidated indexed motion. This is not an infeasibility proof."
        )
    write_json(
        run
        / ("turntable_model_check.json" if args.model_only else "turntable_integration_check.json"),
        report,
    )
    print(report["status"], flush=True)
    return 0 if report["status"].endswith("PASSED") else 1


if __name__ == "__main__":
    raise SystemExit(main())
