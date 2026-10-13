"""Compare local candidate preparation and full Clock checks on fixed real late scenes.

Run with the saved baseline source on PYTHONPATH for the baseline process.
No planning searches, publications, or build-worker mutations are performed.
"""

import argparse
import cProfile
from copy import deepcopy
from pathlib import Path
from time import perf_counter

import pen_art.planning.cell_planner as cell_module
from pen_art.io import read_json, sha256, write_json
from pen_art.planning.clock_path import ClockPath
from pen_art.planning.dual_arm import MotionProposal, ScheduledMotion
from pen_art.planning.motion_library import CachedCollisionModel, digest
from pen_art.planning.motion_runtime import static_shapes
from pen_art.planning.path_certificates import PathCertificateStore
from pen_art.planning.replay_checkpoint import IMPLEMENTATION
from pen_art.planning.scene import MaterialScene
from pen_art.planning.section_build import SectionBuild
from pen_art.planning.turntable import index_angle


def file_bindings(paths):
    return {str(Path(path).resolve()): sha256(Path(path)) for path in paths}


def freeze_fixture(run, profile, output):
    """Bind one accepted late action and all actual preceding depositions."""
    build = SectionBuild(run, profile, 600)
    checkpoint = read_json(run / "forward_checkpoint.json")
    references = checkpoint["witnesses"]
    selected = references[-1]
    saved = read_json(run / selected["file"])
    clock = ClockPath.from_record(saved["trajectory"])
    scene = MaterialScene()
    scene.table_index = build.planner.turntable.home_index
    completed = []
    files = [profile, run / "forward_checkpoint.json", run / "construction_plan.json"]
    for reference in references[:-1]:
        path = run / reference["file"]
        if sha256(path) != reference["sha256"]:
            raise ValueError("Saved witness hash changed")
        files.append(path)
        witness = read_json(path)
        completed.append(witness["section"])
        for number, edge in enumerate(ClockPath.from_record(witness["trajectory"]).edges):
            if edge.table_index != scene.table_index:
                scene = build.planner.turntable.reposition(
                    scene, index_angle(scene.table_index), index_angle(edge.table_index)
                )
            if edge.kind == "DRAW":
                build.planner.add_clock_edge(
                    scene, build.arm.identity, edge, f"fixture/{witness['section']}/{number}"
                )
            if edge.kind == "INDEX_TURN":
                scene = build.planner.turntable.reposition(
                    scene, index_angle(scene.table_index), index_angle(edge.table_to_index)
                )
    if scene.table_index != clock.edges[0].table_index:
        raise ValueError("Actual preceding table index differs from saved action")
    files.append(run / selected["file"])
    record = {
        "scope": "One accepted late local action against actual preceding material; "
        "local start configuration is a declared precondition, not a whole-build result",
        "run": str(run.resolve()),
        "profile": str(profile.resolve()),
        "section": saved["section"],
        "completed": completed,
        "drawing": saved["drawing"],
        "clock": saved["trajectory"],
        "material": scene._physical_segments,
        "table_index": scene.table_index,
        "material_sha256": digest(scene._physical_segments),
        "clock_sha256": digest(saved["trajectory"]),
        "inputs": file_bindings(files),
    }
    write_json(output, record)
    print("FROZEN", saved["section"], len(scene.objects), len(clock.edges), flush=True)


def material_scene(fixture, blocked, arm):
    scene = MaterialScene()
    scene.table_index = fixture["table_index"]
    for segment in fixture["material"]:
        scene.add(segment["start"], segment["end"], segment["radius_m"], segment["name"])
    if blocked:
        # A definite obstruction at an interior DRAW pose, away from a nozzle join.
        clock = ClockPath.from_record(fixture["clock"])
        drawing = [edge for edge in clock.edges if edge.kind == "DRAW"]
        edge = drawing[len(drawing) // 2]
        q = edge.position((edge.start_s + edge.end_s) / 2)
        body = arm.tool.bodies[-1]
        point = (arm.robot.frames(q)[body.link] @ body.origin)[:3, 3]
        scene.add(
            point - [0.004, 0, 0], point + [0.004, 0, 0], 0.003, "explicit-interior-pen-obstruction"
        )
    return scene


def timed_contacts():
    original = cell_module.contact_validator
    metrics = {"constructors": 0, "checks": 0, "setup_seconds": 0.0, "check_seconds": 0.0}

    def instrumented(*args, **kwargs):
        started = perf_counter()
        validator = original(*args, **kwargs)
        metrics["setup_seconds"] += perf_counter() - started
        metrics["constructors"] += 1

        def checked(*a, **k):
            started = perf_counter()
            try:
                return validator(*a, **k)
            finally:
                metrics["checks"] += 1
                metrics["check_seconds"] += perf_counter() - started

        checked.failures = validator.failures
        return checked

    cell_module.contact_validator = instrumented
    return original, metrics


def counter_delta(after, before):
    return {key: value - before.get(key, 0) for key, value in after.items()}


def run_case(
    fixture_path,
    output,
    name,
    blocked=False,
    candidate=True,
    candidate_scope="full_robot",
    defer_drawing_check=False,
    profile_cpu=False,
):
    fixture = read_json(fixture_path)
    build = SectionBuild(Path(fixture["run"]), Path(fixture["profile"]), 1200)
    planner, arm = build.planner, build.arm
    cache = output / "cache"
    arm.collision = CachedCollisionModel(
        arm.collision,
        cache,
        geometry_context=arm.tool.metadata,
        static_geometry=static_shapes(planner.profile, planner.turntable),
    )
    planner.path_certificates = PathCertificateStore(cache / "path_certificates")
    planner.verification_raw = False
    planner.settings["collision_scope"] = candidate_scope
    if candidate_scope == "pen_candidate" and not hasattr(planner, "pen_models"):
        raise ValueError("This core snapshot does not support pen_candidate scope")
    for models in (getattr(planner, "pen_models", {}), getattr(planner, "pen_tip_models", {})):
        if arm.identity in models:
            models[arm.identity] = CachedCollisionModel(
                models[arm.identity],
                cache,
                geometry_context=arm.tool.metadata,
                static_geometry=static_shapes(planner.profile, planner.turntable),
            )
    scene = material_scene(fixture, blocked, arm)
    clock = ClockPath.from_record(fixture["clock"])
    state = build.state(fixture["completed"], clock.edges[0].q0, scene)
    if state.table_index != scene.table_index:
        raise ValueError("State did not retain the actual indexed material scene")
    original, contact = timed_contacts()
    result = {
        "case": name,
        "blocked": blocked,
        "implementation": IMPLEMENTATION,
        "fixture_sha256": sha256(fixture_path),
        "section": fixture["section"],
        "initial_capsules": len(scene.objects),
        "material_sha256": digest(scene._physical_segments),
        "clock_sha256": digest(clock.record()),
        "edges": len(clock.edges),
        "candidate_scope": candidate_scope,
        "candidate_preparation_scope": "prepare saved proposal; produced Clock discarded; "
        "fixed original Clock is separately checked",
        "defer_drawing_check": defer_drawing_check,
        "final_scope": "full robot, full pen, plate/static and actual growing material",
    }
    cpu_profile = cProfile.Profile() if profile_cpu else None
    if cpu_profile is not None:
        cpu_profile.enable()
    try:
        if candidate:
            before = deepcopy(contact)
            count_before = dict(planner.verification_stats)
            seconds_before = dict(planner.verification_seconds)
            started = perf_counter()
            prepared = planner.prepare_drawing(
                state,
                arm.identity,
                deepcopy(fixture["drawing"]),
                None,
                defer_drawing_check=defer_drawing_check,
            )
            result["candidate_seconds"] = perf_counter() - started
            result["candidate_prepared"] = prepared is not None
            result["candidate_contact"] = {key: contact[key] - before[key] for key in contact}
            result["candidate_failures"] = deepcopy(planner.attempts)
            result["candidate_counts"] = counter_delta(planner.verification_stats, count_before)
            result["candidate_breakdown"] = counter_delta(
                planner.verification_seconds, seconds_before
            )
            print(name, "PREPARATION", result["candidate_seconds"], flush=True)
        move = MotionProposal(
            fixture["section"],
            arm.identity,
            "fixed-fixture-full-final",
            clock.duration_s,
            "local-start",
            tuple(clock.edges[-1].q1),
            clock,
        )
        if candidate_scope == "pen_candidate":
            count_before = dict(planner.verification_stats)
            seconds_before = dict(planner.verification_seconds)
            started = perf_counter()
            pen_after, pen_failure = planner.validate_interval(
                state, (ScheduledMotion(move, 0),), clock.duration_s
            )
            result.update(
                candidate_clock_seconds=perf_counter() - started,
                candidate_clock_failure=pen_failure,
                candidate_clock_accepted=pen_failure is None,
                candidate_clock_material_sha256=digest(pen_after._physical_segments)
                if pen_after
                else None,
                candidate_clock_counts=counter_delta(planner.verification_stats, count_before),
                candidate_clock_breakdown=counter_delta(
                    planner.verification_seconds, seconds_before
                ),
                candidate_clock_material_queries=dict(getattr(pen_after, "clearance_stats", {})),
            )
        # Explicitly restore the retained full robot for final qualification.
        planner.settings["collision_scope"] = "full_robot"
        if getattr(planner, "collision_scope", "full_robot") != "full_robot":
            raise ValueError("Final check did not select the full robot")
        before = deepcopy(contact)
        count_before = dict(planner.verification_stats)
        seconds_before = dict(planner.verification_seconds)
        started = perf_counter()
        after, failure = planner.validate_interval(
            state, (ScheduledMotion(move, 0),), clock.duration_s
        )
        result.update(
            final_seconds=perf_counter() - started,
            failure=failure,
            accepted=failure is None,
            final_capsules=len(after.objects) if after is not None else None,
            final_material_sha256=digest(after._physical_segments) if after else None,
            final_contact={key: contact[key] - before[key] for key in contact},
            verification_counts=counter_delta(planner.verification_stats, count_before),
            verification_seconds=counter_delta(planner.verification_seconds, seconds_before),
            material_queries=dict(getattr(after, "clearance_stats", {})),
        )
        if candidate_scope == "full_robot":
            result["candidate_clock_seconds"] = result["final_seconds"]
            result["candidate_clock_accepted"] = result["accepted"]
            result["candidate_clock_failure"] = failure
            result["candidate_clock_measured_once_as_final"] = True
        arm.collision.flush()
    finally:
        cell_module.contact_validator = original
        if cpu_profile is not None:
            cpu_profile.disable()
            output.mkdir(parents=True, exist_ok=True)
            cpu_profile.dump_stats(str(output / f"{name}.prof"))
    write_json(output / f"{name}.json", result)
    print(
        name,
        result.get("candidate_seconds"),
        result["final_seconds"],
        result["accepted"],
        result["failure"],
        flush=True,
    )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--freeze", action="store_true")
    parser.add_argument("--run", type=Path)
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--fixture", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--skip-candidate", action="store_true")
    parser.add_argument("--defer-drawing-check", action="store_true")
    parser.add_argument("--profile-cpu", action="store_true")
    parser.add_argument(
        "--candidate-scope", choices=["full_robot", "pen_candidate"], default="full_robot"
    )
    parser.add_argument("--case", choices=["all", "cold", "warm", "blocked"], default="all")
    args = parser.parse_args()
    if args.freeze:
        freeze_fixture(args.run, args.profile, args.fixture)
    else:
        rows = []
        for name, blocked in (("cold", False), ("warm", False), ("blocked", True)):
            if args.case not in {"all", name}:
                continue
            rows.append(
                run_case(
                    args.fixture,
                    args.output,
                    name,
                    blocked,
                    candidate=not args.skip_candidate,
                    candidate_scope=args.candidate_scope,
                    defer_drawing_check=args.defer_drawing_check,
                    profile_cpu=args.profile_cpu,
                )
            )
        write_json(
            args.output / "result.json",
            {
                "status": "COMPLETE",
                "implementation": IMPLEMENTATION,
                "script_sha256": sha256(Path(__file__)),
                "results": rows,
                "scope": "Fixed late action validation only; no full-build acceptance",
            },
        )


if __name__ == "__main__":
    main()
