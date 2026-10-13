"""Compare exact full-robot checks with whole versus local contact geometry.

The saved late-action fixture contains its actual preceding deposits. This is
a local comparison, not an end-to-end build benchmark. No published run changes.
"""

import argparse
import gc
import importlib.util
from pathlib import Path
from time import perf_counter

from pen_art.io import read_json, sha256, write_json
from pen_art.planning.clock_path import ClockPath
from pen_art.planning.dual_arm import MotionProposal, ScheduledMotion
from pen_art.planning.replay_checkpoint import IMPLEMENTATION
from pen_art.planning.section_build import SectionBuild


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fixture", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Choose a fresh output file")
    spec = importlib.util.spec_from_file_location(
        "local_collision_fixture", Path(__file__).with_name("benchmark_local_collision.py")
    )
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    fixture = read_json(args.fixture)
    build = SectionBuild(Path(fixture["run"]), Path(fixture["profile"]), 1200)
    planner = build.planner
    planner.verification_raw = True  # No invariant/cached collision proofs.
    clock = ClockPath.from_record(fixture["clock"])
    motion = ScheduledMotion(MotionProposal(
        fixture["section"], build.arm.identity, "exact-clock", clock.duration_s,
        "local-start", tuple(clock.edges[-1].q1), clock,
    ), 0)
    results = []
    for blocked in (False, True):
        for local in (False, True):
            scene = helper.material_scene(fixture, blocked, build.arm)
            planner.optimize_contact_geometry = local
            planner.verification_stats.clear()
            planner.verification_seconds.clear()
            planner.checked_intervals = 0
            started = perf_counter()
            after, failure = planner.validate_interval(
                build.state(fixture["completed"], clock.edges[0].q0, scene),
                (motion,), clock.duration_s,
            )
            row = {
                "blocked": blocked, "local_contact_geometry": local,
                "seconds": perf_counter() - started, "failure": failure,
                "accepted": failure is None, "collision_scope": planner.collision_scope,
                "initial_geometry": [scene.geometry_prefix()[0].hex(), scene.table_index],
                "final_geometry": None if after is None else [
                    after.geometry_prefix()[0].hex(), after.table_index
                ],
                "final_capsules": None if after is None else len(after.objects),
                "checked_intervals": planner.checked_intervals,
                "counts": dict(planner.verification_stats),
                "stage_seconds": dict(planner.verification_seconds),
            }
            results.append(row)
            write_json(args.output, {
                "status": "RUNNING", "implementation": IMPLEMENTATION,
                "fixture_sha256": sha256(args.fixture), "results": results,
            })
            print(blocked, local, row["accepted"], f"{row['seconds']:.2f}s", flush=True)
            del after, scene
            gc.collect()
    assert all(row["collision_scope"] == "full_robot" for row in results)
    for baseline, local in zip(results[::2], results[1::2], strict=True):
        assert baseline["accepted"] == local["accepted"]
        assert baseline["initial_geometry"] == local["initial_geometry"]
        assert baseline["final_geometry"] == local["final_geometry"]
        assert baseline["checked_intervals"] == local["checked_intervals"]
        assert baseline["accepted"] is not baseline["blocked"]
    assert not planner.path_certificates.stats
    write_json(args.output, {
        "status": "COMPLETE", "implementation": IMPLEMENTATION,
        "fixture_sha256": sha256(args.fixture), "results": results,
        "scope": "Same late action and actual material, exact fresh full-robot checks",
    })


if __name__ == "__main__":
    main()
