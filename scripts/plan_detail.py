"""Plan a frame with detail tours; drop tours the planner cannot draw; trim to budget.

    .venv/bin/python scripts/plan_detail.py SOURCE BASE_PROFILE OUTPUT \\
        --frame-run RUN [--detail 5.0] [--minutes 15] [--workers 24]

SOURCE is a source run from ``scripts/detail_tours.py``. BASE_PROFILE is the
profile the frame was planned with; ``OUTPUT_profile.yaml`` is derived from it
with the pen tilts detail needs and a smaller clearance for the pen nose (its
tip may run along existing wire). FRAME_RUN is the frame's own planned run; its
print time is the baseline for the added time.

Each round plans the current source with ``scripts/benchmark_parallel_build.py``
(geometry-first, like ``runs/geomfirst_one.sh``).

* Planning stops and every blocked stroke is a detail tour: those tours are
  removed (members, nodes, fused contacts), the plan is rebuilt, next round.
* A blocked frame stroke ends the loop: the frame must plan whole.
* Everything plans but the detail is longer than ``--detail`` or adds more
  than ``--minutes``: the tours giving the least detail per planned second
  are removed, and the next round plans the rest.

``OUTPUT_detail.json`` records every round, what was dropped and why, and the
final run's detail length and added print time.
"""

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

import yaml

from pen_art.curves import curve_length
from pen_art.io import read_json, write_json
from pen_art.planning.section_build import write_section_artifacts
from pen_art.planning.sections import section_actions
from pen_art.structure import analyze_structure

sys.path.insert(0, str(Path(__file__).parent))
from detail_tours import tilt_hints  # noqa: E402

DETAIL_TILTS = [0.0, 60.0, 45.0, 30.0, 15.0]
NOSE_CLEARANCE_M = 0.0003


def detail_profile(base, path):
    """The frame's profile, plus pen tilts for detail and the nose clearance."""
    profile = yaml.safe_load(base.read_text())
    profile["drawing_pen_tilt_candidates_deg"] = DETAIL_TILTS
    profile["environment"]["tool_clearance_m"] = {"tapered_nose": NOSE_CLEARANCE_M}
    profile["setup_description"] = (
        profile.get("setup_description", "")
        + " Detail strokes after the frame may tilt the pen toward the robot; the nose"
        + f" keeps {NOSE_CLEARANCE_M * 1000:g} mm from wire."
    )
    path.write_text(yaml.safe_dump(profile, sort_keys=False))
    return path


def drop_tours(source, target, tours):
    graph = read_json(source / "stroke_graph.json")
    gone = {e["id"] for e in graph["edges"] if e.get("parent_curve") in tours}
    graph["edges"] = [e for e in graph["edges"] if e["id"] not in gone]
    used = {n for e in graph["edges"] for n in (e["start"], e["end"])}
    graph["nodes"] = [n for n in graph["nodes"] if n["id"] in used]
    graph["fused_turns"] = [
        p for p in graph.get("fused_turns", []) if p[0] in used and p[1] in used
    ]
    if target.exists():
        shutil.rmtree(target)
    shutil.copytree(source, target)
    construction = tilt_hints(section_actions(graph, group_sources=True), graph)
    write_json(target / "stroke_graph.json", graph)
    write_json(target / "construction_plan.json", construction)
    write_section_artifacts(target, graph, construction)
    screen = analyze_structure(graph, read_json(source / "fabrication_config.json")["structure"])
    write_json(target / "structural_screen.json", screen)
    run = read_json(target / "run.json")
    run["id"] = target.name
    write_json(target / "run.json", run)
    return screen["modeled_pass"]


def planned_seconds(run):
    plan = read_json(run / "offline_dual_arm_plan.json")
    per_action = {
        a.get("stroke_id", a.get("id")): sum(
            e["end_s"] - e["start_s"] for e in a["trajectory"]["edges"]
        )
        for a in plan["actions"]
        if "trajectory" in a
    }
    return plan["modeled_duration_s"], per_action


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("source", type=Path)
    parser.add_argument("base_profile", type=Path)
    parser.add_argument("output", help="run prefix, e.g. runs/geomfirst_gf90_liberty_detail")
    parser.add_argument("--frame-run", type=Path, required=True)
    parser.add_argument("--detail", type=float, default=5.0, help="m of detail at most")
    parser.add_argument("--minutes", type=float, default=15.0, help="added print time at most")
    parser.add_argument("--workers", type=int, default=24)
    parser.add_argument("--maximum-rounds", type=int, default=80)
    args = parser.parse_args()

    profile = detail_profile(args.base_profile, Path(f"{args.output}_profile.yaml"))
    baseline, _ = planned_seconds(args.frame_run)
    source, rounds, dropped = args.source, [], []
    for number in range(args.maximum_rounds):
        output = f"{args.output}_r{number:02d}"
        with open(f"{output}_benchmark.out", "w") as log:
            subprocess.run(
                [
                    ".venv/bin/python",
                    "scripts/benchmark_parallel_build.py",
                    "--source-run",
                    str(source),
                    "--profile",
                    str(profile),
                    "--output",
                    output,
                    "--workers",
                    str(args.workers),
                    "--maximum-seconds",
                    "7200",
                    "--build-option=--geometry-first",
                    "--verification-option=--accept-geometry-proofs",
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
                check=False,
            )
        run = Path(f"{output}_w{args.workers}")
        status = read_json(run / "section_build.json")["status"]
        plan = {a["id"]: a for a in read_json(source / "construction_plan.json")["actions"]}
        blocked, reason = [], "planner found no motion"
        if (run / "parallel_build.json").exists():
            blocked = [
                row["action"] for row in read_json(run / "parallel_build.json").get("blocked", [])
            ]
        validation = run / "offline_dual_arm_validation.json"
        if not blocked and validation.exists() and status == "FULL_BUILD_REPLAY_REJECTED":
            # Planning checks the pen; the final replay checks the whole robot.
            failures = read_json(validation).get("failures", [])
            blocked = sorted({f["stroke_id"] for f in failures if "stroke_id" in f})
            reason = "full-robot replay: " + ", ".join(
                sorted({f.get("status", "?") for f in failures})
            )
        row = {"round": number, "source": str(source), "run": str(run), "status": status}
        rounds.append(row)
        if blocked:
            tours = sorted({plan[a].get("parent_curve") for a in blocked if a in plan})
            row.update(blocked=blocked, dropped=tours, reason=reason)
            print(f"round {number}: {status}; dropping {tours}", flush=True)
            if not all(plan[a].get("stage") == "artistic" for a in blocked if a in plan):
                row["note"] = "a frame stroke is blocked"
                break
        elif (run / "offline_dual_arm_plan.json").exists():
            total, per_action = planned_seconds(run)
            tours = {}
            for action in plan.values():
                if action.get("stage") == "artistic":
                    rec = tours.setdefault(action["parent_curve"], [0.0, 0.0])
                    rec[0] += curve_length(action)
                    rec[1] += per_action.get(action["id"], 0.0)
            detail, added = sum(r[0] for r in tours.values()), (total - baseline) / 60
            row.update(detail_m=detail, added_minutes=added, tours=len(tours))
            print(
                f"round {number}: {status}; {detail:.2f} m detail in {len(tours)} tours, "
                f"{added:.1f} min added",
                flush=True,
            )
            tours_left = []
            for name in sorted(tours, key=lambda n: tours[n][0] / max(tours[n][1], 1e-9)):
                if detail <= args.detail + 1e-6 and added <= args.minutes:
                    break
                tours_left.append(name)
                detail -= tours[name][0]
                added -= tours[name][1] / 60
            if not tours_left:
                break
            row.update(dropped=tours_left, reason="over the detail or time budget")
            print(f"  trimming {len(tours_left)} tours to fit the budget", flush=True)
            tours = tours_left
        else:
            row["note"] = "planning stopped without blocked strokes"
            break
        dropped += [{"tour": t, "round": number, "reason": row["reason"]} for t in row["dropped"]]
        nxt = Path(f"{args.output}_r{number + 1:02d}_source")
        if not drop_tours(source, nxt, set(row["dropped"])):
            row["note"] = "structure fails without the dropped tours"
            break
        source = nxt
    final = rounds[-1]
    write_json(
        Path(f"{args.output}_detail.json"),
        {
            "rounds": rounds,
            "dropped_tours": dropped,
            "final_run": final["run"],
            "final_source": final["source"],
            "detail_m": final.get("detail_m"),
            "added_minutes": final.get("added_minutes"),
            "frame_seconds": baseline,
            "profile": str(profile),
        },
    )
    print(
        f"final {final['run']} {final['status']}: {final.get('detail_m', 0):.2f} m detail, "
        f"{final.get('added_minutes', float('nan')):.1f} min added; dropped {len(dropped)} tours"
    )


if __name__ == "__main__":
    main()
