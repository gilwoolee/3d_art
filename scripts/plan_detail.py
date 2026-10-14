"""Plan a frame with detail tours; drop tours the planner cannot draw; trim to budget.

    .venv/bin/python scripts/plan_detail.py SOURCE BASE_PROFILE OUTPUT \\
        --frame-run RUN [--detail 5.0] [--minutes 15] [--workers 24]

SOURCE is a source run from ``scripts/detail_tours.py``. BASE_PROFILE is the
profile the frame was planned with; a detail profile is written beside it
with the pen tilts detail needs and a smaller clearance for the pen nose (its
tip may run along existing wire). FRAME_RUN is the frame's own planned run; its
print time is the baseline for the added time.

Each round plans the current source with ``scripts/benchmark_parallel_build.py``
(geometry-first, like ``scripts/plan_frame.sh``).

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

from pen_art.curves import curve_length
from pen_art.io import read_json, write_json
from pen_art.planning.section_build import write_section_artifacts
from pen_art.planning.sections import section_actions
from pen_art.structure import analyze_structure

sys.path.insert(0, str(Path(__file__).parent))
from detail_tours import detail_profile, tilt_hints  # noqa: E402


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


def group_faces(source, target):
    """Draw the detail one turntable face at a time, turning a quarter each time.

    Keeps the generator's order within a face. Tours on different faces seldom
    meet each other's pen nose; the planner checks it either way.
    """
    graph = read_json(source / "stroke_graph.json")
    art = [e for e in graph["edges"] if e.get("stage") == "artistic"]
    tours = {}
    for e in art:
        hint = tours.setdefault(e["parent_curve"], [e["order_hint"], e["turntable_index"]])
        hint[0] = min(hint[0], e["order_hint"])
    first = min(tours.values())[1] if tours else 0
    ranked = sorted(tours, key=lambda t: ((tours[t][1] - first) % 4, tours[t][0]))
    rank = {t: i for i, t in enumerate(ranked)}
    for e in art:
        e["order_hint"] = 20000 + rank[e["parent_curve"]]
    if target.exists():
        shutil.rmtree(target)
    shutil.copytree(source, target)
    construction = tilt_hints(section_actions(graph, group_sources=True), graph)
    write_json(target / "stroke_graph.json", graph)
    write_json(target / "construction_plan.json", construction)
    write_section_artifacts(target, graph, construction)
    run = read_json(target / "run.json")
    run["id"] = target.name
    write_json(target / "run.json", run)
    return target


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
    parser.add_argument("--detail", type=float, default=2.0, help="m of detail at most")
    parser.add_argument("--minutes", type=float, default=15.0, help="added print time at most")
    parser.add_argument("--workers", type=int, default=24)
    parser.add_argument("--maximum-rounds", type=int, default=80)
    args = parser.parse_args()

    # Beside the base profile: its robot and tool paths are relative to it.
    profile = detail_profile(
        args.base_profile, args.base_profile.parent / f"{Path(args.output).name}_profile.yaml"
    )
    baseline, _ = planned_seconds(args.frame_run)
    source = group_faces(args.source, Path(f"{args.output}_r00_source"))
    rounds, dropped = [], []
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
            own = sum(r[1] for r in tours.values()) / 60
            row.update(detail_m=detail, added_minutes=added, detail_minutes=own, tours=len(tours))
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
            "detail_minutes": final.get("detail_minutes"),
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
