"""The three planning stages: plan-geometry, plan-timing and verify-full-robot.

An object's work lives in ``runs/NAME/``: the frame (``frame_source/``), the
detail when there is one (``detail/``), one folder per planning round
(``round-NN/``) and ``state.json``, which names the current round and records
what each stage did. Each stage reads the previous stage's result from there
and refuses to run when it is missing or its inputs changed.

* ``plan_geometry`` finds a collision-free path for every stroke, untimed. A
  detail stroke the arm cannot draw is dropped and planning goes on without
  it; a frame stroke that cannot be drawn stops planning.
* ``plan_timing`` times the paths once. When the detail is longer than its
  budget or adds too many minutes, the strokes giving the least detail per
  minute are dropped and the rest planned again.
* ``verify_full_robot`` replays the whole robot against everything built so
  far. A plan that passes is ``FULL_BUILD_VERIFIED``.
"""

import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path

import yaml

from .curves import curve_length
from .io import read_json, sha256, write_json
from .planning.section_build import write_section_artifacts
from .planning.sections import section_actions
from .structure import analyze_structure

STATE = "state.json"
# The frame.yaml sections read when the paths are planned and timed. The
# others (wireprint, graph, structure, process) are copied into the frame by
# `pen-art frame`, so they take effect when the frame is made again.
PLANNING_SECTIONS = ("timing", "transit", "planning")


class StageOrderError(RuntimeError):
    """A stage was run before the one it builds on, or on changed inputs."""


def _state(folder):
    path = Path(folder) / STATE
    return read_json(path) if path.exists() else {}


def _save(folder, state):
    write_json(Path(folder) / STATE, state)
    return state


def _section_build(run, source, profile, workers, stage, log):
    command = [
        sys.executable, "-m", "pen_art.planning.section_build",
        "--cell-source-run", str(source), "--run", str(run), "--profile", str(profile),
        "--workers", str(workers), "--maximum-seconds", "7200", "--stage", stage,
    ]
    with open(log, "a") as out:
        subprocess.run(command, stdout=out, stderr=subprocess.STDOUT, check=False)
    record = Path(run) / "run.json"
    if record.exists():
        named = read_json(record)
        named["name"] = Path(run).parent.name + (
            ": frame and detail" if Path(source).name != "frame_source" else ": frame"
        )
        write_json(record, named)
    return read_json(Path(run) / "section_build.json")


def drop_strokes(source, target, tours):
    """Copy a source run without the named detail strokes."""
    from .detail import tilt_hints

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


def _inputs(source, profile, cell):
    from .cell_config import load_frame_config

    config = load_frame_config()
    sections = json.dumps({name: config[name] for name in PLANNING_SECTIONS}, sort_keys=True)
    return {
        "stroke_graph": sha256(Path(source) / "stroke_graph.json"),
        "profile": sha256(Path(profile)),
        "cell": sha256(Path(cell)),
        "frame_config": hashlib.sha256(sections.encode()).hexdigest(),
    }


def _check_build_volume(frame, cell):
    """The frame must have been made for the cell's build volume."""
    made = read_json(Path(frame) / "wire_budget.json")["requested_max_extent_m"]
    wanted = yaml.safe_load(Path(cell).read_text())["build_volume_m"]
    if abs(made - wanted) > 1e-9:
        raise StageOrderError(
            f"the frame was made for objects up to {made * 100:g} cm and the cell now says "
            f"{wanted * 100:g} cm: run `pen-art frame` again"
        )


def plan_geometry(folder, profile, workers=24, frame_only=False, maximum_rounds=20, log=print):
    """Plan untimed paths for the frame, and the detail when ``detail/`` exists."""
    folder, profile = Path(folder), Path(profile)
    cell = profile
    frame = folder / "frame_source"
    if not (frame / "stroke_graph.json").exists():
        raise StageOrderError(f"no frame in {frame}: run `pen-art frame` first")
    _check_build_volume(frame, cell)
    has_detail = (folder / "detail" / "stroke_graph.json").exists()
    source = folder / "detail" if has_detail and not frame_only else frame
    if source.name == "detail":
        from .detail import detail_profile

        # Beside the cell profile: its robot paths are relative to it.
        profile = detail_profile(profile, profile.parent / f"{folder.name}_detail.yaml")
    # Written first, so an interrupted run never leaves a state naming a
    # round that is gone.
    state = _save(folder, {"frame": str(frame), "cell": str(cell), "geometry": "RUNNING"})
    for old in folder.glob("round-*"):
        shutil.rmtree(old)
    state.update(source=str(source), profile=str(profile))
    dropped = []
    for number in range(maximum_rounds):
        run = folder / f"round-{number:02d}"
        report = _section_build(
            run, source, profile, workers, "geometry", folder / "planning.log"
        )
        status = report["status"]
        plan = {a["id"]: a for a in read_json(source / "construction_plan.json")["actions"]}
        blocked = [row["action"] for row in report.get("parallel", {}).get("blocked", [])]
        if status == "GEOMETRY_PLANNED" or not blocked:
            break
        tours = sorted({plan[a].get("parent_curve") for a in blocked if a in plan})
        if not all(plan[a].get("stage") == "artistic" for a in blocked if a in plan):
            status = "FRAME_STROKE_BLOCKED"
            state["blocked"] = blocked
            break
        log(f"  dropping detail the arm cannot draw: {', '.join(tours)}")
        dropped += [{"stroke": t, "reason": "no collision-free path"} for t in tours]
        following = folder / f"round-{number + 1:02d}-source"
        if not drop_strokes(source, following, set(tours)):
            status = "STRUCTURE_FAILS_WITHOUT_DROPPED_DETAIL"
            break
        source = following
    state.update(
        geometry=status,
        round=run.name,
        source=str(source),
        inputs=_inputs(source, profile, cell),
        dropped=dropped,
        lift_offs=report.get("lift_offs", []),
        minimum_trails=report.get("minimum_trails"),
    )
    return _save(folder, state)


def _require(folder, stage, status):
    state = _state(folder)
    command = "plan-geometry" if stage == "geometry" else "plan-timing"
    if state.get(stage) is None:
        raise StageOrderError(f"{folder}: run `pen-art {command}` first")
    if state[stage] != status:
        raise StageOrderError(
            f"{folder}: `pen-art {command}` ended with {state[stage]} (see "
            f"{Path(folder) / 'planning.log'}); fix that and run it again"
        )
    _check_build_volume(state["frame"], state["cell"])
    if state["inputs"] != _inputs(state["source"], state["profile"], state["cell"]):
        raise StageOrderError(
            f"{folder}: the frame, the cell or the planning settings in configs/frame.yaml "
            "changed since `pen-art plan-geometry`: run it again"
        )
    return state


def detail_budget(run, source):
    """(detail metres, minutes after the frame, per-stroke metres and seconds)."""
    plan = read_json(Path(run) / "plan.json")
    actions = {a["id"]: a for a in read_json(Path(source) / "construction_plan.json")["actions"]}
    strokes, frame_end = {}, 0.0
    for row in plan["actions"]:
        action = actions.get(row.get("stroke_id"))
        if action is None:
            continue
        if action.get("stage") == "artistic":
            seconds = sum(e["end_s"] - e["start_s"] for e in row["trajectory"]["edges"])
            rec = strokes.setdefault(action["parent_curve"], [0.0, 0.0])
            rec[0] += curve_length(action)
            rec[1] += seconds
        else:
            frame_end = max(frame_end, row["end_s"])
    detail = sum(r[0] for r in strokes.values())
    return detail, (plan["modeled_duration_s"] - frame_end) / 60, strokes


def plan_timing(folder, workers=24, detail_m=2.0, minutes=15.0, maximum_rounds=20, log=print):
    """Time the planned paths; trim detail to its budget, planning again if needed."""
    folder = Path(folder)
    state = _require(folder, "geometry", "GEOMETRY_PLANNED")
    profile, source = Path(state["profile"]), Path(state["source"])
    trimmed = []
    for _ in range(maximum_rounds):
        run = folder / state["round"]
        report = _section_build(run, source, profile, workers, "timing", folder / "planning.log")
        status = report["status"]
        if status != "FULL_BUILD_PLANNED" or source.name == "frame_source":
            break
        detail, added, strokes = detail_budget(run, source)
        drop = []
        for name in sorted(strokes, key=lambda n: strokes[n][0] / max(strokes[n][1], 1e-9)):
            if detail <= detail_m + 1e-6 and added <= minutes:
                break
            drop.append(name)
            detail -= strokes[name][0]
            added -= strokes[name][1] / 60
        if not drop:
            break
        log(f"  trimming {len(drop)} detail strokes to fit {detail_m:g} m and {minutes:g} min")
        trimmed += [{"stroke": t, "reason": "over the detail or time budget"} for t in drop]
        number = int(state["round"].split("-")[1]) + 1
        following = folder / f"round-{number:02d}-source"
        drop_strokes(source, following, set(drop))
        source = following
        state = plan_geometry_round(folder, state, source, number, workers)
        if state["geometry"] != "GEOMETRY_PLANNED":
            status = state["geometry"]
            break
    run = folder / state["round"]
    plan = read_json(run / "plan.json") if (run / "plan.json").exists() else {}
    detail = detail_budget(run, source)[:2] if plan and source.name != "frame_source" else None
    state.update(
        timing=status,
        source=str(source),
        inputs=_inputs(source, profile, state["cell"]),
        trimmed=state.get("trimmed", []) + trimmed,
        print_minutes=plan.get("modeled_duration_s", 0) / 60 or None,
        lift_offs=plan.get("lift_offs", state.get("lift_offs")),
        detail_m=None if detail is None else detail[0],
        detail_minutes=None if detail is None else detail[1],
    )
    state.pop("verification", None)
    return _save(folder, state)


def plan_geometry_round(folder, state, source, number, workers):
    """One more geometry round on a trimmed source (no dropping: it planned before)."""
    run = folder / f"round-{number:02d}"
    report = _section_build(
        run, source, state["profile"], workers, "geometry", folder / "planning.log"
    )
    return {
        **state,
        "geometry": report["status"],
        "round": run.name,
        "source": str(source),
        "inputs": _inputs(source, state["profile"], state["cell"]),
        "lift_offs": report.get("lift_offs", []),
    }


def verify_full_robot(folder, workers=24):
    """Replay the whole robot against everything built so far."""
    folder = Path(folder)
    state = _require(folder, "timing", "FULL_BUILD_PLANNED")
    run = folder / state["round"]
    command = [
        sys.executable, "-m", "pen_art.planning.cell_replay",
        "--run", str(run), "--config", state["profile"], "--workers", str(workers),
        "--maximum-seconds", "7200", "--accept-geometry-proofs",
    ]
    (run / "verification.json").unlink(missing_ok=True)
    with open(folder / "planning.log", "a") as out:
        result = subprocess.run(command, stdout=out, stderr=subprocess.STDOUT, check=False)
    # The verdict of this replay, never one left over from an earlier one.
    finished = (run / "verification.json").exists()
    state["verification"] = (
        read_json(run / "section_build.json")["status"] if finished else "VERIFICATION_ERROR"
    )
    if result.returncode and state["verification"] == "FULL_BUILD_VERIFIED":
        state["verification"] = "VERIFICATION_ERROR"
    return _save(folder, state)
