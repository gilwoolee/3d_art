"""Compare saved one-/two-arm runs only under matched geometry and process inputs."""

import argparse
import hashlib
import json
from pathlib import Path

from ..io import artifact, read_json, write_json


def concurrent_drawing_seconds(plan):
    events = []
    for action in plan["actions"]:
        for edge in action["trajectory"]["edges"]:
            start = action["start_s"] + edge["start_s"]
            end = min(plan["modeled_duration_s"], action["start_s"] + edge["end_s"])
            if edge["kind"] == "DRAW" and end > start:
                events.extend(((start, 1), (end, -1)))
    active, previous, total = 0, 0.0, 0.0
    for time, change in sorted(events):
        if active > 1:
            total += time - previous
        active += change
        previous = time
    return total


def compare_cell(run, single_run=None):
    """Compare legacy colocated plans or separately generated offline runs.

    Separate-run inputs are copied as immutable evidence beside the report so
    the workbench can validate it without following paths outside its run.
    """
    run = Path(run)
    plans, checks, configs, inputs = {}, {}, {}, {}
    sources = {"single_arm": Path(single_run) if single_run is not None else run, "dual_arm": run}

    def capture(channel, filename, expected=None):
        if Path(filename).name != filename:
            raise ValueError("Comparison inputs must use local filenames")
        directory = sources[channel].resolve()
        source = (directory / filename).resolve()
        if source.parent != directory:
            raise ValueError("Comparison inputs must be files within their source run")
        raw = source.read_bytes()
        digest = hashlib.sha256(raw).hexdigest()
        if expected is not None and digest != expected:
            raise ValueError(f"Stale {channel} input: {filename}")
        name = filename if single_run is None else f"comparison_{channel}_{filename}"
        if single_run is not None:
            temporary = run / f".{name}.tmp"
            temporary.write_bytes(raw)
            temporary.replace(run / name)
        inputs[name] = digest
        if directory == run.resolve():
            # Invalidate the comparison if its displayed two-arm run changes.
            inputs[filename] = digest
        return raw, digest

    for name in ("single_arm", "dual_arm"):
        file = "offline_dual_arm_plan.json" if single_run is not None else name + "_plan.json"
        raw, plan_digest = capture(name, file)
        plans[name] = json.loads(raw)
        for source, digest in plans[name]["inputs"].items():
            raw, _ = capture(name, source, digest)
            if source == plans[name]["settings_artifact"]:
                configs[name] = json.loads(raw)
        if name not in configs:
            raise ValueError(f"Unbound settings artifact in {name} plan")
        check_file = file.removesuffix("_plan.json") + "_validation.json"
        if (sources[name] / check_file).exists():
            check = read_json(sources[name] / check_file)
            if check["source_plan_sha256"] == plan_digest:
                raw, _ = capture(name, check_file)
                check = json.loads(raw)
                if check["source_plan_sha256"] != plan_digest:
                    raise ValueError("Replay changed while the comparison was captured")
                checks[name] = check
    for name in (
        "cell_manifest.json",
        "cell_drawing_variants.json",
        "construction_plan.json",
        "stroke_graph.json",
    ):
        if plans["single_arm"]["inputs"][name] != plans["dual_arm"]["inputs"][name]:
            raise ValueError(f"Geometry or cell differs: {name}")
    for name in ("process", "timing", "transit"):
        if configs["single_arm"][name] != configs["dual_arm"][name]:
            raise ValueError(f"Fabrication assumptions differ: {name}")
    for name in ("clock", "clock_revision"):
        if configs["single_arm"]["search"].get(name) != configs["dual_arm"]["search"].get(name):
            raise ValueError(f"Time-law settings differ: {name}")
    if plans["single_arm"]["initial_positions"] != plans["dual_arm"]["initial_positions"]:
        raise ValueError("Initial robot configurations differ")
    drawing_arms = {
        name: {a["arm"] for a in plan["actions"] if not a.get("auxiliary", False)}
        for name, plan in plans.items()
    }
    single_arms, dual_arms = drawing_arms["single_arm"], drawing_arms["dual_arm"]
    if len(single_arms) != 1 or len(set(dual_arms)) != 2 or not set(single_arms) <= set(dual_arms):
        raise ValueError("Comparison requires one enabled arm and the matching two-arm cell")

    def fully_replayed(name):
        plan, check = plans[name], checks.get(name, {})
        strokes = {a["stroke_id"] for a in plan["actions"] if not a.get("auxiliary", False)}
        return (
            (plan.get("full_schedule") or plan["status"] == "COORDINATED_GEOMETRIC_PREVIEW")
            and check.get("status") == "CERTIFIED_GEOMETRIC_REPLAY"
            and check.get("completed_strokes") == len(strokes) == plan["requested_strokes"]
            and plan["modeled_duration_s"] > 0
        )

    complete = all(fully_replayed(name) for name in plans)
    output = artifact(
        "cell_comparison",
        inputs=inputs,
        status="MATCHED_GEOMETRIC_COMPARISON" if complete else "INCOMPLETE_COMPARISON",
        executable=False,
        simulation_only=True,
        source_runs={name: str(path.resolve()) for name, path in sources.items()},
        speedup=(
            plans["single_arm"]["modeled_duration_s"] / plans["dual_arm"]["modeled_duration_s"]
        )
        if complete
        else None,
        results={
            name: {
                "status": plan["status"],
                "completed_strokes": checks.get(name, {}).get("completed_strokes", 0),
                "requested_strokes": plan["requested_strokes"],
                "modeled_duration_s": plan["modeled_duration_s"] if fully_replayed(name) else None,
                "validated_prefix_s": plan["modeled_duration_s"]
                if fully_replayed(name)
                else min(
                    (f.get("time_s", 0) for f in checks.get(name, {}).get("failures", [])),
                    default=0,
                ),
                "concurrent_drawing_s": concurrent_drawing_seconds(plan),
                "replay_status": checks.get(name, {}).get("status", "NOT_CHECKED"),
            }
            for name, plan in plans.items()
        },
        limitations=[
            "Same geometry, world cell, process, dynamics and path clock; bounded searches",
            "Unused arm remains parked in the one-arm baseline and remains a collision obstacle",
            "Modeled subtotals exclude unknown startup/shutdown and unmeasured pen behavior",
            "No physical speedup or global optimality claim",
        ],
    )
    write_json(run / "cell_comparison.json", output)
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, default=Path("runs/palace_15cm_dual"))
    parser.add_argument("--single-run", type=Path, help="Matching offline one-arm baseline run")
    args = parser.parse_args()
    print(compare_cell(args.run, args.single_run)["status"])
