"""Replay existing strokes against material deposited so far.

Fractional cuts here are analysis samples only, never fabrication cuts. Frames
retain true earlier junctions and expose a temporary pen tip, not a future anchor.
"""

import argparse
from collections import Counter
from pathlib import Path
from time import monotonic

import numpy as np

from .curves import curve_interval, curve_length, flatten_curve
from .io import artifact, read_json, sha256, stage_update, write_json
from .process import evaluate_action
from .structure import analyze_structure


def partial_state(graph, action, deposited, fraction):
    """Exact cubic restriction at an arc-length-approximated progress fraction."""
    if not 0 < fraction <= 1:
        raise ValueError("Growth fraction must be in (0,1]")
    members = set(deposited)
    nodes = {node["id"]: node for node in graph["nodes"]}
    if fraction == 1:
        members.update(action["members"])
        tip = action["end"]
        partial = None
    else:
        points, parameters = flatten_curve(action, tolerance=0.000025, max_chord=0.001)
        lengths = np.r_[0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
        parameter = float(np.interp(fraction * lengths[-1], lengths, parameters))
        visits = action["junction_visits"]
        # An exact junction visit inherits its real identity; never add a nearby fake weld.
        preceding = 0
        for index, visit in enumerate(visits[:-1]):
            if visit["segment_boundary"] <= parameter + 1e-10:
                preceding = index
        members.update(action["members"][:preceding])
        start = visits[preceding]
        if abs(parameter - start["segment_boundary"]) <= 1e-10:
            tip, partial = start["node"], None
        else:
            segments = curve_interval(action, start["segment_boundary"], parameter)
            tip = "growth-nozzle-tip"
            point = segments[-1][-1]
            nodes[tip] = {
                "id": tip,
                "position": point,
                "ground": abs(point[2]) <= 1e-9,
                "join_status": "temporary_analysis_tip",
            }
            partial = {
                "id": "growth-partial-member",
                "start": start["node"],
                "end": tip,
                "segments": segments,
                "diameter_m": action["diameter_m"],
                "role": action["role"],
                "source_member": action["members"][preceding],
            }
            partial["length_m"] = curve_length(partial)
    edges = [edge for edge in graph["edges"] if edge["id"] in members]
    if partial:
        edges.append(partial)
    used = {end for edge in edges for end in (edge["start"], edge["end"])}
    return (
        {"nodes": [node for key, node in nodes.items() if key in used], "edges": edges},
        tip,
        partial,
    )


def compact_screen(screen):
    """Store diagnostics and per-member scalars without repeated beam sample arrays."""
    result = {
        key: screen[key]
        for key in (
            "modeled_pass",
            "failure",
            "maximum_displacement_m",
            "maximum_stress_pa",
            "linear_model_valid",
            "relative_residual",
            "resting_com_inside_contacts",
        )
        if key in screen
    }
    result["status"] = screen.get(
        "failure", "MODELED_PASS" if screen["modeled_pass"] else "MODELED_LIMIT_EXCEEDED"
    )
    result["edges"] = [
        {
            key: edge[key]
            for key in (
                "id",
                "maximum_displacement_m",
                "maximum_stress_pa",
            )
        }
        for edge in screen["edges"]
    ]
    return result


def growth_run(run, maximum_seconds=60):
    run = Path(run)
    if maximum_seconds <= 0 or not np.isfinite(maximum_seconds):
        raise ValueError("Growth budget must be finite and positive")
    supported = (run / "backbone_construction.json").is_file()
    graph_file = "backbone_graph.json" if supported else "stroke_graph.json"
    construction_file = "backbone_construction.json" if supported else "construction_plan.json"
    inputs = {
        name: sha256(run / name)
        for name in (graph_file, construction_file, "fabrication_config.json")
    }
    graph, construction, config = (
        read_json(run / name) for name in (graph_file, construction_file, "fabrication_config.json")
    )
    actions = construction["actions"]
    if not actions:
        raise ValueError("A nonempty construction order is required")
    fractions = (0.25, 0.5, 0.75, 1.0)
    destination = run / "growth_screen.json"
    report = artifact(
        "construction_growth_screen",
        policy_version=1,
        inputs=inputs,
        graph_file=graph_file,
        construction_file=construction_file,
        frame="build",
        units="m",
        simulation_only=True,
        physical_status="UNVERIFIED",
        calibrated=config["structure"]["calibrated"],
        fraction_samples=list(fractions),
        requested_frames=len(actions) * len(fractions),
        frames=[],
        process_actions=[],
        settings=config["structure"],
        limitations=[
            "Sampled progress only, not continuous within-stroke certification",
            "All deposited material uses cooled elastic properties; thermal behavior is unmeasured",
            "Held-tip screen assumes an ideal translational point constraint, with free rotation",
            "Rigid welds and clamped bed; these screens do not certify fabrication",
            "Partial cubic cuts are analysis samples and do not change the fabrication strokes",
            "Process ages use nominal serial drawing time; no coordinated motion timing is implied",
        ],
    )
    if destination.is_file():
        saved = read_json(destination)
        if (
            saved.get("inputs") == inputs
            and saved.get("policy_version") == report["policy_version"]
        ):
            report = saved
    started = monotonic()
    stage_update(run, "growth", "running", artifact=destination.name)
    deposited, births, clock = set(), {}, 0.0
    nodes = {node["id"]: node for node in graph["nodes"]}
    completed = len(report["frames"])
    deadline = False
    for index, action in enumerate(actions):
        if index >= len(report["process_actions"]):
            report["process_actions"].append(
                {
                    "action_index": index,
                    "action_id": action["id"],
                    **evaluate_action(
                        action,
                        nodes,
                        config["process"],
                        set(births),
                        {key: clock - birth for key, birth in births.items()},
                    ),
                }
            )
        for fraction in fractions:
            ordinal = index * len(fractions) + fractions.index(fraction)
            if ordinal < completed:
                continue
            if monotonic() - started >= maximum_seconds:
                deadline = True
                break
            state, tip, partial = partial_state(graph, action, deposited, fraction)
            released = compact_screen(analyze_structure(state, config["structure"]))
            held = (
                None
                if fraction == 1
                else compact_screen(
                    analyze_structure(state, config["structure"], fixed_translations=[tip])
                )
            )
            report["frames"].append(
                {
                    "index": ordinal,
                    "action_index": index,
                    "action_id": action["id"],
                    "fraction": fraction,
                    "tip": next(n["position"] for n in state["nodes"] if n["id"] == tip),
                    "material_members": [e["id"] for e in state["edges"] if e is not partial],
                    "partial_edge": partial,
                    "released": released,
                    "held": held,
                }
            )
        if deadline:
            break
        deposited.update(action["members"])
        clock += action["length_m"] / config["process"]["nominal_speed_m_s"]
        for visit in action["junction_visits"]:
            births.setdefault(visit["node"], clock)
        clock += config["process"]["start_lag_s"] + config["process"]["stop_lag_s"]
        write_json(destination, report)
    report.update(
        status="SAMPLED_SCREEN_COMPLETE"
        if len(report["frames"]) == report["requested_frames"]
        else "PARTIAL_TIME_BUDGET",
        checked_frames=len(report["frames"]),
        released_counts=dict(Counter(row["released"]["status"] for row in report["frames"])),
        held_counts=dict(Counter(row["held"]["status"] for row in report["frames"] if row["held"])),
    )
    write_json(destination, report)
    stage_update(
        run,
        "growth",
        "partial",
        artifact=destination.name,
        message=(
            f"{report['checked_frames']}/{report['requested_frames']} cold growth samples; "
            "physical validation pending"
        ),
    )
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--maximum-seconds", type=float, default=60)
    args = parser.parse_args()
    result = growth_run(args.run, args.maximum_seconds)
    print(
        {
            key: result[key]
            for key in (
                "status",
                "checked_frames",
                "requested_frames",
                "released_counts",
                "held_counts",
            )
        }
    )
