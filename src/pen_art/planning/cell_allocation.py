"""Build and solve a fixed motion-catalogue allocation for the shared cell."""

import argparse
from pathlib import Path
from time import monotonic

import numpy as np
import yaml

from ..cell import load_cell
from ..io import artifact, read_json, sha256, stage_update, write_json
from .clock_path import timed_path
from .construction_order import ground_base_actions
from .dual_arm import tasks_from_construction
from .resource_schedule import solve_allocation


def build_problem(run, profile_path, cooling_s=2.0, deadline=None, timing_path=None):
    run = Path(run)
    profile, arms, _ = load_cell(profile_path)
    if profile != read_json(run / "cell_config.json"):
        raise ValueError("Cell profile changed; rebuild the setup")
    cell = read_json(run / "cell_manifest.json")
    for name, digest in cell["inputs"].items():
        if sha256(run / name) != digest:
            raise ValueError(f"Stale cell input: {name}")
    for identity, arm in arms.items():
        for suffix, live in (("robot.urdf", arm.urdf), ("robot.json", arm.manifest_path)):
            if sha256(live) != sha256(run / f"{identity}_{suffix}"):
                raise ValueError(f"Live robot changed: {identity}")
    candidates = read_json(run / "cell_drawing_variants.json")
    for name, digest in candidates["inputs"].items():
        if sha256(run / name) != digest:
            raise ValueError(f"Stale drawing input: {name}")
    source = Path(read_json(run / "run.json")["cell_source_run"])
    process = read_json(source / "fabrication_config.json")["process"]
    timing = yaml.safe_load(Path(timing_path or "configs/timing.yaml").read_text())
    construction = read_json(run / "construction_plan.json")
    tasks, ground = tasks_from_construction(
        construction, read_json(run / "stroke_graph.json"), cooling_s
    )
    # Reuse prior staging when available. A new model need not run the old
    # event-search scheduler: initialize its cell/staging once instead.
    prior_path = run / "dual_arm_plan.json"
    if prior_path.exists():
        prior = read_json(prior_path)
        for name in (
            "cell_manifest.json",
            "cell_drawing_variants.json",
            "construction_plan.json",
            "stroke_graph.json",
        ):
            if prior["inputs"][name] != sha256(run / name):
                raise ValueError(f"Staged-pose source uses different input: {name}")
        initial = prior["initial_positions"]
    else:
        from .cell_planner import CellPlanner

        settings = yaml.safe_load(Path("configs/dual_arm.yaml").read_text())
        settings["assumed_cooling_s"] = cooling_s
        setup = CellPlanner(
            run,
            profile_path,
            settings,
            min(monotonic() + 30, deadline) if deadline is not None else monotonic() + 30,
            timing_override=timing,
        )
        initial = {identity: arm.initial_positions.tolist() for identity, arm in setup.arms.items()}
    config = {
        "profile": profile,
        "process": process,
        "timing": timing,
        "initial_positions": initial,
        "cooling_s": cooling_s,
        "clock_revision": (
            4
            if timing.get("acceleration_constraint") == "cubic_bernstein"
            else 3
            if timing.get("drawing_clock") == "cubic_toppra"
            else 2
        ),
        "transfer_model": "Direct endpoint joint/TCP estimate; approach and withdrawal "
        "are added during selected-motion realization",
    }
    graph = read_json(run / "stroke_graph.json")
    bases = ground_base_actions(construction, graph)
    config["ground_base_first"] = {
        "revision": 1,
        "actions": sorted(bases),
        "policy": "Explicit substrate base strokes before elevated supports and artistic contours",
    }
    scaffold = set()
    if read_json(run / "run.json").get("cell_source_variant") == "backbone":
        report = read_json(source / "backbone_report.json")
        scaffold = set(report["added_members"])
        config["support_first"] = {
            "source_report_sha256": sha256(source / "backbone_report.json"),
            "members": sorted(scaffold),
            "policy": "Complete automatically added start supports before main contours",
        }
    config_path = run / "allocation_config.json"
    if not config_path.exists() or read_json(config_path) != config:
        write_json(config_path, config)
    inputs = {
        name: sha256(run / name)
        for name in (
            "cell_manifest.json",
            "cell_drawing_variants.json",
            "construction_plan.json",
            "stroke_graph.json",
            "allocation_config.json",
        )
    }
    path = run / "allocation_problem.json"
    if path.exists() and read_json(path).get("inputs") == inputs:
        return read_json(path)
    clock_file = run / "allocation_timing.json"
    clock_report = artifact(
        "allocation_timing",
        status="RUNNING",
        revision=1,
        inputs=inputs,
        simulation_only=True,
        executable=False,
        requested_modes=len(candidates["variants"]),
        modes=[],
        scope="Timing eligibility of fixed drawing modes, not full motion feasibility",
    )
    if clock_file.exists():
        previous = read_json(clock_file)
        if previous.get("inputs") == inputs and previous.get("revision") == 1:
            clock_report["modes"] = previous["modes"]
    clocks = {row["mode_id"]: row for row in clock_report["modes"]}
    write_json(clock_file, clock_report)
    modes, positions = [], {}
    actions = {a["id"]: a for a in construction["actions"]}
    for row in candidates["variants"]:
        if deadline is not None and monotonic() >= deadline:
            raise TimeoutError("Allocation input preparation exceeded its planning budget")
        action = actions[row["stroke_id"]]
        if row["arm"] not in action.get("allowed_arms", list(arms)):
            continue
        arm = arms[row["arm"]]
        key = f"{row['arm']}/{row['stroke_id']}/{row['variant']}"
        q = np.asarray(row["positions"])
        if key not in clocks:
            clock_report["active_mode"] = key
            write_json(clock_file, clock_report)
            try:
                drawing = timed_path(
                    [{"kind": "DRAW", "positions": q}],
                    arm,
                    arm.robot.velocity * timing["velocity_fraction"],
                    np.asarray(timing["joint_acceleration_rad_s2"]),
                    process["nominal_speed_m_s"],
                    timing["travel_speed_m_s"],
                    drawing_clock=timing.get("drawing_clock", "quintic_stop_at_geometric_knots"),
                    acceleration_constraint=timing.get("acceleration_constraint", "interpolated"),
                )
            except ValueError as error:
                if str(error) not in {
                    "No finite path time parameterization found",
                    "Retimer produced a stationary interval with nonzero length",
                    "Retiming correction is numerically ill-conditioned",
                }:
                    raise
                clocks[key] = {
                    "mode_id": key,
                    "arm": row["arm"],
                    "stroke_id": row["stroke_id"],
                    "status": "TIMING_UNRESOLVED",
                    "reason": str(error),
                }
                print(f"Timing unresolved for {key}: {error}", flush=True)
            else:
                clocks[key] = {
                    "mode_id": key,
                    "arm": row["arm"],
                    "stroke_id": row["stroke_id"],
                    "status": "TIMED",
                    "drawing_s": drawing.duration_s,
                    "clock_diagnostics": drawing.metadata.get("drawing_timing"),
                }
            clock_report["modes"] = list(clocks.values())
            clock_report.pop("active_mode", None)
            write_json(clock_file, clock_report)
        clock = clocks[key]
        if clock["status"] != "TIMED":
            continue
        modes.append(
            {
                "id": key,
                "arm": row["arm"],
                "task": row["stroke_id"],
                "variant": row["variant"],
                "start_anchor": row["start_anchor"],
                "work_zone": action.get("work_zone"),
                "allocation_group": action.get("allocation_group"),
                "reserve_arms": list(arms) if action.get("work_zone") == "seam" else [],
                "drawing_s": clock["drawing_s"],
                "clock_diagnostics": clock["clock_diagnostics"],
                "duration_s": clock["drawing_s"]
                + sum(process[k] for k in ("join_s", "stop_lag_s", "start_lag_s")),
            }
        )
        positions[key] = (q[0], q[-1])

    clock_report["status"] = (
        "FIXED_MODES_TIMED"
        if all(row["status"] == "TIMED" for row in clocks.values())
        else "SOME_MODE_TIMING_UNRESOLVED"
    )
    write_json(clock_file, clock_report)

    def transfer(identity, a, b):
        arm = arms[identity]
        dq = abs(np.asarray(b) - np.asarray(a))
        distance = np.linalg.norm(
            arm.robot.forward(a, arm.flange_to_tcp)[:3, 3]
            - arm.robot.forward(b, arm.flange_to_tcp)[:3, 3]
        )
        return float(
            max(
                np.max(1.875 * dq / (arm.robot.velocity * timing["velocity_fraction"])),
                np.sqrt(
                    np.max(10 / np.sqrt(3) * dq / np.asarray(timing["joint_acceleration_rad_s2"]))
                ),
                1.875 * distance / timing["travel_speed_m_s"],
            )
        )

    problem = artifact(
        "resource_allocation_problem",
        inputs=inputs,
        simulation_only=True,
        tasks=[
            {
                "id": t.identity,
                "deposits": sorted(t.deposits),
                "cooling_s": t.cooling_s,
                "phase": 0 if t.identity in bases else (
                    1 if scaffold and set(actions[t.identity]["members"]) <= scaffold else 2
                ),
            }
            for t in tasks
        ],
        ground_nodes=sorted(ground),
        modes=modes,
        initial_setup_s={
            m["id"]: transfer(m["arm"], initial[m["arm"]], positions[m["id"]][0]) for m in modes
        },
        final_setup_s={
            m["id"]: transfer(m["arm"], positions[m["id"]][1], initial[m["arm"]]) for m in modes
        },
        transitions=[
            {
                "from": a["id"],
                "to": b["id"],
                "duration_s": transfer(a["arm"], positions[a["id"]][1], positions[b["id"]][0]),
            }
            for a in modes
            for b in modes
            if a["arm"] == b["arm"] and a["task"] != b["task"]
        ],
        exclusive_modes=[],
        forbidden_transitions=[],
        forbidden_modes=[],
        limitations=[
            "Transfer durations are estimates until selected motions are constructed",
            "No inter-arm or growing-material feasibility is implied by allocation",
            "Anchor/cooling readiness is encoded; intermediate stiffness is not certified",
            "Pen and Flexiv process parameters remain uncalibrated",
        ],
    )
    write_json(path, problem)
    return problem


def allocate_cell(
    run, profile_path, maximum_seconds=15, maximum_total_seconds=None, timing_path=None
):
    run = Path(run)
    deadline = None if maximum_total_seconds is None else monotonic() + maximum_total_seconds
    problem = build_problem(run, profile_path, deadline=deadline, timing_path=timing_path)
    results = {}
    arms = sorted(read_json(run / "allocation_config.json")["initial_positions"])
    for name, enabled in (("dual", arms), ("single", arms[:1])):
        if name == "single" and len(arms) == 1:
            results[name] = dict(results["dual"])
            continue
        missing = [
            task["id"]
            for task in problem["tasks"]
            if not any(m["task"] == task["id"] and m["arm"] in enabled for m in problem["modes"])
        ]
        if missing:
            results[name] = {
                "status": "NO_ELIGIBLE_MOTION_MODE",
                "enabled_arms": enabled,
                "missing_strokes": missing,
                "actions": [],
                "makespan_s": None,
                "objective_bound_s": None,
                "wall_time_s": 0.0,
            }
        else:
            remaining = (
                maximum_seconds
                if deadline is None
                else min(maximum_seconds, deadline - monotonic())
            )
            if remaining <= 0:
                raise TimeoutError("Allocation exceeded its planning budget")
            results[name] = solve_allocation(
                problem, enabled_arms=enabled, maximum_seconds=remaining
            )
        label = "single" if len(arms) == 1 else name
        print(f"{label}: {results[name]['status']}, {results[name]['makespan_s']} s", flush=True)
    single, dual = results["single"]["makespan_s"], results["dual"]["makespan_s"]
    output = artifact(
        "cell_allocation",
        status="ALLOCATION_ONLY",
        executable=False,
        simulation_only=True,
        inputs={"allocation_problem.json": sha256(run / "allocation_problem.json")},
        fixed_mode_count=len(problem["modes"]),
        requested_strokes=len(problem["tasks"]),
        results=results,
        estimated_speedup=single / dual if len(arms) == 2 and single and dual else None,
        comparison_scope=(
            "Single robot; no two-arm comparison" if len(arms) == 1 else
            "Same catalogue and estimated transfer costs; not a physical speedup"
        ),
        limitations=problem["limitations"],
    )
    write_json(run / "cell_allocation.json", output)
    stage_update(
        run,
        "cell",
        "partial",
        artifact="cell_allocation.json",
        message="Offline CP-SAT allocation; selected-motion replay pending",
    )
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, default=Path("runs/palace_15cm_dual"))
    parser.add_argument("--config", type=Path, default=Path("configs/cell/two_flexiv.yaml"))
    parser.add_argument("--maximum-seconds", type=float, default=15)
    parser.add_argument("--timing-config", type=Path)
    args = parser.parse_args()
    allocate_cell(args.run, args.config, args.maximum_seconds, timing_path=args.timing_config)
