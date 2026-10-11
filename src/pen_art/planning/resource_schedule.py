"""Offline two-resource scheduling over a fixed motion catalogue.

CP-SAT chooses assignment, anchor and order. It does not generate robot paths or
certify geometry. Transition costs include the incoming pen-off motion, so they
occupy the same arm as deposition. Times are rounded upward to milliseconds.
"""

from math import ceil, isfinite

from ortools.sat.python import cp_model


def solve_allocation(problem, *, enabled_arms=None, maximum_seconds=15):
    tasks = {t["id"]: t for t in problem["tasks"]}
    modes = {
        m["id"]: m for m in problem["modes"] if enabled_arms is None or m["arm"] in enabled_arms
    }
    if len(tasks) != len(problem["tasks"]) or len({m["id"] for m in problem["modes"]}) != len(
        problem["modes"]
    ):
        raise ValueError("Task and mode IDs must be unique")
    if not tasks or any(not any(m["task"] == t for m in modes.values()) for t in tasks):
        raise ValueError("Every task needs at least one eligible motion mode")

    def ticks(value):
        if not isfinite(value) or value < 0:
            raise ValueError("Durations must be finite and nonnegative")
        return ceil(value * 1000)

    base = {key: max(1, ticks(m["duration_s"])) for key, m in modes.items()}
    initial = {key: ticks(problem["initial_setup_s"][key]) for key in modes}
    final = {key: ticks(problem["final_setup_s"][key]) for key in modes}
    transitions = {
        (r["from"], r["to"]): ticks(r["duration_s"])
        for r in problem["transitions"]
        if r["from"] in modes and r["to"] in modes
    }
    cooling = {key: ticks(t["cooling_s"]) for key, t in tasks.items()}
    horizon = (
        sum(base.values())
        + len(tasks) * (max([0, *initial.values(), *transitions.values()]) + max(cooling.values()))
        + max(final.values())
    )
    model = cp_model.CpModel()
    present = {key: model.new_bool_var(f"use/{key}") for key in modes}
    start = {key: model.new_int_var(0, horizon, f"start/{key}") for key in modes}
    end = {key: model.new_int_var(0, horizon, f"end/{key}") for key in modes}
    duration = {key: model.new_int_var(0, horizon, f"duration/{key}") for key in modes}
    finish = model.new_int_var(0, horizon, "makespan")
    incoming, arcs = {key: [] for key in modes}, {}
    arm_ids = sorted({m["arm"] for m in modes.values()})
    occupied = {arm: [] for arm in arm_ids}
    for arm in arm_ids:
        keys = [key for key, m in modes.items() if m["arm"] == arm]
        indices = {key: i + 1 for i, key in enumerate(keys)}
        empty = model.new_bool_var(f"empty/{arm}")
        circuit = [(0, 0, empty)]
        model.add(sum(present[k] for k in keys) == 0).only_enforce_if(empty)
        model.add(sum(present[k] for k in keys) >= 1).only_enforce_if(empty.Not())
        for key in keys:
            index = indices[key]
            circuit.append((index, index, present[key].Not()))
            first = model.new_bool_var(f"first/{key}")
            last = model.new_bool_var(f"last/{key}")
            circuit.extend([(0, index, first), (index, 0, last)])
            arcs[(None, key)], arcs[(key, None)] = first, last
            incoming[key].append(initial[key] * first)
            model.add(finish >= end[key] + final[key]).only_enforce_if(last)
            interval = model.new_optional_interval_var(
                start[key], duration[key], end[key], present[key], f"interval/{key}"
            )
            for resource in {arm, *modes[key].get("reserve_arms", [])}:
                if resource in occupied:
                    occupied[resource].append(interval)
            model.add(start[key] == 0).only_enforce_if(present[key].Not())
            model.add(end[key] == 0).only_enforce_if(present[key].Not())
            model.add(finish >= end[key] + cooling[modes[key]["task"]]).only_enforce_if(
                present[key]
            )
        for (a, b), cost in transitions.items():
            if a not in indices or b not in indices or modes[a]["task"] == modes[b]["task"]:
                continue
            follows = model.new_bool_var(f"next/{a}/{b}")
            circuit.append((indices[a], indices[b], follows))
            arcs[(a, b)] = follows
            incoming[b].append(cost * follows)
            model.add(start[b] >= end[a]).only_enforce_if(follows)
        model.add_circuit(circuit)
    for intervals in occupied.values():
        model.add_no_overlap(intervals)
    for key in modes:
        model.add(duration[key] == base[key] * present[key] + sum(incoming[key]))
    for task in tasks:
        model.add_exactly_one(present[k] for k, m in modes.items() if m["task"] == task)
    # Optional construction phases impose a small number of global barriers.
    # For supported sketches, all starting posts precede the main contours.
    phases = {key: task.get("phase", 0) for key, task in tasks.items()}
    if any(not isinstance(value, int) or value < 0 for value in phases.values()):
        raise ValueError("Construction phases must be nonnegative integers")
    for phase in sorted(set(phases.values()))[:-1]:
        boundary = model.new_int_var(0, horizon, f"phase/{phase}/end")
        for key, mode in modes.items():
            if phases[mode["task"]] <= phase:
                model.add(boundary >= end[key] + cooling[mode["task"]]).only_enforce_if(
                    present[key]
                )
            else:
                model.add(start[key] >= boundary).only_enforce_if(present[key])
    # Connected structural units keep one owner, even if several pen strokes
    # are needed to fabricate the unit. Do not split a ring between the arms.
    for group in sorted(
        {m["allocation_group"] for m in modes.values() if m.get("allocation_group")}
    ):
        members = [key for key, m in modes.items() if m.get("allocation_group") == group]
        owners = {
            arm: model.new_bool_var(f"owner/{group}/{arm}")
            for arm in sorted({modes[key]["arm"] for key in members})
        }
        model.add_exactly_one(owners.values())
        for key in members:
            model.add_implication(present[key], owners[modes[key]["arm"]])

    ground = set(problem["ground_nodes"])
    supports = {}
    for key, mode in modes.items():
        if mode["start_anchor"] in ground:
            continue
        choices = []
        for other, provider in modes.items():
            if (
                provider["task"] == mode["task"]
                or mode["start_anchor"] not in tasks[provider["task"]]["deposits"]
            ):
                continue
            support = model.new_bool_var(f"support/{other}/{key}")
            model.add_implication(support, present[key])
            model.add_implication(support, present[other])
            model.add(start[key] >= end[other] + cooling[provider["task"]]).only_enforce_if(support)
            choices.append(support)
            supports[(other, key)] = support
        model.add_bool_or([*choices, present[key].Not()])
    for a, b in problem.get("exclusive_modes", []):
        if a in modes and b in modes:
            before = model.new_bool_var(f"exclusive/{a}/{b}")
            model.add(end[a] <= start[b]).only_enforce_if([present[a], present[b], before])
            model.add(end[b] <= start[a]).only_enforce_if([present[a], present[b], before.Not()])
    # A fixed path may need to finish before an obstructing strand exists.
    # Anchor/cooling constraints above still apply; contradictions are infeasible.
    for a, b in problem.get("precedence_modes", []):
        if a not in modes or b not in modes or a == b:
            raise ValueError("Material precedence must name distinct existing modes")
        model.add(end[a] <= start[b]).only_enforce_if([present[a], present[b]])
    for a, b in problem.get("forbidden_transitions", []):
        if (a, b) in arcs:
            model.add(arcs[(a, b)] == 0)
    for key in problem.get("forbidden_modes", []):
        if key in present:
            model.add(present[key] == 0)
    # A failed approach is contextual: the same incoming path may work before
    # different material is printed. Exclude only its exact completed-task prefix.
    for index, failure in enumerate(problem.get("forbidden_prefixes", [])):
        key = failure["mode_id"]
        incoming_arc = (failure["predecessor_mode"], key)
        if incoming_arc not in arcs:
            continue
        completed = set(failure["completed_tasks"])
        differs = [arcs[incoming_arc].Not()]
        for task in tasks:
            if task == modes[key]["task"]:
                continue
            before = []
            for other, mode in modes.items():
                if mode["task"] != task:
                    continue
                earlier = model.new_bool_var(f"prefix/{index}/{other}")
                model.add_implication(earlier, present[other])
                model.add(end[other] <= start[key]).only_enforce_if(earlier)
                model.add(end[other] >= start[key] + 1).only_enforce_if(
                    [present[other], earlier.Not()]
                )
                before.append(earlier)
            task_before = model.new_bool_var(f"prefix/{index}/task/{task}")
            model.add(sum(before) == task_before)
            differs.append(task_before.Not() if task in completed else task_before)
        model.add_bool_or(differs)
    model.minimize(finish)
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = maximum_seconds
    solver.parameters.num_search_workers = 8
    solver.parameters.random_seed = 0
    status = solver.solve(model)
    result = {
        "solver": "OR-Tools CP-SAT",
        "status": solver.status_name(status),
        "wall_time_s": solver.wall_time,
        "enabled_arms": arm_ids,
        "bound_scope": "Fixed catalogue, supplied duration estimates and encoded constraints only",
        "actions": [],
        "makespan_s": None,
        "objective_bound_s": None,
    }
    if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        return result
    result.update(
        makespan_s=solver.objective_value / 1000,
        objective_bound_s=solver.best_objective_bound / 1000,
    )
    for key, mode in modes.items():
        if not solver.boolean_value(present[key]):
            continue
        previous = next(a for (a, b), lit in arcs.items() if b == key and solver.boolean_value(lit))
        providers = [
            a for (a, b), lit in supports.items() if b == key and solver.boolean_value(lit)
        ]
        result["actions"].append(
            {
                "mode_id": key,
                "stroke_id": mode["task"],
                "arm": mode["arm"],
                "variant": mode["variant"],
                "start_anchor": mode["start_anchor"],
                "predecessor_mode": previous,
                "support_modes": providers,
                "start_s": solver.value(start[key]) / 1000,
                "end_s": solver.value(end[key]) / 1000,
                "drawing_s": mode.get("drawing_s"),
            }
        )
    result["actions"].sort(key=lambda r: (r["start_s"], r["arm"]))
    return result
