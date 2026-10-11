from pen_art.planning.resource_schedule import solve_allocation


def test_parallel_branches_preserve_cross_arm_support_and_cooling():
    tasks = [
        {"id": "post", "deposits": ["junction"], "cooling_s": 1},
        {"id": "independent", "deposits": [], "cooling_s": 0},
        {"id": "top", "deposits": [], "cooling_s": 0},
    ]
    modes = [
        {
            "id": f"{arm}/{task['id']}",
            "arm": arm,
            "task": task["id"],
            "variant": "fixed",
            "start_anchor": "junction" if task["id"] == "top" else "ground",
            "duration_s": 2 if task["id"] == "top" else 4,
        }
        for arm in ("left", "right")
        for task in tasks
    ]
    problem = {
        "tasks": tasks,
        "modes": modes,
        "ground_nodes": ["ground"],
        "initial_setup_s": {m["id"]: 0 for m in modes},
        "final_setup_s": {m["id"]: 0 for m in modes},
        "transitions": [
            {"from": a["id"], "to": b["id"], "duration_s": 0}
            for a in modes
            for b in modes
            if a["arm"] == b["arm"] and a["task"] != b["task"]
        ],
    }
    parallel = solve_allocation(problem, maximum_seconds=2)
    serial = solve_allocation(problem, enabled_arms=["left"], maximum_seconds=2)
    assert parallel["status"] == serial["status"] == "OPTIMAL"
    assert parallel["makespan_s"] == 7
    assert serial["makespan_s"] == 10
    actions = {a["stroke_id"]: a for a in parallel["actions"]}
    assert actions["top"]["start_s"] >= actions["post"]["end_s"] + 1
    assert actions["independent"]["start_s"] == actions["post"]["start_s"] == 0
    assert actions["independent"]["arm"] != actions["post"]["arm"]

    top = actions["top"]
    completed = sorted(a["stroke_id"] for a in parallel["actions"] if a["end_s"] <= top["start_s"])
    problem["forbidden_prefixes"] = [
        {
            "mode_id": top["mode_id"],
            "predecessor_mode": top["predecessor_mode"],
            "completed_tasks": completed,
        }
    ]
    repaired = solve_allocation(problem, maximum_seconds=2)
    replacement = next(a for a in repaired["actions"] if a["stroke_id"] == "top")
    replacement_prefix = sorted(
        a["stroke_id"] for a in repaired["actions"] if a["end_s"] <= replacement["start_s"]
    )
    assert repaired["status"] == "OPTIMAL"
    assert (replacement["mode_id"], replacement["predecessor_mode"], replacement_prefix) != (
        top["mode_id"],
        top["predecessor_mode"],
        completed,
    )
    assert repaired["makespan_s"] == 7  # Another arm assignment remains available.

    # Moving the independent stroke before the obstructing post must preserve
    # the top's support dependency, even though the schedule gets longer.
    problem["forbidden_prefixes"] = []
    problem["precedence_modes"] = [
        [a["id"], b["id"]]
        for a in modes
        for b in modes
        if a["task"] == "independent" and b["task"] == "post"
    ]
    ordered = solve_allocation(problem, maximum_seconds=2)
    assert ordered["status"] == "OPTIMAL"
    assert ordered["makespan_s"] == 11
    ordered_actions = {a["stroke_id"]: a for a in ordered["actions"]}
    assert ordered_actions["post"]["start_s"] >= ordered_actions["independent"]["end_s"]
    assert ordered_actions["top"]["start_s"] >= ordered_actions["post"]["end_s"] + 1
    # A contradictory material order cannot override a required support.
    problem["precedence_modes"] += [
        [a["id"], b["id"]]
        for a in modes
        for b in modes
        if a["task"] == "top" and b["task"] == "post"
    ]
    assert solve_allocation(problem, maximum_seconds=2)["status"] == "INFEASIBLE"


def test_material_obstructions_keep_preparation_and_replay_names():
    from pen_art.planning.allocation_repair import obstructing_material_modes

    modes = {("arm_1", "post"): "post-mode", ("arm_2", "base"): "base-mode"}
    for name in (
        "arm_1/post/36",
        "arm_1/post/36/0",
        "material/arm_1/post/36",
        "material/arm_1/post/36/0",
    ):
        witness = {
            "attempts": [
                {"status": "COLLISION", "pairs": [{"first": "tool/nozzle_shank", "second": name}]}
            ]
        }
        assert obstructing_material_modes(witness, modes) == ["post-mode"]
    assert (
        obstructing_material_modes(
            {"status": "TRACKING", "pairs": [{"second": "arm_1/post/36/0"}]}, modes
        )
        == []
    )
