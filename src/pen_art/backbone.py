"""Choose permanent start supports and an intact-contour construction order.

The finite solver selects from vertical posts at existing stroke endpoints and
orders existing strokes. It never cuts an artistic contour. The result is a
topological construction proposal; stiffness and deposition remain separate.
"""

import argparse
from copy import deepcopy
from pathlib import Path

import networkx as nx
import numpy as np
from ortools.sat.python import cp_model

from .construction import prefix_grounded
from .graph import straight_edge, topology
from .io import artifact, read_json, sha256, stage_update, write_json
from .process import screen_graph
from .strokes import orient_stroke, pen_strokes
from .structure import analyze_structure


def synthesize_backbone(graph, process, *, maximum_seconds=5):
    strokes = pen_strokes(graph)["curves"]
    if not strokes:
        raise ValueError("A nonempty stroke graph is required")
    nodes = {n["id"]: n for n in graph["nodes"]}
    if any(
        not np.isfinite(n["position"]).all() or n["position"][2] < -1e-9 for n in nodes.values()
    ):
        raise ValueError("Backbone input must be finite and above the build bed")
    ground = {key for key, node in nodes.items() if node["ground"]}
    if not ground:
        raise ValueError("A connected printable base is required")
    choices = {}
    providers = {}
    for stroke in strokes:
        visits = (
            stroke["junction_visits"][:-1]
            if stroke["closed"]
            else [stroke["junction_visits"][0], stroke["junction_visits"][-1]]
        )
        choices[stroke["id"]] = sorted({v["node"] for v in visits})
        for node in {v["node"] for v in stroke["junction_visits"]}:
            providers.setdefault(node, set()).add(stroke["id"])
    candidates = sorted({n for row in choices.values() for n in row} - ground)
    model = cp_model.CpModel()
    posts = {node: model.new_bool_var(f"post/{node}") for node in candidates}
    ranks = {s["id"]: model.new_int_var(0, len(strokes) - 1, f"order/{s['id']}") for s in strokes}
    selected = {}
    costs = {}
    for node in candidates:
        position = np.array(nodes[node]["position"])
        foot = np.r_[position[:2], 0.0]
        link = min(np.linalg.norm(foot - nodes[g]["position"]) for g in ground)
        # Positive tie cost discourages unnecessary posts at bed-level nodes.
        costs[node] = max(1, round((position[2] + link) * 1000000))
    for stroke in strokes:
        identity = stroke["id"]
        ways = []
        for anchor in choices[identity]:
            if anchor in ground:
                key = (identity, anchor, "bed")
                selected[key] = model.new_bool_var("/".join(key))
                ways.append(selected[key])
            else:
                key = (identity, anchor, "post")
                selected[key] = model.new_bool_var("/".join(key))
                model.add_implication(selected[key], posts[anchor])
                ways.append(selected[key])
            for previous in sorted(providers[anchor] - {identity}):
                key = (identity, anchor, previous)
                selected[key] = model.new_bool_var("/".join(key))
                model.add(ranks[identity] > ranks[previous]).only_enforce_if(selected[key])
                ways.append(selected[key])
        model.add_exactly_one(ways)
    model.minimize(sum(costs[n] * posts[n] for n in candidates))
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = maximum_seconds
    solver.parameters.num_search_workers = 8
    solver.parameters.random_seed = 0
    status = solver.solve(model)
    report = artifact(
        "backbone_synthesis",
        status=solver.status_name(status),
        simulation_only=True,
        executable=False,
        candidate_posts=len(candidates),
        original_strokes=len(strokes),
        solver_seconds=solver.wall_time,
        limitations=[
            "Start connectivity only; not a stiffness or hot-filament certificate",
            "Vertical posts and bed links require robot/process validation",
        ],
    )
    if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        return None, None, report
    chosen = [n for n in candidates if solver.boolean_value(posts[n])]
    result = deepcopy(graph)
    target_nodes = {n["id"]: n for n in result["nodes"]}
    root = min(ground)
    supports = []
    for index, node in enumerate(chosen):
        point = np.array(nodes[node]["position"])
        foot_point = np.r_[point[:2], 0.0]
        coincident = [
            key
            for key in ground
            if np.linalg.norm(foot_point - target_nodes[key]["position"]) < 1e-9
        ]
        at_bed = np.linalg.norm(point - foot_point) <= 1e-9
        foot = node if at_bed else min(coincident) if coincident else f"backbone-foot-{index:03}"
        if not coincident:
            if not at_bed:
                target_nodes[foot] = {
                    "id": foot,
                    "position": foot_point.tolist(),
                    "ground": True,
                    "join_status": "proposed",
                }
            nearest = min(
                ground, key=lambda g: np.linalg.norm(foot_point - target_nodes[g]["position"])
            )
            link = straight_edge(
                nearest,
                foot,
                target_nodes,
                f"backbone-base-{index:03}",
                "base",
                process["strand_diameter_m"],
            )
            supports.append(link)
            ground.add(foot)
        if np.linalg.norm(point - foot_point) > 1e-9:
            supports.append(
                straight_edge(
                    foot,
                    node,
                    target_nodes,
                    f"backbone-post-{index:03}",
                    "support",
                    process["strand_diameter_m"],
                )
            )
        else:
            # A bed-level original node is already a physical attachment.
            target_nodes[node]["ground"] = True
    result["nodes"] = list(target_nodes.values())
    result["edges"].extend(supports)
    selected_anchors = {
        key[0]: key[1] for key, lit in selected.items() if solver.boolean_value(lit)
    }
    ordered = sorted(strokes, key=lambda s: (solver.value(ranks[s["id"]]), s["id"]))
    actions = []
    for edge in supports:
        support_graph = {"edges": [edge]}
        action = pen_strokes(support_graph)["curves"][0]
        action["id"] = edge["id"]
        actions.append(action)
    for stroke in ordered:
        anchor = selected_anchors[stroke["id"]]
        if stroke["closed"]:
            index = next(
                i for i, v in enumerate(stroke["junction_visits"][:-1]) if v["node"] == anchor
            )
            action = orient_stroke(stroke, start_visit=index)
        else:
            action = orient_stroke(stroke, reverse=stroke["start"] != anchor)
        actions.append(action)
    printed, births, prefixes = set(), {}, []
    ground = {key for key, node in target_nodes.items() if node["ground"]}
    for index, action in enumerate(actions):
        if action["start"] not in ground | births.keys():
            raise ValueError(f"Backbone order has no start support: {action['id']}")
        action.update(
            index=index,
            source_stroke=action["id"],
            depends_on=[] if action["start"] in ground else [births[action["start"]]],
            existing_join_nodes=sorted(
                {v["node"] for v in action["junction_visits"]} & births.keys()
            ),
        )
        if printed.intersection(action["members"]):
            raise ValueError("A backbone order printed a member twice")
        printed.update(action["members"])
        if not prefix_grounded(result, printed):
            raise ValueError("Backbone construction has a floating prefix")
        prefixes.append(
            {"after_action": index, "printed_members": len(printed), "grounded_topology": True}
        )
        for node in action["junction_visits"]:
            births.setdefault(node["node"], index)
    if printed != {e["id"] for e in result["edges"]} or not nx.is_connected(topology(result)):
        raise ValueError("Backbone does not cover one connected material graph")
    extra = sum(e["length_m"] for e in supports)
    result["metrics"].update(
        material_members=len(result["edges"]),
        junctions=len(target_nodes),
        ground_contacts=len(ground),
        material_components=1,
        total_length_m=sum(e["length_m"] for e in result["edges"]),
        added_length_m=sum(e["length_m"] for e in result["edges"] if e["role"] != "artistic"),
        proposed_pen_strokes=len(actions),
    )
    construction = artifact(
        "construction_plan",
        frame="build",
        units="m",
        status="GROUNDED_INTACT_CONTOUR_ORDER",
        simulation_only=True,
        actions=actions,
        prefixes=prefixes,
        states=int(solver.num_branches),
        all_members_covered_once=True,
        total_length_m=result["metrics"]["total_length_m"],
        continuous_strokes=len(actions),
        robot_feasible=None,
        pen_feasible=None,
        stiffness_feasible=None,
    )
    report.update(
        selected_posts=chosen,
        added_members=[e["id"] for e in supports],
        added_length_m=extra,
        added_drawing_time_s=extra / process["nominal_speed_m_s"],
        complete_actions=len(actions),
        whole_contours_preserved=True,
        root_ground_node=root,
        objective="Minimum estimated added support length over a fixed endpoint-post catalogue",
        objective_limit="Bed links costed to original ground; realized links may share new feet",
    )
    result.setdefault("checks", {}).update(structure_feasible=None, process_feasible=None)
    return result, construction, report


def backbone_run(run, maximum_seconds=5):
    run = Path(run)
    config = read_json(run / "fabrication_config.json")
    graph, construction, report = synthesize_backbone(
        read_json(run / "stroke_graph.json"), config["process"], maximum_seconds=maximum_seconds
    )
    report["inputs"] = {
        name: sha256(run / name) for name in ("stroke_graph.json", "fabrication_config.json")
    }
    if graph:
        graph["provenance"] = {**graph.get("provenance", {}), "backbone_inputs": report["inputs"]}
        write_json(run / "backbone_graph.json", graph)
        write_json(run / "backbone_pen_strokes.json", pen_strokes(graph))
        write_json(
            run / "backbone_process.json",
            screen_graph(graph, config["process"], pen_strokes(graph)),
        )
        construction["inputs"] = {"backbone_graph.json": sha256(run / "backbone_graph.json")}
        write_json(run / "backbone_construction.json", construction)
        screen = analyze_structure(graph, config["structure"])
        write_json(run / "backbone_structure.json", screen)
        report["structure_status"] = screen.get(
            "failure", "MODELED_PASS" if screen["modeled_pass"] else "MODELED_LIMIT_EXCEEDED"
        )
    write_json(run / "backbone_report.json", report)
    stage_update(
        run,
        "backbone",
        "partial",
        artifact="backbone_report.json",
        message="Intact-contour support proposal; physical validation pending",
    )
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--maximum-seconds", type=float, default=5)
    args = parser.parse_args()
    result = backbone_run(args.run, args.maximum_seconds)
    print({key: value for key, value in result.items() if key not in {"inputs", "limitations"}})
