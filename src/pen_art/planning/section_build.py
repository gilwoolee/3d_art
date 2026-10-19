"""Build source sections and save clocks for separate verification and playback."""

import argparse
import shutil
from dataclasses import replace
from pathlib import Path
from time import monotonic

import networkx as nx
import numpy as np
import yaml

from ..assets import prepare_asset
from ..cell import prepare_cell
from ..curves import curve_length, flatten_curve, transform_points
from ..graph import topology
from ..io import artifact, now, read_json, sha256, stage_update, write_json
from ..process import screen_graph
from ..strokes import orient_stroke
from ..structure import analyze_member_prefixes, analyze_structure
from .cell_planner import CellPlanner
from .clock_path import ClockPath, timed_path
from .dual_arm import ScheduleState
from .scene import MaterialScene
from .sections import section_actions


def write_section_artifacts(run, graph, construction, *, preserve_graph=False):
    """Expose source sections and honest provisional screens to the workbench.

    Existing certified runs may add these views without changing their bound
    graph or construction inputs.
    """
    config = read_json(run / "fabrication_config.json")
    method = graph.get("source_method", graph.get("method", "reference_fixture"))
    label = "Reference fixture" if method == "reference_fixture" else "WirePrint framework"
    if any(row["stage"] == "artistic" for row in construction["actions"]):
        label += (
            " + WIR3D detail" if method == "wireprint_with_imported_wir3d" else " + artistic detail"
        )
    strokes = artifact(
        "pen_strokes",
        units="m",
        frame="build",
        method=method,
        label=label,
        curves=construction["actions"],
        total_length_m=sum(row["length_m"] for row in construction["actions"]),
        inputs={"construction_plan.json": sha256(run / "construction_plan.json")},
    )
    if not preserve_graph:
        graph["source_method"] = method
        graph["metrics"] = {
            "input_curves": len({row["parent_curve"] for row in construction["actions"]}),
            "material_members": len(graph["edges"]),
            "proposed_pen_strokes": len(construction["actions"]),
            "material_components": nx.number_connected_components(topology(graph)),
            "ground_contacts": sum(row["ground"] for row in graph["nodes"]),
            "total_length_m": sum(curve_length(row) for row in graph["edges"]),
            "added_length_m": sum(
                curve_length(row)
                for row in graph["edges"]
                if row.get("origin")
                in (
                    "explicit_style_attachment",
                    "explicit_component_connector",
                    "explicit_bed_support",
                )
            ),
        }
        write_json(run / "stroke_graph.json", graph)
    inputs = {name: sha256(run / name) for name in ("stroke_graph.json", "fabrication_config.json")}
    write_json(run / "pen_strokes.json", strokes)
    write_json(run / "geometric_candidates.json", {**strokes, "kind": "curve_candidates"})
    structure = analyze_structure(graph, config["structure"])
    write_json(run / "structural_screen.json", {**structure, "inputs": inputs})
    process = screen_graph(graph, config["process"], strokes)
    write_json(run / "process_screen.json", {**process, "inputs": inputs})
    stage_update(run, "geometric", "complete", artifact="geometric_candidates.json")
    stage_update(run, "graph", "complete", artifact="stroke_graph.json")


def prepare_candidates(run):
    write_json(
        run / "cell_drawing_variants.json",
        artifact(
            "cell_drawing_variants",
            inputs={
                name: sha256(run / name)
                for name in ("cell_manifest.json", "construction_plan.json", "stroke_graph.json")
            },
            variants=[],
            status="LAZY_SECTION_CANDIDATES",
        ),
    )


def _sag(screen):
    return {
        "modeled_pass": screen["modeled_pass"],
        "maximum_displacement_m": screen.get("maximum_displacement_m"),
    }


def prepare_framework(source, run, profile):
    from ..abstraction.wireprint import (
        connect_framework_islands,
        eulerize_frame,
        generate,
        reinforce_base,
    )

    if (run / "run.json").exists():
        return
    if not (source / "fabrication_config.json").exists():
        write_json(
            source / "fabrication_config.json",
            yaml.safe_load(Path("configs/fabrication.yaml").read_text()),
        )
    fabrication = read_json(source / "fabrication_config.json")
    material = fabrication["structure"]

    def framework(**settings):
        graph = eulerize_frame(
            connect_framework_islands(generate(source, **settings)),
            fabrication.get("wireprint", {}).get("maximum_wire_m", float("inf")),
            material,
        )
        screen = analyze_structure(graph, material)
        if not screen["modeled_pass"] and "maximum_displacement_m" in screen:
            graph = reinforce_base(
                graph,
                {
                    "initial_maximum_displacement_m": screen["maximum_displacement_m"],
                    "initial_maximum_stress_pa": screen["maximum_stress_pa"],
                },
            )
            screen = analyze_structure(graph, material)
        return graph, screen

    graph, screen = framework()
    # A frame that still fails is regenerated with each heavier strand the pen
    # can lay, lightest first. The strand diameter is also the wire's collision
    # size. Nothing passing leaves the original for the planner to reject.
    tried = [{"strand_diameter_m": graph["generation"]["strand_diameter_m"], **_sag(screen)}]
    for diameter in fabrication.get("wireprint", {}).get("heavier_strands_m", []):
        if screen["modeled_pass"]:
            break
        heavier, screen = framework(diameter=diameter)
        tried.append({"strand_diameter_m": diameter, **_sag(screen)})
        if screen["modeled_pass"]:
            graph = heavier
    if len(tried) > 1:
        if not screen["modeled_pass"]:
            graph, _ = framework()
        graph["strand_escalation"] = tried
    write_json(source / "framework_graph.json", graph)
    construction = section_actions(graph, group_sources=True)
    write_json(source / "stroke_graph.json", graph)
    write_json(source / "construction_plan.json", construction)
    write_section_artifacts(source, graph, construction)
    stage_update(
        source,
        "construction",
        "partial",
        artifact="construction_plan.json",
        message="Generated source sections; order and motion pending",
    )
    prepare_cell(profile, run, source)
    prepare_candidates(run)


def prepare_mesh(mesh, run, source_up, max_extent_m):
    source = run.with_name(run.name + "_source")
    if (source / "asset_manifest.json").exists():
        manifest = read_json(source / "asset_manifest.json")
        if (
            manifest["source"]["sha256"] != sha256(mesh)
            or manifest["max_extent_m"] != max_extent_m
            or manifest["source_up"] != source_up
        ):
            raise ValueError("Prepared mesh inputs changed; choose a fresh run")
        return source
    source.mkdir(parents=True, exist_ok=True)
    write_json(
        source / "run.json",
        {
            "id": source.name,
            "name": mesh.stem + " WirePrint",
            "created_at": now(),
            "updated_at": now(),
            "source_sha256": sha256(mesh),
            "stages": {},
        },
    )
    prepare_asset(
        mesh,
        source,
        {"source_up": source_up, "max_extent_m": max_extent_m, "geometry_faces": 80000},
    )
    stage_update(source, "asset", "complete", artifact="asset_manifest.json")
    return source


def prepare_mesh_within_wire(mesh, run, source_up, max_extent_m, fabrication=None):
    """Prepare a mesh, shrunk until its wireframe fits ``wireprint.maximum_wire_m``.

    Wire grows with about the square of the size, so each try scales the
    longest side by the square root of the overshoot. The size settled on and
    the wire at each try are written to ``wire_budget.json``.
    """
    from ..abstraction.wireprint import connect_framework_islands, eulerize_frame, generate

    source = run.with_name(run.name + "_source")
    record = source / "wire_budget.json"
    if record.exists():
        saved = read_json(record)
        if saved["requested_max_extent_m"] != max_extent_m:
            raise ValueError("Prepared mesh inputs changed; choose a fresh run")
        return prepare_mesh(mesh, run, source_up, saved["max_extent_m"])
    if fabrication is None:
        fabrication = yaml.safe_load(Path("configs/fabrication.yaml").read_text())
    budget = fabrication.get("wireprint", {}).get("maximum_wire_m")
    extent, tries = max_extent_m, []
    while True:
        source = prepare_mesh(mesh, run, source_up, extent)
        write_json(source / "fabrication_config.json", fabrication)
        if budget is None:
            break
        # Measured with its connector wire, so the frame that fits keeps them.
        graph = eulerize_frame(connect_framework_islands(generate(source)), float("inf"))
        wire = sum(edge["length_m"] for edge in graph["edges"])
        tries.append({"max_extent_m": extent, "wire_m": wire})
        if wire <= budget or len(tries) == 6:
            break
        shutil.rmtree(source)
        extent *= 0.99 * (budget / wire) ** 0.5
    write_json(
        record,
        {
            "requested_max_extent_m": max_extent_m,
            "max_extent_m": extent,
            "maximum_wire_m": budget,
            "within_budget": budget is None or tries[-1]["wire_m"] <= budget,
            "tries": tries,
        },
    )
    return source


class SectionBuild:
    def __init__(self, run, profile, maximum_seconds=3600):
        """Geometry-first planning: paths are checked untimed and timed once the
        whole chain passes. Wire laid earlier is described by merged capsules
        (radius grown by at most 20 micrometres), each straight member is one
        path piece with the pen checked against wire as a row of spheres every
        10 mm, clock knots sit at piece ends and along the ramps, and a stroke
        that starts where the previous one ended is drawn without lifting."""
        from .pen_spheres import PEN_SPHERES_REVISION

        self.run, self.profile = Path(run), Path(profile)
        settings = yaml.safe_load(Path("configs/dual_arm.yaml").read_text())
        settings.update(
            maximum_subdivision_depth=20,
            collision_scope="pen_candidate",
            merge_deposits_m=20e-6,
            path_pieces="members",
            pen_check=PEN_SPHERES_REVISION,
            pen_sample_m=0.010,
            chain_strokes=True,
        )
        timing = yaml.safe_load(Path("configs/timing_continuous_bounded.yaml").read_text())
        timing.pop("target_seconds", None)
        # The scanned clock already holds the tip under its speed limit over
        # every interval, by the tangent bound it is rescaled against. The
        # audit therefore checks joint limits and does not bound drawing tip
        # speed a second time (decided 2026-10-13).
        timing.update(
            retiming_grid_ramps=True,
            retiming_grid_samples=0,
            retiming_grid_midpoints=False,
            retiming_solver="scan",
            audit_drawing_tip_speed=False,
        )
        self.planner = CellPlanner(
            run, profile, settings, monotonic() + maximum_seconds, timing_override=timing
        )
        self.planner.geometry_first = True
        self.planner.adaptation_waypoint_checks = False
        if len(self.planner.arms) != 1:
            raise ValueError("Section builds require exactly one arm")
        self.planner.transit["rrt_maximum_seconds"] = 30.0
        self.arm = next(iter(self.planner.arms.values()))
        self.initial = self.arm.initial_positions.copy()
        self.actions = self.planner.actions
        self.material = read_json(self.run / "fabrication_config.json")["structure"]
        self.transform = np.asarray(self.planner.profile["world_from_build"])
        if not np.allclose(self.transform[:3, :3], np.eye(3)):
            raise ValueError("Structural gravity assumes an upright build frame")
        self.cooling = settings["assumed_cooling_s"]
        self.structure = {}
        self.best = []
        self.search_metadata = {}
        self.checkpoint_binding = {
            "inputs": {
                name: sha256(self.run / name)
                for name in (
                    "cell_manifest.json",
                    "stroke_graph.json",
                    "construction_plan.json",
                    "fabrication_config.json",
                )
            },
            "settings": self.planner.settings,
            "transit": self.planner.transit,
            "timing": self.planner.timing,
            "process": self.planner.process,
            "initial_positions": self.initial.tolist(),
            "drawing_policy": "whole-connected-action-v1",
        }

    def stable(self, ids):
        ids = frozenset(ids)
        if ids not in self.structure:
            screen = analyze_structure(
                self.planner.graph,
                self.material,
                edge_ids={m for key in ids for m in self.actions[key]["members"]},
            )
            # Search revisits verdicts, not deformation meshes. Retaining those
            # meshes for every subset makes a large build's cache enormous.
            self.structure[ids] = (
                screen
                if len(ids) == len(self.actions)
                else {key: value for key, value in screen.items() if key not in {"nodes", "edges"}}
            )
        return self.structure[ids]["modeled_pass"]

    def progressive_structure(self, row, drawing, completed_ids):
        """Screen material prefixes in the proposed traversal, without new supports.

        A final stable loop cannot hide an unstable intermediate cantilever.
        This is a sampled, cooled-elastic screen at each real member boundary;
        it does not claim a calibrated thermal model or clamp the growing tip.
        A newly visited node stays internal to this action: only the timing
        policy can make it a cooled anchor for a later operation.
        """
        oriented = orient_stroke(
            row,
            reverse=drawing.get("reverse", False),
            start_visit=drawing.get("start_visit", 0),
        )
        completed = {member for key in completed_ids for member in self.actions[key]["members"]}
        key = (frozenset(completed), tuple(oriented["members"]))
        cache = self.__dict__.setdefault("progressive_structures", {})
        if key in cache:
            return cache[key]
        report = analyze_member_prefixes(
            self.planner.graph,
            self.material,
            row,
            completed,
            reverse=drawing.get("reverse", False),
            start_visit=drawing.get("start_visit", 0),
        )
        cache[key] = report
        return report

    def scene(self, ids):
        scene = MaterialScene()
        for key in sorted(ids):
            row = self.actions[key]
            points, _ = flatten_curve(row, 0.00005, 0.003)
            points = transform_points(points, self.transform)
            for index, (a, b) in enumerate(zip(points[:-1], points[1:], strict=True)):
                # Enclose both source flattening and the permitted tracking error.
                scene.add(a, b, row["diameter_m"] / 2 + 0.00035, f"{key}/{index}")
        return scene

    def state(self, ids, q, scene=None):
        available = set(self.planner.ground) | {
            self.actions[key][end] for key in ids for end in ("start", "end")
        }
        available.update(
            visit["node"] for key in ids for visit in self.actions[key].get("junction_visits", [])
        )
        return ScheduleState(
            0.0,
            ((self.arm.identity, tuple(q)),),
            frozenset(ids),
            tuple((node, 0.0) for node in sorted(available)),
            payload=self.scene(ids) if scene is None else scene,
        )

    def check_forward_timing(self, row, clock):
        """Qualify timing independently before persisting a complete state."""
        from .clock_path import polynomial_tcp_speed_bound, tcp_tangent_bound
        from .process_clock import audit_process_clock

        transform = self.transform
        if self.planner.turntable is not None:
            transform = self.planner.turntable.build_transform(
                clock.metadata["drawing_table_index"]
            )
        audit_process_clock(
            clock, row, self.planner.process, self.cooling, transform, require_release=True
        )
        store = self.planner.path_certificates
        context = self.planner.path_certificate_context(self.arm.identity, kinematic=True)
        if store.get(context, clock.record(), "joint_speed") is not None:
            return
        drawing_proof = store.get(context, clock.record(), "drawing_speed")
        for edge in clock.edges:
            if drawing_proof is not None and edge.kind == "DRAW":
                continue
            if monotonic() >= self.planner.deadline:
                raise TimeoutError("Forward timing qualification deadline")
            duration = edge.end_s - edge.start_s
            limit = (
                self.planner.process["nominal_speed_m_s"]
                if edge.kind == "DRAW"
                else self.planner.timing["travel_speed_m_s"]
            )
            if edge.coefficients is not None:
                piece = edge.piece()
                ranges = piece.ranges(0, 1)
                if np.any(ranges[:, 0] < self.arm.robot.lower - 1e-9) or np.any(
                    ranges[:, 1] > self.arm.robot.upper + 1e-9
                ):
                    raise ValueError("Forward polynomial exceeds joint limits")
                first, second = piece.derivatives(0, 1)
                velocity, acceleration = first / duration, second / duration**2
                speed = (
                    0.0
                    if edge.kind == "DRAW"
                    and not self.planner.timing.get("audit_drawing_tip_speed", True)
                    else polynomial_tcp_speed_bound(
                        edge, self.arm, limit, deadline=self.planner.deadline
                    )
                )
            else:
                dq = abs(edge.q1 - edge.q0)
                velocity, acceleration = 1.875 * dq / duration, 10 / np.sqrt(3) * dq / duration**2
                speed = 1.875 * tcp_tangent_bound(self.arm, edge.q0, edge.q1) / duration
            if (
                np.any(
                    velocity
                    > self.arm.robot.velocity * self.planner.timing["velocity_fraction"] + 1e-8
                )
                or np.any(
                    acceleration
                    > np.asarray(self.planner.timing["joint_acceleration_rad_s2"]) + 1e-8
                )
                or speed > limit + 1e-8
            ):
                raise ValueError("Forward clock exceeds checked speed or acceleration")
        if drawing_proof is None and any(edge.kind == "DRAW" for edge in clock.edges):
            store.admit(context, clock.record(), "drawing_speed", {"status": "CHECKED"})
        store.admit(
            context,
            clock.record(),
            "joint_speed",
            {"status": "CHECKED", "checked_edges": len(clock.edges)},
        )

    def travel_clock(self, scene, q0, q1, kind="TRAVEL"):
        if np.allclose(q0, q1, atol=1e-10, rtol=0):
            return None
        result = self.planner.travel(self.arm.identity, scene, q0, q1)
        if result["status"] != "CONNECTED":
            return False
        return timed_path(
            [{"kind": kind, "positions": result["positions"], "stroke_id": kind.lower()}],
            self.arm,
            self.arm.robot.velocity * self.planner.timing["velocity_fraction"],
            np.asarray(self.planner.timing["joint_acceleration_rad_s2"]),
            self.planner.process["nominal_speed_m_s"],
            self.planner.timing["travel_speed_m_s"],
        )

    def report(self, status, **extra):
        planning_scope = self.planner.collision_scope
        final_verified = (
            extra.get("final_collision_scope") == "full_robot"
            and extra.get("replay_status") == "CERTIFIED_GEOMETRIC_REPLAY"
        )
        extra["planning_collision_scope"] = planning_scope
        extra["collision_scope"] = "full_robot" if final_verified else planning_scope
        extra["requires_full_robot_validation"] = (
            planning_scope == "pen_candidate" and not final_verified
        )
        if getattr(self.planner, "path_certificates", None) is not None:
            extra["verification"] = {
                "path_certificates": self.planner.path_certificates.report(),
                "counts": dict(self.planner.verification_stats),
                "seconds": dict(self.planner.verification_seconds),
            }
        if self.search_metadata:
            extra["regression"] = self.search_metadata
        write_json(
            self.run / "section_build.json",
            artifact(
                "section_build",
                status=status,
                completed_sections=len(self.best),
                required_sections=len(self.actions),
                best_prefix=self.best,
                structural_states=len(self.structure),
                failures=self.planner.attempts,
                simulation_only=True,
                physical_status="unverified",
                **extra,
            ),
        )

    def run_build(self, workers, *, action_budget=240.0):
        if not self.stable(self.actions):
            self.report(
                "GOAL_STRUCTURE_REJECTED", structure=self.structure[frozenset(self.actions)]
            )
            return
        from .parallel_build import run_parallel_forward

        run_parallel_forward(self, workers, action_budget=action_budget)


def combine_clocks(*paths):
    edges, metadata = [], {}
    for path in paths:
        if path is None:
            continue
        offset = edges[-1].end_s if edges else 0.0
        edges.extend(
            replace(e, start_s=e.start_s + offset, end_s=e.end_s + offset) for e in path.edges
        )
        metadata.update(
            {
                key: value
                for key, value in path.metadata.items()
                if key not in {"deposition_events", "release_ready_s"}
            }
        )
        if "deposition_events" in path.metadata:
            metadata.setdefault("deposition_events", []).extend(
                {
                    **event,
                    "deposited_at_s": event["deposited_at_s"] + offset,
                    "ready_at_s": event["ready_at_s"] + offset,
                }
                for event in path.metadata["deposition_events"]
            )
        if "release_ready_s" in path.metadata:
            metadata["release_ready_s"] = max(
                metadata.get("release_ready_s", 0.0), path.metadata["release_ready_s"] + offset
            )
    clock = ClockPath(edges)
    clock.metadata.update(metadata)
    return clock


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--cell-source-run",
        type=Path,
        required=True,
        help="Bind a fresh run to a generated frame source run",
    )
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--maximum-seconds", type=float, default=3600)
    parser.add_argument("--workers", type=int, default=24)
    parser.add_argument(
        "--action-budget", type=float, default=240.0, help="Seconds per parallel planning job"
    )
    args = parser.parse_args()
    if args.workers < 1:
        parser.error("--workers needs at least one worker")
    if not (args.run / "run.json").exists():
        prepare_cell(args.profile, args.run, args.cell_source_run)
        prepare_candidates(args.run)
    build = SectionBuild(args.run, args.profile, args.maximum_seconds)
    try:
        build.run_build(args.workers, action_budget=args.action_budget)
    except Exception as error:
        build.report("ERROR", error={"type": type(error).__name__, "message": str(error)})
        raise
    print(read_json(args.run / "section_build.json")["status"], flush=True)
    return 0 if read_json(args.run / "section_build.json")["status"] == "FULL_BUILD_PLANNED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
