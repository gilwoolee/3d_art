"""Build source sections and save clocks for separate verification and playback."""

import argparse
import hashlib
import json
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
from .cell_motion import drawing_alternatives
from .cell_planner import CellPlanner
from .clock_path import ClockPath, timed_path
from .dual_arm import MotionProposal, ScheduledMotion, ScheduleState
from .regression import search_regression
from .scene import MaterialScene
from .sections import section_actions
from .source_coverage import audit_source_curve


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


def prepare_reference(fixture, run, profile):
    """Keep the reference topology separate from automatic mesh abstraction."""
    source = run.with_name(run.name + "_source")
    if (run / "run.json").exists():
        return
    source.mkdir(parents=True, exist_ok=True)
    manifest = read_json(fixture / "manifest.json")
    mesh = fixture / manifest["source"]
    write_json(
        source / "run.json",
        {
            "id": source.name,
            "name": manifest["name"] + " reference sections",
            "source_sha256": sha256(mesh),
            "created_at": now(),
            "updated_at": now(),
            "stages": {},
        },
    )
    prepare_asset(
        mesh,
        source,
        {
            "source_up": manifest["source_up"],
            "max_extent_m": manifest["max_extent_m"],
            "geometry_faces": 20000,
        },
    )
    stage_update(source, "asset", "complete", artifact="asset_manifest.json")
    graph = read_json(fixture / "stroke_graph.json")
    construction = section_actions(graph, reference=True)
    write_json(source / "stroke_graph.json", graph)
    write_json(source / "construction_plan.json", construction)
    write_json(source / "fabrication_config.json", read_json(fixture / "fabrication_config.json"))
    write_section_artifacts(source, graph, construction)
    stage_update(
        source,
        "construction",
        "partial",
        artifact="construction_plan.json",
        message="Source sections prepared; order and motion pending",
    )
    prepare_cell(profile, run, source)
    prepare_candidates(run)


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


def prepare_framework(source, run, profile, wir3d_budget=None):
    from ..abstraction.hybrid import attach_style, select_printable_style
    from ..abstraction.wir3d_adapter import import_curves
    from ..abstraction.wireprint import connect_framework_islands, generate, reinforce_base

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
        graph = connect_framework_islands(generate(source, **settings))
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
    if wir3d_budget is not None:
        curves = import_curves(
            source, source / "wir3d_bundle" / f"curves_{wir3d_budget}.json", wir3d_budget
        )["curves"]
        curves, selection = select_printable_style(curves)
        selection["inputs"] = {
            f"wir3d_{wir3d_budget}.json": sha256(source / f"wir3d_{wir3d_budget}.json")
        }
        write_json(source / "style_selection.json", selection)
        graph, attachments = attach_style(graph, curves)
        write_json(source / "style_attachments.json", attachments)
        graph.setdefault("inputs", {}).update(
            {
                name: sha256(source / name)
                for name in (
                    "framework_graph.json",
                    f"wir3d_{wir3d_budget}.json",
                    "style_selection.json",
                    "style_attachments.json",
                )
            }
        )
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


class SectionBuild:
    def __init__(
        self,
        run,
        profile,
        maximum_seconds=3600,
        gpu=False,
        *,
        motion_library=None,
        diffusion_checkpoint=None,
        learning_device="cpu",
        geometry_first=False,
        adaptation_waypoint_checks=False,
        merge_deposits_m=0.0,
        retiming_grid="full",
        structural_check="action",
        retiming_solver="toppra",
        path_pieces="samples",
    ):
        self.run, self.profile = Path(run), Path(profile)
        if diffusion_checkpoint is not None and motion_library is None:
            raise ValueError("A diffusion checkpoint requires a motion library")
        self.motion_library_path = None if motion_library is None else Path(motion_library)
        self.diffusion_checkpoint = (
            None if diffusion_checkpoint is None else Path(diffusion_checkpoint)
        )
        self.learning_device = learning_device
        self.motion_runtime = None
        settings = yaml.safe_load(Path("configs/dual_arm.yaml").read_text())
        settings.update(maximum_subdivision_depth=20, collision_scope="pen_candidate")
        if structural_check == "member":
            settings["structural_check"] = "member"
        if merge_deposits_m:
            # Saved with the plan, so its verification models the wire identically.
            settings["merge_deposits_m"] = merge_deposits_m
        if path_pieces == "members":
            # One path piece per straight member, with the pen checked against
            # wire as a row of spheres at spaced poses. Saved with the plan.
            from .pen_spheres import PEN_SPHERES_REVISION

            settings.update(
                path_pieces="members", pen_check=PEN_SPHERES_REVISION, pen_sample_m=0.010
            )
        timing = yaml.safe_load(Path("configs/timing_continuous_bounded.yaml").read_text())
        timing.pop("target_seconds", None)
        if path_pieces == "members":
            timing["retiming_grid_ramps"] = True
        if retiming_grid != "full":
            # Clock knots between spline knots: none, or one midpoint each.
            timing.update(
                retiming_grid_samples=0, retiming_grid_midpoints=retiming_grid == "midpoints"
            )
        if retiming_solver == "scan":
            # The scanned clock already holds the tip under its speed limit over
            # every interval, by the tangent bound it is rescaled against. The
            # audit therefore checks joint limits and does not bound drawing
            # tip speed a second time (decided 2026-10-13).
            timing.update(retiming_solver="scan", audit_drawing_tip_speed=False)
        self.planner = CellPlanner(
            run, profile, settings, monotonic() + maximum_seconds, timing_override=timing
        )
        self.planner.geometry_first = geometry_first
        self.planner.adaptation_waypoint_checks = adaptation_waypoint_checks
        if self.motion_library_path is None and self.planner.turntable is not None:
            self.motion_library_path = Path("runs/motion_library_phase1_upright27_v1")
        if self.motion_library_path is not None and self.planner.turntable is None:
            raise ValueError("Motion reuse requires an indexed turntable profile")
        if len(self.planner.arms) != 1:
            raise ValueError("Section builds require exactly one arm")
        self.planner.transit["rrt_maximum_seconds"] = 30.0
        self.arm = next(iter(self.planner.arms.values()))
        self.initial = self.arm.initial_positions.copy()
        self.actions = self.planner.actions
        self.planner.settings["maximum_states"] = max(
            self.planner.settings["maximum_states"], 4 * len(self.actions)
        )
        self.material = read_json(self.run / "fabrication_config.json")["structure"]
        self.transform = np.asarray(self.planner.profile["world_from_build"])
        if not np.allclose(self.transform[:3, :3], np.eye(3)):
            raise ValueError("Structural gravity assumes an upright build frame")
        self.cooling = settings["assumed_cooling_s"]
        self.structure = {}
        self.candidates = {}
        self.best = []
        self.search_metadata = {}
        self.gpu = None
        if gpu:
            from .curobo_adapter import CuroboAdapter

            self.gpu = CuroboAdapter(self.arm)
            self.planner.ik_backend = self.gpu
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
        self.checkpoint_records = []
        self.retained_checkpoint_count = 0
        self.resumed_motions = 0
        checkpoint = self.run / "forward_checkpoint.json"
        if checkpoint.exists():
            try:
                saved = read_json(checkpoint)
            except (ValueError, OSError):
                saved = {}
            if (
                isinstance(saved, dict)
                and saved.get("status") == "PROPOSALS_REQUIRE_REVALIDATION"
                and saved.get("binding") == self.checkpoint_binding
                and isinstance(saved.get("witnesses"), list)
            ):
                self.checkpoint_records = saved["witnesses"]
                self.retained_checkpoint_count = len(self.checkpoint_records)
            else:
                print(
                    "Ignoring forward checkpoint: invalid index or changed planning inputs",
                    flush=True,
                )
        self.backward_checkpoint_records = {}
        self.revalidated_backward_motions = 0
        checkpoint = self.run / "backward_checkpoint.json"
        if checkpoint.exists():
            try:
                saved = read_json(checkpoint)
            except (ValueError, OSError):
                saved = {}
            if (
                isinstance(saved, dict)
                and saved.get("status") == "PROPOSALS_REQUIRE_REVALIDATION"
                and saved.get("binding") == self.checkpoint_binding
                and isinstance(saved.get("witnesses"), dict)
                and all(
                    isinstance(key, str) and isinstance(value, list)
                    for key, value in saved["witnesses"].items()
                )
            ):
                self.backward_checkpoint_records = saved["witnesses"]
            else:
                print("Ignoring backward checkpoint: invalid index or changed inputs", flush=True)

    def checkpoint_witness(
        self, key, move, drawing, *, directory="forward_witnesses", context=None
    ):
        """Persist a proposal, never a replacement for fresh validation."""
        if not hasattr(self, "checkpoint_binding"):
            return None
        record = {"section": key, "drawing": drawing, "trajectory": move.payload.record()}
        if context is not None:
            record["context"] = context
        encoded = json.dumps(record, indent=2, allow_nan=False) + "\n"
        digest = hashlib.sha256(encoded.encode()).hexdigest()
        path = self.run / directory / f"{digest}.json"
        if not path.exists() or sha256(path) != digest:
            write_json(path, record)
        return {"file": str(path.relative_to(self.run)), "sha256": digest}

    def checkpoint_proposal(self, index, row, state, q):
        if index >= len(getattr(self, "checkpoint_records", [])):
            return None
        reference = self.checkpoint_records[index]
        try:
            relative = Path(reference["file"])
            if relative.is_absolute() or ".." in relative.parts:
                raise ValueError("Invalid checkpoint witness path")
            path = self.run / relative
            if sha256(path) != reference["sha256"]:
                raise ValueError("Checkpoint witness hash changed")
            saved = read_json(path)
            drawing = saved["drawing"]
            if saved["section"] != row["id"] or drawing["stroke_id"] != row["id"]:
                return None
            clock = ClockPath.from_record(saved["trajectory"])
            if not np.allclose(clock.edges[0].q0, q, atol=1e-10, rtol=0):
                return None
            if drawing["start_anchor"] not in state.ready_nodes():
                return None
            if not self.progressive_structure(row, drawing, state.completed)["modeled_pass"]:
                return None
            points, _ = flatten_curve(row, 0.00005, 0.003)
            transform = self.transform
            if self.planner.turntable is not None:
                transform = self.planner.turntable.build_transform(
                    clock.metadata["drawing_table_index"]
                )
            source_points = transform_points(points, transform)
            store = getattr(self.planner, "path_certificates", None)
            coverage_context = None
            audit = None
            if store is not None and not getattr(self.planner, "verification_raw", False):
                coverage_context = {
                    "invariants": self.planner.path_certificate_context(
                        self.arm.identity, kinematic=True
                    ),
                    "source_points_m": source_points,
                }
                audit = store.get(coverage_context, clock.record(), "source_coverage")
            if audit is None:
                audit = audit_source_curve(self.arm, [clock], source_points)
                if audit["status"] == "BOUNDED" and coverage_context is not None:
                    store.admit(coverage_context, clock.record(), "source_coverage", audit)
            if audit["status"] != "BOUNDED":
                return None
            move = MotionProposal(
                row["id"],
                self.arm.identity,
                drawing["variant"],
                clock.duration_s,
                drawing["start_anchor"],
                tuple(clock.edges[-1].q1),
                clock,
            )
            after, failure = self.planner.validate_interval(
                state, (ScheduledMotion(move, 0.0),), move.duration_s
            )
            if failure:
                return None
        except (KeyError, TypeError, ValueError, OSError) as error:
            print(f"Ignoring checkpoint for {row['id']}: {error}", flush=True)
            return None
        self.resumed_motions += 1
        print(f"Revalidated saved motion for {row['id']}", flush=True)
        return move, drawing, after, reference

    def backward_checkpoint_proposal(self, reference, row, context, state):
        """Recheck a saved section plus its transit in this exact backward state."""
        try:
            relative = Path(reference["file"])
            if relative.is_absolute() or ".." in relative.parts:
                raise ValueError("Invalid checkpoint witness path")
            path = self.run / relative
            if sha256(path) != reference["sha256"]:
                raise ValueError("Checkpoint witness hash changed")
            saved = read_json(path)
            drawing = saved["drawing"]
            if (
                saved.get("context") != context
                or saved["section"] != row["id"]
                or drawing["stroke_id"] != row["id"]
                or drawing["start_anchor"] not in state.ready_nodes()
            ):
                return None
            if not self.progressive_structure(row, drawing, state.completed)["modeled_pass"]:
                return None
            clock = ClockPath.from_record(saved["trajectory"])
            if not np.allclose(
                clock.edges[-1].q1, context["terminal_configuration"], atol=1e-10, rtol=0
            ):
                return None
            points, _ = flatten_curve(row, 0.00005, 0.003)
            audit = audit_source_curve(self.arm, [clock], transform_points(points, self.transform))
            if audit["status"] != "BOUNDED":
                return None
            move = MotionProposal(
                row["id"],
                self.arm.identity,
                drawing["variant"],
                clock.duration_s,
                drawing["start_anchor"],
                tuple(clock.edges[-1].q1),
                clock,
            )
            _, failure = self.planner.validate_interval(
                state, (ScheduledMotion(move, 0.0),), move.duration_s
            )
            if failure:
                return None
        except (KeyError, TypeError, ValueError, OSError, IndexError) as error:
            print(f"Ignoring backward checkpoint for {row['id']}: {error}", flush=True)
            return None
        self.revalidated_backward_motions += 1
        print(f"Revalidated saved backward motion for {row['id']}", flush=True)
        return clock.edges[0].q0, (row["id"], move, drawing)

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

    def forward_snapshot_context(self):
        return {
            "binding": self.checkpoint_binding,
            "physical": self.planner.path_certificate_context(self.arm.identity),
            "mode": "cooled_local_forward_boundaries",
        }

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

    def drawings(self, row, available, q):
        # Geometry candidates may be reused; all state-dependent tests run again.
        for candidate in self.candidates.get(row["id"], []):
            if candidate["start_anchor"] in available:
                yield candidate
        for candidate in drawing_alternatives(
            self.planner.collision_arm(self.arm.identity),
            row,
            available,
            self.transform,
            self.planner.obstacles,
            deadline=self.planner.deadline,
            seeds=[np.asarray(q), self.initial],
            settings={"maximum_subdivision_depth": 20},
            clearance_m=self.planner.margin,
            pen_tilt_deg=self.planner.profile.get("drawing_pen_tilt_deg", 70.0),
            pen_tilt_candidates_deg=self.planner.profile.get("drawing_pen_tilt_candidates_deg"),
            orientation_policy=self.planner.profile.get("drawing_orientation_policy", "fixed"),
            ik_backend=self.gpu,
        ):
            if candidate["status"] == "SAMPLED_DRAWING_PATH":
                candidate["release_cooling_s"] = self.cooling
                candidate["collision_scope"] = self.planner.collision_scope
                if self.planner.collision_scope == "pen_candidate":
                    candidate["static_collision_status"] = "CHECKED_PEN_STATIC_ONLY"
                    candidate["requires_full_robot_validation"] = True
                self.candidates.setdefault(row["id"], []).append(candidate)
                yield candidate

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
        if hasattr(self, "forward_state_restore"):
            extra["forward_state_restore"] = self.forward_state_restore
        if getattr(self, "forward_state_saved", None):
            extra["forward_state_snapshot_sha256"] = self.forward_state_saved
        if getattr(self.planner, "path_certificates", None) is not None:
            extra["verification"] = {
                "path_certificates": self.planner.path_certificates.report(),
                "counts": dict(self.planner.verification_stats),
                "seconds": dict(self.planner.verification_seconds),
            }
        if getattr(self, "motion_runtime", None) is not None:
            extra["motion_reuse"] = self.motion_runtime.report()
        if getattr(self, "resumed_motions", 0):
            extra["revalidated_saved_motions"] = self.resumed_motions
        if getattr(self, "revalidated_backward_motions", 0):
            extra["revalidated_backward_motions"] = self.revalidated_backward_motions
        if self.search_metadata:
            extra["regression"] = self.search_metadata
        if self.gpu is not None:
            extra["gpu"] = {"setup_seconds": self.gpu.setup_seconds, "batches": self.gpu.batches}
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

    def forward(self, order, *, indexed_model=None, allow_snapshot=True):
        """Try the supplied order, backtracking over robot modes within that order."""

        class StructuralOrderRejected(Exception):
            pass

        def choices(index, q, scene):
            if monotonic() >= self.planner.deadline:
                return
            row = order[index]
            ids = {item["id"] for item in order[:index]}
            if not self.stable(ids | {row["id"]}):
                self.report("PROPOSED_ORDER_STRUCTURE_REJECTED", rejected_section=row["id"])
                # The material prefix is fixed by the order, independently of
                # every robot mode. Backtracking poses cannot repair this state.
                raise StructuralOrderRejected
            state = self.state(ids, q, scene)
            cached = self.checkpoint_proposal(index, row, state, q)
            if cached is not None:
                yield cached
            if indexed_model is not None:
                for move, drawing, after in indexed_model.forward_motions(row, state):
                    yield move, drawing, after, self.checkpoint_witness(row["id"], move, drawing)
                return
            for drawing in self.drawings(row, state.ready_nodes(), q):
                if cached is not None and drawing == cached[1]:
                    continue
                structure = self.progressive_structure(row, drawing, ids)
                if not structure["modeled_pass"]:
                    self.planner.failure(
                        row["id"],
                        self.arm.identity,
                        "PROGRESSIVE_STRUCTURE",
                        structure["status"],
                        validation=structure,
                    )
                    continue
                move = self.planner.prepare_drawing(state, self.arm.identity, drawing, q)
                if move is None:
                    continue
                after, failure = self.planner.validate_interval(
                    state, (ScheduledMotion(move, 0.0),), move.duration_s
                )
                if failure:
                    self.planner.failure(
                        row["id"],
                        self.arm.identity,
                        "REPLAY",
                        failure["status"],
                        validation=failure,
                    )
                    continue
                yield move, drawing, after, self.checkpoint_witness(row["id"], move, drawing)

        if not order:
            return [], None
        # A real generated framework can exceed Python's recursion limit.
        # Keep each lazy choice iterator alive so backtracking still retains
        # all robot modes, without consuming a Python frame per section.
        initial_scene = MaterialScene()
        if indexed_model is not None:
            initial_scene.table_index = indexed_model.table.home_index
        stack = [iter(choices(0, self.initial, initial_scene))]
        prefix = []
        references = []
        snapshot_capable = hasattr(self, "checkpoint_binding") and hasattr(
            self.planner, "path_certificate_context"
        )
        restored_count = 0
        snapshot_context = self.forward_snapshot_context() if snapshot_capable else None
        if snapshot_capable and allow_snapshot:
            from .forward_state import restore_forward_state

            restored = restore_forward_state(
                self.run / "forward_state_checkpoint.json",
                snapshot_context,
                [row["id"] for row in order],
                self.checkpoint_records,
                witness_root=self.run,
                raw=getattr(self.planner, "verification_raw", False),
            )
            if restored is not None:
                references = restored["witnesses"]
                for reference in references:
                    saved = read_json(self.run / reference["file"])
                    clock = ClockPath.from_record(saved["trajectory"])
                    drawing = saved["drawing"]
                    move = MotionProposal(
                        saved["section"],
                        self.arm.identity,
                        drawing["variant"],
                        clock.duration_s,
                        drawing["start_anchor"],
                        tuple(clock.edges[-1].q1),
                        clock,
                    )
                    prefix.append((saved["section"], move, drawing))
                state = restored["state"]
                q = dict(state.configurations)[self.arm.identity]
                if not np.allclose(q, prefix[-1][1].end_configuration, atol=1e-10, rtol=0):
                    raise ValueError("Qualified forward snapshot joint boundary differs")
                restored_count = len(prefix)
                self.best = [entry[0] for entry in prefix]
                self.forward_state_restore = {
                    "collision_scope": self.planner.collision_scope,
                    "requires_full_robot_validation": self.planner.collision_scope
                    == "pen_candidate",
                    "sections": restored_count,
                    "material_capsules": len(state.payload.segments),
                    "snapshot_sha256": restored["snapshot_sha256"],
                }
                candidate_only = self.planner.collision_scope == "pen_candidate"
                label = "pen candidate" if candidate_only else "qualified forward"
                print(f"Restored {label} state: {restored_count}/{len(order)}", flush=True)
                self.report(
                    "PEN_CANDIDATE_STATE_RESTORED"
                    if candidate_only
                    else "QUALIFIED_FORWARD_STATE_RESTORED"
                )
                if restored_count == len(order):
                    returned = (
                        self.travel_clock(state.payload, q, self.initial, "RETURN")
                        if indexed_model is None
                        else indexed_model.transition(
                            state.payload,
                            q,
                            state.table_index,
                            self.initial,
                            indexed_model.table.home_index,
                        )
                    )
                    if returned is not False:
                        return prefix, returned
                stack = (
                    [iter(choices(restored_count, q, state.payload))]
                    if restored_count < len(order)
                    else []
                )
        while stack and monotonic() < self.planner.deadline:
            try:
                move, drawing, after, reference = next(stack[-1])
            except StructuralOrderRejected:
                return None
            except StopIteration:
                stack.pop()
                if prefix:
                    prefix.pop()
                    references.pop()
                continue
            if snapshot_capable:
                try:
                    self.check_forward_timing(order[len(prefix)], move.payload)
                except (ValueError, TimeoutError) as error:
                    self.planner.failure(
                        order[len(prefix)]["id"],
                        self.arm.identity,
                        "FORWARD_TIMING",
                        type(error).__name__,
                        detail=str(error),
                    )
                    continue
            prefix.append((order[len(prefix)]["id"], move, drawing))
            references.append(reference)
            if len(prefix) > len(self.best):
                self.best = [item[0] for item in prefix]
                if hasattr(self, "checkpoint_binding") and len(prefix) > getattr(
                    self, "retained_checkpoint_count", 0
                ):
                    write_json(
                        self.run / "forward_checkpoint.json",
                        {
                            "status": "PROPOSALS_REQUIRE_REVALIDATION",
                            "binding": self.checkpoint_binding,
                            "witnesses": references,
                        },
                    )
                    self.retained_checkpoint_count = len(prefix)
                    self.checkpoint_records = list(references)
                self.report("PROPOSED_ORDER_RUNNING")
            if snapshot_capable and references == self.checkpoint_records[: len(references)]:
                from .forward_state import save_forward_state, snapshot_gates

                boundary = self.state([item[0] for item in prefix], move.end_configuration, after)
                self.forward_state_saved = save_forward_state(
                    self.run / "forward_state_checkpoint.json",
                    snapshot_context,
                    [row["id"] for row in order],
                    references,
                    boundary,
                    snapshot_gates(snapshot_context),
                    witness_root=self.run,
                    raw=getattr(self.planner, "verification_raw", False),
                )
            if len(prefix) == len(order):
                returned = (
                    self.travel_clock(after, move.end_configuration, self.initial, "RETURN")
                    if indexed_model is None
                    else indexed_model.transition(
                        after,
                        move.end_configuration,
                        after.table_index,
                        self.initial,
                        indexed_model.table.home_index,
                    )
                )
                if returned is not False:
                    return prefix, returned
                prefix.pop()
                references.pop()
            else:
                stack.append(iter(choices(len(prefix), move.end_configuration, after)))
        if restored_count and monotonic() < self.planner.deadline:
            # A resumed suffix is one robot branch. If it is exhausted, retain
            # the ordinary search's ability to backtrack through older motions.
            return self.forward(order, indexed_model=indexed_model, allow_snapshot=False)
        return None

    def backward(self):
        def motions(row, predecessor, available, q):
            state = self.state(predecessor, self.initial)
            context = {
                "section": row["id"],
                "predecessor": sorted(predecessor),
                "terminal_configuration": np.asarray(q).tolist(),
            }
            key = hashlib.sha256(json.dumps(context, sort_keys=True).encode()).hexdigest()
            records = getattr(self, "backward_checkpoint_records", {}).get(key, [])
            if isinstance(records, list):
                for reference in records:
                    cached = self.backward_checkpoint_proposal(reference, row, context, state)
                    if cached is not None:
                        yield cached
            for drawing in self.drawings(row, available, q):
                structure = self.progressive_structure(row, drawing, predecessor)
                if not structure["modeled_pass"]:
                    self.planner.failure(
                        row["id"],
                        self.arm.identity,
                        "PROGRESSIVE_STRUCTURE",
                        structure["status"],
                        validation=structure,
                    )
                    continue
                move = self.planner.prepare_drawing(state, self.arm.identity, drawing, None)
                if move is None:
                    continue
                after, failure = self.planner.validate_interval(
                    state, (ScheduledMotion(move, 0),), move.duration_s
                )
                if failure:
                    continue
                travel = self.travel_clock(after, move.end_configuration, q)
                if travel is False:
                    continue
                path = combine_clocks(move.payload, travel)
                combined = replace(
                    move, payload=path, duration_s=path.duration_s, end_configuration=tuple(q)
                )
                reference = self.checkpoint_witness(
                    row["id"], combined, drawing, directory="backward_witnesses", context=context
                )
                if reference is not None:
                    records = self.backward_checkpoint_records.setdefault(key, [])
                    if reference not in records:
                        records.append(reference)
                        write_json(
                            self.run / "backward_checkpoint.json",
                            {
                                "status": "PROPOSALS_REQUIRE_REVALIDATION",
                                "binding": self.checkpoint_binding,
                                "witnesses": self.backward_checkpoint_records,
                            },
                        )
                yield path.edges[0].q0, (row["id"], combined, drawing)

        def initial_travel(q):
            result = self.travel_clock(MaterialScene(), self.initial, q)
            if result is False:
                return None
            return False if result is None else result

        self.report("REGRESSION_RUNNING", phase="forward_stiffness_guide")
        result = search_regression(
            list(self.actions.values()),
            self.planner.graph,
            self.material,
            self.initial,
            motions,
            initial_travel,
            deadline=self.planner.deadline,
            maximum_states=self.planner.settings["maximum_states"],
            progress=lambda depth, attempts, section: self.report(
                "REGRESSION_RUNNING", depth=depth, attempts=attempts, section=section
            ),
        )
        self.search_metadata = {key: value for key, value in result.items() if key != "result"}
        if result["status"] != "FOUND":
            self.report(result["status"], search={k: v for k, v in result.items() if k != "result"})
            return None
        incoming, witnesses = result["result"]
        key, move, drawing = witnesses[0]
        path = combine_clocks(incoming if incoming is not False else None, move.payload)
        witnesses[0] = (key, replace(move, payload=path, duration_s=path.duration_s), drawing)
        return witnesses, None  # Last regression witness includes the final home transit.

    def run_build(
        self,
        regression_only=False,
        progression_only=False,
        *,
        workers=None,
        action_budget=240.0,
        inject_rejections=(),
    ):
        if not self.stable(self.actions):
            self.report(
                "GOAL_STRUCTURE_REJECTED", structure=self.structure[frozenset(self.actions)]
            )
            return
        if workers is not None:
            if "turntable" not in self.planner.profile or not progression_only:
                raise ValueError("Parallel workers require an indexed --progression-only build")
            from .parallel_build import run_parallel_forward

            run_parallel_forward(
                self, workers, action_budget=action_budget, inject_rejections=inject_rejections
            )
            return
        if "turntable" in self.planner.profile:
            from .indexed_build import run_indexed, run_indexed_forward

            (run_indexed_forward if progression_only else run_indexed)(self)
            return
        nodes = {row["id"]: row for row in self.planner.graph["nodes"]}
        order = sorted(
            self.actions.values(),
            key=lambda row: (
                row["stage"] != "framework",
                row["priority"],
                min(nodes[row[end]]["position"][2] for end in ("start", "end")),
                row["id"],
            ),
        )
        result = None if regression_only else self.forward(order)
        if result is None and not progression_only and monotonic() < self.planner.deadline:
            result = self.backward()
        if result is None:
            self.report(
                "SEARCH_TIME_BUDGET" if monotonic() >= self.planner.deadline else "NO_COMPLETE_PLAN"
            )
            return
        from .full_build_export import export_sections

        export_sections(self, *result)


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
    parser.add_argument("--fixture", type=Path)
    parser.add_argument("--mesh", type=Path, help="Generate a WirePrint framework from a new mesh")
    parser.add_argument("--source-up", choices=("y", "z"), default="z")
    parser.add_argument("--max-extent-m", type=float, default=0.15)
    parser.add_argument(
        "--source-run", type=Path, help="Prepared mesh run for WirePrint generation"
    )
    parser.add_argument(
        "--wir3d-budget", type=int, help="Import the completed WIR3D job for this mesh"
    )
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--profile", type=Path, default=Path("research/planning_cell/cell.yaml"))
    parser.add_argument("--maximum-seconds", type=float, default=3600)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--regression-only", action="store_true")
    modes.add_argument("--progression-only", action="store_true")
    parser.add_argument("--gpu", action="store_true")
    parser.add_argument(
        "--motion-library", type=Path, help="Reuse and collect verified drawing motions"
    )
    parser.add_argument(
        "--diffusion-checkpoint", type=Path, help="Add learned proposals after cached candidates"
    )
    parser.add_argument("--learning-device", default="cpu", help="Diffusion inference device")
    parser.add_argument(
        "--cell-source-run",
        type=Path,
        help="Bind a fresh run to an existing generated source run, without regenerating it",
    )
    parser.add_argument(
        "--workers",
        type=int,
        help="Plan different whole actions in this many worker processes, then export "
        "playback. One worker runs the same pipeline as the comparison mode",
    )
    parser.add_argument(
        "--action-budget", type=float, default=240.0, help="Seconds per parallel planning job"
    )
    parser.add_argument(
        "--geometry-first",
        action="store_true",
        help="Check new drawings as untimed paths and time them after the whole chain passes",
    )
    parser.add_argument(
        "--retiming-solver",
        choices=("scan", "toppra"),
        default=None,
        help="How drawing clocks are found: a closed-form forward and backward pass, or the "
        "TOPP-RA solver. Either is rescaled against exact bounds. Default: scan with "
        "--geometry-first, otherwise toppra",
    )
    parser.add_argument(
        "--structural-check",
        choices=("action", "member"),
        default="action",
        help="Screen the partial structure after every completed action (default), or "
        "also after every wire member of a half-drawn action",
    )
    parser.add_argument(
        "--retiming-grid",
        choices=("full", "midpoints", "knots"),
        help="Clock resolution for new drawings: every spline knot plus midpoints and 101 "
        "samples per part (full), knots and midpoints, or knots only. Coarser grids time "
        "and audit faster and draw slower. Default: knots with --geometry-first, else full",
    )
    parser.add_argument(
        "--path-pieces",
        choices=("members", "samples"),
        help="members: one path piece per straight member, the pen checked against wire as "
        "a row of spheres at each piece end and every 10 mm (a sampled check, not a "
        "continuous proof), and clock knots placed for the ramps. samples: a piece per "
        "retained path sample, checked continuously. Default: members with "
        "--geometry-first, else samples",
    )
    parser.add_argument(
        "--merge-deposits-um",
        type=float,
        help="Describe earlier actions' wire by merged capsules whose radius grows by at "
        "most this many micrometres; 0 keeps every fine capsule. Default: 20 with "
        "--geometry-first, else 0",
    )
    parser.add_argument(
        "--adaptation-waypoint-checks",
        action="store_true",
        help="While adapting a cached path, check collision at waypoints only; the "
        "final path check remains continuous",
    )
    parser.add_argument(
        "--inject-rejection",
        type=int,
        action="append",
        default=[],
        metavar="ACTION_INDEX",
        help="Testing only: reject this action's first accepted parallel check, forcing repair",
    )
    args = parser.parse_args()
    if args.retiming_solver is None:
        args.retiming_solver = "scan" if args.geometry_first else "toppra"
    if args.path_pieces is None:
        args.path_pieces = "members" if args.geometry_first else "samples"
    if args.retiming_grid is None:
        args.retiming_grid = "knots" if args.geometry_first else "full"
    if args.merge_deposits_um is None:
        args.merge_deposits_um = 20.0 if args.geometry_first else 0.0
    if args.workers is not None and (args.gpu or not args.progression_only or args.workers < 1):
        parser.error("--workers needs --progression-only, at least one worker, and no --gpu")
    if args.diffusion_checkpoint is not None and args.motion_library is None:
        parser.error("--diffusion-checkpoint requires --motion-library")
    sources = (args.fixture, args.source_run, args.mesh, args.cell_source_run)
    if sum(value is not None for value in sources) > 1:
        parser.error("Choose a reference fixture, a new mesh, or a prepared source run")
    if args.fixture:
        prepare_reference(args.fixture, args.run, args.profile)
    elif args.source_run:
        prepare_framework(args.source_run, args.run, args.profile, args.wir3d_budget)
    elif args.mesh:
        source = prepare_mesh(args.mesh, args.run, args.source_up, args.max_extent_m)
        prepare_framework(source, args.run, args.profile, args.wir3d_budget)
    elif args.cell_source_run and not (args.run / "run.json").exists():
        prepare_cell(args.profile, args.run, args.cell_source_run)
        prepare_candidates(args.run)
    build = SectionBuild(
        args.run,
        args.profile,
        args.maximum_seconds,
        args.gpu,
        motion_library=args.motion_library,
        diffusion_checkpoint=args.diffusion_checkpoint,
        learning_device=args.learning_device,
        geometry_first=args.geometry_first,
        adaptation_waypoint_checks=args.adaptation_waypoint_checks,
        merge_deposits_m=args.merge_deposits_um * 1e-6,
        retiming_grid=args.retiming_grid,
        structural_check=args.structural_check,
        retiming_solver=args.retiming_solver,
        path_pieces=args.path_pieces,
    )
    try:
        build.run_build(
            args.regression_only,
            args.progression_only,
            workers=args.workers,
            action_budget=args.action_budget,
            inject_rejections=args.inject_rejection,
        )
    except Exception as error:
        build.report("ERROR", error={"type": type(error).__name__, "message": str(error)})
        raise
    finally:
        if build.motion_runtime is not None:
            build.motion_runtime.flush()
    print(read_json(args.run / "section_build.json")["status"], flush=True)
    return 0 if read_json(args.run / "section_build.json")["status"] == "FULL_BUILD_PLANNED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
