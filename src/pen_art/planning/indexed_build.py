"""Indexed-plate motion callbacks for the existing construction search."""

from collections import deque
from dataclasses import replace
from pathlib import Path
from time import monotonic

import numpy as np

from ..curves import flatten_curve, transform_points
from ..io import read_json, sha256, write_json
from .axis_corridor import orientation_lattice, pen_clear
from .axis_repair import repair_drawing_axes
from .cell_motion import (
    DEFAULT_PEN_TILTS_DEG,
    anchored_strokes,
    drawing_alternatives,
    fixed_rotation_families,
    outward_rotation_families,
    radial_pen_rotations,
)
from .clock_path import ClockEdge, ClockPath
from .dual_arm import MotionProposal, ScheduledMotion
from .motion_library import DRAWING_POLICY_VERSION
from .process_clock import derive_deposition_events
from .regression import greedy_forward_order, search_regression
from .replay_checkpoint import digest
from .scene import MaterialScene
from .source_coverage import audit_source_curve
from .turntable import IndexedTurntable, audit_indexed_clock, index_angle


def at_index(path, index):
    if path is None or path is False:
        return path
    result = ClockPath([replace(edge, table_index=index) for edge in path.edges])
    result.metadata.update(path.metadata)
    if hasattr(path, "timing_operations"):
        result.timing_operations = path.timing_operations
    return result


class IndexedMotions:
    def __init__(self, build):
        self.build = build
        self.table = IndexedTurntable.from_profile(build.planner.profile)
        self.experience = None
        if getattr(build, "motion_library_path", None) is not None:
            from .motion_runtime import DrawingExperience

            if build.motion_runtime is None:
                build.motion_runtime = DrawingExperience(build, self.table)
            self.experience = build.motion_runtime
        self.round = 0
        self.records = {}
        self.rechecked = {}
        self.warm = {}
        self.checkpoint = build.run / "indexed_checkpoint.json"
        # Profile data includes all frame, tool, and actuator assumptions. Saved
        # motions remain proposals and are checked again against current code.
        self.binding = {**build.checkpoint_binding, "profile": build.planner.profile}
        if self.checkpoint.is_file():
            try:
                saved = read_json(self.checkpoint)
                if (
                    saved["binding"] == self.binding
                    and isinstance(saved["witnesses"], dict)
                    and all(isinstance(value, list) for value in saved["witnesses"].values())
                ):
                    self.records = saved["witnesses"]
                    self.round = max(0, int(saved.get("sampling_round", 0)))
                else:
                    print("Ignoring indexed checkpoint: changed planning inputs", flush=True)
            except (OSError, KeyError, TypeError, ValueError):
                print("Ignoring invalid indexed checkpoint", flush=True)
        self.load_source_proposals()
        self.record_ranks = {key: rank for rank, key in enumerate(self.records)}

    def motion_priority(self, row, remaining, terminal):
        """Resume the oldest retained matching branch before resampling orders."""
        context = {"predecessor": sorted(remaining - {row["id"]}), "terminal": list(terminal)}
        key = digest([row["id"], context])
        return self.record_ranks.get(key, len(self.record_ranks))

    def load_source_proposals(self):
        """Reuse source-run paths as hints, without importing their certificates."""
        build = self.build
        source = Path(read_json(build.run / "run.json")["cell_source_run"])
        plan_file = source / "offline_dual_arm_plan.json"
        try:
            source_graph = read_json(source / "stroke_graph.json")
            target_graph = read_json(build.run / "stroke_graph.json")
            if any(
                source_graph[field] != target_graph[field]
                for field in ("schema_version", "frame", "units", "nodes", "edges")
            ):
                return
            # Generation notes can change while every physical member and
            # anchor stays identical. Bind the source graph for provenance;
            # no source certificate replaces checks in the current scene.
            source_graph_digest = sha256(source / "stroke_graph.json")
            if plan_file.is_file():
                plan = read_json(plan_file)
                settings = read_json(
                    source / plan.get("settings_artifact", "offline_dual_arm_config.json")
                )
                actions = plan.get("actions", [])
                source_digest = {"warm_source_plan_sha256": sha256(plan_file)}
            else:
                # A partial forward build can supply local paths as well. Its
                # certificate and previous material state are never imported.
                checkpoint_file = source / "forward_checkpoint.json"
                checkpoint = read_json(checkpoint_file)
                binding = checkpoint["binding"]
                if any(
                    binding["inputs"][name] != sha256(source / name)
                    for name in (
                        "stroke_graph.json",
                        "construction_plan.json",
                        "fabrication_config.json",
                    )
                ):
                    return
                settings = {**binding, "search": binding["settings"]}
                source_rows = {
                    row["id"]: row
                    for row in read_json(source / "construction_plan.json")["actions"]
                }
                actions = []
                for reference in checkpoint["witnesses"]:
                    relative = Path(reference["file"])
                    if relative.is_absolute() or ".." in relative.parts:
                        continue
                    path = source / relative
                    if sha256(path) != reference["sha256"]:
                        continue
                    saved = read_json(path)
                    drawing = saved["drawing"]
                    if drawing["stroke_id"] != saved["section"]:
                        continue
                    actions.append(
                        {
                            **source_rows[saved["section"]],
                            **drawing,
                            "trajectory": saved["trajectory"],
                        }
                    )
                source_digest = {"warm_source_checkpoint_sha256": sha256(checkpoint_file)}
        except (OSError, KeyError, TypeError, ValueError):
            return
        if (
            settings["timing"] != build.planner.timing
            or settings["process"] != build.planner.process
            or settings["search"]["assumed_cooling_s"] != build.cooling
        ):
            return  # Changed timing/process requires a freshly parameterized path.
        for action in actions:
            row = build.actions.get(action.get("stroke_id"))
            if row is None or action.get("auxiliary") or action.get("arm") != build.arm.identity:
                continue
            if any(
                action.get(key) != row.get(key)
                for key in (
                    "parent_curve",
                    "source_interval",
                    "source_intervals",
                    "members",
                    "stage",
                )
            ):
                continue
            try:
                source_path = ClockPath.from_record(action["trajectory"])
                if source_path.metadata.get("process_clock_policy") != DRAWING_POLICY_VERSION:
                    # Historical clocks are never a shortcut around today's
                    # continuous-action timing and material-readiness rules.
                    continue
                first = next(edge.start_s for edge in source_path.edges if edge.kind == "APPROACH")
                last = max(edge.end_s for edge in source_path.edges if edge.kind == "WITHDRAW")
                local = ClockPath(
                    [
                        replace(edge, start_s=edge.start_s - first, end_s=edge.end_s - first)
                        for edge in source_path.edges
                        if first <= edge.start_s and edge.end_s <= last
                    ]
                )
                local.metadata.update(source_path.metadata)
                index = action.get("table_index", self.table.home_index)
                if any(edge.kind == "INDEX_TURN" for edge in local.edges):
                    continue
                local = at_index(local, index)
                # The source clock may include incoming travel. Its material
                # annotations must use the same new origin as the trimmed edges.
                local.metadata["deposition_events"] = derive_deposition_events(
                    local, row, build.cooling, self.table.build_transform(index)
                )
                local.metadata["release_ready_s"] = max(
                    event["ready_at_s"] for event in local.metadata["deposition_events"]
                )
                points, _ = flatten_curve(row, 0.00005, 0.003)
                audit = audit_source_curve(
                    build.arm, [local], transform_points(points, self.table.build_transform(index))
                )
                if audit["status"] != "BOUNDED":
                    continue
                local.metadata.update(source_digest)
                local.metadata["warm_source_graph_sha256"] = source_graph_digest
                drawing = {
                    "stroke_id": row["id"],
                    "variant": "warm-" + action["variant"],
                    "start_anchor": action["start_anchor"],
                    "reverse": action["reverse"],
                    "start_visit": action.get("start_visit", 0),
                    "table_index": index,
                }
                move = MotionProposal(
                    row["id"],
                    build.arm.identity,
                    drawing["variant"],
                    local.duration_s,
                    drawing["start_anchor"],
                    tuple(local.edges[-1].q1),
                    local,
                )
                self.warm.setdefault((row["id"], index), []).append((move, drawing))
            except (KeyError, IndexError, StopIteration, TypeError, ValueError):
                continue
        print(f"Loaded {sum(map(len, self.warm.values()))} source motion proposals", flush=True)

    def structure_admissible(self, row, drawing, predecessor):
        check = getattr(self.build, "progressive_structure", None)
        if check is None:
            return True  # Small synthetic callback fixtures have no structure model.
        if self.build.planner.settings.get("structural_check", "action") != "member":
            # Screen the structure once per completed action. The member-by-
            # member screen of a half-drawn action is opt-in.
            if self.build.stable(set(predecessor) | {row["id"]}):
                return True
            report = {"status": "ACTION_STRUCTURE_REJECTED", "modeled_pass": False}
        else:
            report = check(row, drawing, predecessor)
        if report["modeled_pass"]:
            return True
        self.build.planner.failure(
            row["id"],
            self.build.arm.identity,
            "PROGRESSIVE_STRUCTURE",
            report["status"],
            validation=report,
        )
        return False

    def retained_move(self, row, drawing, index):
        record = drawing.pop("retained_clock", None)
        if record is None:
            return None
        clock = ClockPath.from_record(record)
        if any(edge.kind == "INDEX_TURN" for edge in clock.edges):
            raise ValueError("A retained local drawing must not contain a table turn")
        clock.edges = [
            replace(edge, stroke=row["id"] if edge.stroke else None, table_index=index)
            for edge in clock.edges
        ]
        clock.metadata.update(
            source_reverse=drawing["reverse"], source_start_visit=drawing["start_visit"]
        )
        clock.metadata["deposition_events"] = derive_deposition_events(
            clock, row, self.build.cooling, self.table.build_transform(index)
        )
        clock.metadata["release_ready_s"] = max(
            event["ready_at_s"] for event in clock.metadata["deposition_events"]
        )
        return MotionProposal(
            row["id"],
            self.build.arm.identity,
            drawing["variant"],
            clock.duration_s,
            drawing["start_anchor"],
            tuple(clock.edges[-1].q1),
            clock,
        )

    def restore(self, reference, row, context, available):
        """Revalidate a retained full motion in its exact material/table state."""
        build = self.build
        try:
            relative = Path(reference["file"])
            if relative.is_absolute() or ".." in relative.parts:
                return None
            path = build.run / relative
            if sha256(path) != reference["sha256"]:
                return None
            cache_key = (row["id"], reference["sha256"], digest(context), frozenset(available))
            # A model instance fixes the graph, profile, tool, limits and cooled
            # predecessor-state semantics. Reuse only a full check completed in
            # this process; a persisted proposal never supplies a certificate.
            if cache_key in self.rechecked:
                return self.rechecked[cache_key]
            saved = read_json(path)
            drawing = saved["drawing"]
            if (
                saved["section"] != row["id"]
                or saved.get("context") != context
                or drawing["stroke_id"] != row["id"]
                or drawing["start_anchor"] not in available
            ):
                return None
            if not self.structure_admissible(row, drawing, context["predecessor"]):
                return None
            clock = ClockPath.from_record(saved["trajectory"])
            check = audit_indexed_clock(self.table, clock, build.initial, build.cooling)
            index = clock.metadata["drawing_table_index"]
            terminal = context["terminal"]
            if (
                check["initial_index"] != index
                or check["final_index"] != terminal[7]
                or not np.allclose(clock.edges[-1].q1, terminal[:7], atol=1e-9, rtol=0)
                or any(edge.table_index != index for edge in clock.edges if edge.kind == "DRAW")
            ):
                return None
            points, _ = flatten_curve(row, 0.00005, 0.003)
            check = audit_source_curve(
                build.arm, [clock], transform_points(points, self.table.build_transform(index))
            )
            if check["status"] != "BOUNDED":
                return None
            scene = self.scene(context["predecessor"], index)
            state = build.state(context["predecessor"], clock.edges[0].q0, scene)
            move = MotionProposal(
                row["id"],
                build.arm.identity,
                drawing["variant"],
                clock.duration_s,
                drawing["start_anchor"],
                tuple(clock.edges[-1].q1),
                clock,
            )
            _, failure = build.planner.validate_interval(
                state, (ScheduledMotion(move, 0),), clock.duration_s
            )
            if failure:
                return None
        except (OSError, KeyError, IndexError, TypeError, ValueError):
            return None
        build.revalidated_backward_motions += 1
        result = (*clock.edges[0].q0, index), (row["id"], move, drawing)
        self.rechecked[cache_key] = result
        return result

    def scene(self, ids, index):
        scene = self.build.scene(ids)
        return self.table.reposition(scene, index_angle(self.table.home_index), index_angle(index))

    def transition(self, scene, start, start_index, end, end_index):
        from .section_build import combine_clocks

        build = self.build
        if start_index == end_index:
            return at_index(build.travel_clock(scene, start, end), start_index)
        travel = build.travel_clock(scene, start, build.initial)
        if travel is False:
            return False
        paths = [at_index(travel, start_index)]
        q = build.initial
        # Accepted local actions already wait until release readiness before
        # withdrawal. The incoming travel also counts as elapsed cooling time;
        # adding a new full cooling delay here would charge the same deposit twice.
        current = start_index
        for target in self.table.steps(start_index, end_index):
            parked_arm = replace(
                build.planner.collision_arm(build.arm.identity),
                collision=build.planner.collision_tip_model(build.arm.identity),
            )
            check = self.table.check_parked_turn(
                scene,
                current,
                target,
                parked_arm,
                q,
                build.planner.obstacles,
                margin_m=build.planner.margin,
                deadline=build.planner.deadline,
            )
            if check["status"] != "CERTIFIED_TURNTABLE_INTERVAL":
                return False
            duration = self.table.move_duration()
            paths.append(
                ClockPath(
                    [
                        ClockEdge(
                            0,
                            duration,
                            q,
                            q,
                            "INDEX_TURN",
                            None,
                            table_index=current,
                            table_to_index=target,
                        ),
                        ClockEdge(
                            duration,
                            duration + self.table.settling_s,
                            q,
                            q,
                            "TABLE_SETTLE",
                            None,
                            table_index=target,
                        ),
                    ]
                )
            )
            scene = self.table.reposition(scene, index_angle(current), index_angle(target))
            current = target
        travel = build.travel_clock(scene, q, end)
        if travel is False:
            return False
        paths.append(at_index(travel, end_index))
        return combine_clocks(*paths)

    def drawings(self, row, available, q, index, scene):
        build, planner = self.build, self.build.planner
        transform = self.table.build_transform(index)
        # Radial orientation changes continuously around a whole contour or
        # zigzag. Try the outward 70/80-degree families before broad repair.
        points, _ = flatten_curve(row, 0.00005, 0.003)
        points = transform_points(points, transform)
        tilts = row.get("pen_tilt_candidates_deg") or planner.profile.get(
            "drawing_pen_tilt_candidates_deg", DEFAULT_PEN_TILTS_DEG
        )
        policy = planner.profile.get("drawing_orientation_policy", "fixed")
        fixed = policy in {"fixed", "fixed_axis"}
        families = list(
            fixed_rotation_families(build.arm, transform, tilts)
            if fixed
            else outward_rotation_families(build.arm, points, transform, tilts)
        )
        preferred = families[0]["rotations"] if fixed else families[0]["rotations"][0]
        fallback = [] if fixed else list(orientation_lattice(preferred))
        outward = points.mean(axis=0) - transform[:3, 3]
        outward[2] = 0
        if np.linalg.norm(outward) > 1e-9:
            outward /= np.linalg.norm(outward)
            fallback.sort(key=lambda rotation: float(rotation[:, 2] @ outward))
        rotations = [*families, *fallback]
        original_deadline = planner.deadline
        choices = []
        try:
            planner.deadline = min(
                original_deadline, monotonic() + min(60, 15 * 2 ** min(self.round, 3))
            )
            for descriptor in rotations:
                if monotonic() >= planner.deadline:
                    break
                rotation = descriptor["rotations"] if isinstance(descriptor, dict) else descriptor
                samples = [0, len(points) // 2, len(points) - 1]
                if all(
                    pen_clear(
                        planner,
                        build.arm,
                        points[sample],
                        rotation[sample] if rotation.ndim == 3 else rotation,
                        scene,
                        planner.margin + 0.000002,
                    )
                    for sample in samples
                ):
                    choices.append(descriptor)
                    if len(choices) >= min(len(rotations), 12 * 2 ** min(self.round, 8)):
                        break
        finally:
            planner.deadline = original_deadline
        for drawing in drawing_alternatives(
            planner.collision_arm(build.arm.identity),
            row,
            available,
            transform,
            planner.obstacles,
            deadline=min(planner.deadline, monotonic() + min(120, 30 * 2 ** min(self.round, 8))),
            seeds=[q, build.initial],
            settings={"maximum_subdivision_depth": 20},
            clearance_m=planner.margin,
            rotations=choices[: min(len(choices), 12 * 2 ** min(self.round, 8))],
            ik_backend=build.gpu,
            orientation_policy=policy,
        ):
            if drawing["status"] == "SAMPLED_DRAWING_PATH":
                drawing.update(
                    release_cooling_s=build.cooling,
                    table_index=index,
                    collision_scope=planner.collision_scope,
                    requires_full_robot_validation=planner.collision_scope == "pen_candidate",
                )
                if planner.collision_scope == "pen_candidate":
                    drawing["static_collision_status"] = "CHECKED_PEN_STATIC_ONLY"
                yield drawing
            elif self.experience is not None:
                self.experience.sampling_failure(row, points, index, scene, drawing)
        if fixed:
            return
        # Free-axis solving is also available when no fixed-orientation path
        # exists. Repeated joints here are only seeds, never a motion proposal.
        for oriented in anchored_strokes(row, available):
            if monotonic() >= planner.deadline:
                return
            targets, _ = flatten_curve(oriented, 0.0001, 0.003)
            targets = transform_points(targets, transform)
            drawing = {
                "arm": build.arm.identity,
                "stroke_id": row["id"],
                "start_anchor": oriented["start"],
                "end_anchor": oriented["end"],
                "reverse": oriented["reverse"],
                "start_visit": oriented["start_visit"],
                "variant": f"indexed-{index}-free-{oriented['start_visit']}-{oriented['reverse']}",
                "tcp_targets_m": targets.tolist(),
                "tcp_rotation": radial_pen_rotations(build.arm, targets, transform, tilts[0])[
                    0
                ].tolist(),
                "tcp_rotations": radial_pen_rotations(
                    build.arm, targets, transform, tilts[0]
                ).tolist(),
                "orientation_policy": "outward_radial",
                "pen_tilt_deg": float(tilts[0]),
                "positions": np.tile(q, (len(targets), 1)).tolist(),
                "release_cooling_s": build.cooling,
                "table_index": index,
            }
            original_deadline = planner.deadline
            try:
                planner.deadline = min(
                    original_deadline, monotonic() + min(120, 30 * 2 ** min(self.round, 3))
                )
                repair = repair_drawing_axes(planner, drawing, scene)
            finally:
                planner.deadline = original_deadline
            if repair["status"] == "SAMPLED_FREE_AXIS_PROPOSAL":
                drawing.update(
                    positions=repair["positions"],
                    tcp_targets_m=repair["tcp_targets_m"],
                    free_tool_orientation=True,
                )
                yield drawing
            elif self.experience is not None:
                self.experience.sampling_failure(row, targets, index, scene, repair)

    def forward_motions(self, row, state):
        """Plan against actual preceding deposits rather than future envelopes.

        This is the indexed backend for the existing supplied-order forward
        search. All local, incoming and turning motions receive the usual full
        checks before their resulting material can enter the next state.
        """
        from .section_build import combine_clocks

        build, planner = self.build, self.build.planner
        completed, available = state.completed, state.ready_nodes()
        if row["stage"] == "artistic" and any(
            action["stage"] == "framework" and key not in completed
            for key, action in build.actions.items()
        ):
            return
        start = np.asarray(dict(state.configurations)[build.arm.identity])
        start_index = state.table_index
        proposal_budget = None if self.experience is None else self.experience.budget(row)

        def candidates(index):
            scene = self.table.reposition(
                state.payload, index_angle(start_index), index_angle(index)
            )
            if self.experience is not None:
                for drawing in self.experience.candidates(
                    row, available, index, scene, reference_q=start, budget=proposal_budget
                ):
                    yield scene, drawing, self.retained_move(row, drawing, index)
            for move, drawing in self.warm.get((row["id"], index), []):
                if drawing["start_anchor"] in available:
                    yield scene, drawing, move
            for drawing in self.drawings(row, available, start, index, scene):
                yield scene, drawing, None

        indices = (
            self.experience.preferred_indices(row, start_index)
            if self.experience is not None
            else [start_index, *[i for i in range(4) if i != start_index]]
        )
        indices.sort(key=lambda index: not self.warm.get((row["id"], index)))
        pending = deque((index, iter(candidates(index)), 0) for index in indices)
        while pending and monotonic() < planner.deadline:
            index, iterator, sampled = pending.popleft()
            proposal_started = monotonic()
            try:
                scene, drawing, warm_move = next(iterator)
            except StopIteration:
                continue
            stages = {"proposal": monotonic() - proposal_started}
            drawing["indexed_stage_seconds"] = stages
            drawing["indexed_material_context"] = {
                "origin": "actual_preceding_deposition_repositioned",
                "capsules": len(scene.segments),
                "table_index": scene.table_index,
                "sha256": digest({"capsules": scene.segments, "table_index": scene.table_index}),
            }
            reused = warm_move is not None or drawing.get("proposal_source") in {
                "cache",
                "diffusion",
            }
            count = sampled + (not reused)
            if reused or count < 2 ** min(self.round, 8):
                pending.append((index, iterator, count))
            original_deadline = planner.deadline
            candidate_started = monotonic()
            try:
                planner.deadline = min(original_deadline, monotonic() + 240)
                if reused and proposal_budget is not None:
                    if proposal_budget.remaining_seconds <= 1e-6:
                        continue
                    planner.deadline = min(
                        original_deadline,
                        monotonic() + proposal_budget.remaining_seconds,
                    )
                if not self.structure_admissible(row, drawing, completed):
                    continue
                local_state = build.state(completed, build.initial, scene)
                prepare_started = monotonic()
                move = warm_move or planner.prepare_drawing(
                    local_state,
                    build.arm.identity,
                    drawing,
                    None,
                    defer_drawing_check=True,
                )
                drawing["preparation_seconds"] = monotonic() - prepare_started
                stages["prepare"] = drawing["preparation_seconds"]
                if move is None:
                    stages["total"] = monotonic() - proposal_started
                    if self.experience is not None:
                        self.experience.observe(drawing, scene, failure=planner.attempts[-1:])
                    continue
                local = at_index(move.payload, index)
                stage_started = monotonic()
                incoming = self.transition(
                    state.payload, start, start_index, local.edges[0].q0, index
                )
                stages["incoming"] = monotonic() - stage_started
                if incoming is False:
                    continue
                path = combine_clocks(incoming, local)
                path.metadata["drawing_table_index"] = index
                audit_indexed_clock(self.table, path, build.initial, build.cooling)
                points, _ = flatten_curve(row, 0.00005, 0.003)
                stage_started = monotonic()
                source = audit_source_curve(
                    build.arm, [path], transform_points(points, self.table.build_transform(index))
                )
                stages["source_coverage"] = monotonic() - stage_started
                if source["status"] != "BOUNDED":
                    planner.failure(row["id"], build.arm.identity, "INDEXED_SOURCE", "REJECTED")
                    continue
                combined = replace(move, payload=path, duration_s=path.duration_s)
                stage_started = monotonic()
                after, failure = planner.validate_interval(
                    state, (ScheduledMotion(combined, 0),), combined.duration_s
                )
                stages["final_clock_validation"] = monotonic() - stage_started
                if failure:
                    stages["total"] = monotonic() - proposal_started
                    planner.failure(
                        row["id"], build.arm.identity, "INDEXED_REPLAY", failure["status"]
                    )
                    if self.experience is not None:
                        self.experience.observe(drawing, scene, failure=failure)
                    continue
            finally:
                stages["total"] = monotonic() - proposal_started
                if reused and proposal_budget is not None:
                    proposal_budget.spend(monotonic() - candidate_started)
                planner.deadline = original_deadline
            if self.experience is not None:
                # The recorded scene is already at the drawing index. Store
                # only the matching local clock, excluding incoming turns.
                self.experience.observe(
                    drawing,
                    scene,
                    move=replace(move, payload=local, duration_s=local.duration_s),
                )
            yield combined, drawing, after

    def motions(self, row, predecessor, available, terminal):
        from .section_build import combine_clocks

        build, planner = self.build, self.build.planner
        target, target_index = np.asarray(terminal[:7]), int(terminal[7])
        context = {"predecessor": sorted(predecessor), "terminal": list(terminal)}
        key = digest([row["id"], context])
        proposal_budget = None if self.experience is None else self.experience.budget(row)
        for reference in self.records.get(key, []):
            restored = self.restore(reference, row, context, available)
            if restored is not None:
                yield restored

        def candidates(index):
            scene = self.scene(predecessor, index)
            state = build.state(predecessor, build.initial, scene)
            if self.experience is not None:
                for drawing in self.experience.candidates(
                    row,
                    available,
                    index,
                    scene,
                    reference_q=target,
                    reference_endpoint="end",
                    budget=proposal_budget,
                ):
                    yield scene, state, drawing, self.retained_move(row, drawing, index)
            for move, drawing in self.warm.get((row["id"], index), []):
                if drawing["start_anchor"] in available:
                    yield scene, state, drawing, move
            for drawing in self.drawings(row, available, target, index, scene):
                yield scene, state, drawing, None

        indices = (
            self.experience.preferred_indices(row, target_index)
            if self.experience is not None
            else [target_index, *[i for i in range(4) if i != target_index]]
        )
        pending = deque((index, iter(candidates(index)), 0) for index in indices)
        while pending and monotonic() < planner.deadline:
            index, iterator, sampled = pending.popleft()
            try:
                scene, state, drawing, warm_move = next(iterator)
            except StopIteration:
                planner.failure(
                    row["id"],
                    build.arm.identity,
                    "INDEXED_SAMPLING",
                    "NO_MORE_SAMPLED_DRAWINGS",
                    table_index=index,
                    sampling_round=self.round,
                )
                yield None
                continue
            # Let another source/order run after bounded effort at this state.
            # Later rounds widen the sampling budget without declaring the
            # earlier finite sample to be an infeasibility proof.
            reused = warm_move is not None or drawing.get("proposal_source") in {
                "cache",
                "diffusion",
            }
            count = sampled + (not reused)
            if reused or count < 2 ** min(self.round, 8):
                pending.append((index, iterator, count))
            original_deadline = planner.deadline
            candidate_started = monotonic()
            failures_before = len(planner.attempts)
            try:
                planner.deadline = min(
                    original_deadline, monotonic() + min(240, 60 * 2 ** min(self.round, 3))
                )
                if reused and proposal_budget is not None:
                    if proposal_budget.remaining_seconds <= 1e-6:
                        yield None
                        continue
                    planner.deadline = min(
                        original_deadline,
                        monotonic() + proposal_budget.remaining_seconds,
                    )
                if not self.structure_admissible(row, drawing, predecessor):
                    yield None
                    continue
                candidate_deadline = planner.deadline
                prepare_started = monotonic()
                move = warm_move or planner.prepare_drawing(
                    state, build.arm.identity, drawing, None
                )
                drawing["preparation_seconds"] = monotonic() - prepare_started
            finally:
                if reused and proposal_budget is not None:
                    proposal_budget.spend(monotonic() - candidate_started)
                planner.deadline = original_deadline
                for failure in planner.attempts[failures_before:]:
                    failure["table_index"] = index
            if move is None:
                if self.experience is not None:
                    self.experience.observe(drawing, scene, failure=planner.attempts[-1:])
                yield None
                continue
            validation_started = monotonic()
            try:
                if reused and proposal_budget is not None:
                    if proposal_budget.remaining_seconds <= 1e-6:
                        yield None
                        continue
                    planner.deadline = min(
                        candidate_deadline, monotonic() + proposal_budget.remaining_seconds
                    )
                path = at_index(move.payload, index)
                move = replace(move, payload=path)
                after, failure = planner.validate_interval(
                    state, (ScheduledMotion(move, 0),), move.duration_s
                )
                if failure:
                    planner.failure(
                        row["id"],
                        build.arm.identity,
                        "INDEXED_REPLAY",
                        failure["status"],
                        table_index=index,
                        validation=failure,
                    )
                    if self.experience is not None:
                        self.experience.observe(drawing, scene, failure=failure)
                    yield None
                    continue
                if self.experience is not None:
                    self.experience.observe(drawing, scene, move=move)
                travel = self.transition(after, move.end_configuration, index, target, target_index)
                if travel is False:
                    yield None
                    continue
            finally:
                if reused and proposal_budget is not None:
                    proposal_budget.spend(monotonic() - validation_started)
                planner.deadline = original_deadline
            path = combine_clocks(path, travel)
            path.metadata["drawing_table_index"] = index
            audit_indexed_clock(self.table, path, build.initial, build.cooling)
            combined = replace(
                move, payload=path, duration_s=path.duration_s, end_configuration=tuple(target)
            )
            reference = build.checkpoint_witness(
                row["id"],
                combined,
                drawing,
                directory="indexed_witnesses",
                context=context,
            )
            records = self.records.setdefault(key, [])
            self.record_ranks.setdefault(key, len(self.record_ranks))
            if reference not in records:
                records.append(reference)
                write_json(
                    self.checkpoint,
                    {
                        "status": "PROPOSALS_REQUIRE_REVALIDATION",
                        "binding": self.binding,
                        "witnesses": self.records,
                        "sampling_round": self.round,
                    },
                )
            yield (*path.edges[0].q0, index), (row["id"], combined, drawing)


def run_indexed_forward(build):
    """Use the existing forward mode search with the actual indexed material."""
    from .full_build_export import export_sections

    build.report("INDEXED_FORWARD_ORDER_RUNNING")
    order = greedy_forward_order(
        build.actions.values(), build.planner.ground, build.stable, deadline=build.planner.deadline
    )
    if order is None:
        build.report(
            "SEARCH_TIME_BUDGET"
            if monotonic() >= build.planner.deadline
            else "NO_GREEDY_FORWARD_ORDER"
        )
        return
    build.search_metadata = {
        "strategy": "greedy_supported_order_with_motion_backtracking",
        "material": "actual_preceding_deposition_enclosures",
    }
    write_json(
        build.run / "forward_order.json",
        {
            "binding": build.checkpoint_binding,
            "order": [row["id"] for row in order],
            "scope": "One support/stiffness-guided order; motion alternatives can backtrack, "
            "but this forward mode does not search alternate construction orders.",
        },
    )
    model = IndexedMotions(build)
    model.round = max(model.round, 3)
    build.report("INDEXED_FORWARD_RUNNING")
    result = build.forward(order, indexed_model=model)
    if result is None:
        build.report("INDEXED_FORWARD_INCOMPLETE")
        return
    export_sections(build, *result)


def run_indexed(build):
    from .full_build_export import export_sections
    from .section_build import combine_clocks

    model = IndexedMotions(build)

    def incoming(state):
        scene = MaterialScene()
        scene.table_index = model.table.home_index
        path = model.transition(
            scene, build.initial, model.table.home_index, np.asarray(state[:7]), int(state[7])
        )
        return None if path is False else False if path is None else path

    build.report("INDEXED_SEARCH_RUNNING")
    rounds = []
    overall_deadline = build.planner.deadline
    while monotonic() < overall_deadline:
        started = monotonic()
        # A difficult first branch must not consume the entire job budget.
        # Preserve and revalidate its witnesses while widening later rounds.
        build.planner.deadline = min(overall_deadline, started + 300 * 2 ** min(model.round, 8))
        try:
            result = search_regression(
                list(build.actions.values()),
                build.planner.graph,
                build.material,
                (*build.initial, model.table.home_index),
                model.motions,
                incoming,
                deadline=build.planner.deadline,
                maximum_states=build.planner.settings["maximum_states"] * 2 ** min(model.round, 8),
                motion_priority=model.motion_priority,
                progress=lambda depth, attempts, section: build.report(
                    "INDEXED_SEARCH_RUNNING",
                    depth=depth,
                    attempts=attempts,
                    section=section,
                    sampling_round=model.round,
                ),
            )
        finally:
            build.planner.deadline = overall_deadline
        rounds.append(
            {
                **{key: value for key, value in result.items() if key != "result"},
                "sampling_round": model.round,
                "wall_seconds": monotonic() - started,
            }
        )
        write_json(build.run / "indexed_search_rounds.json", rounds)
        if result["status"] in {"FOUND", "GOAL_STRUCTURE_REJECTED"}:
            break
        model.round += 1
        write_json(
            model.checkpoint,
            {
                "status": "PROPOSALS_REQUIRE_REVALIDATION",
                "binding": model.binding,
                "witnesses": model.records,
                "sampling_round": model.round,
            },
        )
    if not rounds:
        build.report("SEARCH_TIME_BUDGET")
        return
    build.search_metadata = {key: value for key, value in result.items() if key != "result"}
    build.search_metadata["sampling_rounds"] = rounds
    if result["status"] != "FOUND":
        build.report(result["status"])
        return
    incoming_path, witnesses = result["result"]
    key, move, drawing = witnesses[0]
    path = combine_clocks(None if incoming_path is False else incoming_path, move.payload)
    witnesses[0] = (key, replace(move, payload=path, duration_s=path.duration_s), drawing)
    write_json(build.run / "indexed_search.json", build.search_metadata)
    export_sections(build, witnesses, None)
