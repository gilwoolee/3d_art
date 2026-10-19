"""Indexed-plate motion callbacks for the existing construction search."""

from dataclasses import replace
from time import monotonic

import numpy as np

from ..curves import flatten_curve, transform_points
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
from .turntable import IndexedTurntable, index_angle


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
        self.round = 0


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
