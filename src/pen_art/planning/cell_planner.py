"""Couple shared-clock arm motions, grounded strokes and growing material.

All output is simulation-only. Missing pen geometry/flow calibration and material
strength remain explicit; geometric acceptance is not fabrication acceptance.
"""

import argparse
import hashlib
import json
from dataclasses import replace
from itertools import permutations
from pathlib import Path
from time import monotonic

import fcl
import numpy as np
import yaml

from ..cell import load_cell
from ..io import artifact, read_json, sha256, stage_update, write_json
from ..kinematics import inverse_candidates
from ..timing import (
    TimingParameterizationError,
    make_path,
    refine_samples,
    tcp_acceleration_bound,
)
from .axis_repair import (
    AXIS_REPAIR_REVISION,
    clear_intermediate_poses,
    refine_axis_path,
    repair_drawing_axes,
)
from .cell_motion import drawing_alternatives
from .clock_path import timed_path
from .collision import Body, RobotCollisionModel
from .coordinated_motion import check_pair_linear, check_pair_paths
from .dual_arm import (
    MotionProposal,
    ScheduledMotion,
    ScheduleState,
    search_schedule,
    tasks_from_construction,
)
from .rrt import connect
from .scene import MaterialScene
from .smooth_audit import CubicPiece, bounded_segments, check_tracking
from .tracking import tracking_bounds
from .transit import contact_validator, find_departure, local_contact_surfaces


def tracked_drawing_path(arm, drawing, continuous, deadline):
    """Refine only the interpolant when a fixed drawing path misses tracking bounds."""
    original_q = np.asarray(drawing["positions"])
    free_axis = drawing.get("free_tool_orientation", False)
    if free_axis and not continuous:
        raise ValueError("Free-axis drawings require continuous position tracking")
    original_targets = np.repeat(np.eye(4)[None], len(original_q), axis=0)
    original_targets[:, :3, :3] = drawing["tcp_rotation"]
    original_targets[:, :3, 3] = drawing["tcp_targets_m"]
    if continuous:
        keep = np.r_[True, np.linalg.norm(np.diff(original_q, axis=0), axis=1) > 1e-10]
        original_q, original_targets = original_q[keep], original_targets[keep]
        points = np.array([arm.robot.forward(q, arm.flange_to_tcp)[:3, 3] for q in original_q])
    for subdivisions in (2, 4, 8) if continuous else (1,):
        spline, pieces = None, []
        q, targets = original_q, original_targets
        if continuous:
            spline = make_path(q, points, interpolation="pchip", subdivisions=subdivisions)
            q, targets = refine_samples(q, subdivisions), refine_samples(targets, subdivisions)
            pieces = [
                CubicPiece(spline.cspl.c[:, i, :], width)
                for i, width in enumerate(np.diff(spline.cspl.x))
            ]
        worst, worst_angle, failed = 0.0, 0.0, None
        for i, (a, b) in enumerate(zip(q[:-1], q[1:], strict=True)):
            if monotonic() >= deadline:
                return {"status": "TIME_BUDGET"}
            check = (
                check_tracking(
                    pieces[i],
                    arm.robot,
                    arm.flange_to_tcp,
                    targets[i, :3, 3],
                    targets[i + 1, :3, 3],
                    None if free_axis else targets[i, :3, :3],
                    0.0003,
                    0.003,
                )
                if continuous
                else tracking_bounds(arm.robot, arm.flange_to_tcp, a, b, targets[i], targets[i + 1])
            )
            worst = max(worst, check["position_error_bound_m"])
            worst_angle = max(worst_angle, check["orientation_error_bound_rad"])
            if worst > 0.0003 or worst_angle > 0.003:
                failed = check
                break
        if failed is None:
            return {
                "status": "BOUNDED",
                "positions": q,
                "targets": targets,
                "spline": spline,
                "pieces": pieces,
                "subdivisions": subdivisions,
                "position_error_bound_m": worst,
                "orientation_error_bound_rad": worst_angle,
                "orientation_constraint": "free" if free_axis else "fixed",
            }
    return {**failed, "status": "BOUND_EXCEEDS_TOLERANCE"}


class CellPlanner:
    def __init__(self, run, profile, settings, deadline, *, timing_override=None):
        self.run, self.settings, self.deadline = Path(run), settings, deadline
        self.profile, self.arms, self.obstacles = load_cell(profile)
        self.cell = read_json(self.run / "cell_manifest.json")
        if self.profile != read_json(self.run / "cell_config.json"):
            raise ValueError("Cell configuration changed; rebuild the setup")
        for name, digest in self.cell["inputs"].items():
            if sha256(self.run / name) != digest:
                raise ValueError(f"Stale cell input: {name}")
        self.candidates = read_json(self.run / "cell_drawing_variants.json")
        for name, digest in self.candidates["inputs"].items():
            if sha256(self.run / name) != digest:
                raise ValueError(f"Stale drawing candidate input: {name}")
        construction = read_json(self.run / "construction_plan.json")
        self.actions = {a["id"]: a for a in construction["actions"]}
        self.graph = read_json(self.run / "stroke_graph.json")
        self.tasks, self.ground = tasks_from_construction(
            construction, self.graph, settings["assumed_cooling_s"]
        )
        source = Path(read_json(self.run / "run.json")["cell_source_run"])
        self.process = read_json(source / "fabrication_config.json")["process"]
        self.transit = yaml.safe_load(Path("configs/transit.yaml").read_text())
        self.timing = (
            timing_override
            if timing_override is not None
            else yaml.safe_load(Path("configs/timing.yaml").read_text())
        )
        self.margin = self.profile["environment"]["clearance_m"]
        self.tip_models = {}
        self.drawings = {}
        self.cache = {}
        self.attempts = []
        self.checked_intervals = 0
        self.enabled_arms = settings.get("enabled_arms", list(self.arms))
        if not self.enabled_arms or not set(self.enabled_arms) <= self.arms.keys():
            raise ValueError("Enabled arms must name at least one arm in the shared cell")
        for identity, arm in self.arms.items():
            # Verify live inputs used by load_cell against the stored snapshot.
            for suffix, path in (("robot.urdf", arm.urdf), ("robot.json", arm.manifest_path)):
                if sha256(path) != sha256(self.run / f"{identity}_{suffix}"):
                    raise ValueError(f"Live robot changed: {identity}")
            saved = next(a for a in self.cell["arms"] if a["id"] == identity)
            if arm.tool.metadata != saved["tool_model"]:
                raise ValueError(f"Live tool changed: {identity}")
            radius = arm.tool.tip_radius_m or self.transit["virtual_tip_radius_m"]
            tip = Body("virtual_tip", arm.robot.tip, arm.flange_to_tcp, fcl.Sphere(radius), radius)
            self.tip_models[identity] = RobotCollisionModel(arm.robot, [*arm.tool.bodies, tip])
        for row in self.candidates["variants"]:
            self.drawings.setdefault((row["arm"], row["stroke_id"]), []).append(row)
        self.initial_pose_repairs = self.choose_initial_poses(source)

    def choose_initial_poses(self, source):
        """Select collision-free staged job starts, including the known tip.

        An imported home posture may lie inside the raised platform. It cannot
        be used as a collision-free path origin. Staging at the selected start
        is an explicit job precondition, not a fictitious transit through it.
        """
        contacts = {
            identity: self.tip_models[identity].check(
                arm.initial_positions, self.obstacles, self.margin
            )
            for identity, arm in self.arms.items()
        }
        if not any(contacts.values()):
            return {}
        center = np.asarray(self.profile["world_from_build"])[:3, 3]
        source_motion = source / "sequence_motion.json"
        parking = (
            read_json(source_motion).get("parking_pose", {}).get("positions")
            if source_motion.exists()
            else None
        )
        choices = {}
        for identity, arm in self.arms.items():
            seeds = [arm.initial_positions]
            if parking is not None:
                seeds.insert(0, np.asarray(parking))
            choices[identity] = []
            direction = arm.robot.world_from_base[:2, 3] - center[:2]
            direction /= np.linalg.norm(direction)
            # Keep an idle wrist away from the shared drawing volume. Near-tip
            # staging at 10 cm cleared the platform but blocked the other arm.
            for distance in (0.30, 0.40, 0.16, 0.10):
                target = np.eye(4)
                target[:3, :3] = arm.robot.world_from_base[:3, :3] @ np.diag([-1.0, 1.0, -1.0])
                target[:3, 3] = center + np.r_[direction * distance, 0.14]
                for solution in inverse_candidates(arm.robot, target, arm.flange_to_tcp, seeds):
                    q = np.asarray(solution["positions"])
                    if not self.tip_models[identity].check(q, self.obstacles, self.margin):
                        choices[identity].append(q)
            if not choices[identity]:
                raise ValueError(f"No clear staged start found for {identity}")
        from itertools import product

        identities = list(self.arms)
        for candidate in product(*(choices[identity] for identity in identities)):
            if len(identities) == 2:
                first, second = identities
                check = check_pair_linear(
                    self.tip_models[first],
                    self.tip_models[second],
                    candidate[0],
                    candidate[0],
                    candidate[1],
                    candidate[1],
                    margin_m=self.margin,
                    deadline=self.deadline,
                )
                if check["status"] != "CERTIFIED_INTER_ARM_INTERVAL":
                    continue
            for identity, q in zip(identities, candidate, strict=True):
                self.arms[identity].initial_positions = q
            return {
                "source_contacts": contacts,
                "precondition": "Start the job with both robots at these clear staged poses",
            }
        raise ValueError("No collision-free paired start poses found")

    def failure(self, task, arm, phase, status, **extra):
        print(f"Rejected {arm}/{task}: {phase} / {status}", flush=True)
        self.attempts.append(
            {"stroke_id": task, "arm": arm, "phase": phase, "status": status, **extra}
        )
        del self.attempts[:-100]

    def material_key(self, scene):
        return hashlib.sha256(json.dumps(scene.segments, sort_keys=True).encode()).hexdigest()

    def edge_check(self, identity, scene, first, second):
        return (
            self.tip_models[identity].check_edge(
                first,
                second,
                self.obstacles,
                self.margin,
                self.settings["maximum_subdivision_depth"],
                scene,
                deadline=self.deadline,
            )["status"]
            == "CERTIFIED_MODELED_EDGE"
        )

    def travel(self, identity, scene, first, second, seed=42):
        arm = self.arms[identity]
        return connect(
            first,
            second,
            arm.robot.lower,
            arm.robot.upper,
            lambda a, b: self.edge_check(identity, scene, a, b),
            seed=seed,
            step_rad=self.transit["rrt_step_rad"],
            maximum_nodes=self.transit["rrt_maximum_nodes"],
            maximum_seconds=min(
                self.transit["rrt_maximum_seconds"], max(0.001, self.deadline - monotonic())
            ),
        )

    def lift_check(self, identity, scene, q):
        arm = self.arms[identity]
        contact = arm.robot.forward(q, arm.flange_to_tcp)[:3, 3]
        return contact_validator(
            arm.robot,
            arm.collision,
            self.tip_models[identity],
            arm.flange_to_tcp,
            scene,
            self.obstacles,
            contact,
            self.transit["contact_radius_m"],
            arm.tool.tip_radius_m or self.transit["virtual_tip_radius_m"],
            self.margin,
            self.settings["maximum_subdivision_depth"],
            local_contact_surfaces(
                contact[2],
                self.profile["environment"]["table_z_m"],
                self.profile["world_from_build"][2][3],
                self.transit["contact_radius_m"],
            ),
            deadline=self.deadline,
        )

    def add_draw(self, scene, identity, q0, q1, radius, name):
        arm = self.arms[identity]
        a, b = [arm.robot.forward(q, arm.flange_to_tcp)[:3, 3] for q in (q0, q1)]
        inflation = (
            tcp_acceleration_bound(arm.robot, arm.flange_to_tcp, abs(q1 - q0), np.zeros(len(q0)))
            / 8
        )
        scene.add(a, b, radius + inflation + 1e-9, name)

    def add_piece(self, scene, identity, piece, radius, name):
        arm = self.arms[identity]
        for index, (a, b, inflation) in enumerate(
            bounded_segments(piece, arm.robot, arm.flange_to_tcp, 0.00002)
        ):
            ends = [arm.robot.forward(piece.position(t), arm.flange_to_tcp)[:3, 3] for t in (a, b)]
            scene.add(*ends, radius + inflation + 1e-9, f"{name}/{index}")

    def add_clock_edge(self, scene, identity, edge, name, start=None, end=None):
        start = edge.start_s if start is None else start
        end = edge.end_s if end is None else end
        if edge.coefficients is None:
            self.add_draw(
                scene, identity, edge.position(start), edge.position(end), edge.radius_m, name
            )
        else:
            self.add_piece(scene, identity, edge.piece(start, end), edge.radius_m, name)

    def prepare(self, state, task, identity):
        if identity not in self.enabled_arms or monotonic() >= self.deadline:
            return []
        arm = self.arms[identity]
        q_start = np.array(dict(state.configurations)[identity])
        available = task.anchors & state.ready_nodes()
        drawings = [
            row
            for row in self.drawings.get((identity, task.identity), [])
            if row["start_anchor"] in available
        ]
        # Reversal exposes the other endpoint without imposing serial ownership.
        for row in self.drawings.get((identity, task.identity), []):
            if row["end_anchor"] in available and row["start_anchor"] != row["end_anchor"]:
                drawings.append(
                    {
                        **row,
                        "variant": row["variant"] + "-reversed",
                        "start_anchor": row["end_anchor"],
                        "end_anchor": row["start_anchor"],
                        "positions": row["positions"][::-1],
                        "tcp_targets_m": row["tcp_targets_m"][::-1],
                    }
                )
        if not drawings:
            for row in drawing_alternatives(
                arm,
                self.actions[task.identity],
                available,
                np.asarray(self.profile["world_from_build"]),
                self.obstacles,
                deadline=self.deadline,
                seeds=[q_start],
                clearance_m=self.margin,
            ):
                if row["status"] == "SAMPLED_DRAWING_PATH":
                    drawings.append(row)
                    self.drawings.setdefault((identity, task.identity), []).append(row)
                if len(drawings) >= self.settings["alternatives_per_dispatch"]:
                    break
        proposals = []
        for drawing in drawings[: self.settings["alternatives_per_dispatch"]]:
            if monotonic() >= self.deadline:
                break
            key = (
                identity,
                task.identity,
                drawing["variant"],
                tuple(q_start),
                self.material_key(state.payload),
            )
            if key in self.cache:
                if self.cache[key] is not None:
                    proposals.append(self.cache[key])
                continue
            proposal = self.prepare_drawing(state, identity, drawing, q_start)
            self.cache[key] = proposal
            if proposal is not None:
                proposals.append(proposal)
        return proposals

    def prepare_idle(self, state, identity):
        """Allow a finished arm to vacate the shared volume while its peer draws."""
        arm = self.arms[identity]
        q = np.asarray(dict(state.configurations)[identity])
        if np.allclose(q, arm.initial_positions, atol=1e-8, rtol=0):
            return []
        key = (identity, "park", tuple(q), self.material_key(state.payload))
        if key not in self.cache:
            result = self.travel(identity, state.payload, q, arm.initial_positions)
            proposal = None
            if result["status"] == "CONNECTED":
                path = timed_path(
                    [{"kind": "PARK", "positions": result["positions"]}],
                    arm,
                    arm.robot.velocity * self.timing["velocity_fraction"],
                    np.asarray(self.timing["joint_acceleration_rad_s2"]),
                    self.process["nominal_speed_m_s"],
                    self.timing["travel_speed_m_s"],
                )
                proposal = MotionProposal(
                    f"park-{identity}",
                    identity,
                    "park",
                    path.duration_s,
                    "parking",
                    tuple(arm.initial_positions),
                    path,
                    auxiliary=True,
                )
            self.cache[key] = proposal
        return [] if self.cache[key] is None else [self.cache[key]]

    def prepare_drawing(self, state, identity, drawing, q_start):
        arm = self.arms[identity]
        attempt = self._prepare_drawing(state, identity, drawing, q_start)
        if attempt is not None or not self.timing.get("repair_drawing_axis", len(self.arms) == 1):
            return attempt
        if drawing.get("free_tool_orientation") or monotonic() >= self.deadline:
            return None
        witness = self.attempts[-1] if self.attempts else {}
        if witness.get("phase") not in {"APPROACH", "DRAW", "WITHDRAW"}:
            return None
        binding = {
            "axis_solver_revision": AXIS_REPAIR_REVISION,
            "cell_sha256": sha256(self.run / "cell_manifest.json"),
            "construction_sha256": sha256(self.run / "construction_plan.json"),
            "source_drawing_sha256": hashlib.sha256(
                json.dumps(drawing, sort_keys=True).encode()
            ).hexdigest(),
            "material_sha256": self.material_key(state.payload),
        }
        repair_path = self.run / "axis_repairs" / f"{identity}_{drawing['stroke_id']}.json"
        prior = read_json(repair_path) if repair_path.exists() else {}
        if prior.get("status") == "SAMPLED_FREE_AXIS_PROPOSAL" and all(
            prior.get(key) == value for key, value in binding.items()
        ):
            repair = prior
            print(
                f"Restored pen-orientation proposal for {identity}/{drawing['stroke_id']}",
                flush=True,
            )
        else:
            print(f"Repairing pen orientation for {identity}/{drawing['stroke_id']}", flush=True)
            repair = repair_drawing_axes(
                self,
                drawing,
                state.payload,
                progress=lambda done, total, _: print(
                    f"Axis repair {identity}/{drawing['stroke_id']}: {done}/{total}", flush=True
                ),
            )
        report = {
            **repair,
            **binding,
            "stroke_id": drawing["stroke_id"],
            "arm": identity,
            "trigger": witness,
        }
        report.pop("continuous_motion_status", None)
        write_json(repair_path, report)
        if repair["status"] != "SAMPLED_FREE_AXIS_PROPOSAL":
            self.failure(
                drawing["stroke_id"],
                identity,
                "AXIS_REPAIR",
                repair["status"],
                sample=repair.get("sample"),
            )
            return None
        repaired = {
            **drawing,
            "positions": repair["positions"],
            "tcp_targets_m": repair["tcp_targets_m"],
            "free_tool_orientation": True,
        }
        # Projection can stop at a joint limit before reaching its tip target.
        # Repair those samples explicitly; subdividing a wrong endpoint cannot
        # make it follow the intended stroke.
        if any(
            np.linalg.norm(arm.robot.forward(q, arm.flange_to_tcp)[:3, 3] - np.asarray(point))
            > 0.000001
            for q, point in zip(repaired["positions"], repaired["tcp_targets_m"], strict=True)
        ):
            corrected = clear_intermediate_poses(
                self, drawing, repaired["positions"], repaired["tcp_targets_m"], state.payload
            )
            if corrected["status"] != "SAMPLED_FREE_AXIS_PROPOSAL":
                report["projection_failure"] = corrected
                write_json(repair_path, report)
                return None
            repaired.update(
                positions=corrected["positions"], tcp_targets_m=corrected["tcp_targets_m"]
            )
            report.update(
                positions=corrected["positions"], tcp_targets_m=corrected["tcp_targets_m"]
            )
            write_json(repair_path, report)
        for refinement in range(5):
            attempt = self._prepare_drawing(state, identity, repaired, q_start)
            if attempt or self.attempts[-1]["phase"] != "TRACKING" or refinement == 4:
                break
            # Pose repair can move a sample enough to invalidate the original
            # interpolation density. Refine the same Cartesian polyline and
            # recheck its growing-material context before retrying the spline.
            positions, targets = refine_axis_path(
                self.arms[identity],
                repaired["positions"],
                repaired["tcp_targets_m"],
                joint_step=0.025 / 2**refinement,
            )
            print(
                f"Refining repaired path {identity}/{drawing['stroke_id']}: "
                f"pass {refinement + 1}, {len(positions)} poses",
                flush=True,
            )
            finer = clear_intermediate_poses(self, drawing, positions, targets, state.payload)
            if finer["status"] != "SAMPLED_FREE_AXIS_PROPOSAL":
                report["refinement_failure"] = finer
                break
            repaired.update(positions=finer["positions"], tcp_targets_m=finer["tcp_targets_m"])
            report.update(
                positions=finer["positions"],
                tcp_targets_m=finer["tcp_targets_m"],
                interpolation_refinements=report.get("interpolation_refinements", 0) + 1,
            )
            write_json(repair_path, report)
        report["continuous_motion_status"] = "ACCEPTED" if attempt else "REJECTED"
        if attempt:
            attempt.payload.metadata["axis_repair_artifact"] = str(
                repair_path.relative_to(self.run)
            )
        write_json(repair_path, report)
        # Bind the final report, including continuous-check outcome.
        if attempt:
            attempt.payload.metadata["axis_repair_sha256"] = sha256(repair_path)
        return attempt

    def _prepare_drawing(self, state, identity, drawing, q_start):
        arm, scene = self.arms[identity], state.payload
        task = drawing["stroke_id"]
        continuous = self.timing.get("drawing_clock") == "cubic_toppra"
        tracked = tracked_drawing_path(arm, drawing, continuous, self.deadline)
        if tracked["status"] != "BOUNDED":
            self.failure(
                task,
                identity,
                "TRACKING",
                tracked["status"],
                **{key: value for key, value in tracked.items() if key != "status"},
            )
            return None
        q, spline_pieces, spline = tracked["positions"], tracked["pieces"], tracked["spline"]
        # Full approach and its travel must be feasible together.
        approach_attempts = []
        # Free-axis drawing can leave the nozzle nearly horizontal. Pulling
        # along its actual body axis is then distinct from every world-up ray.
        # The ray still receives the same IK, tracking, and collision checks.
        approach_directions = [
            (-arm.robot.forward(q[0], arm.flange_to_tcp)[:3, 2]).tolist(),
            *self.transit["departure_directions"],
        ]
        for direction in approach_directions:
            if monotonic() >= self.deadline:
                self.failure(task, identity, "APPROACH", "TIME_BUDGET", attempts=approach_attempts)
                return None
            lift = find_departure(
                arm.robot,
                arm.flange_to_tcp,
                q[0],
                {**self.transit, "departure_directions": [direction]},
                self.lift_check(identity, scene, q[0]),
                self.deadline,
            )
            approach_attempts.extend(lift["attempts"])
            detail = lift["attempts"][-1]
            if lift["status"] != "SAMPLED_DRAWING_PATH":
                continue
            approach = np.array(lift["positions"])[::-1]
            travel = self.travel(identity, scene, q_start, approach[0])
            detail["travel"] = {key: value for key, value in travel.items() if key != "positions"}
            if travel["status"] == "CONNECTED":
                break
        else:
            self.failure(
                task, identity, "APPROACH", "NO_CONNECTED_APPROACH", attempts=approach_attempts
            )
            return None
        after = scene.copy()
        radius = self.actions[task]["diameter_m"] / 2
        for i, (a, b) in enumerate(zip(q[:-1], q[1:], strict=True)):
            if i % 500 == 0:
                print(
                    f"Checking {identity}/{task}: drawing segment {i + 1}/{len(q) - 1}", flush=True
                )
            name = f"prospective/{identity}/{task}/{i}"
            if continuous:
                piece = spline_pieces[i]
                self.add_piece(after, identity, piece, radius, name)
                check = arm.collision.check_bounded_path(
                    piece.position,
                    piece.deviation,
                    self.obstacles,
                    self.margin,
                    self.settings["maximum_subdivision_depth"],
                    after,
                    self.deadline,
                )
            else:
                self.add_draw(after, identity, a, b, radius, name)
                check = arm.collision.check_edge(
                    a,
                    b,
                    self.obstacles,
                    self.margin,
                    self.settings["maximum_subdivision_depth"],
                    after,
                    deadline=self.deadline,
                )
            if check["status"] != "CERTIFIED_MODELED_EDGE":
                self.failure(
                    task,
                    identity,
                    "DRAW",
                    check["status"],
                    segment_index=i,
                    **{key: value for key, value in check.items() if key != "status"},
                )
                return None
        withdraw = find_departure(
            arm.robot,
            arm.flange_to_tcp,
            q[-1],
            {
                **self.transit,
                "departure_directions": [
                    (-arm.robot.forward(q[-1], arm.flange_to_tcp)[:3, 2]).tolist(),
                    *self.transit["departure_directions"],
                ],
            },
            self.lift_check(identity, after, q[-1]),
            self.deadline,
        )
        if withdraw["status"] != "SAMPLED_DRAWING_PATH":
            self.failure(
                task, identity, "WITHDRAW", withdraw["status"], attempts=withdraw["attempts"]
            )
            return None
        start_contact, end_contact = [
            arm.robot.forward(p, arm.flange_to_tcp)[:3, 3] for p in (q[0], q[-1])
        ]
        operations = [
            {"kind": "TRAVEL", "positions": travel["positions"]},
            {"kind": "APPROACH", "positions": approach, "contact_m": start_contact},
            {
                "kind": "PEN_START_WAIT",
                "positions": [q[0]],
                "contact_m": start_contact,
                "hold_s": self.process["start_lag_s"],
            },
            {
                "kind": "JOIN",
                "positions": [q[0]],
                "contact_m": start_contact,
                "hold_s": self.process["join_s"],
            },
            {"kind": "DRAW", "positions": q, "material_radius_m": radius},
            {
                "kind": "PEN_STOP_WAIT",
                "positions": [q[-1]],
                "contact_m": end_contact,
                "hold_s": self.process["stop_lag_s"],
            },
            {"kind": "WITHDRAW", "positions": withdraw["positions"], "contact_m": end_contact},
        ]
        release_cooling = float(drawing.get("release_cooling_s", 0.0))
        if not np.isfinite(release_cooling) or release_cooling < 0:
            raise ValueError("Release cooling must be finite and nonnegative")
        if release_cooling:
            operations.insert(
                -1,
                {
                    "kind": "WAIT",
                    "positions": [q[-1]],
                    "contact_m": end_contact,
                    "hold_s": release_cooling,
                },
            )
        if continuous:
            next(op for op in operations if op["kind"] == "DRAW")["path"] = spline
        for operation in operations:
            operation["stroke_id"] = task
        try:
            path = timed_path(
                operations,
                arm,
                arm.robot.velocity * self.timing["velocity_fraction"],
                np.array(self.timing["joint_acceleration_rad_s2"]),
                self.process["nominal_speed_m_s"],
                self.timing["travel_speed_m_s"],
                drawing_clock=self.timing.get("drawing_clock", "quintic_stop_at_geometric_knots"),
                acceleration_constraint=self.timing.get("acceleration_constraint", "interpolated"),
            )
        except TimingParameterizationError as error:
            self.failure(
                task, identity, "TIMING", "TIMING_REJECTED", reason=str(error),
                drawing_clock=self.timing.get("drawing_clock"),
                acceleration_constraint=self.timing.get("acceleration_constraint"),
                path_samples=len(q),
                minimum_joint_step_rad=float(np.min(np.linalg.norm(np.diff(q, axis=0), axis=1))),
            )
            return None
        path.metadata["tracking_position_bound_m"] = tracked["position_error_bound_m"]
        path.metadata["tracking_orientation_constraint"] = tracked["orientation_constraint"]
        if tracked["orientation_constraint"] == "fixed":
            path.metadata["tracking_orientation_bound_rad"] = tracked["orientation_error_bound_rad"]
        path.metadata["tracking_subdivisions"] = tracked["subdivisions"]
        path.metadata["tracking_status"] = "BOUNDED"
        proposal = MotionProposal(
            task,
            identity,
            drawing["variant"],
            path.duration_s,
            drawing["start_anchor"],
            tuple(withdraw["positions"][-1]),
            path,
        )
        print(
            f"Prepared {identity}/{task}/{drawing['variant']}: {path.duration_s:.1f}s", flush=True
        )
        return proposal

    def validate_interval(self, state, new, end):
        motions = {m.proposal.arm: m for m in state.active + new}
        scene = state.payload.copy()
        boundaries = {state.time_s, end}
        for motion in motions.values():
            boundaries.update(
                motion.start_s + edge.end_s
                for edge in motion.proposal.payload.edges
                if state.time_s < motion.start_s + edge.end_s < end
            )
        boundaries = sorted(boundaries)
        for a, b in zip(boundaries[:-1], boundaries[1:], strict=True):
            if monotonic() >= self.deadline:
                return None, {"status": "TIME_BUDGET"}
            paths = {}
            for identity in self.arms:
                if identity in motions:
                    motion = motions[identity]
                    edge = motion.proposal.payload.edge_at((a + b) / 2 - motion.start_s)
                    start = motion.start_s

                    def position(u, e=edge, s=start, left=a, right=b):
                        return e.position(left + (right - left) * u - s)

                    def deviation(x, y, e=edge, s=start, left=a, right=b):
                        return e.deviation(
                            left + (right - left) * x - s, left + (right - left) * y - s
                        )
                else:
                    edge = None
                    q = np.array(dict(state.configurations)[identity])

                    def position(u, q=q):
                        return q

                    def deviation(x, y, q=q):
                        return np.zeros(len(q))

                paths[identity] = (edge, position, deviation)
            # Deposit conservative enclosures of BOTH growing strokes before
            # checking the interval. Future intervals are never installed early.
            for identity, (edge, _position, _) in paths.items():
                if edge is not None and edge.kind == "DRAW":
                    motion = motions[identity]
                    self.add_clock_edge(
                        scene,
                        identity,
                        edge,
                        f"material/{identity}/{edge.stroke}/{a:.12f}-{b:.12f}",
                        a - motion.start_s,
                        b - motion.start_s,
                    )
            for identity, (edge, position, deviation) in paths.items():
                arm = self.arms[identity]
                if edge is not None and edge.contact is not None:
                    # Keep the original stroke endpoint as the contact center,
                    # rather than moving the exemption ball with the nozzle.
                    valid = contact_validator(
                        arm.robot,
                        arm.collision,
                        self.tip_models[identity],
                        arm.flange_to_tcp,
                        scene,
                        self.obstacles,
                        edge.contact,
                        self.transit["contact_radius_m"],
                        arm.tool.tip_radius_m or self.transit["virtual_tip_radius_m"],
                        self.margin,
                        self.settings["maximum_subdivision_depth"],
                        local_contact_surfaces(
                            edge.contact[2],
                            self.profile["environment"]["table_z_m"],
                            self.profile["world_from_build"][2][3],
                            self.transit["contact_radius_m"],
                        ),
                        deadline=self.deadline,
                    )
                    if not valid(position(0), position(1)):
                        return None, {
                            "status": "CONTACT_PATH_REJECTED",
                            "arm": identity,
                            "time_s": a,
                            "evidence": valid.failures[:2],
                        }
                else:
                    model = (
                        arm.collision
                        if edge is not None and edge.kind == "DRAW"
                        else self.tip_models[identity]
                    )
                    check = model.check_bounded_path(
                        position,
                        deviation,
                        self.obstacles,
                        self.margin,
                        self.settings["maximum_subdivision_depth"],
                        scene,
                        self.deadline,
                    )
                    if check["status"] != "CERTIFIED_MODELED_EDGE":
                        return None, {"arm": identity, "time_s": a, **check}
            if len(self.arms) == 2:
                left, right = self.arms
                check = check_pair_paths(
                    self.tip_models[left],
                    self.tip_models[right],
                    paths[left][1],
                    paths[right][1],
                    paths[left][2],
                    paths[right][2],
                    margin_m=self.margin,
                    maximum_depth=self.settings["maximum_subdivision_depth"],
                    deadline=self.deadline,
                )
                if check["status"] != "CERTIFIED_INTER_ARM_INTERVAL":
                    return None, {"phase": "INTER_ARM", "time_s": a, **check}
            self.checked_intervals += 1
        return scene, None

    def finish(self, state):
        # Try either return order. Each return is checked against the other
        # stationary arm and the complete material, on the same clock.
        for order in permutations(self.arms):
            current, failed = state, False
            for identity in order:
                arm = self.arms[identity]
                result = self.travel(
                    identity,
                    current.payload,
                    np.array(dict(current.configurations)[identity]),
                    arm.initial_positions,
                )
                if result["status"] != "CONNECTED":
                    failed = True
                    break
                if len(result["positions"]) < 2 or np.allclose(
                    result["positions"], result["positions"][0], atol=1e-12, rtol=0
                ):
                    continue
                path = timed_path(
                    [{"kind": "RETURN", "positions": result["positions"]}],
                    arm,
                    arm.robot.velocity * self.timing["velocity_fraction"],
                    np.array(self.timing["joint_acceleration_rad_s2"]),
                    self.process["nominal_speed_m_s"],
                    self.timing["travel_speed_m_s"],
                )
                proposal = MotionProposal(
                    f"return-{identity}",
                    identity,
                    "return",
                    path.duration_s,
                    "parking",
                    tuple(arm.initial_positions),
                    path,
                )
                move = ScheduledMotion(proposal, current.time_s)
                scene, failure = self.validate_interval(current, (move,), move.end_s)
                if failure:
                    failed = True
                    break
                q = dict(current.configurations)
                q[identity] = tuple(arm.initial_positions)
                current = replace(
                    current,
                    time_s=move.end_s,
                    configurations=tuple(q.items()),
                    payload=scene,
                    history=current.history + (move,),
                )
            if not failed:
                return current, None
        return None, {"status": "NO_RETURN_ORDER_FOUND"}


def _plan_cell(
    run,
    profile_path,
    settings_path,
    maximum_seconds=300,
    output_name="dual_arm_plan",
    enabled_arms=None,
    resume_from=None,
):
    run = Path(run)
    settings = yaml.safe_load(Path(settings_path).read_text())
    if enabled_arms is not None:
        settings["enabled_arms"] = enabled_arms
    if not output_name.replace("_", "").isalnum():
        raise ValueError("Output name must contain only letters, digits and underscores")
    config_name = output_name.removesuffix("_plan") + "_config.json"
    if settings.get("clock") != "quintic_stop_at_geometric_knots" or not settings.get(
        "simulation_only"
    ):
        raise ValueError("Unsupported cell planning configuration")
    deadline = monotonic() + maximum_seconds
    planner = CellPlanner(run, profile_path, settings, deadline)
    resume_state, seed_name = None, None
    if resume_from:
        if resume_from == output_name or not resume_from.replace("_", "").isalnum():
            raise ValueError("Resume into a different named output")
        from .cell_replay import _replay_cell

        seed_name = f"{output_name}_seed.json"
        seed = read_json(run / f"{resume_from}.json")
        seed_config = read_json(run / seed["settings_artifact"])
        for field, current in (
            ("process", planner.process),
            ("timing", planner.timing),
            ("transit", planner.transit),
        ):
            if seed_config[field] != current:
                raise ValueError(f"Cannot resume across changed {field} assumptions")
        if seed.get("enabled_arms") != planner.enabled_arms:
            raise ValueError("Cannot change enabled arms while resuming")
        write_json(run / seed_name, seed)
        audit, resume_state = _replay_cell(
            run,
            profile_path,
            min(120, max(1, deadline - monotonic())),
            seed_name.removesuffix(".json"),
            return_state=True,
        )
        if audit["status"] not in ("CERTIFIED_PREFIX_REPLAY", "CERTIFIED_GEOMETRIC_REPLAY"):
            raise ValueError(f"Resume replay rejected: {audit['status']}")
        for identity, arm in planner.arms.items():
            if not np.allclose(seed["initial_positions"][identity], arm.initial_positions):
                raise ValueError("Staged poses changed; replan from the beginning")
    write_json(
        run / config_name,
        {
            "search": settings,
            "transit": planner.transit,
            "timing": planner.timing,
            "process": planner.process,
        },
    )
    output = artifact(
        "dual_arm_plan",
        status="RUNNING",
        executable=False,
        simulation_only=True,
        frame="world",
        units="m",
        angle_units="rad",
        time_units="s",
        inputs={
            name: sha256(run / name)
            for name in (
                "cell_manifest.json",
                "cell_drawing_variants.json",
                "construction_plan.json",
                "stroke_graph.json",
                config_name,
            )
        },
        actions=[],
        events=[],
        completed_strokes=0,
        requested_strokes=len(planner.tasks),
        modeled_duration_s=0,
        checked_intervals=0,
        failures=[],
        assumptions=settings,
        settings_artifact=config_name,
        enabled_arms=planner.enabled_arms,
        resumed_from=resume_from,
        initial_positions={
            identity: arm.initial_positions.tolist() for identity, arm in planner.arms.items()
        },
        initial_pose_repairs=planner.initial_pose_repairs,
        limitations=[
            "Provisional base placement and tool transforms; unmeasured pen/mount geometry",
            "C2 quintic clocks stop at geometric waypoints; pen flow through stops is uncalibrated",
            "Drawing tip contact, hot-material stability and physical tracking are not certified",
            "Startup/shutdown durations are unknown; cooling and pen response are assumptions",
            "Bounded beam search does not establish optimality or global infeasibility",
        ],
    )
    if seed_name:
        output["inputs"][seed_name] = sha256(run / seed_name)

    def save(state, explored):
        output.update(
            completed_strokes=len(state.completed),
            modeled_duration_s=state.time_s,
            checked_intervals=planner.checked_intervals,
            explored_states=explored,
            material_segments=len(state.payload.segments),
            actions=[
                {
                    "arm": move.proposal.arm,
                    "stroke_id": move.proposal.task,
                    "variant": move.proposal.variant,
                    "start_s": move.start_s,
                    "end_s": move.end_s,
                    "start_anchor": move.proposal.start_anchor,
                    "auxiliary": move.proposal.auxiliary,
                    "trajectory": move.proposal.payload.record(),
                }
                for move in state.history
            ],
            failures=planner.attempts,
            events=sorted(
                [
                    {
                        "time_s": move.start_s + edge.start_s,
                        "arm": move.proposal.arm,
                        "stroke_id": move.proposal.task,
                        "command": "FEED_ON" if edge.kind == "PEN_START_WAIT" else "FEED_OFF",
                    }
                    for move in state.history
                    for edge in move.proposal.payload.edges
                    if edge.kind in ("PEN_START_WAIT", "PEN_STOP_WAIT")
                    and move.start_s + edge.start_s <= state.time_s
                ],
                key=lambda event: (event["time_s"], event["arm"]),
            ),
        )
        write_json(run / f"{output_name}.json", output)
        if output_name.startswith("dual_arm"):
            stage_update(
                run,
                "cell",
                "partial",
                artifact=f"{output_name}.json",
                message=f"{len(state.completed)}/{len(planner.tasks)} strokes scheduled; "
                "joint motion search",
            )
        print(
            f"Scheduled {len(state.completed)}/{len(planner.tasks)}; {state.time_s:.1f}s",
            flush=True,
        )

    def estimate(state):
        active = {m.proposal.task for m in state.active}
        work = sum(
            planner.actions[t.identity]["length_m"] * 1.875 / planner.process["nominal_speed_m_s"]
            for t in planner.tasks
            if t.identity not in state.completed and t.identity not in active
        )
        work += sum(m.end_s - state.time_s for m in state.active)
        return work / len(planner.enabled_arms)

    initial = {identity: tuple(arm.initial_positions) for identity, arm in planner.arms.items()}
    root = ScheduleState(
        0,
        tuple(initial.items()),
        frozenset(),
        tuple((node, 0) for node in planner.ground),
        payload=MaterialScene(),
    )
    if resume_state is not None:
        root = resume_state
    save(root, 0)
    result = search_schedule(
        planner.tasks,
        initial,
        planner.ground,
        planner.prepare,
        planner.validate_interval,
        finish=planner.finish,
        initial_payload=MaterialScene(),
        start_offsets_s=settings["start_offsets_s"],
        maximum_states=settings["maximum_states"],
        beam_width=settings["beam_width"],
        maximum_dispatch_batches=settings.get("maximum_dispatch_batches"),
        maximum_successors_per_state=settings.get("maximum_successors_per_state"),
        completion_first=settings.get("completion_first", False),
        deadline=deadline,
        estimate_remaining=estimate,
        observe=save,
        prepare_idle=planner.prepare_idle,
        initial_state=resume_state,
    )
    output["status"] = (
        "COORDINATED_GEOMETRIC_PREVIEW" if result["status"] == "FOUND" else result["termination"]
    )
    output["search"] = {
        key: value for key, value in result.items() if key not in ("state", "failures")
    }
    output["search_failures"] = result["failures"]
    save(result["state"], result["explored_states"])
    return output


def plan_cell(
    run,
    profile_path,
    settings_path,
    maximum_seconds=300,
    output_name="dual_arm_plan",
    enabled_arms=None,
    resume_from=None,
):
    try:
        return _plan_cell(
            run,
            profile_path,
            settings_path,
            maximum_seconds,
            output_name,
            enabled_arms,
            resume_from,
        )
    except Exception as error:
        path = Path(run) / f"{output_name}.json"
        if path.exists():
            output = read_json(path)
            output.update(
                status="ERROR", error={"type": type(error).__name__, "message": str(error)}
            )
            write_json(path, output)
        if output_name.startswith("dual_arm"):
            stage_update(Path(run), "cell", "partial", message=f"Joint planner error: {error}")
        raise


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, default=Path("runs/palace_15cm_dual"))
    parser.add_argument("--config", type=Path, default=Path("configs/cell/two_flexiv.yaml"))
    parser.add_argument("--settings", type=Path, default=Path("configs/dual_arm.yaml"))
    parser.add_argument("--maximum-seconds", type=float, default=300)
    parser.add_argument("--output-name", default="dual_arm_plan")
    parser.add_argument("--enabled-arms", nargs="+")
    parser.add_argument("--resume-from", help="Replay and continue another saved plan prefix")
    args = parser.parse_args()
    plan_cell(
        args.run,
        args.config,
        args.settings,
        args.maximum_seconds,
        args.output_name,
        args.enabled_arms,
        args.resume_from,
    )
