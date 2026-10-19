"""Couple shared-clock arm motions, grounded strokes and growing material.

All output is simulation-only. Missing pen geometry/flow calibration and material
strength remain explicit; geometric acceptance is not fabrication acceptance.
"""

import hashlib
import json
from collections import Counter
from copy import copy
from dataclasses import replace
from pathlib import Path
from time import monotonic, perf_counter

import fcl
import numpy as np
import yaml

from ..cell import load_cell
from ..curves import transform_points
from ..io import read_json, sha256, write_json
from ..kinematics import inverse_candidates
from ..strokes import orient_stroke
from ..timing import (
    TimingParameterizationError,
    make_path,
    refine_samples,
    tcp_acceleration_bound,
)
from .axis_repair import (
    AXIS_REPAIR_REVISION,
    clear_intermediate_poses,
    outward_pen_value,
    refine_axis_path,
    repair_collision_witness,
    repair_drawing_axes,
)
from .cell_motion import radial_pen_rotations
from .clock_path import timed_path
from .collision import QUERY_STATS, Body, RobotCollisionModel, placed
from .geometric_path import geometric_path
from .pen_spheres import PenSpheres, wire_arrays
from .process_clock import (
    PROCESS_CLOCK_POLICY,
    derive_deposition_events,
    next_visit,
    source_vertex_schedule,
)
from .rrt import connect
from .schedule import (
    MotionProposal,
    tasks_from_construction,
)
from .smooth_audit import CubicPiece, bounded_segments, check_tracking
from .tool import sampled_pen_tilt
from .tracking import tracking_bounds
from .transit import contact_validator, find_departure, local_contact_surfaces


def tracked_drawing_path(arm, drawing, continuous, deadline, *, require_fixed_orientation=False):
    """Refine only the interpolant when a fixed drawing path misses tracking bounds."""
    original_q = np.asarray(drawing["positions"])
    free_axis = drawing.get("free_tool_orientation", False)
    require_fixed_orientation = (
        require_fixed_orientation
        or drawing.get("fixed_orientation", False)
        or drawing.get("orientation_policy") in {"fixed", "fixed_axis"}
    )
    if require_fixed_orientation:
        rotations = np.asarray(drawing.get("tcp_rotations", drawing["tcp_rotation"]))
        if free_axis or not np.allclose(rotations, drawing["tcp_rotation"], atol=1e-10, rtol=0):
            return {
                "status": "FIXED_ORIENTATION_REQUIRED",
                "reason": "Variable-axis drawing or rotation targets",
            }
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
                **(
                    {"fixed_tcp_rotation": np.asarray(drawing["tcp_rotation"]).tolist()}
                    if not free_axis
                    else {}
                ),
            }
    return {**failed, "status": "BOUND_EXCEEDS_TOLERANCE"}


def member_drawing_path(arm, drawing, members, schedule, deadline):
    """One path piece per straight source member, split only where tracking fails.

    ``drawing`` holds a dense joint path. Its knots at member ends and at any
    other change of direction are kept; the rest are dropped, and one is put
    back at the middle of a piece whose tool path leaves its tolerance.
    ``schedule(reached)`` returns the kept knots at which the pen stops.
    Returns None when the drawing does not fit this construction.
    """
    q, targets = np.asarray(drawing["positions"], float), np.asarray(drawing["tcp_targets_m"])
    if drawing.get("free_tool_orientation", False) or len(q) != len(targets):
        return None
    moving = np.r_[True, np.linalg.norm(np.diff(q, axis=0), axis=1) > 1e-10]
    q, targets = q[moving], targets[moving]
    if len(q) < 2:
        return None
    keep, previous = {0, len(q) - 1}, 0
    for point in members:
        index = next_visit(targets, previous, point, 1e-6)
        if np.linalg.norm(targets[index] - point) > 1e-6:
            return None
        keep.add(index)
        previous = index
    steps = np.diff(targets, axis=0)
    lengths = np.linalg.norm(steps, axis=1)
    if np.any(lengths < 1e-9):
        return None
    directions = steps / lengths[:, None]
    keep.update((np.flatnonzero(np.linalg.norm(np.diff(directions, axis=0), axis=1) > 1e-7) + 1)
                .tolist())
    arc = np.r_[0.0, np.cumsum(lengths)]
    rotation = np.asarray(drawing["tcp_rotation"])
    reached = {}
    checked = {}
    while True:
        if monotonic() >= deadline:
            return {"status": "TIME_BUDGET"}
        kept = sorted(keep)
        for index in kept:
            if index not in reached:
                reached[index] = arm.robot.forward(q[index], arm.flange_to_tcp)[:3, 3]
        points = np.array([reached[index] for index in kept])
        spline = make_path(q[kept], points, interpolation="pchip", breaks=schedule(points))
        worst, worst_angle, added = 0.0, 0.0, False
        pieces = []
        for i, width in enumerate(np.diff(spline.cspl.x)):
            piece = CubicPiece(spline.cspl.c[:, i, :], width)
            pieces.append(piece)
            key = (kept[i], kept[i + 1], spline.cspl.c[:, i, :].tobytes())
            if key not in checked:
                checked[key] = check_tracking(
                    piece, arm.robot, arm.flange_to_tcp, targets[kept[i]], targets[kept[i + 1]],
                    rotation, 0.0003, 0.003,
                )
            check = checked[key]
            if (
                check["position_error_bound_m"] <= 0.0003
                and check["orientation_error_bound_rad"] <= 0.003
            ):
                worst = max(worst, check["position_error_bound_m"])
                worst_angle = max(worst_angle, check["orientation_error_bound_rad"])
                continue
            inside = np.arange(kept[i] + 1, kept[i + 1])
            if not len(inside):
                # Nothing left to split with; the caller refines every sample.
                return None
            middle = (arc[kept[i]] + arc[kept[i + 1]]) / 2
            keep.add(int(inside[np.argmin(abs(arc[inside] - middle))]))
            added = True
        if added:
            continue
        positions = q[kept]
        rows = np.repeat(np.eye(4)[None], len(kept), axis=0)
        rows[:, :3, :3] = rotation
        rows[:, :3, 3] = targets[kept]
        return {
            "status": "BOUNDED",
            "positions": positions,
            "targets": rows,
            "spline": spline,
            "pieces": pieces,
            "subdivisions": 1,
            "position_error_bound_m": worst,
            "orientation_error_bound_rad": worst_angle,
            "orientation_constraint": "fixed",
            "fixed_tcp_rotation": rotation.tolist(),
            "member_pieces": {"source_knots": len(q), "members": len(members) - 1},
        }


def add_piece_material(scene, arm, piece, radius, name):
    """Enclose deposited material using the same bound in offline and live checks."""
    for index, (a, b, inflation) in enumerate(
        bounded_segments(piece, arm.robot, arm.flange_to_tcp, 0.00002)
    ):
        ends = [arm.robot.forward(piece.position(t), arm.flange_to_tcp)[:3, 3] for t in (a, b)]
        scene.add(*ends, radius + inflation + 1e-9, f"{name}/{index}")


def fixed_clock_orientation_failure(arm, edge, reference, start=None, end=None):
    """Certify the entire retained DRAW polynomial, including between knots."""
    piece = edge.piece(start, end)
    check = check_tracking(
        piece,
        arm.robot,
        arm.flange_to_tcp,
        np.zeros(3),
        np.zeros(3),
        reference,
        np.inf,
        0.003,
    )
    if check["status"] != "BOUNDED":
        return {
            "status": "FIXED_ORIENTATION_REJECTED",
            "stroke_id": edge.stroke,
            "orientation_error_bound_rad": check["orientation_error_bound_rad"],
            "orientation_limit_rad": 0.003,
            "witness": check.get("witness"),
        }
    return None


def check_growing_drawing_prefix(
    arm,
    tracked,
    scene,
    obstacles,
    radius,
    margin,
    maximum_depth,
    deadline,
    *,
    name_prefix="prospective",
    continuous=True,
    progress=None,
):
    """Check a drawing against its own deposited prefix and supplied initial material.

    Each current interval is conservatively enclosed before checking, matching
    runtime's existing semantics. The physical tip is omitted by arm.collision;
    the actual nozzle, housing, robot, and their self collisions remain checked.
    This geometric proof does not cover approach, departure, or process timing.
    """
    after = scene.copy()
    q = tracked["positions"]
    for i, (a, b) in enumerate(zip(q[:-1], q[1:], strict=True)):
        if progress is not None and (i % 100 == 0 or i == len(q) - 2):
            progress(i, len(q) - 1)
        if monotonic() >= deadline:
            return None, {"status": "TIME_BUDGET", "segment_index": i}
        name = f"{name_prefix}/{i}"
        if continuous:
            piece = tracked["pieces"][i]
            add_piece_material(after, arm, piece, radius, name)
            check = arm.collision.check_bounded_path(
                piece.position,
                piece.deviation,
                obstacles,
                margin,
                maximum_depth,
                after,
                deadline,
            )
        else:
            ends = [arm.robot.forward(p, arm.flange_to_tcp)[:3, 3] for p in (a, b)]
            inflation = (
                tcp_acceleration_bound(arm.robot, arm.flange_to_tcp, abs(b - a), np.zeros(len(a)))
                / 8
            )
            after.add(*ends, radius + inflation + 1e-9, name)
            check = arm.collision.check_edge(
                a,
                b,
                obstacles,
                margin,
                maximum_depth,
                after,
                deadline=deadline,
            )
        if check["status"] != "CERTIFIED_MODELED_EDGE":
            return None, {**check, "segment_index": i}
    return after, None


def enclose_drawing_prefix(arm, tracked, scene, radius, deadline, *, name_prefix, continuous):
    """Construct material for withdrawal planning, granting no collision proof.

    Indexed forward planning may defer its early geometric collision pruning
    because it checks the entire actual timed clock before admitting a motion.
    Withdrawal still sees the complete conservative drawing enclosures.
    """
    after = scene.copy()
    positions = tracked["positions"]
    for index, (a, b) in enumerate(zip(positions[:-1], positions[1:], strict=True)):
        if monotonic() >= deadline:
            return None, {"status": "TIME_BUDGET", "segment_index": index}
        name = f"{name_prefix}/{index}"
        if continuous:
            add_piece_material(after, arm, tracked["pieces"][index], radius, name)
        else:
            ends = [arm.robot.forward(q, arm.flange_to_tcp)[:3, 3] for q in (a, b)]
            inflation = (
                tcp_acceleration_bound(arm.robot, arm.flange_to_tcp, abs(b - a), np.zeros(len(a)))
                / 8
            )
            after.add(*ends, radius + inflation + 1e-9, name)
    return after, None


class CellPlanner:
    def __init__(
        self, run, profile, settings, deadline, *, timing_override=None, verification_raw=False,
        optimize_contact_geometry=None,
    ):
        self.run, self.settings, self.deadline = Path(run), settings, deadline
        if settings.get("collision_scope", "full_robot") not in {"full_robot", "pen_candidate"}:
            raise ValueError("Collision scope must be full_robot or pen_candidate")
        self.profile, self.arms, self.obstacles = load_cell(profile)
        self.turntable = None
        if "turntable" in self.profile:
            from .turntable import IndexedTurntable

            if len(self.arms) != 1:
                raise ValueError("Indexed turntable requires one arm")
            self.turntable = IndexedTurntable.from_profile(self.profile)
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
        # A profile may adjust transit settings (detail drawn on the outside
        # backs away a shorter distance).
        self.transit = {
            **yaml.safe_load(Path("configs/transit.yaml").read_text()),
            **self.profile.get("transit", {}),
        }
        self.timing = (
            timing_override
            if timing_override is not None
            else yaml.safe_load(Path("configs/timing.yaml").read_text())
        )
        self.margin = self.profile["environment"]["clearance_m"]
        self.tip_models = {}
        self.pen_models, self.pen_tip_models = {}, {}
        self._pen_spheres = {}
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
            # Pen-scope planning checks the pen; a profile may add the robot's
            # links, so tilted poses that fold the arm into itself fail early.
            with_robot = bool(self.profile.get("planning_checks_robot", False))
            self.pen_models[identity] = RobotCollisionModel(
                arm.robot, arm.tool.bodies, include_robot=with_robot
            )
            self.pen_tip_models[identity] = RobotCollisionModel(
                arm.robot, [*arm.tool.bodies, tip], include_robot=with_robot
            )
        # A tool body may keep its own clearance from wire (the nose right
        # behind a tip that runs along existing wire): every collision model.
        own = self.profile["environment"].get("tool_clearance_m", {})
        if own:
            margins = {f"tool/{name}": value for name, value in own.items()}
            for identity, arm in self.arms.items():
                for model in (
                    arm.collision,
                    self.tip_models[identity],
                    self.pen_models[identity],
                    self.pen_tip_models[identity],
                ):
                    model.material_margins = margins
        for row in self.candidates["variants"]:
            self.drawings.setdefault((row["arm"], row["stroke_id"]), []).append(row)
        self.initial_pose_repairs = self.choose_initial_poses(source)
        from .path_certificates import PathCertificateStore

        self.verification_raw = verification_raw
        self.optimize_contact_geometry = (
            not verification_raw if optimize_contact_geometry is None else optimize_contact_geometry
        )
        self.path_certificates = PathCertificateStore(
            self.run / "path_certificates", enabled=not verification_raw
        )
        self.verification_stats, self.verification_seconds = Counter(), Counter()
        self._path_model_contexts, self._obstacle_context_refs = {}, {}
        # Check continuous drawings as untimed paths; ``time_local_drawing``
        # times them once the whole object has a collision-free chain.
        self.geometry_first = False

    @property
    def collision_scope(self):
        return (
            "full_robot"
            if getattr(self, "verification_raw", False)
            else getattr(self, "settings", {}).get("collision_scope", "full_robot")
        )

    def collision_arm(self, identity):
        """Select proposal geometry without replacing the retained full robot."""
        arm = self.arms[identity]
        if self.collision_scope == "full_robot":
            return arm
        proposal = copy(arm)
        proposal.collision = self.pen_models[identity]
        return proposal

    def collision_tip_model(self, identity):
        return (
            self.pen_tip_models[identity]
            if self.collision_scope == "pen_candidate"
            else self.tip_models[identity]
        )

    def collision_obstacles(self):
        return self.obstacles

    def path_certificate_context(self, identity, *, kinematic=False, pen=False):
        """Bind invariant checks without including the changing material scene.

        ``kinematic`` binds what the speed and source-coverage audits read: the
        arm's kinematics and limits, the tool point, the source and the process
        settings. Those audits never consult a collision model, so their proofs
        hold whichever collision scope the path was planned under.

        The context is rebuilt on every call and compared with the last one by
        what can differ between calls; an unchanged one is returned as the same
        frozen object, whose hash the proof store then forms only once.
        """
        from .path_certificates import FrozenContext

        context = self._path_certificate_context(identity, kinematic=kinematic, pen=pen)
        arm = self.arms[identity]
        signature = (
            self.collision_scope,
            json.dumps([self.settings, self.timing, self.process, self.transit],
                       sort_keys=True, default=str),
            self.margin,
            *(id(part) for part in (self.profile, self.actions, self.obstacles, arm.collision,
                                    arm.tool.metadata, self.turntable)),
            tuple(sorted(self.obstacles)),
        )
        frozen = self.__dict__.setdefault("_frozen_contexts", {})
        cached = frozen.get((identity, kinematic, pen))
        if cached is None or cached[0] != signature:
            cached = (signature, FrozenContext(context))
            frozen[(identity, kinematic, pen)] = cached
        return cached[1]

    def _path_certificate_context(self, identity, *, kinematic=False, pen=False):
        if pen:
            # What a pen-only collision proof read, whichever scope asks for it:
            # the kinematic context, the fixed obstacles and the pen's geometry.
            from .collision_context import collision_context_hash

            arm, full = self.arms[identity], self._path_certificate_context(identity)
            return {
                **self._path_certificate_context(identity, kinematic=True),
                "obstacles": full["obstacles"],
                "pen_models": {
                    "pen": collision_context_hash(self.pen_models[identity], arm.tool.metadata),
                    "tip": collision_context_hash(
                        self.pen_tip_models[identity], arm.tool.metadata
                    ),
                },
            }
        if kinematic:
            context = {
                name: value
                for name, value in self._path_certificate_context(identity).items()
                if name not in {"collision_scope", "models", "body_origins", "obstacles"}
            }
            context["settings"] = {
                name: value for name, value in self.settings.items() if name != "collision_scope"
            }
            return context
        from .collision_context import (
            collision_context_hash,
            digest,
            obstacle_context,
            static_shapes,
        )

        arm = self.arms[identity]
        model = getattr(arm.collision, "model", arm.collision)
        tip = getattr(self.tip_models[identity], "model", self.tip_models[identity])
        model_key = (identity, id(model), id(tip), digest(arm.tool.metadata))
        if model_key not in self._path_model_contexts:
            self._path_model_contexts[model_key] = {
                "arm": collision_context_hash(model, arm.tool.metadata),
                "tip": collision_context_hash(tip, arm.tool.metadata),
            }
        return {
            "collision_scope": self.collision_scope,
            "models": self._path_model_contexts[model_key],
            "tool_metadata": arm.tool.metadata,
            "world_from_base": getattr(arm.robot, "world_from_base", np.eye(4)),
            "joint_limits": {
                "lower": arm.robot.lower,
                "upper": arm.robot.upper,
                "velocity": arm.robot.velocity,
            },
            "joint_data": arm.robot.joint_data,
            "body_origins": [(body.name, body.origin) for body in model.bodies],
            "profile": self.profile,
            "obstacles": obstacle_context(
                self.obstacles,
                static_shapes(self.profile, self.turntable),
                self._obstacle_context_refs,
            ),
            "source_actions": self.actions,
            "flange_to_tcp": arm.flange_to_tcp,
            "settings": self.settings,
            "timing": self.timing,
            "process": self.process,
            "transit": self.transit,
            "clearance_m": self.margin,
            "position_limit_m": 0.0003,
            "orientation_limit_rad": 0.003,
        }

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
            self.collision_tip_model(identity).check_edge(
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
        if self.edge_check(identity, scene, first, second):
            return {"status": "CONNECTED", "positions": np.asarray([first, second])}
        # A checked route via the known home posture can connect IK branches
        # that Cartesian interpolation and a finite RRT budget fail to join.
        # Home has no special clearance exemption: validate both full edges
        # against the current deposited material before using it as a waypoint.
        home = arm.initial_positions
        if self.edge_check(identity, scene, first, home) and self.edge_check(
            identity, scene, home, second
        ):
            return {
                "status": "CONNECTED",
                "positions": np.asarray([first, home, second]),
                "proposal_backend": "home_waypoint",
                "validation_backend": "mesh_fcl",
            }
        backend = getattr(self, "ik_backend", None)
        if backend is not None:
            for positions in backend.transit_candidates(first, second):
                if monotonic() >= self.deadline:
                    break
                if all(
                    self.edge_check(identity, scene, a, b)
                    for a, b in zip(positions[:-1], positions[1:], strict=True)
                ):
                    return {
                        "status": "CONNECTED",
                        "positions": positions,
                        "proposal_backend": "curobo",
                        "validation_backend": "mesh_fcl",
                    }
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
        arm = self.collision_arm(identity)
        contact = arm.robot.forward(q, arm.flange_to_tcp)[:3, 3]
        return contact_validator(
            arm.robot,
            arm.collision,
            self.collision_tip_model(identity),
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
                self.profile["environment"].get(
                    "platform_top_z_m", self.profile["world_from_build"][2][3]
                ),
                self.transit["contact_radius_m"],
                riser_z=(
                    self.profile["world_from_build"][2][3]
                    if "build_riser" in self.profile["environment"]
                    else None
                ),
            ),
            deadline=self.deadline,
        )

    def pen_sample_fractions(self, identity, position):
        """Fractions of an interval whose tool positions lie within the sampling distance."""
        arm = self.arms[identity]
        spacing = float(self.settings.get("pen_sample_m", 0.010))
        count = 1
        while True:
            fractions = np.linspace(0, 1, count + 1)
            tips = np.array(
                [arm.robot.forward(position(u), arm.flange_to_tcp)[:3, 3] for u in fractions]
            )
            gap = float(np.max(np.linalg.norm(np.diff(tips, axis=0), axis=1)))
            if gap <= spacing or count >= 4096:
                return fractions.tolist()
            count = max(count + 1, int(np.ceil(count * gap / spacing)))

    def pen_wire_failure(self, identity, scene, position, fractions, marks):
        """Check the pen's spheres at each fraction against the wire laid up to it.

        ``marks[k]`` is the number of wire capsules present when the pen is at
        ``fractions[k]``: a member's own wire is only what it has drawn so far.
        """
        arm = self.arms[identity]
        if identity not in self._pen_spheres:
            self._pen_spheres[identity] = PenSpheres(arm.tool.metadata)
        spheres, wires = self._pen_spheres[identity], wire_arrays(scene)
        # A body may keep its own clearance from wire (the nose right behind a
        # tip that runs along existing wire); every other body keeps the margin.
        own = self.profile["environment"].get("tool_clearance_m", {})
        margins = {None: self.margin, **{f"tool/{name}": value for name, value in own.items()}}
        self.verification_stats["pen_sphere_poses"] += len(fractions)
        for u, count in zip(fractions, marks, strict=True):
            flange = arm.robot.forward(position(u), np.eye(4))
            failure = spheres.violation(
                flange, tuple(values[:count] for values in wires), margins
            )
            if failure is None:
                continue
            # The spheres are slightly larger than the tool. Measure the wires
            # they flag against the exact tool shape at this same pose.
            bodies = [placed(body.geometry, flange @ body.origin) for body in arm.tool.bodies]
            exact = min(
                (
                    fcl.distance(
                        body, scene.objects[wire], fcl.DistanceRequest(), fcl.DistanceResult()
                    )
                    - margins.get(tool.name, self.margin),
                    wire,
                    tool.name,
                )
                for wire in failure.pop("wires")
                for body, tool in zip(bodies, arm.tool.bodies, strict=True)
            )
            if exact[0] >= 0:
                self.verification_stats["pen_sphere_cleared_by_exact_shape"] += 1
                continue
            margin = margins.get(exact[2], self.margin)
            return {
                **failure,
                "distance_m": float(exact[0] + margin),
                "margin_m": margin,
                "wire": scene.segments[exact[1]]["name"],
                "body": exact[2],
                "fraction": float(u),
            }
        return None

    def add_draw(self, scene, identity, q0, q1, radius, name):
        arm = self.arms[identity]
        a, b = [arm.robot.forward(q, arm.flange_to_tcp)[:3, 3] for q in (q0, q1)]
        inflation = (
            tcp_acceleration_bound(arm.robot, arm.flange_to_tcp, abs(q1 - q0), np.zeros(len(q0)))
            / 8
        )
        scene.add(a, b, radius + inflation + 1e-9, name)

    def add_piece(self, scene, identity, piece, radius, name):
        add_piece_material(scene, self.arms[identity], piece, radius, name)

    def add_clock_edge(self, scene, identity, edge, name, start=None, end=None):
        start = edge.start_s if start is None else start
        end = edge.end_s if end is None else end
        if edge.coefficients is None:
            self.add_draw(
                scene, identity, edge.position(start), edge.position(end), edge.radius_m, name
            )
        else:
            self.add_piece(scene, identity, edge.piece(start, end), edge.radius_m, name)


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

    def prepare_drawing(self, state, identity, drawing, q_start, *, defer_drawing_check=False):
        if defer_drawing_check:
            if self.profile.get("drawing_orientation_policy") not in {"fixed", "fixed_axis"}:
                raise ValueError(
                    "Deferred drawing checks require the fixed-orientation forward gate"
                )
            return self._prepare_drawing(
                state, identity, drawing, q_start, defer_drawing_check=True
            )
        arm = self.arms[identity]
        attempt = self._prepare_drawing(state, identity, drawing, q_start)
        if self.profile.get("drawing_orientation_policy") in {"fixed", "fixed_axis"}:
            return attempt
        if attempt is not None or not self.timing.get("repair_drawing_axis", len(self.arms) == 1):
            return attempt
        if monotonic() >= self.deadline:
            return None
        witness = self.attempts[-1] if self.attempts else {}
        if witness.get("phase") not in {"APPROACH", "DRAW", "WITHDRAW"}:
            return None
        free_axis = drawing.get("free_tool_orientation", False)
        if free_axis and (witness.get("phase") != "DRAW" or "positions" not in witness):
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
        restored_repair = prior.get("status") == "SAMPLED_FREE_AXIS_PROPOSAL" and all(
            prior.get(key) == value for key, value in binding.items()
        )
        if restored_repair:
            repair = prior
            print(
                f"Restored pen-orientation proposal for {identity}/{drawing['stroke_id']}",
                flush=True,
            )
        elif free_axis:
            # The sampled orientation path already passed tracking. Repair its
            # witnessed interior failure instead of resampling the whole path.
            repair = {
                "status": "SAMPLED_FREE_AXIS_PROPOSAL",
                "positions": drawing["positions"],
                "tcp_targets_m": drawing["tcp_targets_m"],
            }
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
        if free_axis:
            if restored_repair:
                # A retained correction is a proposal, even if a previous
                # preparation accepted it. Recheck it before continuing.
                attempt = self._prepare_drawing(state, identity, repaired, q_start)
            return self._repair_drawing_collisions(
                state, identity, repaired, q_start, attempt, report, repair_path
            )
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
        return self._repair_drawing_collisions(
            state, identity, repaired, q_start, attempt, report, repair_path
        )

    def _repair_drawing_collisions(
        self, state, identity, repaired, q_start, attempt, report, repair_path
    ):
        # Sampled pose clearance does not establish clearance between poses.
        # Refine at the continuous checker's actual failing interval before
        # abandoning this robot mode. The source polyline is unchanged and
        # every proposal still passes the ordinary full-motion validator.
        for correction in range(4):
            if attempt is not None or monotonic() >= self.deadline:
                break
            failure = self.attempts[-1]
            if failure.get("phase") != "DRAW" or "positions" not in failure:
                break
            corrected = repair_collision_witness(self, repaired, state.payload, failure)
            report.setdefault("collision_refinements", []).append(
                {
                    "correction": len(report.get("collision_refinements", [])) + 1,
                    "status": corrected["status"],
                    "trigger": failure,
                    "collocation": corrected.get("collocation"),
                }
            )
            if corrected["status"] != "SAMPLED_FREE_AXIS_PROPOSAL":
                break
            repaired.update(
                positions=corrected["positions"], tcp_targets_m=corrected["tcp_targets_m"]
            )
            report.update(
                positions=corrected["positions"], tcp_targets_m=corrected["tcp_targets_m"]
            )
            write_json(repair_path, report)
            print(
                f"Rechecking {identity}/{repaired['stroke_id']}: "
                f"collision correction {correction + 1}/4",
                flush=True,
            )
            attempt = self._prepare_drawing(state, identity, repaired, q_start)
        report["continuous_motion_status"] = "ACCEPTED" if attempt else "REJECTED"
        write_json(repair_path, report)
        # Backtracking can overwrite this section's working repair file. Keep
        # the exact accepted report immutable for any retained motion witness.
        if attempt:
            digest = sha256(repair_path)
            accepted = repair_path.parent / "accepted" / f"{repair_path.stem}-{digest}.json"
            write_json(accepted, report)
            attempt.payload.metadata["axis_repair_artifact"] = str(accepted.relative_to(self.run))
            attempt.payload.metadata["axis_repair_sha256"] = digest
        return attempt

    def outward_failure(self, arm, task, positions):
        """Apply the outside-in side-wall policy to proposed or replayed poses."""
        if self.profile.get("drawing_orientation_policy") in {"fixed", "fixed_axis"}:
            return None
        action = self.actions.get(task, {})
        if not action.get("parent_curve", "").startswith("zigzag-"):
            return None
        for index, q in enumerate(positions):
            point = arm.robot.forward(q, arm.flange_to_tcp)[:3, 3]
            value = outward_pen_value(self, arm, q, point)
            if value < -1e-8:
                return {
                    "status": "OUTWARD_POLICY_REJECTED",
                    "sample": index,
                    "outward_value": value,
                }
        return None

    def _prepare_drawing(self, state, identity, drawing, q_start, *, defer_drawing_check=False):
        local = self.prepare_local_drawing(identity, drawing)
        if local is None:
            return None
        return self.complete_local_drawing(
            state, identity, drawing, local, q_start, defer_drawing_check=defer_drawing_check
        )

    def prepare_local_drawing(self, identity, drawing):
        """Fix one drawing's joint interpolant and DRAW clock without any scene.

        Tracking, source corner holds and retiming depend only on the drawing
        and the cell. Their result determines the deposited enclosures, so a
        later action can be checked against them before this action's own
        approach, withdrawal and collision verdict exist.
        """
        arm = self.collision_arm(identity)
        task = drawing["stroke_id"]
        continuous = self.timing.get("drawing_clock") == "cubic_toppra"
        fixed_required = self.profile.get("drawing_orientation_policy") in {"fixed", "fixed_axis"}
        transform = (
            self.turntable.build_transform(drawing.get("table_index", 0))
            if self.turntable is not None
            else np.asarray(self.profile["world_from_build"])
        )
        vertex_cooling = float(self.process.get("vertex_cooling_s", 1.0))
        if not np.isfinite(vertex_cooling) or vertex_cooling < 0:
            raise ValueError("Vertex cooling must be finite and nonnegative")
        tracked = None
        if continuous and fixed_required and self.settings.get("path_pieces") == "members":
            oriented = orient_stroke(
                self.actions[task],
                reverse=drawing.get("reverse", False),
                start_visit=drawing.get("start_visit", 0),
            )
            ends = transform_points(
                np.asarray(oriented["segments"])[:, [0, -1]].reshape(-1, 3), transform
            ).reshape(-1, 2, 3)
            tracked = member_drawing_path(
                arm,
                drawing,
                np.r_[ends[:1, 0], ends[:, 1]],
                lambda reached: source_vertex_schedule(
                    self.actions[task], drawing, reached, transform, vertex_cooling, self.process
                )[0],
                self.deadline,
            )
        if tracked is None:
            tracked = tracked_drawing_path(
                arm, drawing, continuous, self.deadline, require_fixed_orientation=fixed_required
            )
        if tracked["status"] != "BOUNDED":
            self.failure(
                task,
                identity,
                "TRACKING",
                tracked["status"],
                **{key: value for key, value in tracked.items() if key != "status"},
            )
            return None
        q, spline = tracked["positions"], tracked["spline"]
        orientation_failure = self.outward_failure(arm, task, q)
        if orientation_failure is not None:
            self.failure(task, identity, "ORIENTATION", **orientation_failure)
            return None
        radius = self.actions[task]["diameter_m"] / 2
        # Numerical samples are not physical stop locations. Only actual source
        # direction changes split the clock, preserving a single action.
        targets = tracked["targets"][:, :3, 3]
        # Stop where the process audit will look for each corner: at the knot
        # whose actual tool position is nearest the source vertex. Ideal targets
        # can name a neighbouring knot when two lie within tracking error.
        reached = np.array([arm.robot.forward(joints, arm.flange_to_tcp)[:3, 3] for joints in q])
        corners, holds, vertical_end = source_vertex_schedule(
            self.actions[task], drawing, reached, transform, vertex_cooling, self.process
        )
        for index, hold in holds.items():
            # The join stays centred on the source vertex itself.
            hold["contact_m"] = targets[index].tolist()
        draw_operation = {
            "kind": "DRAW",
            "positions": q,
            "material_radius_m": radius,
            "stroke_id": task,
        }
        draw_operation.update(
            break_indices=corners,
            vertex_holds=holds,
            start_lag_s=self.process["start_lag_s"],
            stop_lag_s=self.process["stop_lag_s"],
        )
        if continuous:
            draw_operation["path"] = spline
        timed = []
        try:
            if continuous and self.geometry_first:
                timed = None
                draw_clock = geometric_path(
                    [draw_operation], arm, drawing_clock=self.timing["drawing_clock"]
                )
            else:
                draw_clock = self._timed([draw_operation], arm, drawing_capture=timed)
        except TimingParameterizationError as error:
            self.failure(
                task,
                identity,
                "TIMING",
                "TIMING_REJECTED",
                reason=str(error),
                drawing_clock=self.timing.get("drawing_clock"),
                acceleration_constraint=self.timing.get("acceleration_constraint"),
                path_samples=len(q),
                minimum_joint_step_rad=float(np.min(np.linalg.norm(np.diff(q, axis=0), axis=1))),
            )
            return None
        return {
            "tracked": tracked,
            "draw_operation": draw_operation,
            "draw_clock": draw_clock,
            "timed": timed,
            "radius": radius,
            "continuous": continuous,
            "vertex_cooling": vertex_cooling,
            "vertical_end": vertical_end,
        }

    def complete_local_drawing(
        self, state, identity, drawing, local, q_start, *, defer_drawing_check=False,
        chained_in=False, chained_out=False,
    ):
        """Add the scene-dependent approach and withdrawal around a fixed drawing.

        The DRAW polynomials come unchanged from ``prepare_local_drawing``;
        they are never retimed here. ``chained_in`` means the previous stroke
        ended at this one's first joint position with the pen down, so there is
        no travel, approach, feed start or join. ``chained_out`` means the next
        stroke takes over the same way, so there is no hold and no withdrawal;
        a free riser top still gets its cooling hold.
        """
        arm, scene = self.collision_arm(identity), state.payload
        task = drawing["stroke_id"]
        tracked, radius, continuous = local["tracked"], local["radius"], local["continuous"]
        vertex_cooling, vertical_end = local["vertex_cooling"], local["vertical_end"]
        q = tracked["positions"]
        # Full approach and its travel must be feasible together.
        approach_attempts = []
        # Free-axis drawing can leave the nozzle nearly horizontal. Pulling
        # along its actual body axis is then distinct from every world-up ray.
        # The ray still receives the same IK, tracking, and collision checks.
        approach_directions = [
            (-arm.robot.forward(q[0], arm.flange_to_tcp)[:3, 2]).tolist(),
            *self.transit["departure_directions"],
        ]
        if chained_in:
            approach = np.asarray([q[0]])
            travel = {"status": "CONNECTED", "positions": np.asarray([q[0]])}
            approach_directions = []
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
            # Regression samples local extrusion first, then connects its end to
            # an already planned suffix. Incoming travel belongs to its predecessor.
            travel = (
                {"status": "CONNECTED", "positions": np.asarray([approach[0]])}
                if q_start is None
                else self.travel(identity, scene, q_start, approach[0])
            )
            detail["travel"] = {key: value for key, value in travel.items() if key != "positions"}
            if travel["status"] == "CONNECTED":
                break
        else:
            if not chained_in:
                self.failure(
                    task, identity, "APPROACH", "NO_CONNECTED_APPROACH", attempts=approach_attempts
                )
                return None
        if chained_out:
            after, failure = scene, None
        elif defer_drawing_check and local.get("deposits") is not None:
            # The wire this drawing lays was already enclosed when its deposit
            # was predicted from the same path; withdrawal planning reuses it.
            after, failure = scene.copy(), None
            for number, row in enumerate(local["deposits"]):
                after.add(
                    row["start"], row["end"], row["radius_m"],
                    f"prospective/{identity}/{task}/{number}",
                )
        elif defer_drawing_check:
            after, failure = enclose_drawing_prefix(
                arm,
                tracked,
                scene,
                radius,
                self.deadline,
                name_prefix=f"prospective/{identity}/{task}",
                continuous=continuous,
            )
            if hasattr(self, "verification_stats"):
                self.verification_stats["prospective_draw_checks_deferred"] += 1
        else:
            after, failure = check_growing_drawing_prefix(
                arm,
                tracked,
                scene,
                self.obstacles,
                radius,
                self.margin,
                self.settings["maximum_subdivision_depth"],
                self.deadline,
                name_prefix=f"prospective/{identity}/{task}",
                continuous=continuous,
            )
        if failure is not None:
            self.failure(
                task,
                identity,
                "DRAW",
                failure["status"],
                **{key: value for key, value in failure.items() if key != "status"},
            )
            return None
        withdraw = (
            {"status": "SAMPLED_DRAWING_PATH", "positions": [q[-1]]}
            if chained_out
            else find_departure(
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
            local["draw_operation"],
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
        if vertical_end:
            release_cooling = max(release_cooling, vertex_cooling)
        # Feed decay has already elapsed at the final point before this hold.
        release_cooling = max(0.0, release_cooling - self.process["stop_lag_s"])
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
        if chained_in:
            operations = [op for op in operations
                          if op["kind"] not in {"TRAVEL", "APPROACH", "PEN_START_WAIT", "JOIN"}]
        if chained_out:
            start_lag, stop_lag = self.process["start_lag_s"], self.process["stop_lag_s"]
            operations = [op for op in operations
                          if op["kind"] not in {"PEN_STOP_WAIT", "WAIT", "WITHDRAW"}]
            if vertical_end:
                # A free riser top is held like any corner: feed off on
                # arrival, restarted in time for the hold's end.
                operations.extend(
                    {"kind": kind, "positions": [q[-1]], "contact_m": end_contact, "hold_s": hold}
                    for kind, hold in (
                        ("PEN_STOP_WAIT", stop_lag),
                        ("VERTEX_COOL", max(0.0, vertex_cooling - stop_lag - start_lag)),
                        ("PEN_START_WAIT", start_lag),
                    )
                    if hold > 0
                )
        for operation in operations:
            operation["stroke_id"] = task
        geometric = local["timed"] is None
        try:
            if geometric:
                path = geometric_path(
                    operations, arm, drawing_clock=self.timing["drawing_clock"]
                )
                path.timing_operations = operations
            else:
                path = self._timed(operations, arm, drawing_reuse=local["timed"])
        except TimingParameterizationError as error:
            self.failure(
                task,
                identity,
                "TIMING",
                "TIMING_REJECTED",
                reason=str(error),
                drawing_clock=self.timing.get("drawing_clock"),
                acceleration_constraint=self.timing.get("acceleration_constraint"),
                path_samples=len(q),
                minimum_joint_step_rad=float(np.min(np.linalg.norm(np.diff(q, axis=0), axis=1))),
            )
            return None
        if drawing.get("orientation_policy") is not None:
            rotations = drawing.get("tcp_rotations")
            if drawing["orientation_policy"] == "outward_radial" and (
                rotations is None or len(rotations) != len(drawing["tcp_targets_m"])
            ):
                # Pose repair may refine source targets. Regenerate only nominal
                # preference metadata; the accepted joint trajectory is unchanged.
                transform = (
                    self.turntable.build_transform(drawing.get("table_index", 0))
                    if self.turntable is not None
                    else np.asarray(self.profile["world_from_build"])
                )
                rotations = radial_pen_rotations(
                    arm,
                    drawing["tcp_targets_m"],
                    transform,
                    drawing.get("pen_tilt_deg", self.profile.get("drawing_pen_tilt_deg", 70.0)),
                    drawing.get("roll_rad", 0.0),
                ).tolist()
                drawing["tcp_rotations"] = rotations
            path.metadata["orientation_preference"] = {
                "policy": drawing["orientation_policy"],
                "pen_tilt_deg": drawing.get("pen_tilt_deg"),
                "preferred_pen_tilt_deg": drawing.get("preferred_pen_tilt_deg"),
                "seed_preference": drawing.get("seed_orientation_preference"),
                "roll_rad": drawing.get("roll_rad", 0.0),
                "source_points_m": drawing["tcp_targets_m"],
                "target_rotations": rotations,
                "constraint": "preferred_only_with_fresh_full_tool_collision_checks",
            }
            path.metadata["actual_pen_tilt"] = sampled_pen_tilt(arm, q)
        path.metadata.update(
            chained_in=bool(chained_in),
            chained_out=bool(chained_out),
            collision_scope=self.collision_scope,
            requires_full_robot_validation=self.collision_scope == "pen_candidate",
            process_clock_policy=PROCESS_CLOCK_POLICY,
            source_reverse=drawing.get("reverse", False),
            source_start_visit=drawing.get("start_visit", 0),
            vertex_feed_policy="stop_with_lag_then_restart",
            vertex_cooling_s=vertex_cooling,
            prospective_material_check=(
                "DEFERRED_TO_FULL_CLOCK" if defer_drawing_check else "CHECKED_GEOMETRIC_PREFIX"
            ),
        )
        if not geometric:
            self._process_events(path, task, drawing.get("table_index", 0))
        path.metadata["tracking_position_bound_m"] = tracked["position_error_bound_m"]
        path.metadata["tracking_orientation_constraint"] = tracked["orientation_constraint"]
        if tracked["orientation_constraint"] == "fixed":
            path.metadata["fixed_tcp_rotation"] = tracked["fixed_tcp_rotation"]
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
            f"Prepared {identity}/{task}/{drawing['variant']}: "
            + (f"{len(path.edges)} untimed edges" if geometric else f"{path.duration_s:.1f}s"),
            flush=True,
        )
        return proposal

    def _timed(self, operations, arm, **reuse):
        return timed_path(
            operations,
            arm,
            arm.robot.velocity * self.timing["velocity_fraction"],
            np.array(self.timing["joint_acceleration_rad_s2"]),
            self.process["nominal_speed_m_s"],
            self.timing["travel_speed_m_s"],
            drawing_clock=self.timing.get("drawing_clock", "quintic_stop_at_geometric_knots"),
            acceleration_constraint=self.timing.get("acceleration_constraint", "interpolated"),
            grid_samples=self.timing.get("retiming_grid_samples", 101),
            grid_midpoints=self.timing.get("retiming_grid_midpoints", True),
            solver=self.timing.get("retiming_solver", "toppra"),
            grid_ramps=self.timing.get("retiming_grid_ramps", False),
            **reuse,
        )

    def _process_events(self, path, task, table_index):
        transform = (
            self.turntable.build_transform(table_index)
            if self.turntable is not None
            else np.asarray(self.profile["world_from_build"])
        )
        path.metadata["deposition_events"] = derive_deposition_events(
            path, self.actions[task], self.settings["assumed_cooling_s"], transform
        )
        path.metadata["release_ready_s"] = max(
            event["ready_at_s"] for event in path.metadata["deposition_events"]
        )

    def time_local_drawing(self, identity, geometric, task, table_index):
        """Time an accepted geometric action and prove its sweep is unchanged.

        Raises ``TimingParameterizationError`` when no clock exists, and
        ``ValueError`` when the timed clock would leave the checked geometry.
        """
        from .geometric_path import same_sweep

        path = self._timed(geometric.timing_operations, self.collision_arm(identity))
        guard = same_sweep(geometric, path)
        if guard["status"] != "SAME_SWEEP":
            raise ValueError(f"Timed clock leaves its checked geometry: {guard}")
        path.metadata.update(
            {key: value for key, value in geometric.metadata.items() if key != "parameterization"}
        )
        path.metadata["same_sweep"] = guard
        self._process_events(path, task, table_index)
        return path

    def validate_interval(self, state, new, end, *, deposit_only=False, journal=None):
        started, queries = perf_counter(), Counter(QUERY_STATS)
        try:
            return self._validate_interval(
                state, new, end, deposit_only=deposit_only, journal=journal
            )
        finally:
            if hasattr(self, "verification_seconds"):
                self.verification_seconds[
                    "deposit_only" if deposit_only else "interval_validation"
                ] += perf_counter() - started
                for name, value in (Counter(QUERY_STATS) - queries).items():
                    target = (
                        self.verification_seconds
                        if name.endswith("_seconds")
                        else self.verification_stats
                    )
                    target[f"query_{name}"] += value

    def _validate_interval(self, state, new, end, *, deposit_only=False, journal=None):
        """Check one scheduling interval while its material grows.

        ``deposit_only`` runs the identical turn and deposition sequence but no
        collision or orientation check, and grants no certificate. ``journal``
        receives those material operations in order, so another process can
        rebuild the same scene before this interval's verdict exists.
        """
        motions = {m.proposal.arm: m for m in state.active + new}
        # An untimed path has no seconds: cooling and indexed-clock audits wait
        # for its timing pass. Every geometric check below is unchanged.
        untimed = {
            m.proposal.payload.metadata.get("parameterization") == "geometric"
            for m in motions.values()
        }
        if True in untimed and (len(untimed) > 1 or len(self.arms) != 1):
            raise ValueError("Untimed validation needs one arm and only untimed paths")
        untimed = True in untimed
        # Reuse only the clipped contact geometry within this call. Tool,
        # static obstacles, radii and deadline are fixed here; every joint
        # edge still receives fresh arm and tip checks. Deposition or an
        # indexed turn changes the scene token. Raw replay rebuilds each time.
        contact_validators = {}
        certificate_records, invariant_hits, invariant_proofs = {}, set(), {}
        deposition_enclosures = {identity: {} for identity in motions}
        edge_indices = {
            identity: {id(edge): index for index, edge in enumerate(motion.proposal.payload.edges)}
            for identity, motion in motions.items()
        }
        store = None if deposit_only else getattr(self, "path_certificates", None)
        full_scope = self.collision_scope == "full_robot"
        invariant_scope = "invariant_path" if full_scope else "pen_invariant_path"
        if store is not None and not getattr(self, "verification_raw", False):
            for identity, motion in motions.items():
                context = self.path_certificate_context(identity)
                record = motion.proposal.payload.record()
                certificate_records[identity] = (context, record)
                proof = store.get(context, record, invariant_scope)
                if proof is not None:
                    invariant_hits.add(identity)
                    invariant_proofs[identity] = proof
                    self.verification_stats[f"{invariant_scope}_hits"] += 1
                else:
                    self.verification_stats[f"{invariant_scope}_misses"] += 1
        fixed_required = self.profile.get("drawing_orientation_policy") in {"fixed", "fixed_axis"}
        orientation_hits = set()
        if store is not None and not getattr(self, "verification_raw", False):
            for identity, (context, record) in certificate_records.items():
                if store.get(context, record, "drawing_orientation") is not None:
                    orientation_hits.add(identity)
                    self.verification_stats["drawing_orientation_hits"] += 1
        fixed_references = {}
        if fixed_required and not deposit_only:
            for identity, motion in motions.items():
                path = motion.proposal.payload
                if path.metadata.get("tracking_orientation_constraint") == "free":
                    return None, {"status": "FIXED_ORIENTATION_REQUIRED", "arm": identity}
                drawing_edges = [edge for edge in path.edges if edge.kind == "DRAW"]
                strokes = {edge.stroke for edge in drawing_edges}
                for edge in drawing_edges:
                    key = (identity, edge.stroke)
                    if key not in fixed_references:
                        declared = (
                            path.metadata.get("fixed_tcp_rotation") if len(strokes) == 1 else None
                        )
                        fixed_references[key] = (
                            np.asarray(declared)
                            if declared is not None
                            else self.arms[identity].robot.forward(
                                edge.q0, self.arms[identity].flange_to_tcp
                            )[:3, :3]
                        )
        # A motion that lays no wire and turns no table leaves the scene as it is.
        changes = any(
            edge.kind in {"DRAW", "INDEX_TURN"}
            for motion in motions.values()
            for edge in motion.proposal.payload.edges
        )
        enabled = not getattr(self, "verification_raw", False)
        scene = (
            state.payload
            if not changes and state.payload.clearance_query_enabled == enabled
            else state.payload.copy()
        )
        scene.clearance_query_enabled = enabled
        if self.turntable is not None and not deposit_only and not untimed:
            from .turntable import audit_indexed_clock

            for motion in motions.values():
                audit_indexed_clock(
                    self.turntable,
                    motion.proposal.payload,
                    self.arms[motion.proposal.arm].initial_positions,
                    self.settings["assumed_cooling_s"],
                )
        spheres = str(self.settings.get("pen_check", "")).startswith("spheres")
        if spheres and len(self.arms) != 1:
            raise ValueError("The sphere pen check supports one arm")
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
            if self.turntable is not None:
                identity = next(iter(self.arms))
                edge, position, _ = paths[identity]
                if edge is not None:
                    if edge.table_index != scene.table_index:
                        return None, {"status": "TABLE_INDEX_MISMATCH", "time_s": a}
                    if edge.kind == "INDEX_TURN":
                        if not untimed and any(
                            ready > a + 1e-9 for _, ready in state.support_ready
                        ):
                            return None, {"status": "TABLE_TURN_BEFORE_COOLING", "time_s": a}
                        motion = motions[identity]
                        if (
                            abs(a - motion.start_s - edge.start_s) > 1e-9
                            or abs(b - motion.start_s - edge.end_s) > 1e-9
                        ):
                            return None, {"status": "PARTIAL_TURN_INTERVAL"}
                        if not deposit_only:
                            indexed_arm = replace(
                                self.collision_arm(identity),
                                collision=self.collision_tip_model(identity),
                            )
                            check = self.turntable.check_parked_turn(
                                scene,
                                edge.table_index,
                                edge.table_to_index,
                                indexed_arm,
                                position(0),
                                self.obstacles,
                                margin_m=self.margin,
                                maximum_depth=self.settings["maximum_subdivision_depth"],
                                deadline=self.deadline,
                            )
                            if check["status"] != "CERTIFIED_TURNTABLE_INTERVAL":
                                return None, {"phase": "TABLE_TURN", "time_s": a, **check}
                        from .turntable import index_angle

                        scene = self.turntable.reposition(
                            scene, index_angle(edge.table_index), index_angle(edge.table_to_index)
                        )
                        if journal is not None:
                            journal.append(("turn", edge.table_index, edge.table_to_index))
                        if not deposit_only:
                            self.checked_intervals += 1
                        continue
            # Deposit conservative enclosures of BOTH growing strokes before
            # checking the interval. Future intervals are never installed early.
            pen_steps = {}
            for identity, (edge, _position, _) in paths.items():
                if edge is not None and edge.kind == "DRAW":
                    motion = motions[identity]
                    name = f"material/{identity}/{edge.stroke}/{a:.12f}-{b:.12f}"
                    interval_key = (
                        f"{edge_indices[identity][id(edge)]}:"
                        f"{float(a - motion.start_s).hex()}:"
                        f"{float(b - motion.start_s).hex()}"
                    )
                    cached_capsules = (
                        invariant_proofs.get(identity, {})
                        .get("deposition_enclosures", {})
                        .get(interval_key)
                    )
                    started = perf_counter()
                    before = len(scene.segments)
                    # The sphere check looks at the pen at these fractions of
                    # the interval, each against the wire laid up to there, so
                    # the wire is deposited in the same steps.
                    steps = (
                        self.pen_sample_fractions(identity, _position) if spheres else [0.0, 1.0]
                    )
                    marks = [before]
                    if cached_capsules is not None:
                        for capsule in cached_capsules:
                            scene.add(
                                capsule["start"],
                                capsule["end"],
                                capsule["radius_m"],
                                name + capsule["suffix"],
                            )
                        laid = [
                            int(c["suffix"][1:].split("/")[0]) if c["suffix"].startswith("@") else 0
                            for c in cached_capsules
                        ]
                        marks.extend(
                            before + sum(step <= j for step in laid)
                            for j in range(len(steps) - 1)
                        )
                        self.verification_stats["deposition_intervals_reused"] += 1
                        self.verification_stats["deposition_capsules_reused"] += len(
                            cached_capsules
                        )
                        deposition_enclosures[identity][interval_key] = cached_capsules
                    else:
                        low, high = a - motion.start_s, b - motion.start_s
                        for j, (first, second) in enumerate(
                            zip(steps[:-1], steps[1:], strict=True)
                        ):
                            self.add_clock_edge(
                                scene,
                                identity,
                                edge,
                                name if j == 0 else f"{name}@{j}",
                                low if j == 0 else low + (high - low) * first,
                                high if j == len(steps) - 2 else low + (high - low) * second,
                            )
                            marks.append(len(scene.segments))
                        deposition_enclosures[identity][interval_key] = [
                            {
                                "start": row["start"],
                                "end": row["end"],
                                "radius_m": row["radius_m"],
                                "suffix": row["name"][len(name) :],
                            }
                            for row in scene.segments[before:]
                        ]
                    pen_steps[identity] = (steps, marks)
                    if journal is not None:
                        journal.append(("add", [dict(row) for row in scene.segments[before:]]))
                    if hasattr(self, "verification_seconds"):
                        self.verification_seconds["deposition_enclosures"] += (
                            perf_counter() - started
                        )
            if deposit_only:
                continue
            for identity, (edge, position, deviation) in paths.items():
                arm = self.collision_arm(identity)
                if edge is not None and edge.kind == "DRAW":
                    if (fixed_required and identity not in invariant_hits
                            and identity not in orientation_hits):
                        started = perf_counter()
                        motion = motions[identity]
                        failure = fixed_clock_orientation_failure(
                            arm,
                            edge,
                            fixed_references[(identity, edge.stroke)],
                            a - motion.start_s,
                            b - motion.start_s,
                        )
                        if failure is not None:
                            return None, {**failure, "arm": identity, "time_s": a}
                        if hasattr(self, "verification_seconds"):
                            self.verification_seconds["fixed_orientation"] += (
                                perf_counter() - started
                            )
                    orientation_failure = self.outward_failure(
                        arm, edge.stroke, [position(0), position(0.5), position(1)]
                    )
                    if orientation_failure is not None:
                        return None, {**orientation_failure, "arm": identity, "time_s": a}
                if edge is not None and edge.contact is not None:
                    # Keep the original stroke endpoint as the contact center,
                    # rather than moving the exemption ball with the nozzle.
                    contact_key = (
                        identity,
                        scene.geometry_prefix(),
                        np.asarray(edge.contact, dtype=np.float64).tobytes(),
                    )
                    reuse_contact = getattr(
                        self, "optimize_contact_geometry",
                        not getattr(self, "verification_raw", False),
                    )
                    prior = contact_validators.get(identity) if reuse_contact else None
                    valid = prior[1] if prior is not None and prior[0] == contact_key else None
                    if valid is None:
                        started = perf_counter()
                        valid = contact_validator(
                            arm.robot,
                            arm.collision,
                            self.collision_tip_model(identity),
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
                                self.profile["environment"].get(
                                    "platform_top_z_m", self.profile["world_from_build"][2][3]
                                ),
                                self.transit["contact_radius_m"],
                                riser_z=(
                                    self.profile["world_from_build"][2][3]
                                    if "build_riser" in self.profile["environment"]
                                    else None
                                ),
                            ),
                            deadline=self.deadline,
                            optimize_contact_geometry=reuse_contact,
                        )
                        self.verification_seconds["contact_geometry_setup"] += (
                            perf_counter() - started
                        )
                        self.verification_stats["contact_geometry_constructed"] += 1
                        if reuse_contact:
                            contact_validators[identity] = (contact_key, valid)
                    else:
                        self.verification_stats["contact_geometry_reused"] += 1
                    if not valid(position(0), position(1)):
                        return None, {
                            "status": "CONTACT_PATH_REJECTED",
                            "arm": identity,
                            "time_s": a,
                            "evidence": valid.failures[:2],
                        }
                elif edge is not None and edge.kind == "DRAW" and spheres:
                    # The pen as a row of spheres against the wire, at both ends
                    # of the interval and at poses no further apart than the
                    # sampling distance. The cell's fixed obstacles keep the
                    # continuous check.
                    started = perf_counter()
                    if identity not in invariant_hits:
                        check = arm.collision.check_bounded_path(
                            position,
                            deviation,
                            self.obstacles,
                            self.margin,
                            self.settings["maximum_subdivision_depth"],
                            None,
                            self.deadline,
                        )
                        if check["status"] != "CERTIFIED_MODELED_EDGE":
                            return None, {"arm": identity, "time_s": a, **check}
                    self.verification_seconds["pen_fixed_obstacles"] += perf_counter() - started
                    started = perf_counter()
                    failure = self.pen_wire_failure(identity, scene, position, *pen_steps[identity])
                    self.verification_seconds["pen_spheres"] += perf_counter() - started
                    if failure is not None:
                        return None, {"arm": identity, "time_s": a, **failure}
                else:
                    model = (
                        arm.collision
                        if edge is not None and edge.kind == "DRAW"
                        else self.collision_tip_model(identity)
                    )
                    if identity in invariant_hits:
                        from .collision_context import MaterialOnlyCollisionModel

                        model = MaterialOnlyCollisionModel(model)
                        self.verification_stats[
                            "static_intervals_reused"
                            if full_scope
                            else "pen_static_intervals_reused"
                        ] += 1
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
            self.checked_intervals += 1
        if store is not None and not getattr(self, "verification_raw", False):
            for identity, motion in motions.items():
                # A partial scheduling interval cannot certify the full clock.
                if state.time_s <= motion.start_s and end >= motion.end_s:
                    context, record = certificate_records[identity]
                    if (fixed_required and identity not in orientation_hits
                            and any(edge.kind == "DRAW" for edge in motion.proposal.payload.edges)):
                        store.admit(context, record, "drawing_orientation", {"status": "BOUNDED"})
                    if identity not in invariant_hits:
                        store.admit(
                            context,
                            record,
                            invariant_scope,
                            {
                                "status": "CERTIFIED_MODELED_PATH"
                                if full_scope
                                else "CERTIFIED_PEN_MODELED_PATH",
                                "scope": "self_static_and_fixed_orientation_only"
                                if full_scope
                                else "pen_static_and_fixed_orientation_only",
                                "collision_scope": self.collision_scope,
                                "material_checked_fresh": True,
                                "deposition_enclosures": deposition_enclosures[identity],
                            },
                        )
        return scene, None
