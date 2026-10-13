"""Verified local drawing examples; no construction or execution acceptance implied.

The teacher uses the existing FK, redundant IK, layered path search and continuous
collision checker. Every positive is checked at its actual joint interpolation,
including conservative material accumulated while drawing. Candidate collisions
and unresolved searches remain distinct labels.
"""

import hashlib
import json
from collections import Counter
from pathlib import Path
from time import monotonic, perf_counter

import numpy as np
from numpy.polynomial import Polynomial

from .cell import load_cell
from .io import read_json, sha256
from .kinematics import inverse_candidates, pose_error
from .planning.clock_path import ClockEdge, ClockPath
from .planning.curve_motion import search_layers
from .planning.motion_library import (
    CachedCollisionModel,
    MaterialOnlyCollisionModel,
    collision_context_hash,
    obstacle_context,
)
from .planning.motion_runtime import static_shapes
from .planning.path_certificates import PathCertificateStore
from .planning.scene import MaterialScene
from .planning.smooth_audit import PolynomialPiece, bounded_segments, check_tracking
from .planning.source_coverage import audit_source_curve
from .planning.turntable import IndexedTurntable

VALIDATOR = "continuous_local_draw_v1"


def digest(value):
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()


def scene_from_record(scene):
    result = MaterialScene()
    for i, row in enumerate(scene.get("capsules", [])):
        result.add(
            row.get("start", row.get("start_m")),
            row.get("end", row.get("end_m")),
            row["radius_m"],
            row.get("name", f"scene-{i}"),
        )
    return result


class DrawingTeacher:
    def __init__(
        self,
        profile,
        *,
        margin_m=0.001,
        tracking_m=0.0003,
        verification_directory=None,
        verification_raw=False,
    ):
        self.profile_path = Path(profile).resolve()
        self.profile, arms, self.obstacles = load_cell(self.profile_path)
        if len(arms) != 1:
            raise ValueError("Dataset generation requires the indexed single arm")
        self.arm = next(iter(arms.values()))
        if verification_raw:
            while isinstance(
                self.arm.collision, (CachedCollisionModel, MaterialOnlyCollisionModel)
            ):
                self.arm.collision = self.arm.collision.model
        self.table = IndexedTurntable.from_profile(self.profile)
        self.margin, self.tracking = margin_m, tracking_m
        self.model_signature = collision_context_hash(self.arm.collision, self.arm.tool.metadata)
        self.context = digest(
            {
                "model": self.model_signature,
                "profile": self.profile,
            }
        )
        self.verification_raw = verification_raw
        self.verification_counts, self.verification_seconds = Counter(), Counter()
        self.material_metrics = Counter()
        self._known_obstacles = dict(self.obstacles)
        self._static_shapes = static_shapes(self.profile, self.table)
        self._obstacle_references = {}
        self.certificates = None
        if verification_directory is not None and not verification_raw:
            directory = Path(verification_directory)
            self.arm.collision = CachedCollisionModel(
                self.arm.collision,
                directory,
                geometry_context=self.arm.tool.metadata,
                static_geometry=self._static_shapes,
            )
            self.certificates = PathCertificateStore(directory / "teacher_paths")

    def _material(self, scene):
        material = scene_from_record(scene)
        material.clearance_query_enabled = not self.verification_raw
        return material

    def _static_context(self):
        # Declared primitive dimensions describe only the imported obstacles.
        # Replacements require an independent key even at an unchanged pose.
        shapes = {
            name: shape
            for name, shape in self._static_shapes.items()
            if self.obstacles.get(name) is self._known_obstacles.get(name)
        }
        if isinstance(self.arm.collision, CachedCollisionModel):
            self.arm.collision.static_geometry = shapes
        return obstacle_context(self.obstacles, shapes, self._obstacle_references)

    def verification_metrics(self):
        model = self.arm.collision
        return {
            "counts": dict(self.verification_counts),
            "seconds": dict(self.verification_seconds),
            "path_certificates": None if self.certificates is None else self.certificates.report(),
            "clearance": {
                "hits": getattr(model, "hits", 0),
                "misses": getattr(model, "misses", 0),
                "self_static_seconds": getattr(model, "self_static_seconds", 0),
                "material_seconds": getattr(model, "material_seconds", 0),
            },
            "material": dict(self.material_metrics),
            "raw": self.verification_raw,
        }

    def signature(self, configurations, scene):
        self._static_context()
        material = self._material(scene)
        started = perf_counter()
        try:
            return [
                int(bool(self.arm.collision.check(q, self.obstacles, self.margin, material)))
                for q in configurations
            ]
        finally:
            self.verification_seconds["signature"] += perf_counter() - started
            self.material_metrics.update(material.clearance_stats)

    def validate(self, request, q_path, scene, *, deadline=float("inf"), radius_m=0.0005):
        started = perf_counter()
        self.verification_counts["calls"] += 1
        self._active_material = None
        try:
            result = self._validate(request, q_path, scene, deadline=deadline, radius_m=radius_m)
            self.verification_counts[result["status"]] += 1
            return result
        finally:
            self.verification_seconds["total"] += perf_counter() - started
            if self._active_material is not None:
                self.material_metrics.update(self._active_material.clearance_stats)
            if isinstance(self.arm.collision, CachedCollisionModel) and self.arm.collision._pending:
                self.arm.collision.flush()

    def _validate(self, request, q_path, scene, *, deadline, radius_m):
        """Certify actual piecewise-linear joints, not just waypoint feasibility."""
        started = monotonic()
        points, joints = np.asarray(request["source_points_m"]), np.asarray(q_path)
        base = {
            "validator": VALIDATOR,
            "context_hash": self.context,
            "representation": "piecewise_linear_joint_v1",
            "fitted_path_verified": False,
            "scope": "local continuous DRAW geometry; transit/process checked by runtime",
            "clearance_m": self.margin,
            "tracking_limit_m": self.tracking,
            "progressive_deposition": True,
            "material_radius_m": radius_m,
        }
        if (
            points.shape != (len(joints), 3)
            or joints.ndim != 2
            or joints.shape[1] != 7
            or len(joints) < 2
        ):
            return {**base, "status": "CONSTRAINT_VIOLATION", "reason": "shape"}
        if not np.isfinite(points).all() or not np.isfinite(joints).all():
            return {**base, "status": "CONSTRAINT_VIOLATION", "reason": "nonfinite"}
        material = self._material(scene)
        self._active_material = material
        rotations = request.get("allowed_orientations", [])
        declared = request.get("fixed_tcp_rotation")
        rotation = (
            np.asarray(declared if declared is not None else rotations[0])
            if (declared is not None or rotations)
            else None
        )
        worst = 0.0
        try:
            if monotonic() >= deadline:
                raise TimeoutError
            if rotation is None:
                rotation = self.arm.robot.forward(joints[0], self.arm.flange_to_tcp)[:3, :3]
            if (
                rotation.shape != (3, 3)
                or not np.isfinite(rotation).all()
                or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7, rtol=0)
                or np.linalg.det(rotation) < 0.999999
            ):
                raise ValueError("Invalid fixed TCP rotation")
            if not np.isfinite(radius_m) or radius_m <= 0:
                raise ValueError("Material radius must be finite and positive")
            model = getattr(self.arm.collision, "model", self.arm.collision)
            if collision_context_hash(model, self.arm.tool.metadata) != self.model_signature:
                raise ValueError("Teacher robot/tool model changed; rebuild the teacher")
            binding = {
                "teacher_policy": "fixed_piecewise_linear_invariants_v1",
                "context_hash": self.context,
                "static_context": self._static_context(),
                "margin_m": self.margin,
                "tracking_m": self.tracking,
                "orientation_tolerance_rad": 0.003,
                "radius_m": radius_m,
                "table_index": request.get("table_index", 0),
            }
            path_record = {
                "request": request,
                "q_path": joints,
                "fixed_tcp_rotation": rotation,
                "flange_to_tcp": self.arm.flange_to_tcp,
                "representation": "piecewise_linear_joint_v1",
            }
            invariant = (
                None
                if self.certificates is None
                else self.certificates.get(binding, path_record, "invariant_path")
            )
            if invariant is not None and (
                len(invariant.get("enclosures_by_interval", [])) != len(joints) - 1
                or invariant.get("tracking_status") != "BOUNDED"
            ):
                invariant = None
            self.verification_counts["invariant_hits" if invariant else "invariant_misses"] += 1
            collision = (
                MaterialOnlyCollisionModel(self.arm.collision) if invariant else self.arm.collision
            )
            enclosures = []
            if request.get("q_start") is not None and not np.allclose(
                joints[0], request["q_start"], atol=1e-7, rtol=0
            ):
                return {**base, "status": "CONSTRAINT_VIOLATION", "reason": "q_start"}
            if request.get("q_goal") is not None and not np.allclose(
                joints[-1], request["q_goal"], atol=1e-7, rtol=0
            ):
                return {**base, "status": "CONSTRAINT_VIOLATION", "reason": "q_goal"}
            # Explicit witnesses are cheap negatives. In particular, do not let an
            # earlier unresolved near-margin interval hide a later real collision.
            for i, q in enumerate(joints):
                collision_start = perf_counter()
                witnesses = collision.check(
                    q, self.obstacles, self.margin, material, deadline=deadline
                )
                self.verification_seconds["collision"] += perf_counter() - collision_start
                if witnesses:
                    return {
                        **base,
                        "status": "COLLISION",
                        "reason": "sampled_clearance_witness",
                        "knot": i,
                        "witness": witnesses,
                        "seconds": monotonic() - started,
                    }
            for i, (a, b) in enumerate(zip(joints[:-1], joints[1:], strict=True)):
                if monotonic() >= deadline:
                    raise TimeoutError
                self.arm.robot.frames(a)
                self.arm.robot.frames(b)
                piece = PolynomialPiece([Polynomial([x, y - x]) for x, y in zip(a, b, strict=True)])
                if invariant:
                    track = {
                        "status": "BOUNDED",
                        "position_error_bound_m": invariant["tracking_bound_m"],
                    }
                else:
                    tracking_start = perf_counter()
                    track = check_tracking(
                        piece,
                        self.arm.robot,
                        self.arm.flange_to_tcp,
                        points[i],
                        points[i + 1],
                        rotation,
                        self.tracking,
                        0.003,
                    )
                    self.verification_seconds["tracking"] += perf_counter() - tracking_start
                worst = max(worst, track["position_error_bound_m"])
                if track["status"] != "BOUNDED":
                    observed = (
                        track.get("witness", {}).get("position_error_m", 0) > self.tracking
                        or track.get("witness", {}).get("orientation_error_rad", 0) > 0.003
                    )
                    return {
                        **base,
                        "status": "CONSTRAINT_VIOLATION" if observed else "UNRESOLVED",
                        "reason": "tracking",
                        "segment": i,
                        "witness": track,
                        "seconds": monotonic() - started,
                    }
                if invariant:
                    interval_capsules = invariant["enclosures_by_interval"][i]
                else:
                    enclosure_start = perf_counter()
                    interval_capsules = []
                    for lo, hi, inflation in bounded_segments(
                        piece, self.arm.robot, self.arm.flange_to_tcp, 0.00002
                    ):
                        ends = [
                            self.arm.robot.forward(piece.position(t), self.arm.flange_to_tcp)[:3, 3]
                            for t in (lo, hi)
                        ]
                        interval_capsules.append(
                            {
                                "start": ends[0].tolist(),
                                "end": ends[1].tolist(),
                                "radius_m": radius_m + inflation + 1e-9,
                            }
                        )
                    self.verification_seconds["enclosures"] += perf_counter() - enclosure_start
                enclosures.append(interval_capsules)
                for j, capsule in enumerate(interval_capsules):
                    material.add(
                        capsule["start"], capsule["end"], capsule["radius_m"], f"generated-{i}-{j}"
                    )
                collision_start = perf_counter()
                check = collision.check_edge(
                    a, b, self.obstacles, self.margin, 20, material, deadline=deadline
                )
                self.verification_seconds["collision"] += perf_counter() - collision_start
                if check["status"] != "CERTIFIED_MODELED_EDGE":
                    return {
                        **base,
                        "status": "COLLISION" if check["status"] == "COLLISION" else "UNRESOLVED",
                        "reason": "collision_check",
                        "segment": i,
                        "witness": check,
                        "seconds": monotonic() - started,
                    }
        except TimeoutError:
            return {
                **base,
                "status": "UNRESOLVED",
                "reason": "deadline",
                "seconds": monotonic() - started,
            }
        except ValueError as error:
            return {
                **base,
                "status": "CONSTRAINT_VIOLATION",
                "reason": str(error),
                "seconds": monotonic() - started,
            }
        edges = [
            ClockEdge(
                float(i),
                float(i + 1),
                a,
                b,
                "DRAW",
                "dataset",
                coefficients=[[float(x), float(y - x)] for x, y in zip(a, b, strict=True)],
            )
            for i, (a, b) in enumerate(zip(joints[:-1], joints[1:], strict=True))
        ]
        source_audit = (
            None
            if self.certificates is None
            else self.certificates.get(binding, path_record, "source_coverage")
        )
        source_cached = source_audit is not None
        if source_audit is None:
            source_start = perf_counter()
            source_audit = audit_source_curve(
                self.arm,
                [ClockPath(edges)],
                request.get("source_reference_points_m", points),
                self.tracking,
            )
            self.verification_seconds["source_coverage"] += perf_counter() - source_start
        if source_audit["status"] != "BOUNDED":
            return {
                **base,
                "status": "UNRESOLVED",
                "reason": "source_coverage",
                "witness": source_audit,
                "seconds": monotonic() - started,
            }
        if self.certificates is not None:
            if invariant is None:
                self.certificates.admit(
                    binding,
                    path_record,
                    "invariant_path",
                    {
                        "status": "CERTIFIED_MODELED_PATH",
                        "tracking_status": "BOUNDED",
                        "tracking_bound_m": worst,
                        "fixed_tcp_rotation": rotation,
                        "enclosures_by_interval": enclosures,
                    },
                )
            if not source_cached:
                self.certificates.admit(binding, path_record, "source_coverage", source_audit)
        return {
            **base,
            "source_coverage": source_audit,
            "status": "VERIFIED_VALID",
            "fitted_path_verified": True,
            "tracking_bound_m": worst,
            "seconds": monotonic() - started,
        }

    def adapt(self, request, seed_path, *, deadline):
        """Local pose-constrained adaptation using previous and retrieved seeds."""
        points = np.asarray(request["source_points_m"])
        rotations = (
            [request["fixed_tcp_rotation"]]
            if request.get("fixed_tcp_rotation") is not None
            else request.get("allowed_orientations", [])
        )
        rotation = np.asarray(rotations[0]) if rotations else None
        result = []
        for i, point in enumerate(points):
            if monotonic() >= deadline:
                return None
            target = np.eye(4)
            target[:3, 3] = point
            target[:3, :3] = (
                rotation
                if rotation is not None
                else self.arm.robot.forward(seed_path[i], self.arm.flange_to_tcp)[:3, :3]
            )
            seeds = ([result[-1]] if result else []) + [seed_path[i]]
            solutions = inverse_candidates(
                self.arm.robot,
                target,
                self.arm.flange_to_tcp,
                seeds,
                position_tolerance=0.00005,
                orientation_tolerance=0.001,
                maximum_evaluations=40,
            )
            if not solutions:
                return None
            reference = result[-1] if result else seed_path[i]
            result.append(
                min(
                    (row["positions"] for row in solutions),
                    key=lambda q: np.linalg.norm(np.asarray(q) - reference),
                )
            )
        if request.get("q_start") is not None:
            result[0] = request["q_start"]
        if request.get("q_goal") is not None:
            result[-1] = request["q_goal"]
        return result

    def plan(self, request, seeds, *, deadline, scene=None):
        """Bounded multi-seed existing layered planner; output still needs validate."""
        points = np.asarray(request["source_points_m"])
        material = self._material(scene) if scene else None
        targets = np.repeat(np.eye(4)[None], len(points), axis=0)
        targets[:, :3, 3] = points
        rotations = (
            [request["fixed_tcp_rotation"]]
            if request.get("fixed_tcp_rotation") is not None
            else request.get("allowed_orientations", [])
        )
        targets[:, :3, :3] = (
            rotations[0]
            if rotations
            else self.arm.robot.forward(
                request.get("q_start") if request.get("q_start") is not None else seeds[0],
                self.arm.flange_to_tcp,
            )[:3, :3]
        )
        if not rotations and request.get("q_goal") is not None:
            targets[-1, :3, :3] = self.arm.robot.forward(request["q_goal"], self.arm.flange_to_tcp)[
                :3, :3
            ]

        def solve(target, previous):
            if monotonic() >= deadline:
                return []
            fixed = request.get("q_start") if np.array_equal(target, targets[0]) else None
            if fixed is None and np.array_equal(target, targets[-1]):
                fixed = request.get("q_goal")
            if fixed is not None:
                fixed = np.asarray(fixed, float)
                error = pose_error(self.arm.robot.forward(fixed, self.arm.flange_to_tcp), target)
                if np.linalg.norm(error[:3]) > 0.00005 or (
                    rotations and np.linalg.norm(error[3:]) > 0.001
                ):
                    return []
                return (
                    []
                    if self.arm.collision.check(fixed, self.obstacles, self.margin, material)
                    else [fixed]
                )
            return [
                s["positions"]
                for s in inverse_candidates(
                    self.arm.robot,
                    target,
                    self.arm.flange_to_tcp,
                    previous,
                    position_tolerance=0.00005,
                    orientation_tolerance=0.001,
                    maximum_evaluations=40,
                )
                if not self.arm.collision.check(
                    s["positions"], self.obstacles, self.margin, material
                )
            ]

        def check(a, b, p0, p1):
            piece = PolynomialPiece([Polynomial([x, y - x]) for x, y in zip(a, b, strict=True)])
            tracking = check_tracking(
                piece,
                self.arm.robot,
                self.arm.flange_to_tcp,
                p0[:3, 3],
                p1[:3, 3],
                p0[:3, :3] if rotations else None,
                self.tracking,
                0.003,
            )
            if tracking["status"] != "BOUNDED":
                return None
            collision = self.arm.collision.check_edge(
                a, b, self.obstacles, self.margin, 20, material, deadline=deadline
            )
            return (
                tracking["position_error_bound_m"]
                if collision["status"] == "CERTIFIED_MODELED_EDGE"
                else None
            )

        result = search_layers(targets, solve, check, seeds, beam_width=2, deadline=deadline)
        return result.get("positions"), result


def resample(points, joints, count=32):
    points, joints = np.asarray(points), np.asarray(joints)
    progress = np.r_[0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    keep = np.r_[True, np.diff(progress) > 1e-12]
    progress, points, joints = progress[keep], points[keep], joints[keep]
    grid = np.linspace(0, progress[-1], count)
    return (
        np.stack([np.interp(grid, progress, points[:, j]) for j in range(3)], axis=1),
        np.stack([np.interp(grid, progress, joints[:, j]) for j in range(7)], axis=1),
    )


def saved_drawings(run):
    """Read actual retained paths, including partial-build local witnesses."""
    run = Path(run)
    paths = sorted(
        [*(run / "indexed_witnesses").glob("*.json"), *(run / "forward_witnesses").glob("*.json")]
    )
    construction = read_json(run / "construction_plan.json")
    artistic = {row["id"] for row in construction["actions"] if row.get("stage") == "artistic"}
    paths.sort(key=lambda path: read_json(path).get("section") not in artistic)
    for path in paths:
        saved = read_json(path)
        row = saved.get("drawing", {})
        if "positions" not in row or "tcp_targets_m" not in row:
            continue
        points, joints = np.asarray(row["tcp_targets_m"]), np.asarray(row["positions"])
        if len(points) < 2 or len(points) != len(joints):
            continue
        yield path, saved, points, joints


def reachable_prefix_scene(teacher, run, section, index):
    """Reconstruct earlier actual deposition from a verified full reference plan.

    The returned geometry is transformed to the stationary requested table pose.
    A missing/unfinished replay is explicitly not claimed reachable.
    """
    run = Path(run)
    path = run / "offline_dual_arm_plan.json"
    replay = run / "offline_dual_arm_validation.json"
    if not path.exists() or not replay.exists():
        return None
    validation = read_json(replay)
    if validation.get("status") != "CERTIFIED_GEOMETRIC_REPLAY":
        return None
    if validation.get("source_plan_sha256") != sha256(path):
        return None
    config = run / "cell_config.json"
    if not config.exists() or read_json(config) != teacher.profile:
        return None
    plan = read_json(path)
    for name, expected in plan.get("inputs", {}).items():
        source = run / name
        if not source.is_file() or sha256(source) != expected:
            return None
    scene = MaterialScene()
    found = False
    for action in plan.get("actions", []):
        if action.get("stroke_id") == section:
            found = True
            break
        source_index = action.get("table_index", 0)
        transform = teacher.table.build_transform(index) @ np.linalg.inv(
            teacher.table.build_transform(source_index)
        )
        for edge_number, edge in enumerate(ClockPath.from_record(action["trajectory"]).edges):
            if edge.kind != "DRAW":
                continue
            piece = edge.piece()
            for part, (a, b, inflation) in enumerate(
                bounded_segments(piece, teacher.arm.robot, teacher.arm.flange_to_tcp, 0.00002)
            ):
                ends = [
                    transform[:3, :3]
                    @ teacher.arm.robot.forward(piece.position(t), teacher.arm.flange_to_tcp)[:3, 3]
                    + transform[:3, 3]
                    for t in (a, b)
                ]
                scene.add(
                    *ends,
                    edge.radius_m + inflation + 1e-9,
                    f"prefix-{action['stroke_id']}-{edge_number}-{part}",
                )
    if not found:
        return None
    return {
        "capsules": scene.segments,
        "kind": "reachable_verified_reference_prefix",
        "reachable_build_prefix": True,
        "source_plan_sha256": sha256(path),
        "source_replay_sha256": sha256(replay),
    }


def validate_fitted_episode(episode, q_path, source_points):
    """Curator callback: certify the supplied fit, without repairing its target."""
    teacher = _episode_teacher(episode)
    request = dict(episode["request"])
    request.setdefault("source_reference_points_m", request["source_points_m"])
    request["source_points_m"] = np.asarray(source_points).tolist()
    verdict = teacher.validate(
        request,
        q_path,
        episode["scene"],
        deadline=monotonic() + 30,
        radius_m=episode.get("provenance", {}).get("material_radius_m", 0.0005),
    )
    return verdict["status"] == "VERIFIED_VALID"


def validate_generated_episode(episode, q_path, source_points):
    """Evaluation callback matching online generation: refine, then certify."""
    teacher = _episode_teacher(episode)
    request = dict(episode["request"])
    request.setdefault("source_reference_points_m", request["source_points_m"])
    request["source_points_m"] = np.asarray(source_points).tolist()
    deadline = monotonic() + 15
    refined = teacher.adapt(request, np.asarray(q_path), deadline=deadline)
    if refined is None:
        return "UNRESOLVED"
    if request.get("q_start") is not None:
        refined[0] = request["q_start"]
    return teacher.validate(
        request,
        refined,
        episode["scene"],
        deadline=deadline,
        radius_m=episode.get("provenance", {}).get("material_radius_m", 0.0005),
    )["status"]


_EPISODE_TEACHERS = {}


def _episode_teacher(episode):
    profile = episode["provenance"]["profile"]
    if profile not in _EPISODE_TEACHERS:
        _EPISODE_TEACHERS[profile] = DrawingTeacher(profile)
    teacher = _EPISODE_TEACHERS[profile]
    if episode["validation"].get("context_hash") != teacher.context:
        raise ValueError("Episode model/static context changed")
    return teacher


def obstacle_choice_examples(teacher, request, joints, library, *, deadline):
    """Find witnessed same-stroke alternatives in a nonempty arm-obstacle scene.

    Starting posture is deliberately free for this query. This does not certify
    a transit from the preceding build state; the combined planner owns that.
    Only scenes whose obstructing motion and clearing motion both have direct
    evidence are returned as positive/negative pairs.
    """
    from .planning.motion_library import DrawingRequest

    request = {**request, "q_start": None, "q_goal": None}
    joints = np.asarray(joints)
    empty = {"capsules": [], "kind": "synthetic_empty", "reachable_build_prefix": False}
    results = []
    alternatives = []
    for joint in (2, 0, 4):
        for amount in (0.6, -0.6, 1.2, -1.2):
            if monotonic() >= deadline:
                return results
            seeds = joints.copy()
            seeds[:, joint] = np.clip(
                seeds[:, joint] + amount,
                teacher.arm.robot.lower[joint] + 1e-5,
                teacher.arm.robot.upper[joint] - 1e-5,
            )
            alternative = teacher.adapt(request, seeds, deadline=deadline)
            if (
                alternative is None
                or np.max(np.linalg.norm(np.asarray(alternative) - joints, axis=1)) < 0.08
            ):
                continue
            verdict = teacher.validate(request, alternative, empty, deadline=deadline)
            if verdict["status"] != "VERIFIED_VALID":
                continue
            alternatives.append(alternative)
            # Every accepted alternate empty-scene posture is useful multimodal data.
            results.append((request, empty, alternative, verdict, "alternate_empty"))
            midpoint = joints[len(joints) // 2]
            frames = teacher.arm.robot.frames(midpoint)
            for link in ("link4", "link3", "link5", "link2"):
                center = frames[link][:3, 3]
                for offset in ([0, 0, 0], [0.02, 0, 0], [-0.02, 0, 0], [0, 0.02, 0], [0, 0, 0.02]):
                    center_try = center + offset
                    scene = {
                        "capsules": [
                            {
                                "start": center_try.tolist(),
                                "end": center_try.tolist(),
                                "radius_m": 0.003,
                                "name": "synthetic-arm-obstacle",
                            }
                        ],
                        "kind": "synthetic_arm_posture_choice",
                        "reachable_build_prefix": False,
                    }
                    collision = teacher.validate(request, joints, scene, deadline=deadline)
                    if collision["status"] != "COLLISION":
                        continue
                    clear = teacher.validate(request, alternative, scene, deadline=deadline)
                    if clear["status"] != "VERIFIED_VALID":
                        continue
                    # The scene is admitted only with a witnessed colliding original
                    # and continuously verified clearing alternative.
                    results.append(
                        (request, scene, joints.tolist(), collision, "obstructed_cached_motion")
                    )
                    # Retrieve against this scene first, then use scene-aware teacher
                    # if existing library seeds have not retained the alternate yet.
                    query = DrawingRequest(
                        **{
                            k: v
                            for k, v in request.items()
                            if k in DrawingRequest.__dataclass_fields__
                        }
                    )
                    solved = None
                    for record in library.retrieve(query, limit=3):
                        if monotonic() >= deadline:
                            break
                        proposal = library.adapt(
                            record, query, teacher.arm, teacher.obstacles, deadline=deadline
                        )
                        if proposal is None:
                            continue
                        _, candidate = resample(
                            proposal["tcp_targets_m"], proposal["positions"], len(joints)
                        )
                        check = teacher.validate(request, candidate, scene, deadline=deadline)
                        if check["status"] == "VERIFIED_VALID":
                            solved = (candidate.tolist(), check, "scene_retrieval")
                            break
                    if solved is None and monotonic() < deadline:
                        candidate, _ = teacher.plan(
                            request, [alternative[0]], scene=scene, deadline=deadline
                        )
                        if candidate is not None:
                            check = teacher.validate(request, candidate, scene, deadline=deadline)
                            if check["status"] == "VERIFIED_VALID":
                                solved = (candidate, check, "scene_teacher")
                    if solved is None:
                        solved = (alternative, clear, "scene_local_repair")
                    results.append((request, scene, *solved))
                    for radius, delta in ((0.0015, [0.001, 0, 0]), (0.004, [0, 0.001, 0])):
                        point = center_try + delta
                        varied = {
                            **scene,
                            "capsules": [
                                {
                                    "start": point.tolist(),
                                    "end": point.tolist(),
                                    "radius_m": radius,
                                    "name": "synthetic-arm-obstacle",
                                }
                            ],
                        }
                        collision_variant = teacher.validate(
                            request, joints, varied, deadline=deadline
                        )
                        clear_variant = teacher.validate(
                            request, solved[0], varied, deadline=deadline
                        )
                        if (
                            collision_variant["status"] == "COLLISION"
                            and clear_variant["status"] == "VERIFIED_VALID"
                        ):
                            results.append(
                                (
                                    request,
                                    varied,
                                    joints.tolist(),
                                    collision_variant,
                                    "obstructed_cached_motion",
                                )
                            )
                            results.append(
                                (
                                    request,
                                    varied,
                                    solved[0],
                                    clear_variant,
                                    "scene_reused_alternative",
                                )
                            )
                    return results
    return results


def freeze_corpus(sources, output):
    """Merge completed generation batches with immutable split/provenance receipts."""
    from collections import Counter

    output = Path(output)
    if (output / "attempts.jsonl").exists():
        raise FileExistsError("Choose a new output directory for a frozen corpus")
    records = {}
    source_receipts = []
    superseded = 0
    metadata = {}
    for source in map(Path, sources):
        receipt_path = source.parent / "generation_receipt.json"
        receipt = read_json(receipt_path)
        if receipt.get("status") != "COMPLETE":
            raise ValueError(f"Generation batch is incomplete: {source}")
        source_receipts.append(
            {
                "path": str(source.resolve()),
                "sha256": sha256(source),
                "receipt_sha256": sha256(receipt_path),
            }
        )
        for line in source.read_text().splitlines():
            row = json.loads(line)
            run = Path(row["provenance"]["source_run"])
            if run not in metadata:
                graph = read_json(run / "stroke_graph.json")
                actions = read_json(run / "construction_plan.json")["actions"]
                metadata[run] = (
                    graph.get("source_method", graph.get("method", "unknown")),
                    {a["id"]: a.get("stage", "unknown") for a in actions},
                    sha256(run / "stroke_graph.json"),
                )
            method, stages, graph_hash = metadata[run]
            section = row["request"]["family_id"].split(":")[-1]
            stage = stages.get(section, "unknown")
            row["provenance"].update(
                source_method=method, source_stage=stage, source_graph_sha256=graph_hash
            )
            row["provenance"]["geometry_class"] = (
                "WIR3D"
                if stage == "artistic" and "wir3d" in method
                else "WirePrint"
                if "wireprint" in method
                else "Reference"
            )
            if row["validation"]["status"] == "UNRESOLVED" and not row["validation"].get(
                "context_hash"
            ):
                row["validation"]["context_hash"] = row["request"]["context_hash"]
            superseded += row["id"] in records
            records[row["id"]] = row
    rows = list(records.values())
    for field in ("source_object", "family_id"):
        groups = {}
        for row in rows:
            groups.setdefault(row["request"][field], set()).add(row["split"])
        if any(len(splits) != 1 for splits in groups.values()):
            raise ValueError(f"Split leakage across {field}")
    contexts = {r["validation"]["context_hash"] for r in rows}
    if len(contexts) != 1:
        raise ValueError("Corpus mixes model/static contexts")
    valid = [r for r in rows if r["validation"]["status"] == "VERIFIED_VALID"]
    if {r["split"] for r in valid} != {"train", "validation", "test"}:
        raise ValueError("Every frozen split needs a verified positive")
    output.mkdir(parents=True, exist_ok=True)
    path = output / "attempts.jsonl"
    path.write_text("".join(json.dumps(r, separators=(",", ":")) + "\n" for r in rows))
    paired = {}
    for row in rows:
        if row["scene"].get("kind") != "synthetic_arm_posture_choice":
            continue
        key = digest([row["request"], row["scene"]])
        paired.setdefault(key, set()).add(row["validation"]["status"])
    report = {
        "schema_version": 1,
        "status": "FROZEN",
        "records": len(rows),
        "raw_sha256": sha256(path),
        "context_hash": next(iter(contexts)),
        "counts": dict(Counter(r["validation"]["status"] for r in rows)),
        "valid_by_split": dict(Counter(r["split"] for r in valid)),
        "valid_by_geometry_class": dict(Counter(r["provenance"]["geometry_class"] for r in valid)),
        "valid_by_scene": dict(Counter(r["scene"].get("kind", "unknown") for r in valid)),
        "valid_by_index": dict(Counter(str(r["request"]["table_index"]) for r in valid)),
        "valid_by_variation": dict(Counter(r["provenance"]["variation"] for r in valid)),
        "paired_obstructed_and_clear_scenes": sum(
            {"COLLISION", "VERIFIED_VALID"} <= v for v in paired.values()
        ),
        "split_objects": {
            split: sorted({r["request"]["source_object"] for r in rows if r["split"] == split})
            for split in ("train", "validation", "test")
        },
        "sources": source_receipts,
        "superseded_same_request_scene_ids": superseded,
        "freeze_rule": (
            "later supplied batch supersedes matching episode ids; no model evaluation used"
        ),
        "scope": (
            "Modeled local DRAW geometry; physical fabrication and "
            "complete build acceptance separate"
        ),
    }
    from .io import write_json

    write_json(output / "corpus_receipt.json", report)
    return report


def generate_from_attempts(source, output, profile, *, count=32, maximum=32, seconds=20):
    """Offline teacher feedback: runtime misses -> fresh scene-checked examples.

    MotionLibrary attempts are proposals/observations. No stored clock certificate
    certifies a resampled seed here. Every returned positive is freshly checked.
    """
    from .io import write_json
    from .learning.data import read_episodes
    from .planning.motion_library import DrawingRequest, MotionLibrary

    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    target = output / "attempts.jsonl"
    if target.exists():
        raise FileExistsError("Use a new output directory for feedback generation")
    observations = list(read_episodes([source]))
    library = MotionLibrary(output / "motion_library")
    source = Path(source)
    if source.is_dir():
        original = MotionLibrary(source.parent if source.name == "attempts" else source)
        library.records.extend(original.records)
    teachers = {}
    records = []
    started = monotonic()

    def emit(parent, request, scene, candidate, verdict, method):
        provenance = {
            **parent.get("provenance", {}),
            "teacher_feedback": True,
            "source_attempt_id": digest(parent),
            "method": method,
            "profile": str(active_profile.resolve()),
        }
        row = {
            "schema_version": 1,
            "id": digest([parent, method, len(records)]),
            "request": request,
            "scene": scene,
            "trajectory": None
            if candidate is None
            else {
                "q_path": np.asarray(candidate).tolist(),
                "source_points_m": request["source_points_m"],
                "representation": "piecewise_linear_joint_v1",
            },
            "validation": verdict,
            "provenance": provenance,
        }
        if parent.get("split"):
            row["split"] = parent["split"]
        if verdict["status"] == "VERIFIED_VALID":
            query = DrawingRequest(
                **{k: v for k, v in request.items() if k in DrawingRequest.__dataclass_fields__}
            )
            rotation = request.get("fixed_tcp_rotation")
            if rotation is None and request.get("allowed_orientations"):
                rotation = request["allowed_orientations"][0]
            if rotation is None:
                row["cache_admission"] = "REJECTED_MISSING_FIXED_ORIENTATION"
            else:
                try:
                    library.remember(
                        query,
                        {
                            "positions": np.asarray(candidate).tolist(),
                            "tcp_targets_m": request["source_points_m"],
                            "tcp_rotation": rotation,
                            "free_tool_orientation": False,
                        },
                        verdict,
                        scene,
                        provenance,
                        arm=teacher.arm,
                    )
                    row["cache_admission"] = "FIXED_ORIENTATION_VERIFIED"
                except (ValueError, TimeoutError) as error:
                    row["cache_admission"] = "REJECTED_FIXED_ORIENTATION"
                    row["cache_admission_reason"] = str(error)
        records.append(row)
        with target.open("a") as stream:
            stream.write(json.dumps(row, separators=(",", ":")) + "\n")
        return row

    processed = 0
    for observation in observations:
        if observation.get("validation", {}).get("status") == "VERIFIED_VALID":
            continue
        if processed >= maximum:
            break
        processed += 1
        active_profile = Path(observation.get("provenance", {}).get("profile", profile))
        if active_profile not in teachers:
            teachers[active_profile] = DrawingTeacher(active_profile)
        teacher = teachers[active_profile]
        request = dict(observation["request"])
        from .planning.motion_runtime import object_identity

        source_run = observation.get("provenance", {}).get(
            "source_run", request.get("source_object", "unknown")
        )
        metadata_path = Path(source_run) / "run.json"
        metadata = read_json(metadata_path) if metadata_path.exists() else {}
        request["source_object"] = object_identity(source_run, metadata)
        original_points = request.get("source_reference_points_m", request["source_points_m"])
        request["source_reference_points_m"] = original_points
        points, _ = resample(original_points, np.zeros((len(original_points), 7)), count)
        request["source_points_m"] = points.tolist()
        request["context_hash"] = teacher.context
        scene = observation.get("scene", {"capsules": []})
        radius = float(observation.get("provenance", {}).get("material_radius_m", 0.0005))
        query = DrawingRequest(
            **{k: v for k, v in request.items() if k in DrawingRequest.__dataclass_fields__}
        )
        deadline = monotonic() + seconds
        accepted = False
        seeds = [teacher.arm.initial_positions]
        if request.get("q_start") is not None:
            seeds.insert(0, request["q_start"])
        for cached in library.retrieve(query, limit=4):
            seeds.append(cached["trajectory"]["q_path"][0])
            proposal = library.adapt(
                cached, query, teacher.arm, teacher.obstacles, deadline=deadline
            )
            if proposal is None:
                continue
            _, seed = resample(proposal["tcp_targets_m"], proposal["positions"], count)
            candidate = teacher.adapt(request, seed, deadline=deadline)
            if candidate is None:
                continue
            verdict = teacher.validate(
                request, candidate, scene, deadline=deadline, radius_m=radius
            )
            emit(observation, request, scene, candidate, verdict, "feedback_cache_adaptation")
            if verdict["status"] == "VERIFIED_VALID":
                accepted = True
                break
        if not accepted:
            candidate, search = teacher.plan(request, seeds, scene=scene, deadline=deadline)
            verdict = (
                teacher.validate(request, candidate, scene, deadline=deadline, radius_m=radius)
                if candidate is not None
                else {
                    "status": "UNRESOLVED",
                    "context_hash": teacher.context,
                    "validator": VALIDATOR,
                    "reason": "teacher_no_path_found",
                    "search": search,
                }
            )
            emit(observation, request, scene, candidate, verdict, "feedback_scene_teacher")
        write_json(
            Path("/tmp/3d_art_dataset_progress.json"),
            {
                "status": "RUNNING_FEEDBACK",
                "processed": processed,
                "records": len(records),
                "output": str(output.resolve()),
            },
        )
    from collections import Counter

    report = {
        "schema_version": 1,
        "status": "COMPLETE",
        "processed_requests": processed,
        "records": len(records),
        "counts": dict(Counter(r["validation"]["status"] for r in records)),
        "seconds": monotonic() - started,
        "source": str(source.resolve()),
        "scope": "Fresh local DRAW certification from runtime misses; no full-build acceptance",
    }
    write_json(output / "generation_receipt.json", report)
    return report
