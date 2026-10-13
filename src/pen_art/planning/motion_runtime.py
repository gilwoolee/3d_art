"""Shared cached/learned drawing proposals for the indexed construction planner.

Every output still enters normal approach, timing, progressive-material and
replay validation. The persistent library stores the accepted clock as evidence;
its sampled joint arrays remain seeds when the clock has changed their geometry.
"""

from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from time import monotonic, perf_counter

import numpy as np
import yaml

from ..curves import flatten_curve, transform_points
from ..io import read_json, sha256, write_json
from ..strokes import orient_stroke
from .cell_motion import (
    ORIENTATION_POLICY_VERSION,
    anchored_strokes,
    pen_rotation,
    radial_pen_rotations,
)
from .motion_library import (
    DRAWING_POLICY_VERSION,
    GROWING_PREFIX_SCOPE,
    OFFLINE_GEOMETRY_SCOPES,
    CachedCollisionModel,
    DrawingRequest,
    MotionLibrary,
    collision_context_hash,
    compatible_geometry,
    digest,
)
from .tool import sampled_pen_tilt


@dataclass
class ProposalBudget:
    """Shared active proposal effort across every root and index of one action."""

    remaining_seconds: float
    remaining_attempts: int = 8

    @property
    def available(self):
        return self.remaining_seconds > 1e-6 and self.remaining_attempts > 0

    def spend(self, seconds, *, attempt=False):
        self.remaining_seconds = max(0.0, self.remaining_seconds - seconds)
        if attempt:
            self.remaining_attempts -= 1


def model_context(arm, profile, *, timing=None, process=None):
    model = getattr(arm.collision, "model", arm.collision)
    return digest(
        {
            "model": collision_context_hash(model, arm.tool.metadata),
            "profile": profile,
            "drawing_policy": DRAWING_POLICY_VERSION,
            "orientation_policy_version": ORIENTATION_POLICY_VERSION,
            "timing": timing,
            "process": process,
        }
    )


def learning_context(arm, profile):
    """Learned joint seeds bind physical geometry; runtime separately binds timing."""
    model = getattr(arm.collision, "model", arm.collision)
    return digest({
        "model": collision_context_hash(model, arm.tool.metadata),
        "profile": profile,
    })


def load_context_for_run(source_build, profile_path):
    """Read the context a new SectionBuild will use, without creating a planner.

    Full-build export settings do not exist for paused/precomputed-only runs.
    SectionBuild always reads the current continuous timing profile and takes
    process assumptions from the bound source run; mirror that exact ownership.
    """
    from ..cell import load_cell

    source_build, profile_path = Path(source_build), Path(profile_path)
    profile, arms, obstacles = load_cell(profile_path)
    if len(arms) != 1:
        raise ValueError("Whole-action motion cache requires a single-arm profile")
    arm = next(iter(arms.values()))
    timing_path = Path("configs/timing_continuous_bounded.yaml")
    timing = yaml.safe_load(timing_path.read_text())
    timing.pop("target_seconds", None)
    metadata = read_json(source_build / "run.json")
    process_source = Path(metadata.get("cell_source_run", source_build))
    process_path = process_source / "fabrication_config.json"
    process = read_json(process_path)["process"]
    return {
        "profile": profile,
        "arm": arm,
        "obstacles": obstacles,
        "timing": timing,
        "process": process,
        "context_hash": model_context(arm, profile, timing=timing, process=process),
        "inputs": {str(path): sha256(path) for path in (profile_path, timing_path, process_path)},
    }


def object_identity(source_run, metadata):
    """Keep known fixture aliases and repeated imports of one mesh together."""
    from pathlib import Path

    prefix = Path(source_run).name.split("_")[0]
    if prefix in {"cube", "bottle", "torus", "prism", "palace"}:
        return prefix
    if metadata.get("source_sha256"):
        return "mesh:" + metadata["source_sha256"]
    return str(source_run)


def static_shapes(profile, table):
    environment = profile["environment"]
    shapes = {"table": {"type": "Box", "side": environment["table_size_m"]}}
    if table is not None:
        shapes.update(
            platform={
                "type": "Cylinder",
                "radius": table.plate_radius_m,
                "lz": table.plate_thickness_m,
            },
            turntable_housing={
                "type": "Cylinder",
                "radius": table.housing_radius_m,
                "lz": table.housing_height_m,
            },
        )
    else:
        height = (
            environment.get("platform_top_z_m", profile["world_from_build"][2][3])
            - environment["table_z_m"]
        )
        shapes["platform"] = {
            "type": "Box",
            "side": [*environment["platform_size_m"], max(height, 0.006)],
        }
    riser = environment.get("build_riser")
    if riser is not None:
        shapes["build_riser"] = (
            {"type": "Box", "side": [*riser["size_m"], riser["height_m"]]}
            if riser["type"] == "box"
            else {"type": "Cylinder", "radius": riser["radius_m"], "lz": riser["height_m"]}
        )
    return shapes


class DrawingExperience:
    def __init__(self, build, table, *, frozen_records=None):
        self.build, self.table = build, table
        self.library = MotionLibrary(
            build.motion_library_path, load_records=frozen_records is None, lazy=True
        )
        self.frozen = frozen_records is not None
        if self.frozen:
            self.library.records = list(frozen_records)
        self.context_hash = model_context(
            build.arm,
            build.planner.profile,
            timing=build.planner.timing,
            process=build.planner.process,
        )
        self.learning_context_hash = learning_context(build.arm, build.planner.profile)
        self.stats = Counter()
        self.seconds = Counter()
        # The final path check is the authority when waypoint checks are chosen.
        self.library.adaptation_edge_checks = not getattr(
            build.planner, "adaptation_waypoint_checks", False
        )
        metadata = read_json(build.run / "run.json")
        self.source_run = str(metadata["cell_source_run"])
        self.source_object = object_identity(self.source_run, metadata)
        shapes = static_shapes(build.planner.profile, table)
        self.wrappers = []
        raw = getattr(build.planner, "verification_raw", False)
        if not raw:
            from .path_certificates import PathCertificateStore

            build.planner.path_certificates = PathCertificateStore(
                Path(build.motion_library_path) / "path_certificates"
            )
        for arm in build.planner.arms.values():
            if raw:
                continue
            wrapped = CachedCollisionModel(
                arm.collision,
                build.motion_library_path,
                geometry_context=arm.tool.metadata,
                static_geometry=shapes,
            )
            arm.collision = wrapped
            self.wrappers.append(wrapped)
            tip = CachedCollisionModel(
                build.planner.tip_models[arm.identity],
                build.motion_library_path,
                geometry_context={"tool": arm.tool.metadata, "virtual_tip": True},
                static_geometry=shapes,
            )
            build.planner.tip_models[arm.identity] = tip
            self.wrappers.append(tip)
        self.provider = None
        if build.diffusion_checkpoint is not None:
            from ..learning.diffusion import MotionDiffusionProvider

            self.provider = MotionDiffusionProvider(
                build.diffusion_checkpoint,
                self.learning_context_hash,
                signature_fn=self.signature,
                device=build.learning_device,
            )
        self.checkpoint_hash = (
            None if build.diffusion_checkpoint is None else sha256(build.diffusion_checkpoint)
        )
        # The coverage survey compares every action with every record. It only
        # informs the report, so it is left to the first report instead of
        # delaying the start of planning.
        print(f"Motion cache: {len(self.library.records)} records", flush=True)

    @staticmethod
    def scene_record(scene):
        return {
            "capsules": scene.segments,
            "table_index": scene.table_index,
            "kind": "construction_planner_material",
        }

    def signature(self, episode, probe_q):
        from ..learning.oracle import material_scene

        scene = material_scene(episode["scene"])
        planner = self.build.planner
        arm = self.build.arm
        if getattr(planner, "collision_scope", "full_robot") == "pen_candidate":
            arm = planner.collision_arm(arm.identity)
        return np.asarray(
            [
                bool(
                    arm.collision.check(
                        q, planner.obstacles, planner.margin, scene, deadline=planner.deadline
                    )
                )
                for q in probe_q
            ],
            dtype=float,
        )

    def request(self, points, index, row=None):
        parent = "" if row is None else row.get("parent_curve", "")
        path_kind = next(
            (kind for kind in ("zigzag", "contour") if parent.startswith(kind + "-")), "unknown"
        )
        return DrawingRequest(
            source_points_m=points,
            table_index=index,
            source_object=self.source_object,
            family_id=self.source_object,
            context_hash=self.context_hash,
            path_kind=path_kind,
        )

    def budget(self, row):
        # Resolution changes must not shorten the time available to validate the
        # same whole action. The planner still caps this shared allowance by its
        # remaining global deadline.
        # A complete cached candidate also needs source/process and progressive
        # material validation. Give it the same bounded effort as a cold
        # candidate; rejecting a prepared cache path at 90 s only to redo it
        # cold wastes the work without identifying a collision.
        return ProposalBudget(240.0)

    def preferred_indices(self, row, current):
        points, _ = flatten_curve(row, 0.00005, 0.003)
        request = self.request(points, current, row)
        cached = {
            record["request"]["table_index"]
            for record in self.library.records
            if compatible_geometry(request, record)
        }
        return sorted(range(4), key=lambda index: (index not in cached, index != current, index))

    def parent_key(self, record):
        key = getattr(getattr(self, "library", None), "record_key", None)
        return digest(record) if key is None else key(record)

    def retained_seed_proposal(self, record, request):
        """Keep exact-source dense seed knots; the runtime still checks the full path."""
        if record.get("validation", {}).get("scope") != GROWING_PREFIX_SCOPE:
            return None
        old = record["request"]
        if (
            not request.context_hash
            or old.get("context_hash") != request.context_hash
            or old["table_index"] != request.table_index
            or request.q_start is not None
            or request.q_goal is not None
            or request.allowed_orientations
        ):
            return None
        if request.fixed_tcp_rotation is not None and not np.array_equal(
            request.fixed_tcp_rotation, old.get("fixed_tcp_rotation")
        ):
            return None
        first, second = np.asarray(old["source_points_m"]), np.asarray(request.source_points_m)
        if first.shape != second.shape or not np.array_equal(first, second):
            return None
        try:
            MotionLibrary._validate_record(record)
        except (ValueError, TypeError, KeyError):
            return None
        trajectory = record["trajectory"]
        provenance = record.get("provenance", {})
        preference = dict(provenance.get("orientation_preference", {}))
        for name in ("pen_tilt_deg", "roll_rad", "orientation_policy"):
            if provenance.get(name) is not None:
                preference["policy" if name == "orientation_policy" else name] = provenance[name]
        return {
            "status": "SAMPLED_DRAWING_PATH",
            "arm": self.build.arm.identity,
            "variant": "cache-retained-seed",
            "positions": trajectory["q_path"],
            "tcp_targets_m": trajectory["source_points_m"],
            "tcp_rotation": trajectory.get("tcp_rotation"),
            "free_tool_orientation": trajectory.get("free_tool_orientation", False),
            "motion_library_parent": self.parent_key(record),
            "seed_orientation_preference": preference,
            "requires_current_material_process_validation": True,
        }

    def retained_clock_proposal(self, record, request):
        """Keep an exact accepted interpolant only for an identical bound request.

        This remains untrusted until the indexed planner replays current material,
        source tracking, joint/speed limits and process timing. Changed geometry
        or context always takes the ordinary adaptation/retiming path.
        """
        if record["validation"].get("scope") in OFFLINE_GEOMETRY_SCOPES:
            return None
        old = record["request"]
        clock = record["trajectory"].get("exact_clock")
        if (
            clock is None
            or old.get("context_hash") != request.context_hash
            or old["table_index"] != request.table_index
            or request.q_start is not None
            or request.q_goal is not None
            or request.allowed_orientations
        ):
            return None
        if request.fixed_tcp_rotation is not None and not np.array_equal(
            request.fixed_tcp_rotation, old.get("fixed_tcp_rotation")
        ):
            return None
        first, second = np.asarray(old["source_points_m"]), np.asarray(request.source_points_m)
        if first.shape != second.shape or not np.array_equal(first, second):
            return None
        if clock.get("process_clock_policy") != DRAWING_POLICY_VERSION:
            return None
        return {
            "status": "RETAINED_GEOMETRY_REQUIRES_REVALIDATION",
            "arm": self.build.arm.identity,
            "variant": "cache-retained-interpolant",
            "positions": record["trajectory"]["q_path"],
            "tcp_targets_m": record["trajectory"]["source_points_m"],
            "tcp_rotation": record["trajectory"].get("tcp_rotation"),
            "free_tool_orientation": record["trajectory"].get("free_tool_orientation", False),
            "retained_clock": clock,
            "motion_library_parent": self.parent_key(record),
        }

    def _decorate(self, proposal, oriented, request, source):
        profile = getattr(self.build.planner, "profile", {})
        if profile.get("drawing_orientation_policy", "fixed") == "outward_radial":
            preferred = profile.get("drawing_pen_tilt_deg", 70.0)
            seed_preference = proposal.get("seed_orientation_preference") or {}
            seed_tilt = seed_preference.get("pen_tilt_deg")
            seed_roll = seed_preference.get("roll_rad")
            tilt = proposal.get("pen_tilt_deg", preferred if seed_tilt is None else seed_tilt)
            roll = proposal.get("roll_rad", 0.0 if seed_roll is None else seed_roll)
            proposal.update(
                orientation_policy="outward_radial",
                pen_tilt_deg=tilt,
                roll_rad=roll,
                preferred_pen_tilt_deg=preferred,
                orientation_metadata_scope="preferred_targets_not_actual_joint_orientation",
            )
            if proposal.get("tcp_rotations") is None or len(proposal["tcp_rotations"]) != len(
                proposal["tcp_targets_m"]
            ):
                proposal["tcp_rotations"] = radial_pen_rotations(
                    self.build.arm,
                    proposal["tcp_targets_m"],
                    self.table.build_transform(request.table_index),
                    tilt,
                    roll,
                ).tolist()
        proposal.update(
            stroke_id=oriented["id"],
            start_anchor=oriented["start"],
            end_anchor=oriented["end"],
            reverse=oriented["reverse"],
            start_visit=oriented["start_visit"],
            release_cooling_s=self.build.cooling,
            table_index=request.table_index,
            proposal_source=source,
            motion_request=request.as_dict(),
            drawing_policy=DRAWING_POLICY_VERSION,
            collision_scope=getattr(self.build.planner, "collision_scope", "full_robot"),
        )
        if proposal["collision_scope"] == "pen_candidate":
            proposal["requires_full_robot_validation"] = True
            if "static_collision_status" in proposal:
                proposal["static_collision_status"] = "CHECKED_PEN_STATIC_ONLY"
        return proposal

    def candidates(
        self,
        row,
        available,
        index,
        scene,
        reference_q=None,
        reference_endpoint="start",
        budget=None,
        skip_parents=(),
    ):
        planner, arm = self.build.planner, self.build.arm
        if getattr(planner, "collision_scope", "full_robot") == "pen_candidate":
            arm = planner.collision_arm(arm.identity)
        refresh = getattr(self.library, "refresh", None)
        if not getattr(self, "frozen", False) and refresh is not None and refresh():
            self.stats["library_refreshes"] += 1
        budget = self.budget(row) if budget is None else budget
        for oriented in anchored_strokes(row, available):
            if not budget.available:
                self.stats["proposal_budget_exhausted"] += 1
                return
            # Bound proposal effort independently of the teacher's search budget.
            # A populated library must leave time for ordinary planning fallback.
            points, parameters = flatten_curve(oriented, 0.00005, 0.003)
            points = transform_points(points, self.table.build_transform(index))
            members = (
                points[np.isclose(parameters, np.round(parameters), atol=1e-12, rtol=0)]
                if planner.settings.get("path_pieces") == "members"
                else None
            )
            # The retained seed can be much denser than the new source. Share
            # the explicit action allowance with realization instead of
            # clipping adaptation by the source's sample count.
            request = self.request(points, index, row)
            started = perf_counter()
            records = self.library.retrieve(
                request,
                limit=4,
                reference_q=reference_q,
                reference_endpoint=reference_endpoint,
                compatible_only=True,
            )
            retrieval_seconds = perf_counter() - started
            self.seconds["retrieval"] += retrieval_seconds
            budget.spend(retrieval_seconds)
            self.stats["requests"] += 1
            for position, record in enumerate(records):
                if not budget.available or monotonic() >= planner.deadline:
                    break
                if skip_parents and self.parent_key(record) in skip_parents:
                    continue
                # One stubborn seed must leave time for the remaining ones.
                share = budget.remaining_seconds / (len(records) - position)
                started = perf_counter()
                proposal = self.retained_seed_proposal(record, request)
                if proposal is not None:
                    self.stats["retained_seed_proposals"] += 1
                elif not getattr(planner, "geometry_first", False):
                    # A stored clock carries its timing segments. Geometry-first
                    # planning adapts the record so it is checked as a path.
                    proposal = self.retained_clock_proposal(record, request)
                    if proposal is not None:
                        self.stats["retained_interpolant_proposals"] += 1
                if proposal is None:
                    proposal = self.library.adapt(
                        record,
                        request,
                        arm,
                        planner.obstacles,
                        deadline=min(planner.deadline, monotonic() + share),
                        clearance_m=planner.margin,
                        members=members,
                    )
                    if proposal is not None:
                        self.stats["cache_adaptation_backtracks"] += proposal[
                            "adaptation_backtracks"
                        ]
                adaptation_seconds = perf_counter() - started
                self.seconds["adaptation"] += adaptation_seconds
                budget.spend(adaptation_seconds, attempt=True)
                if proposal is not None:
                    proposal["proposal_seconds"] = perf_counter() - started
                    proposal["motion_library_parent"] = self.parent_key(record)
                    proposal["cache_validation_scope"] = record["validation"].get(
                        "scope", "legacy_geometry"
                    )
                    proposal["requires_current_material_process_validation"] = True
                    self.stats["cache_proposals"] += 1
                    yield self._decorate(proposal, oriented, request, "cache")
                else:
                    self.stats["cache_adaptation_unresolved"] += 1
                    adaptation = getattr(self.library, "last_adaptation_failure", None) or {}
                    reason = adaptation.get("reason", "unknown")
                    self.stats[f"cache_adaptation_failed_{reason}"] += 1
                    self.library.record_attempt(
                        request,
                        None,
                        {
                            "status": "UNRESOLVED",
                            "context_hash": self.context_hash,
                            "reason": "cache_adaptation",
                            "adaptation_failure": adaptation,
                            "seconds": perf_counter() - started,
                        },
                        self.scene_record(scene),
                        {"parent_record": self.parent_key(record)},
                    )
            if self.provider is None or not budget.available:
                continue
            refinement_seconds = max(3.0, min(30.0, 0.05 * len(points)))
            proposal_deadline = min(planner.deadline, monotonic() + budget.remaining_seconds)
            try:
                started = perf_counter()
                # The seed model is shared with the dataset teacher. The stricter
                # timing/process context remains on every runtime motion request.
                learned_request = {
                    **request.as_dict(), "context_hash": self.learning_context_hash,
                }
                proposals = self.provider.propose(
                    learned_request, self.scene_record(scene), count=4
                )
                generated_seconds = perf_counter() - started
                self.seconds["diffusion"] += generated_seconds
                budget.spend(generated_seconds)
                self.stats["generated_candidates"] += len(proposals)
                for candidate in proposals:
                    if monotonic() >= proposal_deadline or not budget.available:
                        break
                    started = perf_counter()
                    # Neural joints are untrusted seeds. Project to one declared
                    # orientation before any continuous-path certificate is earned.
                    preferred = pen_rotation(
                        arm,
                        self.table.build_transform(index),
                        tilt_deg=planner.profile.get("drawing_pen_tilt_deg", 0),
                    )
                    seed = {
                        "positions": candidate.q_path.tolist(),
                        "tcp_targets_m": candidate.source_points_m.tolist(),
                        "tcp_rotation": preferred.tolist(),
                        "free_tool_orientation": False,
                        "seed_requires_fixed_orientation_projection": True,
                    }
                    fixed_request = DrawingRequest(
                        **{**request.as_dict(), "allowed_orientations": [preferred.tolist()]}
                    )
                    proposal = None
                    for refinement_request in (fixed_request, request):
                        if monotonic() >= proposal_deadline:
                            break
                        proposal = self.library.adapt_proposal(
                            seed,
                            refinement_request,
                            arm,
                            planner.obstacles,
                            deadline=min(proposal_deadline, monotonic() + refinement_seconds),
                            clearance_m=planner.margin,
                        )
                        if proposal is not None:
                            break
                    refinement_elapsed = perf_counter() - started
                    self.seconds["diffusion_refinement"] += refinement_elapsed
                    budget.spend(refinement_elapsed, attempt=True)
                    if proposal is not None:
                        proposal["proposal_seconds"] = perf_counter() - started
                        proposal["diffusion_metadata"] = candidate.metadata
                        proposal["learning_context_hash"] = self.learning_context_hash
                        self.stats["diffusion_proposals"] += 1
                        yield self._decorate(proposal, oriented, request, "diffusion")
                    else:
                        self.stats["diffusion_refinement_unresolved"] += 1
                        self.library.record_attempt(
                            request,
                            seed,
                            {
                                "status": "UNRESOLVED",
                                "context_hash": self.context_hash,
                                "reason": "diffusion_refinement",
                                "seconds": perf_counter() - started,
                            },
                            self.scene_record(scene),
                            candidate.metadata,
                        )
            except TimeoutError:
                self.stats["diffusion_timeouts"] += 1
            except ValueError:
                self.stats["diffusion_invalid_proposals"] += 1

    def observe(self, drawing, scene, *, move=None, failure=None):
        if move is not None and getattr(
            getattr(self.build, "planner", None), "collision_scope", "full_robot"
        ) == "pen_candidate":
            # Retain the candidate witness in the construction run, but do not
            # promote its reduced collision scope into the qualified library.
            drawing["collision_scope"] = "pen_candidate"
            drawing["requires_full_robot_validation"] = True
            self.stats["pen_candidate_admissions_deferred"] += 1
            return
        source = drawing.get("proposal_source", "planner")
        points = drawing.get("tcp_targets_m")
        if points is None:
            oriented = orient_stroke(
                self.build.actions[drawing["stroke_id"]],
                reverse=drawing.get("reverse", False),
                start_visit=drawing.get("start_visit", 0),
            )
            points, _ = flatten_curve(oriented, 0.00005, 0.003)
            points = transform_points(points, self.table.build_transform(drawing["table_index"]))
        request = (
            DrawingRequest(**drawing["motion_request"])
            if "motion_request" in drawing
            else self.request(
                points, drawing["table_index"], self.build.actions[drawing["stroke_id"]]
            )
        )
        provenance = {
            "run": str(self.build.run),
            "section": drawing["stroke_id"],
            "source_run": self.source_run,
            "parent_curve": self.build.actions[drawing["stroke_id"]].get("parent_curve"),
            "proposal_source": source,
            "checkpoint_sha256": self.checkpoint_hash,
            "learning_context_hash": getattr(self, "learning_context_hash", None),
            "profile": str(self.build.profile.resolve()),
            "material_radius_m": self.build.actions[drawing["stroke_id"]]["diameter_m"] / 2,
            "proposal_seconds": drawing.get("proposal_seconds"),
            "drawing_policy": DRAWING_POLICY_VERSION,
            "preparation_seconds": drawing.get("preparation_seconds"),
            "motion_library_parent": drawing.get("motion_library_parent"),
            "cache_validation_scope": drawing.get("cache_validation_scope"),
            "orientation_preference": {
                "policy": drawing.get("orientation_policy"),
                "pen_tilt_deg": drawing.get("pen_tilt_deg"),
                "preferred_pen_tilt_deg": drawing.get("preferred_pen_tilt_deg"),
                "roll_rad": drawing.get("roll_rad"),
                "seed_preference": drawing.get("seed_orientation_preference"),
                "source_points_m": points,
                "target_rotations": drawing.get("tcp_rotations"),
                "constraint": "preferred_only",
            },
        }
        if move is not None:
            edges = [edge for edge in move.payload.edges if edge.kind == "DRAW"]
            if not edges:
                raise ValueError("An accepted drawing needs a deposited path")
            joints = [edges[0].q0, *[edge.q1 for edge in edges]]
            poses = [self.build.arm.robot.forward(q, self.build.arm.flange_to_tcp) for q in joints]
            provenance["actual_pen_tilt"] = sampled_pen_tilt(self.build.arm, joints)
            stored_drawing = {
                **drawing,
                "positions": np.asarray(joints).tolist(),
                "tcp_targets_m": [pose[:3, 3].tolist() for pose in poses],
                "tcp_rotation": drawing.get("tcp_rotation"),
                "free_tool_orientation": drawing.get("free_tool_orientation", False),
            }
            record_id = self.library.remember(
                request,
                stored_drawing,
                {
                    "status": "VERIFIED_VALID",
                    "context_hash": self.context_hash,
                    "scope": "indexed_local_motion_with_progressive_material_and_timing",
                    "clock_record": move.payload.record(),
                },
                self.scene_record(scene),
                provenance, arm=self.build.arm,
            )
            drawing["motion_library_record"] = record_id
            self.stats[f"accepted_{source}"] += 1
        else:
            # prepare_drawing can repair the candidate internally. Its failure
            # does not establish a collision for the original sampled q_path.
            self.library.record_attempt(
                request,
                drawing if "positions" in drawing else None,
                {
                    "status": "UNRESOLVED",
                    "context_hash": self.context_hash,
                    "planner_failure": failure,
                },
                self.scene_record(scene),
                provenance,
            )
            self.stats[f"unresolved_{source}"] += 1

    def sampling_failure(self, row, points, index, scene, detail):
        """Keep requests whose teacher never yielded a motion candidate."""
        self.library.record_attempt(
            self.request(points, index, row),
            None,
            {
                "status": "UNRESOLVED",
                "context_hash": self.context_hash,
                "reason": "teacher_sampling",
                "detail": detail,
            },
            self.scene_record(scene),
            {
                "run": str(self.build.run),
                "section": row["id"],
                "profile": str(self.build.profile.resolve()),
                "source_run": self.source_run,
                "material_radius_m": row["diameter_m"] / 2,
            },
        )
        self.stats["unresolved_teacher_sampling"] += 1

    def cache_coverage(self):
        """Inspect all actual action/direction/index requests without motion search."""
        previous = getattr(self, "_coverage_snapshot", None)
        signature = (len(self.library.records), getattr(self.library, "generation", 0))
        if previous is not None and previous[0] == signature:
            return previous[1]
        requests, groups = {}, {}
        for identity, row in self.build.actions.items():
            groups[identity] = []
            available = {visit["node"] for visit in row.get("junction_visits", [])}
            available.update([row["start"], row["end"]])
            for oriented in anchored_strokes(row, available):
                points, _ = flatten_curve(oriented, 0.00005, 0.003)
                for index in range(4):
                    key = (
                        f"{identity}/index-{index}/visit-{oriented['start_visit']}"
                        f"/reverse-{int(oriented['reverse'])}"
                    )
                    groups[identity].append(key)
                    requests[key] = self.request(
                        transform_points(points, self.table.build_transform(index)), index, row
                    )
        report = self.library.coverage(requests)
        by_id = {row["id"]: row for row in report["requests"]}
        actions = [
            {
                "id": identity,
                "path_kind": by_id[keys[0]]["path_kind"] if keys else "unknown",
                "requests_with_seeds": sum(by_id[key]["readiness"] != "MISSING" for key in keys),
                "requests": len(keys),
            }
            for identity, keys in groups.items()
        ]
        report["actions"] = actions
        report["actions_without_usable_seeds"] = [
            row["id"] for row in actions if not row["requests_with_seeds"]
        ]
        report["zigzags_without_usable_seeds"] = [
            row["id"]
            for row in actions
            if row["path_kind"] == "zigzag" and not row["requests_with_seeds"]
        ]
        self._coverage_snapshot = (signature, report)
        return report

    def report(self):
        return {
            "mode": "cache_and_diffusion" if self.provider is not None else "cache",
            "drawing_policy": DRAWING_POLICY_VERSION,
            "library": str(self.build.motion_library_path),
            "context_hash": self.context_hash,
            "learning_context_hash": getattr(self, "learning_context_hash", None),
            "checkpoint_sha256": self.checkpoint_hash,
            "counts": dict(self.stats),
            "seconds": dict(self.seconds),
            # A parallel build surveys coverage on a worker and reports it when ready.
            "cache_coverage": getattr(self, "_coverage_pending", None) or self.cache_coverage(),
            "clearance_hits": sum(wrapper.hits for wrapper in self.wrappers),
            "clearance_misses": sum(wrapper.misses for wrapper in self.wrappers),
            "self_static_seconds": sum(
                getattr(wrapper, "self_static_seconds", 0.0) for wrapper in self.wrappers
            ),
            "material_seconds": sum(
                getattr(wrapper, "material_seconds", 0.0) for wrapper in self.wrappers
            ),
            "verification": {
                "counts": dict(getattr(self.build.planner, "verification_stats", {})),
                "seconds": dict(getattr(self.build.planner, "verification_seconds", {})),
                "path_certificates": (
                    self.build.planner.path_certificates.report()
                    if hasattr(self.build.planner, "path_certificates") else None
                ),
            },
        }

    def freeze(self, certificates):
        """Stop publishing to the shared library; keep its proofs readable.

        Parallel workers and repeated benchmark runs must see one unchanged
        library. New path proofs go to ``certificates`` in the caller's run.
        """
        self.frozen = self.library.frozen = True
        store = getattr(self.build.planner, "path_certificates", None)
        if store is not None and store.enabled:
            from .path_certificates import PathCertificateStore

            self.build.planner.path_certificates = PathCertificateStore(
                certificates, fallback=store.directory
            )

    def flush(self):
        for wrapper in () if getattr(self, "frozen", False) else self.wrappers:
            wrapper.flush()
        write_json(self.build.run / "motion_reuse.json", self.report())
