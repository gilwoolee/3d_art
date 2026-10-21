"""Plan one round: untimed paths, then their timing through ``build_round``.

``pen-art plan-geometry`` and ``plan-timing`` call this stage function.
"""

from dataclasses import replace
from pathlib import Path
from time import monotonic

import numpy as np

from ..cell import load_frame_config, prepare_cell
from ..curves import source_points
from ..io import artifact, asset_path, read_json, sha256, write_json
from ..structure import analyze_structure
from .cell_planner import CellPlanner
from .clock_path import ClockPath, timed_path
from .scene import MaterialScene
from .schedule import ScheduleState


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
        settings = dict(load_frame_config()["planning"])
        settings.update(
            collision_scope="pen_candidate",
            merge_deposits_m=20e-6,
            path_pieces="members",
            pen_check=PEN_SPHERES_REVISION,
            pen_sample_m=0.010,
            chain_strokes=True,
        )
        timing = dict(load_frame_config()["timing"])
        timing.update(
            retiming_grid_ramps=True, retiming_grid_samples=0, retiming_grid_midpoints=False
        )
        self.planner = CellPlanner(
            run, profile, settings, monotonic() + maximum_seconds, timing_override=timing
        )
        self.planner.geometry_first = True
        self.arm = self.planner.arm
        self.initial = self.arm.initial_positions.copy()
        self.actions = self.planner.actions
        self.material = read_json(asset_path(self.run, "fabrication_config.json"))["structure"]
        self.transform = np.asarray(self.planner.profile["world_from_build"])
        if not np.allclose(self.transform[:3, :3], np.eye(3)):
            raise ValueError("Structural gravity assumes an upright build frame")
        self.cooling = settings["assumed_cooling_s"]
        self.structure = {}
        self.best = []
        self.search_metadata = {}
        self.checkpoint_binding = {
            "inputs": {
                name: sha256(asset_path(self.run, name))
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

    def scene(self, ids):
        scene = MaterialScene()
        for key in sorted(ids):
            row = self.actions[key]
            points = source_points(row, self.transform)
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
            tuple(q),
            frozenset(ids),
            tuple((node, 0.0) for node in sorted(available)),
            payload=self.scene(ids) if scene is None else scene,
        )

    def check_forward_timing(self, row, clock):
        """Qualify timing independently before persisting a complete state.

        The scanned clock already holds the tip under its speed limit over
        every interval, by the tangent bound it is rescaled against, so the
        drawing tip speed is not bounded a second time (decided 2026-10-13).
        """
        from .clock_path import check_clock_limits
        from .process_clock import audit_process_clock

        audit_process_clock(
            clock, row, self.planner.process, self.cooling, self.transform, require_release=True
        )
        check_clock_limits(
            self.planner, self.arm, clock, clock.record(), drawing_tip_speed=False
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

    def run_build(self, workers, stage):
        if not self.stable(self.actions):
            self.report(
                "GOAL_STRUCTURE_REJECTED", structure=self.structure[frozenset(self.actions)]
            )
            return
        from .parallel_build import run_parallel_forward

        run_parallel_forward(self, workers, stage=stage)


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


def build_round(run, source, profile, workers, stage, maximum_seconds=7200):
    """Prepare one round, run its geometry or timing stage and return its report."""
    run = Path(run)
    if not (run / "run.json").exists():
        prepare_cell(profile, run, source)
    build = SectionBuild(run, profile, maximum_seconds)
    try:
        build.run_build(workers, stage)
    except Exception as error:
        build.report("ERROR", error={"type": type(error).__name__, "message": str(error)})
        raise
    return read_json(run / "section_build.json")
