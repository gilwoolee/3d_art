"""Resumable one-arm construction from the empty base through every source stroke.

Original strokes stay intact geometrically. Motion fragments and automatically
added support strokes are tracked separately from source-stroke completion.
"""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

from ..io import now, read_json, sha256, write_json
from .axis_repair import (
    AXIS_REPAIR_REVISION,
    AXIS_WORK_REVISION,
    WARM_SEED_PREFERRED_CHECK_REVISION,
    refine_axis_path,
    repair_collision_witness,
    repair_drawing_axes,
    solve_joint_pose,
)
from .cell_planner import CellPlanner
from .clock_path import ClockPath
from .dual_arm import ScheduleState
from .endpoint_support import plan_endpoint_support
from .scene import MaterialScene
from .stroke_restarts import attach_support_to_member, posture_break_candidates, release_screen


class SingleArmBuild:
    def __init__(self, run, profile):
        self.run, self.output = Path(run), Path(run) / "full_build"
        self.profile_path = Path(profile)
        cfg = read_json(self.run / "allocation_config.json")
        settings = yaml.safe_load(Path("configs/dual_arm.yaml").read_text())
        settings.update(maximum_subdivision_depth=20, assumed_cooling_s=cfg["cooling_s"])
        self.planner = CellPlanner(
            self.run, Path(profile), settings, float("inf"), timing_override=cfg["timing"]
        )
        # Full-scene edge checks can consume the old three-second RRT budget
        # after only a handful of nodes. Allow a bounded offline transit solve;
        # this changes search effort, never clearance or trajectory limits.
        self.planner.transit["rrt_maximum_seconds"] = max(
            30.0, self.planner.transit["rrt_maximum_seconds"]
        )
        if len(self.planner.arms) != 1:
            raise ValueError("Exactly one arm required")
        self.arm = next(iter(self.planner.arms.values()))
        self.initial = np.asarray(cfg["initial_positions"][self.arm.identity])
        self.arm.initial_positions = self.initial
        self.cooling = cfg["cooling_s"]
        self.material = read_json(self.run / "fabrication_config.json")["structure"]
        self.transform = np.asarray(self.planner.profile["world_from_build"])
        if not np.allclose(self.transform[:3, :3], np.eye(3)):
            raise ValueError("Gravity must be build Z")
        self.selection = read_json(self.run / "cell_allocation.json")["results"]["single"][
            "actions"
        ]
        checkpoint = self.output / "state.json"
        saved = read_json(checkpoint) if checkpoint.exists() else None
        names = [
            "cell_config.json",
            "cell_drawing_variants.json",
            "allocation_config.json",
            "construction_plan.json",
            "stroke_graph.json",
            "fabrication_config.json",
        ]
        optional = [
            "allocation_motions.json",
            "stroke_restarts/trial.json",
            "full_build/support_motion.json",
            "full_build/support_proposal.json",
        ]
        names.extend(name for name in optional if (self.run / name).exists())
        # Preserve existing checkpoint bindings exactly. New builds additionally
        # bind the selected order, which older Palace checkpoints did not include.
        if saved is None or "cell_allocation.json" in saved["inputs"]:
            names.append("cell_allocation.json")
        self.inputs = {name: sha256(self.run / name) for name in names}
        if saved is not None and saved["inputs"] != self.inputs:
            raise ValueError("Full-build inputs changed")
        trial_path = self.run / "stroke_restarts/trial.json"
        trial = read_json(trial_path) if trial_path.exists() else None
        support_path = self.output / "support_motion.json"
        proposal_path = self.output / "support_proposal.json"
        if support_path.exists() != proposal_path.exists():
            raise ValueError("Support bootstrap requires both motion and proposal artifacts")
        self.support = None
        self.supports = []
        self.graph, self.member_replacements, error = self.planner.graph, {}, 0.0
        if support_path.exists():
            support = read_json(support_path)
            proposal = read_json(proposal_path)
            if support["status"] != "PREPARED":
                raise ValueError("Base post motion is not prepared")
            # Legacy support proposals attach to their accompanying restart trial.
            # Standalone proposals must state their original source stroke.
            source = proposal.get("source_stroke") or (trial or {}).get("stroke_id")
            if source not in self.planner.actions:
                raise ValueError("Support proposal must identify an existing source stroke")
            self.support = support["action"]
            self.graph, self.member_replacements, error = attach_support_to_member(
                self.planner.graph,
                self.support,
                proposal["nodes"],
                self.planner.actions[source]["members"],
            )
            self.planner.actions[self.support["id"]] = self.support
            self.supports.append(self.support)
        self.attachment_error = error
        if saved is not None:
            self.state = saved
        else:
            original_path = self.run / "allocation_motions.json"
            original = read_json(original_path)["motions"] if original_path.exists() else []
            expected = [row["stroke_id"] for row in self.selection]
            completed = [m["stroke_id"] for m in original]
            if completed != expected[:len(completed)]:
                raise ValueError("Prepared motions must be a prefix of the selected source order")
            records = [
                {
                    "file": "allocation_motions.json",
                    "index": i,
                    "source_stroke": m["stroke_id"],
                    "kind": "source",
                    "complete": True,
                }
                for i, m in enumerate(original)
            ]
            progress = {}
            if trial and trial["segments"]:
                source = trial["stroke_id"]
                if len(completed) >= len(expected) or source != expected[len(completed)]:
                    raise ValueError("Restart trial must continue the next selected source stroke")
                current = 0
                for segment in trial["segments"]:
                    lo, hi = segment["start_index"], segment["end_index"]
                    if lo != current or hi <= lo:
                        raise ValueError("Restart trial must cover a contiguous source prefix")
                    records.append(
                        {
                            "file": f"stroke_restarts/chunk_{lo:03d}_{hi:03d}.json",
                            "source_stroke": source,
                            "kind": "fragment",
                            "start_index": lo,
                            "end_index": hi,
                            "complete": False,
                        }
                    )
                    current = hi
                progress[source] = current
            if self.support is not None:
                records.append(
                    {
                        "file": "full_build/support_motion.json",
                        "source_stroke": self.support["id"],
                        "kind": "support",
                        "wrapped": True,
                        "complete": True,
                    }
                )
            self.state = {
                "status": "PLANNING",
                "inputs": self.inputs,
                "requested_strokes": len(self.selection),
                "completed_strokes": completed,
                "source_progress": progress,
                "motions": records,
                "failures": [],
                "simulation_only": True,
                "physical_status": "unverified",
            }
        self.scene = MaterialScene()
        self.q = np.asarray(cfg["initial_positions"][self.arm.identity])
        for index, row in enumerate(self.state["motions"]):
            if row["kind"] == "support" and row.get("evidence_sha256"):
                if sha256(self.run / row["file"]) != row["evidence_sha256"]:
                    raise ValueError("Added support evidence changed")
                evidence = read_json(self.run / row["file"])
                prefix = self.state["motions"][:index]
                if evidence["prefix_records_sha256"] != self.records_key(prefix):
                    raise ValueError("Added support prefix changed")
                for name, digest in evidence["prefix_inputs"].items():
                    if sha256(self.run / name) != digest:
                        raise ValueError(f"Added support prefix trajectory changed: {name}")
                self.restore_support(evidence)
            elif row["kind"] == "support" and row["file"] != "full_build/support_motion.json":
                raise ValueError("Added support has no bound preparation evidence")
            path = self.read_motion(row)
            if not np.allclose(path.edges[0].q0, self.q, atol=1e-8, rtol=0):
                raise ValueError(f"Discontinuous full-build checkpoint: {row['file']}")
            self.add_material(path, f"{row['file']}:{row.get('index', '')}")
            self.q = np.asarray(path.edges[-1].q1)
        self.save_structure()
        self.save()

    def read_motion(self, row):
        raw = read_json(self.run / row["file"])
        if "index" in row:
            raw = raw["motions"][row["index"]]["trajectory"]
        elif row.get("wrapped"):
            raw = raw["trajectory"]
        return ClockPath.from_record(raw)

    def add_material(self, path, name):
        for i, edge in enumerate(path.edges):
            if edge.kind == "DRAW":
                self.planner.add_clock_edge(
                    self.scene, self.arm.identity, edge, f"build/{name}/{i}"
                )

    def save(self):
        self.state["updated_at"] = now()
        write_json(self.output / "state.json", self.state)

    @staticmethod
    def records_key(records):
        return hashlib.sha256(json.dumps(records, sort_keys=True).encode()).hexdigest()

    def save_structure(self):
        write_json(
            self.output / "structural_graph.json",
            {
                **self.graph,
                "member_replacements": self.member_replacements,
                "attachment_approximation_m": self.attachment_error,
            },
        )

    def resolved_members(self, members):
        existing = {edge["id"] for edge in self.graph["edges"]}
        result = set()

        def visit(member, ancestors):
            if member in existing:
                result.add(member)
                return
            if member in ancestors or member not in self.member_replacements:
                raise ValueError(f"Invalid structural member replacement: {member}")
            for child in self.member_replacements[member]:
                visit(child, ancestors | {member})

        for member in members:
            visit(member, set())
        return result

    def printed_members(self):
        result = {support["id"] for support in self.supports}
        for source in self.state["completed_strokes"]:
            result.update(self.resolved_members(self.planner.actions[source]["members"]))
        return result

    def restore_support(self, evidence):
        """Reconstruct a committed post from bound evidence, never a cached graph alone."""
        if evidence["status"] != "PREPARED":
            raise ValueError("Added support is not prepared")
        source, support = evidence["source_stroke"], evidence["action"]
        if support["id"] in self.planner.actions:
            raise ValueError("Duplicate added support")
        graph, replacements, error = attach_support_to_member(
            self.graph, support, evidence["nodes"],
            self.resolved_members(self.planner.actions[source]["members"]),
        )
        if graph != evidence["graph"] or replacements != evidence["member_replacements"]:
            raise ValueError("Added support structural reconstruction changed")
        self.graph = graph
        self.member_replacements.update(replacements)
        self.attachment_error = max(self.attachment_error, error)
        self.supports.append(support)
        self.planner.actions[support["id"]] = support

    def try_endpoint_support(self, source, lo, hi):
        """Try one post per accepted endpoint and commit only a complete prepared motion."""
        from .endpoint_support import SUPPORT_POLICY_REVISION

        identity = f"auto-support-{source}-{lo}"
        attempts = self.state.setdefault("support_attempts", {})
        prior = attempts.get(identity)
        retry_old_release_policy = (
            prior is not None
            and prior.get("policy_revision", 1) < SUPPORT_POLICY_REVISION
            and prior["status"] == "SUPPORT_DOES_NOT_RESOLVE_RELEASE"
        )
        if (
            lo <= 0
            or (prior is not None and not retry_old_release_policy)
            or any(s["id"] == identity for s in self.supports)
        ):
            return False
        print(f"Planning endpoint support for {source} at {lo}", flush=True)
        report = plan_endpoint_support(
            self, source, lo, blocked_index=hi,
            progress=lambda done, count, _: print(
                f"{identity}: {done}/{count} poses", flush=True
            ),
        )
        report["prefix_records_sha256"] = self.records_key(self.state["motions"])
        report["prefix_inputs"] = {
            row["file"]: sha256(self.run / row["file"]) for row in self.state["motions"]
        }
        file = f"full_build/support_{source}_{lo:04d}_v{SUPPORT_POLICY_REVISION}.json"
        if (self.run / file).exists() and read_json(self.run / file)["status"] == "PREPARED":
            raise ValueError("Refusing to overwrite accepted support evidence")
        write_json(self.run / file, report)
        digest = sha256(self.run / file)
        if prior is not None:
            self.state.setdefault("support_attempt_history", []).append(
                {"support_id": identity, **prior}
            )
        attempts[identity] = {
            "status": report["status"], "file": file, "sha256": digest,
            "policy_revision": SUPPORT_POLICY_REVISION,
        }
        if report["status"] != "PREPARED":
            self.state["support_diagnostic"] = attempts[identity]
            self.save()
            return False
        path = ClockPath.from_record(report["trajectory"])
        if not np.allclose(path.edges[0].q0, self.q, atol=1e-8, rtol=0):
            raise ValueError("Added support motion start changed")
        self.restore_support(report)
        self.state["motions"].append(
            {
                "file": file,
                "source_stroke": identity,
                "kind": "support",
                "wrapped": True,
                "complete": True,
                "evidence_sha256": digest,
            }
        )
        self.add_material(path, file + ":")
        self.q = np.asarray(path.edges[-1].q1)
        self.state.update(status="SUPPORT_ADDED", active_stroke=source, active_index=lo)
        self.save_structure()
        self.save()
        return True

    def accept(self, source, lo, hi, move, total):
        path = move.payload
        if not np.allclose(path.edges[0].q0, self.q, atol=1e-8, rtol=0):
            raise ValueError("Motion start changed")
        file = f"full_build/motion_{len(self.state['motions']):04d}.json"
        write_json(self.run / file, path.record())
        self.state["motions"].append(
            {
                "file": file,
                "source_stroke": source,
                "kind": "fragment",
                "start_index": lo,
                "end_index": hi,
                "complete": hi == total - 1,
            }
        )
        self.add_material(path, file + ":")
        self.q = np.asarray(path.edges[-1].q1)
        self.state["source_progress"][source] = hi
        self.state["active_index"] = hi
        self.state.pop("blocked_index", None)
        if hi == total - 1 and source not in self.state["completed_strokes"]:
            self.state["completed_strokes"].append(source)
        self.save()

    def prepare_source(self, selected):
        # Every successful pass adds a unique endpoint post. Rebuild the axis
        # proposal against its new material; never reuse the pre-post poses.
        while True:
            result = self._prepare_source_pass(selected)
            if self.state["status"] not in {"SUPPORT_ADDED", "SOURCE_DIRECTION_CHANGED"}:
                return result

    def source_drawing(self, selected):
        """Keep the same closed contour and anchor while allowing either traversal."""
        source = selected["stroke_id"]
        original = next(
            d
            for d in self.planner.candidates["variants"]
            if d["stroke_id"] == source and d["variant"] == selected["variant"]
        )
        if self.state.get("source_directions", {}).get(source) == "reverse":
            if not self.planner.actions[source]["closed"]:
                raise ValueError("Only a closed contour can reverse at its existing anchor")
            if not np.allclose(
                original["tcp_targets_m"][0], original["tcp_targets_m"][-1], atol=1e-6, rtol=0
            ):
                raise ValueError("Closed contour endpoints differ")
            return {
                **original,
                "positions": original["positions"][::-1],
                "tcp_targets_m": original["tcp_targets_m"][::-1],
            }
        return original

    def _prepare_source_pass(self, selected):
        source = selected["stroke_id"]
        original = self.source_drawing(selected)
        points = np.asarray(original["tcp_targets_m"])
        start = self.state["source_progress"].get(source, 0)
        action = self.planner.actions[source]
        alias = f"{source}-part-{start:04d}"
        self.planner.actions[alias] = {**action, "id": alias}
        seed_positions = original["positions"][start:]
        warm_seed = None
        if start:
            # A previous full proposal can supply solved postures after a
            # restart. It supplies seeds only: material and continuous motion
            # checks still run normally against the current deposited scene.
            prefix = f"proposal_{source}_"
            earlier = []
            for path in self.output.glob(prefix + "*.json"):
                suffix = path.stem[len(prefix):]
                if suffix.isdigit() and int(suffix) < start:
                    earlier.append((int(suffix), path))
            for prior_start, path in sorted(earlier, key=lambda item: (-item[0], item[1].name)):
                prior = read_json(path)
                if prior.get("status") != "SAMPLED_FREE_AXIS_PROPOSAL":
                    continue
                prior_targets = np.asarray(prior.get("tcp_targets_m", []), dtype=float)
                prior_positions = np.asarray(prior.get("positions", []), dtype=float)
                count = len(points) - prior_start
                offset = start - prior_start
                if (
                    prior_targets.shape != (count, 3)
                    or prior_positions.shape != (count, len(self.arm.robot.lower))
                    or not np.isfinite(prior_targets).all()
                    or not np.isfinite(prior_positions).all()
                    or np.any(prior_positions < self.arm.robot.lower)
                    or np.any(prior_positions > self.arm.robot.upper)
                    or not np.allclose(prior_targets[offset:], points[start:], atol=1e-9, rtol=0)
                ):
                    continue
                seed_positions = prior_positions[offset:].tolist()
                warm_seed = {
                    "file": str(path.relative_to(self.run)),
                    "sha256": sha256(path),
                    "proposal_start_index": prior_start,
                    "seed_start_index": start,
                    "preferred_check_revision": WARM_SEED_PREFERRED_CHECK_REVISION,
                    "scope": "Seed postures only; current material checks remain required",
                }
                break
        drawing = {
            **original,
            "stroke_id": alias,
            "positions": seed_positions,
            "tcp_targets_m": points[start:].tolist(),
            "release_cooling_s": self.cooling,
        }
        if warm_seed is not None:
            drawing["warm_seed_preferred_check_revision"] = WARM_SEED_PREFERRED_CHECK_REVISION
        self.state.update(status="PLANNING", active_stroke=source, active_index=start)
        self.state.pop("blocked_index", None)
        self.state.pop("diagnostic", None)
        self.save()
        # Repair once in the actual material context; ensuing chunks are all
        # independently certified and may change posture only with feed off.
        proposal_file = self.output / f"proposal_{source}_{start:04d}.json"
        binding = {
            "solver_revision": AXIS_REPAIR_REVISION,
            "work_solver_revision": AXIS_WORK_REVISION,
            "material": self.planner.material_key(self.scene),
            "drawing": hashlib.sha256(json.dumps(drawing, sort_keys=True).encode()).hexdigest(),
        }
        if warm_seed is not None:
            binding["warm_seed"] = warm_seed
            print(
                f"Seeding {source} at {start} from {warm_seed['file']} "
                "with current material revalidation", flush=True,
            )
        proposal = read_json(proposal_file) if proposal_file.exists() else {}
        if proposal.get("binding") != binding:
            print(f"Planning {source} from source vertex {start}/{len(points) - 1}", flush=True)
            proposal = repair_drawing_axes(
                self.planner,
                drawing,
                self.scene,
                refine=False,
                progress=lambda done, count, _: print(
                    f"{source}: {done}/{count} poses", flush=True
                ),
            )
            proposal["binding"] = binding
            write_json(proposal_file, proposal)
        if proposal["status"] != "SAMPLED_FREE_AXIS_PROPOSAL":
            if (
                start == 0
                and action["closed"]
                and self.state.get("source_directions", {}).get(source) != "reverse"
            ):
                # One finite alternative: reverse the unstarted loop. This can
                # avoid trapping the nozzle behind its own newly drawn strand.
                # Both the anchor and every source target remain unchanged.
                self.state.setdefault("direction_failures", {})[source] = proposal
                self.state.setdefault("source_directions", {})[source] = "reverse"
                self.state.update(status="SOURCE_DIRECTION_CHANGED")
                self.save()
                return False
            self.state.update(status="POSE_BLOCKED", diagnostic=proposal)
            self.save()
            return False
        qs = np.asarray(proposal["positions"])
        cuts = [start + i for i in posture_break_candidates(qs)] + [len(points) - 1]
        lo = start
        while cuts:
            hi = cuts.pop(0)
            screen = release_screen(
                self.graph,
                self.material,
                self.printed_members(),
                points[: hi + 1] - self.transform[:3, 3],
                action["diameter_m"],
                original["start_anchor"],
            )
            write_json(self.output / f"release_{source}_{hi:04d}.json", screen)
            if not screen["modeled_pass"]:
                if self.try_endpoint_support(source, lo, hi):
                    return False
                # A first long section may need a brace before its proposed
                # release. Find a supported earlier stop with finite halving,
                # rather than requiring an unprinted endpoint to carry a post.
                cut = (lo + hi) // 2
                while cut > lo:
                    earlier = release_screen(
                        self.graph, self.material, self.printed_members(),
                        points[: cut + 1] - self.transform[:3, 3],
                        action["diameter_m"], original["start_anchor"],
                    )
                    write_json(self.output / f"release_{source}_{cut:04d}.json", earlier)
                    if earlier["modeled_pass"]:
                        cuts[:0] = [cut, hi]
                        break
                    cut = (lo + cut) // 2
                if cut > lo:
                    continue
                self.state.update(status="SUPPORT_REQUIRED", active_index=lo, blocked_index=hi)
                self.save()
                return False
            q = qs[lo - start : hi - start + 1].copy()
            if lo > start or np.max(np.abs(q[1] - q[0])) > 0.14:
                # An initial branch change belongs in the pen-off approach,
                # not in the first tiny extrusion segment. Seed the start from
                # its outgoing posture, including on checkpoint resumption.
                seed = qs[lo - start + 1]
                solved, error, gaps = solve_joint_pose(
                    self.planner,
                    self.arm,
                    points[lo],
                    -self.arm.robot.forward(seed, self.arm.flange_to_tcp)[:3, 2],
                    seed,
                    seed,
                    self.scene,
                    self.planner.margin + 0.00002,
                )
                def static_restart(candidate, name, tolerance, target=points[lo]):
                    candidate = np.asarray(candidate)
                    finite = bool(np.isfinite(candidate).all())
                    in_limits = finite and bool(
                        np.all(candidate >= self.arm.robot.lower)
                        and np.all(candidate <= self.arm.robot.upper)
                    )
                    delta, collisions = None, []
                    if in_limits:
                        delta = float(np.linalg.norm(
                            self.arm.robot.forward(candidate, self.arm.flange_to_tcp)[:3, 3]
                            - target
                        ))
                        collisions = self.arm.collision.check(
                            candidate, self.planner.obstacles,
                            self.planner.margin + 0.000018, self.scene,
                        )
                    return {
                        "name": name,
                        "positions": candidate.tolist() if finite else None,
                        "position_error_m": delta,
                        "position_tolerance_m": tolerance,
                        "clearance_m": self.planner.margin + 0.000018,
                        "collisions": collisions,
                        "acceptable": bool(
                            in_limits and delta is not None and delta <= tolerance
                            and not collisions
                        ),
                    }

                outgoing = static_restart(solved.x, "outgoing_seed", 1e-6)
                outgoing["solver_message"] = str(solved.message)
                outgoing["solver_minimum_clearance_m"] = (
                    float(gaps.min()) if np.isfinite(gaps).all() else None
                )
                outgoing["acceptable"] = outgoing["acceptable"] and bool(
                    np.isfinite(gaps).all()
                    and gaps.min() >= self.planner.margin + 0.000018
                )
                choices = [outgoing]
                chosen = outgoing if outgoing["acceptable"] else None
                if chosen is None:
                    # The preferred outgoing branch is optional. An already
                    # solved endpoint may still be clear after the last draw.
                    original_start = static_restart(q[0], "original_q0", 0.00005)
                    choices.append(original_start)
                    chosen = original_start if original_start["acceptable"] else None
                if chosen is None:
                    # Only one further known pose: the actual accepted source
                    # endpoint. No new seed search or material removal occurs.
                    prior = next(
                        (
                            row for row in reversed(self.state["motions"])
                            if row["kind"] == "fragment"
                            and row["source_stroke"] == source and row["end_index"] == lo
                        ), None,
                    )
                    if prior is not None:
                        endpoint = next(
                            (e.q1 for e in reversed(self.read_motion(prior).edges)
                             if e.kind == "DRAW"), None,
                        )
                        if endpoint is not None:
                            accepted = static_restart(endpoint, "last_accepted_draw_end", 0.00005)
                            choices.append(accepted)
                            chosen = accepted if accepted["acceptable"] else None
                diagnostic_file = f"full_build/restart_pose_{source}_{lo:04d}.json"
                write_json(
                    self.run / diagnostic_file,
                    {
                        "source_stroke": source, "source_index": lo, "choices": choices,
                        "chosen": None if chosen is None else chosen["name"],
                        "scope": "Static restart proposal; full trajectory preparation required",
                    },
                )
                self.state["restart_pose_diagnostic"] = {
                    "file": diagnostic_file, "sha256": sha256(self.run / diagnostic_file)
                }
                if chosen is None:
                    self.state.update(status="RESTART_POSE_BLOCKED", active_index=lo)
                    self.save()
                    return False
                q[0] = np.asarray(chosen["positions"])
            refined, targets = refine_axis_path(self.arm, q.tolist(), points[lo : hi + 1].tolist())
            sub = {
                **drawing,
                "positions": refined,
                "tcp_targets_m": targets,
                "free_tool_orientation": True,
            }
            state = ScheduleState(
                0, ((self.arm.identity, tuple(self.q)),), frozenset(), (), payload=self.scene
            )
            print(f"Checking {source} chunk {lo}:{hi}", flush=True)
            move = self.planner._prepare_drawing(state, self.arm.identity, sub, self.q)
            for correction in range(4):
                if move is not None:
                    break
                failure = self.planner.attempts[-1]
                if failure["phase"] != "DRAW" or "positions" not in failure or hi - lo != 1:
                    break
                # Refine the low-clearance interior of this same polynomial,
                # retaining earlier collocations and the exact source polyline.
                repaired = repair_collision_witness(
                    self.planner, sub, self.scene, failure
                )
                repaired["correction_index"] = correction + 1
                write_json(
                    self.output / f"witness_{source}_{lo:04d}_{hi:04d}.json", repaired
                )
                write_json(
                    self.output / f"witness_{source}_{lo:04d}_{hi:04d}_{correction + 1}.json",
                    repaired,
                )
                if repaired["status"] != "SAMPLED_FREE_AXIS_PROPOSAL":
                    break
                sub = {**sub, **repaired}
                print(
                    f"Rechecking corrected {source} interval {lo}:{hi} "
                    f"(collocation {correction + 1}/4)", flush=True,
                )
                move = self.planner._prepare_drawing(state, self.arm.identity, sub, self.q)
            if move is None:
                failure = self.planner.attempts[-1]
                self.state["failures"].append(
                    {"source": source, "start_index": lo, "end_index": hi, **failure}
                )
                # A finite source-vertex split at the witnessed failed interval.
                # Adjacent vertices cannot be subdivided by this policy.
                if failure["phase"] == "DRAW" and "positions" in failure and hi - lo > 1:
                    tip = self.arm.robot.forward(failure["positions"], self.arm.flange_to_tcp)[
                        :3, 3
                    ]
                    interior = np.arange(lo + 1, hi)
                    cut = int(interior[np.argmin(np.linalg.norm(points[interior] - tip, axis=1))])
                    cuts[:0] = [cut, hi]
                    self.save()
                    continue
                self.state.update(status="MOTION_BLOCKED", active_index=lo, blocked_index=hi)
                self.save()
                return False
            self.accept(source, lo, hi, move, len(points))
            lo = hi
        return True

    def run_remaining(self):
        for selected in self.selection:
            if selected["stroke_id"] in self.state["completed_strokes"]:
                continue
            if not self.prepare_source(selected):
                return
        self.state.update(status="ALL_STROKES_PREPARED_REPLAY_PENDING")
        self.save()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--run",
        required=True,
        help="One-arm cell run with selected allocation; starts empty or resumes its saved prefix",
    )
    parser.add_argument("--profile", default="configs/cell/one_flexiv_straight.yaml")
    args = parser.parse_args()
    build = SingleArmBuild(args.run, args.profile)
    try:
        build.run_remaining()
        if build.state["status"] == "ALL_STROKES_PREPARED_REPLAY_PENDING":
            from .full_build_export import export_full_build

            export_full_build(build)
    except Exception as exc:
        # Keep an unexpected terminal exception distinguishable from a live
        # planner. Preserve the accepted prefix and propagate the traceback.
        build.state.update(
            status="PLANNER_ERROR",
            planner_error={
                "type": type(exc).__name__,
                "message": str(exc),
                "previous_status": build.state["status"],
            },
        )
        build.save()
        raise
    print(
        build.state["status"],
        len(build.state["completed_strokes"]),
        "/",
        len(build.selection),
        flush=True,
    )


if __name__ == "__main__":
    main()
