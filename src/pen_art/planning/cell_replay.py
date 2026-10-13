"""Replay a saved common clock, resuming only an identical verified prefix."""

import argparse
import hashlib
import json
import pickle
from collections import Counter, OrderedDict
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from multiprocessing import get_context
from pathlib import Path
from shutil import rmtree
from time import monotonic, perf_counter

import numpy as np

from ..io import artifact, read_json, sha256, stage_update, write_json
from ..provenance import audit_source_provenance
from .cell_planner import CellPlanner
from .clock_path import (
    POSTURE_LIMIT_DEG_PER_MM,
    ClockPath,
    polynomial_tcp_speed_bound,
    posture_change,
    tcp_tangent_bound,
)
from .dual_arm import MotionProposal, ScheduledMotion, ScheduleState
from .process_clock import PROCESS_CLOCK_POLICY, action_ready_times, audit_process_clock
from .replay_checkpoint import (
    IMPLEMENTATION,
    read_checkpoint,
    restore_state,
    save_checkpoint,
)
from .replay_checkpoint import (
    digest as context_digest,
)
from .scene import MaterialScene
from .sections import coverage_audit, replay_sections

# Smallest parallel replay job, in clock edges, and the number of jobs to aim
# for per worker. Smaller chunks balance load; each still pays for a copy of
# the material preceding it.
PARALLEL_CHUNK_EDGES = 256
PARALLEL_JOBS_PER_WORKER = 8
# Inherited by forked replay workers; never pickled.
_PARALLEL = None
_LOCAL = {}


def _parallel_audit(index):
    row = _PARALLEL["rows"][index]
    started = perf_counter()
    path = ClockPath.from_record(row["trajectory"])
    parse_s = perf_counter() - started
    started = perf_counter()
    audit = _PARALLEL["audit"](row, path)
    audit["worker_clock_parse_s"] = parse_s
    audit["worker_path_audit_s"] = perf_counter() - started
    return path, audit


def _parallel_structural(index):
    return _PARALLEL["structural"](index)


def _parallel_journal(root, key):
    from .material_journal import load_journal

    cache = _LOCAL.setdefault("journals", OrderedDict())
    if key not in cache:
        path = root / f"{key}.json"
        value = (
            load_journal(path)
            if path.exists()
            else pickle.loads(path.with_suffix(".pkl").read_bytes())
        )
        from .parallel_build import deposit_key

        if deposit_key(value) != key:
            raise ValueError("Changed replay deposit journal")
        cache[key] = value
        while len(cache) > 4:
            cache.popitem(last=False)
    cache.move_to_end(key)
    return cache[key]


def _parallel_deposit(index):
    """Journal one interval's turns and deposits, without any collision check."""
    from .parallel_build import deposit_key

    planner = _PARALLEL["planner"]
    a, b, new = _PARALLEL["intervals"][index]
    state, table_index = _PARALLEL["states"][index]
    scene = MaterialScene()
    scene.table_index = table_index
    operations = []
    # A rejected interval is reported by its own full check. Later intervals
    # are never accepted past it, so a partial journal cannot be relied upon.
    planner.validate_interval(
        replace(state, payload=scene), new, b, deposit_only=True, journal=operations
    )
    from .parallel_build import _merged

    operations = _merged(planner, operations)
    key = deposit_key(operations)
    from .material_journal import write_journal

    path = _PARALLEL["root"] / f"{key}.json"
    if not path.exists():
        write_journal(path, operations)
    return key


def _parallel_check(index, chain):
    """Fully check one interval against material rebuilt from earlier journals."""
    planner = _PARALLEL["planner"]
    if "prefix" not in _LOCAL:
        _LOCAL["prefix"] = _PARALLEL["prefix"]()
    a, b, new = _PARALLEL["intervals"][index]
    state, _ = _PARALLEL["states"][index]
    scene = _LOCAL["prefix"].scene_for(chain)
    before = (planner.checked_intervals, Counter(planner.verification_stats))
    seconds = Counter(planner.verification_seconds)
    operations = []
    after, failure = planner.validate_interval(
        replace(state, payload=scene), new, b, journal=operations
    )
    from .parallel_build import _merged, deposit_key

    return {
        "failure": failure,
        # What this full check deposited, in the form later checks receive.
        "deposited": deposit_key(_merged(planner, operations)),
        "before": scene.geometry_prefix(),
        "after": None if failure else after.geometry_prefix(),
        "capsules": None if failure else len(after.objects),
        "checked_intervals": planner.checked_intervals - before[0],
        "counts": Counter(planner.verification_stats) - before[1],
        "seconds": Counter(planner.verification_seconds) - seconds,
    }


def _parallel_arm_check(index):
    """Check the arm, not the pen, on an interval whose pen holds planning's proof.

    Every edge is checked continuously with the full robot against itself, and
    its arm links against the cell and one volume enclosing all printed wire.
    The tool is left out of the obstacle tests only: its proof covers them.
    """
    from .collision import ArmLinksModel

    planner = _PARALLEL["planner"]
    a, b, new = _PARALLEL["intervals"][index]
    state, _ = _PARALLEL["states"][index]
    obstacles = {**planner.obstacles, "printed_volume": _PARALLEL["envelope"]}
    depth, edges = planner.settings["maximum_subdivision_depth"], 0
    for motion in state.active + new:
        arm = planner.arms[motion.proposal.arm]
        models = _LOCAL.setdefault("arm_links", {})
        if motion.proposal.arm not in models:
            models[motion.proposal.arm] = ArmLinksModel(
                arm.collision, {body.name for body in arm.tool.bodies}
            )
        model = models[motion.proposal.arm]
        for edge in motion.proposal.payload.edges:
            if not a - 1e-9 <= motion.start_s + edge.start_s < b - 1e-9:
                continue
            if edge.coefficients is None:
                check = model.check_edge(
                    edge.q0, edge.q1, obstacles, planner.margin, depth, None, planner.deadline
                )
            else:
                piece = edge.piece()
                check = model.check_bounded_path(
                    piece.position,
                    piece.deviation,
                    obstacles,
                    planner.margin,
                    depth,
                    None,
                    planner.deadline,
                )
            if check["status"] != "CERTIFIED_MODELED_EDGE":
                return {"certified": False, "detail": check}
            edges += 1
    return {"certified": True, "edges": edges}


def _printed_volume(planner):
    """One upright cylinder about the build axis holding every wire of the object."""
    import fcl

    from ..curves import transform_points
    from .collision import placed

    points = np.array([node["position"] for node in planner.graph["nodes"]])
    radius = max(edge["diameter_m"] for edge in planner.graph["edges"]) / 2 + 0.001
    transforms = (
        [np.asarray(planner.profile["world_from_build"])]
        if planner.turntable is None
        else [planner.turntable.build_transform(index) for index in range(4)]
    )
    axis = transforms[0][:3, 3]
    world = np.vstack([transform_points(points, transform) for transform in transforms])
    reach = float(np.linalg.norm(world[:, :2] - axis[:2], axis=1).max()) + radius
    low, high = world[:, 2].min() - radius, world[:, 2].max() + radius
    pose = np.eye(4)
    pose[:3, 3] = [axis[0], axis[1], (low + high) / 2]
    return placed(fcl.Cylinder(reach, float(high - low)), pose)


def _replay_cell(*args, **kwargs):
    pools = []
    try:
        return _replay_cell_body(*args, pools=pools, **kwargs)
    finally:
        for pool in pools:
            pool.shutdown(wait=False, cancel_futures=True)


def _replay_cell_body(
    run,
    profile_path,
    maximum_seconds=300,
    plan_name="dual_arm_plan",
    return_state=False,
    *,
    path_certificates=None,
    verification_raw=True,
    reuse_material_checkpoint=True,
    workers=1,
    optimize_contact_geometry=True,
    geometry_proofs=False,
    pen_proofs=False,
    pools,
):
    validation_name = plan_name.removesuffix("_plan") + "_validation.json"
    plan_file = plan_name + ".json"
    run = Path(run)
    phase_seconds, phase_started = {}, perf_counter()

    def mark(name):
        nonlocal phase_started
        phase_seconds[name] = phase_seconds.get(name, 0.0) + perf_counter() - phase_started
        phase_started = perf_counter()

    from .clock_store import load_plan, plain

    saved, plan_digest = load_plan(run / plan_file)
    mark("load_plan")
    for name, digest in saved["inputs"].items():
        if sha256(run / name) != digest:
            raise ValueError(f"Stale plan input: {name}")
    settings = read_json(run / saved.get("settings_artifact", "dual_arm_config.json"))
    # Planning may deliberately defer robot collision checks. A final replay,
    # including its accelerated mode, always restores the complete model.
    replay_search = {**settings["search"], "collision_scope": "full_robot"}
    planner = CellPlanner(
        run,
        profile_path,
        replay_search,
        monotonic() + maximum_seconds,
        timing_override=settings["timing"],
        verification_raw=verification_raw,
        optimize_contact_geometry=optimize_contact_geometry,
    )
    planner.transit, planner.timing, planner.process = (
        settings[k] for k in ("transit", "timing", "process")
    )
    # Path certificates can survive a scheduling repair. Material checkpoints
    # additionally bind the entire plan: changed timing must replay from zero.
    certificate_context = context_digest(
        {
            "inputs": saved["inputs"],
            "settings": settings,
            "profile": planner.profile,
            "implementation": IMPLEMENTATION,
            "process_clock_policy": PROCESS_CLOCK_POLICY,
            "robots": {
                identity: {
                    "urdf": sha256(arm.urdf),
                    "collision_meshes": {
                        mesh.get("filename"): sha256(Path(mesh.get("filename")))
                        for mesh in arm.robot.xml.findall("link/collision/geometry/mesh")
                    },
                    "tool": arm.tool.metadata,
                }
                for identity, arm in planner.arms.items()
            },
        }
    )
    checkpoint_context = context_digest([certificate_context, plan_digest])
    checkpoint_path = run / (plan_name.removesuffix("_plan") + "_replay_checkpoint.json")
    checkpoint = (
        None
        if verification_raw or not reuse_material_checkpoint
        else read_checkpoint(checkpoint_path, checkpoint_context)
    )
    initial = {identity: tuple(q) for identity, q in saved["initial_positions"].items()}
    if set(initial) != set(planner.arms):
        raise ValueError("Saved initial poses do not match the cell")
    tasks = {task.identity: task for task in planner.tasks}
    section_audits = {}
    partial_sections = saved.get("partial_section_schedule", False)
    if geometry_proofs or pen_proofs:
        # Speed and source-coverage proofs planning wrote for these exact clocks.
        # They bind the path, kinematics, source and process settings. Collision,
        # orientation, deposits and structure are still checked here in full.
        from .path_certificates import PathCertificateStore

        planner.path_certificates = PathCertificateStore(
            run / "path_certificates", fallback=run / "parallel" / "path_certificates"
        )
    store = (
        getattr(planner, "path_certificates", None)
        if geometry_proofs or not verification_raw
        else None
    )
    mark("context_setup")

    def audit_action(row, path, tick=None):
        """Every saved-clock audit that needs no other action. Raises on violation."""
        result = {"turns": 0, "initial_index": None, "final_index": None, "checked_edges": 0}
        invariant_context = (
            None if store is None else planner.path_certificate_context(row["arm"], kinematic=True)
        )
        proofs = result["proofs"] = []
        # Looked up here, on the workers: binding a proof digests the whole clock.
        result["pen_proof"] = (
            store.get(
                planner.path_certificate_context(row["arm"], pen=True),
                row["trajectory"],
                "pen_material_path",
            )
            if pen_proofs and store is not None and not row.get("auxiliary", False)
            else None
        )
        if planner.turntable is not None:
            from ..curves import flatten_curve, transform_points
            from .source_coverage import audit_source_curve
            from .turntable import audit_indexed_clock

            result.update(
                audit_indexed_clock(
                    planner.turntable,
                    path,
                    np.asarray(initial[row["arm"]]),
                    planner.settings["assumed_cooling_s"],
                )
            )
            if not row.get("auxiliary", False):
                index = row["table_index"]
                if any(edge.table_index != index for edge in path.edges if edge.kind == "DRAW"):
                    raise ValueError("Drawing index differs from the source reference frame")
                points, _ = flatten_curve(planner.actions[row["stroke_id"]], 0.00005, 0.003)
                source_points = transform_points(points, planner.turntable.build_transform(index))
                coverage_context = {
                    "invariants": invariant_context,
                    "source_points_m": source_points,
                }
                coverage = (
                    None
                    if store is None
                    else store.get(coverage_context, row["trajectory"], "source_coverage")
                )
                if coverage is not None:
                    proofs.append("source_coverage")
                if coverage is None:
                    coverage = audit_source_curve(planner.arms[row["arm"]], [path], source_points)
                    if coverage["status"] == "BOUNDED" and store is not None:
                        store.admit(
                            coverage_context, row["trajectory"], "source_coverage", coverage
                        )
                if coverage["status"] != "BOUNDED":
                    raise ValueError("Indexed drawing misses its material-relative source")
        if path.metadata.get("process_clock_policy") == PROCESS_CLOCK_POLICY and not row.get(
            "auxiliary", False
        ):
            action = planner.actions[row["stroke_id"]]
            transform = (
                planner.turntable.build_transform(row["table_index"])
                if planner.turntable is not None
                else np.asarray(planner.profile["world_from_build"])
            )
            if path.metadata.get("source_reverse", False) != row.get(
                "reverse", False
            ) or path.metadata.get("source_start_visit", 0) != row.get("start_visit", 0):
                raise ValueError("Saved clock traversal differs from its source action")
            audit_process_clock(
                path,
                action,
                planner.process,
                planner.settings["assumed_cooling_s"],
                transform,
                require_release=saved.get("section_schedule", False) or partial_sections,
            )
        arm = planner.arms[row["arm"]]
        swing = posture_change(path, arm)
        if swing:
            raise ValueError(f"Arm changes posture while the pen works: {swing}")
        key = None
        if path_certificates is not None or not verification_raw:
            key = (
                certificate_context,
                row["arm"],
                hashlib.sha256(
                    json.dumps(plain(row["trajectory"]), sort_keys=True).encode()
                ).hexdigest(),
            )
        reuse = not verification_raw and (
            checkpoint is not None or (path_certificates is not None and key in path_certificates)
        )
        speed_proof = (
            None
            if store is None
            else store.get(invariant_context, row["trajectory"], "joint_speed")
        )
        reuse = reuse or speed_proof is not None
        drawing_proof = (
            None
            if store is None
            else store.get(invariant_context, row["trajectory"], "drawing_speed")
        )
        proofs.extend(
            name
            for name, proof in (("joint_speed", speed_proof), ("drawing_speed", drawing_proof))
            if proof is not None
        )
        result.update(reuse=reuse, key=key)
        for edge_index, edge in () if reuse else enumerate(path.edges):
            if drawing_proof is not None and edge.kind == "DRAW":
                result["checked_edges"] += 1
                continue
            if monotonic() >= planner.deadline:
                raise TimeoutError("Replay path-check deadline")
            arm.robot.frames(edge.q0)
            arm.robot.frames(edge.q1)
            duration = edge.end_s - edge.start_s
            dq = abs(edge.q1 - edge.q0)
            limit = (
                planner.process["nominal_speed_m_s"]
                if edge.kind == "DRAW"
                else planner.timing["travel_speed_m_s"]
            )
            if edge.coefficients is not None:
                piece = edge.piece()
                ranges = piece.ranges(0, 1)
                if np.any(ranges[:, 0] < arm.robot.lower - 1e-9) or np.any(
                    ranges[:, 1] > arm.robot.upper + 1e-9
                ):
                    raise ValueError("Saved polynomial exceeds joint position limits")
                first, second = piece.derivatives(0, 1)
                velocity, acceleration = first / duration, second / duration**2
                speed_bound = polynomial_tcp_speed_bound(
                    edge, arm, limit, deadline=planner.deadline
                )
            else:
                velocity = 1.875 * dq / duration
                acceleration = (10 / np.sqrt(3)) * dq / duration**2
                speed_bound = 1.875 * tcp_tangent_bound(arm, edge.q0, edge.q1) / duration
            if np.any(velocity > arm.robot.velocity * planner.timing["velocity_fraction"] + 1e-8):
                raise ValueError("Saved path exceeds velocity limits")
            if np.any(acceleration > np.array(planner.timing["joint_acceleration_rad_s2"]) + 1e-8):
                raise ValueError("Saved path exceeds acceleration limits")
            if speed_bound > limit + 1e-8:
                raise ValueError(
                    "Cannot certify saved tip speed: "
                    f"{row['arm']}/{row['stroke_id']} edge {edge_index}; "
                    f"upper bound {speed_bound:.9g} m/s, limit {limit:.9g} m/s"
                )
            if tick is None:
                result["checked_edges"] += 1
            else:
                tick()
        if store is not None and not reuse:
            if drawing_proof is None and any(edge.kind == "DRAW" for edge in path.edges):
                store.admit(
                    invariant_context, row["trajectory"], "drawing_speed", {"status": "CHECKED"}
                )
            store.admit(
                invariant_context,
                row["trajectory"],
                "joint_speed",
                {"status": "CHECKED", "checked_edges": len(path.edges)},
            )
        return result

    ordered = sorted(saved["actions"], key=lambda row: row["start_s"])
    parallel = workers > 1 and len(planner.arms) == 1 and checkpoint is None
    structural = None

    def audit_pool():
        """Fork once every audit closure exists; workers inherit them all."""
        global _PARALLEL
        if parallel and not pools:
            _PARALLEL = {
                "planner": planner,
                "audit": audit_action,
                "structural": structural,
                "rows": ordered,
            }
            pools.append(ProcessPoolExecutor(max_workers=workers, mp_context=get_context("fork")))
        return pools[-1] if pools else None

    if partial_sections and saved.get("full_schedule", False):
        raise ValueError("A partial section schedule cannot claim full completion")
    if saved.get("section_schedule") or partial_sections:
        sections = [row for row in saved["actions"] if not row.get("auxiliary", False)]
        for row in sections:
            expected = planner.actions.get(row["stroke_id"])
            if expected is None or any(
                row[key] != expected[key]
                for key in (
                    "id",
                    "parent_curve",
                    "source_interval",
                    "members",
                    "stage",
                    "start",
                    "end",
                )
            ):
                raise ValueError("Saved section provenance differs from construction source")
        # Join locations come from the bound source, never untrusted annotations
        # in the saved schedule. Older schedules omit these optional fields.
        for row in sections:
            expected = planner.actions[row["stroke_id"]]
            for key in ("junction_visits", "closed", "source_intervals"):
                if key in row and row[key] != expected.get(key):
                    raise ValueError("Saved section junctions differ from construction source")
                if key in expected:
                    row[key] = expected[key]
        expected_sections = list(planner.actions.values())
        provenance_graph = planner.graph
        if partial_sections:
            from .partial_export import prefix_source_graph

            expected_sections = [planner.actions[row["stroke_id"]] for row in sections]
            if any(row["stage"] == "artistic" for row in sections):
                completed = {row["stroke_id"] for row in sections}
                framework = {
                    key for key, row in planner.actions.items() if row["stage"] == "framework"
                }
                if not framework <= completed:
                    raise ValueError("Partial schedule starts art before the full framework")
            provenance_graph = prefix_source_graph(planner.graph, sections)
        section_audits["source_coverage"] = coverage_audit(sections, expected_sections)
        section_audits["source_provenance"] = audit_source_provenance(sections, provenance_graph)
        section_audits["source_audit_scope"] = (
            "selected_prefix" if partial_sections else "full_build"
        )
        section_audits["stage_audit"] = replay_sections(
            sections, planner.graph, cooling_s=planner.settings["assumed_cooling_s"]
        )
        from ..structure import analyze_member_prefixes, analyze_structure

        material = read_json(run / "fabrication_config.json")["structure"]
        deposited, members = [], set()
        for row in sections:
            deposited.append(frozenset(members))
            members.update(row["members"])

        # Every completed action's structure is screened. Screening a half-drawn
        # action after each of its members is opt-in, saved with the plan.
        member_screens = planner.settings.get("structural_check", "action") == "member"
        section_audits["structural_check"] = "member" if member_screens else "action"

        def structural(index):
            # Each section's member prefixes and release state depend only on
            # the ordered members before it, never on another section's verdict.
            row = sections[index]
            prefixes = (
                analyze_member_prefixes(
                    planner.graph,
                    material,
                    planner.actions[row["stroke_id"]],
                    deposited_members=set(deposited[index]),
                    reverse=row.get("reverse", False),
                    start_visit=row.get("start_visit", 0),
                )
                if member_screens
                else {"modeled_pass": True, "status": "MEMBER_PREFIXES_NOT_SCREENED"}
            )
            screen = analyze_structure(
                planner.graph, material, edge_ids=deposited[index] | set(row["members"])
            )
            # Keep the deformed shape of the finished object only. Every earlier
            # release keeps its verdict and extremes; their per-node and
            # per-member arrays made the report over a gigabyte on a large build.
            if index < len(sections) - 1:
                screen = {k: v for k, v in screen.items() if k not in {"nodes", "edges"}}
            return prefixes, screen

        pool = audit_pool()
        screens = (
            (structural(index) for index in range(len(sections)))
            if pool is None
            else [
                future.result()
                for future in [
                    pool.submit(_parallel_structural, index) for index in range(len(sections))
                ]
            ]
        )
        releases = []
        for row, (prefixes, screen) in zip(sections, screens, strict=True):
            if not prefixes["modeled_pass"]:
                raise ValueError(f"Structural member prefix rejected in {row['id']}: {prefixes}")
            if not screen["modeled_pass"]:
                raise ValueError(f"Structural release rejected after {row['id']}: {screen}")
            releases.append({"section": row["id"], "screen": screen})
        section_audits["release_states"] = releases
    mark("load_and_source_audits")
    horizon = saved["modeled_duration_s"]
    if horizon < 0 or not np.isfinite(horizon):
        raise ValueError("Invalid replay horizon")
    state = ScheduleState(
        0,
        tuple(initial.items()),
        frozenset(),
        tuple((n, 0) for n in planner.ground),
        payload=MaterialScene(),
    )
    if planner.turntable is not None:
        if (
            saved.get("turntable") != planner.profile["turntable"]
            or saved.get("initial_table_index") != planner.turntable.home_index
        ):
            raise ValueError("Turntable profile or initial index does not match the plan")
        state.payload.table_index = planner.turntable.home_index
    output = artifact(
        "dual_arm_validation",
        status="RUNNING",
        phase="PATH_CHECKS",
        simulation_only=True,
        executable=False,
        source_plan_sha256=plan_digest,
        requires_final_verification=bool(saved.get("requires_final_verification")),
        inputs={plan_file: plan_digest},
        horizon_s=horizon,
        requested_strokes=len(tasks),
        completed_strokes=0,
        checked_intervals=0,
        checked_actions=0,
        reused_path_checks=0,
        posture_limit_deg_per_mm=POSTURE_LIMIT_DEG_PER_MM,
        geometry_proofs_accepted=geometry_proofs,
        pen_proofs_accepted=pen_proofs,
        pen_proof_intervals=0,
        pen_proof_fallbacks=0,
        geometry_proof_hits={},
        verification_raw=verification_raw,
        optimize_contact_geometry=optimize_contact_geometry,
        reuse_material_checkpoint=not verification_raw and reuse_material_checkpoint,
        requested_actions=len(saved["actions"]),
        checked_edges=0,
        requested_edges=sum(len(row["trajectory"]["edges"]) for row in saved["actions"]),
        failures=[],
        **section_audits,
        scope=[
            "Saved joint paths and clocks, joint limits, velocity and acceleration",
            "Both arms, static obstacles and growing material on the shared clock; "
            "an identical verified prefix may resume from its exact material checkpoint",
            "Start-anchor readiness and final return for a complete plan",
        ],
        limitations=[
            *saved["limitations"],
            *(
                []
                if planner.turntable is not None
                else [
                    "Cartesian reference tracking uses the planner's preparation audit; "
                    "it is not recomputed here",
                ]
            ),
        ],
    )
    published = [-np.inf]

    def save(final=False):
        """Publish progress cheaply; the structural release states can be tens of
        megabytes and belong only in the finished report. Progress is written
        at most every five seconds: the report grows with the build.
        """
        if not final and monotonic() - published[0] < 5:
            return
        published[0] = monotonic()
        write_json(
            run / validation_name,
            output if final else {k: v for k, v in output.items() if k != "release_states"},
        )

    save()
    last_write = monotonic()

    def budget_result():
        output.update(
            status="REPLAY_TIME_BUDGET",
            failures=[{"status": "TIME_BUDGET", "phase": output["phase"]}],
        )
        save(final=True)
        return (output, state) if return_state else output

    pool = audit_pool()
    if pool is None:
        audits, paths = None, []
        for row in ordered:
            if monotonic() >= planner.deadline:
                return budget_result()
            paths.append(ClockPath.from_record(row["trajectory"]))
    else:
        # Workers parse and audit each saved clock; the parsed clock returns
        # with its audit, so the parent neither reparses nor trusts raw rows.
        audits = [pool.submit(_parallel_audit, index) for index in range(len(ordered))]
        try:
            parsed = [future.result() for future in audits]
        except TimeoutError:
            return budget_result()
        paths, audits = [row[0] for row in parsed], [row[1] for row in parsed]
    if audits is not None:
        output["worker_stage_seconds"] = {
            "clock_parse": sum(row["worker_clock_parse_s"] for row in audits),
            "path_audit": sum(row["worker_path_audit_s"] for row in audits),
            "scope": "Summed worker elapsed seconds; these are not wall times",
        }
    mark("parse_clocks" if pool is None else "parse_and_path_checks")

    def tick():
        # Keep partial progress visible when a later edge exhausts the budget.
        nonlocal last_write
        output["checked_edges"] += 1
        if monotonic() - last_write >= 1:
            save()
            last_write = monotonic()

    moves, pen_proof_rows = [], {}
    previous = {identity: (0.0, q) for identity, q in initial.items()}
    previous_index = saved.get("initial_table_index", 0)
    for position, (row, path) in enumerate(zip(ordered, paths, strict=True)):
        if monotonic() >= planner.deadline:
            return budget_result()
        output["active_action"] = {"arm": row["arm"], "stroke_id": row["stroke_id"]}
        try:
            audit = audit_action(row, path, tick) if audits is None else audits[position]
        except TimeoutError:
            return budget_result()
        if planner.turntable is not None:
            if audit["initial_index"] != previous_index:
                raise ValueError("Discontinuous indexed action sequence")
            previous_index = audit["final_index"]
            output["indexed_turns"] = output.get("indexed_turns", 0) + audit["turns"]
        start, end = row["start_s"], row["end_s"]
        if (
            not np.isfinite([start, end]).all()
            or start < 0
            or abs(end - start - path.duration_s) > 1e-8
        ):
            raise ValueError("Saved action times do not match its trajectory")
        previous_end, previous_q = previous[row["arm"]]
        if start < previous_end - 1e-9 or not np.allclose(
            previous_q, path.edges[0].q0, atol=1e-8, rtol=0
        ):
            raise ValueError("Overlapping or discontinuous arm channel")
        for name in audit["proofs"]:
            output["geometry_proof_hits"][name] = output["geometry_proof_hits"].get(name, 0) + 1
        if not row.get("auxiliary", False):
            pen_proof_rows[row["stroke_id"]] = audit.get("pen_proof")
        if audit["reuse"]:
            output["reused_path_checks"] += 1
            output["checked_edges"] += len(path.edges)
        else:
            output["checked_edges"] += audit["checked_edges"]
        if path_certificates is not None:
            path_certificates.add(audit["key"])
        if monotonic() - last_write >= 1:
            save()
            last_write = monotonic()
        proposal = MotionProposal(
            row["stroke_id"],
            row["arm"],
            row["variant"],
            path.duration_s,
            row["start_anchor"],
            tuple(path.edges[-1].q1),
            path,
            auxiliary=row.get("auxiliary", False),
        )
        moves.append(ScheduledMotion(proposal, start))
        previous[row["arm"]] = (end, proposal.end_configuration)
        output["checked_actions"] += 1
    mark("path_checks")
    drawings = [m for m in moves if m.proposal.task in tasks]
    if len({m.proposal.task for m in drawings}) != len(drawings):
        raise ValueError("A stroke is scheduled more than once")
    expected_events = sorted(
        [
            {
                "time_s": move.start_s + edge.start_s,
                "arm": move.proposal.arm,
                "stroke_id": move.proposal.task,
                "command": "FEED_ON" if edge.kind == "PEN_START_WAIT" else "FEED_OFF",
            }
            for move in moves
            for edge in move.proposal.payload.edges
            if edge.kind in ("PEN_START_WAIT", "PEN_STOP_WAIT")
            and move.start_s + edge.start_s <= horizon
        ],
        key=lambda event: (event["time_s"], event["arm"]),
    )
    if saved.get("events") != expected_events:
        raise ValueError("Saved feed commands do not match the per-arm path clock")
    ready = {node: 0 for node in planner.ground}
    for motion in drawings:
        for node, relative in action_ready_times(
            motion.proposal.payload,
            {"junction_visits": [{"node": node} for node in tasks[motion.proposal.task].deposits]},
            tasks[motion.proposal.task].cooling_s,
        ).items():
            ready[node] = min(ready.get(node, float("inf")), motion.start_s + relative)
    for motion in drawings:
        if ready.get(motion.proposal.start_anchor, float("inf")) > motion.start_s + 1e-9:
            raise ValueError("A stroke starts on future or still-cooling material")
        if motion.proposal.start_anchor not in tasks[motion.proposal.task].anchors:
            raise ValueError("Saved start anchor does not belong to its stroke")
    events = sorted(
        {0.0, horizon, *[t for m in moves for t in (m.start_s, m.end_s) if 0 < t < horizon]}
    )
    if checkpoint is not None:
        state = restore_state(checkpoint, moves, tasks, initial, planner.ground, events)
        planner.checked_intervals = checkpoint["checked_intervals"]
        output.update(
            resumed_from_s=state.time_s,
            completed_strokes=len(state.completed),
            checked_intervals=planner.checked_intervals,
        )
    verified_intervals = planner.checked_intervals
    last_checkpoint = monotonic()
    output.update(phase="COLLISION_REPLAY")
    output.pop("active_action", None)
    save()
    if not moves or horizon == 0:
        output["status"] = "NO_MOTION_TO_REPLAY"
        save(final=True)
        return (output, state) if return_state else output

    def advance(state, new, b, scene):
        """State after a verified interval; independent of the collision work."""
        active = state.active + new
        q = dict(state.configurations)
        done = set(state.completed)
        support_ready = dict(state.support_ready)
        for motion in active:
            if motion.end_s <= b + 1e-9:
                q[motion.proposal.arm] = motion.proposal.end_configuration
                if motion.proposal.task in tasks:
                    done.add(motion.proposal.task)
                    task = tasks[motion.proposal.task]
                    for node, relative in action_ready_times(
                        motion.proposal.payload,
                        {"junction_visits": [{"node": node} for node in task.deposits]},
                        task.cooling_s,
                    ).items():
                        support_ready[node] = min(
                            support_ready.get(node, float("inf")), motion.start_s + relative
                        )
        return replace(
            state,
            time_s=b,
            configurations=tuple(q.items()),
            completed=frozenset(done),
            support_ready=tuple(sorted(support_ready.items())),
            active=tuple(m for m in active if m.end_s > b + 1e-9),
            history=state.history + new,
            payload=scene,
        )

    def locate(failure, new):
        """Name the drawing action a rejected interval belongs to."""
        for motion in new:
            if motion.proposal.task in tasks:
                failure = {
                    **failure,
                    "stroke_id": motion.proposal.task,
                    "action_index": [m.proposal.task for m in drawings].index(motion.proposal.task),
                }
        return failure

    intervals = [
        (a, b, tuple(m for m in moves if abs(m.start_s - a) < 1e-9))
        for a, b in zip(events[:-1], events[1:], strict=True)
        if b > state.time_s
    ]
    failed = False
    if parallel:
        # Each interval is checked by one worker against material rebuilt from
        # deposit journals. The hash chain below proves every rebuilt prefix is
        # exactly the material its predecessor's full check produced.
        from .parallel_build import MaterialPrefix

        journal_root = run / (plan_name.removesuffix("_plan") + "_replay_journals")
        rmtree(journal_root, ignore_errors=True)
        journal_root.mkdir(parents=True)
        # Cut long actions at their own edge boundaries. The validator already
        # checks an action edge by edge, so these chunks are the same checks
        # and the same deposits in the same order, only on several workers.
        # Every job copies and re-indexes its preceding material, so aim for
        # a few jobs per worker rather than the smallest possible chunk.
        total = sum(len(move.proposal.payload.edges) for move in moves)
        step = max(PARALLEL_CHUNK_EDGES, total // (workers * PARALLEL_JOBS_PER_WORKER))
        refined = []
        for a, b, new in intervals:
            cuts = []
            if len(new) == 1:
                ends = [new[0].start_s + edge.end_s for edge in new[0].proposal.payload.edges]
                cuts = [t for t in ends[step - 1 :: step] if a < t < b]
            for first, last in zip([a, *cuts], [*cuts, b], strict=True):
                refined.append((first, last, new if first == a else ()))
        intervals = refined
        states, owners, table_index = [], [], state.table_index
        dry = state
        for _, b, new in intervals:
            if len(dry.active) + len(new) > 1:
                raise ValueError("Parallel replay requires one sequential arm channel")
            states.append((replace(dry, payload=None), table_index))
            owners.append(dry.active + new)
            for motion in owners[-1]:
                for edge in motion.proposal.payload.edges:
                    if (
                        edge.kind == "INDEX_TURN"
                        and dry.time_s - 1e-9 <= motion.start_s + edge.start_s < b - 1e-9
                    ):
                        table_index = edge.table_to_index
            dry = advance(dry, new, b, None)
        # Planning proved each action's pen clear of the cell and of the wire
        # of the actions before it. Accept that for the leading actions whose
        # saved clock holds such a proof and whose chain of preceding deposits
        # is exactly the one the earlier accepted proofs name.
        proven = set()
        if pen_proofs and bool(planner.settings.get("merge_deposits_m", 0.0)):
            from .replay_checkpoint import digest as chain_digest

            chain = []
            for move in sorted(moves, key=lambda move: move.start_s):
                if move.proposal.task not in pen_proof_rows:
                    continue  # A transit between actions; always checked in full.
                proof = (
                    None
                    if any(edge.kind == "INDEX_TURN" for edge in move.proposal.payload.edges)
                    else pen_proof_rows[move.proposal.task]
                )
                if proof is None or proof.get("chain_sha256") != chain_digest(chain):
                    break
                proven.add(move.proposal.task)
                chain.append(proof["deposit"])
        accepted = [len(owner) == 1 and owner[0].proposal.task in proven for owner in owners]
        _PARALLEL.update(
            envelope=_printed_volume(planner) if any(accepted) else None,
            intervals=intervals,
            states=states,
            root=journal_root,
            prefix=lambda: MaterialPrefix(
                planner.turntable, lambda key: _parallel_journal(journal_root, key)
            ),
        )
        # A second fork, so workers inherit the interval states computed above.
        pools[-1].shutdown(wait=True, cancel_futures=True)
        pool = ProcessPoolExecutor(max_workers=workers, mp_context=get_context("fork"))
        pools.append(pool)
        try:
            deposits = [pool.submit(_parallel_deposit, k) for k in range(len(intervals))]
            keys = [future.result() for future in deposits]
            mark("deposit_journals")

            def full_check(k):
                return pool.submit(_parallel_check, k, [[key, None] for key in keys[:k]])

            checks = [
                pool.submit(_parallel_arm_check, k) if accepted[k] else full_check(k)
                for k in range(len(intervals))
            ]
            expected, capsules = state.payload.geometry_prefix(), len(state.payload.objects)
            merging = bool(planner.settings.get("merge_deposits_m", 0.0))
            for k, future in enumerate(checks):
                result = future.result()
                _, b, new = intervals[k]
                if accepted[k] and result["certified"]:
                    # The arm is clear and the pen holds its proof; the material
                    # this interval adds is the journal later checks receive.
                    planner.checked_intervals += result["edges"]
                    output["pen_proof_intervals"] += 1
                    scene = MaterialScene()
                    scene.table_index = expected[1]
                    state = advance(state, new, b, scene)
                    output.update(
                        completed_strokes=len(state.completed),
                        checked_intervals=planner.checked_intervals,
                    )
                    save()
                    continue
                if accepted[k]:
                    # An arm link came near the printed volume: check it exactly.
                    output["pen_proof_fallbacks"] += 1
                    result = full_check(k).result()
                # Merged journals replace a check's fine wire for its successors,
                # so compare what the check deposited with the journal they use.
                if (
                    result["before"] != expected
                    if not merging
                    else not result["failure"] and result["deposited"] != keys[k]
                ):
                    raise ValueError("Parallel replay material prefix differs from its check")
                planner.checked_intervals += result["checked_intervals"]
                planner.verification_stats.update(result["counts"])
                planner.verification_seconds.update(result["seconds"])
                if result["failure"]:
                    failure = locate(result["failure"], owners[k])
                    output.update(
                        status="REPLAY_TIME_BUDGET"
                        if failure.get("status") == "TIME_BUDGET"
                        else "REPLAY_REJECTED",
                        failures=[failure],
                        checked_intervals=planner.checked_intervals,
                    )
                    failed = True
                    break
                expected, capsules = result["after"], result["capsules"]
                scene = MaterialScene()
                scene.table_index = expected[1]
                state = advance(state, new, b, scene)
                output.update(
                    completed_strokes=len(state.completed),
                    checked_intervals=planner.checked_intervals,
                )
                save()
            if merging and not failed:
                # The material every later check received, not the last check's own.
                whole = MaterialPrefix(
                    planner.turntable, lambda key: _parallel_journal(journal_root, key)
                ).scene_for([[key, None] for key in keys])
                expected, capsules = whole.geometry_prefix(), len(whole.objects)
        finally:
            pool.shutdown(wait=False, cancel_futures=True)
            rmtree(journal_root, ignore_errors=True)
        output["parallel_workers"] = workers
        final_material = {"capsules": capsules, "geometry_sha256": expected[0].hex()}
    else:
        save_checkpoint(checkpoint_path, checkpoint_context, state, verified_intervals)
        for _, b, new in intervals:
            scene, failure = planner.validate_interval(state, new, b)
            if failure:
                output.update(
                    status="REPLAY_TIME_BUDGET"
                    if failure.get("status") == "TIME_BUDGET"
                    else "REPLAY_REJECTED",
                    failures=[locate(failure, new)],
                    checked_intervals=planner.checked_intervals,
                )
                save_checkpoint(checkpoint_path, checkpoint_context, state, verified_intervals)
                failed = True
                break
            state = advance(state, new, b, scene)
            output.update(
                completed_strokes=len(state.completed), checked_intervals=planner.checked_intervals
            )
            verified_intervals = planner.checked_intervals
            if monotonic() - last_checkpoint >= 30 or b == horizon:
                save_checkpoint(checkpoint_path, checkpoint_context, state, verified_intervals)
                last_checkpoint = monotonic()
            save()
    mark("collision_replay")
    output["phase_seconds"] = phase_seconds
    if not failed:
        # Identical for serial and parallel replay of one plan.
        output["final_material"] = (
            final_material
            if parallel
            else {
                "capsules": len(state.payload.objects),
                "geometry_sha256": state.payload.geometry_prefix()[0].hex(),
            }
        )
        if planner.turntable is not None:
            output["final_table_index"] = state.table_index
        full = (
            saved.get("full_schedule", False) or saved["status"] == "COORDINATED_GEOMETRIC_PREVIEW"
        )
        if full and (
            len(state.completed) != len(tasks)
            or state.active
            or any(
                not np.allclose(dict(state.configurations)[i], q, atol=1e-8, rtol=0)
                for i, q in initial.items()
            )
            or (planner.turntable is not None and state.table_index != planner.turntable.home_index)
        ):
            output.update(
                status="REPLAY_REJECTED", failures=[{"status": "INCOMPLETE_OR_NOT_PARKED"}]
            )
        else:
            output["status"] = "CERTIFIED_GEOMETRIC_REPLAY" if full else "CERTIFIED_PREFIX_REPLAY"
    output["collision_scope"] = "full_robot"
    if getattr(planner, "path_certificates", None) is not None:
        output["verification"] = {
            "path_certificates": planner.path_certificates.report(),
            "counts": dict(planner.verification_stats),
            "seconds": dict(planner.verification_seconds),
        }
    save(final=True)
    return (output, state) if return_state else output


def replay_cell(
    run,
    profile_path,
    maximum_seconds=300,
    plan_name="dual_arm_plan",
    *,
    path_certificates=None,
    verification_raw=True,
    reuse_material_checkpoint=True,
    workers=1,
    optimize_contact_geometry=True,
    geometry_proofs=False,
    pen_proofs=False,
):
    """Replay a saved plan with the complete robot model.

    With several ``workers`` each scheduling interval still receives the same
    full check, against material rebuilt from deposit journals; a hash chain
    ties every rebuilt prefix to the checked result of the interval before it.
    """
    plan_file = plan_name + ".json"
    validation_name = plan_name.removesuffix("_plan") + "_validation.json"
    stage_update(
        Path(run),
        "validation",
        "running",
        artifact=validation_name,
        message="Replaying saved shared clock",
    )
    try:
        output = _replay_cell(
            run,
            profile_path,
            maximum_seconds,
            plan_name,
            path_certificates=path_certificates,
            verification_raw=verification_raw,
            reuse_material_checkpoint=reuse_material_checkpoint,
            workers=workers,
            optimize_contact_geometry=optimize_contact_geometry,
            geometry_proofs=geometry_proofs,
            pen_proofs=pen_proofs,
        )
        stage_update(
            Path(run),
            "validation",
            "complete" if output["status"] == "CERTIFIED_GEOMETRIC_REPLAY" else "partial",
            artifact=validation_name,
            message=output["status"] + "; modeled geometry only, physical fabrication unverified",
        )
        # New builds only save clocks. This independent step publishes their
        # verdict; the viewer never runs this verifier or certifies a schedule.
        if output.get("requires_final_verification"):
            passed = output["status"] == "CERTIFIED_GEOMETRIC_REPLAY"
            stage_update(
                Path(run),
                "motion",
                "complete" if passed else "partial",
                artifact=plan_file,
                message="Verified playback ready" if passed else "Playback awaiting verification",
            )
            report_path = Path(run) / "section_build.json"
            if report_path.exists():
                report = read_json(report_path)
                report.update(
                    status=(
                        "FULL_BUILD_VERIFIED"
                        if passed
                        else "FULL_BUILD_REPLAY_REJECTED"
                        if output["status"] == "REPLAY_REJECTED"
                        else "FULL_BUILD_VERIFICATION_INCOMPLETE"
                    ),
                    replay_status=output["status"],
                    collision_scope=(
                        "full_robot" if passed else report.get("planning_collision_scope")
                    ),
                    requires_full_robot_validation=not passed,
                    final_collision_scope=output.get("collision_scope"),
                )
                write_json(report_path, report)
        return output
    except Exception as error:
        path = Path(run) / validation_name
        digest = sha256(Path(run) / plan_file)
        try:
            output = read_json(path)
        except (OSError, ValueError):
            output = {}
        if output.get("source_plan_sha256") != digest:
            output = artifact("dual_arm_validation")
        output.update(
            status="REPLAY_ERROR",
            simulation_only=True,
            executable=False,
            source_plan_sha256=digest,
            inputs={plan_file: digest},
            failures=[{"type": type(error).__name__, "message": str(error)}],
        )
        write_json(path, output)
        stage_update(
            Path(run), "validation", "failed", artifact=validation_name, message=str(error)
        )
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, default=Path("runs/palace_15cm_dual"))
    parser.add_argument("--config", type=Path, default=Path("configs/cell/two_flexiv.yaml"))
    parser.add_argument("--maximum-seconds", type=float, default=300)
    parser.add_argument("--plan-name", default="dual_arm_plan")
    parser.add_argument("--workers", type=int, default=1, help="Parallel interval checks")
    parser.add_argument(
        "--accept-geometry-proofs",
        action="store_true",
        help="Accept planning's speed and source-coverage proofs for the exact saved clocks; "
        "collision, orientation, deposits and structure are still checked in full",
    )
    parser.add_argument(
        "--accept-pen-proofs",
        action="store_true",
        help="Accept planning's proof that the pen clears the cell and the preceding wire, for "
        "actions whose exact clock and preceding chain match; the arm is still checked on "
        "every edge, against itself, the cell and a volume enclosing all wire",
    )
    args = parser.parse_args()
    result = replay_cell(
        args.run,
        args.config,
        args.maximum_seconds,
        args.plan_name,
        workers=args.workers,
        geometry_proofs=args.accept_geometry_proofs,
        pen_proofs=args.accept_pen_proofs,
    )
    print(result["status"], result["completed_strokes"], result["checked_intervals"])
    return 0 if result["status"] == "CERTIFIED_GEOMETRIC_REPLAY" else 1


if __name__ == "__main__":
    raise SystemExit(main())
