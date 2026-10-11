"""One bounded support repair at a deposited contour endpoint.

This proposes a permanent vertical post. It does not choose new artistic curves,
search post locations, accept the motion into a build, or certify hot extrusion.
"""

import numpy as np

from ..graph import straight_edge
from ..structure import analyze_structure
from .axis_repair import repair_drawing_axes
from .dual_arm import ScheduleState
from .stroke_restarts import attach_support_to_member, release_screen

SUPPORT_POLICY_REVISION = 2


def plan_endpoint_support(build, source, accepted_index, *, blocked_index, progress=None):
    """Return a prepared post and structural graph, or a specific rejection.

    ``accepted_index`` must be the saved endpoint of an already deposited source
    prefix. ``blocked_index`` is the release point whose failed screen triggered
    this repair. The current prefix and immediate next vertex must pass; the
    distant blocked release is diagnostic. Later releases retain their checks.
    The caller persists/accepts the
    result atomically before resuming. No build state or source artifact changes.
    Existing axis-repair helpers may save their usual bound solver checkpoints.
    """
    planner, arm = build.planner, build.arm
    report = {
        "status": "SUPPORT_NOT_PREPARED",
        "policy_revision": SUPPORT_POLICY_REVISION,
        "source_stroke": source,
        "accepted_index": accepted_index,
        "blocked_index": blocked_index,
        "simulation_only": True,
        "physical_status": "unverified",
    }

    def reject(reason, **evidence):
        return {**report, "status": reason, **evidence}

    if source not in planner.actions:
        return reject("SUPPORT_SOURCE_UNKNOWN")
    if accepted_index <= 0:
        return reject("SUPPORT_REQUIRES_DEPOSITED_PREFIX")
    if build.state["source_progress"].get(source) != accepted_index:
        return reject("SUPPORT_ENDPOINT_NOT_ACCEPTED")
    selected = next((row for row in build.selection if row["stroke_id"] == source), None)
    if selected is None:
        return reject("SUPPORT_SOURCE_NOT_SELECTED")
    drawing = next(
        (
            row for row in planner.candidates["variants"]
            if row["stroke_id"] == source and row["variant"] == selected["variant"]
        ),
        None,
    )
    if drawing is None:
        return reject("SUPPORT_SOURCE_VARIANT_MISSING")
    drawing = build.source_drawing(selected)
    transform = np.asarray(build.transform)
    if not np.allclose(transform[:3, :3], np.eye(3)):
        return reject("SUPPORT_GRAVITY_NOT_BUILD_Z")
    points = np.asarray(drawing["tcp_targets_m"]) - transform[:3, 3]
    if not 0 < accepted_index < blocked_index < len(points):
        return reject("SUPPORT_INVALID_PREFIX_RANGE")
    top = points[accepted_index]
    if not np.isfinite(top).all() or top[2] <= 1e-6:
        return reject("SUPPORT_ENDPOINT_ALREADY_AT_BASE")
    bottom = np.r_[top[:2], 0.0]
    identity = f"auto-support-{source}-{accepted_index}"
    if any(edge["id"] == identity for edge in build.graph["edges"]):
        return reject("SUPPORT_ENDPOINT_ALREADY_BRACED")
    nodes = {
        key: {"id": key, "position": point.tolist(), "ground": ground}
        for key, point, ground in (
            (identity + "-foot", bottom, True),
            (identity + "-top", top, False),
        )
    }
    action = planner.actions[source]
    edge = straight_edge(
        identity + "-foot", identity + "-top", nodes, identity, "support", action["diameter_m"]
    )
    support = {
        **edge,
        "closed": False,
        "members": [identity],
        "junction_visits": [
            {"node": edge["start"], "distance_m": 0, "segment_boundary": 0},
            {"node": edge["end"], "distance_m": edge["length_m"], "segment_boundary": 1},
        ],
    }
    # Resolve previous structural splits to existing graph members. Robot source
    # geometry stays untouched, even if another post split the same beam before.
    existing = {e["id"] for e in build.graph["edges"]}
    members = set()

    def leaves(member, visiting):
        if member in existing:
            members.add(member)
            return
        if member in visiting:
            raise ValueError("Cyclic structural member replacement")
        for child in build.member_replacements.get(member, []):
            leaves(child, visiting | {member})

    for member in action["members"]:
        leaves(member, set())
    try:
        graph, replacements, error = attach_support_to_member(
            build.graph, support, list(nodes.values()), members
        )
    except ValueError as exc:
        return reject("SUPPORT_ATTACHMENT_REJECTED", failure=str(exc))
    report.update(
        action=support,
        nodes=list(nodes.values()),
        graph=graph,
        member_replacements=replacements,
        attachment_approximation_m=error,
    )
    post_screen = analyze_structure(
        {"nodes": list(nodes.values()), "edges": [edge]}, build.material
    )
    report["post_screen"] = post_screen
    if not post_screen["modeled_pass"]:
        return reject("SUPPORT_POST_STRUCTURE_REJECTED")
    printed = build.printed_members()
    for old, new in replacements.items():
        if old in printed:
            printed = (printed - {old}) | set(new)
    screens = {
        index: release_screen(
            graph, build.material, printed | {identity}, points[:index + 1],
            action["diameter_m"], drawing["start_anchor"],
        )
        for index in dict.fromkeys((accepted_index, accepted_index + 1, blocked_index))
    }
    report["contour_screens"] = screens
    required = (accepted_index, accepted_index + 1)
    report["required_release_indices"] = list(required)
    report["distant_release_is_diagnostic"] = True
    if not all(screens[index]["modeled_pass"] for index in required):
        return reject("SUPPORT_DOES_NOT_ENABLE_PROGRESS")

    bottom_world = bottom + transform[:3, 3]
    # One seed selected deterministically from the existing solved catalogue.
    nearest = min(
        (
            (float(np.linalg.norm(np.asarray(point) - bottom_world)), tuple(q))
            for row in planner.candidates["variants"]
            for point, q in zip(row["tcp_targets_m"], row["positions"], strict=True)
        ),
        default=(0.0, tuple(build.q)),
    )
    seed = np.asarray(nearest[1])
    targets = np.linspace(bottom_world, top + transform[:3, 3], 32)
    candidate = {
        "arm": arm.identity,
        "stroke_id": identity,
        "variant": "free-axis-endpoint-post",
        "start_anchor": support["start"],
        "end_anchor": support["end"],
        "positions": np.repeat(seed[None], len(targets), axis=0).tolist(),
        "tcp_targets_m": targets.tolist(),
        "tcp_rotation": arm.robot.forward(seed, arm.flange_to_tcp)[:3, :3].tolist(),
        "release_cooling_s": build.cooling,
    }
    previous = planner.actions.get(identity)
    attempts_start = len(planner.attempts)
    planner.actions[identity] = support
    try:
        repaired = repair_drawing_axes(planner, candidate, build.scene, progress=progress)
        report["axis_repair"] = repaired
        if repaired["status"] != "SAMPLED_FREE_AXIS_PROPOSAL":
            return reject("SUPPORT_POSE_REJECTED")
        candidate.update(repaired)
        state = ScheduleState(
            0, ((arm.identity, tuple(build.q)),), frozenset(), (), payload=build.scene
        )
        move = planner._prepare_drawing(state, arm.identity, candidate, np.asarray(build.q))
        if move is None:
            return reject("SUPPORT_MOTION_REJECTED", failures=planner.attempts[attempts_start:])
        return {
            **report,
            "status": "PREPARED",
            "drawing": candidate,
            "trajectory": move.payload.record(),
            "duration_s": move.duration_s,
        }
    finally:
        if previous is None:
            planner.actions.pop(identity, None)
        else:
            planner.actions[identity] = previous
