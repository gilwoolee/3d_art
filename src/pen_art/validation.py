"""Summarize independent evidence without equating simulation and fabrication."""

import argparse
import hashlib
import json
from pathlib import Path

from .io import artifact, now, sha256, stage_update, write_json


def validation_report(run):
    run = Path(run)
    inputs, checks = {}, []

    def load(name):
        if not (run / name).is_file():
            return None
        raw = (run / name).read_bytes()
        value = json.loads(raw)
        inputs[name] = hashlib.sha256(raw).hexdigest()
        stale = False
        for source, digest in value.get("inputs", {}).items():
            if not (run / source).is_file():
                stale = True
            else:
                current = sha256(run / source)
                inputs[source] = current
                stale |= current != digest
        if stale:
            checks.append({"name": name, "status": "STALE", "detail": "Source inputs changed"})
            return None
        return value

    def add(name, status, detail, evidence):
        checks.append({"name": name, "status": status, "detail": detail, "evidence": evidence})

    graph = load("stroke_graph.json")
    if graph:
        passed = graph.get("checks", {}).get("material_connected") is True
        add(
            "Connected material",
            "MODELED_PASS" if passed else "UNRESOLVED",
            f"{len(graph['edges'])} material members",
            "stroke_graph.json",
        )
    order = load("construction_plan.json")
    if order:
        complete = order.get("all_members_covered_once") is True
        grounded = len(order.get("prefixes", [])) == len(order["actions"]) and all(
            row.get("grounded_topology") is True for row in order.get("prefixes", [])
        )
        add(
            "Construction topology",
            "MODELED_PASS" if complete and grounded else "UNRESOLVED",
            "Whole-member coverage and grounded completed-action prefixes; "
            "does not establish hot-strand stability",
            "construction_plan.json",
        )
    structure = load("structural_screen.json")
    if structure:
        add(
            "Structural screening",
            "UNCALIBRATED"
            if not structure.get("calibrated")
            else "MODELED_PASS"
            if structure.get("modeled_pass")
            else "MODELED_FAILURE",
            structure.get("failure") or structure.get("message") or "Cold-frame screening only",
            "structural_screen.json",
        )
    process = load("process_screen.json")
    if process:
        add(
            "Pen process",
            "UNCALIBRATED"
            if not process.get("calibrated")
            else "MODELED_PASS"
            if process.get("modeled_pass")
            else "MODELED_FAILURE",
            "Unknown process limits require pen measurements",
            "process_screen.json",
        )
    motions = load("allocation_motions.json")
    if motions:
        add(
            "Prepared robot motions",
            "MODELED_PASS"
            if motions["prepared_strokes"] == motions["requested_strokes"]
            else "INCOMPLETE",
            f"{motions['prepared_strokes']} / {motions['requested_strokes']} strokes; "
            "shared replay is checked separately",
            "allocation_motions.json",
        )
    plan = load("offline_dual_arm_plan.json")
    replay = load("offline_dual_arm_validation.json")
    replay_status = replay["status"] if replay else "MISSING"
    if replay and (
        not plan or replay.get("source_plan_sha256") != inputs.get("offline_dual_arm_plan.json")
    ):
        replay_status = "STALE"
    passed = replay_status == "CERTIFIED_GEOMETRIC_REPLAY"

    def failure_statuses(value):
        if isinstance(value, dict):
            if "status" in value:
                yield value["status"]
            for child in value.values():
                yield from failure_statuses(child)
        elif isinstance(value, list):
            for child in value:
                yield from failure_statuses(child)

    reasons = set(failure_statuses(replay.get("failures", []))) if replay else set()
    result_status = "MODELED_PASS" if passed else "INCOMPLETE"
    if replay_status == "REPLAY_REJECTED":
        result_status = (
            "MODELED_FAILURE"
            if reasons & {"COLLISION", "INCOMPLETE_OR_NOT_PARKED"}
            else "INCOMPLETE"
            if "TIME_BUDGET" in reasons
            else "UNRESOLVED"
        )
    add(
        "Shared-clock geometric replay",
        result_status,
        replay_status + (": " + ", ".join(sorted(reasons)) if reasons else ""),
        "offline_dual_arm_validation.json" if replay else None,
    )
    add(
        "Physical fabrication",
        "NOT_TESTED",
        "Pen, mounts, placement, hot-strand stability and physical execution remain unverified",
        None,
    )
    result = artifact(
        "validation_report",
        generated_at=now(),
        inputs=inputs,
        simulation_only=True,
        executable=False,
        checks=checks,
        geometry_status="REPLAY_PASSED" if passed else "NOT_VALIDATED",
        physical_status="NOT_TESTED",
        modeled_duration_s=plan.get("modeled_duration_s") if plan else None,
        duration_target_policy="Advisory; exceeding 30 minutes is not a failure",
    )
    write_json(run / "validation_report.json", result)
    if (run / "run.json").is_file():
        stage_update(
            run,
            "validation",
            "complete" if passed else "partial",
            artifact="validation_report.json",
            message="Evidence summary; physical fabrication remains untested",
        )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    result = validation_report(parser.parse_args().run)
    print(result["geometry_status"], result["physical_status"])


if __name__ == "__main__":
    main()
