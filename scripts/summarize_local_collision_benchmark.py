"""Publish comparisons only for matching fixtures and full final checks."""

import argparse
from pathlib import Path

from pen_art.io import read_json, sha256, write_json

FULL_SCOPE = "full robot, full pen, plate/static and actual growing material"


def contains_timeout(value):
    if isinstance(value, dict):
        return value.get("status") == "TIME_BUDGET" or any(
            contains_timeout(item) for item in value.values()
        )
    if isinstance(value, (tuple, list)):
        return any(contains_timeout(item) for item in value)
    return False


def compare(baseline, candidate):
    for key in (
        "fixture_sha256",
        "material_sha256",
        "clock_sha256",
        "section",
        "initial_capsules",
        "blocked",
    ):
        if baseline[key] != candidate[key]:
            raise ValueError(f"Comparison changed {key}")
    for record in (baseline, candidate):
        if record["final_scope"] != FULL_SCOPE:
            raise ValueError("A full final robot check is required")
        if contains_timeout(record.get("failure")) or contains_timeout(
            record.get("candidate_clock_failure")
        ):
            raise ValueError("A timeout cannot establish verdict parity")
    if baseline["accepted"] != candidate["accepted"]:
        raise ValueError("Full final verdicts differ")
    if baseline["accepted"]:
        for key in ("final_material_sha256", "final_capsules"):
            if baseline[key] != candidate[key]:
                raise ValueError("Accepted checks deposited different material")
    if candidate["candidate_scope"] == "pen_candidate":
        if candidate["candidate_clock_accepted"] != candidate["accepted"]:
            raise ValueError("Pen candidate did not match the expected fixture verdict")
        if (
            candidate["accepted"]
            and candidate["candidate_clock_material_sha256"] != candidate["final_material_sha256"]
        ):
            raise ValueError("Pen candidate deposited different material from full final")
    return {
        "same_inputs": True,
        "full_final_verdict_parity": True,
        "accepted_material_parity": baseline["accepted"],
        "candidate_clock_speedup": baseline["candidate_clock_seconds"]
        / candidate["candidate_clock_seconds"],
        "full_final_speedup": baseline["final_seconds"] / candidate["final_seconds"],
        "baseline_clock_work_seconds": baseline["final_seconds"],
        "optimized_clock_work_seconds": candidate["final_seconds"]
        + (
            candidate["candidate_clock_seconds"]
            if candidate["candidate_scope"] == "pen_candidate"
            else 0
        ),
        "scope": "Baseline candidate Clock is the full final check measured once; "
        "optimized pen candidate and full final checks are separate actual work",
    }


def summarize(directory):
    records, bindings = {}, {}
    for name in ("cube", "bottle"):
        modes = {}
        for mode in ("baseline", "scene_only", "pen_candidate"):
            cases = {}
            for case in ("cold", "warm", "blocked"):
                path = directory / f"{name}_{mode}" / f"{case}.json"
                if path.exists():
                    cases[case] = read_json(path)
                    bindings[str(path)] = sha256(path)
            modes[mode] = cases
        comparisons = {}
        for mode in ("scene_only", "pen_candidate"):
            comparisons[mode] = {
                case: compare(modes["baseline"][case], record)
                for case, record in modes[mode].items()
                if case in modes["baseline"]
            }
        records[name] = {"measurements": modes, "comparisons": comparisons}
    source = directory / "source_freeze.json"
    bindings[str(source)] = sha256(source)
    complete = all(
        all(
            case in records[name]["comparisons"]["pen_candidate"]
            for case in ("cold", "warm", "blocked")
        )
        for name in ("cube", "bottle")
    )
    if complete:
        for name in ("cube", "bottle"):
            if records[name]["measurements"]["pen_candidate"]["blocked"]["accepted"]:
                raise ValueError("An adversarial collision was accepted")
    overlap_path = directory / "adversary_overlap.json"
    if not overlap_path.exists():
        complete = False
        overlaps = None
    else:
        overlaps = read_json(overlap_path)
        bindings[str(overlap_path)] = sha256(overlap_path)
        if overlaps["status"] != "COMPLETE" or len(overlaps["results"]) != 2:
            raise ValueError("Direct collision fixture proof is incomplete")
        for row, name in zip(overlaps["results"], ("cube", "bottle"), strict=True):
            if (
                row["fixture_sha256"]
                != records[name]["measurements"]["baseline"]["cold"]["fixture_sha256"]
                or row["status"] != "WITNESSED_INTERIOR_PEN_COLLISION"
            ):
                raise ValueError("Direct collision proof has changed fixtures")
    return {
        "status": "COMPLETE" if complete else "PARTIAL_MEASUREMENTS",
        "scope": "Two fixed late local actions on large actual material scenes. "
        "Full robot validation remains mandatory; no whole-build speed or completion claim. "
        "Cold preparation uses diagnostic defer=False; production uses defer=True. "
        "Clock comparisons do not depend on regenerated preparation output.",
        "objects": records,
        "adversary_direct_overlap": overlaps,
        "inputs": bindings,
        "source_manifest_sha256": sha256(source),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, required=True)
    args = parser.parse_args()
    write_json(args.directory / "summary.json", summarize(args.directory))


if __name__ == "__main__":
    main()
