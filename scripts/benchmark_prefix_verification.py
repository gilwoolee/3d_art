"""Compare raw, cold, and warm verification on exact checked prefixes.

Every pass reconstructs and checks material from time zero. Output must be new:
the cold accelerated pass may never silently inherit a certificate store.
"""

import argparse
import re
import shutil
from pathlib import Path
from time import monotonic

from pen_art.io import read_json, sha256, write_json
from pen_art.planning.cell_replay import _replay_cell
from pen_art.planning.partial_export import export_forward_prefix
from pen_art.planning.replay_checkpoint import IMPLEMENTATION, digest

ARTIFACT_SUFFIXES = {".json", ".yaml", ".urdf", ".glb"}
DEFAULT_CASES = [
    ("cube", "runs/cube_nozzle27_full_v3", "runs/cube_nozzle27_phase1_v1"),
    ("bottle", "runs/bottle_nozzle27_full_v1", "runs/bottle_nozzle27_phase1_v1"),
]


def copy_artifacts(source, destination):
    destination.mkdir()
    for path in source.iterdir():
        if path.is_file() and path.suffix in ARTIFACT_SUFFIXES:
            shutil.copy2(path, destination / path.name)


def prepare_case(output, name, original, profile_run, sections):
    source, run = output / f"{name}_source_prefix{sections}", output / f"{name}_prefix{sections}"
    checkpoint = read_json(original / "forward_checkpoint.json")
    retained = len(checkpoint["witnesses"])
    if retained < sections:
        raise ValueError(f"{name} has only {retained} checked sections; requested {sections}")
    copy_artifacts(original, source)
    copy_artifacts(profile_run, run)
    checkpoint["witnesses"] = checkpoint["witnesses"][:sections]
    write_json(source / "forward_checkpoint.json", checkpoint)
    for reference in checkpoint["witnesses"]:
        relative = Path(reference["file"])
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError("Unsafe witness path")
        (source / relative).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(original / relative, source / relative)
    plan = export_forward_prefix(source, run)
    return run, {
        "original_checked_prefix_actions": retained,
        "selected_actions": sections,
        "required_fullobject_actions": plan["requested_strokes"],
        "source_run": str(original),
        "plan_sha256": sha256(run / "offline_dual_arm_plan.json"),
        "input_hashes": plan["inputs"],
        "profile_sha256": sha256(profile_run / "profile.yaml"),
        "modes": {},
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="New, nonexistent directory")
    parser.add_argument("--sections", type=int, default=2, help="Explicit checked prefix length")
    parser.add_argument("--maximum-seconds", type=float, default=180, help="Per replay pass")
    parser.add_argument(
        "--case",
        nargs=3,
        action="append",
        metavar=("NAME", "SOURCE_RUN", "PROFILE_RUN"),
        help="Repeat for multiple objects; defaults to the current cube and bottle runs",
    )
    args = parser.parse_args()
    if args.sections < 1 or args.maximum_seconds <= 0:
        parser.error("Sections and per-pass seconds must be positive")
    cases = args.case or DEFAULT_CASES
    if any(not re.fullmatch(r"[A-Za-z0-9_-]+", case[0]) for case in cases):
        parser.error("Case names must contain only letters, digits, underscores, or hyphens")
    if len({case[0] for case in cases}) != len(cases):
        parser.error("Case names must be unique")
    args.output.mkdir(exist_ok=False)
    report = {
        "status": "RUNNING",
        "implementation": IMPLEMENTATION,
        "input_script_sha256": sha256(Path(__file__)),
        "scope": f"Explicit first {args.sections} verified actions per object; unchanged full "
        "source graph and exact clocks; material reconstructed and checked fresh every pass",
        "maximum_seconds_per_pass": args.maximum_seconds,
        "objects": {},
    }

    def save():
        write_json(args.output / "benchmark.json", report)

    save()
    try:
        for name, source, profile in cases:
            original, profile_run = Path(source), Path(profile)
            run, case = prepare_case(args.output, name, original, profile_run, args.sections)
            report["objects"][name] = case
            save()
            for mode, raw in [("raw", True), ("cold_fast", False), ("warm_fast", False)]:
                print("START", name, mode, flush=True)
                started = monotonic()
                audit, state = _replay_cell(
                    run,
                    profile_run / "profile.yaml",
                    args.maximum_seconds,
                    "offline_dual_arm_plan",
                    return_state=True,
                    verification_raw=raw,
                    reuse_material_checkpoint=False,
                )
                result = {
                    "wall_seconds": monotonic() - started,
                    "status": audit["status"],
                    "completed_strokes": audit["completed_strokes"],
                    "checked_edges": audit["checked_edges"],
                    "reused_path_checks": audit["reused_path_checks"],
                    "material_capsules": len(state.payload.segments),
                    "material_sha256": digest(state.payload.segments),
                    "state_sha256": digest(
                        {
                            "q": state.configurations,
                            "completed": sorted(state.completed),
                            "ready": state.support_ready,
                            "time": state.time_s,
                            "table": state.table_index,
                        }
                    ),
                    "verification": audit.get("verification"),
                    "verification_raw": audit["verification_raw"],
                    "material_checkpoint_used": audit["reuse_material_checkpoint"],
                    "failures": audit["failures"],
                }
                case["modes"][mode] = result
                write_json(args.output / f"{name}_{mode}_audit.json", audit)
                save()
                print(
                    "DONE",
                    name,
                    mode,
                    round(result["wall_seconds"], 3),
                    result["status"],
                    result["material_capsules"],
                    flush=True,
                )
                if audit["status"] != "CERTIFIED_PREFIX_REPLAY":
                    report["status"] = "BLOCKED_REPLAY"
                    save()
                    return 1
            case["parity"] = {
                key: len({result[key] for result in case["modes"].values()}) == 1
                for key in ("status", "completed_strokes", "material_sha256", "state_sha256")
            }
            case["warm_speedup_vs_raw"] = (
                case["modes"]["raw"]["wall_seconds"] / case["modes"]["warm_fast"]["wall_seconds"]
            )
            if not all(case["parity"].values()):
                report["status"] = "PARITY_REJECTED"
                save()
                return 1
            save()
    except Exception as error:
        report.update(status="ERROR", error={"type": type(error).__name__, "message": str(error)})
        save()
        raise
    report["status"] = "BENCHMARK_COMPLETE"
    save()
    print("BENCHMARK_COMPLETE", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
