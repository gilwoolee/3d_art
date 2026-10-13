"""Measure strict read-only restoration of a qualified latest forward state."""

import argparse
from pathlib import Path
from time import perf_counter

from pen_art.io import read_json, sha256, write_json
from pen_art.planning.forward_state import restore_forward_state
from pen_art.planning.path_certificates import _canonical
from pen_art.planning.replay_checkpoint import IMPLEMENTATION, digest
from pen_art.planning.section_build import SectionBuild


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--motion-library", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    started = perf_counter()
    build = SectionBuild(args.run, args.profile, 600, motion_library=args.motion_library)
    initialization_s = perf_counter() - started
    checkpoint = read_json(args.run / "forward_checkpoint.json")
    forward_order = read_json(args.run / "forward_order.json")
    assert forward_order["binding"] == checkpoint["binding"] == build.checkpoint_binding
    order = forward_order["order"]
    assert set(order) == set(build.actions) and len(order) == len(build.actions)
    snapshot_path = args.run / "forward_state_checkpoint.json"
    snapshot = read_json(snapshot_path)
    started = perf_counter()
    restored = restore_forward_state(
        snapshot_path,
        build.forward_snapshot_context(),
        order,
        checkpoint["witnesses"],
        witness_root=args.run,
    )
    restore_s = perf_counter() - started
    if restored is None:
        raise ValueError("Qualified forward state was declined; no prefix checks were skipped")
    state = restored["state"]
    actual = _canonical(
        {
            "time_s": state.time_s,
            "configurations": state.configurations,
            "completed": sorted(state.completed),
            "support_ready": state.support_ready,
            "table_index": state.table_index,
            "material_segments": state.payload._physical_segments,
            "material_geometry_sha256": state.payload.geometry_prefix()[0].hex(),
        }
    )
    assert actual == snapshot["state"]
    assert restored["snapshot_sha256"] == snapshot["checksum"]
    report = {
        "status": "EXACT_QUALIFIED_FORWARD_STATE_RESTORED",
        "run": str(args.run),
        "implementation": IMPLEMENTATION,
        "full_source_actions": len(order),
        "restored_actions": len(restored["witnesses"]),
        "snapshot_sha256": restored["snapshot_sha256"],
        "material_sha256": digest(actual["material_segments"]),
        "state_sha256": digest(actual),
        "material_capsules": len(state.payload.objects),
        "exact_saved_state_parity": True,
        "restore_s": restore_s,
        "initialization_s": initialization_s,
        "collision_intervals_revalidated_by_probe": 0,
        "prefix_witnesses_rehashed": len(restored["witnesses"]),
        "profile_sha256": sha256(args.profile),
        "command_script_sha256": sha256(Path(__file__)),
        "scope": "Read-only exact prefix restoration; suffix planning and final replay "
        "remain required",
    }
    write_json(args.output, report)
    print(report)


if __name__ == "__main__":
    main()
