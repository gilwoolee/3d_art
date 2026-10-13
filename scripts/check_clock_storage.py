"""Verify lossless packed storage on an existing complete viewer plan."""

import argparse
from pathlib import Path
from time import perf_counter

from pen_art.io import read_json, sha256, write_json
from pen_art.planning.clock_store import load_plan, plain, write_plan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    started = perf_counter()
    original = read_json(args.plan)
    json_load_s = perf_counter() - started
    path = args.output / args.plan.name
    started = perf_counter()
    write_plan(path, original)
    export_s = perf_counter() - started
    started = perf_counter()
    packed, plan_hash = load_plan(path)
    packed_load_s = perf_counter() - started
    assert set(original) - {'packed_clock_binding'} == set(packed) - {'packed_clock_binding'}
    for key in original.keys() - {'actions', 'packed_clock_binding'}:
        assert original[key] == packed[key]
    intervals = 0
    for before, after in zip(original['actions'], packed['actions'], strict=True):
        for key in before.keys() - {'trajectory'}:
            assert before[key] == after[key]
        first, second = before['trajectory'], after['trajectory']
        assert {k: v for k, v in first.items() if k != 'edges'} == {
            k: v for k, v in second.items() if k != 'edges'}
        for edge, stored in zip(first['edges'], second['edges'], strict=True):
            assert edge == plain(stored)
            intervals += 1
    write_json(args.output / 'receipt.json', {
        'status': 'LOSSLESS_CLOCK_STORAGE_VERIFIED', 'source_plan_sha256': sha256(args.plan),
        'viewer_plan_sha256': plan_hash, 'actions': len(original['actions']),
        'intervals': intervals, 'source_json_bytes': args.plan.stat().st_size,
        'compact_json_bytes': path.stat().st_size,
        'packed_bytes': sum(p.stat().st_size for p in args.output.glob('*.npy')),
        'manifest_bytes': path.with_suffix('.clocks.json').stat().st_size,
        'original_json_load_s': json_load_s, 'packed_load_s': packed_load_s,
        'export_s': export_s, 'exact_numeric_and_metadata_parity': True,
    })


if __name__ == '__main__':
    main()
