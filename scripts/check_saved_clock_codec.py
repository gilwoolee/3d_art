"""Re-export the exact benchmark schedules and independently replay a new codec.

Planning measurements and their original artifacts remain intact. Fresh codec
runs preserve every schedule value and earn their own full-robot acceptance.
"""

import argparse
from pathlib import Path
from shutil import copyfile
from time import perf_counter

from pen_art.io import read_json, sha256, write_json
from pen_art.planning.cell_replay import replay_cell
from pen_art.planning.clock_store import load_plan, plain, write_plan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--benchmark', type=Path, required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--replay-workers', type=int, default=12)
    args = parser.parse_args()
    benchmark = read_json(args.benchmark)
    if not benchmark['all_complete']:
        parser.error('Original full acceptance must complete first')
    receipt_path = Path(args.output + '_codec.json')
    if receipt_path.exists():
        parser.error('Choose a fresh output prefix')
    results = []
    for result in benchmark['runs']:
        source = Path(result['run'])
        run = Path(args.output + f"_w{result['workers']}")
        run.mkdir(parents=True, exist_ok=False)
        for path in source.iterdir():
            if (path.is_file() and not path.name.startswith('offline_dual_arm_plan')
                    and path.name != 'offline_dual_arm_validation.json'):
                copyfile(path, run / path.name)
        metadata = read_json(run / 'run.json')
        metadata.update(id=run.name,
                        name=f"Cube · exact saved schedule · codec · {result['workers']}w",
                        source_benchmark_run=str(source), workbench_primary=False)
        for stage in ('motion', 'timing'):
            metadata['stages'][stage]['status'] = 'pending'
        write_json(run / 'run.json', metadata)
        build = read_json(run / 'section_build.json')
        build['status'] = 'CODEC_REPLAY_RUNNING'
        write_json(run / 'section_build.json', build)
        original_path = source / 'offline_dual_arm_plan.json'
        original = read_json(original_path)
        started = perf_counter()
        path = run / 'offline_dual_arm_plan.json'
        write_plan(path, original)
        packed, plan_hash = load_plan(path)
        assert set(original) == set(packed) - {'packed_clock_binding'}
        for key in original.keys() - {'actions'}:
            assert original[key] == packed[key]
        for before, after in zip(original['actions'], packed['actions'], strict=True):
            assert before == plain(after)
        export_s = perf_counter() - started
        del original, packed
        started = perf_counter()
        replay = replay_cell(run, args.profile, float('inf'), 'offline_dual_arm_plan',
                             workers=args.replay_workers, verification_raw=True,
                             optimize_contact_geometry=True)
        prior = read_json(source / 'offline_dual_arm_validation.json')
        assert replay['status'] == 'CERTIFIED_GEOMETRIC_REPLAY'
        assert replay['collision_scope'] == 'full_robot' and replay['verification_raw']
        for key in ('final_material', 'checked_intervals', 'checked_edges', 'completed_strokes',
                    'requested_strokes', 'final_table_index'):
            assert replay.get(key) == prior.get(key), key
        build.update(status='FULL_BUILD_VERIFIED', codec_original_plan_sha256=sha256(original_path),
                     codec_plan_sha256=plan_hash)
        write_json(run / 'section_build.json', build)
        for stage in ('motion', 'timing'):
            metadata['stages'][stage]['status'] = 'complete'
        write_json(run / 'run.json', metadata)
        results.append({'source_run': str(source), 'run': str(run),
                        'original_plan_sha256': sha256(original_path), 'plan_sha256': plan_hash,
                        'exact_schedule_parity': True, 'export_and_load_s': export_s,
                        'fresh_replay_s': perf_counter() - started,
                        'replay_status': replay['status'],
                        'final_material': replay['final_material']})
        write_json(receipt_path, {'status': 'RUNNING', 'results': results})
        print(run.name, replay['status'], flush=True)
    write_json(receipt_path, {'status': 'PASSED', 'results': results,
                             'scope': 'Identical saved schedules under the corrected codec; '
                                      'original end-to-end timing receipts remain unchanged'})


if __name__ == '__main__':
    main()
