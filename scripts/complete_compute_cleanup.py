"""Apply the prepared codec fix after scaling, then run the remaining C6 gates."""

import argparse
import os
import subprocess
import sys
from pathlib import Path
from shutil import copyfile
from time import sleep

from pen_art.io import read_json, sha256, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--benchmark-pid', type=int, required=True)
    parser.add_argument('--prefix', required=True)
    parser.add_argument('--receipt', type=Path,
                        default=Path('runs/compute_cleanup_completion_v2.json'))
    args = parser.parse_args()
    output = args.receipt
    if output.exists():
        parser.error('Choose a fresh completion receipt')
    replacements = {
        Path('src/pen_art/planning/clock_store.py'):
            Path('runs/compute_cleanup_clock_store_candidate_v2.py'),
        Path('tests/test_clock_store.py'):
            Path('runs/compute_cleanup_clock_tests_candidate_v2.py'),
        Path('src/pen_art/planning/cell_replay.py'):
            Path('runs/compute_cleanup_cell_replay_candidate_v2.py'),
    }
    hashes = {str(path): sha256(path) for path in replacements}
    candidates = {str(candidate): sha256(candidate) for candidate in replacements.values()}
    receipt = {'status': 'WAITING_FOR_SCALING', 'source_hashes': hashes,
               'candidate_hashes': candidates, 'benchmark_prefix': args.prefix, 'steps': []}
    write_json(output, receipt)
    benchmark = Path(args.prefix + '_benchmark.json')
    while not benchmark.exists():
        process = Path(f'/proc/{args.benchmark_pid}/cmdline')
        if not process.exists() or args.prefix not in process.read_text():
            receipt.update(status='FAILED', failure='Scaling stopped before its receipt')
            write_json(output, receipt)
            return 1
        sleep(2)
    if not read_json(benchmark)['all_complete']:
        receipt.update(status='FAILED', failure='Full scaling acceptance failed')
        write_json(output, receipt)
        return 1
    for path, candidate in replacements.items():
        if sha256(path) != hashes[str(path)] or sha256(candidate) != candidates[str(candidate)]:
            receipt.update(status='FAILED', failure=f'Concurrent change: {path}')
            write_json(output, receipt)
            return 1
    for path, candidate in replacements.items():
        copyfile(path, Path('runs') / ('compute_cleanup_before_' + path.name))
        temporary = path.with_suffix('.c2.tmp')
        copyfile(candidate, temporary)
        temporary.replace(path)
    # The full-file storage checker allows only the new format binding as extra metadata.
    checker = Path('scripts/check_clock_storage.py')
    text = checker.read_text().replace(
        'assert set(original) == set(packed)',
        "assert set(original) - {'packed_clock_binding'} == "
        "set(packed) - {'packed_clock_binding'}",
    ).replace("original.keys() - {'actions'}",
              "original.keys() - {'actions', 'packed_clock_binding'}")
    checker.write_text(text)
    receipt.update(status='RUNNING', active_step='critical_suite')
    write_json(output, receipt)
    commands = [
        ('python_tests', [sys.executable, '-m', 'pytest', '-q']),
        ('browser_tests', ['tools/node/bin/node', '--test', 'web/tests/timing.test.mjs',
                           'web/tests/tool-render.test.mjs']),
        ('ruff', ['.venv/bin/ruff', 'check', 'src', 'tests']),
        ('diff', ['git', 'diff', '--check']),
        ('exact_codec_replay', [sys.executable, 'scripts/check_saved_clock_codec.py',
                               '--benchmark', str(benchmark),
                               '--output', 'runs/compute_cleanup_cube_codec_v2',
                               '--profile', 'runs/cube_nozzle27_phase1_v1/profile.yaml']),
        ('remaining_c6', [sys.executable, 'scripts/finish_compute_cleanup_validation.py',
                          '--benchmark-pid', str(args.benchmark_pid), '--prefix', args.prefix,
                          '--receipt', 'runs/compute_cleanup_c6_validation_v2.json',
                          '--viewer-run', 'runs/compute_cleanup_cube_codec_v2_w4']),
    ]
    environment = {**os.environ, **dict.fromkeys(
        ['OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'], '1')}
    environment.pop('DISPLAY', None)
    for name, command in commands:
        receipt.update(active_step=name)
        write_json(output, receipt)
        with Path(f'runs/compute_cleanup_finish_{name}_v1.log').open('w') as stream:
            result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT,
                                    env=environment, check=False)
        receipt['steps'].append({'name': name, 'command': command, 'exit_code': result.returncode})
        if result.returncode:
            receipt.update(status='FAILED', failure=name)
            write_json(output, receipt)
            return 1
    receipt.update(status='PASSED', active_step=None,
                   final_source_hashes={str(path): sha256(path) for path in replacements})
    write_json(output, receipt)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
