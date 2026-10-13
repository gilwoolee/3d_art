"""Run higher CPU-worker scaling after the current acceptance sequence passes."""

import argparse
import os
import subprocess
import sys
from pathlib import Path
from time import sleep

from pen_art.io import read_json, sha256, write_json


def inputs(source, profile, library):
    paths = [*Path('src').rglob('*.py'), *source.glob('*.json'), profile,
             *library.joinpath('motions').glob('*.json')]
    return {str(path): sha256(path) for path in sorted(paths)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--completion-pid', type=int, required=True)
    parser.add_argument('--completion', type=Path,
                        default=Path('runs/compute_cleanup_completion_v2.json'))
    parser.add_argument('--output', required=True)
    parser.add_argument('--workers', type=int, nargs='+', default=[4, 8, 12, 16])
    args = parser.parse_args()
    receipt_path = Path(args.output + '_queue.json')
    if receipt_path.exists():
        parser.error('Choose a fresh output prefix')
    source = Path('runs/cube_nozzle27_phase1_v1_source')
    profile = Path('runs/cube_nozzle27_phase1_v1/profile.yaml')
    library = Path('runs/motion_library_phase1_upright27_v1')
    command = [sys.executable, 'scripts/benchmark_parallel_build.py',
               '--source-run', str(source), '--profile', str(profile),
               '--motion-library', str(library), '--output', args.output,
               '--workers', *map(str, args.workers)]
    receipt = {'status': 'WAITING_FOR_ACCEPTANCE', 'completion': str(args.completion),
               'completion_pid': args.completion_pid, 'command': command,
               'scope': 'Sequential CPU scaling with source/profile/library input guards'}
    write_json(receipt_path, receipt)
    while True:
        acceptance = read_json(args.completion)
        if acceptance['status'] == 'PASSED':
            break
        if acceptance['status'] == 'FAILED':
            receipt.update(status='HELD', reason='Acceptance failed', acceptance=acceptance)
            write_json(receipt_path, receipt)
            return 1
        process = Path(f'/proc/{args.completion_pid}/cmdline')
        if (not process.exists() or 'scripts/complete_compute_cleanup.py'
                not in process.read_text()):
            receipt.update(status='HELD', reason='Acceptance process stopped before passing')
            write_json(receipt_path, receipt)
            return 1
        sleep(2)
    before = inputs(source, profile, library)
    receipt.update(status='RUNNING', input_hashes=before)
    write_json(receipt_path, receipt)
    environment = {**os.environ, **dict.fromkeys(
        ['OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'], '1')}
    with Path(args.output + '.log').open('w') as stream:
        result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT,
                                env=environment, check=False)
    after = inputs(source, profile, library)
    matched = before == after
    receipt.update(status='PASSED' if result.returncode == 0 and matched else 'FAILED',
                   exit_code=result.returncode, inputs_unchanged=matched,
                   benchmark=args.output + '_benchmark.json')
    write_json(receipt_path, receipt)
    return 0 if receipt['status'] == 'PASSED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
