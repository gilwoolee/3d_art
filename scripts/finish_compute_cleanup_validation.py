"""Finish C6 checks after the controlled scaling command leaves the host idle."""

import argparse
import os
import subprocess
import sys
from pathlib import Path
from time import sleep

from pen_art.io import read_json, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--benchmark-pid', type=int, required=True)
    parser.add_argument('--prefix', required=True)
    parser.add_argument('--receipt', type=Path,
                        default=Path('runs/compute_cleanup_c6_validation_v2.json'))
    parser.add_argument('--viewer-run', help='Fresh accepted codec run to browser-check')
    args = parser.parse_args()
    output = args.receipt
    if output.exists():
        parser.error('Choose a fresh validation receipt')
    receipt = {'status': 'WAITING_FOR_SCALING', 'benchmark_pid': args.benchmark_pid,
               'benchmark_prefix': args.prefix, 'steps': []}
    write_json(output, receipt)
    benchmark = Path(args.prefix + '_benchmark.json')
    while not benchmark.exists():
        process = Path(f'/proc/{args.benchmark_pid}/cmdline')
        if not process.exists() or args.prefix not in process.read_text():
            receipt.update(status='FAILED', failure='Scaling stopped before its final receipt')
            write_json(output, receipt)
            return 1
        sleep(2)
    scaling = read_json(benchmark)
    if not scaling['all_complete']:
        receipt.update(status='FAILED', failure='Full scaling acceptance did not pass')
        write_json(output, receipt)
        return 1
    run = args.prefix + '_w4'
    viewer_run = args.viewer_run or run
    commands = [
        ('blocked_and_cold', [sys.executable, 'scripts/probe_parallel_blocked_material.py',
                             '--run', run, '--profile', 'runs/cube_nozzle27_phase1_v1/profile.yaml',
                             '--action-index', '20', '--predecessor-index', '19',
                             '--output', 'runs/compute_cleanup_blocked_cold_v1.json',
                             '--cold-proposal']),
        ('browser_replay', ['tools/node/bin/node', 'web/scripts/check-replay.mjs',
                           Path(viewer_run).name, viewer_run + '/viewer_browser']),
    ]
    environment = {**os.environ, **dict.fromkeys(
        ['OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'], '1')}
    environment.pop('DISPLAY', None)
    for name, command in commands:
        receipt.update(status='RUNNING', active_step=name)
        write_json(output, receipt)
        with Path(f'runs/compute_cleanup_{name}_v1.log').open('w') as stream:
            result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT,
                                    env=environment, check=False)
        receipt['steps'].append({'name': name, 'command': command, 'exit_code': result.returncode})
        if result.returncode:
            receipt.update(status='FAILED', failure=name)
            write_json(output, receipt)
            return 1
    receipt.update(status='PASSED', active_step=None)
    write_json(output, receipt)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
