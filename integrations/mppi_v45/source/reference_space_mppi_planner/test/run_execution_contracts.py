#!/usr/bin/env python3
"""Build production core regression tests without ROS or GTest.

The test harness comes from this package; --source-root can point at the
unmodified ZIP current/ to demonstrate the same ten legacy cases before a fix.
This does NOT build the ROS node, occupancy-grid implementation, or old GTests.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time

REL = Path('aichallenge/workspace/src/aichallenge_submit')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--suite', choices=('legacy', 'full'), default='full')
    parser.add_argument('--compiler', default=os.environ.get('CXX', 'g++'))
    parser.add_argument('--sanitize', action='store_true')
    parser.add_argument('--expected-failures', type=int, default=0)
    args = parser.parse_args()
    submit = args.source_root.resolve() / REL
    package = submit / 'reference_space_mppi_planner'
    harness = Path(__file__).resolve().with_name('test_execution_contract.cpp')
    sources = [harness, package / 'src/reference_space_mppi.cpp', package / 'src/batch_optimizer.cpp']
    for source in sources:
        if not source.is_file():
            parser.error(f'Missing source: {source}')
    compiler = shutil.which(args.compiler)
    if compiler is None:
        parser.error(f'Compiler not found: {args.compiler}')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    binary = output / 'execution_contracts'
    command = [compiler, '-std=c++17', '-pthread', '-Wall', '-Wextra', '-Werror']
    command += ['-O1', '-g', '-fsanitize=address,undefined', '-fno-omit-frame-pointer'] if args.sanitize else ['-O2']
    if args.suite == 'full':
        command += ['-DMPPI_EXECUTION_CONTRACT_V2']
    command += [f'-I{package / "include"}', f'-I{submit / "cma_pure_pursuit/include"}']
    command += [str(p) for p in sources] + ['-o', str(binary)]
    metadata = {'compile_command': command, 'suite': args.suite,
                'platform': platform.platform(), 'python': sys.version,
                'compiler': subprocess.check_output([compiler, '--version'], text=True).splitlines()[0],
                'source_root': str(args.source_root.resolve()), 'sanitize': args.sanitize,
                'expected_failures': args.expected_failures,
                'source_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}}
    try:
        build = subprocess.run(command, capture_output=True, text=True, timeout=180)
        (output / 'build.log').write_text(build.stdout + build.stderr)
        metadata['compile_exit_code'] = build.returncode
        if build.returncode:
            return build.returncode
        started = time.perf_counter()
        run = subprocess.run([str(binary), str(output / 'cases.json')],
                             capture_output=True, text=True, timeout=120)
        metadata['suite_elapsed_sec'] = time.perf_counter() - started
        metadata['run_exit_code'] = run.returncode
        (output / 'run.log').write_text(run.stdout + run.stderr)
        print(run.stdout, end='')
        print(run.stderr, end='', file=sys.stderr)
        if not (output / 'cases.json').is_file():
            metadata['error'] = 'No result JSON; process may have crashed.'
            return 2
        cases = json.loads((output / 'cases.json').read_text())
        failed = sum(not case['passed'] for case in cases)
        metadata.update(total=len(cases), failed=failed, passed=len(cases)-failed)
        good = failed == args.expected_failures and run.returncode == (1 if failed else 0)
        metadata['expectation_matched'] = good
        return 0 if good else 1
    except (subprocess.TimeoutExpired, OSError, ValueError) as exc:
        metadata['error'] = str(exc)
        print(str(exc), file=sys.stderr)
        return 2
    finally:
        (output / 'metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')


if __name__ == '__main__':
    raise SystemExit(main())
