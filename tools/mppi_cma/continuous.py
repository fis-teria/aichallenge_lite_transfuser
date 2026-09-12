"""Run checkpoint-continuing CMA rounds automatically under one finite campaign."""
from __future__ import annotations

import argparse
import copy
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from continuation_state import campaign_view
from run_episode import ROOT


def save(path: Path, value: dict) -> None:
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2)); temporary.replace(path)


def source_manifest() -> dict[str, str]:
    paths = [ROOT/'refinement/state.json']
    paths += [ROOT/f'refinement/{condition}/{name}' for condition in ('normal', 'leader')
              for name in ('optimizer.pkl', 'optimizer_status.json', 'optimizer_config.json', 'seed_anchors.json')]
    return {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--hours', type=float, default=3.)
    parser.add_argument('--max-rounds', type=int, default=6)
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args()
    if not math.isfinite(args.hours) or not 0 < args.hours <= 24:
        raise ValueError('The campaign requires a finite duration in (0, 24] hours')
    if not 1 <= args.max_rounds <= 32:
        raise ValueError('The campaign requires 1 to 32 rounds')
    directory = ROOT/'continuous'; directory.mkdir(exist_ok=True)
    lock = (directory/'supervisor.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    path = directory/'journal.json'
    stop_file = directory/'stop_requested'
    if path.exists():
        journal = json.loads(path.read_text())
        if not args.resume:
            raise RuntimeError('Campaign already exists; use --resume with an explicit new finite budget')
        if journal.get('last_error'):
            raise RuntimeError('Inspect the recorded error before resuming; no automatic retry is permitted')
        if journal.get('child_pid'):
            command = Path(f"/proc/{journal['child_pid']}/cmdline")
            if command.exists() and b'/tools/refine.py' in command.read_bytes():
                raise RuntimeError('The preceding evaluation child is still running')
        if args.max_rounds <= len(journal['completed_rounds']):
            raise ValueError('Increase the total round ceiling when resuming a finished campaign')
        journal.update(completed=False, phase='preflight', deadline_unix_s=time.time()+args.hours*3600,
                       maximum_rounds=args.max_rounds, pause_reason=None)
        if stop_file.exists():
            stop_file.unlink()
    else:
        previous = json.loads((ROOT/'refinement/state.json').read_text())
        if not previous['completed'] or not all(p['validated_preferred_feasible'] for p in previous['conditions'].values()):
            raise RuntimeError('The preceding refinement and both selected-route validations must be complete')
        journal = {'completed': False, 'phase': 'preflight', 'started_unix_s': time.time(),
                   'deadline_unix_s': time.time()+args.hours*3600, 'maximum_rounds': args.max_rounds,
                   'completed_rounds': [], 'round_subdirs': [], 'round_index': 0,
                   'initial_state': previous, 'initial_source_sha256': source_manifest(),
                   'last_error': None, 'child_pid': None, 'exports': []}
    if source_manifest() != journal['initial_source_sha256']:
        raise RuntimeError('The source refinement changed; preserve and inspect it before continuation')
    save(path, journal)

    def stop(_signum: int, _frame: object) -> None:
        stop_file.touch(exist_ok=True)

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    def publish() -> None:
        rounds = [json.loads((ROOT/subdir/'state.json').read_text()) for subdir in journal['round_subdirs']
                  if (ROOT/subdir/'state.json').exists()]
        journal['heartbeat_unix_s'] = time.time()
        save(path, journal)
        save(directory/'state.json', campaign_view(journal, journal['initial_state'], rounds))

    child = None
    try:
        publish()
        while len(journal['completed_rounds']) < journal['maximum_rounds']:
            if stop_file.exists() or time.time() >= journal['deadline_unix_s']:
                journal.update(completed=True, phase='paused', pause_reason='operator_stop' if stop_file.exists() else 'time_budget')
                break
            number = len(journal['completed_rounds'])
            subdir = f'continuous/round-{number:03d}'
            preceding = journal['completed_rounds'][-1] if journal['completed_rounds'] else 'refinement'
            if subdir not in journal['round_subdirs']:
                journal['round_subdirs'].append(subdir)
            journal.update(round_index=number+1, phase='search')
            command = [sys.executable, '-u', str(ROOT/'tools/refine.py'), '--study-subdir', subdir,
                       '--previous-subdir', preceding, '--episode-prefix', f'c3-r{number:02d}',
                       '--continue-optimizer', '--deadline-unix-s', str(journal['deadline_unix_s']),
                       '--stop-file', 'continuous/stop_requested']
            if (ROOT/subdir/'state.json').exists():
                command.append('--resume-budget')
            save(directory/'active_command.json', {'command': command, 'previous_study': preceding})
            with (directory/f'round-{number:03d}.log').open('a') as log:
                child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                journal['child_pid'] = child.pid
                while child.poll() is None:
                    publish(); time.sleep(3)
                code = child.returncode
            journal['child_pid'] = None
            if code == 3:
                state = json.loads((ROOT/subdir/'state.json').read_text())
                journal.update(completed=True, phase='paused', pause_reason=state['pause_reason'])
                break
            if code:
                raise RuntimeError(f'Round {number+1} failed with exit code {code}; see {subdir} and its log')
            state = json.loads((ROOT/subdir/'state.json').read_text())
            if not state['completed'] or not all(p['validated_preferred_feasible'] for p in state['conditions'].values()):
                raise RuntimeError('Round validation did not close successfully')
            if source_manifest() != journal['initial_source_sha256']:
                raise RuntimeError('The preceding refinement was changed during continuation')
            output = f'continuous/results/round-{number:03d}'
            subprocess.run([sys.executable, str(ROOT/'tools/report_refinement.py'),
                            '--state-subdir', subdir, '--output-subdir', output], check=True, timeout=60)
            journal['completed_rounds'].append(subdir)
            journal['exports'].append(output)
            save(directory/'latest_result.json', {'output': output, 'round': number+1,
                                                 'comparisons': {k:v['comparison'] for k,v in state['conditions'].items()}})
            publish()
            print(json.dumps({'round_completed': number+1, 'next_generation': {
                k:v['next_generation'] for k,v in state['conditions'].items()}, 'output': output}), flush=True)
        else:
            journal.update(completed=True, phase='complete', pause_reason='round_budget')
        journal['finished_unix_s'] = time.time()
        journal['source_preserved'] = source_manifest() == journal['initial_source_sha256']
        if not journal['source_preserved']:
            raise RuntimeError('Source checkpoint preservation failed')
        publish()
        print('CONTINUOUS CAMPAIGN CHECKPOINTED: '+str(journal.get('pause_reason')), flush=True)
    except Exception as exc:
        stop_file.touch(exist_ok=True)
        # Let the current bounded batch drain before exiting; no new work is admitted.
        if child is not None and child.poll() is None:
            child.wait(timeout=600)
        journal.update(completed=True, phase='error', last_error=str(exc), child_pid=None,
                       finished_unix_s=time.time())
        save(path, journal)
        current = json.loads((directory/'state.json').read_text()) if (directory/'state.json').exists() else copy.deepcopy(journal)
        current.update(completed=True, phase='error', last_error=str(exc), active_episodes=[], active_episode=None)
        save(directory/'state.json', current)
        raise


if __name__ == '__main__':
    main()
