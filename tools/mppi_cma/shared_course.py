"""Run two finite four-car ghost checks without reopening the completed CMA study."""
from __future__ import annotations

import fcntl
import argparse
import json
from pathlib import Path
import time

from analyze_episode import analyze
from run_episode import ROOT, run_episode
from verify_tracks import verify


def save(path: Path, value: dict) -> None:
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2)); temporary.replace(path)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--same-start', action='store_true')
    args = parser.parse_args()
    lock = (ROOT / 'search.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    directory = ROOT / ('shared_course_same_start' if args.same_start else 'shared_course')
    directory.mkdir(exist_ok=True)
    path = directory / 'state.json'
    if path.exists():
        raise RuntimeError('This finite shared-course check already exists; inspect its results before a new run')
    state = {'mode': 'shared_course', 'completed': False, 'phase': 'validation',
             'started_unix_s': time.time(), 'deadline_unix_s': time.time() + 1200,
             'new_episodes_started': 0, 'maximum_new_episodes': 2,
             'conditions': {}, 'active_episodes': [], 'ignore_other_vehicles': True,
             'start_mode': 'd1' if args.same_start else 'grid'}
    save(path, state)
    try:
        for condition, target in [('normal', 10.), ('leader', 7.5)]:
            if time.time() > state['deadline_unix_s']:
                raise RuntimeError('Finite shared-course deadline reached')
            name = f"shared-{'d1-' if args.same_start else ''}{condition}-ghost01"
            state.update(active_episode=name, active_episodes=[name])
            state['new_episodes_started'] += 1
            save(path, state)
            output = run_episode(name, handicap=condition == 'leader',
                                 reference=ROOT / 'deliverables' / f'{condition}_reference.csv',
                                 target_mps=target, vehicle_count=4, ghost=True,
                                 ignore_other_vehicles=True, same_start=args.same_start)
            metrics = [analyze(output, vehicle=number) for number in range(1, 5)]
            dense = [verify(ROOT, name, vehicle=number) for number in range(1, 5)]
            state['conditions'][condition] = {'evaluations': [{'metrics': m} for m in metrics],
                                              'dense_checks': dense}
            state['last_episode'] = name
            state['active_episodes'] = []; state.pop('active_episode', None)
            save(path, state)
        state.update(completed=True, phase='complete')
        save(path, state)
    except Exception as exc:
        state.update(phase='error', last_error=str(exc), active_episodes=[])
        state.pop('active_episode', None)
        save(path, state)
        raise


if __name__ == '__main__':
    main()
