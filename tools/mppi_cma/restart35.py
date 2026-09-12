"""Remeasure the selected normal route at 35 km/h, then start a finite campaign."""
from __future__ import annotations

import argparse
import copy
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
from statistics import median
import sys
import time

from analyze_episode import analyze
from continuation_state import clone_checkpoint, condition_targets, study_path
from continuous import save, source_manifest
from run_episode import ROOT
from run_shared_candidates import run_shared_candidates
from verify_tracks import verify

NORMAL_TARGET_MPS = 35. / 3.6


def prepare_seed(source: Path, destination: Path, runs: list[dict], dense: list[dict]) -> dict:
    """Preserve old scores; reset normal CMA and retain the committed leader CMA."""
    if len(runs) != 4 or len(dense) != 4:
        raise ValueError('Four normal baseline runs and dense checks are required')
    if not all(row['preferred_feasible'] and not row['handicap_enabled'] and
               math.isclose(row['target_mps'], NORMAL_TARGET_MPS, rel_tol=0, abs_tol=1e-8) and
               math.isfinite(row['command_speed_max_mps']) and
               row['command_speed_max_mps'] <= NORMAL_TARGET_MPS + 1e-6 and
               math.isfinite(row['flying_lap_s']) and row['flying_lap_s'] > 0 for row in runs):
        raise ValueError('The 35 km/h baseline failed feasibility or execution speed checks')
    if not all(row['passed'] for row in dense):
        raise ValueError('The 35 km/h baseline failed dense OT checks')
    previous = json.loads((source/'state.json').read_text())
    if not previous['completed'] or not all(v['validated_preferred_feasible'] for v in previous['conditions'].values()):
        raise ValueError('Both preceding selected routes must be validated')
    if condition_targets(previous)['leader'] != 7.5:
        raise ValueError('This restart preserves the existing 7.5 m/s leader condition')
    destination.mkdir(parents=True, exist_ok=False)
    result = {'completed': True, 'phase': 'complete', 'study_kind': 'refinement',
              'optimizer_image_id': previous['optimizer_image_id'], 'conditions': {},
              'restart_baseline_evaluations': 4, 'new_episodes_started': 4,
              'maximum_new_episodes': 4, 'previous_study': str(source),
              'source_state_sha256': hashlib.sha256((source/'state.json').read_bytes()).hexdigest()}
    for name in ('normal', 'leader'):
        old = previous['conditions'][name]
        selected = copy.deepcopy(old.get('selected') or old['best'])
        folder = destination/name
        folder.mkdir()
        reference = folder/'selected_reference.csv'
        shutil.copy2(selected['reference'], reference)
        selected['reference'] = str(reference)
        if name == 'normal':
            baseline_median = median(row['flying_lap_s'] for row in runs)
            selected['metrics'] = copy.deepcopy(min(runs, key=lambda row: abs(row['flying_lap_s']-baseline_median)))
            result['conditions'][name] = {
                'target_mps': NORMAL_TARGET_MPS, 'selected': selected, 'best': copy.deepcopy(selected),
                'incumbent': copy.deepcopy(selected), 'next_generation': 0, 'evaluations': [],
                'optimizer_restart_required': True, 'validated_preferred_feasible': True,
                'baseline_runs': runs, 'baseline_dense': dense, 'baseline_median_s': baseline_median}
            save(folder/'seed_anchors.json', selected['anchors_m'])
            save(folder/'optimizer_config.json', {'sigma_m': .16, 'seed': 202609130})
        else:
            clone_checkpoint(source/name, folder, old['next_generation'])
            result['conditions'][name] = copy.deepcopy(old)
            result['conditions'][name].update(selected=selected, target_mps=7.5, evaluations=[])
    save(destination/'state.json', result)
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--previous-subdir', default='continuous/round-004')
    parser.add_argument('--campaign-subdir', default='continuous35_20260913')
    parser.add_argument('--episode-prefix', default='c35')
    parser.add_argument('--hours', type=float, default=3.)
    parser.add_argument('--max-rounds', type=int, default=6)
    args = parser.parse_args()
    if not math.isfinite(args.hours) or not 0 < args.hours <= 24 or not 1 <= args.max_rounds <= 32:
        raise ValueError('A finite duration and round ceiling are required')
    if not re.fullmatch(r'[a-z][a-z0-9-]*', args.episode_prefix):
        raise ValueError('Invalid episode prefix')
    source = study_path(ROOT, args.previous_subdir)
    campaign = study_path(ROOT, args.campaign_subdir)
    if source == campaign or source.is_relative_to(campaign):
        raise ValueError('Source study must be preserved outside the new campaign')
    lock = (ROOT/'search.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    previous = json.loads((source/'state.json').read_text())
    if not previous['completed'] or not all(v['validated_preferred_feasible'] for v in previous['conditions'].values()):
        raise ValueError('The preceding selected routes must be validated')
    if condition_targets(previous)['leader'] != 7.5:
        raise ValueError('Unexpected leader speed condition')
    before = source_manifest(args.previous_subdir)
    campaign.mkdir(parents=True, exist_ok=False)
    deadline = time.time() + args.hours * 3600
    name = args.episode_prefix+'-baseline-normal'
    state = {'study_kind': 'continuous', 'mode': 'shared_course', 'start_mode': 'd1',
             'completed': False, 'phase': 'rebaseline', 'round_index': 0, 'completed_rounds': [],
             'started_unix_s': time.time(), 'deadline_unix_s': deadline,
             'new_episodes_started': 4, 'maximum_new_episodes': 4+64*args.max_rounds,
             'candidate_limit_per_condition': 24*args.max_rounds, 'parallel_workers': 4,
             'active_episode': name, 'active_episodes': [name], 'last_error': None,
             'conditions': {'normal': {'target_mps': NORMAL_TARGET_MPS, 'evaluations': []},
                            'leader': {**copy.deepcopy(previous['conditions']['leader']), 'target_mps': 7.5, 'evaluations': []}}}
    save(campaign/'state.json', state)
    save(campaign/'restart_preflight.json', {'source_sha256': before, 'normal_target_mps': NORMAL_TARGET_MPS,
                                          'leader_target_mps': 7.5, 'deadline_unix_s': deadline})
    stop_file = campaign/'stop_requested'

    def stop(_signum: int, _frame: object) -> None:
        stop_file.touch(exist_ok=True)

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        normal = previous['conditions']['normal']['selected']
        output = run_shared_candidates(name, [Path(normal['reference'])]*4, target_mps=NORMAL_TARGET_MPS)
        runs = []
        for number in range(1, 5):
            measured = analyze(output, vehicle=number)
            measured.update(candidate_id=normal['name'], output_episode=name)
            runs.append(measured)
        dense = [verify(ROOT, name, vehicle=number) for number in range(1, 5)]
        if source_manifest(args.previous_subdir) != before:
            raise RuntimeError('The source checkpoint changed during baseline measurement')
        seed = prepare_seed(source, campaign/'seed', runs, dense)
        state.update(conditions=seed['conditions'], phase='preflight', active_episodes=[], last_episode=name)
        state.pop('active_episode', None)
        save(campaign/'state.json', state)
        print(json.dumps({'baseline_median_s': seed['conditions']['normal']['baseline_median_s'],
                          'target_mps': NORMAL_TARGET_MPS, 'source_preserved': True}), flush=True)
        if stop_file.exists() or time.time() >= deadline:
            state.update(completed=True, phase='paused', pause_reason='operator_stop' if stop_file.exists() else 'time_budget')
            save(campaign/'state.json', state)
            return
    except Exception as exc:
        state.update(completed=True, phase='error', last_error=str(exc), active_episodes=[])
        state.pop('active_episode', None)
        save(campaign/'state.json', state)
        raise
    finally:
        lock.close()
    command = [sys.executable, '-u', str(ROOT/'tools/continuous.py'),
               '--campaign-subdir', args.campaign_subdir, '--previous-subdir', args.campaign_subdir+'/seed',
               '--episode-prefix', args.episode_prefix, '--hours', str(args.hours),
               '--max-rounds', str(args.max_rounds), '--deadline-unix-s', str(deadline)]
    save(campaign/'continuation_command.json', command)
    os.execv(sys.executable, command)


if __name__ == '__main__':
    main()
