"""Finite refinement: 24 candidates and eight comparisons for each speed condition."""
from __future__ import annotations

import fcntl
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import re

from analyze_episode import analyze
from geometry import generate_reference
from parallel_dispatch import dispatch
from refinement_selection import compare
from run_episode import ROOT, run_episode
from run_shared_candidates import run_shared_candidates
from verify_tracks import verify
from continuation_state import EvaluationPause, check_admission, clone_checkpoint, study_path


def save(path: Path, value: object) -> None:
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2)); temporary.replace(path)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--study-subdir', default='refinement')
    parser.add_argument('--previous-subdir', default='search')
    parser.add_argument('--episode-prefix', default='r2')
    parser.add_argument('--continue-optimizer', action='store_true')
    parser.add_argument('--deadline-unix-s', type=float)
    parser.add_argument('--stop-file')
    parser.add_argument('--resume-budget', action='store_true')
    args = parser.parse_args()
    if not re.fullmatch(r'[a-z][a-z0-9-]*', args.episode_prefix):
        raise ValueError('Episode prefix must use lowercase letters, digits and hyphens')
    study = study_path(ROOT, args.study_subdir)
    previous_directory = study_path(ROOT, args.previous_subdir)
    if study == previous_directory:
        raise ValueError('The preceding study must be preserved in a different directory')
    stop_file = study_path(ROOT, args.stop_file) if args.stop_file else None
    lock = (ROOT / 'search.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    study.mkdir(parents=True, exist_ok=True)
    path = study / 'state.json'
    previous = json.loads((previous_directory / 'state.json').read_text())
    if not previous['completed']:
        raise RuntimeError('The preceding CMA search must be complete')
    if path.exists():
        state = json.loads(path.read_text())
        if state['completed']:
            print('Refinement already completed', flush=True); return 0
        if state.get('last_error'):
            raise RuntimeError('Inspect the recorded error before resuming this refinement')
        if args.resume_budget:
            if args.deadline_unix_s is None:
                raise ValueError('Resuming a budget requires an explicit finite deadline')
            state['deadline_unix_s'] = args.deadline_unix_s
            state.pop('pause_reason', None)
            save(path, state)
    else:
        state = {'study_kind': 'refinement', 'mode': 'independent', 'start_mode': 'd1',
                 'phase': 'preflight', 'completed': False, 'started_unix_s': time.time(),
                 'deadline_unix_s': args.deadline_unix_s if args.deadline_unix_s is not None else time.time() + 5400,
                 'maximum_new_episodes': 64,
                 'new_episodes_started': 0, 'candidate_limit_per_condition': 24,
                 'parallel_workers': 4, 'active_episodes': [], 'conditions': {},
                 'optimizer_image_id': previous['optimizer_image_id'],
                 'previous_search_sha256': hashlib.sha256((previous_directory / 'state.json').read_bytes()).hexdigest(),
                 'previous_study': args.previous_subdir, 'checkpoint_source_hashes': {}}
        for condition in ('normal', 'leader'):
            preceding = previous['conditions'][condition]
            incumbent = preceding.get('selected', preceding['best'])
            start_generation = preceding['next_generation'] if args.continue_optimizer else 0
            state['conditions'][condition] = {'incumbent': incumbent, 'next_generation': start_generation,
                                             'starting_generation': start_generation,
                                             'generation_limit': start_generation + 3, 'evaluations': []}
            directory = study / condition; directory.mkdir(exist_ok=True)
            if args.continue_optimizer:
                state['checkpoint_source_hashes'][condition] = clone_checkpoint(
                    previous_directory / condition, directory, start_generation)
            else:
                save(directory / 'seed_anchors.json', incumbent['anchors_m'])
                save(directory / 'optimizer_config.json', {'sigma_m': .16, 'seed': 202609120 + (condition == 'leader')})
        save(path, state)

    def reserve(count: int) -> None:
        check_admission(state['new_episodes_started'], count, state['maximum_new_episodes'],
                        time.time(), state['deadline_unix_s'], bool(stop_file and stop_file.exists()))
        state['new_episodes_started'] += count

    def optimizer(directory: Path, command: str) -> None:
        subprocess.run(['docker', 'run', '--rm', '--network', 'none', '--user', f'{os.getuid()}:{os.getgid()}',
                        '-v', str(ROOT) + ':/work', '--entrypoint', 'python3', state['optimizer_image_id'],
                        '/work/tools/cma_refine_step.py', '/work/' + str(directory.relative_to(ROOT)), command],
                       check=True, timeout=45)

    def measurements(condition: str, jobs: list[dict], group_name: str) -> list[dict]:
        if condition == 'normal':
            state['mode'] = 'shared_course'
            output = ROOT / 'episodes' / group_name
            if not (output / 'runtime_result.json').exists():
                if output.exists():
                    raise RuntimeError('Incomplete shared candidate group: ' + group_name)
                reserve(4)
                state.update(active_episode=group_name, active_episodes=[group_name]); save(path, state)
                run_shared_candidates(group_name, [Path(job['reference']) for job in jobs])
            state['last_episode'] = group_name
            measured = []
            for number, job in enumerate(jobs, 1):
                result = analyze(output, vehicle=number)
                result.update(candidate_id=job['name'], output_episode=group_name)
                measured.append(result)
            state['active_episodes'] = []; state.pop('active_episode', None); save(path, state)
            return measured

        state['mode'] = 'independent'; state.pop('last_episode', None)
        cached = {}; pending = []
        for job in jobs:
            output = ROOT / 'episodes' / job['name']
            if (output / 'runtime_result.json').exists():
                cached[job['name']] = analyze(output)
            elif output.exists():
                raise RuntimeError('Incomplete rank-1 candidate: ' + job['name'])
            else:
                pending.append(job)

        if pending:
            reserve(len(pending)); save(path, state)

        def started(_index: int, job: dict) -> None:
            state['active_episodes'].append(job['name'])
            state['active_episode'] = state['active_episodes'][0]; save(path, state)

        def run(job: dict) -> dict:
            output = run_episode(job['name'], handicap=True, target_mps=7.5, reference=Path(job['reference']))
            return analyze(output)

        def finished(_index: int, job: dict, result: dict | Exception) -> None:
            state['active_episodes'].remove(job['name'])
            if state['active_episodes']:
                state['active_episode'] = state['active_episodes'][0]
            else:
                state.pop('active_episode', None)
            if isinstance(result, Exception):
                state['last_error'] = {'candidate': job['name'], 'error': str(result)}
            else:
                cached[job['name']] = result
            save(path, state)

        dispatch(pending, run, started, finished, 4)
        return [cached[job['name']] for job in jobs]

    try:
        for condition, progress in state['conditions'].items():
            if progress.get('comparison'):
                continue
            directory = study / condition
            if not (directory / 'optimizer.pkl').exists():
                optimizer(directory, 'init')
            for generation in range(progress['next_generation'], progress.get('generation_limit', 3)):
                proposal_path = directory / f'generation_{generation:02d}.json'
                if not proposal_path.exists():
                    optimizer(directory, 'ask')
                    save(proposal_path, json.loads((directory / 'proposals.json').read_text()))
                proposals = json.loads(proposal_path.read_text())
                jobs = []
                for index, anchors in enumerate(proposals):
                    name = f'{args.episode_prefix}-{condition}-g{generation:02d}-c{index:02d}'
                    reference = directory / name / 'reference.csv'
                    if not reference.exists():
                        generate_reference(ROOT / 'snapshot/base_reference.csv', reference, anchors)
                    jobs.append({'name': name, 'reference': str(reference), 'anchors_m': anchors})
                state.update(phase='search', current_condition=condition, generation=generation + 1); save(path, state)
                ordered = []
                for batch in range(2):
                    subset = jobs[batch * 4:(batch + 1) * 4]
                    measured = measurements(condition, subset, f'{args.episode_prefix}-{condition}-g{generation:02d}-b{batch}')
                    for job, result in zip(subset, measured):
                        ordered.append(result)
                        if not any(record['name'] == job['name'] for record in progress['evaluations']):
                            progress['evaluations'].append({**job, 'metrics': result})
                        print(json.dumps({'candidate': job['name'], 'metrics': result}), flush=True)
                    save(path, state)
                save(directory / 'evaluated.json', {'generation': generation, 'candidates': proposals,
                                                   'objectives': [m['objective'] for m in ordered]})
                optimizer(directory, 'tell')
                progress['next_generation'] = generation + 1; save(path, state)
            feasible = [record for record in progress['evaluations'] if record['metrics']['preferred_feasible']]
            best = min(feasible, key=lambda r: r['metrics']['objective']) if feasible else progress['incumbent']
            progress['best'] = best
            state.update(phase='validation', current_condition=condition); save(path, state)
            collected = {'candidate': [], 'incumbent': []}; dense = {'candidate': [], 'incumbent': []}
            for batch, roles in enumerate([['candidate', 'incumbent', 'candidate', 'incumbent'],
                                           ['incumbent', 'candidate', 'incumbent', 'candidate']]):
                jobs = [{'name': f'{args.episode_prefix}-{condition}-validate-b{batch}-d{number}', 'role': role,
                         'reference': (best if role == 'candidate' else progress['incumbent'])['reference']}
                        for number, role in enumerate(roles, 1)]
                group = f'{args.episode_prefix}-{condition}-validate-b{batch}'
                measured = measurements(condition, jobs, group)
                for number, (job, result) in enumerate(zip(jobs, measured), 1):
                    collected[job['role']].append(result)
                    dense[job['role']].append(verify(ROOT, group, vehicle=number) if condition == 'normal'
                                              else verify(ROOT, job['name']))
                progress['comparison_runs'] = collected; progress['comparison_dense'] = dense; save(path, state)
            decision = compare(collected['candidate'], collected['incumbent'], dense['candidate'], dense['incumbent'])
            progress.update(comparison=decision, selected=best if decision['promoted'] else progress['incumbent'],
                            validated_preferred_feasible=decision['candidate_validated'] if decision['promoted']
                            else decision['incumbent_validated'])
            print(json.dumps({'condition': condition, 'comparison': decision}), flush=True); save(path, state)
        state.update(completed=True, phase='complete', active_episodes=[])
        state.pop('active_episode', None); save(path, state)
        print('FINITE REFINEMENT COMPLETE', flush=True)
        return 0
    except EvaluationPause as exc:
        state.update(phase='paused', pause_reason=str(exc), active_episodes=[])
        state.pop('active_episode', None); save(path, state)
        print('CHECKPOINTED PAUSE: ' + str(exc), flush=True)
        return 3
    except Exception as exc:
        state.update(phase='error', last_error=str(exc), active_episodes=[])
        state.pop('active_episode', None); save(path, state)
        raise


if __name__ == '__main__':
    raise SystemExit(main())
