"""Checkpoint-preserving continuation state; seconds and vehicle counts are explicit."""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import re
import shutil


class EvaluationPause(RuntimeError):
    """The current budget or an operator stop forbids starting another batch."""


def study_path(root: Path, relative: str) -> Path:
    if not re.fullmatch(r'[a-z][a-z0-9_-]*(?:/[a-z0-9][a-z0-9_-]*)*', relative):
        raise ValueError('Study path must be a relative lowercase directory without traversal')
    path = root / relative
    if not path.resolve().is_relative_to(root.resolve()):
        raise ValueError('Study path leaves the experiment root')
    return path


def check_admission(started: int, count: int, maximum: int, now_s: float,
                    deadline_s: float, stop_requested: bool = False) -> None:
    """Admit a complete batch before launch; an admitted batch is allowed to drain."""
    if any(type(value) is not int or value < 0 for value in (started, count, maximum)) or count == 0:
        raise ValueError('Vehicle counts must be nonnegative integers and batch count positive')
    if not all(math.isfinite(value) for value in (now_s, deadline_s)):
        raise ValueError('Time and deadline must be finite seconds')
    if stop_requested:
        raise EvaluationPause('operator_stop')
    if now_s >= deadline_s:
        raise EvaluationPause('time_budget')
    if started + count > maximum:
        raise EvaluationPause('evaluation_budget')


def clone_checkpoint(source: Path, destination: Path, expected_generation: int) -> dict[str, str]:
    """Copy pycma's distribution and RNG bytes without resetting or loading pickle."""
    status = json.loads((source / 'optimizer_status.json').read_text())
    if status['generation'] != expected_generation or status['pending']:
        raise ValueError('Only a completed, committed CMA generation can seed the next round')
    destination.mkdir(parents=True, exist_ok=True)
    hashes = {}
    for name in ('optimizer.pkl', 'optimizer_status.json', 'optimizer_config.json', 'seed_anchors.json'):
        original = source / name; target = destination / name
        content = original.read_bytes()
        if target.exists() and target.read_bytes() != content:
            raise FileExistsError('Continuation destination contains different checkpoint data: ' + str(target))
        if not target.exists():
            shutil.copy2(original, target)
        hashes[name] = hashlib.sha256(content).hexdigest()
        if hashlib.sha256(target.read_bytes()).hexdigest() != hashes[name]:
            raise RuntimeError('CMA checkpoint transfer failed')
    return hashes


def campaign_view(journal: dict, previous: dict, rounds: list[dict]) -> dict:
    """Expose aggregate progress while retaining only repeatedly validated selections."""
    view = {key: copy.deepcopy(value) for key, value in journal.items() if key != 'initial_state'}
    view.update(study_kind='continuous', start_mode='d1', parallel_workers=4,
                maximum_new_episodes=journal['maximum_rounds'] * 64,
                candidate_limit_per_condition=journal['maximum_rounds'] * 24,
                new_episodes_started=sum(item['new_episodes_started'] for item in rounds),
                conditions=copy.deepcopy(previous['conditions']))
    for progress in view['conditions'].values():
        progress['evaluations'] = []
        progress['validation_history'] = []
    for item in rounds:
        for condition, data in item['conditions'].items():
            progress = view['conditions'][condition]
            progress['evaluations'].extend(copy.deepcopy(data['evaluations']))
            progress['next_generation'] = data['next_generation']
            if data.get('comparison'):
                if not data.get('validated_preferred_feasible'):
                    raise RuntimeError('Incumbent validation failed; automatic continuation is stopped')
                for key in ('selected', 'best', 'incumbent', 'comparison', 'comparison_runs',
                            'comparison_dense', 'validated_preferred_feasible'):
                    progress[key] = copy.deepcopy(data[key])
                progress['validation_history'].append(copy.deepcopy(data['comparison']))
    live = rounds[-1] if rounds else {}
    for key in ('mode', 'active_episode', 'active_episodes', 'current_condition', 'generation', 'last_episode'):
        view[key] = copy.deepcopy(live.get(key, [] if key == 'active_episodes' else None))
    if not journal['completed']:
        view['phase'] = live.get('phase', 'preflight')
    return view
