"""Continuation preserves optimizer bytes, finite bounds, and validated incumbents."""
import copy
import json

import pytest

from tools.mppi_cma.continuation_state import (
    EvaluationPause, campaign_view, check_admission, clone_checkpoint, study_path,
)


def test_admit_whole_batch_before_deadline_and_pause_without_resetting_counts():
    check_admission(60, 4, 64, 100., 101.)
    for values, reason in [((60, 4, 64, 101., 101.), 'time_budget'),
                           ((61, 4, 64, 100., 101.), 'evaluation_budget')]:
        with pytest.raises(EvaluationPause, match=reason):
            check_admission(*values)
    with pytest.raises(EvaluationPause, match='operator_stop'):
        check_admission(0, 4, 64, 100., 101., True)
    for now, deadline in [(float('nan'), 101.), (100., float('inf'))]:
        with pytest.raises(ValueError, match='finite'):
            check_admission(0, 4, 64, now, deadline)
    with pytest.raises(ValueError, match='counts'):
        check_admission(0, 0, 64, 100., 101.)


def test_clone_preserves_source_and_refuses_pending_or_changed_destinations(tmp_path):
    source = tmp_path/'source'; source.mkdir()
    (source/'optimizer.pkl').write_bytes(b'opaque-distribution-and-rng-state')
    (source/'optimizer_status.json').write_text(json.dumps({'generation': 3, 'pending': False}))
    (source/'optimizer_config.json').write_text('{}')
    (source/'seed_anchors.json').write_text('[0]')
    before = {p.name:p.read_bytes() for p in source.iterdir()}
    destination = tmp_path/'next'
    hashes = clone_checkpoint(source, destination, 3)
    assert len(hashes) == 4
    assert {p.name:p.read_bytes() for p in destination.iterdir()} == before
    assert {p.name:p.read_bytes() for p in source.iterdir()} == before
    assert clone_checkpoint(source, destination, 3) == hashes
    (destination/'optimizer.pkl').write_bytes(b'advanced')
    with pytest.raises(FileExistsError):
        clone_checkpoint(source, destination, 3)
    with pytest.raises(ValueError, match='committed'):
        clone_checkpoint(source, tmp_path/'wrong-generation', 4)
    (source/'optimizer_status.json').write_text(json.dumps({'generation': 3, 'pending': True}))
    with pytest.raises(ValueError, match='committed'):
        clone_checkpoint(source, tmp_path/'pending', 3)


def test_study_paths_reject_traversal_and_absolute_paths(tmp_path):
    assert study_path(tmp_path, 'continuous/round-000') == tmp_path/'continuous/round-000'
    for value in ['../previous', '/tmp/other', 'continuous/../refinement', 'continuous//round', '']:
        with pytest.raises(ValueError):
            study_path(tmp_path, value)


def test_campaign_keeps_validated_incumbent_while_next_round_is_unvalidated():
    incumbent = {'selected': {'reference': 'known-good.csv'}, 'evaluations': [], 'comparison': {'promoted': False}}
    previous = {'conditions': {'normal': copy.deepcopy(incumbent), 'leader': copy.deepcopy(incumbent)}}
    journal = {'maximum_rounds': 6, 'completed': False, 'phase': 'search', 'initial_state': previous}
    first = {'new_episodes_started': 64, 'mode': 'independent', 'conditions': {}}
    for name in previous['conditions']:
        first['conditions'][name] = {'evaluations': [{'name': 'first'}], 'next_generation': 6,
            'selected': {'reference': name+'-validated.csv'}, 'best': {}, 'incumbent': {},
            'comparison': {'promoted': True}, 'comparison_runs': {}, 'comparison_dense': {},
            'validated_preferred_feasible': True}
    second = {'new_episodes_started': 4, 'phase': 'search', 'mode': 'shared_course',
              'active_episodes': ['c3-r01-normal-g06-b0'], 'conditions': {
        name:{'evaluations': [{'name': 'second'}], 'next_generation': 6, 'best': {'reference': 'unvalidated.csv'}}
        for name in previous['conditions']}}
    before = copy.deepcopy((journal, previous, first, second))
    view = campaign_view(journal, previous, [first, second])
    assert view['new_episodes_started'] == 68 and view['maximum_new_episodes'] == 384
    assert view['conditions']['normal']['selected']['reference'] == 'normal-validated.csv'
    assert len(view['conditions']['normal']['evaluations']) == 2
    assert view['active_episodes'] == ['c3-r01-normal-g06-b0']
    assert (journal, previous, first, second) == before
    first['conditions']['normal']['validated_preferred_feasible'] = False
    with pytest.raises(RuntimeError, match='validation failed'):
        campaign_view(journal, previous, [first])
