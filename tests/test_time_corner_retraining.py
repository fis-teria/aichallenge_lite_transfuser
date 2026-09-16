from collections import Counter
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from train_time_corner_recovery import build_sampler, require_completed_collection, execution_plan
from test_time_recovery_matched_mix_v1 import setup


def test_expanded_budget_keeps_all_teachers_and_nominal_once():
    full, _ = setup()
    sampler = build_sampler(full, ['recovery_train', 'new_recovery'], ['old', 'new_a'],
                            repeats=2, fraction=.5, seed=42)
    assert set(sampler.anchor_ids) == set(full.anchor_ids)
    counts = Counter(sampler.anchor_ids)
    assert counts['nominal_a'] == counts['nominal_b'] == 1
    assert sampler.recovery_presentations == 6
    assert sampler.unique_recovery_anchors == 3
    assert len(sampler) == 8


@pytest.mark.parametrize('target', ['validation_anchor', 'nominal_a'])
def test_expanded_sampler_rejects_nonrecovery_or_validation_targets(target):
    full, _ = setup()
    with pytest.raises(ValueError):
        build_sampler(full, ['recovery_train', 'new_recovery'], [target], repeats=2, fraction=.5, seed=42)


@pytest.mark.parametrize('field,value', [('result_status', 'FAILED'), ('fault', 'STOPPING_SWEEP_OCCUPIED'),
    ('stop_confirmed', False), ('closed_bag', False)])
def test_unfinished_or_faulted_recording_is_not_adopted(field, value):
    summary = dict(result_status='COMPLETE_LAP', fault=None, stop_confirmed=True, closed_bag=True)
    require_completed_collection(summary)
    summary[field] = value
    with pytest.raises(ValueError, match='completed lap'):
        require_completed_collection(summary)


def test_serial_loading_preserves_batch_precision_budget_and_original_plan():
    from dataclasses import asdict
    plan = dict(training=dict(epochs=3,batch_size=32,workers=4,seed=42,learning_rate=3e-5,precision='float32'))
    original = asdict(execution_plan(plan,None))
    serial = asdict(execution_plan(plan,0))
    assert {k for k in original if original[k] != serial[k]} == {'workers'}
    assert serial['workers'] == 0 and plan['training']['workers'] == 4
    with pytest.raises(ValueError):
        execution_plan(plan,-1)
