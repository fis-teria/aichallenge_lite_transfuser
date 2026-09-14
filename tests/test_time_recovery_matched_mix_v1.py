from collections import Counter
from types import SimpleNamespace

import pytest

from aic_transfuser_lite.data.time_recovery_training_v1 import (
    MatchedRecoveryMixDataset, RecoveryMixDataset, recovery_extension)
from test_time_recovery_training_v1 import extension


def setup():
    old_split = extension()
    new_split = recovery_extension(old_split['base_manifest'], old_split['additional_runs'] + [
        {'run_id': 'new_recovery', 'split': 'train', 'speed_cap_kmh': 5,
         'sources': [{'path': 'runs/new_recovery/bag/a.db3', 'sha256': 'c' * 64}]}])
    nominal = next(r['run_id'] for r in old_split['runs'] if r['split'] == 'train' and r['run_id'] != 'recovery_train')
    old = SimpleNamespace(split_manifest=old_split, run_ids=[nominal, 'recovery_train', nominal],
                          anchor_ids=['nominal_a', 'old', 'nominal_b'])
    new = SimpleNamespace(split_manifest=new_split,
        run_ids=[nominal, 'recovery_train', nominal, 'new_recovery', 'new_recovery'],
        anchor_ids=['nominal_a', 'old', 'nominal_b', 'new_a', 'new_b'])
    return new, RecoveryMixDataset(old, ['recovery_train'], repeats=4)


def test_matching_keeps_nominal_slots_budget_and_covers_all_recovery_anchors():
    new, reference = setup()
    mixed = MatchedRecoveryMixDataset(new, reference, ['recovery_train', 'new_recovery'], seed=42)
    assert len(mixed) == len(reference) == 6
    assert mixed.anchor_ids[0] == 'nominal_a' and mixed.anchor_ids[-1] == 'nominal_b'
    counts = Counter(mixed.anchor_ids[1:-1])
    assert set(counts) == {'old', 'new_a', 'new_b'} and sorted(counts.values()) == [1, 1, 2]
    assert mixed.recovery_presentations == 4 and mixed.unique_recovery_anchors == 3
    assert MatchedRecoveryMixDataset(new, reference, ['recovery_train', 'new_recovery'], seed=42).indices == mixed.indices


@pytest.mark.parametrize('mutation', ['validation', 'duplicate', 'drop', 'nominal', 'run_identity', 'budget'])
def test_matching_rejects_population_or_split_drift(mutation):
    new, reference = setup()
    if mutation == 'validation':
        new.run_ids[-1] = 'recovery_val'
    elif mutation == 'duplicate':
        new.anchor_ids[-1] = new.anchor_ids[0]
    elif mutation == 'drop':
        new.anchor_ids[1] = 'replacement'
    elif mutation == 'nominal':
        new.run_ids[-1] = new.run_ids[0]
    elif mutation == 'run_identity':
        new.run_ids[1] = 'new_recovery'
    else:
        reference = RecoveryMixDataset(reference.dataset, ['recovery_train'], repeats=1)
    with pytest.raises(ValueError):
        MatchedRecoveryMixDataset(new, reference, ['recovery_train', 'new_recovery'], seed=42)


@pytest.mark.parametrize('seed', [-1, True, 1.5])
def test_matching_rejects_invalid_seed(seed):
    new, reference = setup()
    with pytest.raises(ValueError, match='seed'):
        MatchedRecoveryMixDataset(new, reference, ['recovery_train', 'new_recovery'], seed=seed)
