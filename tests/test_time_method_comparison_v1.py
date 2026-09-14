from collections import Counter
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from aic_transfuser_lite.data.time_outward_balanced_v1 import OutwardBalancedMixDataset
from aic_transfuser_lite.data.time_recovery_training_v1 import MatchedRecoveryMixDataset, RecoveryMixDataset
from aic_transfuser_lite.evaluation.time_method_selection_v1 import pp_agreement_score, select_epoch, validation_partitions


def test_comparison_cannot_change_historical_inference_batch_order():
    assert validation_partitions(['a', 'new', 'b', 'a', 'new'], ['a', 'b'], ['new']) == ([0, 2, 3], [1, 4])


@pytest.mark.parametrize('selected,compared', [(['a'], ['a']), (['a'], ['new', 'missing']), ([], ['a'])])
def test_comparison_rejects_leaking_missing_or_incomplete_roles(selected, compared):
    with pytest.raises(ValueError, match='validation roles'):
        validation_partitions(['a', 'new'], selected, compared)


def mixed():
    from test_time_recovery_matched_mix_v1 import setup
    full, reference = setup()
    reference = RecoveryMixDataset(reference.dataset, ['recovery_train'], repeats=12)
    return MatchedRecoveryMixDataset(full, reference, ['recovery_train', 'new_recovery'], seed=42)


def test_state_balance_preserves_data_nominal_positions_and_budget():
    reference = mixed()
    result = OutwardBalancedMixDataset(reference, ['recovery_train', 'new_recovery'], ['old', 'new_a'],
        target_fraction=.5, seed=42)
    assert len(result) == len(reference)
    assert set(result.indices) == set(reference.indices)
    assert result.audit['target_presentations'] == 6
    assert result.audit['target_run_presentations'] == {'recovery_train': 3, 'new_recovery': 3}
    assert result.audit['nominal_slots_preserved']
    for i, aid in enumerate(reference.anchor_ids):
        if aid.startswith('nominal'):
            assert result.anchor_ids[i] == aid
    assert result.dataset is reference.dataset
    again = OutwardBalancedMixDataset(reference, ['recovery_train', 'new_recovery'], ['old', 'new_a'],
        target_fraction=.5, seed=42)
    assert result.indices == again.indices


@pytest.mark.parametrize('targets', [['validation_anchor'], ['nominal_a'], ['old', 'old'], []])
def test_state_balance_rejects_leaked_absent_or_duplicate_targets(targets):
    with pytest.raises(ValueError):
        OutwardBalancedMixDataset(mixed(), ['recovery_train', 'new_recovery'], targets,
            target_fraction=.5, seed=42)


@pytest.mark.parametrize('fraction', [0., 1., float('nan'), .01, .99])
def test_state_balance_rejects_invalid_or_insufficient_budget(fraction):
    with pytest.raises(ValueError):
        OutwardBalancedMixDataset(mixed(), ['recovery_train', 'new_recovery'], ['old', 'new_a'],
            target_fraction=fraction, seed=42)


def pp(angle=0., accepted=True):
    return {'applicable': True, 'accepted': accepted, 'reason': 'PP_ACCEPTED' if accepted else 'REJECTED',
            'steer_rad': angle}


def test_selection_penalizes_rejection_without_dropping_denominator_and_weights_runs():
    truth = [pp(.2), pp(.2), pp(-.2)]
    prediction = [pp(.1), pp(accepted=False), pp(-.2)]
    score = pp_agreement_score(truth, prediction, ['a', 'a', 'b'])
    assert score['teacher_supported'] == 3
    assert score['candidate_rejected'] == 1
    assert score['run_macro_penalized_rad'] == pytest.approx(.175)
    assert score['per_run']['a']['paired_mean_rad'] == pytest.approx(.1)
    perfect = pp_agreement_score(truth, truth, ['a', 'a', 'b'])
    assert perfect['run_macro_penalized_rad'] == 0.


def test_selection_excludes_only_teacher_support_and_checks_physical_units():
    score = pp_agreement_score([pp(accepted=False), pp(.1)], [pp(), pp(.2)], ['a', 'a'])
    assert score['teacher_supported'] == 1
    assert score['run_macro_penalized_rad'] == pytest.approx(.1)
    with pytest.raises(ValueError, match='physical tire'):
        pp_agreement_score([pp()], [pp(1.)], ['a'])
    with pytest.raises(ValueError):
        pp_agreement_score([pp()], [], ['a'])


def test_two_selection_policies_can_choose_different_epochs_with_fixed_support():
    rows = [{'epoch': 1, 'xy_3s_m': .2, 'pp': pp_agreement_score([pp()], [pp(.01)], ['a'])},
            {'epoch': 2, 'xy_3s_m': .1, 'pp': pp_agreement_score([pp()], [pp(.1)], ['a'])}]
    assert select_epoch(rows, 'endpoint_3s') == 2
    assert select_epoch(rows, 'teacher_pp') == 1
    changed = deepcopy(rows)
    changed[1]['pp']['teacher_supported'] = 2
    with pytest.raises(ValueError, match='denominator'):
        select_epoch(changed, 'teacher_pp')


def test_zero_support_cannot_select_pp_epoch():
    empty = pp_agreement_score([pp(accepted=False)], [pp()], ['a'])
    with pytest.raises(ValueError, match='finite support'):
        select_epoch([{'epoch': 1, 'xy_3s_m': .1, 'pp': empty}], 'teacher_pp')


def test_retaining_epoch_checkpoints_preserves_training_predictions_and_resume(tmp_path):
    from test_time_pipeline_p1 import setup, samples
    from aic_transfuser_lite.data.time_split_v1 import content_sha256
    from aic_transfuser_lite.training.time_checkpoint_v1 import TimeCheckpointIdentity
    from aic_transfuser_lite.training.time_corpus_runner_v1 import CorpusTrainingPlan, run_training_arm
    manifest, config = setup(tmp_path)
    train_id = next(r['run_id'] for r in manifest['runs'] if r['split'] == 'train')
    val_id = next(r['run_id'] for r in manifest['runs'] if r['split'] == 'validation')
    train, validation = samples(train_id, config), samples(val_id, config)
    teacher = {'format': 'epoch_retention_test'}
    teacher['manifest_sha256'] = content_sha256(teacher)
    identity = TimeCheckpointIdentity(manifest['manifest_sha256'], teacher['manifest_sha256'], 'c'*64, 'SCRATCH')
    kwargs = dict(train_run_ids=[train_id]*len(train), validation_run_ids=[val_id]*len(validation),
        split_manifest=manifest, teacher_manifest=teacher, identity=identity, config=config,
        plan=CorpusTrainingPlan(epochs=2, batch_size=2, workers=0, precision='float32'), device='cpu')
    plain = run_training_arm(train, validation, output=tmp_path/'plain', **kwargs)
    retained = run_training_arm(train, validation, output=tmp_path/'retained', retain_epoch_checkpoints=True, **kwargs)
    assert plain['optimizer_steps'] == retained['optimizer_steps']
    assert plain['initial_weights_sha256'] == retained['initial_weights_sha256']
    assert not list((tmp_path/'plain').glob('epoch_*.pt'))
    for epoch in (1, 2):
        np.testing.assert_array_equal(np.load(tmp_path/'plain'/f'validation_epoch_{epoch:02d}.npy'),
                                      np.load(tmp_path/'retained'/f'validation_epoch_{epoch:02d}.npy'))
        payload = torch.load(tmp_path/'retained'/f'epoch_{epoch:02d}.pt', weights_only=False)
        assert payload['epoch'] == epoch and payload['next_anchor_index'] == 0
    with pytest.raises(ValueError, match='resume plan differs'):
        run_training_arm(train, validation, output=tmp_path/'retained', resume=True, **kwargs)
    resumed = run_training_arm(train, validation, output=tmp_path/'retained', resume=True,
                               retain_epoch_checkpoints=True, **kwargs)
    assert resumed['optimizer_steps'] == retained['optimizer_steps']
