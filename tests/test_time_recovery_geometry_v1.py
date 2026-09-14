from copy import deepcopy
from dataclasses import replace
from pathlib import Path
import subprocess

import numpy as np
import pytest
import torch

from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose
from aic_transfuser_lite.control.time_trial_v1 import time_trial_control
from aic_transfuser_lite.data.time_split_v1 import content_sha256
from aic_transfuser_lite.training.time_corpus_runner_v1 import train_corpus_batch
from aic_transfuser_lite.training.time_recovery_geometry_v1 import (
    RecoveryGeometryObjective, RecoveryGeometryPlan, fixed_time_pp_angle)
from test_time_batched_evaluation_v1 import _manifest, _sample, _Model


def recovery_manifest(manifest=None):
    manifest = deepcopy(manifest or _manifest())
    for row in manifest['runs']:
        row['run_id'] = 'codex-time-recovery-' + row['run_id']
    manifest['manifest_sha256'] = content_sha256({k: v for k, v in manifest.items()
                                               if k not in {'manifest_sha256', 'sources_verified'}})
    return manifest


def objective_context():
    manifest = recovery_manifest()
    run = next(r['run_id'] for r in manifest['runs'] if r['split'] == 'train')
    sample = _sample(run)
    rows = [dict(anchor_id=sample.anchor_id, run_id=run, horizon_s=1.55, response_length_m=1.2)]
    objective = RecoveryGeometryObjective(rows, split_manifest=manifest, plan=RecoveryGeometryPlan(), controller={})
    return sample, objective, manifest


@pytest.mark.parametrize('speed,bend', [(0.7, 0.), (1.25, .03), (1.38, -.05)])
def test_teacher_fixed_time_matches_actual_pp_angle_and_metres_radians(speed, bend):
    t = np.arange(1, 31) * .1
    xy = np.stack([1.4*t, bend*t*t], axis=1).astype(np.float32)
    pose = TimedBodyPose(10**9, 'sim', '0', 'map', 'base_link', 0., 0., 0.)
    offset = RecoveryGeometryPlan().rear_axle_offset_m
    actual = time_trial_control(TimePlan('teacher', pose, xy), pose, speed_mps=speed,
        rear_axle_offset_m=offset, speed_policy='fixed_5kmh', lookahead_policy='stopping_preview_extended_v1',
        vehicle_model_policy='awsim_understeer_v1')
    pred = torch.tensor(xy[None], dtype=torch.float64, requires_grad=True)
    h = torch.tensor([actual['lookahead_selection']['observation_horizon_s']], dtype=torch.float64)
    length = torch.tensor([actual['nominal_response_length_m']], dtype=torch.float64)
    value = fixed_time_pp_angle(pred, h, length, offset)
    assert value.shape == (1,) and value.item() == pytest.approx(actual['steer_rad'], abs=1e-7)
    value.sum().backward()
    assert pred.grad is not None and torch.isfinite(pred.grad).all()
    assert pred.grad.abs().sum() > 0


def test_perfect_teacher_zero_loss_and_far_error_has_correct_units_and_gradient():
    sample, objective, _ = objective_context()
    teacher = torch.tensor(sample.teacher.xy_m[None], requires_grad=True)
    support = torch.ones((1, 30), dtype=torch.bool)
    assert objective(teacher, teacher, support, [sample]).item() == 0
    prediction = teacher.detach().clone()
    prediction[:, 19:, 1] += .1
    prediction.requires_grad_(True)
    # Only 2..3 s changed; the 1.55 s near target remains exact.
    loss = objective(prediction, teacher, support, [sample])
    assert loss.item() == pytest.approx(.05)
    loss.backward()
    assert teacher.grad is None
    assert torch.all(prediction.grad[:, 19:, 1] > 0)
    assert torch.all(prediction.grad[:, :19] == 0)
    assert torch.all(prediction.grad[:, :, 0] == 0)


def test_auxiliary_ignores_nominal_partial_support_and_normalizes_over_all_supported():
    sample, objective, _ = objective_context()
    nominal = replace(_sample('nominal'), anchor_id='nominal')
    model = _Model()
    optimizer = torch.optim.SGD(model.parameters(), lr=.01)
    pred = torch.full((2, 30, 2), .5, requires_grad=True)
    target = torch.ones_like(pred)
    support = torch.ones((2, 30), dtype=torch.bool)
    support[1] = False
    expected_aux = objective(pred, target, support, [sample, nominal])
    expected_aux.backward()
    assert torch.all(pred.grad[1] == 0)
    # The runner reports base loss separately; both supported anchors divide the
    # total gradient, even though only the recovery row has auxiliary supervision.
    p = torch.tensor(.5, requires_grad=True)
    outputs = torch.ones((2, 30, 2))*p
    total = (outputs-target).abs().mean((1, 2)).sum() + objective(
        outputs, target, torch.ones_like(support), [sample, nominal])
    (total/2).backward()
    result = train_corpus_batch(model, [sample, nominal], optimizer, precision='float32',
        max_grad_norm=100., recovery_objective=objective)
    assert result['supported'] == 2 and result['loss_sum_m'] == pytest.approx(1.)
    assert result['auxiliary_loss_sum_m'] == pytest.approx(expected_aux.item())
    assert model.anchor.item() == pytest.approx(.5-.01*p.grad.item())


def test_missing_or_partial_recovery_teacher_is_not_silently_dropped():
    sample, objective, _ = objective_context()
    prediction = torch.ones((1, 30, 2), requires_grad=True)
    support = torch.ones((1, 30), dtype=torch.bool)
    support[0, -1] = False
    with pytest.raises(ValueError, match='complete audited'):
        objective(prediction, prediction, support, [sample])
    with pytest.raises(ValueError, match='missing from frozen'):
        objective(prediction, prediction, support, [replace(sample, anchor_id='unknown')])
    with pytest.raises(ValueError, match='run/input mismatch'):
        objective(prediction, prediction, support, [replace(sample, run='nominal')])
    with pytest.raises(ValueError, match='aligned'):
        objective(prediction[:, :29], prediction, support, [sample])


@pytest.mark.parametrize('split', ['validation', 'test'])
def test_auxiliary_targets_cannot_use_validation_or_sealed_test(split):
    _, _, manifest = objective_context()
    run = next(r['run_id'] for r in manifest['runs'] if r['split'] == split)
    with pytest.raises(ValueError, match='split boundary'):
        RecoveryGeometryObjective([dict(anchor_id='leaked', run_id=run, horizon_s=1.5, response_length_m=1.2)],
            split_manifest=manifest, plan=RecoveryGeometryPlan(), controller={})


@pytest.mark.parametrize('changes', [dict(near_weight_m_per_rad=-1.), dict(far_weight=float('nan')),
    dict(near_weight_m_per_rad=0., far_weight=0.), dict(far_start_s=1.), dict(rear_axle_offset_m=(1., 0.))])
def test_geometry_plan_rejects_invalid_units_and_weights(changes):
    with pytest.raises(ValueError):
        replace(RecoveryGeometryPlan(), **changes).validate()


@pytest.mark.parametrize('horizon,length', [(0., 1.), (3.1, 1.), (1.5, 0.), (float('nan'), 1.)])
def test_fixed_time_pp_rejects_invalid_geometry(horizon, length):
    with pytest.raises(ValueError):
        fixed_time_pp_angle(torch.ones((1, 30, 2)), torch.tensor([horizon]), torch.tensor([length]), (0., 0.))


def test_auxiliary_plan_persists_and_resume_rejects_changed_objective(tmp_path):
    from test_time_pipeline_p1 import setup, samples
    from aic_transfuser_lite.training.time_checkpoint_v1 import TimeCheckpointIdentity
    from aic_transfuser_lite.training.time_corpus_runner_v1 import CorpusTrainingPlan, run_training_arm
    base, config = setup(tmp_path)
    manifest = recovery_manifest(base)
    train_run = next(r['run_id'] for r in manifest['runs'] if r['split'] == 'train')
    val_run = next(r['run_id'] for r in manifest['runs'] if r['split'] == 'validation')
    train, validation = samples(train_run, config), samples(val_run, config)
    rows = [dict(anchor_id=s.anchor_id, run_id=s.run, horizon_s=1.55, response_length_m=1.2) for s in train]
    objective = RecoveryGeometryObjective(rows, split_manifest=manifest, plan=RecoveryGeometryPlan(), controller={})
    teacher = {'format': 'synthetic_auxiliary_resume_test'}
    teacher['manifest_sha256'] = content_sha256(teacher)
    identity = TimeCheckpointIdentity(manifest['manifest_sha256'], teacher['manifest_sha256'], 'c'*64, 'SCRATCH')
    kwargs = dict(train_run_ids=[s.run for s in train], validation_run_ids=[s.run for s in validation],
        split_manifest=manifest, teacher_manifest=teacher, identity=identity, config=config,
        plan=CorpusTrainingPlan(epochs=2, batch_size=2, workers=0, precision='float32'), device='cpu')
    result = run_training_arm(train, validation, output=tmp_path/'arm', recovery_objective=objective, **kwargs)
    assert result['reload_predictions_exact'] and result['optimizer_steps'] == 2
    saved = torch.load(tmp_path/'arm'/'last.pt', weights_only=False)
    assert saved['training_state']['plan']['recovery_objective'] == objective.identity
    assert all(r['train_counts']['auxiliary_loss_sum_m'] > 0 for r in saved['training_state']['history'])
    resumed = run_training_arm(train, validation, output=tmp_path/'arm', resume=True,
                               recovery_objective=objective, **kwargs)
    assert resumed['optimizer_steps'] == 2
    changed = RecoveryGeometryObjective(rows, split_manifest=manifest,
        plan=RecoveryGeometryPlan(near_weight_m_per_rad=2.), controller={})
    for other in (changed, None):
        with pytest.raises(ValueError, match='resume plan differs'):
            run_training_arm(train, validation, output=tmp_path/'arm', resume=True,
                             recovery_objective=other, **kwargs)


def test_default_runner_ast_matches_pinned_baseline_and_detects_unrelated_change(monkeypatch):
    repo = Path(__file__).resolve().parents[1]
    monkeypatch.syspath_prepend(str(repo/'tools'))
    from compare_time_recovery_objectives import default_runner_parity
    path = 'src/aic_transfuser_lite/training/time_corpus_runner_v1.py'
    old = subprocess.check_output(['git', 'show', '249cf9841edb6db01aadb031567602572675726b:'+path], cwd=repo, text=True)
    current = (repo/path).read_text()
    default_runner_parity(old, current)
    with pytest.raises(ValueError, match='historical runner differs'):
        default_runner_parity(old, current.replace('lr=plan.learning_rate', 'lr=0.1'))
