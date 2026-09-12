from dataclasses import replace
import torch
import pytest
from aic_transfuser_lite.training.time_corpus_runner_v1 import CorpusTrainingPlan, epoch_order, train_corpus_batch
from test_time_batched_evaluation_v1 import _sample, _Model


def test_corpus_batch_counts_invalid_and_unsupported_without_loss_dilution():
    model=_Model();optimizer=torch.optim.SGD(model.parameters(),lr=.01)
    result=train_corpus_batch(model,[_sample('run'),_sample('run',teacher=False),_sample('run',invalid=True)],
                              optimizer,precision='float32',max_grad_norm=100.)
    assert result['visited']==3 and result['input_invalid']==1
    assert result['teacher_unsupported']==1 and result['supported']==1
    assert result['loss_sum_m']==pytest.approx(.5)
    assert model.anchor.item()==pytest.approx(.51)


def test_zero_support_does_not_advance_optimizer_or_scheduler():
    model=_Model();optimizer=torch.optim.AdamW(model.parameters(),lr=.01)
    scheduler=torch.optim.lr_scheduler.StepLR(optimizer,1)
    before=model.anchor.detach().clone();last=scheduler.last_epoch
    result=train_corpus_batch(model,[_sample('run',teacher=False)],optimizer,
                              precision='float32',max_grad_norm=1.,scheduler=scheduler)
    assert not result['updated'] and optimizer.state=={}
    assert scheduler.last_epoch==last
    torch.testing.assert_close(model.anchor,before,rtol=0,atol=0)


def test_frozen_order_and_finite_plan_validation():
    a=epoch_order(99,42,0)
    assert a==epoch_order(99,42,0) and a!=epoch_order(99,42,1)
    assert sorted(a)==list(range(99))
    with pytest.raises(ValueError):replace(CorpusTrainingPlan(),epochs=0).validate()
    with pytest.raises(ValueError):replace(CorpusTrainingPlan(),learning_rate=float('nan')).validate()


def test_finite_runner_resumes_mid_epoch_and_reloads_identical_best(tmp_path, monkeypatch):
    import numpy as np
    from test_time_pipeline_p1 import setup, samples
    from aic_transfuser_lite.data.time_split_v1 import content_sha256
    from aic_transfuser_lite.training.time_checkpoint_v1 import TimeCheckpointIdentity
    from aic_transfuser_lite.training import time_corpus_runner_v1 as runner

    manifest, config = setup(tmp_path)
    train_run = next(r['run_id'] for r in manifest['runs'] if r['split'] == 'train')
    val_run = next(r['run_id'] for r in manifest['runs'] if r['split'] == 'validation')
    train = samples(train_run, config) * 2
    validation = samples(val_run, config)
    teacher = {'format': 'synthetic_resume_test_v1'}
    teacher['manifest_sha256'] = content_sha256(teacher)
    identity = TimeCheckpointIdentity(manifest['manifest_sha256'], teacher['manifest_sha256'], 'b' * 64, 'SCRATCH')
    kwargs = dict(train_run_ids=[train_run] * 4, validation_run_ids=[val_run] * 2,
                  split_manifest=manifest, teacher_manifest=teacher, identity=identity,
                  config=config, plan=CorpusTrainingPlan(epochs=2, batch_size=2, workers=0,
                      precision='float32', checkpoint_every_steps=1), device='cpu')
    reference = runner.run_training_arm(train, validation, output=tmp_path / 'reference', **kwargs)
    original = runner.train_corpus_batch
    calls = 0

    def interrupt_after_saved_batch(*args, **kw):
        nonlocal calls
        calls += 1
        if calls == 2:
            raise RuntimeError('simulated interruption')
        return original(*args, **kw)

    monkeypatch.setattr(runner, 'train_corpus_batch', interrupt_after_saved_batch)
    with pytest.raises(RuntimeError, match='simulated interruption'):
        runner.run_training_arm(train, validation, output=tmp_path / 'resumed', **kwargs)
    monkeypatch.setattr(runner, 'train_corpus_batch', original)
    resumed = runner.run_training_arm(train, validation, output=tmp_path / 'resumed', resume=True, **kwargs)
    assert resumed['anchors_visited'] == reference['anchors_visited'] == 8
    assert resumed['optimizer_steps'] == reference['optimizer_steps'] == 4
    assert resumed['best_epoch'] == reference['best_epoch']
    assert resumed['best_validation_run_macro_3s_m'] == reference['best_validation_run_macro_3s_m']
    for epoch in (1, 2):
        np.testing.assert_array_equal(
            np.load(tmp_path / 'reference' / f'validation_epoch_{epoch:02d}.npy'),
            np.load(tmp_path / 'resumed' / f'validation_epoch_{epoch:02d}.npy'))
    completed = runner.run_training_arm(train, validation, output=tmp_path / 'resumed', resume=True, **kwargs)
    assert completed['anchors_visited'] == 8 and completed['optimizer_steps'] == 4
