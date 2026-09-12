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
