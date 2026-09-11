import torch
from types import SimpleNamespace

from aic_transfuser_lite.training.time_objective_v1 import (
    accumulate_prediction_window, accumulate_time_window, time_loss_sum_and_count,
)


def test_window_weights_unequal_support_instead_of_microbatch_mean():
    p = [torch.zeros(2, 30, 2), torch.zeros(1, 30, 2)]
    t = [torch.ones_like(p[0]), torch.ones_like(p[1]) * 3]
    m = [torch.zeros(2, 30, dtype=torch.bool), torch.zeros(1, 30, dtype=torch.bool)]
    m[0][0, :30] = True
    m[1][0, 0] = True
    total, count = accumulate_prediction_window(p, t, m)
    assert count == 2
    assert float(total / count) == 2.0


def test_zero_support_returns_zero_without_nan():
    p = torch.zeros(1, 30, 2, requires_grad=True)
    t = torch.full_like(p, float("nan"))
    value, count = time_loss_sum_and_count(p, t, torch.zeros(1, 30, dtype=torch.bool))
    assert count == 0
    value.backward()
    assert p.grad is not None and torch.equal(p.grad, torch.zeros_like(p))


class _TinyModel(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.weight = torch.nn.Parameter(torch.tensor(0.0))

    def forward(self, batch):
        return self.weight.expand(batch.targets.trajectory_xy_m.shape)


def _fake_batch(target, mask):
    from aic_transfuser_lite.contracts.model_batch_v3 import TrainingTargetsV3
    return SimpleNamespace(batch_size=target.shape[0], targets=TrainingTargetsV3(
        target, mask, torch.zeros_like(mask,dtype=torch.float32), mask))


def test_fixed_prediction_split_full_gradients_match_with_unequal_anchor_support():
    p=torch.zeros(4,30,2,requires_grad=True)
    target=torch.full_like(p,float('nan'));mask=torch.zeros(4,30,dtype=torch.bool)
    mask[0,:]=True;mask[1,0]=True;mask[3,:5]=True
    target[mask]=torch.arange(int(mask.sum()),dtype=torch.float32)[:,None]+1
    summed,count=time_loss_sum_and_count(p,target,mask)
    (summed/count).backward();expected=p.grad.clone();p.grad=None
    summed,count=accumulate_prediction_window([p[:3],p[3:]],[target[:3],target[3:]],[mask[:3],mask[3:]])
    (summed/count).backward()
    torch.testing.assert_close(p.grad,expected,rtol=0,atol=0)


def test_all_invalid_window_does_not_step_or_scheduler():
    model = _TinyModel()
    optimizer = torch.optim.SGD(model.parameters(), lr=1.0)
    scheduler = torch.optim.lr_scheduler.StepLR(optimizer, 1)
    target = torch.zeros(1, 30, 2)
    batch = _fake_batch(target, torch.zeros(1, 30, dtype=torch.bool))
    result = accumulate_time_window(model, [batch], optimizer, scheduler=scheduler)
    assert not result.updated and result.supported_anchors == 0
    assert model.weight.item() == 0.0 and scheduler.last_epoch == 0


def test_invalid_microbatch_after_valid_does_not_erase_valid_gradient():
    model = _TinyModel()
    optimizer = torch.optim.SGD(model.parameters(), lr=1.0)
    target = torch.ones(1, 30, 2)
    valid = _fake_batch(target, torch.ones(1, 30, dtype=torch.bool))
    invalid = _fake_batch(torch.full_like(target, float("nan")), torch.zeros(1, 30, dtype=torch.bool))
    result = accumulate_time_window(model, [valid, invalid], optimizer)
    assert result.updated and model.weight.item() > 0.0
