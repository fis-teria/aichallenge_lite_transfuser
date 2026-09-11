import torch

from aic_transfuser_lite.evaluation.time_metrics_v1 import (
    constant_velocity_baseline, time_horizon_metrics, zero_velocity_baseline,
)


def test_metrics_keep_invalid_rejected_stale_in_denominator_and_support_zero():
    pred = torch.zeros(4, 30, 2)
    target = torch.zeros_like(pred)
    target[:, 4, 0] = 1.0
    mask = torch.zeros(4, 30, dtype=torch.bool)
    mask[:3, 4] = True
    result = time_horizon_metrics(
        pred, target, mask,
        input_valid=torch.tensor([True, False, True, True]),
        accepted=torch.tensor([True, True, False, True]),
        stale=torch.tensor([False, False, True, False]),
        run_ids=["a", "a", "b", "b"],
    )
    h = result["horizons"]
    assert h["0.5s"]["raw_count"] == 3
    assert h["0.5s"]["accepted_count"] == 1
    assert h["0.5s"]["total_count"] == 4
    assert result["input_invalid_count"] == 1
    assert result["rejected_count"] == 1
    assert result["stale_count"] == 1
    assert h["1s"]["status"] == "NOT_EVALUATED"


def test_metrics_reject_nonfinite_prediction_and_keep_denominator():
    pred = torch.zeros(1, 30, 2)
    pred[0, 0, 0] = float("nan")
    result = time_horizon_metrics(pred, torch.zeros_like(pred), torch.ones(1, 30, dtype=torch.bool))
    assert result["prediction_invalid_count"] == 1
    assert result["anchor_count"] == 1
    # Preserve raw finite points while rejecting the complete nonfinite plan.
    assert result["horizons"]["0.5s"]["raw_count"] == 1
    assert result["horizons"]["0.5s"]["accepted_count"] == 0
    assert result["horizons"]["0.5s"]["total_count"] == 1


def test_causal_baselines_do_not_read_future_targets():
    velocity = torch.tensor([[2.0, 0.0]])
    first = constant_velocity_baseline(velocity)
    velocity[0, 0] = 3.0
    second = constant_velocity_baseline(velocity)
    assert not torch.equal(first, second)
    assert torch.equal(zero_velocity_baseline(1), torch.zeros(1, 30, 2))


def test_partial_teacher_nan_does_not_erase_supported_horizons():
    p=torch.zeros(2,30,2);p[1]=float('nan')
    t=torch.full_like(p,float('nan'));t[:,:5]=1.
    mask=torch.zeros(2,30,dtype=torch.bool);mask[:,:5]=True
    r=time_horizon_metrics(p,t,mask,input_valid=torch.tensor([True,False]))
    assert r['target_invalid_count']==0
    assert r['horizons']['0.5s']['teacher_support_count']==2
    assert r['horizons']['0.5s']['raw_count']==1
    assert r['horizons']['1s']['raw_error_m'] is None
    assert r['all_teacher_support_count']==10 and r['all_point_count']==5
