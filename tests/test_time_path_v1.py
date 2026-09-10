from dataclasses import replace

import pytest
import torch

from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
from aic_transfuser_lite.models.time_path_v1 import (
    TimePathV1, interval_speed, masked_time_loss, time_contract, validate_time_contract,
)


def batch() -> ModelBatchV3:
    return ModelBatchV3(image=torch.randn(2,1,3,32,32), image_mask=torch.ones(2,1,dtype=torch.bool),
        lidar=torch.rand(2,1,2,32), lidar_mask=torch.ones(2,1,dtype=torch.bool),
        ego=torch.zeros(2,1,4), ego_feature_mask=torch.ones(2,1,4,dtype=torch.bool),
        command_history=torch.zeros(2,1,3), command_mask=torch.ones(2,1,dtype=torch.bool),
        sensor_dt_sec=torch.zeros(2,1,2))


def model() -> TimePathV1:
    return TimePathV1(image_height=32,image_width=32,lidar_points=32,ego_dim=4,
        hidden_dim=16,camera_tokens_hw=(2,2),lidar_tokens=4,fusion_depth=1,fusion_heads=4).eval()


def test_contract_rejects_distance_and_wrong_time_grid():
    validate_time_contract(time_contract())
    for change in ({'grid_m':[1,2]}, {'time_sec':[i/10 for i in range(30)]},
                   {'frame':'map'}, {'runtime_ready':True}):
        with pytest.raises(ValueError):
            validate_time_contract({**time_contract(),**change})


def test_constant_motion_and_stationary_speed_units():
    xy=torch.zeros(2,30,2)
    xy[0,:,0]=torch.arange(1,31)*0.2
    torch.testing.assert_close(interval_speed(xy)[0],torch.full((30,),2.0))
    assert interval_speed(xy)[1].count_nonzero()==0
    with pytest.raises(ValueError):interval_speed(torch.zeros(2,36,2))


def test_masked_nan_does_not_poison_backward_or_weight_longer_anchor_more():
    p=torch.zeros(3,30,2,requires_grad=True)
    t=torch.full_like(p,float('nan'))
    mask=torch.zeros(3,30,dtype=torch.bool)
    mask[0,0]=True;mask[1]=True
    t[0,0]=2;t[1]=4
    loss=masked_time_loss(p,t,mask)
    torch.testing.assert_close(loss,torch.tensor(3.0))
    loss.backward()
    assert torch.isfinite(p.grad).all()
    assert p.grad[~mask].count_nonzero()==0
    assert masked_time_loss(p,t,torch.zeros_like(mask)) is None
    with pytest.raises(ValueError):masked_time_loss(p,t,torch.ones_like(mask))


def test_decoder_shape_teacher_isolation_and_command_ablation():
    m=model();b=batch()
    m.use_command_history=False
    with torch.no_grad():
        a=m(b)
        # Deliberately invalid teacher object: must not be read by forward.
        other=m(replace(b,targets=object(),command_history=torch.full_like(b.command_history,9.0)))
    assert a.shape==(2,30,2)
    torch.testing.assert_close(a,other,rtol=0,atol=0)
    assert not any('trajectory_head' in k or 'speed_profile_head' in k for k in m.state_dict())


def test_decoder_backward_reaches_fusion_and_checkpoint_roundtrip():
    m=model();b=batch();p=m(b)
    loss=masked_time_loss(p,torch.zeros_like(p),torch.ones(2,30,dtype=torch.bool))
    loss.backward()
    assert m.delta_head.weight.grad is not None
    assert torch.isfinite(m.delta_head.weight.grad).all()
    assert any(p.grad is not None and p.grad.abs().sum()>0 for p in m.backbone.fusion.parameters())
    restored=model();restored.load_state_dict(m.state_dict(),strict=True)
    with torch.no_grad():torch.testing.assert_close(m(b),restored(b),rtol=0,atol=0)
