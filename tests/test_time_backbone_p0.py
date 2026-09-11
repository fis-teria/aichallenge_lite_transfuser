from copy import deepcopy
from dataclasses import replace
import pytest
import torch
from aic_transfuser_lite.models.time_path_v1 import TimePathV1, validate_time_contract,time_contract
from aic_transfuser_lite.models.time_backbone_v1 import SlotGRU,encode_valid
from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3


def model(**kwargs):
    return TimePathV1(image_height=32,image_width=32,lidar_points=32,ego_dim=4,
        hidden_dim=16,camera_tokens_hw=(2,2),lidar_tokens=4,fusion_depth=1,fusion_heads=4,**kwargs)


def batch():
    mask=torch.tensor([[False,True,False,True]]*2)
    return ModelBatchV3(image=torch.randn(2,4,3,32,32),image_mask=mask,
        lidar=torch.rand(2,4,2,32),lidar_mask=mask.clone(),ego=torch.zeros(2,10,4),
        ego_feature_mask=torch.ones(2,10,4,dtype=torch.bool),command_history=torch.zeros(2,10,3),
        command_mask=torch.ones(2,10,dtype=torch.bool),sensor_dt_sec=torch.zeros(2,4,2))


def test_b03_train_masked_camera_lidar_cannot_change_predictions_or_bn():
    torch.manual_seed(7);a=model().train();b=deepcopy(a);inputs=batch()
    changed=replace(inputs,image=inputs.image.clone(),lidar=inputs.lidar.clone())
    changed.image[~inputs.image_mask]=300.;changed.lidar[~inputs.lidar_mask]=-300.
    torch.manual_seed(9);p=a(inputs)
    torch.manual_seed(9);q=b(changed)
    torch.testing.assert_close(p,q,rtol=0,atol=0)
    for (name,x),(_,y) in zip(a.named_buffers(),b.named_buffers()):
        if 'running_' in name:torch.testing.assert_close(x,y,rtol=0,atol=0)
    p.square().mean().backward()
    assert a.backbone.camera.projection.weight.grad.abs().sum()>0


def test_b03_padding_count_does_not_change_valid_cnn_features_or_bn():
    a=model().backbone.camera.train();b=deepcopy(a)
    x=torch.randn(2,2,3,32,32)
    y=torch.cat((torch.full((2,2,3,32,32),100.),x),dim=1)
    p=encode_valid(a,x,torch.ones(2,2,dtype=torch.bool))
    q=encode_valid(b,y,torch.tensor([[False,False,True,True]]*2))
    torch.testing.assert_close(p,q[:,2:],rtol=0,atol=0)
    for (name,x),(_,y) in zip(a.named_buffers(),b.named_buffers()):
        if 'running_' in name:torch.testing.assert_close(x,y,rtol=0,atol=0)


def test_d01_gap_position_changes_fixed_slot_representation():
    torch.manual_seed(12);gru=SlotGRU(4)
    a=torch.tensor([[[1.]*4,[0.]*4,[2.]*4,[3.]*4]])
    b=torch.tensor([[[0.]*4,[1.]*4,[2.]*4,[3.]*4]])
    p=gru(a,torch.tensor([[True,False,True,True]]))
    q=gru(b,torch.tensor([[False,True,True,True]]))
    assert not torch.allclose(p,q)


def test_config_types_and_command_semantics_cannot_silently_load():
    bad=time_contract();bad['runtime_ready']=0
    with pytest.raises(ValueError):validate_time_contract(bad)
    bad=time_contract();bad['time_sec'][9]=True
    with pytest.raises(ValueError):validate_time_contract(bad)
    off=model(use_command_history=False,trajectory_steps=30)
    with pytest.raises(ValueError,match='configuration'):model().load_state_dict(off.state_dict(),strict=True)
    with pytest.raises(ValueError):model(trajectory_steps=15)
    restored=model(use_command_history=False);restored.load_state_dict(off.state_dict(),strict=True)


def test_optional_ego_feature_missing_is_masked_in_current_and_history_paths():
    m=model().eval();inputs=batch()
    mask=inputs.ego_feature_mask.clone();mask[:,:,3]=False
    a=replace(inputs,ego_feature_mask=mask)
    changed=a.ego.clone();changed[:,:,3]=1234.
    with torch.no_grad():
        torch.testing.assert_close(m(a),m(replace(a,ego=changed)),rtol=0,atol=0)
    mask[:,-1,0]=False
    with pytest.raises(ValueError,match='longitudinal speed'):
        m(replace(a,ego_feature_mask=mask))
