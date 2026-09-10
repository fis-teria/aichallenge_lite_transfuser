import numpy as np
import pytest
import torch

from aic_transfuser_lite.models.spatial_path_long_v4 import SpatialPathLongV4
from aic_transfuser_lite.models.spatial_path_ten_v4 import SpatialPathTenV4
from aic_transfuser_lite.training.spatial_ten_v4 import loss_ten, metrics_ten, OnDemandInputs, check_host_space
from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3


def test_head_import_preserves_first_36_points_and_strict_keys():
    old = SpatialPathLongV4(image_height=16, image_width=16, lidar_points=16, ego_dim=4)
    new = SpatialPathTenV4(image_height=16, image_width=16, lidar_points=16, ego_dim=4)
    new.initialize_from_twenty(old.state_dict())
    features = torch.randn(2, 128)
    torch.testing.assert_close(new.path_head(features).reshape(2,36,2), old.path_head(features).reshape(2,46,2)[:,:36])
    state = dict(old.state_dict())
    state['extra'] = torch.zeros(1)
    with pytest.raises(RuntimeError):
        new.initialize_from_twenty(state)
    state = dict(old.state_dict())
    state['path_head.2.bias'] = torch.zeros(72)
    with pytest.raises(ValueError):
        new.initialize_from_twenty(state)


def test_loss_10m_equal_bands_and_masked_nan_gradients():
    pred = torch.ones(2,36,2,requires_grad=True)
    target = torch.zeros_like(pred)
    mask = torch.ones(2,36,dtype=torch.bool)
    mask[1] = False
    target[1] = float('nan')
    loss = loss_ten(pred,target,mask)
    torch.testing.assert_close(loss,torch.tensor(.95))
    loss.backward()
    assert pred.grad[1].count_nonzero() == 0
    torch.testing.assert_close(pred.grad[0,:20].sum(),pred.grad[0,20:].sum())
    assert loss_ten(pred.detach(),target,torch.zeros_like(mask)) is None
    with pytest.raises(ValueError):
        loss_ten(pred[:,:20],target[:,:20],mask[:,:20])


def test_metrics_only_count_real_36_point_horizon():
    items = [dict(split=s,run_id=s,geometry={'shape':'straight'}) for s in ('train','validation')]
    target = np.zeros((2,36,2))
    pred = target.copy()
    mask = np.ones((2,36),dtype=bool)
    mask[1,20:] = False
    pred[1,20:] = 100
    metrics = metrics_ten(pred,target,mask,items)
    assert metrics['train']['unobserved_points'] == 0
    assert metrics['validation']['unobserved_points'] == 16
    assert metrics['validation']['bands']['at_10m']['anchors'] == 0
    assert 'far_10_20m' not in metrics['validation']['bands']


def test_on_demand_cache_is_bounded_input_only_and_regenerates(monkeypatch):
    import aic_transfuser_lite.training.spatial_ten_v4 as module
    from types import SimpleNamespace
    calls = []
    def build(rows,index,read,keys):
        calls.append(index)
        batch = ModelBatchV3(image=torch.zeros(1,4,3,16,16), image_mask=torch.ones(1,4,dtype=torch.bool),
            lidar=torch.zeros(1,4,2,16), lidar_mask=torch.ones(1,4,dtype=torch.bool),
            ego=torch.zeros(1,10,4), ego_feature_mask=torch.ones(1,10,4,dtype=torch.bool),
            command_history=torch.zeros(1,10,3), command_mask=torch.ones(1,10,dtype=torch.bool), sensor_dt_sec=torch.zeros(1,4,2))
        return batch, {'index':index}
    monkeypatch.setattr(module,'build_inputs',build)
    cache = OnDemandInputs(SimpleNamespace(rows=[],read=None),[{'row_index':0},{'row_index':1}],[],capacity=1)
    for i in (0,0,1,0):
        assert cache.get(i).targets is None
        assert len(cache.memory) == 1
    assert calls == [0,1,0]
    with pytest.raises(ValueError):
        cache.get(-1)


def test_host_capacity_guard_checks_physical_mount(monkeypatch):
    import aic_transfuser_lite.training.spatial_ten_v4 as module
    from types import SimpleNamespace
    monkeypatch.setattr(module.Path,'is_mount',lambda self: True)
    monkeypatch.setattr(module.shutil,'disk_usage',lambda path: SimpleNamespace(free=2*1024**3))
    with pytest.raises(RuntimeError,match='backing volume'):
        check_host_space()
    monkeypatch.setattr(module.shutil,'disk_usage',lambda path: SimpleNamespace(free=4*1024**3))
    check_host_space()
