import pytest
import torch

from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
from aic_transfuser_lite.data.spatial_diagnostic_inputs_v4 import INPUT_FIELDS
from aic_transfuser_lite.training.spatial_long_full_v4 import InputCache, epoch_batches, all_train


def batch():
    return ModelBatchV3(image=torch.rand(1,4,3,16,16), image_mask=torch.ones(1,4,dtype=torch.bool),
        lidar=torch.rand(1,4,2,16), lidar_mask=torch.ones(1,4,dtype=torch.bool),
        ego=torch.rand(1,10,4), ego_feature_mask=torch.ones(1,10,4,dtype=torch.bool),
        command_history=torch.rand(1,10,3), command_mask=torch.ones(1,10,dtype=torch.bool),
        sensor_dt_sec=torch.zeros(1,4,2))


def test_epoch_visits_every_teacher_once_including_tail():
    for epoch in (1,2,16):
        batches=epoch_batches(1786,epoch)
        assert len(batches)==224 and len(batches[-1])==2
        assert sorted(i for b in batches for i in b)==list(range(1786))
        assert epoch_batches(1786,epoch)==batches
    assert epoch_batches(1786,1)!=epoch_batches(1786,2)
    with pytest.raises(ValueError): epoch_batches(0,1)


def test_all_train_excludes_validation_and_rejects_inconsistent_audit():
    rows=[dict(sample_id=str(i),run_id='r',stamp_ns=i,split=s,diagnostic_eligible=ok,reasons=[])
          for i,(s,ok) in enumerate([('train',True),('validation',True),('train',False),('test',True)])]
    assert all_train(rows)==[rows[0]]
    with pytest.raises(ValueError): all_train(rows+rows)
    rows[0]['reasons']=['bad']
    with pytest.raises(ValueError): all_train(rows)


def test_cache_exact_tensor_parity_bounded_memory_and_corruption(tmp_path):
    originals=[batch(),batch()]
    hashes={i:InputCache.save(tmp_path/f'{i:05d}.pt',b) for i,b in enumerate(originals)}
    cache=InputCache(tmp_path,hashes,capacity=1)
    for i in (0,1,0):
        loaded=cache.get(i)
        assert loaded.targets is None
        for field in INPUT_FIELDS:
            torch.testing.assert_close(getattr(loaded,field),getattr(originals[i],field),rtol=0,atol=0)
        assert len(cache.memory)==1
    with pytest.raises(ValueError): cache.get(2)
    with pytest.raises(ValueError): InputCache.save(tmp_path/'00000.pt',originals[0])
    (tmp_path/'00001.pt').write_bytes(b'corrupt')
    with pytest.raises(ValueError,match='identity mismatch'): cache.get(1)
