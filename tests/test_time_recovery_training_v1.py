from copy import deepcopy
from types import SimpleNamespace
import pytest
from aic_transfuser_lite.data.time_recovery_training_v1 import (
    phase_windows, invalid_yaw_history, recovery_extension, RecoveryMixDataset)
from aic_transfuser_lite.data.time_recovery_collection_v1 import recovery_teacher_mask
from aic_transfuser_lite.data.time_split_v1 import validate_time_split, content_sha256
from test_time_batched_evaluation_v1 import _manifest


def extension():
    return recovery_extension(_manifest(),[
        {'run_id':'recovery_train','speed_cap_kmh':5,'split':'train',
         'sources':[{'path':'runs/recovery_train/bag/a.db3','sha256':'a'*64}]},
        {'run_id':'recovery_val','speed_cap_kmh':5,'split':'validation',
         'sources':[{'path':'runs/recovery_val/bag/b.db3','sha256':'b'*64}]}])


def rehash(value):
    value['manifest_sha256']=content_sha256({k:v for k,v in value.items() if k not in {'sources_verified','manifest_sha256'}})


def test_extension_preserves_original_split_and_rejects_duplicate_content():
    value=extension();validate_time_split(value,require_verified=True)
    bad=deepcopy(value)
    bad['runs'][0]['split']='test';rehash(bad)
    with pytest.raises(ValueError):validate_time_split(bad)
    bad=deepcopy(value)
    bad['additional_runs'][1]['sources'][0]['sha256']='a'*64
    bad['runs']=sorted(bad['base_manifest']['runs']+bad['additional_runs'],key=lambda r:r['run_id'])
    rehash(bad)
    with pytest.raises(ValueError,match='duplicate raw'):validate_time_split(bad)
    value['sources_verified']=False
    with pytest.raises(ValueError,match='not verified'):validate_time_split(value,require_verified=True)


def test_phase_gap_and_hold_cannot_leak_into_future_labels():
    rows=[{'sim_ns':i*100_000_000,'phase':'recovery' if i<12 else 'hold'} for i in range(50)]
    windows=phase_windows(rows)
    mask=recovery_teacher_mask(0,windows)
    assert mask.shape==(30,) and mask[:11].all() and not mask[11:].any()
    rows=[{'sim_ns':i*100_000_000,'phase':'recovery'} for i in range(50) if i not in (10,11)]
    assert not recovery_teacher_mask(0,phase_windows(rows))[8:].any()


def test_invalid_yaw_checks_all_interpolated_history_endpoints():
    row={'history_row_ids':{'velocity':[[1,2],[],[3,4]]}}
    assert invalid_yaw_history(row,{2,4,9})==[2,4]
    assert invalid_yaw_history(row,{9})==[]


def test_repetition_is_train_only_and_does_not_multiply_validation():
    manifest=extension()
    nominal=next(r['run_id'] for r in manifest['base_manifest']['runs'] if r['split']=='train')
    dataset=SimpleNamespace(split_manifest=manifest,run_ids=[nominal,'recovery_train'],anchor_ids=['a','b'])
    mixed=RecoveryMixDataset(dataset,['recovery_train'],repeats=4)
    assert mixed.indices==[0,1,1,1,1] and len(mixed)==5
    with pytest.raises(ValueError):RecoveryMixDataset(dataset,['recovery_val'],repeats=4)
    with pytest.raises(ValueError):RecoveryMixDataset(dataset,['recovery_train'],repeats=0)
    dataset.run_ids=['recovery_val']
    with pytest.raises(ValueError):RecoveryMixDataset(dataset,['recovery_val'],repeats=4)


def test_finetune_inherits_pinned_weights_but_not_optimizer_or_progress(tmp_path, monkeypatch):
    import torch
    from test_time_pipeline_p1 import setup, samples
    from aic_transfuser_lite.training.time_config_v1 import build_time_model
    from aic_transfuser_lite.training.time_checkpoint_v1 import TimeCheckpointIdentity, save_time_checkpoint
    from aic_transfuser_lite.training import time_corpus_runner_v1 as runner
    from aic_transfuser_lite.data.time_training_cache_v1 import _sha
    manifest, config = setup(tmp_path)
    teacher={'format':'synthetic_finetune'};teacher['manifest_sha256']=content_sha256(teacher)
    old=TimeCheckpointIdentity(manifest['manifest_sha256'],teacher['manifest_sha256'],'c'*64,'source')
    new=TimeCheckpointIdentity(manifest['manifest_sha256'],teacher['manifest_sha256'],'d'*64,'finetune')
    torch.manual_seed(999)
    model=build_time_model(config)
    optimizer=torch.optim.AdamW(model.parameters(),lr=.01)
    next(model.parameters()).sum().backward();optimizer.step()
    source=tmp_path/'source.pt'
    save_time_checkpoint(source,config=config,identity=old,model=model,optimizer=optimizer,scheduler=None,
                         epoch=7,global_step=99,split_manifest=manifest,teacher_manifest=teacher)
    train_id=next(r['run_id'] for r in manifest['runs'] if r['split']=='train')
    val_id=next(r['run_id'] for r in manifest['runs'] if r['split']=='validation')
    def stop_before_update(*args,**kwargs):
        raise RuntimeError('stop before update')
    monkeypatch.setattr(runner,'train_corpus_batch',stop_before_update)
    kwargs=dict(train_run_ids=[train_id]*2,validation_run_ids=[val_id]*2,split_manifest=manifest,
        teacher_manifest=teacher,identity=new,config=config,
        plan=runner.CorpusTrainingPlan(epochs=1,batch_size=2,workers=0,precision='float32'),
        device='cpu',initialization=source,initialization_identity=old)
    with pytest.raises(ValueError,match='hash mismatch'):
        runner.run_training_arm(samples(train_id,config),samples(val_id,config),output=tmp_path/'bad',
                                initialization_sha256='0'*64,**kwargs)
    with pytest.raises(RuntimeError,match='stop before update'):
        runner.run_training_arm(samples(train_id,config),samples(val_id,config),output=tmp_path/'new',
                                initialization_sha256=_sha(source),**kwargs)
    initial=torch.load(tmp_path/'new/initial.pt',weights_only=False)
    assert initial['epoch']==initial['global_step']==0 and initial['optimizer']['state']=={}
    assert initial['identity']==new.__dict__
    for name,value in model.state_dict().items():
        if isinstance(value,torch.Tensor):torch.testing.assert_close(initial['model'][name],value,rtol=0,atol=0)
