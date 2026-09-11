"""Real TimePath + synthetic events + bounded optimizer + checkpoint integration."""
from dataclasses import asdict, replace
import hashlib

import numpy as np
import pytest
import torch

from aic_transfuser_lite.data.mcap_converter_v2 import TimedImage, TimedLidar, TimedPose, TimedVelocity
from aic_transfuser_lite.data.time_history_v1 import TimeEvent
from aic_transfuser_lite.data.time_dataset_v1 import assemble_time_sample
from aic_transfuser_lite.data.time_split_v1 import build_time_split, verify_time_split_sources, content_sha256
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig, build_time_model
from aic_transfuser_lite.training.time_checkpoint_v1 import (
    TimeCheckpointIdentity, save_time_checkpoint, load_time_checkpoint, inspect_time_checkpoint,
)
from aic_transfuser_lite.training.train_time_v1 import (
    TimeTrainingBudget, train_time_epoch, evaluate_time_samples,
)


def setup(tmp_path):
    records=[]
    for i in range(10):
        data=f'run{i}'.encode();path=tmp_path/f'run{i}.db3';path.write_bytes(data)
        records.append({'run_id':f'run{i}','speed_cap_kmh':5,
                        'sources':[{'path':path.name,'sha256':hashlib.sha256(data).hexdigest()}]})
    manifest=verify_time_split_sources(build_time_split(records,receipt_sha256='a'*64),tmp_path)
    config=TimeModelConfig(image_height=32,image_width=32,lidar_points=32,hidden_dim=16,
        camera_tokens_hw=(2,2),lidar_tokens=4,fusion_depth=1,fusion_heads=4,
        image_history_length=2,lidar_history_length=2,ego_history_length=2,command_history_length=2)
    return manifest,config


def samples(run,config):
    events=[]
    for i in range(43):
        t=i*100_000_000
        for role,payload in (
            ('pose',TimedPose(t,i*.1,0.,0.,'map','base_link')),
            ('camera',TimedImage(t,np.zeros((8,8,3),dtype=np.uint8))),
            ('lidar',TimedLidar(t,np.ones(32)*3,-np.pi,2*np.pi/32,.1,25.)),
            ('velocity',TimedVelocity(t,1.,0.,0.))):
            events.append(TimeEvent(role,run,'epoch','sim','bag_receipt',t,t+1,i,payload))
    return [assemble_time_sample(events,anchor,config=config.dataset_config(),epoch_start_ns=0,
             epoch_end_ns=4_200_000_000,freeze_ns=anchor.available_ns)
            for anchor in events if anchor.role=='camera' and anchor.capture_ns in (1_000_000_000,1_100_000_000)]


def test_dataset_optimizer_evaluation_and_checkpoint_are_connected(tmp_path):
    manifest,config=setup(tmp_path)
    train_run=next(r['run_id'] for r in manifest['runs'] if r['split']=='train')
    validation_run=next(r['run_id'] for r in manifest['runs'] if r['split']=='validation')
    train=samples(train_run,config);validation=samples(validation_run,config)
    assert all(s.inputs is not None and s.teacher.xy_mask.all() for s in train)
    torch.manual_seed(12);model=build_time_model(config)
    optimizer=torch.optim.AdamW(model.parameters(),lr=.0001)
    scheduler=torch.optim.lr_scheduler.StepLR(optimizer,step_size=1,gamma=.9)
    before=model.delta_head.weight.detach().clone()
    progress,report=train_time_epoch(model,train,optimizer,split_manifest=manifest,
                                     budget=TimeTrainingBudget(2,1),scheduler=scheduler)
    assert progress.optimizer_steps==1 and progress.next_anchor_index==2
    assert report['supported_anchors']==2 and report['loss_m'] is not None
    assert not torch.equal(before,model.delta_head.weight)
    metrics=evaluate_time_samples(model,validation,split_manifest=manifest)
    assert metrics['anchor_count']==2 and metrics['horizons']['3s']['raw_count']==2
    assert metrics['stop_intent_status']=='NOT_EVALUATED' and metrics['physical_progress_m'] is None
    teacher={'format':'synthetic_teacher_manifest_v1','anchor_ids':[s.anchor_id for s in train],
             'config':config.to_dict()}
    teacher['manifest_sha256']=content_sha256(teacher)
    identity=TimeCheckpointIdentity(manifest['manifest_sha256'],teacher['manifest_sha256'],'b'*64,'SCRATCH')
    path=tmp_path/'state.pt'
    save_time_checkpoint(path,config=config,identity=identity,model=model,optimizer=optimizer,
        scheduler=scheduler,epoch=0,global_step=progress.optimizer_steps,split_manifest=manifest,
        teacher_manifest=teacher,next_anchor_index=progress.next_anchor_index,
        training_state={'progress':asdict(progress),'budget':asdict(TimeTrainingBudget(2,1))})
    metadata=inspect_time_checkpoint(path,config=config,identity=identity)
    assert metadata['next_anchor_index']==2
    restored=build_time_model(config);opt2=torch.optim.AdamW(restored.parameters(),lr=.0001)
    sch2=torch.optim.lr_scheduler.StepLR(opt2,step_size=1,gamma=.9)
    assert load_time_checkpoint(path,config=config,identity=identity,model=restored,optimizer=opt2,scheduler=sch2)==(0,1)
    restored.eval()
    with torch.no_grad():torch.testing.assert_close(restored(validation[0].inputs),model(validation[0].inputs),rtol=0,atol=0)
    with pytest.raises(ValueError,match='sealed'):
        evaluate_time_samples(model,validation,split_manifest=manifest,split='test')
    with pytest.raises(ValueError,match='crosses'):
        train_time_epoch(model,validation,optimizer,split_manifest=manifest,budget=TimeTrainingBudget(2,1))


def test_command_pair_initialization_is_identical_and_unverified_data_cannot_train(tmp_path):
    manifest,config=setup(tmp_path)
    torch.manual_seed(7);off=build_time_model(config)
    torch.manual_seed(7);on=build_time_model(replace(config,use_command_history=True))
    for a,b in zip(off.parameters(),on.parameters(),strict=True):
        torch.testing.assert_close(a,b,rtol=0,atol=0)
    manifest['sources_verified']=False
    with pytest.raises(ValueError,match='not been hash verified'):
        train_time_epoch(off,[],torch.optim.SGD(off.parameters(),lr=.01),
                         split_manifest=manifest,budget=TimeTrainingBudget(1,1))
