from copy import deepcopy
from dataclasses import replace
import numpy as np
import pytest
import torch
from aic_transfuser_lite.data.time_split_v1 import content_sha256
from aic_transfuser_lite.training.native_fit_v1 import NativeGeometryObjective,comparison_schedule
from test_time_batched_evaluation_v1 import _manifest,_sample


def context():
    manifest=deepcopy(_manifest())
    for row in manifest['runs']:row['run_id']='lidar-v45-pc10-'+row['run_id']
    manifest['manifest_sha256']=content_sha256({k:v for k,v in manifest.items() if k not in {'manifest_sha256','sources_verified'}})
    run=next(r['run_id'] for r in manifest['runs'] if r['split']=='train')
    s=_sample(run)
    obj=NativeGeometryObjective([dict(anchor_id=s.anchor_id,run_id=run,horizon_s=1.5,response_length_m=1.2)],split_manifest=manifest)
    return s,obj,manifest


def test_geometry_zero_perfect_and_far_metres_detach():
    s,obj,_=context(); t=torch.tensor(s.teacher.xy_m[None],requires_grad=True)
    mask=torch.ones(1,30,dtype=torch.bool)
    assert obj(t,t,mask,[s]).item()==0
    p=t.detach().clone();p[:,19:,1]+=.1;p.requires_grad_(True)
    loss=obj(p,t,mask,[s]);assert loss.item()==pytest.approx(.05)
    loss.backward();assert t.grad is None
    assert torch.all(p.grad[:,19:,1]>0) and torch.all(p.grad[:,:19]==0)


def test_native_rejects_mismatch_partial_and_holdout():
    s,obj,m=context();p=torch.ones(1,30,2);mask=torch.ones(1,30,dtype=torch.bool)
    with pytest.raises(ValueError,match='matching run'):obj(p,p,mask,[replace(s,run='other')])
    mask[0,-1]=False
    with pytest.raises(ValueError,match='complete native'):obj(p,p,mask,[s])
    with pytest.raises(ValueError,match='expected'):obj(p[:,:2],p,mask,[s])
    run=next(r['run_id'] for r in m['runs'] if r['split']=='validation')
    with pytest.raises(ValueError):NativeGeometryObjective([dict(anchor_id='x',run_id=run,horizon_s=1.5,response_length_m=1.)],split_manifest=m)


def test_schedule_fixed_budget_reproducible_and_preserves_old_order():
    kw=dict(old_count=500,native_count=20,front=[1,2,3],steps=40)
    a=comparison_schedule(**kw,focused=False);b=comparison_schedule(**kw,focused=True)
    assert a==comparison_schedule(**kw,focused=False)
    assert all(len(r)==32 for r in a+b)
    assert sum(k=='native' for row in a for k,_ in row)==40*32*3380//63988
    assert sum(k=='native' for row in b for k,_ in row)==40*8
    old_a=[i for row in a for k,i in row if k=='old'];old_b=[i for row in b for k,i in row if k=='old']
    assert old_a[:len(old_b)]==old_b
    for row in b:assert all(i in kw['front'] for k,i in row[-8:-4])


@pytest.mark.parametrize('front',[[],[0,0],[99]])
def test_invalid_front_selection_is_explicit(front):
    with pytest.raises(ValueError):comparison_schedule(old_count=3,native_count=4,front=front,steps=3,focused=True)
