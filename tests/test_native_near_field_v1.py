from dataclasses import replace

import numpy as np
import pytest
import torch

from aic_transfuser_lite.training.native_near_field_v1 import (
    NativeNearFieldObjective, station_values, teacher_station_brackets,
)
from test_time_batched_evaluation_v1 import _manifest, _sample


class Base:
    last_terms = {'old_retention_sum_m': .25}
    def validate_samples(self, samples):
        pass
    def __call__(self, prediction, target, support, samples):
        return prediction.sum() * 0 + .25


def fixture():
    manifest = _manifest()
    run = next(r['run_id'] for r in manifest['runs'] if r['split']=='train')
    sample = _sample(run)
    xy = np.stack([np.arange(1,31)*.2, np.arange(1,31)*.02], axis=1).astype(np.float32)
    sample = replace(sample, teacher=replace(sample.teacher, xy_m=xy))
    lower, alpha = teacher_station_brackets(xy.astype(float))
    row = dict(anchor_id=sample.anchor_id, run_id=run, teacher_xy_m=xy.tolist(), lower=lower, alpha=alpha)
    return manifest, sample, row


def test_station_interpolation_uses_label_time_and_preserves_longitudinal_error():
    _, sample, row = fixture()
    target = torch.tensor(sample.teacher.xy_m)[None]
    lower, alpha = torch.tensor([row['lower']]), torch.tensor([row['alpha']])
    teacher = station_values(target, lower, alpha)
    assert torch.allclose(teacher[0,:,0], torch.tensor([1.,2.,3.]))
    predicted = target.clone()
    predicted[:,:,0] *= .5
    assert torch.allclose(station_values(predicted, lower, alpha)[0,:,0], torch.tensor([.5,1.,1.5]))


def test_no_extrapolation_no_artificial_origin_no_backward_prefix():
    _, sample, _ = fixture()
    xy = sample.teacher.xy_m.copy()
    assert teacher_station_brackets(xy*.1) is None
    xy[:,0] += 2
    assert teacher_station_brackets(xy) is None
    xy = sample.teacher.xy_m.copy(); xy[3,0] = xy[2,0]-.1
    assert teacher_station_brackets(xy) is None
    xy[0,0] = np.nan
    with pytest.raises(ValueError): teacher_station_brackets(xy)


def test_near_loss_units_gradient_direction_native_only_and_target_detached():
    manifest, sample, row = fixture()
    old = replace(sample, anchor_id='old')
    obj = NativeNearFieldObjective(Base(), [row], weight=2., split_manifest=manifest)
    target = torch.tensor(np.stack([sample.teacher.xy_m]*2), requires_grad=True)
    p = target.detach().clone(); p[0,:,1] -= .1; p.requires_grad_()
    result = obj(p, target, torch.ones(2,30,dtype=torch.bool), [sample,old])
    assert result.item() == pytest.approx(.25 + 2*.09/2, abs=1e-6)
    result.backward()
    assert target.grad is None and torch.all(p.grad[1]==0)
    assert p.grad[0,:,1].sum() < 0 and torch.all(p.grad[0,:,0]==0)
    assert obj.last_terms['old_retention_sum_m'] == .25


def test_identity_and_train_split_are_enforced():
    manifest, sample, row = fixture()
    obj = NativeNearFieldObjective(Base(), [row], weight=1., split_manifest=manifest)
    with pytest.raises(ValueError): obj.validate_samples([replace(sample,run='wrong')])
    heldout = next(r['run_id'] for r in manifest['runs'] if r['split']=='validation')
    with pytest.raises(ValueError): NativeNearFieldObjective(Base(), [dict(row,run_id=heldout)], weight=1., split_manifest=manifest)
    with pytest.raises(ValueError): NativeNearFieldObjective(Base(), [row,row], weight=1., split_manifest=manifest)


def test_missing_support_wrong_target_and_shapes_rejected():
    manifest, sample, row = fixture()
    obj = NativeNearFieldObjective(Base(), [row], weight=1., split_manifest=manifest)
    p = torch.tensor(sample.teacher.xy_m)[None]
    mask = torch.ones(1,30,dtype=torch.bool); mask[0,0]=False
    with pytest.raises(ValueError): obj(p,p,mask,[sample])
    with pytest.raises(ValueError): obj(p,p+1,torch.ones_like(mask),[sample])
    with pytest.raises(ValueError): obj(p[:,:29],p[:,:29],mask[:,:29],[sample])


@pytest.mark.parametrize('weight', [-1., float('nan'), float('inf')])
def test_invalid_weight(weight):
    manifest, _, row = fixture()
    with pytest.raises(ValueError): NativeNearFieldObjective(Base(), [row], weight=weight, split_manifest=manifest)


def test_bad_station_index_rejected():
    with pytest.raises(ValueError):
        station_values(torch.zeros(1,30,2), torch.tensor([[0,1,29]]), torch.zeros(1,3))
