"""Cache replay, missing histories, metadata retention and immutable identity."""
from dataclasses import asdict, replace
import json
from types import SimpleNamespace
import numpy as np
import pytest
import torch
from aic_transfuser_lite.data.clock_segments import ClockEpoch
from aic_transfuser_lite.data.time_corpus_v1 import audit_anchor
from aic_transfuser_lite.data.time_split_v1 import content_sha256
from aic_transfuser_lite.data.time_training_cache_v1 import (
    TimeTrainingCacheDataset, _prepare_run, _sha, cached_inputs)
from test_time_dataset_p1 import fixture, cfg
from test_time_batched_evaluation_v1 import _manifest


def make_cache(tmp_path, monkeypatch):
    manifest = _manifest()
    run_id = next(r['run_id'] for r in manifest['runs'] if r['split'] == 'train')
    events, _ = fixture()
    events = [replace(e, run=run_id, sequence=i+1) for i,e in enumerate(events)
              if not (e.role == 'lidar' and e.capture_ns > 2_040_000_000)]
    anchors = [e for e in events if e.role == 'camera' and e.capture_ns in (2_000_000_000, 2_100_000_000)]
    config = replace(cfg(), image_shape=(3,4,5))
    source = tmp_path/'source'; source.mkdir()
    rows, labels = [], []
    for i, anchor in enumerate(anchors):
        teacher, row = audit_anchor(events, anchor, config=config, bounds=(0,5_500_000_000),
                                    freeze_ns=anchor.available_ns, intervention_ns=None)
        row.update(run_id=run_id,split='train',label_index=i)
        rows.append(row); labels.append(teacher)
    (source/'anchors.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows),encoding='utf-8')
    np.savez(source/'teachers.npz', **{k:np.stack([getattr(t,k) for t in labels])
             for k in ('xy_m','xy_mask','velocity_mps','velocity_mask','interval_mask')})
    index = SimpleNamespace(events=events,epochs=(ClockEpoch('ep',0,55,0,5_500_000_000,0,5_500_000_000,None),))
    monkeypatch.setattr('aic_transfuser_lite.data.time_training_cache_v1.read_time_sqlite_run',lambda *args:index)
    monkeypatch.setattr('aic_transfuser_lite.data.time_training_cache_v1.load_event',lambda path,event:event)
    root = tmp_path/'cache'
    result = _prepare_run(source,root/'train'/run_id,run_id,config)
    result['split']='train'
    files=[{'path':p.relative_to(root).as_posix(),'bytes':p.stat().st_size,'sha256':_sha(p)}
           for p in sorted(root.rglob('*')) if p.is_file()]
    identity={'format':'time_training_cache_v1','config':asdict(config),'split_manifest':manifest,
              'runs':[result],'cache_files':files}
    identity['manifest_sha256']=content_sha256(identity)
    (root/'identity.json').write_text(json.dumps(identity),encoding='utf-8')
    return root


def test_cache_exact_replay_and_invalid_input_keeps_teacher(tmp_path,monkeypatch):
    dataset=TimeTrainingCacheDataset(make_cache(tmp_path,monkeypatch),'train')
    assert len(dataset)==len(dataset.run_ids)==len(dataset.anchor_ids)==2
    assert dataset[0].inputs.image.shape==(1,2,3,4,5)
    assert dataset[1].inputs is None
    assert dataset[1].teacher.xy_mask.all()
    assert dataset[1].input_invalid_reason=='CURRENT_SENSOR_MISSING'
    assert dataset[1].teacher_reasons==('OK',)
    original=np.load
    def no_repeated_npz(path,*args,**kwargs):
        assert not str(path).endswith('.npz')
        return original(path,*args,**kwargs)
    monkeypatch.setattr(np,'load',no_repeated_npz)
    assert dataset[0].teacher.xy_m.shape==(30,2)


def test_missing_history_is_zero_after_normalization_not_last_sensor(tmp_path,monkeypatch):
    dataset=TimeTrainingCacheDataset(make_cache(tmp_path,monkeypatch),'train')
    sample=dataset[0]
    inputs=dataset._runs[0]['inputs']
    inputs['camera_refs'][0,0]=-1;inputs['lidar_refs'][0,0]=-1
    camera,lidar=dataset._sensors[0]
    result=cached_inputs(inputs,0,camera,lidar,dataset.config)
    assert not result.image_mask[0,0] and not result.lidar_mask[0,0]
    assert torch.count_nonzero(result.image[0,0])==0
    assert torch.count_nonzero(result.lidar[0,0])==0
    torch.testing.assert_close(result.image[0,1],sample.inputs.image[0,1],rtol=0,atol=0)


def test_cache_hash_change_and_sealed_test_rejected(tmp_path,monkeypatch):
    root=make_cache(tmp_path,monkeypatch)
    with pytest.raises(ValueError,match='sealed'):
        TimeTrainingCacheDataset(root,'test')
    path=next(root.rglob('camera_rgb.npy'))
    with path.open('ab') as stream:stream.write(b'corruption')
    with pytest.raises(ValueError,match='cache file mismatch'):
        TimeTrainingCacheDataset(root,'train')


def test_legacy_epoch_reason_refinement_is_narrow_and_preserves_source():
    from copy import deepcopy
    from aic_transfuser_lite.data.time_training_cache_v1 import _refine_legacy_epoch_reason
    row = dict(anchor_id='run:0', input_invalid_reason='CURRENT_SENSOR_MISSING',
               input_eligible=False, usable_partial=False, usable_full=False,
               observation_ns=99, epoch_bounds_ns=[100, 200], history_row_ids={'camera': [[], []]})
    changed = deepcopy(row)
    refinement = _refine_legacy_epoch_reason(changed, 'ANCHOR_OUTSIDE_EPOCH', (100, 200))
    assert refinement['source_reason'] == changed['source_input_invalid_reason'] == 'CURRENT_SENSOR_MISSING'
    assert changed['input_invalid_reason'] == 'ANCHOR_OUTSIDE_EPOCH'
    assert not changed['input_eligible'] and changed['history_row_ids'] == row['history_row_ids']
    for values, reason in (({'observation_ns': 100}, 'ANCHOR_OUTSIDE_EPOCH'),
                           ({'input_eligible': True}, 'ANCHOR_OUTSIDE_EPOCH'),
                           ({'history_row_ids': {'camera': [[1], []]}}, 'ANCHOR_OUTSIDE_EPOCH'),
                           ({}, 'CURRENT_LONGITUDINAL_SPEED_MISSING')):
        with pytest.raises(ValueError, match='input audit drift'):
            _refine_legacy_epoch_reason({**row, **values}, reason, (100, 200))
