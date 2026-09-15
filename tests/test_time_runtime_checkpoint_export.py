"""Regression: comparison checkpoints omitted the teacher runtime contract."""
from copy import deepcopy
from dataclasses import asdict
import hashlib
import json

import numpy as np
import pytest
import torch

from aic_transfuser_lite.contracts.model_batch_v3 import ModelBatchV3
from aic_transfuser_lite.data.time_split_v1 import content_sha256
from aic_transfuser_lite.runtime.time_runtime_v1 import TimeRuntimeModel
from aic_transfuser_lite.training.time_checkpoint_v1 import TimeCheckpointIdentity, save_time_checkpoint
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig, build_time_model
from tools.export_time_recovery_runtime_checkpoint import export_runtime_checkpoint
from test_time_batched_evaluation_v1 import _manifest


FORMATS = ['recovery_sampling_geometry_comparison_v1', 'observed_multiscale_recovery_training_v1']


def fixture(tmp_path, teacher_format=FORMATS[0]):
    cfg = TimeModelConfig(image_height=32, image_width=32, lidar_points=32,
        hidden_dim=16, camera_tokens_hw=(2, 2), lidar_tokens=4, fusion_depth=1, fusion_heads=4)
    split = _manifest(); dc = json.loads(json.dumps(asdict(cfg.dataset_config())))
    cache = dict(format='time_training_cache_v1', config=dc, split_manifest=split,
        contract=dict(config=dc, frame='base_link_at_observation', points=30, dt_s=.1,
                      freeze_delay_receipt_ns=50_000_000))
    cache['manifest_sha256'] = content_sha256(cache)
    teacher = dict(format=teacher_format, cache_sha256=cache['manifest_sha256'])
    teacher['manifest_sha256'] = content_sha256(teacher)
    identity = TimeCheckpointIdentity(split['manifest_sha256'], teacher['manifest_sha256'], 'c'*64, 'comparison')
    model = build_time_model(cfg).eval(); source = tmp_path/'source.pt'
    save_time_checkpoint(source, config=cfg, identity=identity, model=model, optimizer=None,
        scheduler=None, epoch=3, global_step=4281, teacher_manifest=teacher, split_manifest=split)
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    return source, digest, cache, model


@pytest.mark.parametrize('teacher_format', FORMATS)
def test_export_preserves_source_state_predictions_and_runtime_rejects_original(tmp_path, teacher_format):
    source, digest, cache, model = fixture(tmp_path, teacher_format)
    with pytest.raises(ValueError, match='TEACHER_RUNTIME_CONTRACT_MISMATCH'):
        TimeRuntimeModel(source, expected_sha256=digest, device='cpu')
    output = tmp_path/'runtime.pt'
    result = export_runtime_checkpoint(source, expected_sha256=digest, cache=cache, output=output)
    runtime = TimeRuntimeModel(output, expected_sha256=result['output_sha256'], device='cpu')
    mask = torch.ones(1,4,dtype=torch.bool)
    batch = ModelBatchV3(torch.randn(1,4,3,32,32), mask, torch.rand(1,4,2,32), mask,
        torch.zeros(1,10,4), torch.ones(1,10,4,dtype=torch.bool), torch.zeros(1,10,3),
        torch.zeros(1,10,dtype=torch.bool), torch.zeros(1,4,2), requested_outputs=frozenset({'trajectory'}))
    with torch.inference_mode(): expected = model(batch).numpy()[0]
    np.testing.assert_array_equal(runtime.predict(batch), expected)
    assert expected.shape == (30,2) and np.isfinite(expected).all()
    assert hashlib.sha256(source.read_bytes()).hexdigest() == digest
    original = torch.load(source, weights_only=False); exported = torch.load(output, weights_only=False)
    assert set(original) == set(exported) and exported['epoch'] == 3 and exported['global_step'] == 4281
    assert all(torch.equal(original['rng'][k], exported['rng'][k]) for k in ['torch'])
    assert result['model_weights_changed'] is False
    with pytest.raises(ValueError, match='FRESH_EXPORT_PATH'):
        export_runtime_checkpoint(source, expected_sha256=digest, cache=cache, output=output)


@pytest.mark.parametrize('change', ['digest', 'cache', 'points', 'dt_s', 'frame', 'freeze_delay_receipt_ns', 'config'])
@pytest.mark.parametrize('teacher_format', FORMATS)
def test_export_rejects_unproven_source_or_incompatible_contract(tmp_path, change, teacher_format):
    source, digest, cache, _ = fixture(tmp_path, teacher_format)
    if change == 'digest': digest = '0'*64
    elif change == 'cache': cache['manifest_sha256'] = '0'*64
    else:
        cache = deepcopy(cache)
        if change == 'config': cache['contract']['config']['ego_features'] = 9
        else: cache['contract'][change] = {'points':29,'dt_s':.2,'frame':'map','freeze_delay_receipt_ns':0}[change]
        cache['manifest_sha256'] = content_sha256({k:v for k,v in cache.items() if k!='manifest_sha256'})
        payload = torch.load(source, weights_only=False)
        teacher = payload['teacher_manifest']; teacher['cache_sha256'] = cache['manifest_sha256']
        teacher['manifest_sha256'] = content_sha256({k:v for k,v in teacher.items() if k!='manifest_sha256'})
        payload['identity']['teacher_manifest_sha256'] = teacher['manifest_sha256']
        torch.save(payload, source); digest = hashlib.sha256(source.read_bytes()).hexdigest()
    output = tmp_path/'runtime.pt'
    with pytest.raises(ValueError, match='HASH_MISMATCH|ORIGINAL_TRAINING_CACHE|CACHE_RUNTIME_CONTRACT'):
        export_runtime_checkpoint(source, expected_sha256=digest, cache=cache, output=output)
    assert not output.exists()


def test_multiscale_training_manifest_loads_without_export(tmp_path, monkeypatch):
    from pathlib import Path
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / 'tools'))
    from tools.train_time_multiscale_recovery import training_teacher_manifest

    source, _, cache, _ = fixture(tmp_path, FORMATS[1])
    payload = torch.load(source, weights_only=False)
    teacher = training_teacher_manifest(plan={'experiment': 'test'}, cache=cache,
        auxiliary_identity={'objective': 'test'}, preparation_sha256='a'*64, source='b'*40)
    payload['teacher_manifest'] = teacher
    payload['identity']['teacher_manifest_sha256'] = teacher['manifest_sha256']
    torch.save(payload, source)
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    runtime = TimeRuntimeModel(source, expected_sha256=digest, device='cpu')
    assert runtime.epoch == 3
    assert teacher['contract'] == cache['contract']
    cache['contract']['points'] = 29
    assert teacher['contract']['points'] == 30


def test_unknown_teacher_format_cannot_be_exported(tmp_path):
    source, digest, cache, _ = fixture(tmp_path, 'unrecognized_v1')
    with pytest.raises(ValueError, match='MISSING_COMPARISON_CONTRACT_REQUIRED'):
        export_runtime_checkpoint(source, expected_sha256=digest, cache=cache, output=tmp_path/'runtime.pt')
    assert not (tmp_path/'runtime.pt').exists()
