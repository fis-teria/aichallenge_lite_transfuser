from copy import deepcopy
import json

import numpy as np
import pytest

from aic_transfuser_lite.data.time_recovery_append_v1 import append_split, validate_prepared
from test_time_recovery_training_v1 import extension


def addition():
    return dict(run_id='new_recovery', split='train', speed_cap_kmh=5,
                sources=[dict(path='runs/new_recovery/bag/new.db3', sha256='c'*64)])


def test_append_preserves_every_original_assignment_without_mutation():
    original = extension()
    frozen = deepcopy(original)
    result = append_split(original, [addition()])
    assert original == frozen
    assert result['base_manifest'] == original['base_manifest']
    assert result['additional_runs'][:-1] == original['additional_runs']
    assert {r['run_id']: r for r in result['runs'] if r['run_id'] != 'new_recovery'} == {
        r['run_id']: r for r in original['runs']}


@pytest.mark.parametrize('fault', ['test', 'reassign', 'duplicate_run', 'duplicate_raw', 'empty'])
def test_append_rejects_leakage_and_duplicate_sources(fault):
    new = addition()
    additions = [new]
    if fault == 'test':
        new['split'] = 'test'
    elif fault == 'reassign':
        new['run_id'] = 'recovery_val'
    elif fault == 'duplicate_run':
        additions.append(deepcopy(new))
    elif fault == 'duplicate_raw':
        new['sources'][0]['sha256'] = 'a'*64
    elif fault == 'empty':
        additions = []
    with pytest.raises(ValueError):
        append_split(extension(), additions)


def fixture(path, events=True):
    rows = [dict(anchor_id=f'run:{i}', run_id='run', split='validation', label_index=i,
                 input_invalid_reason=None, **({'recovery_event_id': 1} if events else {})) for i in range(2)]
    arrays = dict(ego=np.zeros((2, 10, 4)), command=np.zeros((2, 10, 3)), dt=np.zeros((2, 4, 2)),
                  input_valid=np.ones(2, bool), camera_refs=np.zeros((2, 4), np.int64),
                  lidar_refs=np.zeros((2, 4), np.int64))
    labels = dict(xy_m=np.zeros((2, 30, 2)), xy_mask=np.ones((2, 30), bool))
    np.save(path/'camera_rgb.npy', np.zeros((1, 224, 384, 3), np.uint8))
    np.save(path/'lidar.npy', np.zeros((1, 2, 750), np.float32))
    return rows, arrays, labels


def save(path, rows, arrays, labels):
    (path/'anchors.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    np.savez(path/'inputs.npz', **arrays)
    np.savez(path/'labels.npz', **labels)


@pytest.mark.parametrize('events', [True, False])
def test_full_teachers_preserve_event_identity(tmp_path, events):
    rows, arrays, labels = fixture(tmp_path, events)
    save(tmp_path, rows, arrays, labels)
    assert validate_prepared(tmp_path, run_id='run', split='validation', count=2,
                             event_ids=[1] if events else []) == rows


@pytest.mark.parametrize('fault', ['split', 'duplicate', 'order', 'missing_event', 'wrong_event',
    'nan_xy', 'mask', 'xy_shape', 'invalid_input', 'ego_shape', 'nan_dt',
    'negative_ref', 'overflow_ref', 'float_ref', 'missing_current', 'nan_lidar'])
def test_prepared_rejects_inconsistent_or_incomplete_training_inputs(tmp_path, fault):
    rows, arrays, labels = fixture(tmp_path)
    if fault == 'split': rows[0]['split'] = 'train'
    elif fault == 'duplicate': rows[1]['anchor_id'] = rows[0]['anchor_id']
    elif fault == 'order': rows[0]['label_index'] = 1
    elif fault == 'missing_event': rows[0].pop('recovery_event_id')
    elif fault == 'wrong_event': rows[0]['recovery_event_id'] = 2
    elif fault == 'nan_xy': labels['xy_m'][0, 0, 0] = np.nan
    elif fault == 'mask': labels['xy_mask'][0, -1] = False
    elif fault == 'xy_shape': labels['xy_m'] = np.zeros((2, 29, 2))
    elif fault == 'invalid_input': arrays['input_valid'][0] = False
    elif fault == 'ego_shape': arrays['ego'] = np.zeros((2, 9, 4))
    elif fault == 'nan_dt': arrays['dt'][0, 0, 0] = np.nan
    elif fault == 'negative_ref': arrays['camera_refs'][0, 0] = -2
    elif fault == 'overflow_ref': arrays['lidar_refs'][0, 0] = 1
    elif fault == 'float_ref': arrays['lidar_refs'] = arrays['lidar_refs'].astype(float)
    elif fault == 'missing_current': arrays['camera_refs'][0, -1] = -1
    elif fault == 'nan_lidar': np.save(tmp_path/'lidar.npy', np.full((1, 2, 750), np.nan))
    save(tmp_path, rows, arrays, labels)
    with pytest.raises(ValueError):
        validate_prepared(tmp_path, run_id='run', split='validation', count=2, event_ids=[1])
