from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
import sys

import numpy as np
import pytest

from aic_transfuser_lite.data.time_corpus_v1 import audit_anchor
from aic_transfuser_lite.data.time_dataset_v1 import TimeDatasetConfig, assemble_time_sample
from aic_transfuser_lite.data.time_native_replay_v1 import (
    AdditiveReplayDataset, check_native_replay, extend_train_split,
)
from test_time_dataset_p1 import fixture
from test_time_recovery_training_v1 import extension
from test_time_stage_selection_v1 import report

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from train_time_native_replay import select_retained_native


class Samples:
    def __init__(self, run_ids, anchor_ids):
        self.run_ids, self.anchor_ids = run_ids, anchor_ids
        self.samples = [object() for _ in anchor_ids]

    def __len__(self):
        return len(self.anchor_ids)

    def __getitem__(self, index):
        return self.samples[index]


def split_fixture():
    base = extension()
    additions = [dict(run_id='native', speed_cap_kmh=5, split='train',
                      sources=[dict(path='native/run.mcap.zstd', sha256='c' * 64)])]
    return base, additions, extend_train_split(base, additions, group='same_scenario')


def test_every_previous_slot_and_all_old_split_assignments_are_preserved():
    base, additions, split = split_fixture()
    assert all(r in split['runs'] for r in base['runs'])
    assert additions[0] not in base['runs']
    nominal = next(r['run_id'] for r in base['base_manifest']['runs'] if r['split'] == 'train')
    previous = Samples([nominal, 'recovery_train', 'recovery_train'], ['launch', 'recover', 'recover'])
    native = Samples(['native', 'native'], ['cone0', 'cone1'])
    combined = AdditiveReplayDataset(previous, native, repeats=10, split_manifest=split)
    assert len(combined) == 23
    assert combined.anchor_ids[:3] == previous.anchor_ids
    assert all(combined[i] is previous[i] for i in range(3))
    assert all(combined[3+i] is native[i % 2] for i in range(20))
    assert combined.audit['previous_slots_preserved']
    assert combined.audit['native_unique'] == 2
    with pytest.raises(IndexError):
        combined[-1]


@pytest.mark.parametrize('mutation', ['validation', 'duplicate_hash', 'duplicate_run', 'unverified'])
def test_native_group_cannot_cross_holdout_or_duplicate_old_source(mutation):
    base, additions, _ = split_fixture()
    if mutation == 'validation':
        additions[0]['split'] = 'validation'
    elif mutation == 'duplicate_hash':
        additions[0]['sources'][0]['sha256'] = base['runs'][0]['sources'][0]['sha256']
    elif mutation == 'duplicate_run':
        additions[0]['run_id'] = base['runs'][0]['run_id']
    else:
        base['sources_verified'] = False
    with pytest.raises(ValueError):
        extend_train_split(base, additions, group='same_scenario')


@pytest.mark.parametrize('repeats', [0, -1, True, 1.5])
def test_replay_repetition_requires_positive_finite_integer(repeats):
    _, _, split = split_fixture()
    with pytest.raises(ValueError):
        AdditiveReplayDataset(Samples(['recovery_train'], ['old']), Samples(['native'], ['new']),
                              repeats=repeats, split_manifest=split)


def native_fixture():
    events, _ = fixture(steering=True)
    events = [replace(e, available_clock='bag_receipt', sequence=i) for i, e in enumerate(events)]
    anchor = next(e for e in events if e.role == 'camera' and e.capture_ns == 2_000_000_000)
    cfg = TimeDatasetConfig()
    bounds = (0, 5_500_000_000)
    sample = assemble_time_sample(events, anchor, config=cfg, epoch_start_ns=bounds[0],
                                 epoch_end_ns=bounds[1], freeze_ns=2_000_000_001)
    _, replay = audit_anchor(events, anchor, config=cfg, bounds=bounds, freeze_ns=2_000_000_001,
                             intervention_ns=None)
    row = dict(split='unassigned', split_group='all_corners_20260918',
        allowed_targets=['xy_m', 'velocity_mps'], stop_label_valid=False, mode_label_valid=False,
        forward_avoidance_eligible=False, teacher_pose_prefix_eligible=True,
        anchor_id=sample.anchor_id, run_id=sample.run, observation_ns=sample.observation_ns,
        freeze_ns=sample.freeze_ns, history_row_ids=replay['history_row_ids'])
    return row, replay, sample


def test_native_import_keeps_metre_targets_history_and_unknown_stop():
    row, replay, sample = native_fixture()
    check_native_replay(row, replay, sample, sample.teacher.xy_m, sample.teacher.velocity_mps)
    assert sample.stop_probability is None and sample.inputs.targets is None
    assert sample.teacher.xy_m.shape == (30, 2)
    np.testing.assert_allclose(sample.teacher.xy_m[-1], [6., 0.], atol=1e-6)


@pytest.mark.parametrize('mutation', ['history', 'target', 'shape', 'stop', 'split', 'freeze'])
def test_native_import_rejects_replay_label_or_split_drift(mutation):
    row, replay, sample = native_fixture()
    xy = sample.teacher.xy_m.copy()
    if mutation == 'history':
        row['history_row_ids'] = {}
    elif mutation == 'target':
        xy[0, 0] += .1
    elif mutation == 'shape':
        xy = xy[:29]
    elif mutation == 'stop':
        row['stop_label_valid'] = True
    elif mutation == 'split':
        row['split'] = 'validation'
    else:
        sample = replace(sample, freeze_ns=sample.freeze_ns + 1)
    with pytest.raises((ValueError, AssertionError)):
        check_native_replay(row, replay, sample, xy, sample.teacher.velocity_mps)


def candidate(name='initial', fit_factor=1.):
    result = report(name)
    result.update(checkpoint=name + '.pt', checkpoint_sha256='a' * 64,
        recovery_by_run={'recovery_val': dict(anchors=20, runs=1, ade_m=.02, endpoint_3s_m=.05)},
        native_fit={kind: dict(anchors=10, runs=2, ade_m=.4 * fit_factor, endpoint_3s_m=.8 * fit_factor)
                    for kind in ('all', 'static_cone_xy_speed')})
    return result


def choose(*rows):
    plan = json.loads((Path(__file__).resolve().parents[1] /
        'configs/time_path_p1/native_obstacle_replay_20260918.json').read_text())
    return select_retained_native(list(rows), plan)


def test_native_fit_can_improve_with_small_retained_error_change():
    new = candidate('new', .5)
    new['xy']['recovery']['ade_m'] *= 1.01
    result = choose(candidate(), new)
    assert result['selected_candidate_id'] == 'new'
    assert result['automatic_runtime_promotion'] is False
    assert result['test_usage'] == 'sealed'


@pytest.mark.parametrize('mutation', ['macro', 'one_run', 'launch', 'native_nan', 'no_gain'])
def test_new_obstacle_fit_cannot_hide_recovery_or_launch_regression(mutation):
    new = candidate('new', .5)
    if mutation == 'macro':
        new['xy']['recovery']['ade_m'] = .08
    elif mutation == 'one_run':
        new['recovery_by_run']['recovery_val']['endpoint_3s_m'] = .08
    elif mutation == 'launch':
        new['launch'][0]['accepted'] = False
    elif mutation == 'native_nan':
        new['native_fit']['static_cone_xy_speed']['ade_m'] = float('nan')
    else:
        new = candidate('new', 1.)
    assert choose(candidate(), new)['selected_candidate_id'] == 'initial'


def test_fit_population_cannot_change_for_model_selection():
    new = candidate('new', .5)
    new['native_fit']['all']['anchors'] -= 1
    with pytest.raises(ValueError):
        choose(candidate(), new)
