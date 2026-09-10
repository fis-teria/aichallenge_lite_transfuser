from copy import deepcopy
import numpy as np
import pytest
import torch

from aic_transfuser_lite.training.spatial_long_diagnosis_v4 import expand_selection, cohort_summary, restore_optimizer


def test_optimizer_branches_do_not_share_step_or_moments():
    parameter = torch.nn.Parameter(torch.tensor([1.]))
    parent = torch.optim.AdamW([parameter], lr=.01)
    parameter.grad = torch.ones_like(parameter)
    parent.step()
    state = deepcopy(parent.state_dict())
    baseline = deepcopy(state)
    a_param = torch.nn.Parameter(parameter.detach().clone())
    b_param = torch.nn.Parameter(parameter.detach().clone())
    a = torch.optim.AdamW([a_param], lr=.01)
    b = torch.optim.AdamW([b_param], lr=.01)
    restore_optimizer(a, state)
    restore_optimizer(b, state)
    a_param.grad = torch.ones_like(a_param)
    a.step()
    for key in ('step', 'exp_avg', 'exp_avg_sq'):
        torch.testing.assert_close(state['state'][0][key], baseline['state'][0][key])
        torch.testing.assert_close(b.state[b_param][key], baseline['state'][0][key])
    assert a.state[a_param]['step'].item() == 2
    assert b.state[b_param]['step'].item() == 1


def row(name, stamp, run='a', support=20, split='train', eligible=True):
    return {'sample_id': name, 'stamp_ns': stamp, 'run_id': run, 'split': split,
            'diagnostic_eligible': eligible, 'normal_recovery': 'normal',
            'geometry': {'covered_grid_m': support, 'shape': 'straight'}}


def test_expansion_preserves_quota_gap_and_originals():
    old = [row('a0', 0), row('b0', 0, 'b', 1)]
    audit = old + [row('a_near', 100_000_000), row('a1', 1_000_000_000),
                   row('b1', 1_000_000_000, 'b', 1), row('val', 2_000_000_000, split='validation'),
                   row('bad', 3_000_000_000, eligible=False)]
    before = deepcopy(audit)
    expanded = expand_selection(old, audit)
    assert expanded[:2] == old
    assert {r['sample_id'] for r in expanded} == {'a0', 'a1', 'b0', 'b1'}
    assert audit == before
    assert expand_selection(old, audit) == expanded


def test_expansion_rejects_insufficient_or_changed_source():
    old = [row('a0', 0)]
    with pytest.raises(ValueError, match='insufficient'):
        expand_selection(old, old + [row('a1', 1_000_000_000, split='validation')])
    changed = deepcopy(old)
    changed[0]['stamp_ns'] = 1
    with pytest.raises(ValueError, match='metadata mismatch'):
        expand_selection(old, changed)
    with pytest.raises(ValueError, match='duplicate'):
        expand_selection(old, old + old)
    with pytest.raises(ValueError, match='eligible train'):
        expand_selection([row('v', 0, split='validation')], [])


def test_cohort_counts_observed_support_not_placeholder_values():
    rows = [row('a', 0), row('b', 0, 'b', 1)]
    mask = np.zeros((2, 46), dtype=bool)
    mask[0] = True
    mask[1, :10] = True
    summary = cohort_summary(rows, mask)
    assert summary['bands']['near_0_2m'] == {'anchors': 2, 'points': 30, 'runs': 2}
    assert summary['bands']['at_20m'] == {'anchors': 1, 'points': 1, 'runs': 1}
    with pytest.raises(ValueError):
        cohort_summary(rows, mask.astype(float))
