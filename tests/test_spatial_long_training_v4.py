import numpy as np
import pytest
import torch

from aic_transfuser_lite.training.spatial_long_v4 import (
    audit_target, band_loss, directed_distance_upper, evaluate_metrics,
)


def future(speed=8.):
    f = np.zeros((30, 8))
    f[:, 0] = np.arange(1, 31)/10
    f[:, 1] = f[:, 0]*speed
    f[:, 4] = speed
    f[:, 7] = 1
    return f


def test_long_geometry_and_unknown_first():
    view, reasons = audit_target(future())
    assert reasons == []
    assert view['mask'].all()
    assert view['long_corner_cut_m'] < 1e-5
    assert view['deviation_upper_m'] <= .026
    f = future()
    f[0, 7] = 0
    view, reasons = audit_target(f)
    assert view['distance_status'] == 'UNKNOWN_FIRST_MISSING'
    assert 'no_observed_grid_support' in reasons
    assert not view['mask'].any()


@pytest.mark.parametrize('kind,reason', [('reverse', 'reverse_motion'), ('jump', 'position_jump'),
    ('time', 'invalid_time_grid_or_gap'), ('nan', 'nonfinite_valid')])
def test_audit_rejects_faults(kind, reason):
    f = future()
    if kind == 'reverse':
        f[:, 4] = -8
    elif kind == 'jump':
        f[5, 1] += 10
    elif kind == 'time':
        f[5, 0] = .8
    else:
        f[5, 2] = np.nan
    assert reason in audit_target(f)[1]


def test_long_corner_is_measured_beyond_2m():
    f = future()
    f[:, 1] = np.minimum(f[:, 0]*8, 12.3)
    f[:, 2] = np.maximum(f[:, 0]*8-12.3, 0)
    view, reasons = audit_target(f)
    assert view['deviation_upper_m'] > .15
    assert 'long_resampling_deviation' in reasons


def test_distance_upper_bounds_corner_and_rejects_nan():
    source = np.array([[0., 0.], [1., 1.], [2., 0.]])
    chord = np.array([[0., 0.], [2., 0.]])
    assert 1 <= directed_distance_upper(source, chord) <= 1.025
    with pytest.raises(ValueError):
        directed_distance_upper(source * np.nan, chord)


def test_loss_equal_bands_and_mask_zero_gradient():
    p = torch.ones(2, 46, 2, requires_grad=True)
    t = torch.zeros_like(p)
    mask = torch.ones(2, 46, dtype=torch.bool)
    mask[1] = False
    t[1] = float('nan')
    loss = band_loss(p, t, mask)
    torch.testing.assert_close(loss, torch.tensor(.95))
    loss.backward()
    assert p.grad[1].count_nonzero() == 0
    torch.testing.assert_close(p.grad[0, :20].sum(), p.grad[0, 20:36].sum())
    torch.testing.assert_close(p.grad[0, :20].sum(), p.grad[0, 36:].sum())
    assert band_loss(p.detach(), t, torch.zeros_like(mask)) is None


def test_partial_band_and_invalid_loss_contract():
    p = torch.ones(1, 46, 2, requires_grad=True)
    t = torch.full_like(p, float('nan'))
    m = torch.zeros(1, 46, dtype=torch.bool)
    m[:, :3] = True
    t[:, :3] = 0
    band_loss(p, t, m).backward()
    assert p.grad[:, 3:].count_nonzero() == 0
    with pytest.raises(ValueError):
        band_loss(p, t, m.float())
    with pytest.raises(ValueError):
        band_loss(p[:, :20], t[:, :20], m[:, :20])
    t[:, 0] = float('nan')
    with pytest.raises(ValueError):
        band_loss(p, t, m)


def test_metrics_do_not_score_unobserved_far_points():
    target = np.zeros((2, 46, 2))
    pred = np.ones_like(target)
    mask = np.zeros((2, 46), dtype=bool)
    mask[0] = True
    mask[1, :3] = True
    items = [{'split': s, 'run_id': s, 'geometry': {'shape': 'straight'}} for s in ('train', 'validation')]
    r = evaluate_metrics(pred, target, mask, items)
    assert r['validation']['bands']['far_10_20m']['anchors'] == 0
    assert r['validation']['bands']['at_20m']['anchor_mean_error_m'] is None
    assert r['train']['bands']['at_20m']['anchors'] == 1
