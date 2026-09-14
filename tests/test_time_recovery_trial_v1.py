import numpy as np
import pytest

from aic_transfuser_lite.evaluation.time_recovery_trial_v1 import recovery_window_metrics


def measured():
    t = np.linspace(0., 10., 101)
    return np.column_stack([t, .1*np.exp(-t), .05*np.exp(-t), np.full_like(t, 1.27)])


def test_measured_return_requires_motion_continuity_and_full_window():
    x = measured()
    result = recovery_window_metrics(x)
    assert result['recovered_in_ten_second_window']
    assert result['first_sustained_return_s'] == pytest.approx(.7)
    stopped = x.copy(); stopped[7:, 3] = 0.
    assert recovery_window_metrics(stopped)['first_sustained_return_s'] is None
    missing = x[::10]
    assert recovery_window_metrics(missing)['first_sustained_return_s'] is None
    assert not recovery_window_metrics(x[:30])['recovered_in_ten_second_window']


def test_transient_return_and_already_aligned_are_not_success():
    x = measured(); x[-1, 1] = .1
    result = recovery_window_metrics(x)
    assert result['first_sustained_return_s'] is not None
    assert not result['recovered_in_ten_second_window']
    x = measured(); x[:, 1:3] = 0.
    assert not recovery_window_metrics(x)['recovered_in_ten_second_window']


@pytest.mark.parametrize('samples', [np.zeros((2, 3)), np.zeros((2, 4)), np.full((2, 4), np.nan)])
def test_invalid_measurement_rejected(samples):
    with pytest.raises(ValueError, match='SHAPE_FINITE_TIME'):
        recovery_window_metrics(samples)
