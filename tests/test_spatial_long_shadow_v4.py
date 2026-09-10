import numpy as np
import pytest
import torch
from aic_transfuser_lite.runtime.spatial_long_shadow_v4 import (
    CHECKPOINT_SHA, LongShadowSession, snapshot, validate_trial)
from aic_transfuser_lite.runtime.slam_shadow_geometry_v4 import display_points
from test_publisherless_shadow_v4 import setup, apply


def test_long_session_preserves_all_points_and_has_no_control():
    runner, _, _ = setup(lambda _: np.zeros((46, 2), np.float32))
    runner.__class__ = LongShadowSession
    result = apply(runner, 1)
    assert result['event'] == 'PLAN' and len(result['raw_xy_m']) == 46
    assert result['source'] == LongShadowSession.output_source
    assert not result['accepted'] and result['control_publish_count'] == 0
    result['observation_pose_xyyaw'] = [1., 2., np.pi / 2]
    assert np.allclose(display_points(result), [[1., 2.]] * 46)
    result['source'] = 'FIXED_V4_UNCORRECTED'
    with pytest.raises(ValueError, match='SHAPE'):
        display_points(result)


@pytest.mark.parametrize('shape,dtype', [((1,20,2),torch.float32),((1,46,2),torch.float64)])
def test_wrong_output_rejected(shape, dtype):
    with pytest.raises(ValueError):
        snapshot(torch.zeros(shape, dtype=dtype))


def test_snapshot_copy_finite_and_identity():
    value = torch.ones((1, 46, 2))
    result = snapshot(value)
    value.zero_()
    assert result.shape == (46, 2) and np.all(result == 1.)
    value[0,0,0] = float('nan')
    with pytest.raises(ValueError): snapshot(value)
    validate_trial(dict(checkpoint='/trial/epoch_12.pt', sha256=CHECKPOINT_SHA))
    with pytest.raises(ValueError): validate_trial(dict(checkpoint='/trial/epoch_12.pt', sha256='bad'))
    with pytest.raises(ValueError): validate_trial(dict(checkpoint='relative.pt', sha256=CHECKPOINT_SHA))
