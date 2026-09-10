import numpy as np
import pytest
import torch
from aic_transfuser_lite.runtime.spatial_ten_shadow_v4 import (
    CHECKPOINT_SHA, TenShadowSession, snapshot, validate_trial)
from aic_transfuser_lite.runtime.slam_shadow_geometry_v4 import display_points
from test_publisherless_shadow_v4 import setup, apply


def test_long_session_preserves_all_points_and_has_no_control():
    runner, _, _ = setup(lambda _: np.zeros((36, 2), np.float32))
    runner.__class__ = TenShadowSession
    result = apply(runner, 1)
    assert result['event'] == 'PLAN' and len(result['raw_xy_m']) == 36
    assert result['source'] == TenShadowSession.output_source
    assert not result['accepted'] and result['control_publish_count'] == 0
    result['observation_pose_xyyaw'] = [1., 2., np.pi / 2]
    assert np.allclose(display_points(result), [[1., 2.]] * 36)
    result['source'] = 'FIXED_V4_UNCORRECTED'
    with pytest.raises(ValueError, match='SHAPE'):
        display_points(result)


@pytest.mark.parametrize('shape,dtype', [((1,20,2),torch.float32),((1,36,2),torch.float64)])
def test_wrong_output_rejected(shape, dtype):
    with pytest.raises(ValueError):
        snapshot(torch.zeros(shape, dtype=dtype))


def test_snapshot_copy_finite_and_identity():
    value = torch.ones((1, 36, 2))
    result = snapshot(value)
    value.zero_()
    assert result.shape == (36, 2) and np.all(result == 1.)
    value[0,0,0] = float('nan')
    with pytest.raises(ValueError): snapshot(value)
    validate_trial(dict(checkpoint='/trial/epoch_02.pt', sha256=CHECKPOINT_SHA))
    with pytest.raises(ValueError): validate_trial(dict(checkpoint='/trial/epoch_02.pt', sha256='bad'))
    with pytest.raises(ValueError): validate_trial(dict(checkpoint='relative.pt', sha256=CHECKPOINT_SHA))


def test_ten_tracker_strict_shape_and_identical_near_policy():
    from aic_transfuser_lite.control.long_sim_tracking_v4 import tracking_command
    from aic_transfuser_lite.data.spatial_long_view_v4 import LONG_GRID
    raw = np.c_[LONG_GRID,np.zeros(46)]
    original = raw.copy()
    a = tracking_command(raw,(0.,0.,0.),(0.,0.,0.),.1)
    b = tracking_command(raw[:36],(0.,0.,0.),(0.,0.,0.),.1,expected_points=36)
    assert a == b and np.array_equal(original,raw)
    with pytest.raises(ValueError):
        tracking_command(raw,(0.,0.,0.),(0.,0.,0.),.1,expected_points=36)


def test_stock_rviz_ten_keeps_existing_twenty_display():
    import importlib.util
    from pathlib import Path
    spec = importlib.util.spec_from_file_location('rviz_integrator',Path(__file__).parents[1]/'tools/integrate_normal_rviz_v4.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    original='Visualization Manager:\n  Class: ""\n  Displays:\n'
    existing=module.integrate(original)
    updated=module.integrate(existing,ten=True)
    stanza=module.DISPLAY.replace('V4-20','V4-10').replace('v4_20','v4_10')
    assert updated.replace(stanza,'',1)==existing
    with pytest.raises(ValueError):module.integrate(updated,ten=True)
