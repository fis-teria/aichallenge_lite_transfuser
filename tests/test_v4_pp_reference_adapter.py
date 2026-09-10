"""Artificial geometry only; no ROS, model, dataset or simulator access."""
from dataclasses import replace
import math

import numpy as np
import pytest

from aic_transfuser_lite.control.path_control_bridge import Limits, PathPose, Plan
from aic_transfuser_lite.control.v4_pp_reference_adapter import V4PPReferenceAdapter


def fixture():
    limits = Limits('ARTIFICIAL_TEST_ONLY', True, 1., -.2, .6, 1., .8, .5, 1., .3,
                    .5, .1, .5, 1., .6, .8, 1., 2., .2, .01, .05, .1)
    xy = tuple((float(x), 0.) for x in np.linspace(0., 2., 20))
    plan = Plan('p1', 'FIXED_V4_UNCORRECTED', 1., 50., 3., 'sim', '0',
                'base_link', 'BASE_LINK_ORIGIN', xy)
    pose = PathPose('p1', 1., 'sim', '0', (10., 20., math.pi / 2), 'SYNTHETIC')
    adapter = V4PPReferenceAdapter(limits, fixed_frame='local', fixture_mode=True)
    return adapter, plan, pose


def accept(a, p, tf, now=1.):
    return a.accept(p, tf, pose_frame='local', now_s=now)


def poll(a, now=1., epoch='0'):
    return a.poll(now_s=now, clock='sim', epoch=epoch)


def test_raw_transform_speed_and_repeat_cycles():
    a, p, tf = fixture()
    raw = p.xy_m
    assert accept(a, p, tf)
    r = poll(a)
    assert r.status == 'PREPARED_NOT_CONTROL_AUTHORIZED'
    assert np.allclose(r.xy_m[0], (10., 20.))
    assert np.allclose(r.xy_m[-1], (10., 22.))
    assert np.allclose(r.yaw_rad, math.pi / 2)
    assert r.speed_mps[-1] == 0.
    assert 0 < r.speed_mps[0] <= .3
    assert np.all(np.diff(r.speed_mps) <= 1e-12)
    assert r.raw_reference_difference_m < 1e-12
    assert r.speed_source == 'EXPLICIT_TRIAL_POLICY_NOT_MODEL_SPEED'
    assert p.xy_m == raw
    assert poll(a, 1.05) is r and poll(a, 1.1) is r
    assert r.source_s == 1. and r.generated_mono_s == 50.


@pytest.mark.parametrize('change,reason', [
    ({'xy_m': ()}, 'V4_OUTPUT_CONTRACT_MISMATCH'),
    ({'source': 'RVIZ_PATH'}, 'V4_OUTPUT_CONTRACT_MISMATCH'),
    ({'speed_mps': 1.}, 'V4_SPEED_CONTRACT_MISMATCH'),
    ({'xy_m': tuple((float('nan'), 0.) for _ in range(20))}, 'PATH_NONFINITE'),
    ({'expires_s': .9}, 'PLAN_STALE'),
])
def test_reject_clears_previous(change, reason):
    a, p, tf = fixture()
    assert accept(a, p, tf)
    p2 = replace(p, id='p2', source_s=1.1, **change)
    assert not accept(a, p2, replace(tf, plan_id='p2', stamp_s=1.1), 1.1)
    assert a.reason == reason
    assert poll(a, 1.1) is None


@pytest.mark.parametrize('now,epoch,reason', [(3., '0', 'PLAN_STALE'),
                                          (.9, '0', 'CLOCK_RESET'),
                                          (1.1, '1', 'CLOCK_RESET')])
def test_expiry_reset_no_resurrection(now, epoch, reason):
    a, p, tf = fixture()
    assert accept(a, p, tf)
    assert poll(a, now, epoch) is None
    assert a.reason == reason
    assert poll(a, max(now, 1.2)) is None


def test_unknown_limits_and_pose_frame_disabled():
    a, p, tf = fixture()
    assert not a.accept(p, tf, pose_frame='map', now_s=1.)
    assert a.reason == 'FIXED_FRAME_UNKNOWN'
    for limits in (None, a._bridge.limits):
        disabled = V4PPReferenceAdapter(limits, fixed_frame='local')
        assert not accept(disabled, p, tf)
        assert disabled.reason == 'VEHICLE_CONTRACT_UNKNOWN'


@pytest.mark.parametrize('sign', [-1, 1])
def test_gentle_arc(sign):
    a, p, tf = fixture()
    t = np.linspace(0., .25, 20)
    p = replace(p, xy_m=tuple(map(tuple, np.c_[8*np.sin(t), sign*8*(1-np.cos(t))])))
    assert accept(a, p, tf)
    r = poll(a)
    assert all(math.isfinite(x) for x in r.yaw_rad)
    assert r.speed_mps[-1] == 0.
