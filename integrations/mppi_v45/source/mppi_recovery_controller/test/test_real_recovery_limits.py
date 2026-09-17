from dataclasses import replace

import pytest

from mppi_recovery_controller.core import Ego, RecoveryAdapter, RecoveryConfig, ReferenceGeometry, ReferencePoint


def adapter():
    cfg = replace(RecoveryConfig(), require_steering_feedback=True,
                  maximum_recovery_propulsion_mps2=0.5,
                  reverse_speed_mps=-0.5, forward_speed_mps=0.5)
    return RecoveryAdapter(cfg, drive_gear=2, reverse_gear=20)


def tick(a, time, measured=None, speed=0.):
    ref = ReferenceGeometry([ReferencePoint(x, 0, 0, 10, -6, 6) for x in range(-20, 31)], closed=False)
    r = a.prepare(now_sec=time, ego=Ego(0, 0, 0, speed), reference=ref, measured_tire_angle_rad=measured)
    a.commit(r)
    return r


def test_reverse_from_rest_has_gear_relative_propulsion_bound():
    a = adapter()
    tick(a, 0.0, 0.0)
    r = tick(a, 4.01, 0.0)
    assert r.update.after.state == 'BACKING_UP'
    assert r.decision.published_speed_mps == 0.5
    assert r.decision.published_acceleration_mps2 == 0.5


@pytest.mark.parametrize('measured', [None, 0.2, float('nan')])
def test_missing_or_unsettled_steering_holds_propulsion_and_move_clock(measured):
    a = adapter()
    tick(a, 0.0, measured)
    r = tick(a, 4.01, measured)
    assert r.decision.published_speed_mps == 0.0
    assert r.decision.published_acceleration_mps2 == 0.0
    for t in (4.1, 4.5, 5., 5.5, 6., 6.5, 7., 7.5):
        r = tick(a, t, measured)
        assert r.update.after.state == 'BACKING_UP'
        assert r.decision.published_speed_mps == 0.0
    r = tick(a, 7.51, 0.0)
    assert r.decision.published_speed_mps == 0.5
    assert r.decision.published_acceleration_mps2 == 0.5


def test_reverse_braking_is_not_changed_into_propulsion_or_erased():
    a = adapter()
    tick(a, 0.0, 0.0)
    tick(a, 4.01, 0.0)
    r = tick(a, 4.1, 0.0, speed=-0.8)
    assert r.decision.published_acceleration_mps2 < 0.0
    assert r.decision.published_acceleration_mps2 == -a.config.acceleration_max_mps2


def test_forward_recovery_obeys_same_propulsion_bound():
    a = adapter()
    a.recovery.state = 'ALIGNING_FORWARD'
    a.recovery.move_since = 0.0
    r = tick(a, 0.1, 0.0)
    assert r.decision.published_speed_mps == 0.5
    assert r.decision.published_acceleration_mps2 == 0.5
