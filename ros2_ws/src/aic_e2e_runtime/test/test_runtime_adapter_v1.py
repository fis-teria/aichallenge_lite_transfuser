from types import SimpleNamespace

import pytest

from aic_e2e_runtime.runtime_adapter import (
    steering_report_to_angle,
    strict_message_stamp_to_seconds,
    velocity_report_to_state,
)


def stamp(sec: int, nanosec: int = 0):
    return SimpleNamespace(sec=sec, nanosec=nanosec)


def test_strict_stamp_accepts_header_or_direct_stamp_and_rejects_zero() -> None:
    header_message = SimpleNamespace(header=SimpleNamespace(stamp=stamp(12, 500_000_000)))
    direct_message = SimpleNamespace(stamp=stamp(8, 250_000_000))
    assert strict_message_stamp_to_seconds(header_message) == pytest.approx(12.5)
    assert strict_message_stamp_to_seconds(direct_message) == pytest.approx(8.25)
    with pytest.raises(ValueError, match="finite positive"):
        strict_message_stamp_to_seconds(SimpleNamespace(stamp=stamp(0)))


def test_vehicle_reports_keep_longitudinal_sign_yaw_rate_and_actual_steering() -> None:
    velocity = SimpleNamespace(longitudinal_velocity=-2.0, heading_rate=0.3)
    steering = SimpleNamespace(steering_tire_angle=-0.25)
    assert velocity_report_to_state(velocity) == pytest.approx((-2.0, 0.3))
    assert steering_report_to_angle(steering) == pytest.approx(-0.25)
