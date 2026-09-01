import numpy as np

from aic_transfuser_lite.control.safety_supervisor import (
    SafetyConfig,
    SensorStamps,
    apply_safety,
    clamp_command_envelope,
    stopping_distance,
)
from aic_transfuser_lite.control.waypoint_controller import ControlCommand


def fresh(now: float) -> SensorStamps:
    return SensorStamps(camera_sec=now, lidar_sec=now, ego_sec=now)


def test_stopping_distance_increases_with_speed() -> None:
    low = stopping_distance(1.0, 0.2, 3.0, 0.5)
    high = stopping_distance(5.0, 0.2, 3.0, 0.5)
    assert high > low


def test_front_obstacle_forces_brake() -> None:
    now = 100.0
    scan = np.full(181, 30.0, dtype=np.float32)
    scan[90] = 1.0
    decision = apply_safety(
        ControlCommand(0.1, 1.0),
        speed_mps=5.0,
        lidar_ranges_m=scan,
        angle_min_rad=-np.pi / 2,
        angle_increment_rad=np.pi / 180,
        stop_probability=0.1,
        confidence=1.0,
        stamps=fresh(now),
        now_sec=now,
    )
    assert decision.overridden
    assert decision.reason == "front_obstacle_inside_stopping_distance"
    assert decision.command.acceleration_mps2 < 0.0


def test_timeout_forces_brake() -> None:
    now = 100.0
    scan = np.full(181, 30.0, dtype=np.float32)
    stamps = SensorStamps(camera_sec=99.0, lidar_sec=100.0, ego_sec=100.0)
    decision = apply_safety(
        ControlCommand(0.0, 0.5),
        speed_mps=1.0,
        lidar_ranges_m=scan,
        angle_min_rad=-np.pi / 2,
        angle_increment_rad=np.pi / 180,
        stop_probability=0.0,
        confidence=1.0,
        stamps=stamps,
        now_sec=now,
        config=SafetyConfig(camera_timeout_sec=0.3),
    )
    assert decision.reason == "camera_timeout"


def test_clear_path_passes_nominal_command() -> None:
    now = 100.0
    scan = np.full(181, 30.0, dtype=np.float32)
    nominal = ControlCommand(0.1, 0.5)
    decision = apply_safety(
        nominal,
        speed_mps=1.0,
        lidar_ranges_m=scan,
        angle_min_rad=-np.pi / 2,
        angle_increment_rad=np.pi / 180,
        stop_probability=0.0,
        confidence=1.0,
        stamps=fresh(now),
        now_sec=now,
    )
    assert not decision.overridden
    assert decision.command == nominal


def test_disabled_model_stop_ignores_untrained_probability() -> None:
    now = 100.0
    scan = np.full(181, 30.0, dtype=np.float32)
    nominal = ControlCommand(0.1, 0.5)
    decision = apply_safety(
        nominal,
        speed_mps=1.0,
        lidar_ranges_m=scan,
        angle_min_rad=-np.pi / 2,
        angle_increment_rad=np.pi / 180,
        stop_probability=1.0,
        confidence=1.0,
        stamps=fresh(now),
        now_sec=now,
        config=SafetyConfig(enable_model_stop=False),
    )
    assert not decision.overridden
    assert decision.reason == "normal"
    assert decision.command == nominal


def test_disabled_model_stop_does_not_require_head_output() -> None:
    now = 100.0
    scan = np.full(181, 30.0, dtype=np.float32)
    nominal = ControlCommand(0.1, 0.5)
    decision = apply_safety(
        nominal,
        speed_mps=1.0,
        lidar_ranges_m=scan,
        angle_min_rad=-np.pi / 2,
        angle_increment_rad=np.pi / 180,
        stop_probability=None,
        confidence=1.0,
        stamps=fresh(now),
        now_sec=now,
        config=SafetyConfig(enable_model_stop=False),
    )
    assert not decision.overridden
    assert decision.reason == "normal"


def test_enabled_model_stop_preserves_fail_safe_behavior() -> None:
    now = 100.0
    scan = np.full(181, 30.0, dtype=np.float32)
    decision = apply_safety(
        ControlCommand(0.1, 0.5),
        speed_mps=1.0,
        lidar_ranges_m=scan,
        angle_min_rad=-np.pi / 2,
        angle_increment_rad=np.pi / 180,
        stop_probability=1.0,
        confidence=1.0,
        stamps=fresh(now),
        now_sec=now,
        config=SafetyConfig(enable_model_stop=True),
    )
    assert decision.overridden
    assert decision.reason == "model_stop"
    assert decision.command.acceleration_mps2 < 0.0


def test_enabled_model_stop_fails_safe_when_output_is_missing() -> None:
    now = 100.0
    scan = np.full(181, 30.0, dtype=np.float32)
    decision = apply_safety(
        ControlCommand(0.1, 0.5),
        speed_mps=1.0,
        lidar_ranges_m=scan,
        angle_min_rad=-np.pi / 2,
        angle_increment_rad=np.pi / 180,
        stop_probability=None,
        confidence=1.0,
        stamps=fresh(now),
        now_sec=now,
        config=SafetyConfig(enable_model_stop=True),
    )
    assert decision.overridden
    assert decision.reason == "model_stop_missing"
    assert decision.command.acceleration_mps2 < 0.0


def test_zero_valid_front_beams_are_not_clear() -> None:
    now = 100.0
    decision = apply_safety(
        ControlCommand(0.0, 0.5), speed_mps=1.0,
        lidar_ranges_m=np.full(181, np.nan, dtype=np.float32),
        angle_min_rad=-np.pi / 2, angle_increment_rad=np.pi / 180,
        stop_probability=None, confidence=1.0, stamps=fresh(now), now_sec=now,
    )
    assert decision.overridden
    assert decision.reason == "lidar_no_valid_front_beams"


def test_strict_safety_requires_explicit_now() -> None:
    with np.testing.assert_raises_regex(ValueError, "explicit now_sec"):
        apply_safety(
            ControlCommand(0.0, 0.0), speed_mps=0.0,
            lidar_ranges_m=np.ones(3), angle_min_rad=-0.1, angle_increment_rad=0.1,
            stop_probability=None, confidence=None, stamps=fresh(1.0), now_sec=None,
        )


def test_future_sensor_timestamp_forces_brake() -> None:
    decision = apply_safety(
        ControlCommand(0.0, 0.5), speed_mps=1.0, lidar_ranges_m=np.ones(3) * 30.0,
        angle_min_rad=-0.1, angle_increment_rad=0.1, stop_probability=None,
        confidence=1.0, stamps=SensorStamps(101.0, 100.0, 100.0), now_sec=100.0,
    )
    assert decision.overridden
    assert decision.reason == "camera_future_timestamp"


def test_speed_and_validity_deadline_are_clamped() -> None:
    envelope = clamp_command_envelope(
        proposed_speed_mps=99.0, source_observation_stamp_sec=10.0,
        generated_stamp_sec=10.05, requested_valid_until_sec=20.0, now_sec=10.05,
        config=SafetyConfig(max_speed_mps=8.0, max_command_validity_sec=0.1),
    )
    assert envelope.speed_mps == 8.0
    assert envelope.valid_until_sec == 10.15


def test_late_command_is_rejected_not_forwarded() -> None:
    with np.testing.assert_raises_regex(TimeoutError, "deadline_missed"):
        clamp_command_envelope(
            proposed_speed_mps=1.0, source_observation_stamp_sec=9.0,
            generated_stamp_sec=9.1, requested_valid_until_sec=9.2, now_sec=10.0,
        )
