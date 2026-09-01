import numpy as np

from aic_transfuser_lite.control.safety_supervisor import (
    SafetyConfig,
    SensorStamps,
    apply_safety,
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
