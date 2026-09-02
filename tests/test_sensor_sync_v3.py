from __future__ import annotations

import pytest

from aic_transfuser_lite.runtime.sensor_sync import SettledCameraSynchronizer


def make_sync(*, queue_size: int = 4) -> SettledCameraSynchronizer[str, str]:
    return SettledCameraSynchronizer(
        required_roles=("lidar", "velocity", "steering"),
        queue_size=queue_size,
        max_skew_sec=0.03,
    )


def test_settled_sync_waits_for_every_stream_to_cross_camera_stamp() -> None:
    sync = make_sync()
    sync.add_sensor("lidar", 0.96, "lidar-old")
    sync.add_sensor("velocity", 0.99, "velocity-old")
    sync.add_sensor("steering", 0.98, "steering-old")
    assert sync.add_camera(1.0, "camera") is None
    assert sync.pop_ready() is None

    sync.add_sensor("lidar", 1.02, "lidar-new")
    sync.add_sensor("velocity", 1.01, "velocity-new")
    assert sync.pop_ready() is None
    sync.add_sensor("steering", 1.005, "steering-new")

    settled = sync.pop_ready()
    assert settled is not None
    assert settled.camera == "camera"
    assert settled.decision.accepted
    assert settled.decision.samples == {
        "lidar": "lidar-new",
        "velocity": "velocity-old",
        "steering": "steering-new",
    }
    assert settled.decision.max_skew_sec == pytest.approx(0.03)
    assert sync.pending_camera_count == 0


def test_settled_sync_rejects_final_cross_sensor_span_over_limit() -> None:
    sync = make_sync()
    assert sync.add_camera(1.0, "camera") is None
    sync.add_sensor("lidar", 1.001, "lidar")
    sync.add_sensor("velocity", 1.032, "velocity")
    sync.add_sensor("steering", 1.015, "steering")

    settled = sync.pop_ready()
    assert settled is not None
    assert not settled.decision.accepted
    assert settled.decision.reason == "sensor_skew"
    assert settled.decision.samples == {}
    assert settled.decision.max_skew_sec == pytest.approx(0.032)
    assert sync.pop_ready() is None


def test_settled_sync_reports_bounded_camera_queue_overflow() -> None:
    sync = make_sync(queue_size=2)
    assert sync.add_camera(1.0, "camera-1") is None
    assert sync.add_camera(1.1, "camera-2") is None
    dropped = sync.add_camera(1.2, "camera-3")
    assert dropped is not None
    assert dropped.stamp_sec == pytest.approx(1.0)
    assert dropped.value == "camera-1"
    assert sync.pending_camera_count == 2


def test_settled_sync_rejects_non_increasing_camera_and_sensor_stamps() -> None:
    sync = make_sync()
    sync.add_camera(1.0, "camera")
    with pytest.raises(ValueError, match="camera timestamps"):
        sync.add_camera(1.0, "duplicate-camera")

    sync.add_sensor("lidar", 1.0, "lidar")
    with pytest.raises(ValueError, match="lidar timestamps"):
        sync.add_sensor("lidar", 0.9, "older-lidar")


def test_settled_sync_rejects_invalid_configuration_and_unknown_role() -> None:
    with pytest.raises(ValueError, match="queue_size"):
        make_sync(queue_size=0)
    sync = make_sync()
    with pytest.raises(KeyError, match="unknown sync role"):
        sync.add_sensor("camera", 1.0, "not-a-sensor-role")
