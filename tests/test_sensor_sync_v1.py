from __future__ import annotations

import pytest

from aic_transfuser_lite.runtime.sensor_sync import CameraMasterSynchronizer


def make_sync() -> CameraMasterSynchronizer[str]:
    return CameraMasterSynchronizer(
        required_roles=("lidar", "velocity", "steering"),
        queue_size=10,
        max_skew_sec=0.03,
    )


def test_camera_master_selects_closest_samples_and_consumes_them_once() -> None:
    sync = make_sync()
    sync.add("lidar", 0.98, "lidar-old")
    sync.add("lidar", 1.01, "lidar-near")
    sync.add("velocity", 0.995, "velocity")
    sync.add("steering", 1.005, "steering")
    decision = sync.match(1.0)
    assert decision.accepted
    assert decision.reason == "synchronized"
    assert decision.samples == {
        "lidar": "lidar-near",
        "velocity": "velocity",
        "steering": "steering",
    }
    assert decision.deltas_sec["lidar"] == pytest.approx(0.01)
    assert decision.max_skew_sec == pytest.approx(0.015)

    rejected = sync.match(1.1)
    assert not rejected.accepted
    assert rejected.reason.startswith("missing:")


def test_cross_sensor_span_over_30ms_is_rejected_even_if_each_is_close_to_camera() -> None:
    sync = make_sync()
    sync.add("lidar", 0.975, "lidar")
    sync.add("velocity", 1.025, "velocity")
    sync.add("steering", 1.0, "steering")
    decision = sync.match(1.0)
    assert not decision.accepted
    assert decision.reason == "sensor_skew"
    assert decision.max_skew_sec == pytest.approx(0.05)
    assert decision.samples == {}


def test_failed_skew_is_not_final_until_every_stream_reaches_camera_time() -> None:
    sync = make_sync()
    sync.add("lidar", 0.96, "lidar-old")
    sync.add("velocity", 1.01, "velocity")
    sync.add("steering", 1.0, "steering")
    assert not sync.match(1.0).accepted
    assert not sync.all_streams_reached(1.0)

    sync.add("lidar", 1.02, "lidar-new")
    assert sync.all_streams_reached(1.0)
    decision = sync.match(1.0)
    assert decision.accepted
    assert decision.samples["lidar"] == "lidar-new"


def test_exact_30ms_true_span_is_accepted_despite_float_rounding() -> None:
    sync = make_sync()
    sync.add("lidar", 0.985, "lidar")
    sync.add("velocity", 1.015, "velocity")
    sync.add("steering", 1.0, "steering")
    decision = sync.match(1.0)
    assert decision.accepted
    assert decision.max_skew_sec == pytest.approx(0.03)


def test_sync_rejects_unknown_role_invalid_stamp_and_invalid_configuration() -> None:
    sync = make_sync()
    with pytest.raises(KeyError, match="unknown sync role"):
        sync.add("camera", 1.0, "image")
    with pytest.raises(ValueError, match="finite and positive"):
        sync.add("lidar", 0.0, "scan")
    with pytest.raises(ValueError, match="queue_size"):
        CameraMasterSynchronizer(required_roles=("lidar",), queue_size=0, max_skew_sec=0.03)
    with pytest.raises(ValueError, match="max_skew"):
        CameraMasterSynchronizer(required_roles=("lidar",), queue_size=1, max_skew_sec=0.0)
