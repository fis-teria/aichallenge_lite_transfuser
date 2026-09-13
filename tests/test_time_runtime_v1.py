"""Causal online/offline equivalence and ROS-independent runtime boundaries."""
from dataclasses import fields
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from aic_transfuser_lite.data.mcap_converter_v2 import TimedImage, TimedLidar, TimedVelocity
from aic_transfuser_lite.data.time_dataset_v1 import assemble_time_inputs
from aic_transfuser_lite.runtime.time_runtime_v1 import TimeInputBuffer, decode_ros_image, TimeRuntimeModel
from aic_transfuser_lite.training.time_config_v1 import TimeModelConfig


def populated():
    config = TimeModelConfig(image_height=8, image_width=12, lidar_points=8)
    buffer = TimeInputBuffer(config, run_id="online")
    buffer.on_clock(0)
    for i in range(15):
        t = i * 100_000_000
        buffer.on_clock(t)
        receipt = 10**12 + t
        buffer.add("velocity", TimedVelocity(t, .1 * i, 0., 0.), received_ns=receipt)
        buffer.add("lidar", TimedLidar(t, np.full(8, 10.), -np.pi, np.pi/4, .1, 25.), received_ns=receipt)
        buffer.add("camera", TimedImage(t, np.full((8, 12, 3), i, dtype=np.uint8)), received_ns=receipt)
    return buffer


def test_online_materialization_exactly_matches_shared_offline_builder():
    buffer = populated()
    assert buffer.freeze_latest(10**12 + 1_400_000_000 + 49_999_999).anchor.capture_ns == 1_300_000_000
    snapshot = buffer.freeze_latest(10**12 + 1_450_000_000)
    actual, provenance = snapshot.assemble(buffer.config)
    expected, refs = assemble_time_inputs(snapshot.events, snapshot.anchor, config=buffer.config,
        epoch_start_ns=snapshot.epoch_start_ns, epoch_end_ns=snapshot.epoch_end_ns, freeze_ns=snapshot.freeze_ns)
    for field in fields(actual):
        a = getattr(actual, field.name)
        if isinstance(a, torch.Tensor):
            torch.testing.assert_close(a, getattr(expected, field.name), rtol=0, atol=0)
    assert provenance == refs and actual.targets is None
    assert actual.image.shape == (1, 4, 3, 8, 12)
    assert actual.lidar.shape == (1, 4, 2, 8)
    assert actual.ego.shape == (1, 10, 4)
    assert not actual.command_mask.any()
    assert {s["available_clock"] for r in refs for s in r["sources"]} == {"monotonic"}


def test_late_duplicate_excluded_even_when_worker_starts_after_arrival():
    buffer = populated()
    freeze = 10**12 + 1_450_000_000
    buffer.add("velocity", TimedVelocity(1_400_000_000, 99., 0., 0.), received_ns=freeze + 1)
    snapshot = buffer.freeze_latest(freeze + 1_000_000)
    batch, refs = snapshot.assemble(buffer.config)
    assert batch.ego[0, -1, 0].item() == pytest.approx(1.4)
    assert all(s["available_ns"] <= freeze for r in refs for s in r["sources"])
    buffer.add("velocity", TimedVelocity(1_400_000_000, 88., 0., 0.), received_ns=freeze + 2)
    after, _ = snapshot.assemble(buffer.config)
    torch.testing.assert_close(batch.ego, after.ego, rtol=0, atol=0)


def test_clock_reset_discards_history_pending_and_rejects_old_epoch_capture():
    buffer = populated()
    assert buffer.on_clock(0)
    assert buffer.epoch == "1" and not buffer.pending
    assert all(not queue for queue in buffer.events.values())
    assert buffer.freeze_latest(10**13) is None
    with pytest.raises(ValueError, match="CAPTURE_OUTSIDE"):
        buffer.add("velocity", TimedVelocity(1_400_000_000, 1., 0., 0.), received_ns=10**13)
    with pytest.raises(ValueError, match="UNSUPPORTED_ROLE"):
        buffer.add("pose", object(), received_ns=10**13)


def test_image_padding_bgr_and_invalid_stride():
    message = SimpleNamespace(encoding="bgr8", width=2, height=2, step=8,
        data=bytes([1, 2, 3, 4, 5, 6, 99, 99, 7, 8, 9, 10, 11, 12, 99, 99]))
    rgb = decode_ros_image(message)
    np.testing.assert_array_equal(rgb, [[[3, 2, 1], [6, 5, 4]], [[9, 8, 7], [12, 11, 10]]])
    message.step = 5
    with pytest.raises(ValueError, match="STRIDE"):
        decode_ros_image(message)
    message.encoding = "mono16"
    with pytest.raises(ValueError, match="ENCODING"):
        decode_ros_image(message)


def test_checkpoint_hash_checked_before_unpickling(tmp_path):
    checkpoint = tmp_path / "not-a-model.pt"
    checkpoint.write_bytes(b"not a pickle")
    with pytest.raises(ValueError, match="CHECKPOINT_SHA_MISMATCH"):
        TimeRuntimeModel(checkpoint, expected_sha256="0" * 64, device="cpu")


def test_pending_and_history_memory_are_bounded():
    buffer = populated()
    for i in range(400):
        t = 2_000_000_000 + i * 10_000_000
        buffer.on_clock(t)
        buffer.add("camera", TimedImage(t, np.zeros((2, 2, 3), np.uint8)), received_ns=t + 10**12)
    assert len(buffer.events["camera"]) == 32 and len(buffer.pending) == 8
    assert buffer.dropped_anchors > 0
