"""Issue #3 temporal future-target and cache contract tests."""
from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest
import torch

from aic_transfuser_lite.data.future_sequence_dataset_v1 import (
    FutureSequenceCache,
    FutureSequenceConfig,
    TemporalTrainingDataset,
    build_future_sequence_targets,
    build_temporal_training_sample,
    cache_identity,
)
from aic_transfuser_lite.data.mcap_converter_v2 import TimedLidar
from test_time_dataset_p1 import cfg, ev, fixture


def _setup():
    events, anchor = fixture(steering=True)
    return events, anchor, cfg(), FutureSequenceConfig(), (0, 5_500_000_000)


def test_policy_input_is_independent_from_future_observation_and_action():
    events, anchor, dataset_config, future_config, bounds = _setup()
    first = build_temporal_training_sample(
        events,
        anchor,
        dataset_config=dataset_config,
        future_config=future_config,
        epoch_bounds=bounds,
        freeze_ns=2_000_000_001,
        environment_stop_intent=False,
    )
    changed = []
    for event in events:
        if event.capture_ns > anchor.capture_ns and event.role == "camera":
            changed.append(replace(event, payload=replace(event.payload, image_rgb=np.full((2, 3, 3), 255, np.uint8))))
        elif event.capture_ns > anchor.capture_ns and event.role == "final_command":
            changed.append(replace(event, payload=replace(event.payload, steering_rad=0.5)))
        else:
            changed.append(event)
    second = build_temporal_training_sample(
        changed,
        anchor,
        dataset_config=dataset_config,
        future_config=future_config,
        epoch_bounds=bounds,
        freeze_ns=2_000_000_001,
        environment_stop_intent=False,
    )
    assert first.policy.inputs is not None and first.policy.inputs.targets is None
    for field in ("image", "lidar", "ego", "command_history"):
        torch.testing.assert_close(getattr(first.policy.inputs, field), getattr(second.policy.inputs, field), rtol=0, atol=0)
    assert not torch.equal(first.future.image, second.future.image)
    assert not torch.equal(first.future.applied_action, second.future.applied_action)
    assert first.target_speed_mask.item() and first.stop_mask.item()


def test_future_masks_run_end_missing_data_and_invalid_lidar():
    events, anchor, dataset_config, _, _ = _setup()
    config = FutureSequenceConfig(horizons_sec=(0.5, 1.0), tolerance_ns=0)
    invalid_stamp = anchor.capture_ns + 500_000_000
    invalid = TimedLidar(invalid_stamp, np.asarray([np.nan, np.inf, -1.0, 1.0]), -np.pi, np.pi / 2, 0.1, 25.0)
    events = [replace(event, payload=invalid) if event.role == "lidar" and event.capture_ns == invalid_stamp else event for event in events]
    result = build_future_sequence_targets(
        events,
        anchor,
        dataset_config=dataset_config,
        future_config=config,
        epoch_bounds=(0, anchor.capture_ns + 500_000_000),
    )
    assert result.image_mask.tolist() == [True, False]
    assert result.lidar_mask.tolist() == [True, False]
    assert result.applied_action_mask.tolist() == [True, False]
    assert torch.isfinite(result.lidar).all()
    assert result.source_timestamp_ns[1].tolist() == [-1, -1, -1, -1]


def test_stop_requires_explicit_annotation_and_target_speed_has_mask():
    events, anchor, dataset_config, future_config, bounds = _setup()
    unknown = build_temporal_training_sample(
        events,
        anchor,
        dataset_config=dataset_config,
        future_config=future_config,
        epoch_bounds=bounds,
        freeze_ns=2_000_000_001,
    )
    stopped = build_temporal_training_sample(
        events,
        anchor,
        dataset_config=dataset_config,
        future_config=future_config,
        epoch_bounds=bounds,
        freeze_ns=2_000_000_001,
        environment_stop_intent=True,
    )
    assert unknown.target_speed_mask.item() and unknown.target_speed_mps.item() == pytest.approx(2.0)
    assert not unknown.stop_mask.item() and unknown.stop_target.item() == 0.0
    assert stopped.stop_mask.item() and stopped.stop_target.item() == 1.0


def test_cache_reuses_exact_identity_and_rejects_changed_source_or_preprocess(tmp_path):
    events, anchor, dataset_config, future_config, bounds = _setup()
    identity = cache_identity(source_sha256="a" * 64, dataset_config=dataset_config, future_config=future_config)
    cache = FutureSequenceCache(tmp_path / "cache", identity)
    calls = 0

    def builder():
        nonlocal calls
        calls += 1
        return build_future_sequence_targets(
            events,
            anchor,
            dataset_config=dataset_config,
            future_config=future_config,
            epoch_bounds=bounds,
        )

    first, reused_first = cache.get_or_build("run:ep:2000000000", builder)
    second, reused_second = cache.get_or_build("run:ep:2000000000", builder)
    assert calls == 1 and not reused_first and reused_second
    torch.testing.assert_close(first.image, second.image, rtol=0, atol=0)
    # A later process constructs a new cache object from the same typed config.
    # Tuples in that config must compare equal to lists in its JSON manifest.
    reopened = FutureSequenceCache(tmp_path / "cache", identity)
    third, reused_third = reopened.get_or_build("run:ep:2000000000", builder)
    assert calls == 1 and reused_third
    torch.testing.assert_close(first.image, third.image, rtol=0, atol=0)
    changed_source = cache_identity(source_sha256="b" * 64, dataset_config=dataset_config, future_config=future_config)
    with pytest.raises(ValueError, match="another source/config"):
        FutureSequenceCache(tmp_path / "cache", changed_source)
    changed_preprocess = cache_identity(
        source_sha256="a" * 64,
        dataset_config=dataset_config,
        future_config=replace(future_config, preprocess_version="time_preprocess_v2"),
    )
    assert changed_preprocess["identity_sha256"] != identity["identity_sha256"]


def test_dataset_does_not_cross_run_epoch_and_validates_shapes():
    events, anchor, dataset_config, future_config, bounds = _setup()
    dataset = TemporalTrainingDataset(
        [(events, anchor, 2_000_000_001)],
        dataset_config=dataset_config,
        future_config=future_config,
        epoch_bounds={("run", "ep"): bounds},
    )
    sample = dataset[0]
    sample.future.validate(image_shape=dataset_config.image_shape, lidar_shape=dataset_config.lidar_shape)
    with pytest.raises(ValueError, match="epoch bounds missing"):
        TemporalTrainingDataset(
            [(events, replace(anchor, run="other"), 2_000_000_001)],
            dataset_config=dataset_config,
            future_config=future_config,
            epoch_bounds={("run", "ep"): bounds},
        )[0]


def test_dataset_preserves_collection_intervention_without_inventing_stop():
    events, anchor, dataset_config, future_config, bounds = _setup()
    kwargs = dict(dataset_config=dataset_config, future_config=future_config,
                  epoch_bounds={("run", "ep"): bounds})
    records = [(events, anchor, 2_000_000_001)]
    original = TemporalTrainingDataset(records, **kwargs)[0]
    blocked = TemporalTrainingDataset(
        records, **kwargs, intervention_ns={("run", "ep"): anchor.capture_ns + 2_000_000_000},
    )[0]
    assert original.policy.teacher.xy_mask.all()
    assert blocked.policy.inputs is not None
    assert not blocked.policy.teacher.xy_mask.any()
    assert not blocked.policy.teacher.velocity_mask.any()
    assert not blocked.target_speed_mask.item() and not blocked.stop_mask.item()
    assert blocked.policy.stop_reason == "COLLECTION_INTERVENTION"
    # Observation targets remain real observations, even when IL labels are held.
    torch.testing.assert_close(original.future.image, blocked.future.image, rtol=0, atol=0)
    # An intervention in another epoch cannot suppress this run's labels.
    other = TemporalTrainingDataset(records, **kwargs, intervention_ns={("run", "other"): 0})[0]
    assert other.policy.teacher.xy_mask.all()


@pytest.mark.parametrize("invalid", [-1, 1.5, True])
def test_collection_intervention_requires_integer_nanoseconds(invalid):
    events, anchor, dataset_config, future_config, bounds = _setup()
    with pytest.raises(ValueError, match="intervention_ns"):
        build_temporal_training_sample(
            events, anchor, dataset_config=dataset_config, future_config=future_config,
            epoch_bounds=bounds, freeze_ns=2_000_000_001, intervention_ns=invalid,
        )
