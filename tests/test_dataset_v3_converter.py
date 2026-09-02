from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from aic_transfuser_lite.data.canonical_converter_v3 import (
    _index_run_streams,
    _interpolate_pose_indexed,
    convert_decoded_run_v3,
    load_dataset_v3_converter_config,
    write_prepared_dataset_v3,
)
from aic_transfuser_lite.data.canonical_schema_v3 import MissingReason
from aic_transfuser_lite.data.clock_segments import ClockEpoch
from aic_transfuser_lite.data.mcap_converter_v2 import (
    RunStreams,
    TimedCommand,
    TimedGear,
    TimedImage,
    TimedLidar,
    TimedPose,
    TimedVelocity,
    interpolate_pose,
)
from aic_transfuser_lite.data.storage_v3 import validate_complete_dataset


CONFIG = Path(__file__).parents[1] / "configs" / "data" / "dataset_v3.yaml"
STEP_NS = 100_000_000


def _streams(duration_steps: int = 45) -> RunStreams:
    stamps = [index * STEP_NS for index in range(duration_steps + 1)]
    return RunStreams(
        images=tuple(
            TimedImage(stamp, np.full((4, 6, 3), index, dtype=np.uint8))
            for index, stamp in enumerate(stamps)
        ),
        lidars=tuple(
            TimedLidar(
                stamp,
                np.array([1.0, np.inf, 3.0, 4.0], dtype=np.float32),
                -0.3,
                0.2,
                0.1,
                10.0,
                "lidar",
            )
            for stamp in stamps
        ),
        poses=tuple(TimedPose(stamp, stamp / 1e9, 0.0, 0.0, "map", "base_link") for stamp in stamps),
        velocities=tuple(TimedVelocity(stamp, 1.0, 0.0, 0.0) for stamp in stamps),
        actual_steering=(),
        nominal_commands=tuple(TimedCommand(stamp, 1.0, 0.0, 0.1) for stamp in stamps),
        final_commands=tuple(TimedCommand(stamp, 1.0, 0.0, 0.1) for stamp in stamps),
        gears=tuple(TimedGear(stamp, 2) for stamp in stamps),
        topic_types={},
        timestamp_fallback_counts={},
    )


def _epoch(last_stamp: int = 4_500_000_000) -> ClockEpoch:
    return ClockEpoch("epoch0000", 0, 45, 0, last_stamp, 0, last_stamp, None)


def _convert(streams: RunStreams | None = None, epochs=None):
    return convert_decoded_run_v3(
        streams or _streams(),
        run_id="run01",
        scenario_id="scenario01",
        source_uri="file:///bag/run01",
        source_hash="a" * 64,
        topic_profile_id="default",
        epochs=epochs or (_epoch(),),
        config=load_dataset_v3_converter_config(CONFIG),
    )


def test_converter_generates_dense_measured_pose_future_and_native_lidar() -> None:
    result = _convert()
    sample = result.samples[0]
    future = sample.sample.future_state
    assert future is not None
    assert future.relative_time_sec.shape == (30,)
    assert future.relative_time_sec[0] == pytest.approx(0.1)
    assert future.relative_time_sec[-1] == pytest.approx(3.0)
    assert future.valid.all()
    assert future.x_m == pytest.approx(future.relative_time_sec)
    assert future.y_m == pytest.approx(np.zeros(30))
    assert sample.lidar_ranges_m.shape == (4,)
    assert sample.lidar_valid.tolist() == [1, 0, 1, 1]
    assert sample.lidar_ranges_m[1] == pytest.approx(10.0)
    assert sample.sample.lidar.points == 4
    assert sample.sample.provenance.labels["future_state"].provenance.value == "measured_pose"


def test_missing_actual_steering_is_nan_and_valid_false() -> None:
    steering = _convert().samples[0].sample.ego_state.actual_steering_rad
    assert not steering.valid and np.isnan(steering.value)
    assert steering.missing_reason is MissingReason.NOT_RECORDED


def test_indexed_pose_interpolation_matches_frozen_v2_math() -> None:
    streams = _streams(duration_steps=2)
    target_ns = STEP_NS // 2
    expected_pose, expected_timing = interpolate_pose(
        streams.poses, target_ns, tolerance_ms=50.0
    )
    indexed_pose, indexed_timing = _interpolate_pose_indexed(
        _index_run_streams(streams).poses, target_ns, tolerance_ms=50.0
    )
    assert indexed_pose == expected_pose
    assert indexed_timing == expected_timing


def test_future_state_never_crosses_clock_epoch() -> None:
    epoch = ClockEpoch("epoch0000", 0, 10, 0, 1_000_000_000, 0, 1_000_000_000, None)
    result = _convert(epochs=(epoch,))
    last = result.samples[-1].sample.future_state
    assert last is not None
    assert not last.valid.any()
    assert np.isnan(last.x_m).all()


def test_lidar_geometry_drift_fails_closed() -> None:
    streams = _streams()
    changed = list(streams.lidars)
    changed[-1] = TimedLidar(
        changed[-1].timestamp_ns,
        changed[-1].ranges_m,
        changed[-1].angle_min_rad,
        0.25,
        changed[-1].range_min_m,
        changed[-1].range_max_m,
        changed[-1].frame_id,
    )
    with pytest.raises(ValueError, match="geometry changed"):
        _convert(RunStreams(**{**streams.__dict__, "lidars": tuple(changed)}))


def test_prepared_dataset_is_written_atomically(tmp_path: Path) -> None:
    prepared = _convert()
    output = tmp_path / "dataset"
    summary = write_prepared_dataset_v3(
        output,
        dataset_id="dataset01",
        topic_profile_id="default",
        runs=(prepared,),
        jpeg_quality=90,
    )
    manifest = validate_complete_dataset(output)
    assert manifest["manifest_sha256"] == summary.manifest_sha256
    assert (output / "samples.csv").is_file()
    assert len(list((output / "trajectories" / "run01").glob("*.npy"))) == len(
        prepared.samples
    )


def test_converter_config_rejects_non_dense_initial_contract(tmp_path: Path) -> None:
    text = CONFIG.read_text(encoding="utf-8").replace("future_step_sec: 0.1", "future_step_sec: 0.2")
    path = tmp_path / "bad.yaml"
    path.write_text(text, encoding="utf-8")
    with pytest.raises(ValueError, match="future_step_sec"):
        load_dataset_v3_converter_config(path)
