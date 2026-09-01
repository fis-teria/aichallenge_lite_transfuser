from __future__ import annotations

import math

import numpy as np
import pytest

from aic_transfuser_lite.data.mcap_converter_v2 import (
    RunStreams,
    TimedCommand,
    TimedGear,
    TimedImage,
    TimedLidar,
    TimedPose,
    TimedSteering,
    TimedVelocity,
    V2ConverterConfig,
    calibrate_run_delay,
    prepare_run_samples,
)
from aic_transfuser_lite.data.schema import validate_v2_row


def _streams(*, include_actual_steering: bool = True) -> RunStreams:
    duration_sec = 5.0
    camera_times = np.arange(0.0, duration_sec + 1e-9, 0.1)
    state_times = np.arange(0.0, duration_sec + 1e-9, 0.05)
    images = tuple(
        TimedImage(
            timestamp_ns=int(round(t * 1e9)),
            image_rgb=np.full((4, 6, 3), int(t * 10) % 255, dtype=np.uint8),
            bag_timestamp_ns=int(round(t * 1e9)),
            timestamp_source="header",
        )
        for t in camera_times
    )
    lidars = tuple(
        TimedLidar(
            timestamp_ns=int(round((t + 0.005) * 1e9)),
            ranges_m=np.array([1.0, np.inf, 3.0, 4.0], dtype=np.float32),
            angle_min_rad=-1.0,
            angle_increment_rad=0.5,
            range_min_m=0.1,
            range_max_m=10.0,
            frame_id="laser",
            bag_timestamp_ns=int(round((t + 0.005) * 1e9)),
            timestamp_source="header",
        )
        for t in camera_times
    )
    poses = tuple(
        TimedPose(
            timestamp_ns=int(round(t * 1e9)),
            x_world_m=2.0 * t,
            y_world_m=0.0,
            yaw_world_rad=0.0,
            frame_id="map",
            child_frame_id="base_link",
        )
        for t in state_times
    )
    velocities = tuple(
        TimedVelocity(
            timestamp_ns=int(round(t * 1e9)),
            longitudinal_mps=2.0,
            lateral_mps=0.0,
            yaw_rate_rps=0.0,
        )
        for t in state_times
    )
    steering = (
        tuple(
            TimedSteering(int(round(t * 1e9)), steering_rad=0.1)
            for t in state_times
        )
        if include_actual_steering
        else ()
    )
    commands = tuple(
        TimedCommand(
            timestamp_ns=int(round(t * 1e9)),
            speed_mps=2.0,
            acceleration_mps2=0.0,
            steering_rad=0.5,
        )
        for t in state_times
    )
    gears = tuple(TimedGear(int(round(t * 1e9)), gear=1) for t in state_times)
    return RunStreams(
        images=images,
        lidars=lidars,
        poses=poses,
        velocities=velocities,
        actual_steering=steering,
        nominal_commands=commands,
        final_commands=commands,
        gears=gears,
        topic_types={},
        timestamp_fallback_counts={},
    )


def test_v2_samples_use_measured_pose_not_turning_command() -> None:
    result = prepare_run_samples(
        _streams(),
        run_id="run_a",
        scenario_id="normal_course",
        config=V2ConverterConfig(waypoint_times_sec=(0.5, 1.0)),
    )

    assert result.samples
    row = result.samples[0].row
    assert row["wp_0_x"] == pytest.approx(1.0)
    assert row["wp_1_x"] == pytest.approx(2.0)
    assert row["wp_0_y"] == pytest.approx(0.0, abs=1e-7)
    assert row["wp_1_y"] == pytest.approx(0.0, abs=1e-7)
    assert row["final_command_steering_rad"] == pytest.approx(0.5)
    assert row["actual_steering_rad"] == pytest.approx(0.1)
    assert row["label_provenance"] == "measured_pose"
    validate_v2_row(row, num_waypoints=2)


def test_missing_actual_steering_is_nan_with_explicit_invalid_flag() -> None:
    result = prepare_run_samples(
        _streams(include_actual_steering=False),
        run_id="run_no_steer",
        scenario_id="normal_course",
        config=V2ConverterConfig(waypoint_times_sec=(0.5, 1.0)),
    )

    row = result.samples[0].row
    assert row["actual_steering_valid"] == 0
    assert math.isnan(row["actual_steering_rad"])
    validate_v2_row(row, num_waypoints=2)


def test_native_lidar_geometry_and_valid_mask_are_preserved() -> None:
    result = prepare_run_samples(
        _streams(),
        run_id="run_a",
        scenario_id="normal_course",
        config=V2ConverterConfig(waypoint_times_sec=(0.5, 1.0)),
    )

    sample = result.samples[0]
    assert sample.lidar_ranges_m.shape == (4,)
    assert sample.lidar_valid.shape == (4,)
    np.testing.assert_array_equal(sample.lidar_valid, [1, 0, 1, 1])
    assert sample.lidar_ranges_m[1] == pytest.approx(10.0)
    assert result.lidar_geometry["source_points"] == 4
    assert result.lidar_geometry["resampling"] == "none_native_beam_order"
    assert abs(result.samples[0].row["lidar_dt_ms"]) == pytest.approx(5.0)


def test_mixed_native_lidar_beam_count_fails_closed() -> None:
    streams = _streams()
    mixed = list(streams.lidars)
    mixed[1] = TimedLidar(
        timestamp_ns=mixed[1].timestamp_ns,
        ranges_m=np.ones(5, dtype=np.float32),
        angle_min_rad=mixed[1].angle_min_rad,
        angle_increment_rad=mixed[1].angle_increment_rad,
        range_min_m=mixed[1].range_min_m,
        range_max_m=mixed[1].range_max_m,
        frame_id=mixed[1].frame_id,
        bag_timestamp_ns=mixed[1].bag_timestamp_ns,
        timestamp_source=mixed[1].timestamp_source,
    )
    invalid = RunStreams(**{**streams.__dict__, "lidars": tuple(mixed)})

    with pytest.raises(ValueError, match="LiDAR geometry changed"):
        prepare_run_samples(
            invalid,
            run_id="run_bad_lidar",
            scenario_id="normal_course",
            config=V2ConverterConfig(waypoint_times_sec=(0.5, 1.0)),
        )


def test_v2_quality_metrics_report_rate_skew_and_pose_drop_fraction() -> None:
    result = prepare_run_samples(
        _streams(),
        run_id="run_a",
        scenario_id="normal_course",
        config=V2ConverterConfig(waypoint_times_sec=(0.5, 1.0)),
    )

    metrics = result.quality_metrics
    assert 9.8 <= metrics["effective_sample_rate_hz"] <= 10.2
    assert metrics["camera_lidar_p95_skew_ms"] < 30.0
    assert metrics["pose_velocity_missing_fraction"] < 0.01
    assert metrics["waypoint_provenance"] == "measured_pose"


def test_delay_calibration_rejects_non_contiguous_state_coverage() -> None:
    streams = _streams()
    sparse_velocity = (
        TimedVelocity(0, 2.0, 0.0, 0.0),
        TimedVelocity(4_950_000_000, 2.0, 0.0, 0.0),
        TimedVelocity(5_000_000_000, 2.0, 0.0, 0.0),
    )
    sparse = RunStreams(**{**streams.__dict__, "velocities": sparse_velocity})

    result = calibrate_run_delay(sparse, V2ConverterConfig())

    assert not result.individual_valid
    assert result.validity_reasons == ("insufficient_contiguous_coverage",)
