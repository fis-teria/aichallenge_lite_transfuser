from __future__ import annotations

import math

import pytest

from aic_transfuser_lite.runtime.m3_acceptance import (
    TimedPlanDiagnosticV3,
    TimedPose2DV3,
    TimedScalarV3,
    TimedTrajectoryPredictionV3,
    summarize_m3_interval_v3,
    summarize_trajectory_tracking_v3,
)


def _summary(**overrides):
    values = {
        "arm_start_sec": 10.0,
        "arm_end_sec": 14.0,
        "velocity_mps": [
            TimedScalarV3(10.0, 0.0),
            TimedScalarV3(10.5, 0.12),
            TimedScalarV3(14.0, 0.74),
        ],
        "yaw_rate_rps": [
            TimedScalarV3(10.1, 0.0),
            TimedScalarV3(11.0, 0.1),
            TimedScalarV3(12.0, -0.1),
        ],
        "plans": [
            TimedPlanDiagnosticV3(
                10.1, True, 0.75, 0.4, "launching", None, False, (), ()
            ),
            TimedPlanDiagnosticV3(
                13.9, True, 0.75, 0.1, "moving", None, False, (), ()
            ),
        ],
        "safety_reasons": ["normal", "normal"],
        "displacement_m": 2.5,
        "collision_topic_present": True,
        "collision_true_count": 0,
        "speed_cap_mps": 0.75,
    }
    return summarize_m3_interval_v3(**{**values, **overrides})


def test_m3_summary_reports_launch_cap_turns_and_collision_evidence() -> None:
    result = _summary()
    assert result["launch_pass"] is True
    assert result["launch_latency_sec"] == pytest.approx(0.5)
    assert result["speed_cap_pass"] is True
    assert result["straight_sample_count"] == 1
    assert result["left_turn_sample_count"] == 1
    assert result["right_turn_sample_count"] == 1
    assert result["collision_clear"] is True
    assert result["controller_fault_counts"] == {}


def test_silent_unmatched_collision_observer_is_not_collision_clearance() -> None:
    result = _summary(collision_topic_present=False)
    assert result["collision_true_count"] == 0
    assert result["collision_clear"] is None


def test_m3_summary_reports_launch_timeout_cap_and_fault() -> None:
    result = _summary(
        velocity_mps=[TimedScalarV3(10.0, 0.0), TimedScalarV3(14.0, 0.9)],
        plans=[
            TimedPlanDiagnosticV3(
                10.1,
                True,
                0.75,
                -4.0,
                "response_fault",
                "launch_response_missing",
                True,
                ("trajectory_invalid",),
                (),
            )
        ],
        collision_true_count=1,
    )
    assert result["launch_pass"] is False
    assert result["speed_cap_pass"] is False
    assert result["collision_clear"] is False
    assert result["controller_fault_counts"] == {"launch_response_missing": 1}
    assert result["stop_required_count"] == 1
    assert result["decision_reason_counts"] == {"trajectory_invalid": 1}


@pytest.mark.parametrize(
    "overrides",
    [
        {"arm_end_sec": 10.0},
        {"velocity_mps": []},
        {"plans": []},
        {"collision_true_count": -1},
        {"speed_cap_mps": 0.0},
        {
            "velocity_mps": [
                TimedScalarV3(11.0, 0.1),
                TimedScalarV3(10.0, 0.2),
            ]
        },
    ],
)
def test_m3_summary_rejects_invalid_or_missing_evidence(overrides) -> None:
    with pytest.raises(ValueError):
        _summary(**overrides)


def test_tracking_compares_prediction_in_observation_ego_frame() -> None:
    result = summarize_trajectory_tracking_v3(
        predictions=[TimedTrajectoryPredictionV3(
            observation_time_sec=1.0,
            waypoint_times_sec=(0.5, 1.0, 1.5),
            trajectory_xy_m=((0.5, 0.0), (1.0, 0.0), (1.5, 0.0)),
        )],
        poses=[
            TimedPose2DV3(1.0, 10.0, 20.0, math.pi / 2.0),
            TimedPose2DV3(2.0, 10.0, 21.0, math.pi / 2.0),
        ],
        horizon_sec=1.0,
    )
    assert result["tracking_matched_count"] == 1
    assert result["tracking_coverage_ratio"] == pytest.approx(1.0)
    assert result["tracking_euclidean_error_p95_m"] == pytest.approx(0.0)


def test_tracking_interpolates_pose_and_prediction() -> None:
    result = summarize_trajectory_tracking_v3(
        predictions=[TimedTrajectoryPredictionV3(
            observation_time_sec=0.5,
            waypoint_times_sec=(0.5, 1.5),
            trajectory_xy_m=((0.5, 0.0), (1.5, 0.0)),
        )],
        poses=[
            TimedPose2DV3(0.0, 0.0, 0.0, 0.0),
            TimedPose2DV3(1.0, 1.0, 0.0, 0.0),
            TimedPose2DV3(2.0, 2.0, 0.0, 0.0),
        ],
    )
    assert result["tracking_euclidean_error_p50_m"] == pytest.approx(0.0)


def test_tracking_reports_unmatched_future_pose_as_coverage_gap() -> None:
    result = summarize_trajectory_tracking_v3(
        predictions=[TimedTrajectoryPredictionV3(
            observation_time_sec=2.0,
            waypoint_times_sec=(0.5, 1.0),
            trajectory_xy_m=((0.5, 0.0), (1.0, 0.0)),
        )],
        poses=[TimedPose2DV3(2.0, 0.0, 0.0, 0.0)],
    )
    assert result["tracking_matched_count"] == 0
    assert result["tracking_coverage_ratio"] == 0.0
    assert result["tracking_euclidean_error_p95_m"] is None


@pytest.mark.parametrize(
    "predictions,poses,horizon",
    [
        ([], [], 0.0),
        ([TimedTrajectoryPredictionV3(0.0, (1.0,), ((1.0, 0.0),))], [], 1.0),
        ([TimedTrajectoryPredictionV3(0.0, (1.0, 0.5), ((1.0, 0.0), (0.5, 0.0)))], [], 1.0),
        ([], [TimedPose2DV3(1.0, float("nan"), 0.0, 0.0)], 1.0),
        ([], [TimedPose2DV3(2.0, 0.0, 0.0, 0.0), TimedPose2DV3(1.0, 0.0, 0.0, 0.0)], 1.0),
    ],
)
def test_tracking_rejects_invalid_timing_shape_and_pose(
    predictions, poses, horizon
) -> None:
    with pytest.raises(ValueError):
        summarize_trajectory_tracking_v3(
            predictions=predictions,
            poses=poses,
            horizon_sec=horizon,
        )
