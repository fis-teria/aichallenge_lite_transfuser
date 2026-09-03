from __future__ import annotations

import pytest

from aic_transfuser_lite.runtime.m3_acceptance import (
    TimedPlanDiagnosticV3,
    TimedScalarV3,
    summarize_m3_interval_v3,
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
            TimedPlanDiagnosticV3(10.1, True, 0.75, 0.4, "launching", None),
            TimedPlanDiagnosticV3(13.9, True, 0.75, 0.1, "moving", None),
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
                10.1, True, 0.75, -4.0, "response_fault", "launch_response_missing"
            )
        ],
        collision_true_count=1,
    )
    assert result["launch_pass"] is False
    assert result["speed_cap_pass"] is False
    assert result["collision_clear"] is False
    assert result["controller_fault_counts"] == {"launch_response_missing": 1}


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
