import pytest

from aic_transfuser_lite.evaluation.compare_v3 import (
    paired_run_bootstrap_v3,
    screening_gate_status_v3,
)


def test_paired_bootstrap_resamples_runs_not_frames() -> None:
    baseline = [
        {"sample_id": "a1", "run_id": "a", "ade_m": 1.0},
        {"sample_id": "a2", "run_id": "a", "ade_m": 3.0},
        {"sample_id": "b1", "run_id": "b", "ade_m": 2.0},
    ]
    candidate = [
        {"sample_id": "a1", "run_id": "a", "ade_m": 0.0},
        {"sample_id": "a2", "run_id": "a", "ade_m": 2.0},
        {"sample_id": "b1", "run_id": "b", "ade_m": 4.0},
    ]
    result = paired_run_bootstrap_v3(
        baseline, candidate, metric="ade_m", resamples=1000, seed=42
    )
    assert result["run_count"] == 2
    assert result["run_delta"] == {"a": -1.0, "b": 2.0}
    assert result["paired_run_equal_delta"] == pytest.approx(0.5)


def test_paired_bootstrap_rejects_different_cohorts() -> None:
    with pytest.raises(ValueError, match="cohorts differ"):
        paired_run_bootstrap_v3(
            [{"sample_id": "a", "run_id": "run", "ade_m": 1.0}],
            [{"sample_id": "b", "run_id": "run", "ade_m": 1.0}],
            metric="ade_m",
        )


def test_screening_gate_requires_launch_and_ade_non_regression() -> None:
    baseline = {
        "launch_gate_pass": False,
        "teacher_quality": {"trajectory_waypoint_weighted_ade_m": 0.13},
    }
    candidate = {
        "launch_gate_pass": True,
        "teacher_quality": {"trajectory_waypoint_weighted_ade_m": 0.14},
    }

    result = screening_gate_status_v3(baseline, candidate)

    assert result["candidate_launch_gate_pass"] is True
    assert result["candidate_trajectory_regression_gate_pass"] is False
    assert result["candidate_screening_gate_pass"] is False


def test_screening_gate_reads_legacy_launch_only_key() -> None:
    baseline = {
        "screening_gate_pass": False,
        "teacher_quality": {"trajectory_waypoint_weighted_ade_m": 0.13},
    }
    candidate = {
        "screening_gate_pass": True,
        "teacher_quality": {"trajectory_waypoint_weighted_ade_m": 0.12},
    }

    result = screening_gate_status_v3(baseline, candidate)

    assert result["candidate_launch_gate_pass"] is True
    assert result["candidate_trajectory_regression_gate_pass"] is True
    assert result["candidate_screening_gate_pass"] is True
