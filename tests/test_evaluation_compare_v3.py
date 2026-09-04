import pytest

from aic_transfuser_lite.evaluation.compare_v3 import paired_run_bootstrap_v3


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
