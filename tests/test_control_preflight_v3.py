from __future__ import annotations

import pytest

from aic_transfuser_lite.runtime.control_preflight import (
    evaluate_control_preflight_v3,
)


def _evaluate(**overrides):
    values = {
        "gear_report": 2,
        "control_mode_report": 1,
        "awsim_state": "Start",
        "gear_age_sec": 0.1,
        "control_mode_age_sec": 0.1,
        "awsim_state_age_sec": 0.1,
        "maximum_status_age_sec": 0.5,
        "expected_drive_gear": 2,
        "expected_autonomous_mode": 1,
        "required_awsim_state": "Start",
        "nominal_publishers": 1,
        "nominal_subscribers": 1,
        "final_publishers": 1,
        "final_subscribers": 1,
    }
    return evaluate_control_preflight_v3(**{**values, **overrides})


def test_graneple_drive_autonomous_start_route_is_ready() -> None:
    result = _evaluate()
    assert result.ready
    assert result.reasons == ()


@pytest.mark.parametrize(
    ("overrides", "reason"),
    [
        ({"gear_report": 1}, "gear_not_drive"),
        ({"control_mode_report": 4}, "control_mode_not_autonomous"),
        ({"awsim_state": "Grounded"}, "awsim_not_started"),
        ({"gear_age_sec": 0.6}, "gear_stale"),
        ({"nominal_publishers": 2}, "nominal_publisher_count"),
        ({"final_publishers": 0}, "final_publisher_count"),
        ({"final_subscribers": 0}, "final_subscriber_count"),
    ],
)
def test_preflight_fails_closed_for_each_environment_gate(overrides, reason) -> None:
    result = _evaluate(**overrides)
    assert not result.ready
    assert reason in result.reasons


def test_preflight_rejects_invalid_age_and_count_config() -> None:
    with pytest.raises(ValueError, match="maximum_status_age"):
        _evaluate(maximum_status_age_sec=0.0)
    with pytest.raises(ValueError, match="endpoint counts"):
        _evaluate(final_subscribers=-1)
