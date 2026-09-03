import numpy as np
import pytest

from aic_transfuser_lite.runtime.control_projection import (
    PreviousControlState,
    ProjectedControlSequence,
)
from aic_transfuser_lite.runtime.full_control_gate import (
    ControlAuthorityMode,
    FullControlReadiness,
    authority_change_allowed,
    choose_full_control_or_same_trajectory_fallback,
    previous_nominal_command_history,
)
from aic_transfuser_lite.runtime.residual_control import ExternalControllerCommand
from aic_transfuser_lite.runtime.rollout_consistency import ConsistencyMetrics


SHA = "a" * 64


def _sequence() -> ProjectedControlSequence:
    return ProjectedControlSequence(
        commands=np.array([[0.1, 2.0, 0.2], [0.2, 2.0, 0.0]]),
        steering_rate_radps=np.array([0.1, 0.1]),
        jerk_mps3=np.array([0.2, -0.2]),
        source_stamp_sec=1.0,
        valid_until_sec=1.2,
        limits_source="verified",
        dt_sec=0.1,
        initial_state=PreviousControlState(0.0, 0.0, 0.0),
    )


def _consistency(passed: bool) -> ConsistencyMetrics:
    return ConsistencyMetrics(0.1, 0.2, 0.1, 0.1, 0.2, 0.2, passed, () if passed else ("max_position_error_m>0.1",))


def _readiness(**changes: object) -> FullControlReadiness:
    values: dict[str, object] = {
        "capabilities": frozenset({"trajectory", "control_sequence"}),
        "calibration_state": "shadow",
        "deployment_stage": "limited_odd_trial",
        "safety_supervisor_ready": True,
        "evidence_sha256": SHA,
        "evidence_passed": True,
        "trial_speed_cap_mps": 0.8,
    }
    values.update(changes)
    return FullControlReadiness(**values)  # type: ignore[arg-type]


def test_limited_odd_model_command_is_speed_capped_and_safety_owned() -> None:
    result = choose_full_control_or_same_trajectory_fallback(
        _sequence(), _consistency(True), None,
        readiness=_readiness(), selected_trajectory_id="candidate0", fallback_trajectory_id="candidate0",
    )
    assert result.source == "model_control_sequence"
    assert result.command.speed_mps == 0.8
    assert result.requires_safety_supervisor


def test_inconsistency_uses_exact_same_trajectory_fallback() -> None:
    fallback = ExternalControllerCommand(-0.2, 0.4, -0.1)
    result = choose_full_control_or_same_trajectory_fallback(
        _sequence(), _consistency(False), fallback, readiness=_readiness(),
        selected_trajectory_id="candidate0", fallback_trajectory_id="candidate0",
    )
    assert result.command is fallback
    assert result.source == "same_trajectory_external_fallback"
    assert result.consistency_reasons


def test_inconsistency_without_usable_same_trajectory_fallback_fails_closed() -> None:
    with pytest.raises(ValueError, match="requires a same-trajectory fallback"):
        choose_full_control_or_same_trajectory_fallback(
            _sequence(), _consistency(False), None, readiness=_readiness(),
            selected_trajectory_id="candidate0", fallback_trajectory_id="candidate0",
        )


@pytest.mark.parametrize(
    "readiness",
    [
        _readiness(capabilities=frozenset({"trajectory"})),
        _readiness(safety_supervisor_ready=False),
        _readiness(evidence_passed=False),
        _readiness(evidence_sha256="bad"),
        _readiness(trial_speed_cap_mps=1.1),
        _readiness(calibration_state="candidate"),
    ],
)
def test_full_control_readiness_fails_closed(readiness: FullControlReadiness) -> None:
    with pytest.raises(ValueError):
        choose_full_control_or_same_trajectory_fallback(
            _sequence(), _consistency(True), ExternalControllerCommand(0.0, 0.0, 0.0),
            readiness=readiness, selected_trajectory_id="x", fallback_trajectory_id="x",
        )


def test_fallback_must_reference_same_selected_trajectory() -> None:
    with pytest.raises(ValueError, match="same selected trajectory"):
        choose_full_control_or_same_trajectory_fallback(
            _sequence(), _consistency(False), ExternalControllerCommand(0.0, 0.0, 0.0),
            readiness=_readiness(), selected_trajectory_id="x", fallback_trajectory_id="y",
        )


def test_authority_changes_require_inactive_or_stopped_state() -> None:
    assert not authority_change_allowed(
        ControlAuthorityMode.SHADOW, ControlAuthorityMode.FULL_CONTROL,
        lifecycle_inactive=False, longitudinal_speed_mps=0.2,
    )
    assert authority_change_allowed(
        ControlAuthorityMode.SHADOW, ControlAuthorityMode.FULL_CONTROL,
        lifecycle_inactive=False, longitudinal_speed_mps=0.01,
    )
    assert authority_change_allowed(
        ControlAuthorityMode.SHADOW, ControlAuthorityMode.FULL_CONTROL,
        lifecycle_inactive=True, longitudinal_speed_mps=3.0,
    )


def test_previous_nominal_command_is_fed_to_next_receding_horizon() -> None:
    values, valid = previous_nominal_command_history(
        ExternalControllerCommand(0.1, 0.8, 0.4)
    )
    assert values == (0.1, 0.8, 0.4)
    assert valid
    assert previous_nominal_command_history(None) == ((0.0, 0.0, 0.0), False)


@pytest.mark.parametrize(
    "command",
    [
        ExternalControllerCommand(float("nan"), 0.8, 0.0),
        ExternalControllerCommand(0.0, -0.1, 0.0),
        ExternalControllerCommand(0.0, 0.8, float("inf")),
    ],
)
def test_invalid_previous_nominal_command_is_rejected(
    command: ExternalControllerCommand,
) -> None:
    with pytest.raises(ValueError, match="previous nominal"):
        previous_nominal_command_history(command)
