from __future__ import annotations

import numpy as np
import pytest

from aic_transfuser_lite.control.executable_reference import (
    AuthoritativePlanV3,
    ExecutableReferenceConfigV3,
    build_executable_reference_v3,
    estimate_polyline_curvature_per_m,
    polyline_arc_length_m,
)


def _plan(
    trajectory: np.ndarray,
    speeds: np.ndarray,
    *,
    stop_probability: float | None = 0.1,
) -> AuthoritativePlanV3:
    count = len(trajectory)
    return AuthoritativePlanV3(
        trajectory_xy_m=np.asarray(trajectory, dtype=np.float64),
        speed_profile_mps=np.asarray(speeds, dtype=np.float64),
        waypoint_times_sec=np.arange(1, count + 1, dtype=np.float64) * 0.5,
        observation_stamp_sec=123.0,
        stop_probability=stop_probability,
    )


def _config(**overrides: float | bool | None) -> ExecutableReferenceConfigV3:
    values: dict[str, float | bool | None] = {
        "odd_speed_cap_mps": 2.0,
        "max_lateral_acceleration_mps2": 1.0,
    }
    values.update(overrides)
    return ExecutableReferenceConfigV3(**values)  # type: ignore[arg-type]


def test_straight_plan_is_arc_length_retimed_after_odd_speed_cap() -> None:
    plan = _plan(
        np.asarray([[1.0, 0.0], [2.0, 0.0], [3.0, 0.0]]),
        np.asarray([3.0, 3.0, 3.0]),
    )
    decision = build_executable_reference_v3(
        plan,
        current_speed_mps=2.0,
        config=_config(),
    )

    assert not decision.stop_required
    assert decision.reasons == ()
    assert decision.reference is not None
    reference = decision.reference
    np.testing.assert_allclose(reference.arc_length_m, [1.0, 2.0, 3.0])
    np.testing.assert_allclose(reference.speed_mps, 2.0)
    np.testing.assert_allclose(reference.time_from_observation_sec, [0.5, 1.0, 1.5])
    np.testing.assert_allclose(reference.curvature_per_m, 0.0)
    assert reference.transformations == ("odd_speed_cap",)
    assert len(reference.reference_id) == 64
    with pytest.raises(ValueError, match="read-only"):
        reference.speed_mps[0] = 0.0


def test_curvature_and_safety_caps_are_applied_before_retime() -> None:
    trajectory = np.asarray([[1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
    plan = _plan(trajectory, np.asarray([4.0, 4.0, 4.0]))
    decision = build_executable_reference_v3(
        plan,
        current_speed_mps=1.0,
        config=_config(
            odd_speed_cap_mps=5.0,
            max_lateral_acceleration_mps2=0.5,
            safety_speed_cap_mps=0.5,
        ),
    )

    assert decision.reference is not None
    reference = decision.reference
    assert bool((reference.curvature_per_m > 0.0).any())
    assert float(reference.speed_mps.max()) <= 0.5
    assert reference.transformations == (
        "curvature_speed_cap",
        "safety_speed_cap",
    )
    assert bool((np.diff(reference.time_from_observation_sec) > 0.0).all())


def test_stop_probability_and_missing_required_stop_fail_closed() -> None:
    trajectory = np.asarray([[0.5, 0.0], [1.0, 0.0]])
    stopped = build_executable_reference_v3(
        _plan(trajectory, np.ones(2), stop_probability=0.8),
        current_speed_mps=0.0,
        config=_config(stop_probability_threshold=0.6),
    )
    assert stopped.stop_required
    assert stopped.reference is None
    assert stopped.reasons == ("model_stop",)

    missing = build_executable_reference_v3(
        _plan(trajectory, np.ones(2), stop_probability=None),
        current_speed_mps=0.0,
        config=_config(require_stop_probability=True),
    )
    assert missing.stop_required
    assert missing.reference is None
    assert missing.reasons == ("invalid_plan:stop_probability is required",)


@pytest.mark.parametrize(
    ("trajectory", "speeds", "reason"),
    [
        (np.asarray([[0.5, 0.0], [np.nan, 0.0]]), np.ones(2), "finite"),
        (np.asarray([[0.5, 0.0], [1.0, 0.0]]), np.asarray([1.0, -0.1]), "non-negative"),
        (np.asarray([[-0.5, 0.0], [-1.0, 0.0]]), np.ones(2), "not_forward"),
    ],
)
def test_invalid_or_behind_plan_fails_closed(
    trajectory: np.ndarray, speeds: np.ndarray, reason: str
) -> None:
    decision = build_executable_reference_v3(
        _plan(trajectory, speeds),
        current_speed_mps=0.0,
        config=_config(),
    )
    assert decision.stop_required
    assert decision.reference is None
    assert reason in decision.reasons[0]


def test_moving_path_with_zero_speed_is_not_retimed_by_hidden_floor() -> None:
    decision = build_executable_reference_v3(
        _plan(np.asarray([[0.5, 0.0], [1.0, 0.0]]), np.zeros(2)),
        current_speed_mps=0.0,
        config=_config(),
    )
    assert decision.stop_required
    assert decision.reference is None
    assert decision.reasons == ("non_executable_speed:segment=0",)


def test_polyline_geometry_uses_each_chord_instead_of_endpoint_distance() -> None:
    trajectory = np.asarray([[1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
    segment, cumulative = polyline_arc_length_m(trajectory)
    np.testing.assert_allclose(segment, [1.0, 1.0, 1.0])
    np.testing.assert_allclose(cumulative, [1.0, 2.0, 3.0])
    curvature = estimate_polyline_curvature_per_m(trajectory)
    assert curvature.shape == (3,)
    assert curvature[0] > 0.0


def test_reference_id_is_deterministic_and_changes_with_observation() -> None:
    trajectory = np.asarray([[0.5, 0.0], [1.0, 0.0]])
    plan = _plan(trajectory, np.ones(2))
    first = build_executable_reference_v3(
        plan, current_speed_mps=1.0, config=_config()
    ).reference
    second = build_executable_reference_v3(
        plan, current_speed_mps=1.0, config=_config()
    ).reference
    changed = build_executable_reference_v3(
        AuthoritativePlanV3(
            trajectory,
            np.ones(2),
            np.asarray([0.5, 1.0]),
            observation_stamp_sec=124.0,
            stop_probability=0.1,
        ),
        current_speed_mps=1.0,
        config=_config(),
    ).reference
    assert first is not None and second is not None and changed is not None
    assert first.reference_id == second.reference_id
    assert first.reference_id != changed.reference_id
