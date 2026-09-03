from __future__ import annotations

import json

import numpy as np

from aic_transfuser_lite.control.executable_reference import (
    AuthoritativePlanV3,
    ExecutableReferenceConfigV3,
)
from aic_transfuser_lite.runtime.plan_diagnostics import (
    PLAN_DIAGNOSTICS_FORMAT_V3,
    plan_diagnostics_payload_v3,
)


def test_plan_diagnostic_contains_raw_e_plan_and_executable_reference() -> None:
    plan = AuthoritativePlanV3(
        trajectory_xy_m=np.asarray([[0.1, 0.0], [0.2, 0.0]]),
        speed_profile_mps=np.asarray([1.0, 1.0]),
        waypoint_times_sec=np.asarray([0.1, 0.2]),
        observation_stamp_sec=12.5,
        stop_probability=None,
    )
    payload = plan_diagnostics_payload_v3(
        plan,
        current_speed_mps=1.0,
        reference_config=ExecutableReferenceConfigV3(
            odd_speed_cap_mps=0.75,
            max_lateral_acceleration_mps2=1.0,
        ),
    )

    assert payload["format"] == PLAN_DIAGNOSTICS_FORMAT_V3
    assert payload["authority"] == "shadow_diagnostic_only"
    assert payload["e_plan"]["mean_absolute_error_mps"] == 0.0
    assert payload["decision"] == {"stop_required": False, "reasons": []}
    reference = payload["executable_reference"]
    assert reference is not None
    assert reference["speed_mps"] == [0.75, 0.75]
    assert "odd_speed_cap" in reference["transformations"]
    assert "stop_probability_unavailable" in reference["transformations"]
    assert json.loads(json.dumps(payload)) == payload


def test_plan_diagnostic_records_model_stop_without_a_reference() -> None:
    plan = AuthoritativePlanV3(
        trajectory_xy_m=np.asarray([[0.1, 0.0], [0.2, 0.0]]),
        speed_profile_mps=np.asarray([1.0, 1.0]),
        waypoint_times_sec=np.asarray([0.1, 0.2]),
        observation_stamp_sec=12.5,
        stop_probability=0.9,
    )
    payload = plan_diagnostics_payload_v3(
        plan,
        current_speed_mps=0.0,
        reference_config=ExecutableReferenceConfigV3(
            odd_speed_cap_mps=0.75,
            max_lateral_acceleration_mps2=1.0,
        ),
    )
    assert payload["decision"] == {
        "stop_required": True,
        "reasons": ["model_stop"],
    }
    assert payload["executable_reference"] is None
