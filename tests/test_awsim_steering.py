import json
from pathlib import Path

import pytest

from aic_transfuser_lite.control.awsim_steering import (
    CALIBRATED_ASSETS, CALIBRATED_POLICY, command_steering, steering_asset_contract,
)
from aic_transfuser_lite.control.time_trial_v1 import validate_trial_config


@pytest.mark.parametrize("sign", [-1., 1.])
def test_calibration_reaches_requested_tire_angle_and_preserves_input_limits(sign):
    previous = 0.
    for _ in range(10):
        mapped = command_steering(sign*.24, previous, .05, policy=CALIBRATED_POLICY)
        issued = mapped["issued_input_rad"]
        assert abs(issued-previous) <= .8*.05 + 1e-12
        assert abs(issued) <= .5
        assert mapped["issued_tire_target_rad"] == pytest.approx(.6*issued)
        assert mapped["previous_tire_target_rad"] == pytest.approx(.6*previous)
        previous = issued
    # Independently follow the verified AWSIM rad -> deg -> factor -> report.
    import math
    actual_deg = -.6 * math.degrees(previous)
    assert -math.radians(actual_deg) == pytest.approx(sign*.24)
    assert previous == pytest.approx(sign*.4)
    boundary = command_steering(sign*.3, sign*.5, .05, policy=CALIBRATED_POLICY)
    assert boundary["issued_input_rad"] == pytest.approx(sign*.5)
    with pytest.raises(ValueError, match="ACTUATOR_INFEASIBLE"):
        command_steering(sign*.300001, previous, .05, policy=CALIBRATED_POLICY)


def test_legacy_identity_and_zero_time_do_not_change_angle_contract():
    assert command_steering(.5, .45, .1)["issued_input_rad"] == .5
    result = command_steering(-.25, .1, 0., policy=CALIBRATED_POLICY)
    assert result["issued_input_rad"] == .1
    assert result["issued_tire_target_rad"] == .06


@pytest.mark.parametrize("required,previous,dt", [
    (float("nan"), 0., .05), (0., float("inf"), .05), (0., .50001, .05),
    (0., 0., -.01), (0., 0., .1001), (0., 0., float("nan")),
])
def test_invalid_actuator_state_is_rejected(required, previous, dt):
    with pytest.raises(ValueError, match="ACTUATOR_CONTRACT"):
        command_steering(required, previous, dt, policy=CALIBRATED_POLICY)


def test_calibrated_profile_requires_exact_asset_identity():
    config = json.loads((Path(__file__).parents[1]/"configs/control/time_path_calibrated_turning_5kmh_20260913.json").read_text())
    assert validate_trial_config(config) == "fixed_5kmh"
    assert steering_asset_contract(config) == CALIBRATED_ASSETS
    for bad in ({}, {**CALIBRATED_ASSETS, "AWSIM_Data/Managed/Assembly-CSharp.dll": "0"*64}):
        with pytest.raises(ValueError, match="ASSET_IDENTITY"):
            validate_trial_config({**config, "steering_asset_sha256": bad})
    with pytest.raises(ValueError, match="STEERING_POLICY"):
        validate_trial_config({**config, "steering_policy": "arbitrary_gain"})
    assert steering_asset_contract({}) == {}
