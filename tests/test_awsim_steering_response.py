"""Dynamic tracking against an independently integrated delay/first-order plant."""
from collections import deque
from dataclasses import replace
import json
import math
from pathlib import Path

import numpy as np
import pytest

from aic_transfuser_lite.control.awsim_steering import (
    CALIBRATED_POLICY, LEAD_POLICY, command_steering,
)
from aic_transfuser_lite.control.awsim_steering_response import (
    SteeringResponseState, compensate_steering_response,
)
from aic_transfuser_lite.control.time_trial_v1 import validate_trial_config


def run_plant(policy: str, shape: str, extra_transport_steps: int = 0) -> tuple[float, float]:
    # Independent radians implementation of the hash-pinned Vehicle.cs:
    # command holds 50 ms, simulation 5 ms, queue delay 70 ms, implicit Euler
    # time constant 20 ms, physical rate 60 deg/s. No measured-angle feedback.
    delayed: deque[tuple[int, float]] = deque()
    angle = input_rad = 0.
    state = None
    errors = []
    for step in range(2001):
        t = step*.005
        desired = .025*t if shape == "ramp" else .1*math.sin(.8*t)
        if step % 10 == 0:
            target, state, _ = compensate_steering_response(desired, step*5_000_000, state, policy=policy)
            mapping = command_steering(target, input_rad, .05, policy=policy)
            issued = mapping["issued_input_rad"]
            assert abs(issued) <= .5 and abs(issued-input_rad) <= .04+1e-12
            input_rad = issued
        delayed.append((step, .6*input_rad))
        while delayed and delayed[0][0] < step-34-extra_transport_steps:
            delayed.popleft()
        old_targets = [value for capture, value in delayed if capture <= step-14-extra_transport_steps]
        pending = old_targets[-1] if old_targets else angle
        unconstrained = angle + (.005/(.02+.005))*(pending-angle)
        angle += np.clip(unconstrained-angle, -math.pi/3*.005, math.pi/3*.005)
        if 1. <= t <= 10.:
            errors.append(abs(angle-desired))
    return float(np.mean(errors)), float(np.max(errors))


@pytest.mark.parametrize("shape", ["ramp", "sine"])
@pytest.mark.parametrize("extra_transport_steps", [0, 8])
def test_lead_reduces_dynamic_physical_tracking_error(shape, extra_transport_steps):
    # Also cover the additional 40 ms suggested by the stamped turn14 journal;
    # this is a plant sensitivity check, not a new runtime calibration.
    baseline = run_plant(CALIBRATED_POLICY, shape, extra_transport_steps)
    lead = run_plant(LEAD_POLICY, shape, extra_transport_steps)
    assert lead[0] < baseline[0]*.45
    assert lead[1] < baseline[1]*.60


def test_constant_request_and_legacy_have_no_feedforward():
    state = None
    for ns in (0, 50_000_000, 100_000_000):
        target, state, metadata = compensate_steering_response(.12, ns, state, policy=LEAD_POLICY)
        assert target == .12 and metadata["applied_correction_rad"] == 0.
    target, state, metadata = compensate_steering_response(.12, 150_000_000, state, policy=CALIBRATED_POLICY)
    assert target == .12 and state is None and metadata["lead_s"] == 0.


@pytest.mark.parametrize("stamp", [50_000_000, 100_000_000, 250_000_001])
def test_duplicate_reset_and_long_gap_prime_without_kick(stamp):
    previous = SteeringResponseState(.1, .2, 100_000_000)
    target, state, metadata = compensate_steering_response(-.15, stamp, previous, policy=LEAD_POLICY)
    assert target == -.15 and state.filtered_rate_radps == 0. and metadata["reset"]


@pytest.mark.parametrize("direction", [-1., 1.])
def test_headroom_clips_only_compensation_and_original_infeasible_stays_rejected(direction):
    previous = SteeringResponseState(direction*.28, direction*.48, 0)
    target, _, metadata = compensate_steering_response(direction*.299, 50_000_000, previous, policy=LEAD_POLICY)
    assert target == pytest.approx(direction*.3) and metadata["headroom_limited"]
    assert abs(metadata["requested_correction_rad"]) <= .0432
    with pytest.raises(ValueError, match="ACTUATOR_INFEASIBLE"):
        compensate_steering_response(direction*.300001, 50_000_000, previous, policy=LEAD_POLICY)


def test_alternating_requests_preserve_angle_and_rate_limits():
    state = None
    issued = 0.
    for step in range(100):
        target, state, details = compensate_steering_response(.29*(-1)**step, step*50_000_000, state, policy=LEAD_POLICY)
        mapped = command_steering(target, issued, .05, policy=LEAD_POLICY)
        assert abs(target) <= .3 and abs(details["filtered_rate_radps"]) <= .48
        assert abs(mapped["issued_input_rad"]-issued) <= .04+1e-12
        issued = mapped["issued_input_rad"]


@pytest.mark.parametrize("nominal,stamp", [(float("nan"), 0), (float("inf"), 0), (0., -1), (0., .5), (0., True)])
def test_malformed_request_or_clock_rejected(nominal, stamp):
    with pytest.raises(ValueError, match="RESPONSE_CONTRACT"):
        compensate_steering_response(nominal, stamp, None, policy=LEAD_POLICY)


@pytest.mark.parametrize("field,value", [("nominal_tire_rad", float("nan")), ("nominal_tire_rad", .31),
    ("filtered_rate_radps", float("inf")), ("filtered_rate_radps", .480001), ("stamp_ns", -1), ("stamp_ns", True)])
def test_malformed_prior_state_rejected(field, value):
    bad = replace(SteeringResponseState(.1, 0., 0), **{field: value})
    with pytest.raises(ValueError, match="RESPONSE_STATE"):
        compensate_steering_response(.1, 50_000_000, bad, policy=LEAD_POLICY)


def test_response_profile_changes_only_steering_policy_from_turn13():
    root = Path(__file__).parents[1]/"configs/control"
    before = json.loads((root/"time_path_preview_turning_5kmh_20260913.json").read_text())
    after = json.loads((root/"time_path_response_turning_5kmh_20260913.json").read_text())
    assert after == {**before, "steering_policy": LEAD_POLICY}
    assert validate_trial_config(after) == "fixed_5kmh"
