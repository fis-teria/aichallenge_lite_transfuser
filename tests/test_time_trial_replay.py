from copy import deepcopy

import numpy as np
import pytest

from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose
from aic_transfuser_lite.control.time_trial_v1 import time_trial_control
from aic_transfuser_lite.control.awsim_steering import command_steering, CALIBRATED_POLICY, LEAD_POLICY
from aic_transfuser_lite.control.awsim_steering_response import compensate_steering_response
from tools.evaluate_time_awsim_trial import replay_recorded_control, response_record_matches


def test_nested_interpolation_lists_use_existing_scalar_replay_tolerance():
    expected = {'fraction_interval': [.2881617239797429, .4804064803709701], 'indices': [22, 23]}
    recorded = {'fraction_interval': [.2881617239797542, .48040648037097017], 'indices': [22, 23]}
    assert response_record_matches(recorded, expected)
    assert not response_record_matches({**recorded, 'fraction_interval': [.28816172, .48040648]}, expected)
    assert not response_record_matches({**recorded, 'indices': [22]}, expected)
    assert not response_record_matches({**recorded, 'indices': [22, 24]}, expected)


def recorded_turn():
    pose = TimedBodyPose(0, "sim", "0", "map", "base_link", 0., 0., 0.)
    distance = np.arange(1, 31)*.1
    xy = np.column_stack((np.sin(.1*distance)/.1, (1-np.cos(.1*distance))/.1))
    plan = {"plan_id": "p", "raw_xy_m": xy.tolist()}
    details = time_trial_control(TimePlan("p", pose, xy), pose, speed_mps=.1, rear_axle_offset_m=(.001, 0.))
    details.update(observation_pose=pose.__dict__, current_pose=pose.__dict__)
    command = {"plan_id": "p", "reason": "TIME_PATH_TRACKING", "speed_mps": .1, "details": details,
               "steer_rad": details["steer_rad"], "acceleration_mps2": details["acceleration_mps2"],
               "target_speed_mps": details["target_speed_mps"]}
    return command, plan


def test_post_pp_scan_rejection_is_separate_from_mathematical_replay():
    good, plan = recorded_turn()
    rejected = deepcopy(good)
    rejected.update(reason="STOPPING_SWEEP_OCCUPIED", acceleration_mps2=-1., target_speed_mps=0.)
    result = replay_recorded_control([good, rejected], [plan], .001, obstacle_policy="steering_sweep_v1")
    assert result["matched_commands"] == 2
    assert result["post_control_rejections"] == {"STOPPING_SWEEP_OCCUPIED": 1}
    assert result["scan_guard_decisions_replayed"] is False
    with pytest.raises(ValueError, match="admission"):
        replay_recorded_control([rejected], [plan], .001)
    rejected["acceleration_mps2"] = .1
    with pytest.raises(ValueError, match="did not brake"):
        replay_recorded_control([rejected], [plan], .001, obstacle_policy="steering_sweep_v1")


def test_calibrated_mapping_and_raw_command_must_match():
    command, plan = recorded_turn()
    mapped = command_steering(command["details"]["steer_rad"], .1, .05, policy=CALIBRATED_POLICY)
    command["details"]["steering_actuator"] = mapped
    command["steer_rad"] = mapped["issued_input_rad"]
    result = replay_recorded_control([command], [plan], .001, steering_policy=CALIBRATED_POLICY)
    assert result["actuator_mapping_matched"] == 1
    command["steer_rad"] *= .6
    with pytest.raises(ValueError, match="issued steering"):
        replay_recorded_control([command], [plan], .001, steering_policy=CALIBRATED_POLICY)


def test_replay_still_rejects_wrong_math_unknown_reason_and_nan():
    command, plan = recorded_turn()
    command["reason"] = "UNKNOWN_FAULT"
    with pytest.raises(ValueError, match="admission"):
        replay_recorded_control([command], [plan], .001, obstacle_policy="steering_sweep_v1")
    command["reason"] = "TIME_PATH_TRACKING"
    command["details"]["steer_rad"] = float("nan")
    with pytest.raises(ValueError, match="control differs"):
        replay_recorded_control([command], [plan], .001)


def test_response_replay_checks_sequential_state_and_compensated_command():
    command, plan = recorded_turn()
    rows = []
    state = None
    previous_input = .15
    for i in range(2):
        row = deepcopy(command)
        row["sim_ns"] = i*50_000_000
        target, state, response = compensate_steering_response(row["details"]["steer_rad"], row["sim_ns"], state, policy=LEAD_POLICY)
        mapping = command_steering(target, previous_input, .05, policy=LEAD_POLICY)
        row["details"].update(steering_response=response, steering_actuator=mapping)
        row["steer_rad"] = previous_input = mapping["issued_input_rad"]
        rows.append(row)
    result = replay_recorded_control(rows, [plan], .001, steering_policy=LEAD_POLICY)
    assert result["steering_response_matched"] == 2
    corrupted = deepcopy(rows)
    corrupted[1]["details"]["steering_response"]["previous_state"]["nominal_tire_rad"] += .01
    with pytest.raises(ValueError, match="response differs"):
        replay_recorded_control(corrupted, [plan], .001, steering_policy=LEAD_POLICY)
    corrupted = deepcopy(rows)
    del corrupted[0]["details"]["steering_response"]
    with pytest.raises(ValueError, match="response missing"):
        replay_recorded_control(corrupted, [plan], .001, steering_policy=LEAD_POLICY)
