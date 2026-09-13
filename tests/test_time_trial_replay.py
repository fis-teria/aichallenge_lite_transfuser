from copy import deepcopy

import numpy as np
import pytest

from aic_transfuser_lite.control.time_reference_v1 import TimePlan, TimedBodyPose
from aic_transfuser_lite.control.time_trial_v1 import time_trial_control
from aic_transfuser_lite.control.awsim_steering import command_steering, CALIBRATED_POLICY
from tools.evaluate_time_awsim_trial import replay_recorded_control


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
