# V3 M2 External Controller and Launch Preflight

## Implemented control law

The trajectory-authoritative path uses a stateful longitudinal controller after
the existing delay-aware lateral controller:

```text
a_ref = (v_exec - v_measured) / reference_horizon
a_ff  = (a_ref - calibration_bias) / calibration_gain
a_cmd = clip(a_ff + Kp * speed_error + Ki * integral_error)
```

The integrator has conditional anti-windup. Acceleration and jerk are limited in
SI units on every cycle. The checked-in limited configuration uses a 0.75 m/s
speed cap, 0.5 m/s2 launch floor, and 0.4 m/s2 maximum positive acceleration
change per 100 ms cycle. The launch floor therefore ramps through the jerk limit;
it is not an instantaneous acceleration jump.

The controller states are `stopped`, `launching`, `moving`, `blocked`, and
`response_fault`. Launch is permitted only after the environment preflight passes.
If speed does not increase by 0.02 m/s within 1.0 s of a launch attempt, the
response monitor latches `launch_response_missing` and commands a fail-closed
zero-speed brake proposal through Safety.

## Environment preflight

The runtime checks all of these immediately before every nominal command:

- GearReport equals Drive (`2` in the installed Autoware messages);
- ControlModeReport equals Autonomous (`1`);
- the transient-local AWSIM state equals `Start`;
- Gear and Control Mode reports are no older than 0.5 s;
- `/nominal_control_cmd` has exactly one publisher and one subscriber;
- `/control/command/control_cmd` has exactly one publisher and one subscriber.

The final topic publisher must be Safety and its one subscriber must be AWSIM.
The preflight cannot identify endpoint process names from counts alone, so the
launch/graph check on Graneple must also record `ros2 topic info -v`.

## 2026-09-04 Graneple M0 observations

Read-only inspection of the already-running official graph on
`graneple@192.168.3.10`, `ROS_DOMAIN_ID=1`, confirmed:

- AWSIM node `awsim_d1` directly subscribes to
  `/control/command/control_cmd` as `AckermannControlCommand`;
- the final topic had zero publishers before the V3 launch and one AWSIM
  subscriber;
- `/vehicle/status/gear_status` reported `2` (`DRIVE`);
- `/vehicle/status/control_mode` reported `1` (`AUTONOMOUS`);
- `/awsim/state` was `Grounded` during the observation;
- `/awsim/control_mode_request_topic` is a Bool input to AWSIM and the official
  autostart orchestrator publishes `true` after seeing AWSIM state `Start`;
- the official challenge controller documentation and source use steering tire
  angle plus longitudinal acceleration as the vehicle command, while also filling
  the Ackermann speed setpoint.

These observations validate topic/message/routing semantics, not vehicle
response. A speed-only versus acceleration-only response experiment has not yet
been run for this patch.

## Unit and negative tests

```bash
PYTHONPATH=src python -m pytest -q \
  tests/test_longitudinal_controller_v3.py \
  tests/test_control_preflight_v3.py \
  tests/test_trajectory_authoritative_controller_v3.py \
  tests/test_trajectory_authoritative_ros_wiring_v3.py
```

ROS build, Start transition, launch response, straight/curve tracking, and lap
completion remain unverified until the corresponding Graneple runs are executed.
