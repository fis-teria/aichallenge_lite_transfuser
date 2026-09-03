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

For lateral tracking, the limited-ODD profile combines the delay-aware preview
time with a 1.0 m minimum arc-length lookahead. The farther of the time-selected
point and the arc-length-selected point is used. This prevents a 0.37 m
low-speed preview from converting centimetre-scale near-waypoint noise into a
large Pure Pursuit steering command. The value is controller configuration, not
a modification of the predicted trajectory.

## Environment preflight

The runtime checks all of these immediately before every nominal command:

- GearReport equals Drive (`2` in the installed Autoware messages);
- ControlModeReport equals Autonomous (`1`);
- the transient-local AWSIM state is `Start` or `Ready`, and the retained
  `/overtake/race_armed` value is `true`;
- Gear and Control Mode reports are no older than 0.5 s;
- `/nominal_control_cmd` has exactly one publisher and one subscriber;
- `/control/command/control_cmd` has exactly one publisher and one subscriber.

The final topic publisher must be Safety and its one subscriber must be AWSIM.
The preflight cannot identify endpoint process names from counts alone, so the
launch/graph check on Graneple must also record `ros2 topic info -v`.

AWSIM emits `Start` as a transition and then returns the per-vehicle state to
`Ready`. Therefore `Ready` by itself never opens the preflight: the official
autostart service must also have accepted the start and published
`race_armed=true`. Negative tests cover `Grounded`, unarmed `Ready`, missing arm
state, stale Drive/Autonomous reports, and non-unique routing.

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

## 2026-09-04 Graneple launch finding

The immutable `2bd5537` source archive built successfully in the official
`aichallenge-2025-dev:latest` container. Runtime startup must source the
Autoware underlay before the V3 overlay:

```bash
source /opt/ros/humble/setup.bash
source /autoware/install/setup.bash
source /work/ros2_ws/install/setup.bash
ros2 launch aic_e2e_runtime \
  transfuser_lite_v3_trajectory_authoritative.launch.py \
  param_file:=/artifacts/runtime.v3.trajectory_authoritative.param.yaml \
  model_path:=/artifacts/last.pt \
  artifact_manifest_path:=/artifacts/runtime_artifact.json \
  launch_rviz:=false use_sim_time:=true
```

Before Start, both nominal and final topics had exactly one publisher and one
subscriber, and the controller failed closed with `awsim_not_started` while
AWSIM was `Grounded`. The official start sequence then proved this state
machine on the installed simulator:

```text
admin: WaitStart --one-shot start--> Start
vehicle: Grounded --start event--> Start --settled--> Ready
official_start service: success=true
race_armed: false -> true
```

The original M2 preflight incorrectly required the retained vehicle state to
remain exactly `Start`. It therefore continued to command zero speed and
-4.0 m/s2 after the accepted official start, even though the retained state had
correctly settled to `Ready`. The corrected contract accepts `Start` or
`Ready` only when `race_armed=true`; unarmed `Ready` remains fail-closed. The
corrected runtime has not yet been rerun at the time of this entry, so launch
response and M3 are not claimed.

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
