# V3 M3 0.75 m/s Limited ODD

M3 validates the trajectory-authoritative path in an obstacle-free AWSIM run.
It does not authorize a higher speed cap and does not claim a full lap.

## Required launch state

- AWSIM is reset to `Grounded` before each independent launch trial.
- Autoware initialization reports `true`, Gear is Drive, and Control Mode is
  Autonomous.
- The one-shot admin Start is followed by an accepted
  `/autostart/official_start` service call.
- `/overtake/race_armed=true` is retained while vehicle control is authorized.
- `/nominal_control_cmd` and `/control/command/control_cmd` each have exactly
  one publisher and one intended downstream subscriber.

Do not record or echo either control topic during the trial: an additional
subscriber intentionally makes the strict routing preflight fail closed.
Command speed, acceleration, controller state, and preflight results are already
included in `/plan_diagnostics`.

## Evidence recording

Record only non-control evidence:

```bash
ros2 bag record -o /artifacts/m3_trial \
  /clock /awsim/state /awsim/status \
  /autostart/initialization_ready /overtake/race_armed \
  /vehicle/status/velocity_status /vehicle/status/steering_status \
  /plan_diagnostics /safety_reason /localization/kinematic_state
```

The installed AWSIM binary defines
`/awsim/ground_truth/on_collision` as `std_msgs/msg/Bool`, publishing `true` from
`OnCollisionEnter`. Start a typed observer before Start and record both its
matched-publisher status and any messages. A zero-byte observer log without a
matched publisher is **unverified**, not proof of zero collisions.

Analyze the bag in the official ROS container:

```bash
export PYTHONPATH=/work/src:${PYTHONPATH}
/usr/bin/python3 /work/tools/analyze_m3_rosbag.py \
  /artifacts/m3_trial --speed-cap-mps 0.75
```

The analyzer reports launch latency, measured maximum/final speed, commanded
speed and acceleration bounds, preflight ratio, controller faults, Safety
states, displacement, signed yaw-rate coverage, and collision evidence state.

## M3 gates

Run at least three independent stopped launches. For each trial:

- measured longitudinal speed reaches 0.10 m/s within 3.0 s after
  `race_armed=true`;
- measured speed remains at or below 0.85 m/s (0.75 m/s cap plus 0.10 m/s
  measurement/control tolerance);
- no `launch_response_missing` or other controller fault occurs;
- Safety remains `normal` during accepted nominal motion;
- the run contains measurable straight and gentle-turn samples;
- a matched collision observer reports no `true` event.

Left and right curve coverage may be accumulated across the independent trials.
M4 remains blocked until repeated launch, tracking, curve coverage, speed cap,
and collision evidence all pass. ROS/AWSIM outcomes not present in an analyzed
artifact must remain reported as unverified.
