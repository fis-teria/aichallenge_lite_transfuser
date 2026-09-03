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
It also compares 1.0 s future odometry in the observation-time ego frame with
both the raw fixed-time Trajectory Head and the speed-capped, arc-length-retimed
Executable Reference. These are separate metrics: M3 controller tracking is
judged against the reference that was actually executed; the raw result remains
visible as Head-consistency evidence.

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

The checked-in M3 profile uses a 1.0 m minimum arc-length lookahead in addition
to the time preview. The two pre-change Graneple trials used approximately
0.37 m lookahead at 0.75 m/s and each stopped after only 6.6--6.7 m. They are
diagnostic evidence, not M3 passes. Any post-change result must use a new commit
and artifact identity.

## Graneple evidence before bounded origin normalization

Commit `7a856818718878156347bab158b14d988af5f1df` was run on the designated
Graneple environment. These artifacts are diagnostic and do not pass M3:

- `m3_lookahead_trial1_7a85681`: 37.39 s, 24.36 m displacement, launch in
  1.92 s, maximum measured speed 0.7345 m/s, Safety `normal` 745/745, and
  Executable Reference tracking p95 0.0713 m. It contained straight samples
  only.
- `m3_curve_trial2_7a85681`: 92.23 s, 29.72 m displacement, launch in 2.06 s,
  maximum measured speed 0.7484 m/s, right-turn samples 93, and Executable
  Reference tracking p95 0.0804 m for matched references. It then produced
  `initial_waypoint_not_forward` for 427 frames and stopped.

Inspection of that stop episode found a leading point only 4--9 mm behind ego,
followed by forward points at 47--71 mm and endpoints 0.65--1.16 m ahead. The
Executable Reference therefore performs the bounded 0.05 m leading-noise trim
documented in `v3_m0_plan_contract_and_executable_reference.md`; larger or
nonrecoverable reverse paths remain fail-closed.

The installed AWSIM graph exposed no publisher for
`/awsim/ground_truth/on_collision` in either run. Consequently collision absence
is **unverified**, even though no collision message was recorded. A new immutable
commit and independent trials are required after origin normalization. M4 is
still blocked.
