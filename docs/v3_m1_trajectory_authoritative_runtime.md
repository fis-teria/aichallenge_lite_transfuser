# V3 M1 Trajectory-Authoritative Runtime

## Authority

`trajectory_authoritative` makes the validated Trajectory Head and Speed Profile
Head the only motion-intent source. The runtime transforms them into an
ODD/curvature-capped, arc-length-retimed executable reference and the external
delay-aware controller publishes `/nominal_control_cmd` from that reference.

Future Control Sequence remains loaded and is published only on
`/shadow_model_control_sequence`. It is never selected by a consistency gate and
never owns nominal authority. The old `full_control` runtime profile is rejected
at node construction; its historical files remain only as evidence/rollback
material.

Publisher ownership is fixed:

| Role | Owner | Topic |
|---|---|---|
| Motion intent | Trajectory / Speed Heads | predicted Plan topics |
| Auxiliary control | Future Control Sequence Head | `/shadow_model_control_sequence` |
| Nominal command | executable-reference external controller | `/nominal_control_cmd` |
| Final command | external Safety Supervisor | `/control/command/control_cmd` |

An executable-reference STOP decision produces a bounded zero-speed brake
proposal for Safety. It never falls back to Current Control or Future Control
Sequence. If inference stops publishing altogether, Safety's independent nominal
freshness timeout remains the final fail-safe.

## Launch

After replacing all three expected hashes in the parameter file or artifact
launch inputs, use:

```bash
ros2 launch aic_e2e_runtime \
  transfuser_lite_v3_trajectory_authoritative.launch.py \
  model_path:=/absolute/path/to/checkpoint.pt \
  artifact_manifest_path:=/absolute/path/to/runtime_artifact.json \
  launch_rviz:=true
```

The checked-in limited configuration applies a 0.75 m/s executable and Safety
speed cap. `executable_reference_require_stop_probability` remains explicitly
false until M5 supplies a trained Stop Head.

## Verification boundary

Run the ROS-independent unit/negative and source ownership tests with:

```bash
PYTHONPATH=src python -m pytest -q \
  tests/test_trajectory_authoritative_controller_v3.py \
  tests/test_trajectory_authoritative_ros_wiring_v3.py \
  tests/test_executable_reference_v3.py
```

This source implementation does not prove ROS build, AWSIM command semantics,
gear/enable state, vehicle launch, tracking, or completion. Those gates must be
run in order on `graneple@192.168.3.10` before M3 or M4 can pass.
