# V3 gated full-control runtime

`transfuser_lite_v3_full_control_trial.launch.py` is an AWSIM-only limited-ODD
entry point. It launches the V3 inference node, the independent Safety
Supervisor, and RViz by default. The inference node publishes
`nominal_control_cmd`; only Safety publishes `/control/command/control_cmd`.

Startup fails unless all of the following are true:

- the runtime artifact declares both `trajectory` and `control_sequence`;
- checkpoint, runtime manifest, and trial-evidence hashes match;
- the calibration artifact is in `shadow` state for a limited trial;
- authoritative steering, rate, acceleration, jerk, speed, and timing limits
  are finite and valid;
- Safety is explicitly included in the launch contract; and
- authority is changed while inactive/stopped.

At each observation, the model sequence is re-projected through authoritative
absolute/rate limits. The calibrated actuator+bicycle rollout is compared only
with candidate zero of the same model trajectory and speed profile. A failed
consistency check selects the external controller computed from that exact
trajectory; it never switches trajectory candidates. Safety remains the final
authority for either source.

The checked-in parameter file contains replacement markers and cannot be used
unchanged. Create a deployment copy outside Git and replace the checkpoint,
runtime manifest, calibration, and trial authorization paths/hashes. The
limited trial caps command speed at 0.8 m/s in both inference and Safety.

Run the ROS-independent unit and negative tests with:

```bash
python3 -m pytest -q \
  tests/test_control_projection_v3.py \
  tests/test_rollout_consistency_v3.py \
  tests/test_full_control_gate_v3.py \
  tests/test_full_control_ros_wiring_v3.py
```

The actual ROS launch form is:

```bash
ros2 launch aic_e2e_runtime transfuser_lite_v3_full_control_trial.launch.py \
  param_file:=/absolute/path/runtime.v3.full_control_trial.deployed.yaml \
  model_path:=/absolute/path/last.pt \
  artifact_manifest_path:=/absolute/path/runtime_artifact.json \
  calibration_artifact_path:=/absolute/path/calibration.shadow.json \
  full_control_evidence_path:=/absolute/path/v3_full_control_trial_authorization.yaml \
  launch_rviz:=true use_sim_time:=true
```

Do not report ROS 2, AWSIM, collision avoidance, or course completion as
successful until those commands have actually run and the resulting logs are
reviewed.
