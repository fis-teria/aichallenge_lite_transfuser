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

Heading consistency is gated only at trajectory samples whose predicted speed
is at least `consistency_min_heading_speed_mps`; tangent heading is not
observable on a near-stationary path. Position, lateral, speed, and endpoint
checks remain active at every sample. The previous published nominal command
is fed back as the next one-step command history and as the projection's prior
acceleration, so receding-horizon execution does not restart from zero on every
observation.

The trial profile also contains an explicitly authorized stopped-launch
assist. Before authoritative projection, it can raise the model sequence's
acceleration proposals to a bounded floor only while actual speed is at or
below `0.1 m/s`, the model commands at least `0.2 m/s`, and the assist has not
previously observed the vehicle moving. All normal acceleration and jerk
limits still apply. It does not alter an external fallback command.

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

## RViz frames and views

The V3 RViz profile uses `map` as its Fixed Frame and opens the `AWSIM Map
Fixed` top-down view near the course start. It displays the transient-local
`/map/vector_map_marker`, TF tree, `base_link` axes, `/localization/pose`,
best-effort LiDAR, and the predicted trajectory path. The axes are used instead
of `RobotModel` because the V3 runtime image does not install the external
`racing_kart_description` mesh.
The predicted path remains correctly stamped in `base_link`; RViz transforms
it into `map` using the live `map -> base_link` transform.

The saved `Ego Chase` view targets `base_link` for sensor-relative inspection.
In that view the vehicle intentionally remains centered, so use `AWSIM Map
Fixed` when checking whether the localization pose and vehicle actually move
through the course. The map view starts around `x=89631 m`, `y=43128 m`, which
is the observed AWSIM course start; use RViz's Focus Camera tool if a different
scenario starts elsewhere.

Do not report ROS 2, AWSIM, collision avoidance, or course completion as
successful until those commands have actually run and the resulting logs are
reviewed.

## Build and execution verification record

Commit `aebee06` passed the focused WSL runtime suite (`84 passed`) and the
complete repository suite (`435 passed, 32 warnings`). Its tracked source
archive SHA-256
`2d7e93f30299f03bc2c4ee8219f8b002026cde7039ed7aafbc1edd6acca15792`
then passed the full-control focused suite in
`aichallenge-2025-dev:latest` on Graneple (`55 passed`). In that same official
container, `colcon build --packages-select aic_e2e_runtime` completed one
package and `ros2 launch ... --show-args` loaded the new launch description.

Later commit `9abdf7e` passed the complete WSL suite (`447 passed, 32
warnings`), a focused official-container unit/negative suite (`70 passed`), and
an official-container one-package ROS build. Its ROS shadow graph and RViz were
actually run against AWSIM. A limited full-control graph was also run with
Safety as the sole final-command publisher, but the 30 s stopped-start probe
reached only `0.012603 m/s` maximum speed and `0.039785 m` displacement.

Therefore ROS wiring and the attempted trial are verified, but successful
launch, route progress, collision avoidance, and course completion are not.
The exact evidence and hashes are in `docs/v3_m11_limited_odd_report.md`.
