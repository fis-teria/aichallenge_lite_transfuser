# V3 M11 limited-ODD execution report

## Decision

M11 is **incomplete**. The stopped-start full-control trial was executed, but
the vehicle did not establish meaningful forward motion. The calibration
artifact remains in `shadow` state. Course completion and collision avoidance
are not claimed, and V3-024 was not started.

## Immutable inputs

- Windows source commit: `9abdf7e66d1430a86408dbc67498c37b38b0dbf7`
- Graneple source archive SHA-256:
  `60e9bbd4f79affc7e5c2fa0d3ca94924c153bc635083304067f11f92060ed7f5`
- checkpoint SHA-256:
  `ad3a96cdd83f541557633223b5640b53ecb4f3a81e3613314f03bbdf6b7acbc6`
- runtime artifact SHA-256:
  `d8db5b8bb4ae4d2c29fc04f4d114ed9ed0f6b5f59dabba8ce118b9aae5f436b4`
- run manifest SHA-256:
  `c58ac7a8192dae5540341dcd9c532acf6a2b6a59fa5cb2f7deb23e49a9402671`
- offline evaluation SHA-256:
  `72ea290c9575bdf1d34421dcd21ade42e5b421f783cc6c491d5aa9f8df779448`
- model contract SHA-256:
  `fc2330566fb2a5e73f574da1b5ba1237a7054eaebe83a24802a7f26631e9a3e1`
- serialized shadow calibration SHA-256:
  `40268801bc7f620ad07e30564db26685404703073f30fe6e5734007ee6a5bd73`
- deployed full-control parameters SHA-256:
  `d877a10c01d2e8263a6316daaf9adcd891021b8dac36fa0fa9c2451f9eb88d96`

The training Dataset contains one run, so the recorded offline metrics are
train-split diagnostics rather than an independent validation result:

- trajectory ADE: `0.196641 m`
- target-speed MAE: `0.157529 m/s`
- sequence MAE `[steering, speed, acceleration]`:
  `[0.09826 rad, 0.61862 m/s, 0.57654 m/s^2]`

## Verification commands

The complete WSL suite was run under the shared training/sync lock:

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
./tools/with_wsl_training_lock.sh ./.venv/bin/python -m pytest -q
```

Result at `9abdf7e`: `447 passed, 32 warnings`.

The tracked source archive was then tested in
`aichallenge-2025-dev:latest` on Graneple. The focused unit/negative suite
passed `70` tests, and this ROS build completed one package:

```bash
source /opt/ros/humble/setup.bash
source /autoware/install/setup.bash
colcon build --packages-select aic_e2e_runtime
```

The limited trial used the actual launch entry point below after the official
AWSIM reset/start helper reported ready:

```bash
ros2 launch aic_e2e_runtime \
  transfuser_lite_v3_full_control_trial.launch.py \
  param_file:=/artifacts/runtime.v3.full_control_trial.9abdf7e.yaml \
  model_path:=/artifacts/last.pt \
  artifact_manifest_path:=/artifacts/runtime_artifact.json \
  calibration_artifact_path:=/artifacts/calibration_v3_stable_2c8388f_shadow.json \
  full_control_evidence_path:=/work/configs/runtime/v3_full_control_trial_authorization.yaml \
  launch_rviz:=true use_sim_time:=true
```

## Actual ROS/AWSIM observations

The preceding M9 shadow run published the predicted trajectory and model
control diagnostics at about 9 Hz, started RViz2 with OpenGL 4.6, and retained
zero publishers on `/control/command/control_cmd`.

That run used an ego-fixed RViz view (`Fixed Frame` and view target both
`base_link`), so it could not visually demonstrate global vehicle motion. The
underlying `/localization/kinematic_state` was nevertheless `map -> base_link`
at about 50 Hz, and the M11 motion measurements below came from global pose and
vehicle velocity rather than screen motion. The RViz profile was subsequently
changed to a map-fixed view with TF, `base_link` axes, localization pose, and
vector-map displays; this display correction does not change the failed-launch
decision below.

During M11, the inference node published nominal commands and the independent
Safety Supervisor was the sole publisher of
`/control/command/control_cmd`. The final 30 s observation produced:

| Measurement | Result |
|---|---:|
| maximum / mean / final speed | `0.012603 / 0.002424 / 0.003526 m/s` |
| displacement | `0.039785 m` |
| final-command acceleration min / mean / max | `0.107219 / 0.255404 / 0.455796 m/s^2` |
| maximum absolute steering command | `0.134082 rad` |
| Safety samples | `600 normal`, no sampled exception |
| direct model decisions | `223` |
| same-trajectory heading fallbacks | `47` |
| sensor-skew rejections | `15` |
| finish reached | no |
| route progress | not established |
| operator intervention | trial stopped after failed launch |
| collision/contact | `NOT_EVALUATED`; no result/contact source was present |

The retained measurement JSON SHA-256 is
`aed02228517f7c260136a9664ee0b89f73e2959f240c849dd992240cbd7d9af4`.
The retained launch-log SHA-256 is
`8f53014ffa686bc874d931e1ad7b156a3fea85c5f7240e712d311ac55beacddb`.
After shutdown, the final-command publisher count was zero and the measured
longitudinal speed was `0.001570 m/s`.

## Changes made during the bounded trial

Four fail-closed runtime defects or mismatches were corrected and covered by
unit/negative tests:

1. heading consistency is evaluated only where predicted speed makes heading
   observable; position, lateral, speed, and endpoint checks remain active;
2. command validity and nominal timeout are `0.45 s`, sized from measured live
   command age/interval while sensor timeouts remain stricter;
3. the previous nominal command is fed back into the next receding-horizon
   model batch and projection state; and
4. an explicitly authorized, bounded, one-shot stopped-launch acceleration
   floor is available while the actual speed remains at or below `0.1 m/s`.

These changes removed the initial preview rejection and command-timeout
oscillation, but they did not make the learned longitudinal sequence sustain
the approximately `0.35 m/s^2` equilibrium command observed during calibration.

## Gate for the next M11 attempt

Do not promote the calibration or raise the fallback acceleration blindly.
First collect and train on multiple independent stopped-start and low-speed
runs whose command labels include the AWSIM drive bias/deadzone, and include a
held-out run/scenario split. The next checkpoint must pass:

1. independent sequence-field and trajectory metrics;
2. stopped-start shadow review with forward, observable trajectories;
3. continuous nominal acceleration above the calibrated launch/equilibrium
   requirement without relying on fallback; and
4. a bounded re-run with an explicit collision/contact or result observer.

Only then repeat M11. This report does not authorize V3-024.
