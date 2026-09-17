# TimePath control without GNSS/IMU

The TimePath controller now derives its observation/current poses from reported
longitudinal speed and measured tire steering, in `time_wheel_odom`. It no longer
subscribes to `/localization/kinematic_state`, IMU, GNSS or TF. Both ROS launch
and the Python trial launcher use this same controller. This changes the
execution of the existing model; it does not retrain or change its input schema.

## Coordinate and input contract

- `TimeControlOdometry`: scalar speed [m/s], measured physical tire angle [rad],
  original capture stamps [ns]. Steering must bracket the speed stamp; no
  extrapolation or current-time relabeling. Maximum input/integration gap: 0.15 s.
- Rear-axle motion uses the existing vehicle response policy and wheelbase
  1.087 m. The explicit rear-axle/base_link offset transforms the integrated pose
  back to base_link. Origin is the first accepted sample, not map/GNSS position.
- A 256-pose history provides the existing observation-time interpolation and
  scan-time alignment. Faults, invalid source/frame, gaps and clock resets brake;
  there is no EKF fallback. The ordinary plan, source, age and command guards remain.
- This is wheel/steering dead reckoning, not LiDAR odometry or map localization.
  Slip and accuracy at operating speed remain unvalidated. Local coordinates
  must not be used as global map coordinates or to follow a global teacher route.
- `--recovery-reference` / `--recovery-side` evaluation bootstrap is rejected
  before starting nodes/AWSIM: that old evaluation depended on a global teacher.
  Separate teacher data-collection tools, stored datasets and past runs are unchanged.

The learned model still receives the existing `VelocityReport` channels (forward
speed, lateral speed, yaw rate) and steering. The existing stopping-motion check
also uses reported lateral speed/yaw rate. In the inspected AWSIM decompiled
`VehicleReportRos2Publisher`, these come from `vehicle.LocalVelocity` and
`vehicle.AngularVelocity`, not GNSS or IMU topics. This is an AWSIM interface
contract, **not** a claim that a physical wheel encoder measures lateral speed.
Any other vehicle publisher needs its own input provenance check.

The ordinary Autoware visualization/teacher stack can still run localization
nodes. Their global pose is not a TimePath command input. Mixed-traffic ego
readiness no longer gates on global odometry; background baseline cars retain
their own readiness checks. Optional evaluator global-pose journals are not
used for ego steering, acceleration, or stop admission. GNSS/IMU-free launch of
the entire simulator/Autoware stack is not claimed by this controller change.

## Reproduction

Commit on Windows, use the unchanged `tools/sync_to_wsl.ps1` workflow (a clean
Windows clone of the exact commit if unrelated staged work exists), then:

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
```

Build `aic_e2e_runtime` in the official Humble image. In an isolated container
with `--network none`, `ROS_DOMAIN_ID=93`, the matching checkpoint and installed
package, run:

```bash
python3 tools/check_time_ros_connection.py \
  --checkpoint /time/command_off_best.pt \
  --checkpoint-sha256 1fe12ca066791d3eb8907c6921120e2cfbb38925c422d5be54940f4acbdd120a \
  --trial-config configs/control/time_path_dev.json --output /time/smoke_new
```

This fixture publishes camera/scan/vehicle reports/clock and synthetic plans,
with **no GNSS, IMU or external pose publisher**. It checks local pose generation,
actual control-node subscriptions, shadow PP/scan checks and braking failures.
It never publishes vehicle commands and does not prove closed-loop performance.

On a source-verified prepared deployment, the entry remains:

```bash
make dev DEV_CONTROLLER=time MAX_SPEED_KMH=20 CORNER_MAX_SPEED_KMH=10
```

The existing proximity `log_only_awsim_v1` diagnostic and speed/stopping-distance
settings are not changed by this task. This is not an obstacle-avoidance release.
AWSIM driving, lap completion and wheel-odometry accuracy require separate
closed-loop verification; old GNSS-based lap results do not validate this version.

## Verified deployment (2026-09-18)

Runtime commit: `992c1ce7effaa1374a9fcd4a313974bb70d409b3`.

- Native WSL shared-lock focused suite: 96 passed. Full suite: **2992 passed,
  4 skipped**, 130.54 s. Skips are existing optional OSQP/schema/official-package checks.
- Official Humble image: package build and installed Python source hashes passed.
- Isolated ROS smoke: PASS, no GNSS/IMU/external-pose publishers, five controller
  subscriptions only (clock, scan, model plan, velocity, steering). 257 local-pose
  records, 109 positive shadow controls, 109 stopping-sweep evaluations. Stale
  plans, paused clock, overspeed, infeasible steering and invalid reported motion
  produced braking. Vehicle command publisher count: **0**.
- Installed ROS launch smoke: PASS, both processes, recorded read-only speed
  parameters and default shadow-only output verified.
- First ROS smoke stopped before node startup due to an unwritable default ROS
  log directory in the uid-1000 container. Retry used `ROS_LOG_DIR=/time/ros_logs`;
  the failure remains recorded alongside the successful `smoke_r2` result.

New prepared deployment: `/home/graneple/e2e_autonomous/time_no_gnss_20260918`.
The existing `~/e2e_autonomous/time_path_dev_20260917/Makefile` now forwards to it,
after verifying the exact old entrypoint hash and that no containers were active.
The old Makefile/metadata are backed up in the new deployment. A `make -n` check
confirmed the new source and speed-variable forwarding. Old source, installs,
traffic deployments and data-collection runs were not overwritten.

Remote receipts are in the new deployment: `install_verification.json`,
`smoke_r2/summary.json`, `launch_smoke/summary.json`, `entrypoint_verification.json`.
Windows local receipts: `tmp/time_no_gnss_20260918/`. No AWSIM drive or training
was performed; no Git push was made from either Windows or remote hosts.
