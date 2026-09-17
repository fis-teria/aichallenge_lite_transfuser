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
  Absolute accuracy and slip remain unvalidated despite the single-lap test below. Local coordinates
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
The GNSS/IMU-free single-lap result is recorded below. Wheel-odometry accuracy
requires separate verification; old GNSS-based laps do not validate this version.

## Initial deployment before AWSIM driving (2026-09-18)

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
The existing `~/e2e_autonomous/time_path_dev_20260917/Makefile` initially forwarded to it,
after verifying the exact old entrypoint hash and that no containers were active.
The old Makefile/metadata are backed up in the new deployment. A `make -n` check
confirmed the new source and speed-variable forwarding. Old source, installs,
traffic deployments and data-collection runs were not overwritten.

Remote receipts are in the new deployment: `install_verification.json`,
`smoke_r2/summary.json`, `launch_smoke/summary.json`, `entrypoint_verification.json`.
Windows local receipts: `tmp/time_no_gnss_20260918/`. At that stage no AWSIM drive
or training was performed; no Git push was made from either Windows or remote hosts.

## AWSIM single-lap verification (2026-09-18)

Tested runtime: `967f3c0fc21595cb26cbcf37826268895bf3dcf2`.
Current deployment: `/home/graneple/e2e_autonomous/time_no_gnss_20260918_r2`,
source directory `source_967f3c0`. The stable entry above now forwards to this
deployment. Checkpoint and vehicle/model policies are unchanged.

```bash
make -C /home/graneple/e2e_autonomous/time_path_dev_20260917 dev \
  MAX_SPEED_KMH=20 CORNER_MAX_SPEED_KMH=10 TIME_RECORD_VIDEO=1 \
  TIME_NPCS=0 TIME_PP_VEHICLES=0 TIME_RUN_ID=codex-time-no-gnss-lap02 DISPLAY=:0
```

Use a new run ID when repeating; existing runs are never overwritten.
The first attempt (`codex-time-no-gnss-lap01`, source `992c1ce`) stopped before
official Start with no positive command: a report captured at 174999996 ns
arrived while `/clock` was still 149999996 ns. The 25 ms lead was incorrectly
latched as `ODOMETRY_STALE_velocity`. The correction buffers original-stamped
vehicle inputs until the source clock catches up. It preserves the 20 ms lead
tolerance, 150 ms capture-age bound, 300 ms wall-receipt bound and bounded queue;
it does not relabel stale inputs or relax freshness checks.

| Result, one ego vehicle / one lap | Observed value |
|---|---:|
| Official result | finished, 1 lap |
| Official lap time | 132.36354064941407 s |
| Official crash / wall / course-out penalty counts | 0 / 0 / 0 |
| Ordered sections | 0, 1, 2, 3, 4, 5, 6, 7, 0 |
| Configured overall / corner ceiling | 20 / 10 km/h |
| Maximum measured speed | 12.990933 km/h |
| Stop confirmed before cleanup | true |
| Cleanup errors / remaining running containers | 0 / 0 |
| Command replay | 2817 matched, 1 unavailable; max difference 6.999e-13 |

During actual driving the controller subscribed only to `/clock`,
`/sensing/lidar/scan`, `/time_path/plan`, `/vehicle/status/steering_status`, and
`/vehicle/status/velocity_status`. It was the sole vehicle-command publisher.
4651 recorded local poses used `time_wheel_odom`. GNSS/IMU/EKF nodes remained
in the ordinary simulator/visualization stack; their output was not a controller
input. This is not a test with those sensor publishers physically disabled.

Of 2818 active commands before the lap stop request, 2797 tracked the model;
8 braked for `MOTION_REAR_LATERAL_INVALID`, 1 for `PLAN_STALE`, and 12 for
`TIME_PATH_INITIAL_DIRECTION`. These recovered without a latched fault. The
unavailable replay is the stale-plan command. Replay checks geometry, PP and
actuator mapping; it does not independently replay scan admission.

Native WSL full pytest after the timing correction: **2996 passed, 4 skipped**
(116.17 s). Official Humble package build, installed source-hash verification,
isolated ROS connection smoke and installed launch smoke passed. AWSIM and RViz
videos (161.2 / 161.7 s) passed transfer hashes and full-frame decode. The three
checked simulator assets (vehicle YAML, Assembly-CSharp.dll and level1) and the
external repository diff hash matched before/after. No training was performed.

Small receipts are in [evidence](evidence/time_no_gnss_20260918/manifest.json).
Raw logs, videos and replay are retained locally in
`tmp/time_no_gnss_awsim_20260918/`; remote run logs remain below the current
deployment. Native WSL replay input/output is in
`/home/thistle/e2e_autonomous/runs/time_no_gnss_awsim_20260918`.

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  tools/evaluate_time_awsim_trial.py \
  --run /home/thistle/e2e_autonomous/runs/time_no_gnss_awsim_20260918/raw \
  --output /home/thistle/e2e_autonomous/runs/time_no_gnss_awsim_20260918/replay
```

This verifies a bounded single-car lap, not repeated-lap reliability, obstacle
avoidance, or LiDAR/map alignment. Proximity occupancy remains diagnostic-only.
Wheel-odometry yaw-rate mean absolute difference from AWSIM VelocityReport was
0.02790 rad/s over 4058 moving samples (95th percentile absolute difference
0.05877 rad/s); that internal comparison is not global-position accuracy.
SLAM, map-wall registration and wheel-slip/drift validation remain open.
