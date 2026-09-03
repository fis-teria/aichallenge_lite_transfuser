# AIC TransFuser Lite ROS 2 runtime

This package keeps the legacy v0 runtime intact and adds a separately named,
strict Dataset-v2 static runtime.  Do not replace `ckpt/best.pt`; v1 uses
`ckpt/transfuser_lite_v1_best_ade.pt` and verifies its SHA-256 before loading.

## v1 input and output contract

| Direction | Default topic | Type / contract |
|---|---|---|
| Sub | `/sensing/camera/image_raw` | `sensor_msgs/msg/Image`, Camera master |
| Sub | `/sensing/lidar/scan` | `sensor_msgs/msg/LaserScan`, native `lidar` frame, exactly 750 beams |
| Sub | `/vehicle/status/velocity_status` | `autoware_auto_vehicle_msgs/msg/VelocityReport` |
| Sub | `/vehicle/status/steering_status` | `autoware_auto_vehicle_msgs/msg/SteeringReport` |
| Pub | `/nominal_control_cmd` | `autoware_auto_control_msgs/msg/AckermannControlCommand` |
| Pub | `/control/command/control_cmd` | safety-supervised command |
| Debug | `/predicted_waypoints` | flattened `[6,2]` waypoints |
| Debug | `/runtime_sync_debug` | Camera stamp, input deltas, true span, accepted flag |
| Debug | `/runtime_control_debug` | compute/age/skew/delay/control values |
| Debug | `/runtime_status` | synchronization and inference state |

The model receives the same tensors as Dataset v2 training: ImageNet-normalized
RGB `[1,3,180,320]`, native LiDAR range plus explicit validity `[1,2,750]`,
and signed measured longitudinal speed divided by 10 `[1,1]`.  Geometry,
frame, checkpoint format, embedded config, and checkpoint hash drift all fail
closed.  A sensor set whose true cross-sensor timestamp span exceeds 30 ms
does not produce a new nominal command; the independent safety timeout then
brakes.

The controller projects waypoints through the configured measured pure delay,
interpolates a continuous preview target, and applies pure pursuit.  The
Dataset-v2 calibration measured `estimated_delay_sec: 0.0`; fault scenarios
must override that parameter explicitly.  Steering-rate limiting remains off
until an authoritative limit is measured.  Curvature speed limiting and model
stop remain disabled because their prerequisites have not been established.

## Build and smoke test

```bash
cd /path/to/aichallenge_lite_transfuser
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --base-paths ros2_ws/src \
  --packages-select aic_e2e_runtime
source install/setup.bash

AIC_V1_CHECKPOINT_PATH="$PWD/ros2_ws/src/aic_e2e_runtime/ckpt/transfuser_lite_v1_best_ade.pt" \
  python3 -m pytest -q \
  ros2_ws/src/aic_e2e_runtime/test/test_checkpoint_smoke_v1.py
```

## Launch

```bash
ros2 launch aic_e2e_runtime transfuser_lite_v1.launch.py \
  use_sim_time:=true \
  model_path:=/aichallenge/workspace/src/aichallenge_submit/aic_e2e_runtime/ckpt/transfuser_lite_v1_best_ade.pt
```

For artificial delay gates, change only the runtime parameter under test, for
example `estimated_delay_sec:=0.1` through a dedicated parameter file.  Do not
encode the fitted first-order steering time constant as pure delay.

The legacy `transfuser_lite.launch.py`, `inference_node`, configuration, and
`ckpt/best.pt` remain available for rollback and v0 comparison.

## v3 ego input contract

V3 inference builds its four ego features only from Wheel Odometry and Steer
Angle reports. It does not subscribe to GNSS, map pose, or
`/localization/kinematic_state`.

| Model feature | ROS message field | Source |
|---|---|---|
| `longitudinal_speed_mps` | `VelocityReport.longitudinal_velocity` | Wheel Odometry |
| `lateral_speed_mps` | `VelocityReport.lateral_velocity` | Wheel Odometry |
| `yaw_rate_rps` | `VelocityReport.heading_rate` | Wheel Odometry |
| `actual_steering_rad` | `SteeringReport.steering_tire_angle` | Steer Angle |

The default topics are `/vehicle/status/velocity_status` and
`/vehicle/status/steering_status`; the four model inputs use SI units.

V3 trajectory-only inference validates both candidate-zero outputs before it
publishes either message and reports success:

| Output topic | Type / contract |
|---|---|
| `/predicted_trajectory` | `Float32MultiArray`, flattened `[N,2]` `(x,y)` in metres |
| `/predicted_trajectory_path` | `nav_msgs/Path`, stamped `[N]` poses in `base_link` for RViz2 |
| `/predicted_speed_profile` | `Float32MultiArray`, `[N]` non-negative target speeds in m/s |
| `/plan_diagnostics` | `std_msgs/msg/String`, JSON raw Plan + `E_plan` + retimed executable reference; shadow-only |

The point counts must match and all values must be finite. Invalid shape,
non-finite values, or negative speed fail closed before either message is
published. These topics are model predictions only: the trajectory-only launch
still has no nominal-control publisher and does not command the vehicle.

`/plan_diagnostics` uses the same Camera observation stamp inside its JSON
payload. For the external-controller shadow profile it also records the preview
time/target and controller command from that sample. The record is diagnostic
only and cannot bypass the Safety Supervisor or acquire nominal authority.

Run the pure shape/unit/negative tests and ROS publisher-ownership contract
with:

```bash
python3 -m pytest -q \
  tests/test_runtime_v3.py \
  ros2_ws/src/aic_e2e_runtime/test/test_safety_p0_v3.py
```

### 2026-09-03 Graneple speed-profile shadow verification

Commit `304754a` passed all 318 repository tests in the WSL training
environment. In `aichallenge-2025-dev:latest`, the 30 focused tests passed and
the `aic_e2e_runtime` ROS 2 package built successfully. A subsequent 70-second
direct observation of the native AWSIM graph recorded:

- matching trajectory and speed-profile messages: 569 each (count delta 0)
- trajectory shape: 569/569 flattened `[15,2]` messages
- speed-profile shape: 569/569 `[15]` messages
- non-finite trajectory or speed values: 0
- speed profiles containing a negative value: 0
- observed predicted-speed range: 0.0070 to 9.1246 m/s
- relay used: no
- `/v3_speed_profile/nominal_control_cmd` publisher: absent

The observation JSON SHA-256 is
`5b36a6efbddd0487e7c97c2421e4c5fce6e3dfd1581b06b4bc48e3c11f59a821`.
This verifies the trajectory-plus-speed output interface only. The model did
not own control authority, its predictions were not sent to the vehicle, and
neither controller quality nor lap completion was tested.

## v3 trajectory-authoritative runtime (M1)

The `trajectory_authoritative` profile is the A-prime nominal path. It validates
and retimes the predicted Trajectory/Speed Plan, applies the 0.75 m/s limited-ODD
and curvature caps, then feeds that same executable reference to the external
delay-aware controller. The inference node owns only `/nominal_control_cmd`; the
separate Safety Supervisor remains the sole publisher of
`/control/command/control_cmd`.

Future Control Sequence is published on `/shadow_model_control_sequence` only.
The legacy `full_control` runtime mode is rejected at node construction so a
rollout-consistency threshold cannot restore direct model-control authority.
Invalid or stopping executable-reference decisions issue a zero-speed braking
proposal to Safety and never fall back to either direct-control Head.

```bash
ros2 launch aic_e2e_runtime \
  transfuser_lite_v3_trajectory_authoritative.launch.py \
  model_path:=/absolute/path/to/checkpoint.pt \
  artifact_manifest_path:=/absolute/path/to/runtime_artifact.json \
  launch_rviz:=true
```

See `docs/v3_m1_trajectory_authoritative_runtime.md`. Source/unit verification
does not establish ROS build, gear/enable/routing, AWSIM launch, or lap completion.

## v3 external-controller shadow

The separate `external_controller` runtime profile passes candidate-zero
trajectory and speed predictions through the ROS-independent delay-aware pure
pursuit/P-speed controller. The 15 predictions correspond to the first 15
Dataset V3 future samples at 0.1 s intervals (0.1 through 1.5 s). Lateral and
longitudinal targets are interpolated at the same delay-adjusted preview time.

This profile publishes `AckermannControlCommand` only on
`/shadow_external_control`. The message is diagnostic: it is not remapped to
`/nominal_control_cmd`, is not consumed by Safety Supervisor, and cannot command
the vehicle. `controller_calibration_status` must remain `unverified` in this
stage. The current 1.087 m wheelbase, 0.0 s delay, and disabled steering-rate
limit are baseline assumptions pending the V3 calibration artifact; they are
not promotion evidence.

Run the pure controller unit/negative tests and publisher-ownership checks:

```bash
python3 -m pytest -q \
  tests/test_shadow_trajectory_controller.py \
  tests/test_runtime_v3.py \
  ros2_ws/src/aic_e2e_runtime/test/test_safety_p0_v3.py
```

Launch the non-authoritative profile with an exact-hash runtime artifact:

```bash
ros2 launch aic_e2e_runtime \
  transfuser_lite_v3_external_controller_shadow.launch.py \
  model_path:=/absolute/path/to/last.pt \
  artifact_manifest_path:=/absolute/path/to/runtime_artifact.json
```

To open the checked-in top-down RViz2 view together with the shadow node, add
`launch_rviz:=true`:

```bash
ros2 launch aic_e2e_runtime \
  transfuser_lite_v3_external_controller_shadow.launch.py \
  model_path:=/absolute/path/to/last.pt \
  artifact_manifest_path:=/absolute/path/to/runtime_artifact.json \
  launch_rviz:=true
```

The green `V3 Predicted Trajectory` display is the model's local future path
in `base_link`; the red points are the native AWSIM LaserScan using best-effort
QoS. It is not a global driven-history trace. The Path uses the source Camera
stamp, contains one pose per validated trajectory point, and is published only
after trajectory/speed/path validation succeeds. RViz2 is optional and has no
control publisher. On a remote desktop host, `DISPLAY` and `XAUTHORITY` must
refer to the logged-in graphical session before launching.

### 2026-09-03 Graneple RViz2 shadow verification

Commit `8824a4a` passed the 40 focused unit/negative/ownership tests and the
362-test combined WSL suite. In `aichallenge-2025-dev:latest`, the 46 focused
tests passed and the ROS 2 package built successfully. With AWSIM running on
Graneple and no Start request sent, the dedicated 1400 x 900 RViz2 window was
confirmed on `DISPLAY=:1`. A 45-second direct observation recorded:

- valid Camera-stamped `nav_msgs/Path` messages: 404
- invalid Path messages: 0
- pose count: 15 for all 404 messages
- frame: `base_link` for all 404 messages
- dedicated RViz2 node present: yes
- V3 ownership of `/nominal_control_cmd`: absent

The observation JSON SHA-256 is
`c03a6f0e3e20a3e64c5dc1ebc6029220213fbe5262f64c52ad6935e145d2e2b4`.
The container reported direct-rendering driver fallback warnings, but RViz2
initialized OpenGL 4.5 and its X11 window remained alive throughout the
observation. This check visualized the local predicted trajectory and native
LaserScan only. The vehicle was not started, the shadow command was not
connected to Safety Supervisor or vehicle control, and lap completion was not
tested.

Before any connection to Safety Supervisor or vehicle control, complete the V3
calibration artifact, validate the controller against held-out scenarios, and
run a separate authority review.

### 2026-09-03 Graneple external-controller shadow verification

Commit `09bd0a5` passed all 325 repository tests in WSL. The official
`aichallenge-2025-dev:latest` environment passed the 39 focused tests and built
the ROS 2 package. A 70-second direct AWSIM observation then recorded:

- matching valid trajectory/speed outputs: 577 (`[15,2]` and `[15]`)
- finite, stamped shadow proposals: 308
- invalid or zero-stamp shadow proposals: 0
- steering range: -0.5979 to 0.6000 rad
- commanded-speed range: 0.0536 to 8.8700 m/s
- acceleration range: -0.8552 to 2.0000 m/s^2
- V3 `/nominal_control_cmd` publisher: absent
- relay used: no; calibration status: `unverified`

The remaining 269/577 controller attempts (46.6%) failed closed because the
0.5 s preview target was not ahead of the ego frame. This is a model/controller
quality blocker, not a synchronization or QoS failure, and must not be hidden
by clamping the preview point. The observation JSON SHA-256 is
`bef70c0c590a6882bb6a9535f561ee1a189ecba5602dc94fc2e26eb892ea85c2`.

This run did not publish nominal or final vehicle commands, did not evaluate
Safety Supervisor intervention, and did not test lap completion.

## v3 model-control shadow authority

The `shadow_control` profile requires a runtime artifact with the
`current_control` capability and publishes candidate-zero model control only
as a Camera-stamped diagnostic on `/shadow_model_control`. The pure authority
contract fixes three distinct owners:

| Role | Owner | Topic |
|---|---|---|
| Debug model proposal | `inference_node_v3` | `/shadow_model_control` |
| Nominal vehicle proposal | external controller | `/nominal_control_cmd` |
| Final vehicle command | Safety Supervisor | `/control/command/control_cmd` |

The V3 shadow launch starts only the inference/debug adapter. It deliberately
does not start, remap, or replace the authority-bearing external controller or
Safety Supervisor. Therefore this launch alone cannot command the vehicle:

```bash
ros2 launch aic_e2e_runtime transfuser_lite_v3_shadow.launch.py \
  model_path:=/absolute/path/to/last.pt \
  artifact_manifest_path:=/absolute/path/to/runtime_artifact.json \
  launch_rviz:=true
```

Run the authority, shape, finite-value, negative-speed, and source-ownership
tests with:

```bash
python3 -m pytest -q \
  tests/test_runtime_authority_v3.py \
  ros2_ws/src/aic_e2e_runtime/test/test_safety_p0_v3.py
```

Publishing the debug proposal does not establish actuator calibration,
trajectory-control consistency, Safety intervention, or closed-loop quality.

### 2026-09-03 Graneple model-control shadow verification

Commit `a49b5ce` passed the 51 focused authority/runtime/ownership tests and
the 399-test combined WSL suite. Its tracked archive SHA-256 is
`8145034c8d8a4beee25192b80e40adbf3dda89b3794441959d8db5946d99f325`.
The official `aichallenge-2025-dev:latest` image built the ROS package and
passed the same 51 focused tests.

AWSIM was then started without sending its Start request, and the optional V3
RViz2 window was displayed on Graneple `DISPLAY=:1`. A 45-second direct
observation recorded:

- finite Camera-stamped `/shadow_model_control` proposals: 251
- invalid proposals and zero timestamps: 0
- predicted paths: 252
- steering range: -0.2237 to -0.0999 rad
- predicted speed range: 2.1982 to 3.0764 m/s
- predicted acceleration range: -0.1482 to -0.0849 m/s^2
- V3 ownership of nominal or final control topics: absent
- dedicated RViz2 node present: yes

There were 22 fail-closed inference rejections during the bounded observation.
The observation JSON SHA-256 is
`8dfefb91cff59acfa774b964fc17bd4dee2f6af72d091a5640a0f8a8658bb21c`.
The external controller and Safety Supervisor were not launched by this
shadow profile, model proposals were not connected to the vehicle, and no lap
or closed-loop controller-quality result is claimed.

Camera and LaserScan subscriptions use ROS 2 Sensor Data QoS (best effort,
volatile) so the V3 node can consume the native AWSIM publishers without a
reliability-changing relay. Wheel Odometry and Steer Angle keep their existing
reliable subscriptions; command, safety, and status topics are not changed by
this sensor compatibility rule. Do not apply best effort globally.

The source-level QoS contract and its negative regression check run with:

```bash
python3 -m pytest -q \
  ros2_ws/src/aic_e2e_runtime/test/test_safety_p0_v3.py
```

In the official AWSIM graph, verify the native endpoints and then confirm that
V3 publishes without a test relay:

```bash
ros2 topic info --verbose /sensing/camera/image_raw
ros2 topic info --verbose /sensing/lidar/scan
ros2 topic echo --once /v3_shadow/runtime_status
```

QoS compatibility only establishes delivery. Timestamp skew, model accuracy,
control authority, and lap completion remain separate promotion gates.

### 2026-09-03 Graneple direct-QoS verification

Commit `37a1e55` was built in `aichallenge-2025-dev:latest` and connected to
the native AWSIM graph on `graneple@192.168.3.10` without a QoS relay. The
70-second trajectory-only observation recorded:

- Camera: 598 messages
- LaserScan: 1,257 messages
- finite 15-point trajectories: 270
- non-finite trajectories: 0
- incompatible-QoS warnings: 0
- rejected sensor-skew callbacks: 325

This verifies native sensor delivery only. V3 retained no control authority,
the test was stopped after the bounded observation, and no lap-completion claim
is made. The remaining sensor-skew rejection requires timestamp-buffered
synchronization rather than a further QoS change.

## v3 buffered timestamp synchronization

V3 keeps the 30 ms cross-sensor span gate. A Camera callback is queued until
LiDAR, Wheel Odometry, and Steer Angle have each observed the Camera timestamp
or a later timestamp. Only then does the runtime choose the nearest sample per
role, with past samples winning an equal-distance tie. Selected sensor samples
are consumed once. Callback arrival order is therefore not treated as sensor
time order.

If nearest matching selects a sensor sample slightly later than the Camera
stamp, the accepted observation waits in a second bounded queue until ROS
simulation time reaches every selected stamp. `sync_clock_poll_sec` controls
that readiness check. The strict future-timestamp validator remains enabled;
the runtime does not weaken it to accommodate callback ordering.

The bounded Camera queue is configured with `sync_queue_size`; overflow,
non-increasing timestamps, final skew rejection, stale observations, and future
timestamps remain explicit fail-closed status values. `runtime_sync_debug`
publishes Camera stamp, the three signed sensor deltas in ms, the final true
span in ms, and an accepted flag.

Run the ROS-independent synchronization unit and negative tests with:

```bash
python3 -m pytest -q \
  tests/test_sensor_sync_v1.py \
  tests/test_sensor_sync_v3.py
```

### 2026-09-03 Graneple buffered-sync verification

Commit `504b4ed` was tested for 70 seconds against the same native AWSIM topics
and model artifact as the direct-QoS check, without a relay and with the 30 ms
limit unchanged:

- Camera observations: 588
- finite 15-point trajectories: 565 (96.1%)
- final sensor-skew rejections: 23 (3.9%)
- future-timestamp and stale rejections: 0
- synchronization span: p50 14.33 ms, p95 29.22 ms
- non-finite trajectories and incompatible-QoS warnings: 0

For comparison, callback-latest commit `37a1e55` published 270 of 598 Camera
observations (45.2%) and rejected 325 for skew. The first buffered revision
reduced skew but exposed 172 strict future-timestamp rejections while ROS time
lagged a selected future-side sample; the bounded runtime-clock queue removed
those rejections without weakening timestamp validation. This remains a
trajectory-only shadow test and is not a lap-completion or control-authority
claim.

The shared Safety Supervisor keeps `ego_speed_source:=odometry` as its default
so the frozen V1 launch and runtime behavior remain compatible. A V3 parameter
file must explicitly select `ego_speed_source:=velocity_report`; unsupported
source names are rejected during node construction.

## V3 automated calibration capture

`calibration_capture_v3.launch.py` starts only the hash-armed excitation node
and an independent Safety Supervisor. The intended entry point is the
dry-run-first collector, which also records the exact V3 calibration topics and
writes machine-readable result/provenance manifests:

```bash
PYTHONPATH=src python3 tools/collect_calibration_v3.py \
  --plan configs/calibration/excitation_steering_low_speed_v1.yaml \
  --topic-profile configs/data/topic_profile_v3.yaml \
  --output-root /absolute/native/linux/path/calibration_bags/v3 \
  --run-id steering_r01 \
  --scenario-id awsim_calibration_pad
```

The collector creates no ROS process unless `--execute` is supplied. Execution
requires zero pre-existing publishers on both command topics. The launch then
requires the excitation node to be the sole nominal publisher and Safety to be
the sole final publisher; stale telemetry, unsafe speed, a non-normal Safety
reason, publisher drift, or subscriber loss aborts to a held stop command.

See `docs/v3_calibration_capture.md` for the environment, plan bounds, repeated
run procedure, Dataset conversion, and promotion boundary. No AWSIM excitation
or completed calibration capture is implied by source/unit verification alone.
