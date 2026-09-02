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

The shared Safety Supervisor keeps `ego_speed_source:=odometry` as its default
so the frozen V1 launch and runtime behavior remain compatible. A V3 parameter
file must explicitly select `ego_speed_source:=velocity_report`; unsupported
source names are rejected during node construction.
