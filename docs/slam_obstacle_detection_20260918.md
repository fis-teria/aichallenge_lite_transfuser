# LiDAR + wheel-odometry local SLAM obstacle detection

Base-map wall matching is suspended. This separate mode runs the existing
Cartographer installation with scan + current E2E wheel odometry only. No
preloaded Lanelet/map, GNSS, IMU, reference/global pose or simulator objects enter
perception. The E2E model and motion controller remain unchanged.

The scan-time SLAM pose aligns current occupied surfaces and a recent occupancy
grid (50 m square, 0.2 m cells, 2 s TTL). Free rays clear previous hit locations;
unknown remains unknown. Detection always uses current returns, including
stationary obstacles already present in the grid. No subtraction of newly
learned "background" can erase a stopped car. Surface IDs are tentative; this
does not classify vehicles or estimate their true speed.

Fresh E2E waypoints are transformed at their original observation-time SLAM pose.
The detector reports observed points overlapping a 0.85 m half-width path
corridor plus a 1.8 m front extension. Without a matched fresh plan,
`path_valid=false` and `path_blocked=null`, never an empty-road assertion.
This diagnostic corridor is not a certified vehicle swept footprint. No brake,
steering or target-speed command is published by the SLAM/detection sidecar.

The original AWSIM scan is preserved. Only Cartographer's copy uses the verified
snapshot interpretation (`time_increment=0`), dedicated frame `time_slam_lidar`,
and +inf for invalid/saturated returns. Mount: base forward 1.65 m, z 0.0377 m.
Wheel odometry is converted from base origin to LiDAR origin for Cartographer.
SLAM local matching is active; global loop-closure optimization is disabled.
Cartographer's TF subscriptions are isolated from the normal Autoware TF tree.

Outputs:

| Topic | Meaning |
|---|---|
| `/time_path/slam/scan` | Scan displayed at its own SLAM observation pose |
| `/time_path/slam/pose` | Local SLAM base pose, no global-map alignment |
| `/time_path/slam/recent_occupancy` | Recent observed hit/free/unknown cells |
| `/time_path/slam/obstacles` | Occupied surfaces, confirmation, path overlap |
| `/time_path/slam/markers` | Yellow surfaces; red overlaps predicted path |
| `/time_path/slam/path` | Same E2E output expressed in the local SLAM frame |
| `/time_path/slam/status` | Validity, missing/stale inputs, counters and graph |

Dedicated RViz uses `time_slam_map`; it does not overlay the suspended base map.
Scan/SLAM timeouts clear markers and mark results invalid. Backwards clocks
require a restart of the coupled SLAM session. Display availability is not
localization accuracy, and phantom LiDAR returns can still appear as occupancy.

```bash
# Native WSL validation, after Windows commit/sync:
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
# Official ROS image with prepared Cartographer and matching runtime install:
python3 tools/check_slam_obstacles_ros.py --output <new-smoke-directory>
# Dedicated AWSIM host deployment, same finite E2E runner:
timeout --signal=TERM --kill-after=10s 710s python3 "$DEPLOYMENT/source/tools/run_time_path_awsim_trial.py" \
  --deployment "$DEPLOYMENT" --run-id codex-time-slam-obstacles01 --display :0 \
  --config configs/control/time_path_dev.json --ros-launch \
  --max-speed-kmh 20 --corner-max-speed-kmh 10 --npcs 1 --pp-vehicles 0 \
  --record-video --slam-obstacles
```

The runner requires a prepared install at
`/home/graneple/e2e_autonomous/cartographer_extrap_build_20260910`, records the
actual binary/config hashes, checks fresh SLAM output and RViz subscription
before motion, owns all child processes, and restores normal RViz afterwards.
`--slam-obstacles` and `--lidar-map-comparison` are mutually exclusive.

Validation and AWSIM results pending. Static obstacle detection, moving vehicle
classification, stopping/avoidance control and true-pose accuracy are separate
claims and must be supported individually.
