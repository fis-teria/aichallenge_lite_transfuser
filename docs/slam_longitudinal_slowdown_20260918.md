# SLAM front-obstacle longitudinal limiter

Use the existing E2E model/path/tracking steering with an explicit speed-only
limiter. No lateral avoidance, route regeneration or obstacle-driven steering.
Configuration: `configs/control/time_path_slam_slowdown.json`; ordinary
`time_path_dev.json` retains its previous behavior.

The detector uses LiDAR plus wheel-speed/steering odometry, no GNSS/IMU/global
map pose. Current occupied returns overlapping the forward E2E path supply a
distance from the observed base position. The controller subtracts 1.8 m for the
front body, 1 m stopping margin and travel during observation age. It derives a
speed cap assuming 1 m/s² braking and a further 0.5 s response allowance:
`v_cap = sqrt(0.5² + 2 * max(0, remaining_distance_m)) - 0.5`.
These are explicit simulation assumptions; this is not a measured guarantee.
Speeds below 0.05 m/s become a stop. Reductions apply immediately and release
is limited to 0.5 m/s². Existing target speed and acceleration can only decrease.
The selected steering command is retained even while the limiter requests zero.

Input admission checks the publisher, run ID, local frame, valid path, original
scan time (350 ms maximum), local receipt time (350 ms) and plan observation time
(500 ms). Missing, malformed, stale or unmatched observations request braking;
they do not declare the road clear. Recovery is possible on fresh valid input.
Model/vehicle/clock faults retain the existing controller's separate handling.
The usual finite-run supervisor still ends a prolonged standstill.

```bash
# After committing Windows source and safe native-WSL synchronization:
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
# ROS_DOMAIN_ID=93, --network none, official prepared ROS/Cartographer image:
python3 tools/check_slam_slowdown_ros.py --output <new-output-directory>
# Normal finite AWSIM lap, with speed limiting and recorded RViz:
timeout --signal=TERM --kill-after=10s 710s python3 "$DEPLOYMENT/source/tools/run_time_path_awsim_trial.py" \
  --deployment "$DEPLOYMENT" --run-id codex-time-slam-slowdown01 --display :0 \
  --config configs/control/time_path_slam_slowdown.json --ros-launch \
  --max-speed-kmh 20 --corner-max-speed-kmh 10 --npcs 0 --pp-vehicles 0 \
  --slam-obstacles --record-video
# Physical three-box barrier, finish after measured obstacle-induced stop:
# Add --slam-stop-box-test and use a new run ID.
```

The box fixture is scenario setup only. Coordinates never enter inference,
SLAM perception or control. It uses the calibrated standard start-grid 1 and
reference s=44 m placement from the existing scenario tool. Object identity,
moving-vehicle speed, true-pose accuracy and collision-free operation are not
implied by a speed-cap activation. Phantom LiDAR returns and walls can also
cause conservative slowing.

Validation results pending.
