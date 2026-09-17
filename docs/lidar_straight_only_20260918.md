# Straight-only map correction alongside E2E

The experiment retains `simulation_aggressive` registration but sets
`correction_schedule=straight_only`. E2E model and controller inputs remain
unchanged; localization is still display/evaluation only, with no motion authority.

Straight detection uses differences between scan-time wheel odometry poses.
It requires absolute curvature <= 0.02 /m AND yaw rate <= 0.06 rad/s for 0.6 s.
It exits immediately at curvature > 0.03 /m OR yaw rate > 0.10 rad/s. These are
motion-classification thresholds, not limits on correction amplitude. No GNSS,
IMU, reference pose, map route position, or ground truth enters this decision.

During turns and the return-to-straight dwell, the matcher is not called and
`map->time_wheel_odom` remains constant. With a previously confirmed anchor,
fresh wheel/scan inputs continue to publish the predicted pose and cyan scan.
Status is `ODOMETRY_ONLY`, `pose_source=wheel_prediction`, and
`map_match_attempted=map_match_applied=false`. This is available dead reckoning,
not a fresh map match or a statement of accuracy. Unconfirmed initialization,
rejected matches, >0.5 s scan-time gaps, clock resets, and stale input do not
authorize a valid predicted pose. A return to straight after continuous corner
prediction can resume map registration without a forced reinitialization.

Existing default `all` preserves correction in both straights and corners.
The standalone ROS launch exposes `correction_schedule:=straight_only`.
For the same finite, single-ego 20/10 km/h E2E/RViz comparison as the previous lap:

```bash
timeout --signal=TERM --kill-after=10s 710s python3 "$DEPLOYMENT/source/tools/run_time_path_awsim_trial.py" \
  --deployment "$DEPLOYMENT" --run-id codex-time-lidar-straight01 --display :0 \
  --config configs/control/time_path_dev.json --ros-launch \
  --max-speed-kmh 20 --corner-max-speed-kmh 10 --npcs 0 --pp-vehicles 0 \
  --record-video --lidar-map-comparison --lidar-map-initial-pose 89631 43128 2.12 \
  --lidar-map-correction-schedule straight_only
```

Validation: native WSL through `tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q`;
official ROS smoke through `python3 tools/check_lidar_map_ros.py --correction-mode simulation_aggressive
--correction-schedule straight_only --output <new-output-directory>`. The ROS smoke
checks actual constant map TF during a synthetic turn, continued display outputs,
return-to-straight matching, timeout behavior, and the GNSS/IMU-free input graph.
Live results are pending.
