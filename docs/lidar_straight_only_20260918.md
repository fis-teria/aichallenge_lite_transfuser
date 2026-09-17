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
## Live result: codex-time-lidar-straight01

Source/install `bd98c2f4f4c1b09729cd01376a5b155ef763ffd4`, dedicated deployment
`/home/graneple/e2e_autonomous/time_lidar_straight_20260918`. The existing model,
checkpoint, manual initial pose, steering assets and 20/10 km/h speed caps match
the previous all-section correction trial. This is a separate closed-loop run,
not replay of identical sensor observations.

- One lap completed in **132.8840484619 s**, penalty count **0** (crash/wall/over).
  Stop confirmed, controller fault null, actual peak speed 3.623043 m/s.
- **712** distinct processed scan timestamps between controller arming and
  confirmed stop (144.8 simulation seconds, including travel to the lap line
  and stopping). Gate disabled on **559**; matcher calls while disabled **0**.
- **244** scans published using `ODOMETRY_ONLY`. Changes to the held map-to-odom
  transform across these predictions: **0**. Confirmed fresh map matching
  published **96** scans. Total displayed scans **340/712 (47.8%)** includes
  wheel prediction and is **not map-matching success or position accuracy**.
- Matching was attempted on 153 scans and accepted on 107; 11 accepted samples
  remained in initialization. Insufficient support rejected 46 attempts.
  `WAIT_STRAIGHT` accounted for 315 samples without a confirmed usable anchor.
  The largest accepted map correction was 7.08 m: amplitude limits remain off.
- ROS graph again confirmed no GNSS/IMU/global-pose inputs to inference,
  controller, or localizer. The observer alone recorded the reference pose.
  Map correction still has no E2E control input or command authority.
- RViz: **162.4 s**, 1624 frames, 1280x786; AWSIM: **161.9 s**, 1619 frames,
  1280x960. Both H.264, 10 fps, no audio, full-file decoding passed. Extracted
  frames showed cyan predicted/corrected scans and orange original scans.
- Native WSL full pytest: **3070 passed, 4 skipped**, 84 warnings, 132.83 s.
  Targeted localization/RViz tests: 22 passed. Official ROS bounded, aggressive,
  and straight-only smokes passed; the latter verified frozen corner TF.
- All owned containers stopped with no cleanup errors. Normal RViz restored
  byte-for-byte (SHA256 `981c28955e9bceedb8845ec8d59e57c4f47805040cd84c5e09f2f1430c7bb3f1`),
  and the standard development entrypoint was unchanged.

The schedule works, but reliable localization was **not** achieved. The video
still shows large offsets after returning to straight sections and intervals
where support is insufficient to resume output. Do not interpret completion
of this E2E lap as improved driving caused by the map localizer.

Ground truth was again unavailable. Against the existing Autoware
`/localization/pose` estimate (NOT AWSIM ground truth), the 340 published poses
have XY median **16.12 m**, 95th percentile **32.73 m**, maximum **37.58 m**,
and yaw median **23.68 degrees**. 62/340 are within 1 m AND 10 degrees.
The 96 fresh-match poses alone have XY median **22.29 m**, and 36/96 within
1 m AND 10 degrees. Fourteen conflicting duplicate reference timestamps were
excluded; interpolation gaps were limited to 100 ms.

For context, the previous all-section trial published 269/713 scans, with
reference XY median 9.15 m and maximum 63.73 m. Populations differ (especially
the added wheel predictions), so these figures do not establish a paired
accuracy improvement. Straight-only gating has not resolved the underlying
registration problem.

Small evidence files: `docs/evidence/lidar_straight_e2e_20260918/`.
Videos, raw journals and analysis script remain outside Git at
`E:/workspace/e2e_lite_transfuser/artifacts/lidar_straight_e2e_20260918/`.
RViz SHA256 `565ff3a8c3babb34430be8333f99a693bfc657243be62b82691122bec79e83b6`;
AWSIM SHA256 `9b33440e6d4c6bd0b1a59aa9c41db0100a076a3c3c18470bb513a83f06b7f749`.
The originals remain in the dedicated remote deployment/run directory.
