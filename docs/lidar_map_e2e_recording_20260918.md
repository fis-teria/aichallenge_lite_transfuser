# Strong map correction alongside E2E: RViz recording

This trial runs the existing GNSS/IMU-free E2E controller and the
`simulation_aggressive` map localizer concurrently. Corrected localization is
**display/evaluation only**, not an E2E model or controller input. The manual
initial pose is explicit. Ground truth, if published, is recorded by a separate
read-only observer and never enters the localizer or controller.

In the normal RViz window, cyan points/arrow show corrected LiDAR/pose and
orange points show the original LiDAR using the existing visualization TF.
The wide view follows `base_link`. Original sensor TF is unchanged. The RViz
configuration is backed up and restored by the runner. Existing map/raw display
TF may use the official Autoware localization; that is not an E2E control input.

After preparing a dedicated deployment with matching installed ROS sources and
the hash-verified `command_off_best.pt`, run on the AWSIM host:

```bash
timeout --signal=TERM --kill-after=10s 710s python3 "$DEPLOYMENT/source/tools/run_time_path_awsim_trial.py" \
  --deployment "$DEPLOYMENT" --run-id codex-time-lidar-record01 --display :0 \
  --config configs/control/time_path_dev.json --ros-launch \
  --max-speed-kmh 20 --corner-max-speed-kmh 10 --npcs 0 --pp-vehicles 0 \
  --record-video --lidar-map-comparison --lidar-map-initial-pose 89631 43128 2.12
```

The runner waits for valid corrected scans, an RViz subscription, and the
expected GNSS/IMU-free controller/localizer input graph before starting motion.
Localization rejection during the lap is logged; it does not change E2E motion.
`localization_observations.jsonl` includes timestamped estimated/reference poses,
available ground-truth topic types, and graph snapshots. `localization_status.jsonl`
contains matcher status. Acceptance is not position accuracy. `rviz.mp4` and
`awsim.mp4` are finite window-only recordings; the normal one-lap stop and outer
timeout remain active. Live results will be added after the experiment.
