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
timeout remain active.

## Live result: codex-time-lidar-record01

- Source/install: `1e32bde4b014c25770693dfe4e4ef05a4d586d8a`;
  dedicated deployment `/home/graneple/e2e_autonomous/time_lidar_record_20260918`.
- Completed one lap, official result **132.3585357666 s**, penalties **0**
  (crash/wall/over all zero). Stop confirmed; controller fault was null.
- Actual peak speed 3.594745 m/s (12.94 km/h); requested caps were 20/10 km/h.
- Live graph confirmed no GNSS/IMU/global-pose inputs to inference, controller,
  or localizer. Corrected localization was not supplied to E2E control.
- Both videos PASS: RViz 162.0 s, 1280x786, 1620 frames; AWSIM 161.5 s,
  1280x960, 1615 frames. Both H.264 at 10 fps, no audio. Full-file decoding
  passed, and extracted RViz frames visibly showed the two coloured scans.
- RViz returned byte-for-byte to its backup (SHA256
  `981c28955e9bceedb8845ec8d59e57c4f47805040cd84c5e09f2f1430c7bb3f1`).
  The normal development entrypoint was unchanged. No running containers
  remained; cleanup reported no errors.
- Native WSL `tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q`:
  **3047 passed, 4 skipped**, 84 warnings, 127.23 s. Official ROS install
  verified 248 source files; bounded and aggressive synthetic smokes passed.

The localization result remains poor. From controller arming to confirmed
stop (144.75 simulation seconds, including travel to the lap start line and
stopping), the localizer processed 713 distinct scan timestamps and published
269 corrected scans (**37.7%**). This is availability at the localizer's
approximately 5 Hz processing rate, **not position accuracy**. The recording
shows incorrect alignments at corners and gaps when support is insufficient.

`/awsim/ground_truth/vehicle/pose` was not published, so true accuracy cannot
be claimed. For the 269 valid pose samples, timestamp-interpolated comparison
against the existing `/localization/pose` estimate gives XY median **9.15 m**,
95th percentile **48.44 m**, maximum **63.73 m**, and yaw median **44.35 deg**.
59/269 samples are within 1 m AND 10 deg; 2/269 within 0.5 m AND 5 deg.
Conflicting duplicate reference timestamps were excluded; interpolation gaps
were limited to 100 ms. This reference is an Autoware estimate, not ground truth.
The completed lap therefore demonstrates concurrent operation/recording,
not improved driving from map correction or reliable map localization.

Small result files are retained in `docs/evidence/lidar_map_e2e_recording_20260918/`.
Videos and raw journals remain outside Git at
`E:/workspace/e2e_lite_transfuser/artifacts/lidar_map_e2e_20260918/` and in the
dedicated remote run directory. RViz SHA256:
`d7c91780950227073f1e873098b8a01463af4f97a93d72fb278e1eac62518896`;
AWSIM SHA256:
`8c87471b0926dec79c3e6a939bd2c55489cc19340a3defe53e8bf0bbdd5744da`.
