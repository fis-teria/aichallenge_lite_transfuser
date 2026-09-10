# Time-based teacher collection pilot on SI26-PC008

2026-09-11: connected as `si26-pc008@192.168.3.13` (the spelling
`si26-pc0008` failed authentication), working beneath
`/home/si26-pc008/git/autonomous_ai`. No remote push. Existing dirty source files
and the unrelated `ga_yaw_slip_ablation_ws-rosbag-cleaner-1` were preserved.

## Scope and execution

Single simulated vehicle, camera and LiDAR enabled, existing Pure Pursuit,
requested speed 0.8333333333 m/s. No learning-model controller. Three bounded
startup/collection attempts; the third completed a 60-s drive window and braking.
Dedicated Docker names `time-teacher-20260911-pilot01` through `pilot03`.
Only the owned containers were stopped. None remain running.

Per-attempt host artifacts:
`/home/si26-pc008/git/autonomous_ai/ai-work/raw/time_teacher_20260911_pilotNN/`.
Each contains `boot.bash` and exact Docker argv in `docker_command.json`.
Pilot02/03 contain `collect.py`, probe logs, SQLite ROS bag and launch logs.
Source was mounted read-only; existing application code/configuration was not edited.
The inherited harness references `.harness/` and a `codex-dev-harness` skill which
were absent at the inspected paths; collection used the available project scripts.

The working simulator arguments are normal rendered mode (not `-headless`),
`--venue citycircuit --vehicles 1 --npcs 0 --boosts 0 --camera true --lidar true
--start-mode sync --start-count-seconds 3 --laps unlimited --timeout 240
--steer-source ackermann --sound off --collisions on`.
No real vehicle devices were mounted. DDS was restricted to loopback using the
existing CycloneDDS configuration. Runtime GPU observation was approximately
1.4 GiB allocated and 45-46% utilization, not a load benchmark.

Autoware launch arguments:
`simulation:=true use_sim_time:=true run_rviz:=false domain_id:=1
control_method_override:=pure_pursuit use_external_target_vel:=true
external_target_vel:=0.8333333333 reference_execution_speed_cap_mps:=0.8333333333
rosbag:=false` (the dedicated collector records explicit topics separately).
Before Start, pilot03 set the trajectory generator `csv_path` to
`/aichallenge/workspace/src/aichallenge_submit/simple_trajectory_generator/data/raceline_awsim_15km.csv`.
The speed ceiling remained 0.8333333333 m/s despite the CSV filename.

Do not blindly rerun the pilot scripts as a production collector: the startup
watchdog, stopping timestamps and source ownership checks need hardening first.

## Results and exclusions

- Pilot01: inherited H2H headless/CPU sensor options were incompatible with this
  binary. Headless explicitly disabled camera/LiDAR. No teacher bag was accepted.
- Pilot02: default reference produced near-stationary behavior after initial
  movement. Dynamic `external_target_vel=0` was accepted by ROS but did not change
  the cached controller output. `fault=null` in the probe is NOT successful-braking
  evidence. Preserve this attempt as diagnostic, exclude it from normal teachers.
- Pilot03: changed to the existing AWSIM reference. Requested drive window 60 sim s;
  speed-integrated distance 24.9424 m, maximum measured speed 0.488959 m/s
  (1.76025 km/h), not the requested 3 km/h. Approximately 67.3 s total recorded.
  Controller process was interrupted before the sole final brake command;
  velocity subsequently reached zero and remained there for over 3 sim s before
  simulator shutdown. Brake command had a zero timestamp: retain as intervention
  metadata, not an ordinary timestamped control-history teacher.

Pilot03 saved 642 camera frames (384x256 BGR, median interval 0.105 s), 1,347
LiDAR scans (median 0.050001 s), 3,368 kinematic-state messages (median 0.020 s),
1,926 velocity reports, 1,925 steering reports, 1,346 GNSS fixes and 1,347 IMU
messages. Camera/LiDAR nearest-stamp skew p95 23.94 ms, maximum 28.21 ms.
No nonpositive camera/LiDAR/kinematic-state timestamp intervals were found.
Start/mid/end camera samples were decoded and visually checked; this does not
establish collision-free or full-route quality. Command timestamps contain
duplicates and the final zero timestamp, requiring explicit treatment before
canonical conversion.

These are raw teacher candidates, NOT a finalized 3-second training dataset.
GNSS/IMU, actual velocity/steering and localization estimates are present;
`/localization/kinematic_state` is an estimate, not verified simulator ground truth.
`/v2x/vehicle_positions` recorded zero messages. Full pose provenance, frame
calibration, maneuver quality, usable future masks and run-level splits remain
to be validated. Initial demonstrated motion is chiefly a short straight section.

## WSL verification and preservation

Pilot02/03 copied to
`/home/thistle/e2e_autonomous/runs/time_teacher_si26_20260911/`.
Pilot03 bag SHA256 matched host and WSL:
`6fbde69113d94c632ceade1f113cf5a931eea2f239d19806f27a8943e6f37fe5`.

Read-only evaluation command (writes derived reports beside the bag):

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
bash tools/with_wsl_training_lock.sh .venv/bin/python \
 /home/thistle/e2e_autonomous/runs/time_teacher_si26_20260911/audit.py \
 /home/thistle/e2e_autonomous/runs/time_teacher_si26_20260911/time_teacher_20260911_pilot03
```

Outputs: `data_audit.json`, `audit_arrays.npz`, three decoded camera PNGs.
Windows small copies and audit source: `tmp/time_teacher_si26_20260911/`.
No package implementation was changed; no training or full pytest was run.
Next collection should validate the low-speed expert and explicit stop handling,
then cover bends/start-stop/recovery with separate runs. Do not infer data quantity
or model improvement from this single pilot.
