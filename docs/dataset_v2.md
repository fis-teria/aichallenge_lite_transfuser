# Dataset v2 recording and conversion

Dataset v2 removes command-integrated waypoint labels. Camera is assigned to a
10 Hz regular grid, LiDAR is matched to the selected Camera stamp, measured
pose/velocity/actual steering are interpolated, and future waypoints are built
from measured global pose in the observation ego frame.

## Safety and provenance contract

- Run recording on vehicle ROS domain `101` when the AWSIM base domain is `100`.
- Check unrelated GA/AWSIM processes before starting; do not stop another run.
- Actual steering is `/vehicle/status/steering_status`. A command value is never
  substituted for it. If actual steering is unavailable during conversion, the
  row stores `actual_steering_valid=0` and `actual_steering_rad=NaN`.
- Native LaserScan beam count and angular geometry are inferred from the bag and
  must stay identical within and across runs. No design-time value such as 750
  or 1080 is silently selected.
- Stop, collision, off-track, and recovery values require separate annotations.
  They are not inferred from low speed or control commands.
- The existing format-v1 dataset and converter remain unchanged.

## Record one run

Run in the Autoware/ROS environment that contains the custom Autoware message
packages and can see vehicle domain 101:

```bash
cd /path/to/aichallenge_lite_transfuser
source /opt/ros/humble/setup.bash
source /autoware/install/setup.bash
export ROS_DOMAIN_ID=101
PYTHONPATH=src python3 tools/record_dataset_v2.py \
  --output-root datasets/raw/aic_real_dataset_v2 \
  --run-id normal_course_seed001_run01 \
  --scenario-id normal_course_seed001 \
  --ros-domain-id 101
```

The recorder obtains one coherent `ros2 topic list -t` graph snapshot, retries
bounded DDS participant startup, and then checks all required topic names and
exact message types before it creates the bag. It records `/clock`, uses
simulation time for bag timestamps, writes MCAP with file-level zstd
compression, and saves a recording manifest.
Each snapshot explicitly uses `--no-daemon --spin-time 2.0`; at most three
snapshots are attempted before the same strict missing-topic check fails.
Stop with `Ctrl-C`, or pass `--duration-sec` for a bounded diagnostic run.

Required streams are Camera, native LaserScan, global Odometry, measured
VelocityReport, SteeringReport, GearReport, final control command, and nominal
control command. All are recorded even though actual steering is an explicitly
optional converter input, so a temporary missing status is visible rather than
replaced with the command.

## Convert five or more runs

Use the official vehicle config as the source of wheelbase and maximum steering;
its path and SHA-256 are saved in metadata.

```bash
cd /path/to/aichallenge_lite_transfuser
PYTHONPATH=src .venv/bin/python tools/convert_mcap_dataset_v2.py \
  --input-root datasets/raw/aic_real_dataset_v2 \
  --output datasets/processed/aic_real_dataset_v2 \
  --vehicle-config /path/to/racing_kart_description/config/vehicle_info.param.yaml \
  --val-ratio 0.2 \
  --test-ratio 0.2 \
  --split-seed 42
```

`--expected-lidar-points` is optional. Use it only after a live LaserScan probe
has established the native count. Without it, the converter infers the first
run's count and still rejects any within-run or cross-run geometry change.

The output directory must be absent or empty. Conversion never overwrites the
format-v1 dataset. Inspect `metadata.yaml` before training. Dataset Phase 2 is
complete only when it reports at least five runs and all of these gates pass:

- effective sample rate 9.8 to 10.2 Hz for every run;
- Camera-LiDAR p95 skew below 30 ms;
- pose/velocity missing fraction below 1 percent;
- measured-pose waypoint provenance;
- no run overlap across train/validation/test;
- individually valid and cross-run-consistent delay calibration.

The recorder and converter have unit-tested pure contracts. Live recording in
the official AWSIM environment remains required before claiming Dataset gate
completion.
