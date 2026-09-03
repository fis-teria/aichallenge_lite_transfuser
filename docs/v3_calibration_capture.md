# V3 automated actuator-calibration capture

The V3 capture path records one independently identified AWSIM run while a
hash-armed ROS node publishes a bounded excitation plan through the independent
Safety Supervisor. It does not promote calibration, grant model authority, send
an AWSIM Start request, or reset the scenario.

## Safety and authority contract

Dry-run is the default. `--execute` is required before any command publisher is
started. The collector refuses an existing publisher on either
`/nominal_control_cmd` or `/control/command/control_cmd`; after launch, the
excitation node requires exactly one nominal publisher (itself), exactly one
final publisher (Safety Supervisor), a subscriber on both paths, fresh velocity
telemetry, an allowed Safety reason, and speed inside the plan limit.

The checked-in plans are additionally bounded by hard collection guards:

- steering: at most `0.25 rad` absolute
- target and observed speed: at most `3.0 m/s`
- acceleration: `[-2.0, 1.0] m/s^2`
- duration: at most `180 s`
- first segment: stationary settle
- final segment: zero steering/speed with braking for the declared stop hold

These values are conservative collection guards only. They are not authoritative
vehicle limits and must not be copied into a promoted full-control artifact.
Every runtime rejection enters an abort state and holds the final stop command
before exiting nonzero.

## Environment prerequisites

Run collection on `graneple@192.168.3.10` in the official AWSIM/ROS 2
environment, not on Windows or `/mnt/e`. Before `--execute`:

1. build and source `aic_e2e_runtime` from the synchronized commit;
2. start AWSIM and the sensor/localization path, but leave every existing
   nominal/final controller publisher stopped;
3. arm the AWSIM vehicle/control mode through the environment's normal official
   procedure;
4. place the stopped vehicle in a bounded open calibration area;
5. verify that Camera, LiDAR, pose, velocity, steering, gear, and `/clock` are
   present on the selected ROS domain.

The collector performs topic/type and publisher-count checks again. It does not
work around a failed preflight.

## Dry-run

Dry-run validates the plan, prints its SHA-256 arm token and exact rosbag/launch
commands, and creates no directory or ROS process:

```bash
cd /home/graneple/e2e_lite_transfuser
PYTHONPATH=src python3 tools/collect_calibration_v3.py \
  --plan configs/calibration/excitation_steering_low_speed_v1.yaml \
  --topic-profile configs/data/topic_profile_v3.yaml \
  --output-root /home/graneple/calibration_bags/v3 \
  --run-id steering_r01 \
  --scenario-id awsim_calibration_pad
```

Use the actual checkout path on Graneple if it differs. Dry-run output paths are
resolved absolutely, so review them before execution.

## Execute one run

Use a new run ID for every attempt. Existing bags, manifests, and result files
are never overwritten.

```bash
PYTHONPATH=src python3 tools/collect_calibration_v3.py \
  --plan configs/calibration/excitation_steering_low_speed_v1.yaml \
  --topic-profile configs/data/topic_profile_v3.yaml \
  --output-root /home/graneple/calibration_bags/v3 \
  --run-id steering_r01 \
  --scenario-id awsim_calibration_pad \
  --execute
```

Repeat with unique IDs for:

- `configs/calibration/excitation_steering_low_speed_v1.yaml`
- `configs/calibration/excitation_drive_low_speed_v1.yaml`
- `configs/calibration/excitation_brake_low_speed_v1.yaml`

Collect at least three independent runs per target mode. Stop and reset/reposition
the scenario between runs rather than concatenating repetitions into one bag.
Inspect `<run-id>.calibration_capture.json`; only `status: complete`, an MCAP
`metadata.yaml`, a matching plan hash, and `excitation_result.status: complete`
constitute a completed capture. A completed capture is still only input data,
not a valid calibration.

## Convert and fit

Validate and convert the collected MCAP directories through the existing V3 bag
and Dataset commands. Build one Dataset V3 root containing the independent run
IDs, then fit a candidate in the WSL native environment:

```bash
tools/with_wsl_training_lock.sh .venv/bin/python \
  tools/fit_calibration_v3.py \
  --dataset-root /home/thistle/e2e_autonomous/datasets/calibration_v3 \
  --vehicle-profile configs/calibration/v3_baseline_vehicle_unverified.yaml \
  --output /home/thistle/e2e_autonomous/calibration/calibration_v3_candidate.json
```

Promotion still requires individually valid steering, drive, and brake fits,
cross-run stability, an authoritative vehicle profile, held-out validation, and
later Safety/AWSIM gates.

## Verification boundary

The ROS-independent plan validation, command construction, parser, authority,
timeout, non-finite, and bounds tests can run under pytest. An actual AWSIM
excitation must not be reported successful unless `--execute` was run on
Graneple and the resulting bag/result manifest passed the gates above.
