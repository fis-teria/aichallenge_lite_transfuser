# V3 actuator calibration workflow

V3-019 creates a versioned `aic_actuator_calibration_v1` candidate. It does not
grant ROS control authority. The artifact keeps steering, drive, and brake
identification separate and records the source run hashes, vehicle-profile
hash, fitted applicability ranges, quality metrics, rejection counts, and
promotion state.

## Models and units

- Steering: existing command-to-actual steering delay estimator, with pure
  delay (s) and first-order lag (s) kept separate. Steering is rad, yaw rate is
  rad/s, speed is m/s, and wheelbase is m.
- Drive and brake: fitted independently. Each stores pure delay (s), first-order
  lag (s), gain, acceleration bias (m/s^2), command range (m/s^2), and observed
  speed range (m/s).
- Actual acceleration: derivative of measured longitudinal speed on its source
  time grid, followed by an explicit odd-width moving mean. Independent run
  clocks, differentiation, pure-delay hold, and first-order state are reset at
  each `run_id`; a repeated non-contiguous run ID is rejected.

The lateral wrapper excludes non-finite values, samples below 0.1 m/s, and yaw
rates beyond the declared vehicle-profile applicability limit before calling
the existing estimator. Longitudinal fitting counts implausible derived
acceleration samples as excluded rather than clipping them into the fit. Brake
fitting also excludes samples below 0.1 m/s: a negative command held after the
vehicle has stopped contains no brake-response information and otherwise
biases stronger commands toward zero acceleration.

Initial candidate-quality gates are explicit and provisional: steering NRMSE
must be below 0.7, yaw-rate NRMSE below 0.8, and each longitudinal mode must
have correlation above 0.5 and NRMSE below 0.8. Passing these gates would still
not justify promotion; their purpose is to prevent visibly weak fits from being
reported as individually valid while broader multi-run thresholds are pending.

## Candidate generation

For new calibration data, use the hash-armed, fail-closed one-run collector in
[`docs/v3_calibration_capture.md`](v3_calibration_capture.md). Keep steering,
drive, and brake repetitions as independent run IDs; do not inject excitation
during normal racing or merge repeated trials into one synthetic run.

Run this in the WSL native environment, not under `/mnt/e`:

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
tools/with_wsl_training_lock.sh .venv/bin/python \
  tools/fit_calibration_v3.py \
  --dataset-root /home/thistle/e2e_autonomous/datasets/d1log_0902_pilot_v3 \
  --vehicle-profile configs/calibration/v3_baseline_vehicle_unverified.yaml \
  --output /home/thistle/e2e_autonomous/calibration/d1log_0902_candidate.json
```

Run unit and negative tests with:

```bash
python3 -m pytest -q tests/test_calibration_v3.py tests/test_delay_estimation.py
```

`v3_baseline_vehicle_unverified.yaml` deliberately limits promotion to
`candidate`: wheelbase 1.087 m is inherited from the existing runtime default
and still lacks primary-source confirmation. The fitter never changes this
state automatically.

For the racing-kart AWSIM environment, prefer
`configs/calibration/v3_racingkart_vehicle_verified.yaml`. Its 1.087 m
wheelbase and 0.64 rad maximum steering are bound to the tracked official
`racing_kart_description/config/vehicle_info.param.yaml` file and its SHA-256.
The profile still limits generated artifacts to `candidate`; fit quality,
cross-run stability, shadow evaluation, and Safety gates remain separate.

## Promotion boundary

The current `d1log_0902` dataset contains one run. A single-run artifact may be
used to inspect fit quality and later shadow behavior, but it is not a
last-known-good calibration and must not be marked `shadow` or `promoted`.
Before promotion, collect independent calibration runs spanning drive, brake,
speed, and steering excitation; verify cross-run parameter stability; confirm
the vehicle profile; and evaluate Safety Supervisor intervention in AWSIM.

RViz2 visualization is useful for the later AWSIM shadow and low-speed stages,
but visual inspection is not calibration evidence by itself.

## Recorded `d1log_0902` candidate result

The candidate generated from commit `c536014` is intentionally rejected by
the provisional gates. Its internal canonical-payload SHA-256 is
`11418c4d047cbf0dc1c905d4056bf82533fdf6ab2613dd3553097c1ef76be3ba`;
the serialized JSON file SHA-256 is
`d147910a830bc2fd98df4aa376ba0e8bf29a96d759e9a58e0206946f230f9b3b`.
It contains one source run and remains in the `candidate` promotion state.

- Steering: invalid; delay 0.00 s, lag 0.50 s, correlation 0.9653,
  steering NRMSE 0.7449, yaw-rate NRMSE 0.8537, 138 excluded samples.
- Drive: invalid; delay 0.00 s, lag 0.18 s, gain 0.25877, bias
  -0.12746 m/s^2, correlation 0.30649, NRMSE 0.95188.
- Brake: invalid; delay 0.01 s, lag 0.06 s, gain 0.14146, bias
  -1.03338 m/s^2, correlation 0.59229, NRMSE 0.80572.

The focused calibration/delay tests passed in WSL (`10 passed`) and in the
official `aichallenge-2025-dev:latest` container on Graneple (`10 passed`). The
full WSL suite passed (`332 passed`). Because all three fits are not valid, no
AWSIM calibration shadow, low-speed control, Safety Supervisor integration, or
RViz2 calibration review was executed for this artifact. These skipped stages
must not be reported as successful.

## Recorded live `v3_0efecab` campaign

Nine hash-bound AWSIM runs (three steering, three drive, and three brake) were
executed on Graneple from source commit `0efecab`. Their capture manifests and
MCAP metadata passed, the transferred raw tree matched SHA-256
`c94df6b66ae1f4a9f96cece54e3f6ea65dbdc144aa2d3956af570d890725a6ad`,
all nine Dataset V3 scans/validations passed, and the processed dataset audit
passed. Three runs recorded one transient `camera_timeout` at Safety startup;
the excitation did not start until the required stable `normal` preflight.

The first multi-run fit exposed and then fixed the independent-clock reset
bug. The corrected fit completed but remained a rejected candidate: drive
passed, steering failed `dynamic_sample_count<500` and yaw-rate NRMSE, and
brake failed with non-positive gain because stationary brake holds dominated
the original plan. It was not promoted or used for control. The V2 excitation
plans keep steering excitation moving and start brake steps from a higher
speed; the moving-speed fit gates prevent stationary data from hiding an
insufficient capture.
