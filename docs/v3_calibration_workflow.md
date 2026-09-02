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
  time grid, followed by an explicit odd-width moving mean.

The lateral wrapper excludes non-finite values, reverse-speed samples, and yaw
rates beyond the declared vehicle-profile applicability limit before calling
the existing estimator. Longitudinal fitting counts implausible derived
acceleration samples as excluded rather than clipping them into the fit.

Initial candidate-quality gates are explicit and provisional: steering NRMSE
must be below 0.7, yaw-rate NRMSE below 0.8, and each longitudinal mode must
have correlation above 0.5 and NRMSE below 0.8. Passing these gates would still
not justify promotion; their purpose is to prevent visibly weak fits from being
reported as individually valid while broader multi-run thresholds are pending.

## Candidate generation

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

## Promotion boundary

The current `d1log_0902` dataset contains one run. A single-run artifact may be
used to inspect fit quality and later shadow behavior, but it is not a
last-known-good calibration and must not be marked `shadow` or `promoted`.
Before promotion, collect independent calibration runs spanning drive, brake,
speed, and steering excitation; verify cross-run parameter stability; confirm
the vehicle profile; and evaluate Safety Supervisor intervention in AWSIM.

RViz2 visualization is useful for the later AWSIM shadow and low-speed stages,
but visual inspection is not calibration evidence by itself.
