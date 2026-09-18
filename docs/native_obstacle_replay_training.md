# Native obstacle teachers with retained recovery replay

This is a separate TimePath experiment. Existing launch-balanced configurations,
checkpoints, and the runtime checkpoint remain unchanged. The initializer is
`runs/time_launch_protection_v2_20260916/launch_balanced/epoch_03.pt`, SHA256
`1fe12ca066791d3eb8907c6921120e2cfbb38925c422d5be54940f4acbdd120a`.

The frozen plan is `configs/time_path_p1/native_obstacle_replay_20260918.json`.
It adds 338 selected native MPPI V45 windows (86 nominal, 252 static-cone context)
to the exact existing 60,608 presentations per epoch. Every old normal, launch,
and recovery presentation remains; ten passes over the new 338 unique windows
add 3,380 presentations (5.28% of the new 63,988 total). Repetition is sampling,
not additional independent data. All new runs share `all_corners_20260918`, so
the whole group is train-only. Original validation assignments remain fixed and
the test split remains sealed.

The converter hash-checks the raw bags, recreates all 338 causal sensor histories,
and checks all 30 observed XY/velocity targets against the frozen curation.
Labels after the trustworthy pose prefix and held/excluded windows are not used.
Stop/mode labels remain unknown. These selected prefixes do not have strict
full-run avoidance certification. The current model optimizes 30 future XY points
at 0.1 s intervals; observed velocity is retained as metadata, without adding a
speed/stop head or changing the existing model/loss. The same recovery steering
geometry and far lateral loss remain active on all existing recovery samples.

Training uses three epochs, batch 32, workers 0, FP32, seed 42, LR 1e-5, 6,000
optimizer steps. This is a bounded continuation with replay, not a replacement
of the existing learning recipe. No model is deployed automatically.

## WSL commands

Edit and commit on Windows, synchronize with `tools/sync_to_wsl.ps1`, and run in
the native WSL checkout. In the current environment the checkout is
`/home/thistle/e2e_autonomous/e2e_lite_transfuser_lidar_v2x_margin`, with datasets
and runs under its parent directory. Do not train from `/mnt/e`.

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
tools/with_wsl_training_lock.sh timeout --signal=INT --kill-after=60s 10800s \
  env PYTHONPATH=src .venv/bin/python -u tools/train_time_native_replay.py run \
  --root /home/thistle/e2e_autonomous \
  --plan configs/time_path_p1/native_obstacle_replay_20260918.json
```

`prepare`, `train`, and `evaluate` can also be run separately. Preparation and
evaluation refuse to overwrite their output directories. After an interrupted
training, `train --resume` resumes the frozen optimizer state under the same
source commit/plan, followed by `evaluate`. Do not overwrite partial preparation;
retain it for diagnosis and use a separately named experiment if preparation
needs a source correction.

## Comparison and limits

Every epoch and the immutable initializer are compared on the existing fixed
development validation: 11,505 normal frames/4 runs and 8,962 recovery frames/
42 runs, plus 210 existing launch/age Pure Pursuit cases. This launch calibration
uses the previous fixed controller; it does not prove current 20 km/h control.

Normal/recovery run-macro ADE and 3 s error may regress by at most the greater of
5% or 1 mm/2 mm. Each recovery run also has a 20% or 5 mm/10 mm limit, so one
weak run cannot disappear in the average. All teacher-supported launch cases
must pass with the existing steering-margin gate. Native all/cone training-fit
ADE must improve at least 1%; among eligible candidates, native cone fit ranks
epochs. The original model is retained if no new epoch satisfies the gates.

Native fit is measured on training examples, and existing validation has already
been used for development. These are retention/fit checks, not held-out obstacle
generalization or closed-loop collision-avoidance results. A later AWSIM test is
required to assess the new candidate in motion; no new ROS/AWSIM runtime code is
changed by this experiment.
