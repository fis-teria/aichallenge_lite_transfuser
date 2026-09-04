# V3 path-only launch recovery

## Why this change exists

The mixed lap/recovery fine-tune improved held-out trajectory and speed metrics,
but three closed-loop M3 trials never established a launch. The executable
reference requested at most about 0.113 m/s, below the controller's configured
0.2 m/s launch threshold. This change separates path geometry authority from
speed authority and prevents a similar checkpoint from being promoted solely
because its average ADE is low.

Dataset V3 currently stores measured future pose and measured future velocity
as trajectory/speed targets. It does not store the planned recovery Reference
as a canonical target. Consequently this change does not synthesize or silently
substitute a planned Reference label. Adding that target requires a versioned
converter and collection-contract update first.

## Measured target contradiction

An initial strict audit of the live mixed dataset at
`/home/thistle/e2e_autonomous/datasets/d1log_recovery_mixed_20260904_v3`
used the first 15 valid future steps (1.5 s). Among 2,208 anchors where current
absolute longitudinal speed was at most 0.05 m/s and both available command
streams requested at least 0.5 m/s:

- 94.66% had maximum future speed below 0.2 m/s;
- 94.84% had maximum future displacement below 0.1 m;
- 94.66% met both stationary conditions;
- the raw/retained-motion counts were 1326/72 train, 451/24 validation, and
  431/22 test.

The implemented loader follows the actual nominal-first/final-fallback teacher
selection contract instead of requiring both streams. A post-change live
loader check produced:

| split | usable after filter | contradictory anchors rejected | retained launch candidates |
|---|---:|---:|---:|
| train | 43,680 | 1,462 | 188 |
| validation | 13,153 | 476 | 54 |
| test | 13,382 | 462 | 58 |

The 54 held-out validation candidates exceed the launch gate's minimum sample
count of 20.

These frames teach a path/speed model to remain near the origin even though the
teacher command requests motion. They are excluded only when all of the
following are true:

1. current absolute speed is at most 0.05 m/s;
2. selected nominal/fallback command speed is at least 0.5 m/s;
3. maximum measured future speed is below 0.2 m/s; and
4. maximum measured future displacement is below 0.1 m over 15 steps.

Genuine stop commands and already-moving frames remain eligible. Rejected
counts are written to dry-run output and `run_manifest.json`.

## Runtime authority

`runtime.v3.trajectory_authoritative.param.yaml` now uses
`executable_reference_speed_source: path_only_constant`. The model owns only
the predicted path geometry. The executable-reference layer starts from a
0.75 m/s external target and still applies curvature, Limited-ODD, Safety, and
stop-probability constraints before the controller tracks the reference.

The model speed head remains in the artifact for compatibility and as a
diagnostic/auxiliary training signal, but it is ignored by this runtime profile.
Other profiles retain the default `model` speed source.

## Offline promotion gate

The trajectory-authoritative fine-tune config enables a held-out launch gate.
For validation inputs stopped at no more than 0.05 m/s with commanded speed at least
0.5 m/s, at least 20 samples must be available and at least 80% must predict
0.1 m or more forward progress. Epochs that fail this gate are not copied to
`best_trajectory.pt`. If no epoch passes, training preserves `last.pt` for
diagnosis, writes no `runtime_artifact.json`, and exits with gate code 5.

This is an offline readiness gate, not M3 success. M3 still requires fresh
closed-loop launch, tracking, curve coverage, Safety, and collision evidence.

## WSL verification and training

Run a non-mutating dataset/config check first:

```bash
tools/with_wsl_training_lock.sh \
  env PYTHONPATH=src python3 -m aic_transfuser_lite.cli train \
  --config configs/models/trajectory_authoritative_finetune_v3.yaml \
  --dataset-root /home/thistle/e2e_autonomous/datasets/d1log_recovery_mixed_20260904_v3 \
  --split-manifest /home/thistle/e2e_autonomous/datasets/d1log_recovery_mixed_20260904_split_manifest.json \
  --view-config configs/data/view_temporal_v3.yaml \
  --behavior-view /home/thistle/e2e_autonomous/datasets/d1log_recovery_mixed_20260904_behavior_v1 \
  --output /home/thistle/e2e_autonomous/runs/path_only_launch_recovery_v3 \
  --device cuda --dry-run
```

Because the filter and gate change the experiment contract, do not resume the
old mixed run. Initialize a new output directory from its retained best
checkpoint:

```bash
tools/with_wsl_training_lock.sh \
  env PYTHONPATH=src python3 -m aic_transfuser_lite.cli train \
  --config configs/models/trajectory_authoritative_finetune_v3.yaml \
  --dataset-root /home/thistle/e2e_autonomous/datasets/d1log_recovery_mixed_20260904_v3 \
  --split-manifest /home/thistle/e2e_autonomous/datasets/d1log_recovery_mixed_20260904_split_manifest.json \
  --view-config configs/data/view_temporal_v3.yaml \
  --behavior-view /home/thistle/e2e_autonomous/datasets/d1log_recovery_mixed_20260904_behavior_v1 \
  --output /home/thistle/e2e_autonomous/runs/path_only_launch_recovery_v3 \
  --epochs 5 --batch-size 2 --device cuda --checkpoint-every-steps 100 \
  --init-checkpoint /home/thistle/e2e_autonomous/runs/d1log_recovery_mixed_20260904_waypoint_e02b804/best_trajectory.pt
```

Confirm the initialization checkpoint exists and record its SHA-256 before
starting. A passing offline artifact must still be deployed and evaluated under
the M3 procedure in `docs/v3_m3_limited_odd.md`.
