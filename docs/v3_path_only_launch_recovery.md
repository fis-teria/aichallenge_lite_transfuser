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
selection contract instead of requiring both streams. It also requires all 15
future points before declaring a contradictory stationary future. A
post-change live loader check produced:

| split | raw rows | base exclusions | unfiltered valid | complete contradictory rejected | censored | teacher quality | stopped-commanded retained |
|---|---:|---:|---:|---:|---:|---:|---:|
| train | 45,190 | 48 | 45,142 | 1,363 | 162 | 43,779 | 287 |
| validation | 13,641 | 12 | 13,629 | 450 | 37 | 13,179 | 80 |
| test | 13,866 | 22 | 13,844 | 444 | 27 | 13,400 | 76 |

The previously reported 1,462/476/462 exclusions counted 99/26/18 censored
stationary prefixes as if the full 1.5 s future had been observed. Those rows
are now retained as censored/unknown. The 82 rows outside the user-provided
usable-plus-excluded total are 60 invalid-current-ego rows and 22 rows with no
valid future point, not split leakage. The runtime-compatible launch replay
keeps all 530 validation stopped-commanded anchors in its denominator: 450
complete contradictory outcomes, 43 observed-motion outcomes, and 37 censored
outcomes.

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
For validation inputs stopped at no more than 0.05 m/s with commanded speed at
least 0.5 m/s, at least 20 samples, two runs, and three estimated launch
episodes must be available. At least 80% must pass the same executable-reference
and controller dry-run used by the trajectory-authoritative runtime, including
valid finite XY, initial-point handling, endpoint forward progress, curvature
caps, and controller request speed of at least 0.2 m/s. A maximum-X test alone
cannot pass the gate. Epochs must also meet the initial-checkpoint trajectory
ADE non-regression gate before research-candidate promotion. If no epoch passes
both gates, training preserves epoch snapshots and `last.pt` for diagnosis,
writes no `runtime_artifact.json`, and returns the `GATE_FAILED` status.

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

## 2026-09-05 experiment A result

The audited five-epoch run is documented in
`docs/v3_experiment_a_audit_20260905.md`. No epoch passed trajectory
non-regression and runtime-compatible launch readiness simultaneously, so no
research or runtime candidate was promoted. Epoch 3 is retained only as a
diagnostic checkpoint: it reached 506/530 offline launch-ready anchors but
regressed waypoint-weighted ADE from 0.133046 m to 0.136326 m and had a 96.8%
initial-noise trim rate. No AWSIM closed-loop run was performed.
