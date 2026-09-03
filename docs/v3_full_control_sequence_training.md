# V3 future control-sequence training

The full-control model keeps trajectory and speed-profile outputs mandatory and
adds a ten-step physical control sequence at 0.1 s intervals. Each item is
ordered as steering (rad), speed (m/s), and acceleration (m/s2). The decoder
predicts steering-rate, bounded speed setpoint, and jerk internally. It
integrates steering and acceleration from measured ego state/the most recent
command issued strictly before the prediction anchor
and applies the configured absolute and rate limits at every step. Speed is
decoded as an Ackermann command setpoint; it is not incorrectly integrated as
if it were measured vehicle speed.

Dataset targets use the immediate teacher command followed by commands from
the same run and clock segment. Missing tail steps are explicitly masked. A
command from another run or clock epoch is never used as a future label.

`command_history` is a causal input with the explicit
`causal_previous_only` alignment. It is fixed-length and left-padded with an
invalid mask at run/clock-epoch starts. The command at the prediction anchor is
the step-zero target only; it must never appear in its own input history or be
used as the projection's initial acceleration. Runtime artifacts with a
`control_sequence` capability must carry the same alignment marker and are
rejected otherwise. This intentionally prevents checkpoints created under the
older leaking history contract from being reused for full control.

To initialize the new sequence head from an existing V3 single-control model
while preserving its learned trajectory/behavior representation:

```bash
PYTHONPATH=src python3 -m aic_transfuser_lite.cli train \
  --config configs/models/full_control_lite_v3.yaml \
  --dataset-root /home/thistle/e2e_autonomous/datasets/d1log_0902_pilot_v3 \
  --split-manifest /home/thistle/e2e_autonomous/datasets/d1log_0902_pilot_split_manifest.json \
  --view-config configs/data/view_temporal_v3.yaml \
  --behavior-view /home/thistle/e2e_autonomous/datasets/d1log_0902_pilot_behavior_v1 \
  --output /home/thistle/e2e_autonomous/runs/d1log_0902_full_sequence_v3 \
  --init-checkpoint /home/thistle/e2e_autonomous/runs/d1log_0902_pilot_full_control_v3/last.pt \
  --freeze-migrated --epochs 1 --device cuda
```

In the Windows/WSL workflow, wrap the command with
`tools/with_wsl_training_lock.sh`. The run manifest records the source
checkpoint SHA-256, loaded/missing key counts, and whether migrated parameters
were frozen. `--resume` and `--init-checkpoint` are mutually exclusive; a
frozen run must pass `--freeze-migrated` again when resumed.

The current `d1log_0902` Dataset V3 contains one recorded run. It is suitable
for a training/smoke artifact but cannot provide an independent run-level
validation split. Closed-loop or promotion claims therefore require separately
recorded AWSIM evidence. A completed training command alone is not a ROS 2 or
AWSIM result.
