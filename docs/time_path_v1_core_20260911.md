# Proposed time-path baseline: first implementation stage

This is a new offline model core, not a trained model or an AWSIM deployment.
Training and large dataset transfers remain paused pending SSD migration.

## Implemented

- `models/time_path_v1.py`: existing V3 Camera/LiDAR/ego fusion with a new GRU
  incremental XY decoder. Output [B,30,2] metres in base_link at observation time,
  t = 0.1, 0.2, ..., 3.0 seconds. Targets are stripped before feature extraction.
- Explicit time output contract and rejection of distance-grid/wrong-frame metadata.
- Derived interval chord-speed norm [B,30] m/s. This is not signed longitudinal
  speed, a learned speed head, a stopping decision or a stop probability.
- Masked L1 XY baseline loss, equal weight per supported anchor; missing targets
  sanitized before arithmetic, empty support returns None.
- Explicit command-history ablation option, leaving the original batch unchanged.
- Tests for units, metadata rejection, missing-target gradients, decoder shape,
  teacher isolation, command ablation, feature gradients and weight roundtrip.

Original TransFuser uses a goal-conditioned autoregressive waypoint decoder.
This proposed decoder has no goal input because the current input contract has none.
It is inspired by the decoder design, not a reproduction or a guarantee of physical
feasibility. Existing V4/V3 code and runtime identities are unchanged.
Reference: https://github.com/autonomousvision/transfuser/blob/cvpr2021/transfuser/model.py

## Validation commands

Windows source commit followed by:

```powershell
.\tools\sync_to_wsl.ps1 -CheckOnly
.\tools\sync_to_wsl.ps1
```

WSL native checkout:

```bash
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q tests/test_time_path_v1.py
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
```

## Remaining migration work

This completes only the isolated model/loss core stage. Before actual training:
implement the time-based dataset view and intervention masks, fixed-time history
parity, run/scenario splits and metrics, bounded trainer/checkpoint input metadata,
and explicit representation migration with provenance. The metadata helper alone
is not yet a checkpoint loader. Existing D: host-volume guard must follow the new
SSD location after migration.

Before driving: time-aware runtime/controller adapter, sensor/update timing checks,
and closed-loop validation. Optional speed/stop heads and consistency regularizers
are separate comparisons after the waypoint baseline; no learned stop capability
is claimed. No datasets or checkpoints are created by this change.
