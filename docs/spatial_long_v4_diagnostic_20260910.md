# 20 m observed-teacher diagnostic

Separate from the fixed 2 m model and runtime. Source of truth is Windows;
teacher generation, CUDA learning and evaluation run under the shared WSL lock.
No automatic push, deployment, raw acquisition, or final test access.

## Predeclared scope

Hash-verified canonical dataset and split identities reuse the existing V4 guard.
Run/source-hash split overlap is rejected. Audit up to 2,048 train and 1,024
validation future arrays, deterministic seed-42 run round-robin, never test assets.
This is a bounded diagnostic inventory, not a full-dataset audit.
Preserve every audited row with future hash, run, segment, timestamp, derived
history epoch, unknown physical teacher eligibility and rejection reasons.
Generate every eligible audited 46-point XY/mask teacher into a new native output.

The existing contiguous h30 prefix checks remain; no stitching, extrapolation,
origin scoring or unsupported tail scoring. Record missing/hold/reverse/jump.
Recompute corner cut over the whole covered long grid and a bidirectional
polyline-distance upper bound (sample gap <=0.05 m plus half gap). Diagnostic
limits are 0.15 m deviation, and max(0.1 m, 2% covered arc) shortening.
These are predeclared provisional thresholds, not vehicle safety tolerances.
The old 2 m corner-cut flag is recorded but does not decide long eligibility.
Stopped/reverse current ego, faulty prefixes and flagged geometry are excluded.
Short moving recovery prefixes remain masked; no far teacher is invented.

Select at most 128 train / 64 validation anchors by support-band/run round-robin,
with >=0.5 s separation within a run. Requires at least eight each and observed
20 m support in both. Selection is frozen before learning. Adjacent/overlapping
futures are not independent scenes; validation is run-separated, not final test.

Inputs reuse Camera/LiDAR/ego/strictly previous command histories (4/4/10/10),
with epoch boundaries at run/segment changes, gaps >0.2 s or nonincreasing time.
Future/teacher/mask never enter forward. Scratch initialization, no weight loading.
Float32, seed42, AdamW backbone LR 1e-4/head 1e-3, weight decay0, clipping1,
microbatch2 with four accumulations. SmoothL1 beta0.1 m: mean observed points
per band, mean present bands (0–2, >2–10, >10–20 m), then mean supported anchors.
All-masked anchors and unsupported bands are excluded, not counted as zero loss.

Independent eight-anchor overfit: 100 steps, first step also gradient smoke;
its weights/optimizer are discarded. Main diagnostic: at most 500 steps, final
checkpoint only, no validation-based tuning/selection. Total training/evaluation
active budget 900 s, reserve90 s for final evaluation; preprocessing/I/O excluded.
Single RTX4080, PyTorch peak allocation recorded; other processes are not stopped.
Failure/timeout/OOM is recorded and never silently retried.

## Reproduce

```powershell
# Stage only owned long-horizon files, commit on Windows, then:
.\tools\sync_to_wsl.ps1 -CheckOnly
.\tools\sync_to_wsl.ps1
```

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
# Choose a NEW output; redirect stdout/stderr to sibling log files.
tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 .venv/bin/python -u -m aic_transfuser_lite.training.spatial_long_v4 --config configs/spatial_long_v4.yaml --output /home/thistle/e2e_autonomous/runs/spatial_long_v4_20260910_run01
```

Artifacts include teacher audit/NPZ, frozen selection/input histories and hashes,
initial/final predictions and metrics with distance-band anchor/point/run counts,
straight/train-mean baselines, independent overfit log, main optimizer log,
initial/final checkpoint, code/config/data hashes and execution commit.
Read assets are rehashed after learning. Physical route correctness, obstacle
clearance, closed-loop completion, stopping and runtime fitness remain UNKNOWN.
