# V4-20: remaining 10 m teacher expansion

Audit every 12,698 train candidate whose saved h30 provisional contiguous prefix
reaches 10 m in the frozen coverage ledger. The 10.34 GB d1log_0902.zip source is
already represented in the mixed canonical dataset; do not import it twice.
The prior 1,786 anchors covered only a bounded 2,048-candidate audit.

Regenerate each candidate's observed 46-point XY/mask using the existing long
geometry checks, current ego validity and moving-observation requirements.
No extrapolation, cross-anchor stitching, or new safety eligibility claims.
Union every accepted candidate with all prior 1,786 anchors by sample ID, retaining
near/recovery teachers and all observed points beyond 10 m. No distance thinning.
Adjacent observations remain correlated. Do not read test futures or sensor assets.

Continue the same model, optimizer, float32 preprocessing and masked band loss
from full-run epoch 12, SHA256
`07f2a04b9da0e2673036a1f0de6e13d841f1c4f84aa12e7a33ff61c070286c33`.
This is data expansion plus additional training, not a paired equal-compute study.
Fixed 128 train/64 validation diagnostic anchors remain unchanged; evaluate before
training and after each epoch, including 10 m error and supported self-intersections.
Retain validation as validation. This repeatedly used development set is not a new
independent evaluation. Report near/far regressions as well as 10 m improvements.

Budget: RTX 4080, 2 full shuffled epochs (each teacher exactly once per epoch),
batch 8/microbatch 2, maximum 3,600 active seconds with 600 seconds reserved for
final evaluation; audit/input preparation maximum 1,800 seconds. Native disk input
cache, at least 80 GiB free. No automatic runtime replacement or simulation launch.
Record rejected candidates, source hashes, tensor cache hashes, checkpoint identity,
visits and failure status. All prior artifacts are preserved.

```powershell
# Verify physical host capacity as well as native WSL free space. The recovered
# distro's expanding VHDX is on D:, independently of the E: source checkout.
if ((Get-Volume -DriveLetter D).SizeRemaining -lt 80GB) {
    throw 'WSL VHDX host D: requires at least 80 GiB free for this disk-cache run'
}
.\tools\sync_to_wsl.ps1 -CheckOnly
.\tools\sync_to_wsl.ps1
```

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
bash tools/with_wsl_training_lock.sh env OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 .venv/bin/python -m pytest -q
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 .venv/bin/python -u -m aic_transfuser_lite.training.spatial_long_expand10_v4 --parent /home/thistle/e2e_autonomous/runs/spatial_long_v4_20260910_run01 --checkpoint /home/thistle/e2e_autonomous/runs/spatial_long_v4_full_20260910_run01/epoch_12.pt --output /home/thistle/e2e_autonomous/runs/spatial_long_v4_expand10_20260910_run02
```

Use a fresh output directory and preserve stdout/stderr in sibling logs.
Execution and result details are added after the finite run completes.

Initial run01 (`9592b9e`) audited all 12,698 candidates, accepting 12,661 and
rejecting 37, but stopped before training at the identity guard because live epoch
keys were tuples and previously serialized keys were lists. No optimizer updates
occurred. Run01 artifacts/logs are preserved. `037cc6e` normalizes sequence types
while retaining exact run/segment/epoch values, with a regression test. The bounded
retry uses run02 and the same data/model/budget; no guard is bypassed.

## Actual result: teacher audit complete, training blocked by host storage

Execution source: `037cc6edba2c85eba02d610114f63a7e9878e17c`.
WSL full pytest: **1,842 passed / 4 skipped / 52 warnings, 67.39 s**.

Run02 repeated the audit with identical counts and passed source/teacher identity
checks. Accepted >=10 m teachers: **12,661**, rejected **37**, overlapping reason
counts invalid current ego 20, direction reversal 17, observed self-intersection 6.
After removing the 376 already-used >=10 m anchors, **12,285 new anchors** join
the prior 1,786, giving **14,071 train** (normal 13,051, recovery 1,020; 16 runs).
10 m support increases 376 -> 12,661; 20 m support 165 -> 5,854 (7 runs).
Fixed validation remains 64 anchors / 5 runs. No test future/sensor assets read.

Run02 input-cache progress reached **3,800 / 14,135** in the last readable log.
Then the process exited with Bus error; even `/usr/bin/free` returned Input/output
error and `df` segfaulted. `/proc/mounts` reported `/dev/sdd` ext4 `emergency_ro`.
Windows registry confirmed `Ubuntu-22.04-Recovered` BasePath `D:\WSL-Recovery`;
host D: had only **27,787,264 bytes free**, whereas the prior native WSL `df`
reported 807 GiB available. Native capacity alone was an insufficient preflight.
The additional cache writes are consistent with exhausting the backing volume.
This does not establish physical drive failure or confirm filesystem integrity.

**No training optimizer steps, new checkpoint, or additional model evaluation
completed.** The last execution record may still say STARTED because the OS I/O
failure prevented exception/finally reporting. Do not infer a live training job
from that stale state. Source/runtime checkpoint was not replaced.

The readable stdout and host volume snapshot were copied to Windows
`tmp/v4_expand10_storage_20260910/`. Attempts to copy execution/summary JSON then
also hit I/O errors; the summary counts above had already been read before those
errors. Do not claim full artifact backup or successful post-run hash verification.
The affected distro was stopped with `wsl --terminate Ubuntu-22.04-Recovered`.
No original datasets/checkpoints or unrelated host files were deleted.

Before resuming: free physical D: capacity and inspect WSL filesystem health;
verify source, teacher and checkpoint hashes again. Preserve failure evidence.
The per-anchor float32 disk cache can exceed 55 GiB for the full cohort; consider
bounded in-memory/on-demand preprocessing to avoid requiring that much persistent
space. That alternative is not yet implemented or tested. Do not automatically
restart the failed disk-cache command on the full backing volume.
