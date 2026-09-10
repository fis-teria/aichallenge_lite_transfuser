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
.\tools\sync_to_wsl.ps1 -CheckOnly
.\tools\sync_to_wsl.ps1
```

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
bash tools/with_wsl_training_lock.sh env OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 .venv/bin/python -m pytest -q
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 .venv/bin/python -u -m aic_transfuser_lite.training.spatial_long_expand10_v4 --parent /home/thistle/e2e_autonomous/runs/spatial_long_v4_20260910_run01 --checkpoint /home/thistle/e2e_autonomous/runs/spatial_long_v4_full_20260910_run01/epoch_12.pt --output /home/thistle/e2e_autonomous/runs/spatial_long_v4_expand10_20260910_run01
```

Use a fresh output directory and preserve stdout/stderr in sibling logs.
Execution and result details are added after the finite run completes.
