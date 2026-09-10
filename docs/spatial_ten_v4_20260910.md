# V4-10: 36-point model with on-demand training inputs

User authorized training V4-10 after removing D:/UbuntuExport/Ubuntu_2.7z.
WSL restarted read/write without emergency_ro; the V4-20 epoch12 checkpoint
SHA256 was verified. This is not a full offline filesystem health certification.

The output is 36 XY points in base_link@t_obs metres: 0.1..2.0 by 0.1 m,
2.5..10.0 by 0.5 m. Copy the V4-20 epoch12 backbone, hidden head and first 72
final-head coordinates. Strict state validation and initial replay against the
source's first 36 predictions are required. AdamW is NEW, with backbone LR 1e-4,
head LR 1e-3, zero weight decay. This is warm-start fine-tuning, not scratch or
restored optimizer continuation. Mean SmoothL1(beta=0.1 m) gives equal weight to
the observed near/middle bands per anchor; beyond-10 m points are not outputs.

Re-audit all 12,698 >=10 m candidates and union accepted anchors with the prior
1,786 train teachers, retaining recovery/short teachers. Existing long-horizon
geometry audit is conservatively retained; eligibility is not newly relaxed just
because the model ends at 10 m. Expected train count 14,071 (10 m support 12,661).
All 64 fixed validation anchors remain unchanged. Test future/sensor assets remain
unread. Adjacent anchors are correlated. No runtime replacement or AWSIM launch.

No persistent input tensor cache is generated. A 16-anchor CPU LRU builds exact
float32 input-only tensors from hash-verified canonical assets as needed. Native
WSL data only; /mnt/d is used solely to measure physical backing-volume free space.
Require 3 GiB physical/native reserve at startup and check D: during execution.
Keep the failed prior disk caches and logs unchanged in this task.

Budget: 1 RTX4080, 2 complete epochs, batch8/microbatch2, at most3,518 optimizer
updates /28,142 presentations if the expected cohort is confirmed. Preparation
limit1,800 s; active training/evaluation limit3,600 s with600 s final reserve.
Evaluate epoch0/1/2 on the same fixed128 train/64 validation; final expanded-train
evaluation is diagnostic. Save both epochs, source/data hashes and exact visit
counts. Compare only the common <=10 m range with the previous V4-20 model.

```powershell
if ((Get-Volume -DriveLetter D).SizeRemaining -lt 3GB) { throw 'D: backing volume below reserve' }
.\tools\sync_to_wsl.ps1 -CheckOnly
.\tools\sync_to_wsl.ps1
```

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
bash tools/with_wsl_training_lock.sh env OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 .venv/bin/python -m pytest -q
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 .venv/bin/python -u -m aic_transfuser_lite.training.spatial_ten_v4 --parent /home/thistle/e2e_autonomous/runs/spatial_long_v4_20260910_run01 --checkpoint /home/thistle/e2e_autonomous/runs/spatial_long_v4_full_20260910_run01/epoch_12.pt --output /home/thistle/e2e_autonomous/runs/spatial_ten_v4_20260910_run01
```

Preserve stdout/stderr in a new sibling logs directory. Do not reuse failed output.
