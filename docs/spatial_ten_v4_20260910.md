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

## Measured training and fixed development validation

Execution commit: `db6f1dd34fb2fe086f1ab1e2aca3fb33f7df581f`.
WSL full pytest: **1,847 passed / 4 skipped / 53 warnings, 82.82 s**.
Teacher re-audit accepted12,661/rejected37, unchanged after the storage incident.
Union train14,071, normal13,051/recovery1,020; fixed validation64/5 runs.
Epoch1 and2 each visited every train anchor exactly once: **28,142 presentations,
3,518 updates**, with the final7-anchor batch included in each epoch.

Errors below are XY Euclidean metres; band results average observed points per
anchor and then anchors. Origin/unsupported points are not scored. Epoch0 is the
V4-20 epoch12 output restricted to the same <=10 m range, before any new updates.

| Validation metric | Anchors / runs | Epoch0 | Epoch1 | Epoch2 |
|---|---:|---:|---:|---:|
| Near <=2 m | 64 / 5 | 0.049873 | 0.034438 | 0.023732 |
| Middle >2..10 m | 33 / 2 | 0.266633 | 0.178074 | 0.168159 |
| At2 m | 36 / 2 | 0.121316 | 0.051027 | 0.043484 |
| At5 m | 31 / 2 | 0.208906 | 0.103620 | 0.135196 |
| At10 m | 24 / 2 | 0.543970 | 0.436443 | 0.334413 |
| Self-intersecting predictions on observed support | 64 / 5 | 0 | 0 | 0 |

Epoch2 at10 m improves38.52% versus epoch0; middle36.93%, near52.41%.
At5 m improves versus epoch0 but regresses versus epoch1; improvement is not
monotonic at every distance. Both checkpoints are retained, with no runtime promotion.
All three recovery validation runs improve on their *observed short support*:
left-far0.022595->0.015782, left-near0.019085->0.013822,
right-far0.019448->0.013825 m (6 anchors per run).

These are repeatedly used development validation results, not final test or AWSIM
closed-loop evidence. Dataset expansion, shortened output horizon, a fresh
optimizer and additional updates changed together; do not attribute the gain to
data volume alone. A36-point model still has no predicted validity/length head or
explicit curvature/continuity loss; teacher masks are training-only. Short/recovery
anchors do not establish that their unsupported tail is a valid10 m driving path.
Physical teacher safety and ideal route intent remain UNKNOWN.

## Completion and artifacts

Run status **COMPLETE**, process exit0. Preparation184.355 s, active
training/evaluation2,125.649 s, total2,310.580 s (38.51 min), within both budgets.
Initial source-prefix replay max absolute difference **0 m**; final checkpoint
reload replay difference **0 m**. Post-run canonical asset, source checkpoint,
manifest and split hashes **PASS**. Persistent input cache **0 bytes**; GPU peak
allocated550,455,296 bytes (PyTorch allocator metric, not total device usage).
Host D: free after completion13,144,973,312 bytes (~12.24 GiB).

Native WSL run directory:
`/home/thistle/e2e_autonomous/runs/spatial_ten_v4_20260910_run01`.

| Checkpoint | SHA256 |
|---|---|
| `epoch_01.pt` | `b2c73ee6c2cda97ae114fabd05a2612eccee76803e9e755d6880f959ef177234` |
| `epoch_02.pt` | `32752af8d3ebd023382ec405a0f72d529b4120471adabf4f12469c6baf7be22e` |

The final epoch2 model is the completed V4-10 training artifact. Both checkpoints
include the model type,36-point grid, optimizer, epoch, execution commit and
selection identity. They are not loaded by the existing46-point ROS runtime.
Existing V4-20 weights and failed run caches/logs remain untouched.

Final expanded-train diagnostics: near0.025094 m (14,071 anchors/16 runs),
middle0.171865 m (12,920/7), at10 m0.314532 m (12,661/7), observed-support
self-intersections0/14,071. These training errors are not generalization metrics.
Final validation remains as above, observed-support self-intersections0/64.

Execution, audit summary, fixed metrics and stdout/stderr copies are on Windows at
`E:\workspace\e2e_lite_transfuser\tmp\spatial_ten_v4_20260910_run01`.
Large teachers, prediction arrays, model weights and source assets remain in WSL.
No AWSIM trial, runtime switch, push, original-data deletion or global filesystem
repair was performed. The successful checks establish integrity of the assets used
by this run, not every file in the recovered distro.
