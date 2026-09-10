# All audited train teachers: bounded continuation

User authorized using the unused eligible teachers. Use all1,786 audited train
anchors (normal766/recovery1,020; observed20 m165) and the same fixed64 validation.
These are all eligible train anchors from the prior bounded2,048-candidate audit,
not all train anchors in the underlying canonical dataset. Test remains untouched.

Start from `spatial_long_v4_diagnosis_20260910_run01/expanded256_2000.pt`, SHA256
`dc350ed3e892ecdb265795328aef0c2f6940f03a0f257e45e585d13c13362bf9`.
Strict model and independent optimizer restore at optimizer step2,500; RNG seed44
is new. Keep model/loss/LR/batch/float32/TF32/preprocessing contracts unchanged.
This is a continuation with more data, not a scratch or paired equal-compute trial.

Every train anchor is used exactly once per shuffled epoch, with the final smaller
batch retained and accumulation normalized by its real size. Sixteen epochs,
224 updates/epoch, at most3,584 additional updates and28,576 train presentations.
All1,786 rows must have16 visits before COMPLETE. No 0.5 s thinning is applied;
adjacent observations are correlated. Recovery fraction changes from54/256 to
1,020/1,786. This distribution change and long-support counts are reported.

Budget: one RTX4080; global active training/evaluation2,400 s with180 s final
reserve. Input generation is finite1,850 anchors. Native disk caches retain exact
float32 preprocessing tensors; 16-anchor CPU LRU avoids keeping all inputs in RAM.
At least12 GiB free is required. No cache/weights are written on /mnt/e or Git.
Checkpoint/intermediate evaluation at epochs4/8/12/16, fixed old128 train +64 val;
final evaluation also includes all1,786 train. No validation-based best selection.
Failure/OOM/NaN/time-limit is recorded, no automatic unbounded retry or sweep.

Every used future is rehashed and target regeneration compared exactly. Input
cache fields exclude teachers and are hash-verified on load. Two real anchors
are rebuilt to prove tensor parity. Final checkpoint strict reload/replay, all
read source assets and all cache files are rehashed before completion.
Observed masks never enter forward. No raw acquisition, teacher promotion,
control/ROS/AWSIM edit, runtime deployment, final test or push is performed.

```powershell
# Commit owned source on Windows, then:
.\tools\sync_to_wsl.ps1 -CheckOnly
.\tools\sync_to_wsl.ps1
```

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
tools/with_wsl_training_lock.sh env OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 .venv/bin/python -m pytest -q
tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 .venv/bin/python -u -m aic_transfuser_lite.training.spatial_long_full_v4 --parent /home/thistle/e2e_autonomous/runs/spatial_long_v4_20260910_run01 --checkpoint /home/thistle/e2e_autonomous/runs/spatial_long_v4_diagnosis_20260910_run01/expanded256_2000.pt --output /home/thistle/e2e_autonomous/runs/spatial_long_v4_full_20260910_run01
```

Choose a new run output and sibling logs; preserve stdout/stderr. Source, dataset,
teacher, cache, checkpoint hashes, visit counts and execution commit are recorded.
Same development validation is already observed, not new independent evaluation.
Safety, route intent, obstacle clearance and closed-loop performance remain unverified.
