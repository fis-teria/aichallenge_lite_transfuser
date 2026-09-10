# 20 m diagnosis: learning duration versus amount of used data

User authorized a finite cause-isolation experiment after the first diagnostic.
Windows source, native WSL GPU execution under the shared worktree lock.
Only long-horizon training/tests/docs are owned; control/ROS/AWSIM are preserved.

## Predeclared comparisons

Parent: `/home/thistle/e2e_autonomous/runs/spatial_long_v4_20260910_run01`.
The parent initial/final weights, teachers, selection and audit are hash verified.
Every selected teacher is regenerated from its hash-verified canonical future.
No additional candidate audit, teacher threshold changes or final test reads.
The same original 64 validation anchors/masks are used throughout. They have
already been observed in the first experiment: this is a development comparison,
not a new independent final evaluation. No validation-based checkpoint selection.

| Probe | Starting point | Data | Additional updates | Purpose |
|---|---|---|---:|---|
| tiny8 | Original saved random initialization, fresh optimizer | Same eight original overfit anchors | 1,000 | Ability to fit a tiny fixed set |
| same128 | Original step500 model AND optimizer | Original128 | 1,000 | More optimization with unchanged data |
| expanded256 | Identical step500 model AND optimizer | Original128 plus128 | 1,000 then2,000 | Equal-compute comparison, then equal additional presentations per anchor |

Expansion doubles every original run/observed-support-bin quota, using eligible
train-only audited candidates, deterministic seed43, >=0.5 s same-run separation
against all chosen anchors. Insufficient quota is an explicit failure, not a
relaxed selection. It adds anchors within the same runs; it does not add new
run diversity or prove independent scene diversity. Shape composition is recorded.

At +1,000 steps each paired branch has 8,000 presentations; original128 averages
62.5 presentations/anchor, expanded256 averages31.25. Expanded +2,000 averages62.5.
Matching is for additional average exposure only: original pretraining exposure is
inherited and individual uniform-random presentation counts vary. Those counts
are saved. B at2,000 versus A at1,000 also differs in compute and optimizer age.
Both branches evaluate original128, their own train cohort, and fixed validation.

Model, input contract, equal-present-band SmoothL1, LR (1e-4/1e-3), AdamW,
clip1, float32, TF32 disabled and microbatch2 x accumulation4 are unchanged.
No LR search/decay or geometry regularization is introduced. Paired branches
restart RNG identically at43; they are not bit-exact resumes of historical RNG.
Tiny100/500/1,000 and paired500/1,000/2,000 snapshots are predefined, never best-picked.

Budget: one RTX4080; total4,000 optimizer updates; global training/evaluation
active limit2,400 s with120 s reserve. Preparation/checkpoint/artifact I/O also
has finite work counts; active timer includes snapshots/checkpoint I/O after
training starts. Failure/OOM/NaN/timeout is recorded, with no automatic sweep.
All new artifacts go to a NEW native run; parent and 2 m weights stay untouched.

## Interpretation rules fixed before execution

- Tiny fit improves materially: prior small-set failure was at least partly a
  training-budget issue; it does not prove generalization or physical correctness.
- Same128 train improves but validation stalls/worsens: a generalization gap is
  evidenced under fixed inputs/model/loss, not proof that dataset quantity alone
  is the cause. Fixed-LR optimization limits remain possible.
- Expanded improves fixed validation at equal compute: more used training anchors
  helped this controlled comparison. Within-run diversity is not new-run diversity.
- Expanded helps only with more updates: data and optimization budget interact.
- Persistent near error/self-crossing: insufficient support for treating this as
  a ready runtime model. No inferred safety from average distance error.

## Reproduction

```powershell
# After committing owned source on Windows:
.\tools\sync_to_wsl.ps1 -CheckOnly
.\tools\sync_to_wsl.ps1
```

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
tools/with_wsl_training_lock.sh env OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 .venv/bin/python -m pytest -q
tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 .venv/bin/python -u -m aic_transfuser_lite.training.spatial_long_diagnosis_v4 --parent /home/thistle/e2e_autonomous/runs/spatial_long_v4_20260910_run01 --output /home/thistle/e2e_autonomous/runs/spatial_long_v4_diagnosis_20260910_run01
```

Redirect stdout/stderr to a new sibling logs directory. The execution manifest
records source/parent/data hashes, phases, metrics, exposures and checkpoint
replay. Preserve the failed logs too if a setup error occurs.
