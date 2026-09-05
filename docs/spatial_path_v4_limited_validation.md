# Spatial Path V4 fixed-step500 limited validation

## Frozen scope before inference

Base/result commit `dfedd6de00eb3592b3e0566a7fc99b7c1f08ea5c`; prior execution
`f33b197df9eb1e6ae2af70c9661d8b1d88ab4ef0`. No changes to prior input, teacher,
model, trainer, runtime, S1/S2/Ledger, controller or Safety. A separate read-only
adapter protects fixed train64 and selected validation assets; test assets are denied.
Existing coverage ledger is used only as explicitly located, hash-checked annotation
metadata. Its raw source_uri strings are never followed. No new acquisition ledger.

Checkpoint is only prior run/checkpoints/final.pt, expected SHA
`0316692543a901d9d6b96718c5651d6739367aa851f83fb3a133c126ccc8919f`.
Hash before deserialization, weights_only=True, exact format/key/shape/dtype and
strict full load, no allowlist partial initialization, scratch/CPU/other-weight fallback.
eval + inference_mode + requires_grad false; complete parameter/buffer hashes before/after.
No optimizer construction/step, BN updates, interventions, retry or training.

Replay prior train64 in same order and microbatch2; compare all 20 XY points with
rtol1e-5/atol1e-6m, fixed numerical tolerance, not performance/safety gate.
Mismatch stops before any validation candidate/prediction. Saved train baseline
template is recomputed solely from fixed train64 and compared to prior baseline bytes.
Prior critical artifact hashes come from the verified first review packet; no val fitting.

Validation: seed42, each of five fixed runs <=256 future candidates, <=32 main,
<=4 observation-only; totals <=1280/160/20. Metadata sorted run/segment/time,
per-run seeded shuffle and reverse-pop matching prior candidate mechanics.
Eligible main selection uses shape round robin and >=0.5s same-run gap. The exact
old predicates are copied into a pure eligibility helper so the trainer is not imported:
invalid current ego, any nonfinite valid future, abs current speed <=0.05m/s,
processed support unknown/<0.5m, every teacher diagnostic flag, and cut reason
position_jump/invalid_time_grid_or_gap/nonfinite_valid hold the sample out of main.
Observation buckets retain old (cut, first reason or heldout marker, normal/recovery)
round robin among unselected current-ego-valid anchors; no new eligibility thresholds.
Geometric shape is not route intent. Missing labels remain UNKNOWN.

Selection and candidate ledger are written before val sensor inference, with a
content identity. No selection changes after predictions. Selected histories only,
same 4/4/10/10 preprocessing, no cross-epoch command/state. Upper active budget
1800s includes replay, candidate/input preparation and val evaluation (stricter than
GPU-only timing); final hashing/metrics/figures/package I/O is separately outside it.
Timeout is PARTIAL with processed mask and full selected IDs; nonfinite output is
BLOCKED and not silently dropped. All-NaN unprocessed prediction padding is not
a numeric prediction and has explicit processed=false/unknown status.

Metrics: anchor-mean Euclidean ADE on teacher support, role nested before each
run/shape/normal-recovery/collection slice, support/scored denominators, distance
errors, lengths, paired baseline wins/losses and mean model-minus-baseline error.
Undefined train-template distances remain NaN/UNKNOWN, never val mean or zero.
Proper crossings are separate all20 vs teacher prefix; origin assists geometry,
never ADE. Collinear/touching crossings are not detected. No safety inference.

## Execution

Windows source commit -> CheckOnly -> normal sync -> same WSL commit under lock.
No push, reset, process-stop, lock-delete or destructive sync.

```powershell
.\tools\sync_to_wsl.ps1 -CheckOnly
.\tools\sync_to_wsl.ps1
```

Native WSL checkout; create only new external log/run destinations. Capture stdout
and stderr separately, preserve JUnit and include raw logs in the new run afterward.

```bash
OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -vv tests/test_spatial_diagnostic_validation_v4.py tests/test_spatial_diagnostic_geometry_v4.py tests/test_spatial_diagnostic_package_v4.py --junitxml=<new_logs>/junit.xml
OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 bash tools/with_wsl_training_lock.sh .venv/bin/python tools/evaluate_spatial_diagnostic_validation_v4.py --config configs/spatial_diagnostic_validation_v4.yaml --output <new_run>
```

No full pytest or old optimizer/smoke tests. New tests actively forbid optimizer
construction. Reproduction failure and missing input stop, not auto-retry.

Package after separately committing results/review request:

```powershell
python tools/package_spatial_validation_v4.py --repo E:/workspace/e2e_lite_transfuser --run <explicit_native_run_UNC> --request <attachment> --report <result_md> --review <independent_review_md> --output tmp/spatial_v4_validation_review_<run>.zip --base dfedd6de00eb3592b3e0566a7fc99b7c1f08ea5c
```

Fixed executed Git source bytes; prior trainer included only for static predicate
comparison, not executed. Weights/original sensors/raw/full Dataset omitted.
Hash manifest, compressed64MiB/expanded128MiB limits and fresh extraction check
reuse the tested previous packager's safe verification. Two actual input examples,
all selected futures/targets/predictions/baselines and logs remain core evidence.
No independent inference without weights; no independent preprocessing rebuild
without original sensors. Run point estimates are not strong population/safety claims.
