# Future-sequence Dataset V1

Issue #3 adds a training-only wrapper around the existing causal TimePath
Dataset. Policy inputs remain in `TimeSample.inputs` as `ModelBatchV3`; future
Camera, LiDAR, ego state, and actually applied final commands are returned in a
sibling `FutureSequenceTargets` object and never inserted into the policy batch.

Initial future horizons are 0.5 s and 1.0 s. Shapes and units are:

- image `[F,3,H,W]`, normalized RGB;
- LiDAR `[F,2,P]`, normalized range plus validity in native angular order;
- ego `[F,4]`: longitudinal speed m/s, lateral speed m/s, yaw rate rad/s,
  measured steering rad;
- applied action `[F,3]`: steering rad, speed m/s, acceleration m/s2;
- a boolean validity mask for every modality/horizon and every ego feature.

Targets never cross a run/clock epoch. A missing future remains zero with a
false mask; it is not interpolated or copied into a synthetic label. Stop is
valid only with an explicit boolean stop-intent annotation. Target speed is
measured longitudinal speed at t+0.5 s and has its own mask.

Pass the existing `intervention_ns[(run, epoch)]` map into
`TemporalTrainingDataset`. It preserves the baseline's conservative exclusion:
if the 3 s IL horizon meets collection braking, XY and target-speed teachers are
masked. Real future observations remain available for auxiliary prediction;
collection braking never invents an environment-stop label. The legacy field
`applied_action` contains the recorded final command at its message timestamp,
not a measurement of physical actuator application time.

The optional immutable NPZ cache identity includes the source SHA-256, complete
Dataset config, horizon/tolerance config, preprocessing version, encoder
identity, and augmentation identity. Reopening a cache root with any different
identity fails rather than reusing stale targets. Raw cached targets declare
augmentation disabled; a future frozen-encoder feature cache must use a distinct
encoder and augmentation identity.

Cache identities are normalized to their JSON representation before comparison,
so a new process can reopen a cache using the same tuple-valued configuration.
The regression test closes over the original builder and verifies that a new
cache instance reuses the record without rebuilding it; changed identities still
fail.

Run the focused tests on Windows:

```powershell
python -m pytest -q tests/test_future_sequence_dataset_v1.py tests/test_time_dataset_p1.py tests/test_time_training_cache_v1.py
```

Observed on 2026-09-27 with the existing Windows test environment: 18 passed in
7.06 s. A bounded synthetic DataLoader smoke with 100 records, batch size 8,
`num_workers=0`, and list collation produced 13 batches, 100 valid policy
inputs, and 200 valid future Camera targets in 0.482 s (207.6 samples/s). This
only checks plumbing and must not be reported as real-corpus storage throughput.

The unfiltered Windows suite cannot collect `test_mppi_collection_isolation.py`
because `fcntl` is Linux-only. A follow-up run excluding that file completed
with 3310 passed, 38 failed, and 25 skipped in 202.31 s. The failures are
pre-existing environment/fixture constraints (Linux `fcntl`, absent ignored
legacy checkpoints, CP932 text decoding, older Torch AMP APIs, and dirty-tree
identity guards); none names the Issue #3 module or focused tests. WSL full-suite
validation remains required after the existing unrelated dirty/staged work can
pass the prescribed synchronization preflight.

For Linux/CUDA validation, first commit on Windows, synchronize the exact commit
using the documented workflow, and run the same command through
`tools/with_wsl_training_lock.sh`. Do not train from `/mnt/e`.

## Bounded real-data check

After synchronization, run from the native WSL checkout:

```bash
bash tools/with_wsl_training_lock.sh bash tools/with_workspace_cache.sh \
  timeout 900 env PYTHONPATH=src .venv/bin/python tools/check_future_sequence_real_data.py \
  --corpus-root ../datasets/processed/time_teacher_20laps_20260913 \
  --run-ids 5kmh_run01 8kmh_run01 --samples-per-run 32 --batch-size 4 \
  --threads 2 --max-source-gib 4 --output runs/validation/future_sequence_real_20260927
```

Use a new output directory for each invocation; existing reports/cache are never
overwritten. Budget: two existing train runs, 64 selected anchors, at most 4 GiB
of source bags, zero optimizer updates, 900 s outer timeout. Test runs remain
sealed. Start/end, missing-input and collection-intervention cases stay in the
denominator. Source hashes and original teacher replay are checked only for the
selected runs. Only the selected sensors are retained as decoded payloads.

`report.json` separates hashing/indexing/decoding, DataLoader CPU assembly, and
future-cache build/read measurements. A fresh Python process must reuse every
future-cache record and reproduce its tensor digest. DataLoader timings start
after bag decoding; future-cache timings omit policy inputs and encoder work.
OS caches are uncontrolled. The input freeze is the existing camera bag receipt
+ 50 ms proxy, not measured preprocessing completion. Existing-bag preparation
time is not a newly measured collection-to-training cycle. This check does not
train a model or establish driving performance. Pretrained DINO forward/backward
remains pending official weights.
