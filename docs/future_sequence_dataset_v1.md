# Future-sequence Dataset V1

Issue #3 adds a training-only wrapper around the existing causal TimePath
Dataset. Policy inputs remain in `TimeSample.inputs` as `ModelBatchV3`; future
Camera, LiDAR, ego state, and recorded final commands are returned in a
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

Observed on 2026-09-27 with the existing Windows test environment: **22 passed
in 14.57 s**, including collection-intervention masks and timestamp validation.
The earlier synthetic timing and Windows-only full-suite failures are superseded
by the native WSL real-data and full-suite results below. Linux-only tests must
run in the native WSL environment.

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

## Measured result: 2026-09-27

Source commit: `6e8c8c6f2ff262f89aa1b01dd5d4b7071128c330`. Native WSL Python 3.10,
PyTorch 2.7.1+cu128; this probe uses CPU processing with two Torch threads.
The full native WSL suite at that commit passed: **3,333 passed, 4 skipped,
88 warnings in 119.26 s**. Skips are optional OSQP, two JSON-schema validator
checks, and the optional official LiDAR package.
The existing 5 km/h and 8 km/h **setting** runs contain 3,907 and 2,213 camera
anchors. These settings are not claims about measured vehicle speed. Two raw
bags total 2,020,335,616 bytes; their hashes match the frozen train split.

| Selected-anchor result | Count |
| --- | ---: |
| Audited anchors, including boundary and missing-input cases | 64 |
| Valid policy inputs | 62 |
| Valid inputs and all 30 XY teacher points | 58 |
| Target-speed teacher masks, including two input-invalid anchors | 60 |
| Valid future image / LiDAR / command at each of 0.5 s and 1.0 s | 62 / 62 / 62 |
| Collection-intervention anchors, held from IL | 4 |
| Explicit environment-stop annotations | 0 |

The two invalid policy inputs are `CURRENT_SENSOR_MISSING`. Every original XY
and velocity teacher and mask replays exactly, including collection-braking
exclusions. Mutating future image, LiDAR, velocity and commands at one real
anchor per run leaves every policy-input tensor unchanged. All future tensors
pass shape, dtype and finiteness checks. Missing future at the bag ends remains
masked. No environment-stop label is fabricated; a stop head cannot yet be
trained from these runs without explicit annotation.

| Measured operation | Elapsed |
| --- | ---: |
| Source and selected-artifact hashing, two runs | 5.235 s |
| SQLite index and basic integrity checks, two runs | 9.953 s |
| Selected sensor decoding, two runs | 0.500 s |
| Existing bags to decoded records, including setup | 16.565 s |
| DataLoader assembly, batch 4 / workers 0 / list collation | 1.666 s |
| Existing bags to 64 assembled samples | 18.230 s |
| Future-only cache build, write and digest verification | 5.380 s |
| Future-only cache read and digest verification in a fresh process | 0.672 s |

DataLoader assembly is 38.42 selected samples/s, excluding bag reads. The fresh
process reused 64/64 records without calling a builder; the cache is 20,265,916
bytes. Cache read timing excludes interpreter startup and is not end-to-end
training throughput. The main preparation costs in this bounded run were
indexing and source verification. Reuse the verified preparation for training
iterations instead of repeating a bag-wide audit each epoch.

Evidence is in native WSL `runs/validation/future_sequence_real_20260927/`
(`plan.json`, `report.json`, `cache_expected.json`, `cache/`, `pytest.xml`).
Windows report/log copies are in `tmp/future_sequence_real_probe_20260927/`.
The large bags and cache are not committed. Pretrained DINO verification,
training quality, RTX 5060 latency and AWSIM closed-loop behavior remain outside
this data check.

Full-suite reproduction from the synchronized native checkout:

```bash
bash tools/with_wsl_training_lock.sh bash tools/with_workspace_cache.sh \
  timeout 600 env PYTHONPATH=src .venv/bin/python -m pytest -q \
  --junitxml=runs/validation/future_sequence_real_20260927/pytest.xml
```
