# Common IL V1: six waypoints, learned speed and explicit stop intent

Issue #5 adds an independent model and checkpoint family for the ResNet18 /
frozen DINOv3 comparison. The retained TimePathV1 model, its 30-point contract,
checkpoint format and runtime remain unchanged. This is offline training code;
it does not replace a running controller or Safety Supervisor.

## Output and teacher contract

The fixed contract from Issue #2 is:

| Output | Shape | Meaning |
| --- | --- | --- |
| `waypoints_m` | `[B,6,2]` | x forward/y left, metres in base_link at observation; 0.5, 1.0, 1.5, 2.0, 2.5, 3.0 s |
| `target_speed_mps` | `[B,1]` | signed measured longitudinal speed at t+0.5 s; learned independently of XY |
| `stop_logit` / `stop_probability` | `[B,1]` | logit / sigmoid for explicitly annotated environment stop intent at observation |

The existing 0.1 s / 30-point TimeTeacher supplies XY indices 4,9,14,19,24,29
and velocity index 4. No future is interpolated or fabricated here. The reused
`TimeModelConfig` describes the existing input/preprocessing/fusion and teacher
source; `common_il_contract()` exclusively defines the new six-point output.

`collate_common_il` returns input-only `ModelBatchV3` and sibling
`CommonILTargets`. A policy forward rejects a batch containing targets. From a
`TemporalTrainingSample`, pass only `.policy`; its future observation/action
branch is unused. TimeTeacher masks preserve collection-intervention exclusions.
Stop supervision comes only from `environment_stop_intent`, never low speed,
observed stationary state, collection braking, or bag end.

Losses are per-supported-anchor XY L1 in m, supported scalar-speed L1 in m/s,
and supported binary cross entropy on stop logits. Each head has an independent
denominator; weights default to 1/1/1. Masked NaNs are removed before arithmetic.
Valid nonfinite values, wrong shapes/masks/devices and nonbinary stop labels
raise an error. Missing or disabled heads contribute no computation graph;
their gradients remain None so AdamW decay and old momentum do not update them.
An entirely unsupported batch skips forward, BatchNorm updates and the optimizer.

## Matched arms and old-model compatibility

`configs/common_il_v1/resnet18.json` and `dinov3.json` differ only in the camera
selector. They share teacher selection, run split, sampler seed, loss weights,
head/fusion initialization seed and finite update budget. Camera construction
uses a separate RNG stream after initializing the common backbone, and heads
use another fixed stream. All non-camera parameter initializations match exactly.

The initial ResNet18 arm is random-initialized and trainable. DINOv3 uses official
ViT-S/16 LVD-1689M weights with the backbone frozen; its projection remains
trainable. These pretraining/freezing differences are intentional and recorded,
not an architecture-only ablation. Portable DINO paths are null. Resolve them
with the workspace-generated `runs/setup/dinov3/model_config.json` after placing
the official weights. No fallback to random DINO weights or network download is
used by the experiment loader.

The checkpoint format is `aic_common_il_checkpoint_v1`. It stores model/config,
output contract, camera provenance and dataset/update/support metadata. The
loader rejects old checkpoint formats; legacy model loaders remain untouched.
This diagnostic checkpoint is not an optimizer-resume format or portable vehicle
deployment artifact. DINO reload still needs its declared local source/weight
assets; inference packaging belongs to Issue #6.

## Finite reproduction

Commit on Windows, synchronize the exact source to native WSL, then use the
shared training lock and workspace cache wrapper. Use a new output directory
for each run. Experiments do not overwrite existing reports/checkpoints.

First fit the three shared heads to four explicit synthetic feature/label pairs:

```bash
bash tools/with_wsl_training_lock.sh bash tools/with_workspace_cache.sh \
  timeout 900 env PYTHONPATH=src .venv/bin/python tools/train_common_il_pilot.py head-fit \
  --config configs/common_il_v1/resnet18.json --device cpu \
  --output runs/validation/common_il_head_fit_20260927
```

Budget: 64 optimizer steps, four fixed feature vectors, LR 0.01, AdamW decay 0,
gradient clip 1.0, two CPU threads. This tests the heads' ability to fit simple
labels, including both stop classes. It does not establish encoder or driving
quality. `plan.json` is written before fitting; `report.json` records every step.

Then run the real-cache ResNet pilot:

```bash
bash tools/with_wsl_training_lock.sh bash tools/with_workspace_cache.sh \
  timeout 900 env PYTHONPATH=src .venv/bin/python tools/train_common_il_pilot.py real \
  --config configs/common_il_v1/resnet18.json \
  --cache ../datasets/cache/time_training_20260913_v2 --device cuda \
  --output runs/validation/common_il_resnet_pilot_20260927
```

Budget: the two declared train runs, at most 32 selected anchors/run, batch 4,
32 optimizer steps / 256 presentations, float32, TF32 disabled, at most 900 s.
The existing cache inventory is hash-verified once at startup, not each epoch.
Only selected anchors enter the pilot. Source manifest and selected anchor IDs
form `dataset_sha256`; identical arms reuse this selection and sampler seed.
Reports contain before/after training-subset losses, independent support counts,
updates, missing inputs/labels, time, peak allocated VRAM and exact checkpoint
replay. These are train-fit diagnostics, not validation/test/generalization scores.

The DINO arm uses the same command with `--config configs/common_il_v1/dinov3.json`
and `--dinov3-model-config runs/setup/dinov3/model_config.json`. Local overrides
must change only the matching camera assets. Real DINO fitting is pending the
official checkpoint; fake-backbone unit tests are not that validation.

Optional explicit stop annotations are passed using `--stop-annotations`:

```json
{
  "format": "explicit_environment_stop_v1",
  "cache_manifest_sha256": "<the cache identity SHA-256>",
  "rule_version": "manual_environment_intent_v1",
  "labels": {"<actual train anchor ID>": true}
}
```

Only known train-anchor IDs and boolean labels are accepted. Unknown anchors
remain unlabelled; the file hash enters dataset identity. No annotation file is
supplied for the initial real-data pilot, so the stop loss is null and its head
must remain unchanged. Stop positives and negative controls need targeted
annotation/collection before that head can be trained or evaluated on real runs.

## Controller / Safety boundary

`control.common_il_adapter_v1.adapt_common_il_output` maps a prediction to the
existing `AuthoritativePlanV3`: six timed base_link XY points and a speed value
held across the reference. Repeating the scalar is an explicit controller
assumption, not a learned six-step speed profile. The forward-only controller
reference clips signed speed to `[0, speed_cap_mps]`.

The adapter requires explicit head-supervision status. Unsupported speed cannot
be used as a learned controller target. Unsupported stop remains None; callers
requiring model stop will fail the existing plan validation. Stop supervision
does not prove calibration. Frame/origin transforms, data age, tracking control,
sensor timeout, stopping distance and final braking stay outside the model.
No ROS topic, active checkpoint, controller setting or Safety behavior is changed.

Focused tests:

```bash
python -m pytest -q tests/test_common_il_v1.py tests/test_dinov3_model_integration.py tests/test_future_sequence_dataset_v1.py
```

Tests cover six-point SI contracts, source/annotation identity, masked-NaN loss,
AdamW old-momentum/decay isolation, all-unsupported batches, equal common
initialization across encoders, frozen DINO gradients through a fixture,
checkpoint replay, legacy TimePath loading and the offline controller boundary.

## Observed diagnostic results (2026-09-27)

Implementation and experiment source: `d5b75590f2b7ac50e982c3c3a2d5b7547dd9f724`.
The focused Windows suite passed 36 tests. The full native WSL suite passed
3,348 tests, with four skips and 93 warnings, in 113.84 s. The skips cover
unavailable OSQP, two existing jsonschema/Draft2020 checks, and an optional
official LiDAR package; those integrations remain unverified. Native WSL used
Python 3.10, PyTorch 2.7.1+cu128 and the existing RTX 4080. DINO pretrained
experiments were not run because official weights are still absent.

The full-suite command, run from the native WSL workspace, was:

```bash
bash tools/with_wsl_training_lock.sh bash tools/with_workspace_cache.sh \
  timeout 600 env PYTHONPATH=src .venv/bin/python -m pytest -q \
  --junitxml=runs/validation/common_il_resnet_pilot_20260927/pytest.xml
```

The 64-step fixed-feature head fit (`synthetic_fixed_fused_features_v1`) used
four labelled samples, including two stop positives and two negatives:

| Synthetic head loss | Before | After |
| --- | ---: | ---: |
| XY L1, coordinate mean in m | 1.219004 | 0.038731 |
| Scalar speed L1 in m/s | 0.764807 | 0.063234 |
| Stop BCE | 0.850513 | 0.000576 |

The ResNet pilot selected 32 uniformly spaced anchors from each of
`5kmh_run01` and `8kmh_run01`, retaining run-based train membership. This differs
from Issue #3's boundary-enriched audit selection; counts should not be compared
as an input-validity improvement. All 64 selected inputs were valid; 61 had
waypoint/speed support. Three remained unsupported under the source masks.
The sampler presented 128 anchors over 32 optimizer updates. Losses below are
evaluation-mode **training-subset** losses on the same selected samples:

| Real-cache head loss | Before | After | Supported anchors |
| --- | ---: | ---: | ---: |
| XY L1, coordinate mean in m (not Euclidean ADE) | 1.493029 | 0.362948 | 61 |
| Scalar speed L1 in m/s | 1.701119 | 0.112004 | 61 |
| Stop BCE | null | null | 0 |

Stop parameters remained bit-identical, with no stop supervision. Reopening
`pilot.pt` reproduced all three output tensors exactly. The lower XY/speed
training loss demonstrates that the training path can optimize these heads;
it does not establish heldout quality, high-speed driving or closed-loop gains.

Dataset identity:
`d5b6756a6f26f18539c707efe459f7d30e023b0d9caf23aab78bce27c730457f`.
`dataset.json` contains the cache manifest identity and all 64 anchor IDs.
The camera was random-initialized ResNet18; no pretrained-camera claim applies.

Preparation, including a one-time verification of the existing cache inventory,
took 56.050 s. The 32-update training loop took 3.537 s. Peak allocated CUDA
memory was 839,936,000 bytes (about 0.782 GiB); this is training on RTX 4080,
not total device allocation or an RTX 5060/Jetson inference measurement.

Evidence stays inside the native WSL workspace:

- `runs/validation/common_il_head_fit_20260927/{plan,report}.json`
- `runs/validation/common_il_resnet_pilot_20260927/{plan,dataset,report}.json`
- `runs/validation/common_il_resnet_pilot_20260927/pilot.pt`

Small report/log copies are in Windows `tmp/common_il_20260927/`. No checkpoint,
bag, cache or dataset was added to Git. Remaining work is the real pretrained
DINO pilot, explicit positive/negative stop annotations, then the later matched
comparison, inference packaging and closed-loop evaluation. The current pilot
must not replace the retained driving model.
