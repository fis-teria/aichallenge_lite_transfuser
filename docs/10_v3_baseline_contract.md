# V3-000: V1 Baseline Contract

## 1. Purpose and scope

This document freezes the observable V1 contract before V3 implementation begins. It is the only deliverable for `V3-000` in `specs/v3_task_manifest.yaml`.

- Specification baseline: `main` at `af0faf6ffe23e5be2f7be1c3137ee3f22e8aaeac`
- Audited checkout: `codex/windows-wsl-training-sync` at `78c59de8a8f5158394391a937b0450400f348e24`
- Repository: `fis-teria/aichallenge_lite_transfuser`
- Audit date: 2026-09-02 JST
- Runtime behavior changed by this task: **none**
- V3 schema, model, configuration, ROS node, and compatibility-view implementation: **not started**

The audited HEAD descends from the specification baseline. The two later commits add only the Windows/WSL workflow and its documentation. `git diff` reports no changes between the specification baseline and the audited HEAD for any frozen path listed below. Before this document was added, the only dirty files were the untracked, user-supplied `design_docs/aic_transfuser_v3_codex_package/` package.

## 2. Frozen paths

The following files MUST NOT have their V1 meaning changed by V3 work. Shared behavior must be added through a new V3 module or adapter unless a later manifest task explicitly grants an exception.

The SHA-256 values below are for the LF checkout bytes at audited HEAD in WSL. A Windows CRLF working tree can have a different byte hash without a semantic change, so regression checks should use an identical checkout and line-ending policy.

| Frozen path | SHA-256 |
|---|---|
| `configs/transfuser_lite_v1_static.yaml` | `f03a21bbdb3cc8211a3e81337a8fc34391d9ded8ea60038a7da47d115ad63ebb` |
| `src/aic_transfuser_lite/data/topic_contract_v2.py` | `f1aae27677e11d7d4405f6ffa119cc8d25f8bc7ce27d43c7677057e755f83642` |
| `src/aic_transfuser_lite/data/mcap_converter_v2.py` | `7632aac289a1422fca76cbc50f3f88c4684a17a50f7e0248631274d16e05d641` |
| `src/aic_transfuser_lite/data/dataset_v2.py` | `806c4d5dc9b6d36650b607f6fed7b19bb17c04fc74ba2923242b70b482850218` |
| `src/aic_transfuser_lite/models/transfuser_lite_v1.py` | `732204e42420b51dbd7a0fb52add1906228879d169664596746a04f500c3ade5` |
| `src/aic_transfuser_lite/training/train_v1.py` | `ea73bbffcf2ec748d209b0ad186264753de7ce8fbcc6db6a313bda77a9b3495e` |
| `src/aic_transfuser_lite/training/checkpoint_v1.py` | `ffda246b1c96961c3dd4860288eb53abb78004b83bcdbf2db846999acff65f4a` |
| `src/aic_transfuser_lite/runtime/model_loader_v1.py` | `ca5a32a3b4a341feaea38e5a21df73e26977e3a5c82e7f63a898f5949eae8258` |
| `ros2_ws/src/aic_e2e_runtime/aic_e2e_runtime/inference_node_v1.py` | `d331ea61ea7f2985b92154dc890e2553d8e8ed97aa26e1b5ed73eebc9ced3b25` |
| `ros2_ws/src/aic_e2e_runtime/config/runtime.v1.param.yaml` | `8792a639ed5e690fb5f6f7cbc01aaf2e14e31dcde7008aa9c523014ad5d8f632` |

## 3. Dataset V2 and model-input contract

### 3.1 Recording and canonical data

Dataset V2 is `format_version: 2`. Its required conversion streams are Camera, native LaserScan, measured global Odometry, measured velocity, gear, final control command, and nominal control command. Actual steering is optional for conversion and `/clock` is recording-only. Missing actual steering is represented as `NaN` with `actual_steering_valid=0`; it is never silently replaced with a command or zero.

The converter uses a 10 Hz regular Camera grid, nearest LiDAR, interpolated measured pose/velocity/actual steering, and causal previous commands. LiDAR geometry is inferred from the bag and must remain identical within and across runs. It is not resampled. Future waypoints use measured global poses transformed into the observation ego frame:

- frame: observation ego frame
- x-axis: forward
- y-axis: left
- distance: m
- angle: rad
- waypoint times: 0.5, 1.0, 1.5, 2.0, 2.5, and 3.0 s
- target speed: measured longitudinal speed at observation time +0.5 s
- split unit: `run_id`, never a frame-random split

Collision, off-track, recovery, and intentional-stop labels are not inferred from low speed or commands.

### 3.2 Inference inputs

`DrivingDatasetV2` exposes only inference-time inputs and trained targets. State, command, quality, and debug columns remain in the dataframe for audits but do not enter the model input batch.

| Key | Shape | Unit / preprocessing | Contract |
|---|---:|---|---|
| `image` | `[B, 3, 180, 320]` | RGB, resized then ImageNet-normalized | finite float tensor |
| `lidar` | `[B, 2, 750]` | channel 0 normalized range from raw m; channel 1 validity | native beam order and exact geometry |
| `ego` | `[B, 1]` | measured signed longitudinal speed divided by 10 m/s | speed only; no steering or command input |

Runtime LaserScan must have exactly 750 beams, frame `lidar`, range [0, 25] m, `angle_min=-1.5666074752807617` rad, and `angle_increment=0.004188789986073971` rad within the explicit tolerances in `V1LidarContract`. Geometry, frame, shape, non-finite speed, or batch-size drift is rejected.

## 4. Model-output and training contract

`AICTransFuserLiteV1.forward(image, lidar, ego)` returns exactly these keys:

| Key | Shape | Unit / constraint |
|---|---:|---|
| `waypoints` | `[B, 6, 2]` | m in observation ego frame; x forward, y left |
| `target_speed` | `[B, 1]` | m/s; non-negative through `softplus` |

The static V1 configuration disables the stop, behavior-mode, and direct-control auxiliary heads. Their corresponding loss weights are zero. Enabling a loss without its head is an error. The V1 objective consists of waypoint Smooth L1, target-speed Smooth L1, and an optional target-relative shape term; raw and weighted components are reported separately.

Training uses Dataset V2 for the static model, rejects train/validation run overlap, records deterministic sampler order and source hashes, and refuses resume when the resolved configuration, dataset manifest, embedded configuration, scheduler step, history, or RNG state does not match.

## 5. Checkpoint contract and pinned artifact

The checkpoint format identifier is:

```text
transfuser_lite_v1_checkpoint_v1
```

The V1 loader requires at least `model`, `config`, `resolved_config`, `epoch`, `global_step`, `resolved_config_sha256`, and `dataset_manifest_sha256`. The runtime additionally requires:

1. an externally supplied lowercase SHA-256 equal to the checkpoint bytes;
2. equality of `config` and `resolved_config`;
3. a valid V1 embedded configuration and matching embedded configuration hash;
4. model construction from that embedded configuration;
5. `model.load_state_dict(..., strict=True)`.

Legacy V0 or unversioned checkpoints are intentionally rejected. V1 checkpoints MUST NOT be converted in place or have their state-dict keys renamed.

The current V1 runtime parameters pin this external artifact in the WSL repository checkout:

| Evidence | Value |
|---|---|
| Checkpoint role/path | `runs/transfuser_lite_v1_static_dataset_v2_exclusive_dropout_p020_seed42_50ep/best_ade.pt` |
| Checkpoint SHA-256 | `1b82e33aa676ccc433a66781658ba9a919d88de34df6c0bc6948738e130dbb84` |
| Selected epoch/global step | 49 / 1029 |
| Embedded resolved-config SHA-256 | `26e5195b243bc6436d2639c55617af56a73e94b86b18f71b1bf9aa549da7c1f1` |
| Training dataset-manifest SHA-256 | `8825d68497143bada4695ce181710ca2cdcd976b6794be57cb4c2bf589854b57` |

These artifacts are ignored external files and are not committed to Git. The frozen source config is not byte- or value-identical to this checkpoint's resolved config: the artifact enables mutually exclusive Camera/LiDAR full dropout at probability 0.2 for each modality. Runtime construction is still deterministic because `model_loader_v1.py` uses the checkpoint's embedded resolved config, not the repository config file.

The artifact's `run_manifest.json` records `git_revision: no-git-repository`. Therefore, checkpoint bytes and source/config hashes are available, but the original training commit is not proven by that manifest. This provenance limitation must remain visible in any V3 migration report.

## 6. Dataset, split, and open-loop evidence snapshot

The following ignored WSL artifacts were inspected read-only; they were not regenerated by `V3-000`.

| Artifact | SHA-256 / result |
|---|---|
| `datasets/processed/aic_real_dataset_v2/metadata.yaml` | `fcb4465e96baa697301cf5af07c4613c1b9090fd99bc7f6f4f2b510ef71cbcbb` |
| `train_index.csv` | `d64ccd2840f165808286ae0933b575f84b7c04dd0845436a1430eb37eea03554` |
| `val_index.csv` | `b642739e09cd0688fae518e298c06ca3584c60606bcc6e8401c332ddd19042bc` |
| `test_index.csv` | `9db937339bfe05504979036e855c0475fcf2935b24067daab71170ef40e73b27` |
| Split rows | train 1342, validation 447, test 447; total 2236 |
| Training manifest overlap | none between the three train runs and one validation run |
| `evaluation_best_ade.json` | `181512aad8b8e05d6c799ce33d8cc84549f949b9d06b0956b818627dac197674` |
| Historical test result in that report | 447 samples; ADE 0.2264289184 m; FDE 0.3855733893 m; speed MAE 0.0909322039 m/s |
| Historical batch-1 latency | CUDA AMP, RTX 4080, p95 9.9681900072 ms over 100 runs |

The open-loop report is bound to the checkpoint, test-index, metadata, evaluator, and reference hashes recorded inside it. It is historical evidence only; `V3-000` did not rerun open-loop evaluation and does not treat it as ROS 2 or AWSIM validation.

## 7. ROS 2 runtime contract

`transfuser_lite_v1.launch.py` starts two separate nodes:

1. `InferenceNodeV1` produces `nominal_control_cmd` and `predicted_waypoints`.
2. `SafetySupervisorNode` receives the nominal command and is the only node in this launch that publishes the final `control_cmd`.

The inference node is Camera-mastered and requires synchronized LiDAR, measured velocity, and measured steering. The current parameter snapshot is 10 Hz inference, 30 ms maximum sensor skew, 0.35 s input timeout, and exact checkpoint SHA validation. Invalid timestamps, non-increasing Camera stamps, unavailable synchronization, excessive skew, stale observations, preprocessing errors, non-finite model outputs, or controller errors do not produce a nominal command for that observation.

The independent Safety Supervisor checks Camera/LiDAR/ego timeout, finite values, front obstacle stopping distance, configured model-stop behavior, confidence reduction, and steering/acceleration clamps before publishing the final command. Safety behavior and authority MUST NOT be removed or bypassed by V3 work.

## 8. Rollback and compatibility path

V3 additions must remain side-by-side with V1. To return to this baseline without destructive history rewriting:

1. select the existing `transfuser_lite_v1.launch.py` and `runtime.v1.param.yaml`;
2. provide the checkpoint whose SHA-256 is pinned above;
3. retain Dataset V2 and the V1 model/checkpoint/runtime classes unchanged;
4. validate the checkpoint and embedded config through `load_runtime_model_v1`;
5. run the regression commands in section 9 before deployment.

The specification baseline commit can be inspected in a separate worktree at `af0faf6ffe23e5be2f7be1c3137ee3f22e8aaeac`. Do not reset a dirty checkout or overwrite external datasets/checkpoints to perform rollback.

## 9. Tests and commands executed

### 9.1 Full unit/smoke suite

The Windows default interpreter was checked first:

```powershell
python --version
python -m pytest -q
```

Result: Python 3.10.10 was present, but the command stopped before collection with `No module named pytest`. This is an environment dependency failure, not a test-suite result.

The canonical WSL checkout was confirmed at the same audited commit with a clean tracked worktree, no active training process, Python 3.10.12, and pytest 9.1.1. The suite was then run under the required shared-worktree lock:

```bash
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
```

Result on 2026-09-02 JST:

```text
166 passed, 14 warnings in 18.79s
```

All warnings were the PyTorch Transformer `enable_nested_tensor` warning caused by `norm_first=True`; there were no failures or errors.

### 9.2 Focused negative regression

The following fail-closed cases were explicitly selected: legacy V1 config, missing actual-state zero-fill, Dataset LiDAR geometry drift, mixed converter beam count, model shape/batch drift, legacy checkpoint, runtime LiDAR geometry drift, and optional-output drift.

```bash
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q \
  tests/test_config_v1.py::test_v1_loader_rejects_legacy_config_explicitly \
  tests/test_dataset_v2.py::test_v2_loader_does_not_zero_fill_missing_actual_state \
  tests/test_dataset_v2.py::test_v2_loader_rejects_geometry_drift \
  tests/test_mcap_converter_v2.py::test_mixed_native_lidar_beam_count_fails_closed \
  tests/test_model_v1_shapes.py::test_static_v1_rejects_input_shape_or_batch_drift \
  tests/test_training_v1.py::test_v1_checkpoint_round_trip_and_legacy_rejection \
  tests/test_runtime_preprocessing_v1.py::test_native_lidar_rejects_geometry_drift \
  tests/test_inference_core_v1.py::test_static_v1_inference_rejects_optional_head_drift
```

Result:

```text
12 passed, 1 warning in 4.68s
```

Some selected tests are parametrized, which is why eight node IDs produced twelve cases.

## 10. Compatibility, safety, and unverified items

- Compatibility impact: documentation only; no V1 source, state-dict key, config, Dataset V2, checkpoint, or runtime behavior changed.
- Safety impact: none; the Safety Supervisor and publisher authority remain unchanged.
- ROS 2 launch/build/runtime test: **not run** in this task.
- AWSIM closed-loop test: **not run** in this task.
- Live recording and Dataset V2 reconversion: **not run** in this task.
- Open-loop report regeneration: **not run** in this task; section 6 records an existing hashed report.
- The deployed checkpoint's training commit is unproven because its run manifest contains `no-git-repository`.
- The frozen source config differs from the deployed checkpoint's embedded resolved config in full-dropout settings. The embedded config is authoritative for V1 runtime loading; any future V1-to-V3 migration must report this difference explicitly.

`V3-000` ends with this baseline contract. No `V3-001` or later task has been started.
