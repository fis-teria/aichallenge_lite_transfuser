# Spatial Path V4: first observed-canonical diagnostic

## Scope fixed before execution

Base: `66749534ef3b0b3dc4c82143734240c25aff149f`. This is a single train-only
diagnostic, not formal teacher adoption or a runtime artifact. No raw/MCAP/bag,
S1/S2/Ledger, controller, Safety, ROS, val/test assets or external weights are used.
No automatic push, retry, selection change or full training is authorized.

Teacher: contiguous h30 prefix in `base_link@t_obs`, X forward/Y left, meters.
Origin is a geometry aid, never scored. Grid 0.1 through 2.0 m; no extrapolation.
The 5 mm reliable-point accumulation, 20 m/s jump cutoff, 0.2 s time gap,
0.5 s hold and corner-cut threshold (max 0.05 m, 10% covered arc) are provisional
diagnostic rules, not calibrated safety requirements. Rear-axle origin is UNKNOWN.
Raw, processed and resampled lengths remain separately recorded. Missing-first
prefix has unknown distance and zero loss points. Teacher mask is coverage only.

Inputs preserve V3 4/4/10/10 camera/LiDAR/ego/past-command preprocessing.
Diagnostic history additionally starts an epoch at >0.2 s gaps/non-increasing
timestamps; no cross-run/segment histories. Two selected recorded anchors are
compared tensor-for-tensor against the V3 lazy materializer using the same epoch
keys. This is not full runtime parity. Sensor-delta tensors are retained but not
fused by the existing V3 architecture. IDs/teacher/mask never enter the path head.

Initialization is **scratch**: no explicitly located trusted checkpoint was
provided. No weight discovery or download. V3 encoder/fusion architecture is reused,
all used parameters trainable, old heads excluded, new 20x2 XY head. V3 receives
only a `forward_features` extraction; default outputs/state keys are regression
tested. S1/Ledger/runtime/controller files are unchanged.

The config fixes seed 42, <=2048 candidate futures, <=64 train +16 observation,
float32/no augmentation, AdamW fresh/weight decay 0, backbone LR 1e-4/head 1e-3,
anchor-balanced masked SmoothL1 beta 0.1 m, clipping norm 1,
microbatch 2 x accumulation 4, <=500 main steps + independent <=1 smoke step.
Training/evaluation active time is <=1800 seconds, with a 60-second final-eval
reserve. Selection/input preparation and checkpoint/archive I/O are outside that
active budget. An overrun/NaN/OOM/identity error is a recorded failure, not a retry.
Fewer than 8 train anchors or fewer than 2 geometric shape classes reduces main
execution to a one-step degenerate smoke. Shape labels are not route annotations.

## Commands (Windows commit, then native WSL under lock)

```powershell
git add <only the files for this task>
git commit -m "Add bounded Spatial V4 diagnostic and review packaging"
.\tools\sync_to_wsl.ps1 -CheckOnly
.\tools\sync_to_wsl.ps1
```

In native WSL checkout (not `/mnt/e`), choose new immutable run/log directories.
Capture stdout and stderr separately, and copy these exact logs into run/logs
after the process exits. Do not pre-create the training output directory.

```bash
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -vv tests/test_spatial_diagnostic_geometry_v4.py tests/test_spatial_diagnostic_training_v4.py tests/test_spatial_diagnostic_package_v4.py tests/test_full_control_lite_v3_shape.py tests/test_runtime_input_history_v3.py --junitxml=<new_logs>/junit.xml > <new_logs>/tests_stdout.log 2> <new_logs>/tests_stderr.log
bash tools/with_wsl_training_lock.sh .venv/bin/python tools/train_spatial_diagnostic_v4.py --config configs/spatial_diagnostic_v4.yaml --output <new_run> > <new_logs>/train_stdout.log 2> <new_logs>/train_stderr.log
```

Full pytest is intentionally not run: the current request limits tests to synthetic
and relevant non-ROS/non-raw tests. Synthetic tests permit optimizer execution.

Packaging on Windows, after a separate results-document commit:

```powershell
python tools/package_spatial_diagnostic_v4.py --repo E:/workspace/e2e_lite_transfuser --run <explicit_native_run_UNC> --request <user_attachment> --report docs/<result_report>.md --output tmp/spatial_v4_diagnostic_review_<run_id>.zip --base 66749534ef3b0b3dc4c82143734240c25aff149f --report-commit <report_sha>
```

Package source is read with `git show <execution_sha>:<file>`; result commits do
not replace executed bytes. Limits: compressed 64 MiB, expanded 128 MiB.
The package is extracted once into new workspace tmp and every entry/hash/README
reference verified. Raw, original sensor files, weights and the full root manifest
are omitted. Two actual input tensor examples support tensor inspection but not
independent reconstruction of preprocessing. Checkpoint hashes cannot support
third-party re-inference without weights. These limitations must remain in reports.

## Interpretation

Compare initial/final same fixed teacher mask and IDs, with zero, mean-train and
straight baselines; final means last step, never best snapshot. A single post-fit
sensor swap may probe sensitivity, not prove sensor understanding. Unknown tails
stay unknown even though the head produces 20 points. Proper-crossing flags do
not detect every collinear/touching self-intersection. Curvature remains uncomputed,
not zero. Independent review has not been performed by this implementation task.

Report IMPLEMENTATION_COMPLETE, OVERFIT_EXECUTED and FIT_TARGET_MET separately.
Even >=80% improvement and <=0.02 m ADE on this set proves neither physical
teacher correctness, unseen-run generalization, launch, safety nor controller
tracking. Stop here; do not automatically collect data or run full training.
