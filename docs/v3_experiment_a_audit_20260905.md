# V3 path-only experiment A audit (2026-09-05)

## 1. Training start decision

Experiment A was allowed to start after all declared prerequisites passed:

- Windows source of truth: `E:\workspace\e2e_lite_transfuser`, branch
  `codex/windows-wsl-training-sync`.
- Requested baseline commit `19e9619e995c5b151ac139cae2f7c18201ce907f`
  exists and the audited HEAD is its descendant. Training used source commit
  `7a06d37e1a71740040079b42041fb41f5878020e`; the post-run result-label fix is
  commit `7ac377ec359083dcda1d2ebb854d7d65112b04a3`.
- WSL training checkout was clean and detached at the same source commit before
  training. No WSL or SSH push was performed.
- Dataset, split, checkpoint, temporal input parity, core tests, A0, smoke run,
  GPU, and output capacity were all verified. A0's poor launch score was not a
  start blocker because improving it was the experiment hypothesis.

The run completed all five epochs and 13,685 optimizer steps. It ended with
`GATE_FAILED`, not with a training crash. `best_validation`,
`research_candidate_checkpoint`, `selected_checkpoint`, and
`runtime_deployment_checkpoint` are null; no runtime artifact was written.

## 2. Prior-audit findings rechecked

Confirmed and fixed:

- Runtime supplied one current sample while training used Camera/LiDAR 4 frames
  and ego/command 10 steps. Runtime now keeps ordered timestamped history,
  left-pads warm-up entries with masks, resets at gap/reversal boundaries, and
  never inserts anchor/current-or-future command data into past-only history.
- ROS and training preprocessing could drift. Both now use the same image and
  LiDAR normalization contracts, with explicit 750-beam angle/range/frame
  validation and sensor timing features.
- Incomplete future prefixes could be classified as complete stationary 1.5 s
  futures. A filter decision now requires all 15 future points; incomplete
  examples remain censored/unknown.
- Evaluation could inherit the training filter and hide failure outcomes.
  Teacher-quality, unfiltered-valid, and stopped-commanded cohorts are now
  separate and retain stable sample/run identities.
- The old launch proxy could pass on maximum X alone. Offline launch evaluation
  now calls the same executable-reference builder and longitudinal controller
  request logic as runtime and records rejection/cap/trim reasons.
- YAML trajectory regression settings were not sufficient evidence of an
  enforced promotion gate. Selection now requires both launch and trajectory
  non-regression gates; smoke and failed epochs cannot create a runtime best.

Not duplicated or not applicable:

- No original TransFuser migration, head reinitialization, loss ablation,
  geometry loss, multi-candidate model, resplit, or fabricated planned
  Reference was added.
- Model speed remains trained for compatibility/diagnostics, but path-only
  runtime ignores it and starts from an external 0.75 m/s target before caps.
- Stop probability is not connected in the selected runtime profile. No dummy
  probability or behavior-derived substitute was introduced.

## 3. Code changes

The baseline-to-audit change set is intentionally concentrated in these areas:

- `runtime/input_history_v3.py` and ROS `inference_node_v3.py`: stateful
  4/4/10/10 history, causal ordering, masks, warm-up, reset, common preprocessing,
  and LiDAR geometry validation.
- `data/dataset_view_v3.py`: false ego-padding masks corrected, command padding
  zeroed, canonical sensor timing populated, full-horizon filter requirement,
  and censored accounting.
- `evaluation/launch_replay_v3.py`: runtime YAML loading, shared executable
  Reference/controller replay, external speed authority, cap/rejection/coverage
  diagnostics, and fail-closed invalid measured speed handling.
- `training/train_v3.py` and `cli.py`: baseline-before-training, separate
  filtered/unfiltered validation, run-equal/worst-run metrics, fixed cohort
  identities, regression gate, promotion types, resolved config, timestamps,
  optimizer/parameter provenance, and smoke isolation.
- `tools/audit_path_only_dataset_v3.py`, `evaluate_path_only_v3.py`, and
  `compare_path_only_v3.py`: reproducible accounting, detailed A0/candidate
  evaluation, and paired run bootstrap.
- Post-run reporting fix: standalone evaluation now names the launch-only result
  `launch_gate_pass`; comparison computes trajectory non-regression separately
  and combines both into `candidate_screening_gate_pass`.

## 4. Verification

Pre-training checks:

- Focused history/filter/runtime/promotion tests: 54 passed.
- Launch replay regression tests after reverse-speed fix: 5 passed.
- Full WSL suite before training: 612 passed, 40 warnings.
- Smoke run: two micro-batches, one optimizer step, `SMOKE_COMPLETE`, no OOM or
  NaN, and no promoted/runtime artifact. Its weights and optimizer were not
  reused.
- Golden parity builds one recorded segment independently through the training
  loader and runtime history builder, then compares all input tensors, masks,
  and eval-mode model outputs with `rtol=0`, `atol=0`.

Post-run focused gate tests: 11 passed. The final full WSL suite passed with
615 tests and 40 warnings in 67.51 s; there were no failures or skips.

No official ROS/AWSIM runtime was launched in this audit. ROS wiring was checked
by source-level and pure-Python tests; that is not an official-environment pass.

## 5. Preflight and A0 baseline

Environment and identities:

- Ubuntu/WSL, Python 3.10.12, PyTorch 2.7.1+cu128, CUDA 12.8,
  NVIDIA RTX 4080 with 16,376 MiB VRAM.
- Dataset: `d1log_recovery_mixed_20260904_v3`, 26 runs, 72,697 rows.
  Manifest identity: `181cf909b80589110574859990b0885005b7f9a0bb07cff1c24f38d6b090f388`.
- Split manifest file SHA-256:
  `7d0e433dbd032ad695227051573e7d8d17072fa4ea3b4e28f4c44f56fde27b4f`;
  16/5/5 train/validation/test runs. Run ID and source leakage counts are zero.
  Repeated sessions may still be correlated; run-level separation is not proof
  of scene independence.
- Initial checkpoint SHA-256:
  `9dea8c47f7b446c10661fb38090a377457b639e762ffe6cfe80ed061df0b6d19`.
  Exact load: 219 keys, 12,227,191/12,227,191 parameter elements and
  10,135/10,135 buffer elements, with no missing, unexpected, unmapped, or
  shape-mismatched entry.

Dataset accounting:

| split | raw | invalid ego | zero future | unfiltered valid | complete contradiction | censored | teacher quality |
|---|---:|---:|---:|---:|---:|---:|---:|
| train | 45,190 | 33 | 15 | 45,142 | 1,363 | 162 | 43,779 |
| validation | 13,641 | 10 | 2 | 13,629 | 450 | 37 | 13,179 |
| test | 13,866 | 17 | 5 | 13,844 | 444 | 27 | 13,400 |

The 82-row discrepancy in the supplied totals is exactly 60 invalid-current-ego
plus 22 zero-valid-future rows. Supplied exclusions 1,462/476/462 additionally
counted 99/26/18 censored stationary prefixes; those are retained as unknown
because a complete 1.5 s non-motion outcome was not observed.

A0 detailed validation metrics on cohort
`059ee4dc903047853deca8f4fd458d048c27d2b5c29356ea0663b84a98eba588`:

| metric | A0 |
|---|---:|
| waypoint-weighted ADE | 0.133046 m |
| frame-weighted ADE | 0.134172 m |
| run-equal ADE | 0.096714 m |
| worst-run ADE | 0.189009 m |
| FDE | 0.331374 m |
| speed MAE | 0.114822 m/s |
| heading error | 0.354458 rad |
| curvature error | 16.5444 1/m |
| frame p90 / p95 ADE | 0.290545 / 0.394261 m |

A0 stopped-commanded replay used 530 anchors from five runs and an estimated
41 episodes: 450 complete contradictory outcomes, 43 observed-motion outcomes,
and 37 censored outcomes. Only 2/530 (0.38%) were launch-ready. Mean path length
was 0.14825 m, endpoint displacement 0.02638 m, endpoint forward displacement
0.00816 m, and lookahead 0.10134 m. Initial waypoint forward rate was 1.13%,
trim rate 12.83%, Reference rejection 86.04%, and controller requested speed
was mean/median/p95 0.1780/0.1041/0.5988 m/s among accepted references. This is
offline requested speed, not measured vehicle speed.

## 6. Experiment A configuration and execution

Initialization used the verified A0 checkpoint with a fresh AdamW optimizer;
`--resume` and `--freeze-migrated` were not used. All 12,227,191 model
parameters were trainable. Precision was float32, scheduler and augmentation
were null, weight decay 0.01, learning rate 1e-4, seed 42, micro-batch 2, and
gradient accumulation 8 (effective batch 16).

Loss weights remained trajectory 2.0, speed profile 1.0, plan consistency 0.25,
current control 0.02, control sequence 0.02, behavior 0.02, and behavior side
0.01. The 43,779 post-fix training anchors produced 21,890 micro-batches and
2,737 optimizer steps per epoch.

The run started at `2026-09-04T21:20:12.677637Z`, completed at
`2026-09-05T02:07:57.653040Z`, and took 17,264.98 s. No OOM or NaN occurred.

| epoch | step | ADE m | run-equal ADE m | worst run m | speed MAE m/s | ready / 530 | launch | regression | promoted |
|---:|---:|---:|---:|---:|---:|---:|:---:|:---:|:---:|
| 1 | 2,737 | 0.132324 | 0.091605 | 0.191438 | 0.108698 | 4 | fail | pass | no |
| 2 | 5,474 | 0.141244 | 0.102818 | 0.199146 | 0.108569 | 8 | fail | fail | no |
| 3 | 8,211 | 0.136326 | 0.094200 | 0.197185 | 0.103470 | 506 | pass | fail | no |
| 4 | 10,948 | 0.135267 | 0.089671 | 0.200045 | 0.115798 | 4 | fail | fail | no |
| 5 | 13,685 | 0.134472 | 0.093621 | 0.194594 | 0.106457 | 4 | fail | fail | no |

Every epoch snapshot and `last.pt` is retained. No `best_trajectory.pt` or
runtime artifact was created. Epoch 3 SHA-256
`c7f42f0dd4ae70bd67a0c894168af9dc69a1a5cd5dfe5c05eed572b3b98a1305`
is used below only for diagnosis because it most directly tests the launch
hypothesis; it is not a selected or promoted checkpoint.

## 7. A0 versus diagnostic epoch 3

Both evaluations have the same cohort identity. Detailed epoch-3 results:

| metric | A0 | epoch 3 | interpretation |
|---|---:|---:|---|
| waypoint ADE | 0.133046 | 0.136326 | 2.47% worse; regression gate fails |
| run-equal ADE | 0.096714 | 0.094200 | improved, but not the fixed primary gate |
| worst-run ADE | 0.189009 | 0.197185 | worse |
| FDE | 0.331374 | 0.353609 | worse |
| speed MAE | 0.114822 | 0.104339 | diagnostic improvement |
| heading error rad | 0.354458 | 0.286586 | improved |
| curvature error 1/m | 16.5444 | 12.2724 | improved, but short paths remain unstable |
| launch ready | 2/530 (0.38%) | 506/530 (95.47%) | launch-only gate passes |
| path length mean m | 0.14825 | 0.26708 | longer, still far below 1 m |
| endpoint forward mean m | 0.00816 | 0.19811 | improved |
| lookahead mean m | 0.10134 | 0.22057 | improved, not 1 m coverage |
| Reference rejection | 86.04% | 1.89% | improved |
| trim rate | 12.83% | 96.79% | severe dependency on bounded trim |
| mean max abs curvature 1/m | 198.20 | 320.79 | worse/noisy |

Run-wise waypoint ADE improved on all three recovery runs (left-far
0.04084 to 0.02809, left-near 0.04130 to 0.03011, right-far 0.04312 to
0.03224) but regressed on both normal runs (0.18901 to 0.19719 and 0.16931 to
0.18337). Right-near is absent from validation. Straight is not explicitly
annotated and remains in geometry `unknown`; it is not inferred as a measured
straight slice.

The labeled approach, baseline, hold, recovery, left-curve, right-curve, left,
right, and none slices improved in ADE. Geometry/phase/side `unknown` worsened,
and these unknown samples dominate the validation frame count. Behavior
FORWARD_AVOID and FORWARD_RETURN improved; FORWARD_NORMAL, FORWARD_FOLLOW,
RECOVERY, and behavior `unknown` worsened.

Paired bootstrap resampled five runs, not frames. For per-frame ADE the run-equal
candidate-minus-A0 delta was +0.000885 m with 95% CI
[-0.006214, +0.009067], so its direction is unresolved. FDE delta was
+0.023914 m with 95% CI [+0.011479, +0.036277], indicating degradation in this
five-run cohort. Speed-MAE delta was +0.000656 m/s with 95% CI
[-0.011033, +0.012346], also unresolved. Five runs give weak uncertainty
resolution and correlated repeated sessions may further reduce independence.

Final screening is fail: launch readiness passes, but the fixed
waypoint-weighted ADE non-regression gate fails. Unknown/censored samples were
not removed to inflate readiness. No test-split performance was used for
selection or threshold tuning.

## 8. Runtime and closed-loop decision

Do not deploy any experiment-A checkpoint. The trainer correctly produced no
research or runtime candidate and no runtime artifact. Epoch 3's offline
controller request is not measured motion; its high ready rate does not prove a
launch. The path still has only 0.221 m mean lookahead, 96.8% trimming, and very
large short-path curvature. Stop probability is unconnected, closed-loop is not
evaluated, and collision observability is unverified. M3 is not achieved.

For a later offline-passing candidate, perform at least three independent
stopped-launch trials first, recording command/requested/measured speed, raw and
executable path, all caps, controller state, Safety reason, and matched
collision evidence. Only proceed to gentle left/right curves after every launch
meets latency, speed, controller, Safety, and collision-observation gates in
`v3_m3_limited_odd.md`.

## 9. Remaining issues

- Planned recovery Reference is absent from canonical Dataset V3. Measured
  future is an outcome label and cannot establish the intended path during a
  failed recovery.
- The launch improvement is unstable across epochs: ready counts were
  4, 8, 506, 4, and 4. It is coupled to near-universal initial-point trimming
  and insufficient lookahead.
- Normal-run/global trajectory quality regressed when recovery slices improved;
  the five-run validation set is too small for a precise confidence interval.
- Stop-probability integration, closed-loop distribution shift, curve tracking,
  and collision publisher evidence remain unverified.
- Temporal overlap drift was not reported as a separate metric; logged-future
  evaluation remains offline replay, never a rollout.

## 10. One next experiment

Run one stability experiment with the same initial checkpoint, split, filter,
losses, seed, batch, and five-epoch limit, changing only AdamW learning rate
from 1e-4 to 1e-5. The hypothesis is that the epoch-3 launch-path transition can
be approached without the observed ADE regression and epoch-to-epoch collapse.
Keep the same dual gate and do not promote unless both pass. This does not prove
that transfer from original TransFuser is required, and it does not change or
ablate speed/plan-consistency losses.

## Artifact locations

- Preflight audit:
  `/home/thistle/e2e_autonomous/runs/path_only_experiment_a_76c5979/preflight/dataset_audit.json`
- A0:
  `/home/thistle/e2e_autonomous/runs/path_only_experiment_a_7a06d37/a0`
- Smoke:
  `/home/thistle/e2e_autonomous/runs/path_only_experiment_a_7a06d37/smoke`
- Experiment A:
  `/home/thistle/e2e_autonomous/runs/path_only_experiment_a_7a06d37/experiment_a`
- Diagnostic epoch-3 evaluation:
  `/home/thistle/e2e_autonomous/runs/path_only_experiment_a_7a06d37/candidate_epoch3_eval`
- Corrected paired comparison:
  `/home/thistle/e2e_autonomous/runs/path_only_experiment_a_7a06d37/comparison_epoch3_v2`

Dataset, rosbag, and checkpoint binaries remain outside Git.
