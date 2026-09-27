# DINOv3 + LiDAR development baseline and comparison contract

Date: 2026-09-27 JST

GitHub issue: [#2](https://github.com/fis-teria/aichallenge_lite_transfuser/issues/2)

Parent issue: [#1](https://github.com/fis-teria/aichallenge_lite_transfuser/issues/1)

## 1. Scope and ownership

This document is the only repository deliverable for Issue #2. It records the
handoff baseline, reusable assets, minimum data contract, comparison groups,
and bounded pilot budgets. It does not change model, Dataset, ROS, controller,
or Safety Supervisor behavior.

Issue #2 owns only this document. Existing staged, unstaged, deleted, and
untracked files belong to other work and must not be reset, stashed, cleaned,
bulk-staged, or included in an Issue #2 commit.

## 2. Handoff baseline

The read baseline is the Windows canonical checkout at:

| Item | Observed value |
| --- | --- |
| Checkout | `E:\workspace\e2e_lite_transfuser` |
| Branch | `codex/windows-wsl-training-sync` |
| Handoff commit | `ed8192294c1093beaf7a6f7840e1fb1b30e2d83b` |
| Remote `main` | `e27e3caae00d721f22e750a06fe32e9eccbec193` |
| Remote work branch | `d77b47727c9a160eb011596ddc5c1a2d7033338d` |
| Relation to remote work branch | local branch is 66 commits ahead |
| Existing dirty entries | 2185 deleted, 17 modified, 1 added, 21 untracked |

The remote values were read with `git ls-remote`, not inferred from stale
remote-tracking refs. The dirty tree is the preserved starting state. The WSL
checkout was separately observed at commit
`a9d5939efe1312af986fe624129abc3c6a3a2b8c`; therefore it must not be treated as
synchronized with the Windows source until the normal committed sync preflight
succeeds. No sync, checkout, training, or AWSIM run was performed for Issue #2.

## 3. Reusable assets

### 3.1 Deployed TimePath reference (comparison reference only)

`docs/model_distribution/current_time_path.json` identifies
`timepath_launch_balanced_epoch03_20260916`:

- checkpoint filename: `epoch_03.pt` (runtime name `command_off_best.pt`)
- checkpoint SHA-256:
  `1fe12ca066791d3eb8907c6921120e2cfbb38925c422d5be54940f4acbdd120a`
- source commit recorded by the checkpoint identity:
  `7b19a1bab61bb7d30bc1cb0f8fc44b9d1c506136`
- model input history: 4 Camera frames, 4 LiDAR scans, 10 ego entries;
  command history is disabled in the distributed configuration
- model output: `[B, 30, 2]` XY waypoints in metres at 0.1 s intervals
- controller config: `configs/control/time_path_dev.json`
- ROS launch: `time_path_awsim.launch.py`

The weight is external and was not found as a matching `epoch_03.pt` in the
inspected WSL `runs/` tree. Its registered distribution record and hash are
reusable references, but weight availability must be checked again before a
comparison run. The one bounded AWSIM lap recorded in the distribution file is
historical evidence, not a new validation or a general success-rate result.

TimePath's `interval_speed` is chord length divided by 0.1 s. It is neither a
learned target-speed head nor a stop probability. The retained TimePath model
must therefore remain a separate reference and must not be included as a
same-head DINO ablation arm.

### 3.2 V1 ResNet18 + LiDAR reference

The WSL file below was verified read-only:

| Item | Value |
| --- | --- |
| Checkpoint | `runs/transfuser_lite_v1_static_dataset_v2_exclusive_dropout_p020_seed42_50ep/best_ade.pt` |
| SHA-256 | `1b82e33aa676ccc433a66781658ba9a919d88de34df6c0bc6948738e130dbb84` |
| Config family | `configs/transfuser_lite_v1_static.yaml` plus the embedded resolved config |
| Outputs | waypoints `[B, 6, 2]` m and target speed `[B, 1]` m/s |
| Stop head | disabled; no trained stop probability |
| Runtime | `transfuser_lite_v1.launch.py` and `runtime.v1.param.yaml` |

The embedded checkpoint config is authoritative for loading. The frozen source
config differs in its mutually exclusive full-modality dropout settings, and
the original training commit is not proven by its run manifest. Preserve both
limitations in later comparison reports.

The existing independent Safety Supervisor remains the only publisher of the
final command in the V1 launch. Future policy work must preserve timeout,
finite-value, stopping-distance, clamp, and model-stop checks and must not
bypass this authority.

### 3.3 Dataset V2 and split

The following WSL artifacts were verified without regeneration:

| Artifact | SHA-256 / observed result |
| --- | --- |
| `metadata.yaml` | `fcb4465e96baa697301cf5af07c4613c1b9090fd99bc7f6f4f2b510ef71cbcbb` |
| `train_index.csv` | `d64ccd2840f165808286ae0933b575f84b7c04dd0845436a1430eb37eea03554` / 1342 rows |
| `val_index.csv` | `b642739e09cd0688fae518e298c06ca3584c60606bcc6e8401c332ddd19042bc` / 447 rows |
| `test_index.csv` | `9db937339bfe05504979036e855c0475fcf2935b24067daab71170ef40e73b27` / 447 rows |

The five runs all belong to scenario `normal_course_center_mpc`. Runs 01, 04,
and 05 are train, run 03 is validation, and run 02 is test. The split unit is
`run_id`; frame-random splitting is forbidden. Metadata reports 10 Hz sampling,
native 750-beam LiDAR, 18.73 ms maximum per-run Camera/LiDAR p95 skew, no
pose/velocity missing samples, and all Dataset V2 gates passing.

## 4. Common input, target, output, and time contract

### 4.1 Policy inputs available at observation time `t`

| Field | Initial contract | Unit / frame | Availability rule |
| --- | --- | --- | --- |
| Camera | `[B,Tc,3,H,W]`, `Tc` configurable; initial history 4 | normalized RGB | capture at or before freeze time only |
| 2D LiDAR | `[B,Tl,2,750]`, `Tl` configurable; initial history 4 | range in m plus validity, native beam order, `lidar` frame | nearest causal scan within configured tolerance |
| Ego history | `[B,Te,E]`, initial `Te=10` | measured speed m/s; any added yaw rate/steering must be named and scaled | measured state only, no future state |
| Applied-control history | `[B,Ta,A]` with mask, initial `Ta=10` | steering rad, speed m/s, acceleration m/s2 | permitted only when it is causal and represents the actually applied/final command |

The exact field set and scale are versioned in the Dataset/cache manifest.
GNSS/global pose, collision state, future observation, future action, teacher
command, reward, and debug fields never enter the policy forward path.

`capture_clock=sim` and availability/receipt time are separate. A sample is
eligible only when each policy input was available before the observation
freeze. Command issue time, application time, and resulting observation time
must not be treated as interchangeable.

### 4.2 Training-only targets

| Target | Shape | Definition | Missing-value rule |
| --- | --- | --- | --- |
| Future waypoints | `[B,K,2]` | measured future pose transformed into ego frame at `t`; x forward, y left, metres | boolean `[B,K]` mask; never synthesize an unobserved future |
| Target speed | `[B,1]` | measured longitudinal speed at a configured future offset; initial offset 0.5 s | boolean `[B,1]` mask |
| Stop | `[B,1]` | explicit intentional/unavoidable-stop annotation under a versioned rule | boolean `[B,1]` mask; low speed, bag end, or stack alone is not positive |
| Future observations/features | horizon-dependent, train-only | future Camera/LiDAR/ego targets for the auxiliary branch | per-modality, per-horizon masks |
| Future applied actions | `[B,Kf,A]`, train-only | action actually applied during the teacher future interval | per-step mask; never enter policy input |

For the common IL heads, the initial waypoint times remain 0.5, 1.0, 1.5,
2.0, 2.5, and 3.0 s so the common output is `waypoints [B,6,2]`,
`target_speed [B,1]`, and `stop_probability [B,1]`. Speed is m/s and stop
probability is finite in `[0,1]`. Any later horizon change must be a new
versioned comparison, not a silent config edit.

## 5. Data sufficiency and targeted collection

| Condition | Reusable evidence | Decision |
| --- | --- | --- |
| Normal driving | five `normal_course_center_mpc` runs | sufficient to start Dataset/DINO plumbing and a small IL smoke |
| Obstacle avoidance | no labelled representative run in Dataset V2 | collect a small fixed obstacle set before claiming comparative ability |
| Unavoidable stop | stop annotation absent | explicit stop events and negative controls are required before training/evaluating the stop head |
| Recovery | `recovery_flag` is empty in Dataset V2 | collect labelled off-line/perturbed starts; do not infer from low speed |
| High speed | Dataset V2 is one normal-course source and does not establish a high-speed stratum | collect/version a small high-speed set before making high-speed claims |

This is a targeted deficiency list, not a requirement to re-audit or visualize
every frame. Issue #3 may proceed using the normal-driving data and masks.
Issue #5 may implement and test the stop head with synthetic unit fixtures, but
must not claim a trained stop capability until explicit labels exist.

## 6. Comparison groups

| Group | Role | Required outputs |
| --- | --- | --- |
| R | retained TimePath checkpoint | historical/reference only; XY path contract remains unchanged |
| A | ResNet18 + LiDAR IL with common heads | waypoint, target speed, stop probability |
| B | frozen DINOv3 ViT-S/16 + LiDAR IL with the same common heads | same as A |
| C | group B plus train-only future-prediction auxiliary loss | same policy outputs as B; auxiliary branch absent from inference artifact |

A/B/C use the same versioned run split, samples, labels and masks,
augmentation policy, common-head initialization, IL losses, sampler order,
seed, update count, validation cadence, and controller/Safety settings. B/C
also share policy initialization; the only intended C difference is the
auxiliary branch and loss. Record unavoidable random-number-consumption or
encoder-preprocessing differences explicitly.

## 7. Bounded pilots and evaluation contract

No pilot starts until its code/config is committed on Windows and synchronized
under the documented WSL lock.

- Dataset/cache smoke (#3): at most 100 batches or 15 wall-clock minutes,
  whichever comes first; no training.
- Encoder and common-head smoke (#4/#5): at most 100 forward/backward steps or
  15 minutes per configuration.
- First comparable IL pilot (#8): seed 42, at most 500 optimizer updates or 30
  wall-clock minutes per A/B/C arm, whichever comes first. Do not extend only a
  favorable arm.
- First AWSIM smoke (#9): one episode per representative scenario, maximum
  10 simulation minutes and 12 wall-clock minutes per episode. A later success
  rate needs a separately recorded trial count.

Offline comparison reports waypoint ADE/FDE in m, target-speed MAE in m/s,
stop AUROC/AUPRC plus threshold precision/recall only where both labelled
classes exist, valid target counts, loss components, elapsed time, and peak
VRAM. Missing stop labels produce `not_evaluable`, never a zero error.

Closed-loop comparison fixes scenario, initial state, sensor rates, controller,
Safety Supervisor, speed limits, and intervention logging. It records
completion/collision/off-track, lap or route time, speed distribution, stopping
position, recovery, control smoothness, latency/deadline misses, and Safety or
external-controller interventions. A success caused by an external controller
is not attributed wholly to the learned policy.

The RTX 5060 Laptop target is initially 10 Hz, not a promised result. Measure
batch 1 with the contracted image size, LiDAR shape, and history after at least
100 warm-up calls and at least 1000 timed samples. Record preprocessing, host to
device transfer, model, and postprocessing separately and together, with
p50/p95/p99/max and VRAM peak. The 10 Hz candidate passes only if the complete
fresh-observation pipeline has p99 at or below 100 ms and deadline misses at or
below 1%; repeated inference on a stale observation does not count as sensor
throughput. Timeout or non-finite output still delegates to Safety Supervisor.

## 8. Verification commands and result

Commands used for this documentation-only issue:

```powershell
git status --short --branch
git rev-parse HEAD
git ls-remote origin refs/heads/main refs/heads/codex/windows-wsl-training-sync
gh issue view 1 --repo fis-teria/aichallenge_lite_transfuser
gh issue view 2 --repo fis-teria/aichallenge_lite_transfuser
```

Read-only WSL checks used the canonical path and verified the V1 checkpoint and
Dataset V2 hashes shown above. No new executable logic was added, so pytest was
not rerun, as allowed by the parent Issue's documentation-only completion rule.
No checkpoint, dataset, rosbag, or secret was added to Git.

## 9. Next issue gate and unresolved items

Issues #3 and #4 may start from this contract; they can be implemented in
parallel only if their owned files do not overlap. Issue #5 remains dependent
on both. Before #3 completes, it must establish temporal batch/mask fields and
cache identity without weakening the causal boundary. Before #5 can claim a
trained stop head, explicit stop labels and negative controls must exist.

Still unresolved and intentionally not hidden:

- the distributed TimePath weight was not present in the inspected WSL runs;
- the TimePath distribution record pins split/teacher hashes, but the matching
  manifests were not found in the inspected Dataset V2 tree;
- Dataset V2 has only the normal-course scenario and no explicit stop,
  collision, off-track, or recovery labels;
- RTX 5060 Laptop latency/VRAM and all new-model AWSIM behavior are unmeasured;
- WSL is not at the Windows handoff commit, so no training or Linux validation
  may start until a later committed sync passes the prescribed preflight.
