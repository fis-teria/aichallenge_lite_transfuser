# V3 M3 0.75 m/s Limited ODD

M3 validates the trajectory-authoritative path in an obstacle-free AWSIM run.
It does not authorize a higher speed cap and does not claim a full lap.

## Required launch state

- AWSIM is reset to `Grounded` before each independent launch trial.
- Autoware initialization reports `true`, Gear is Drive, and Control Mode is
  Autonomous.
- The one-shot admin Start is followed by an accepted
  `/autostart/official_start` service call.
- `/overtake/race_armed=true` is retained while vehicle control is authorized.
- `/nominal_control_cmd` and `/control/command/control_cmd` each have exactly
  one publisher and one intended downstream subscriber.

Do not record or echo either control topic during the trial: an additional
subscriber intentionally makes the strict routing preflight fail closed.
Command speed, acceleration, controller state, and preflight results are already
included in `/plan_diagnostics`.

## Evidence recording

Record only non-control evidence:

```bash
ros2 bag record -o /artifacts/m3_trial \
  /clock /awsim/state /awsim/status \
  /autostart/initialization_ready /overtake/race_armed \
  /vehicle/status/velocity_status /vehicle/status/steering_status \
  /plan_diagnostics /safety_reason /localization/kinematic_state
```

The installed AWSIM binary defines
`/awsim/ground_truth/on_collision` as `std_msgs/msg/Bool`, publishing `true` from
`OnCollisionEnter`. Start a typed observer before Start and record both its
matched-publisher status and any messages. A zero-byte observer log without a
matched publisher is **unverified**, not proof of zero collisions.

Analyze the bag in the official ROS container:

```bash
export PYTHONPATH=/work/src:${PYTHONPATH}
/usr/bin/python3 /work/tools/analyze_m3_rosbag.py \
  /artifacts/m3_trial --speed-cap-mps 0.75
```

The analyzer reports launch latency, measured maximum/final speed, commanded
speed and acceleration bounds, preflight ratio, controller faults, Safety
states, displacement, signed yaw-rate coverage, and collision evidence state.
It also compares 1.0 s future odometry in the observation-time ego frame with
both the raw fixed-time Trajectory Head and the speed-capped, arc-length-retimed
Executable Reference. These are separate metrics: M3 controller tracking is
judged against the reference that was actually executed; the raw result remains
visible as Head-consistency evidence.

## M3 gates

Run at least three independent stopped launches. For each trial:

- measured longitudinal speed reaches 0.10 m/s within 3.0 s after
  `race_armed=true`;
- measured speed remains at or below 0.85 m/s (0.75 m/s cap plus 0.10 m/s
  measurement/control tolerance);
- no `launch_response_missing` or other controller fault occurs;
- Safety remains `normal` or reports the bounded `speed_limit_guard` while
  enforcing the configured cap; no timeout, future-timestamp, obstacle, NaN,
  or other fail-safe reason occurs during accepted nominal motion;
- the run contains measurable straight and gentle-turn samples;
- a matched collision observer reports no `true` event.

Left and right curve coverage may be accumulated across the independent trials.
M4 remains blocked until repeated launch, tracking, curve coverage, speed cap,
and collision evidence all pass. ROS/AWSIM outcomes not present in an analyzed
artifact must remain reported as unverified.

The checked-in M3 profile uses a 1.0 m minimum arc-length lookahead in addition
to the time preview. The two pre-change Graneple trials used approximately
0.37 m lookahead at 0.75 m/s and each stopped after only 6.6--6.7 m. They are
diagnostic evidence, not M3 passes. Any post-change result must use a new commit
and artifact identity.

## Graneple evidence before bounded origin normalization

Commit `7a856818718878156347bab158b14d988af5f1df` was run on the designated
Graneple environment. These artifacts are diagnostic and do not pass M3:

- `m3_lookahead_trial1_7a85681`: 37.39 s, 24.36 m displacement, launch in
  1.92 s, maximum measured speed 0.7345 m/s, Safety `normal` 745/745, and
  Executable Reference tracking p95 0.0713 m. It contained straight samples
  only.
- `m3_curve_trial2_7a85681`: 92.23 s, 29.72 m displacement, launch in 2.06 s,
  maximum measured speed 0.7484 m/s, right-turn samples 93, and Executable
  Reference tracking p95 0.0804 m for matched references. It then produced
  `initial_waypoint_not_forward` for 427 frames and stopped.

Inspection of that stop episode found a leading point only 4--9 mm behind ego,
followed by forward points at 47--71 mm and endpoints 0.65--1.16 m ahead. The
Executable Reference therefore performs the bounded 0.05 m leading-noise trim
documented in `v3_m0_plan_contract_and_executable_reference.md`; larger or
nonrecoverable reverse paths remain fail-closed.

The installed AWSIM graph exposed no publisher for
`/awsim/ground_truth/on_collision` in either run. Consequently collision absence
is **unverified**, even though no collision message was recorded. A new immutable
commit and independent trials are required after origin normalization. M4 is
still blocked.

## Bounded origin normalization result

Commit `da0ccfd` passed the full WSL suite (`534 passed, 34 warnings`) and the
official Graneple ROS package build. The 1.0 m lookahead trial
`m3_origin_trim_trial1_da0ccfd` proved that the normalization removed the former
Plan rejection (`stop_required_count=0`) but did not pass M3:

- duration 112.20 s; launch 2.05 s; maximum speed 0.7322 m/s;
- displacement 24.89 m; right-turn samples 104; left-turn samples 0;
- Executable Reference tracking p95 0.0679 m over matched predictions;
- `front_obstacle_inside_stopping_distance` 623 samples and final speed
  0.0034 m/s;
- collision topic publisher absent, so collision state unverified.

A single-variable 1.25 m lookahead A/B artifact, SHA-256
`1aed4e2052be7d617c3779b7d74d31ef35b057424383e1d5d2525963354f3268`,
was also run with the same source, model, Safety settings, and 0.75 m/s cap.
`m3_lookahead1p25_trial2_da0ccfd` stopped after 13.37 m and reported 801
front-obstacle samples. It was worse than 1.0 m and was rejected; the runtime
was restored to the checked-in 1.0 m setting.

The official debug raceline is not an inference input. A read-only comparison
showed the closed-loop localization path departing by up to 3.78 m from that
line before the obstacle stop. Because the controller's short-horizon measured
tracking error remained small, this is treated as a Trajectory/Speed model
closed-loop generalization failure rather than a Safety or coordinate-sign
failure. M3 remains failed, so M4 speed escalation and lap testing have not
started.

## Model recovery training gate

`configs/models/trajectory_authoritative_finetune_v3.yaml` defines a separate,
non-destructive M3 recovery artifact. It initializes from the retained causal
checkpoint, makes Trajectory and Speed the dominant losses, and adds a
differentiable SI-unit Huber loss between each predicted trajectory segment's
geometric speed and its paired Speed Head value. Control and behavior heads stay
load-compatible at low auxiliary weight but remain non-authoritative at runtime.

The trainer now consumes `gradient_accumulation_steps`; one logged global step is
one optimizer step over that many micro-batches. The model config bytes are part
of the experiment contract hash, so changing loss weights, accumulation, or
model settings cannot silently resume an incompatible run.

For a full run (without the smoke-only `--max-batches` option), the trainer also
loads the run-separated `validation` split. It stops each training chunk at the
exact epoch boundary, records SI-unit `trajectory_ade_m` and
`speed_profile_mae_mps`, writes immutable `epoch_NNN.pt` snapshots, and promotes
`best_trajectory.pt` by lowest trajectory ADE with speed MAE as the tie-breaker.
`validation_history.json` binds those decisions to the experiment identity and
epoch size. Resume rejects mismatched or ahead-of-checkpoint history. The
runtime artifact points to the promoted checkpoint; `last.pt` remains the exact
resume state. A `--max-batches` smoke run deliberately skips validation and
promotion and therefore cannot be used as M3 evidence.

Run in WSL under the shared training lock and write a new output directory:

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  -m aic_transfuser_lite.cli train \
  --config configs/models/trajectory_authoritative_finetune_v3.yaml \
  --dataset-root /home/thistle/e2e_autonomous/datasets/d1log_0902_all_v3 \
  --split-manifest /home/thistle/e2e_autonomous/datasets/d1log_0902_all_v3/split_manifest.json \
  --view-config configs/data/view_temporal_v3.yaml \
  --behavior-view /home/thistle/e2e_autonomous/datasets/d1log_0902_all_behavior_v1 \
  --output /home/thistle/e2e_autonomous/runs/m3_trajectory_authoritative_finetune_v3 \
  --epochs 5 --batch-size 2 --device cuda \
  --init-checkpoint /home/thistle/e2e_autonomous/runs/d1log_0902_all_full_control_v3_causal_1b7d298/last.pt
```

Training completion alone does not pass M3. The candidate must first pass
held-out Trajectory ADE, Speed MAE, and Plan-consistency comparison without
regression, then repeat the bounded Graneple launch/curve/tracking trials.

## Recovery fine-tune live evidence

Commit `44670a4f` added exact sampler-epoch validation and resumed the recovery
run from optimizer step 200. Epoch 2 (`step=3868`) replaced epoch 1 as the
current validation best:

| epoch | trajectory ADE [m] | speed profile MAE [m/s] | promoted |
|---:|---:|---:|:---:|
| 1 | 0.236507 | 0.160619 | yes |
| 2 | **0.198744** | **0.155845** | yes |

The epoch-2 checkpoint SHA-256 was
`0bb6c6143e404de72d54092a45a5ca916596329471ab4f99ce23781d67c03294`.
It was copied to Graneple with matching source and a generated runtime artifact
whose SHA-256 was
`95212bb706eb9cf6cc499796444d6b9ea07e8bb58f9b1af6ddeac1b8af063e46`.

The first attempted trial accidentally omitted Docker GPU access. Its 4.3 Hz
inference caused `nominal_command_timeout` for 2477 Safety samples and a 71.6 s
reported launch latency. That artifact is retained as infrastructure-negative
evidence but is not used to judge the model. The container was replaced with an
otherwise identical `--gpus all` instance before the valid trial.

The valid GPU-backed trial `m3_epoch2_trial2_gpu_44670a4` produced:

- launch latency 1.972 s and maximum measured speed 0.7448 m/s;
- 15.04 m displacement with 105 right-turn samples and no left-turn samples;
- Executable Reference tracking p95 0.0788 m;
- Safety `normal` 630 times but
  `front_obstacle_inside_stopping_distance` 2397 times;
- final speed 0.0 m/s, with no controller fault and no Plan rejection;
- collision topic still absent, so collision state remains unverified.

The analyzer JSON SHA-256 was
`33ca2e1a64c2bb27230c86c300f4903c70e803824dcb9067baa3d7af2e59bf4c`;
the ROS bag database SHA-256 was
`a36b175b0e5f37563af5aa981ad3c824ab768af2e384c041da49ea64ad04661d`.
M3 therefore remains failed: the new checkpoint launches and tracks the
executed reference, but its closed-loop path reaches the front-obstacle stop
after only 15 m. Training continues; M4 and M5 remain blocked.

Epoch 3 improved the validation-primary metric again: trajectory ADE
`0.193207 m` (epoch 2: `0.198744 m`), while speed MAE regressed to
`0.165734 m/s` (epoch 2: `0.155845 m/s`). Because trajectory ADE is the declared
primary metric, `epoch_003.pt` was promoted. Its SHA-256 was
`1201609dabdb596b8e99b95c7ad25f5ecfebdece8c7b3d6d0c2d258213c67fb0`;
the per-epoch runtime artifact SHA-256 was
`75e9f9c672a72b6d06a6684a9aeee713e6b33f0710af1c2a45b0b2cc0dece425`.

The GPU-backed `m3_epoch3_trial1_gpu_44670a4` result was materially better but
still did not pass M3:

- displacement 36.24 m; reconstructed path length 36.68 m;
- maximum measured speed 0.8041 m/s, below the 0.85 m/s tolerance limit;
- left/right/straight sample counts 89/18/4180;
- Safety `normal` 2996/3001 samples and no Plan rejection or controller fault;
- Executable Reference tracking p95 0.1338 m;
- reported launch latency 3.036 s, 0.036 s above the 3.0 s gate;
- collision topic absent, so collision state remains unverified.

The vehicle ended 2.32 m from the official debug raceline and reached a maximum
2.60 m nearest-raceline separation. The prior checkpoint reached a larger
3.78 m separation, so the recovery training is moving in the intended
direction, but the final near-zero vehicle speed under otherwise-normal Safety
is consistent with physical blocking after leaving the drivable corridor. This
is an inference from localization/raceline evidence; collision remains
unconfirmed because AWSIM did not publish the configured collision topic.

Evidence SHA-256 values: analyzer JSON
`79be6f082169d31ddae5a09f92b2fcd43dfeea2d2129a2c31808118dbfd7e400`,
path/raceline JSON
`d65accda861c9910e4f844671c2ab56a19516f7734a8bfb3ba32770f088b2240`,
and ROS bag database
`6f70a1e4308af76c6834bff76bb9e8bd868ba22ab7969412b21138b264c985e5`.
M4 and M5 remain blocked while later training epochs are evaluated.

The five-epoch run completed at global step 9670. Epoch 4 was not promoted
(trajectory ADE `0.218578 m`, speed MAE `0.176984 m/s`), and epoch 5 was not
promoted (trajectory ADE `0.196940 m`, speed MAE `0.153472 m/s`). The final
selected checkpoint therefore remains `epoch_003.pt`.

After training released the WSL worktree lock, the initial causal checkpoint
was evaluated on the exact same held-out validation split. Its checkpoint
SHA-256 was
`6e8fc01b55ba438f299731a01fd1e35ef7f853c2399f5514454eefab30f93d0e`.
It produced trajectory ADE `0.237094 m` and speed MAE `0.166722 m/s` over
8,895 samples. The selected epoch 3 checkpoint improves trajectory ADE by
18.51% and speed MAE by 0.59%, so it passes the offline regression gate against
the initial causal baseline. This offline result does not override the failed
M3 closed-loop gate above; M4 and M5 remain blocked.

## Predictive speed-guard rerun

Bag readback of the recovery teacher captures revealed that clamping only the
command speed field did not constrain the AWSIM vehicle. Commit `92b37d6`
added measured-speed braking, but its 30 s Graneple probe still reached
0.8987 m/s because braking began only after the 0.75 m/s limit was crossed.

Commit `ef56060` adds a 0.10 m/s predictive guard margin. A fresh 30 s probe
passed the declared 0.85 m/s tolerance with maximum speed 0.7887 m/s and P99
speed 0.7475 m/s. The recorder completed with exit code zero; its compressed
MCAP SHA-256 was
`03204b3c2ea323f6a6c09ed2173ec8a2a866c430ba5528a1fc29c57314db5e11`.

The subsequent independent model trial `m3_speed_guard_trial2_ef56060` did not
pass M3 even though the speed cap did:

- duration 112.27 s; launch latency 3.141 s (`FAIL`, limit 3.0 s);
- maximum speed 0.7458 m/s (`PASS`);
- displacement 23.72 m and reconstructed path length 24.06 m;
- left/right/straight samples 36/16/3,155;
- Safety reasons: normal 2,112, speed-limit guard 131, plus one
  `lidar_future_timestamp` and one `nominal_command_timeout`;
- Executable Reference tracking p95 0.3028 m over 553/1,034 matched
  predictions;
- official-raceline separation p95 2.35 m, maximum 2.60 m, and final 1.47 m;
- final speed 0.0038 m/s; collision publisher absent, so collision state is
  still unverified.

Evidence SHA-256 values: analyzer JSON
`57015b296f28ee553a5b6df5ec9d3639cb178af8a4901f30efd5e8742bf7d782`,
path/raceline JSON
`c6134f431395fb8f42a3f2fc8a439386dd850c19a841899aee6d9303fcb92224`,
ROS bag database
`3b69ab735c479f9349c65e032ebccac3856737ca77b0527bfc1ed5a2deb4c254`,
and collision-topology log
`8d184377b1ebda602d21287946940042c24d2a3efa1e4a247c7de9fabdbf8bcc`.
The repeated 2.60 m maximum raceline separation and physical stop confirm that
the remaining primary blocker is closed-loop trajectory generalization, not
the speed-limit implementation. M4 and M5 remain blocked.

## Mixed lap/recovery epoch-4 closed-loop result

The mixed-data five-epoch run documented in
`docs/v3_teacher_collection_pilot_20260904.md` selected epoch 4. The Graneple
deployment used source commit `2115c958102eb07e8d24efb2888015cdabaa862e`,
checkpoint SHA-256
`9dea8c47f7b446c10661fb38090a377457b639e762ffe6cfe80ed061df0b6d19`,
runtime artifact SHA-256
`948fe1c3810023e0aef7166bc2463da7024013edf9b0caf20945e60b0a1fbab4`,
and runtime parameter SHA-256
`4f6d86d95d72ea151c86479e48fe156ff6e4fa1f235518d54d26d2ff70b827b7`.
The immutable source archive SHA-256 was
`798bbf5940c17430a361d92683af351c59499f587f6c5e81aaef290c7e7a229f`.

The official `aichallenge-2025-dev:latest` container passed 86 focused V3
runtime/controller/M3 tests and built `aic_e2e_runtime` successfully. A full
archive-only pytest run reported 577 passed, one skipped, and five failures;
all five failures were missing legacy v0/e3a checkpoints intentionally excluded
from the Git archive. The Windows/WSL source checkout had already passed the
complete 583-test suite with those protected ignored assets present.

Three independent runtime starts were evaluated. Each trial reset AWSIM,
restarted Autoware, stopped the competing standard `hybrid_control_mux_node`
publisher and teacher-only `wall_recovery_planner_node` subscriber, verified
one publisher and one subscriber on both nominal and final control routing,
then issued exactly one official Start. Only non-control evidence topics were
recorded. Each bounded interval was approximately 112.3--112.7 s.

| metric | trial 1 | trial 2 | trial 3 |
|---|---:|---:|---:|
| maximum commanded speed [m/s] | 0.112449 | 0.112757 | 0.112821 |
| maximum measured speed [m/s] | 0.037637 | 0.037526 | 0.037531 |
| displacement [m] | 2.3440 | 2.3383 | 2.3415 |
| executable-reference tracking p95 [m] | 0.04748 | 0.04740 | 0.04754 |
| raw-trajectory tracking p95 [m] | 0.29335 | 0.29367 | 0.29342 |
| left/right turn samples | 0/0 | 0/0 | 0/0 |
| launch pass | no | no | no |

The speed cap passed in all trials and Safety was `normal` for a mean 99.896%
of recorded samples. Preflight was ready for a mean 99.904%. Nevertheless,
every recorded longitudinal-controller state was `blocked`. The executable
reference never reached the configured 0.2 m/s launch threshold after the
pre-arm `race_not_armed` block; its maximum requested speed was only about
0.113 m/s. Thus the vehicle never established a launch, travelled only 2.341 m
on average, and reached no curve. The strong offline held-out metrics therefore
did not generalize to the initial closed-loop observation distribution.

The collision topic was absent in all three trials, with topology-log SHA-256
`8d184377b1ebda602d21287946940042c24d2a3efa1e4a247c7de9fabdbf8bcc`.
Collision-clear status remains unverified. Analyzer JSON SHA-256 values were
`fd77b60cce9ca48d1a4cf3e6943a8591a892e036e8e837905d09c387387a54f7`,
`9cdca326ca3484e383eaa5815fe55f04b3b5251d841550cc68215e9c0f32b6a3`,
and `cf6d8306176ac4fa7f2d2519389efef6e004e17382acb8caca3c19a9f3d8514c`.
The corresponding bag database SHA-256 values were
`964c833b3fede54d92c757f6d8209fb591c7567e875bd76ad9f0af71cc274787`,
`f3bd62a49da13284e9f8b99f642ae93d8c8837c58b7f20c8a61dd832fede5437`,
and `8cae28a16693b36768a8b71166edf1f769e2c773d5570fe5978a1c1305c44de0`.
The aggregate evidence is
`/home/graneple/v3_runtime/m3_mixed_2115c95/evidence/m3_three_trial_summary.json`
with SHA-256
`5c738bfb479aa40cbc06c89496b238e9eaaad3e390b46f48db03dbb7d23f7068`.

M3 remains failed: offline trajectory/speed regression passes, but launch and
curve-coverage gates fail and collision evidence is unavailable. The dedicated
runtime container was stopped after evidence capture. Autoware was restarted,
the standard mux and wall-recovery nodes were restored, and AWSIM was confirmed
`Grounded`.
