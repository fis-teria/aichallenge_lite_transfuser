# Dataset V3 teacher collection pilot (2026-09-04)

## Outcome

One bounded official-MPC teacher pilot was captured on the AWSIM D1 domain and
audited against the recovery collection Reference. It is a usable partial raw
capture, not a coverage-complete training dataset. The vehicle was returned to
`Grounded` and the teacher MPC and Safety Supervisor processes were stopped
after capture.

The simulator exposed Camera, LiDAR, pose, and vehicle-state streams only on
ROS domain 1. Domains 2--4 did not expose the required sensor set, so this
pilot did not recreate the simulator in a multi-vehicle configuration.

## Accepted partial capture

- Run: `mpc_standard_pilot_005`
- Scenario: `d1_sim_mpc_standard`
- Collection case: `standard_launch_curve`
- Teacher: official MPC on `/nominal_control_cmd`
- Executed command: independent Safety Supervisor on
  `/control/command/control_cmd`
- Safety speed cap: 0.75 m/s
- Duration: 120 s
- Sampled audit states: 1,046 at the configured audit rate
- Raw location:
  `/home/graneple/git/autononous_ai/aichallenge-racingkart/output/teacher_recovery_v3/50dc441/valid_candidates/mpc_standard_pilot_005`
- MCAP SHA-256:
  `0bfd2b1a568de23303b0c5ea52c82fbe942efa4c82eb757df332cc625e39dc76`
- Route Reference SHA-256:
  `93aa7c9e3d5369c71cdfa15b6161b5f7431bdb797f7b99c697c9cafe04621368`

The capture reached the Safety Supervisor's
`front_obstacle_inside_stopping_distance` state and stopped. Treat it as a
partial launch/right-curve/stop/recovery candidate, not as a clean lap.

## Coverage result

The overall result is `FAIL`, as expected for one pilot run. The report is at:

`/home/graneple/git/autononous_ai/aichallenge-racingkart/output/teacher_recovery_v3/50dc441/reports/mpc_standard_pilot_005.coverage.json`

Report SHA-256:
`312a53e01ecd16032d9be8e870f83c94467dfa014a8e32c6ba922f9f811fbc9c`

Gap report SHA-256:
`c33e249228230f9690b3a440bf82eeee87535b034506909fa6073cfe14515ff1`

Observed bucket counts include:

- right curve: 985 samples, 2 episodes, 1 run;
- straight: 61 samples, 1 episode, 1 run;
- launch: 13 samples, 2 episodes, 1 run;
- stop approach: 13 samples, 1 episode, 1 run;
- recovery left: 52 samples, 3 episodes, 1 run;
- recovery right: 43 samples, 3 episodes, 1 run;
- left curve: 0 samples.

All buckets still fail their minimum distinct-run and episode requirements.
The next acquisition must prioritize left curves, additional launch/stop
episodes, both near-offset bands, right-side far offsets, and independent runs.

## Rejected pilots

The following capture-complete bags are operational evidence only and must not
be included in conversion or training:

- `mpc_standard_pilot_001`: official start timed out; vehicle stayed Grounded.
- `mpc_standard_pilot_002`: Safety used wall time while MPC used simulation
  time, so commands were rejected as stale.
- `mpc_standard_pilot_003`: the retained admin Start prevented a new one-shot
  transition.
- `mpc_standard_pilot_004`: MPC control-enable subscription was not remapped to
  `/awsim/control_mode_request_topic`, so the controller calculated a target
  speed while publishing zero speed.

Recorder manifest `status: complete` means that rosbag finalized correctly; it
does not override these semantic rejection decisions.

## Fixed collection contract

`teacher_capture_safety_v3.launch.py` now pins Safety to simulation time and
single final-command authority. `run_official_mpc_teacher_v3.sh` pins the MPC
control-enable and command remappings. Both must be used for subsequent runs.

The Windows repository is the source of truth. This pilot did not sync into the
WSL training checkout because a training process was active, and no Git push
was performed from the experiment host.

## Seeded lateral-offset capture

After the trajectory-authoritative fine-tune completed, the AWSIM executable
was restarted with its built-in deterministic start randomizer:

```text
--start-random=true --start-random-seed=103 \
--start-random-range=0.80,0.00 --start-random-min-separation=0
```

AWSIM logged the enabled seed and the jittered `GoKart1` position. A subsequent
bag-derived comparison (using the first synchronized pose from each capture)
showed that seed 103 was displaced 0.275 m from the fixed-start teacher pilot.
Its signed lateral offset relative to route Reference point 59 increased from
+0.664 m in `mpc_standard_pilot_005` to +0.937 m in this capture. This is a
verified left-far start, rather than a case label inferred from the requested
arguments alone. The earlier live estimate taken before restarting Autoware was
stale and is not used as evidence.

The same one-publisher teacher chain passed recorder preflight and produced a
second bounded capture:

- Run: `mpc_random_left_far_seed103`
- Scenario: `d1_sim_recovery_left_far`
- Collection case: `offset_left_far`
- Duration: 120 s
- MCAP size: 27,015,097 bytes
- MCAP SHA-256:
  `eaa9f68630d3697ab41d6a7eb92c8eb02be4dbc54681cf13bde44e7790549e44`
- Raw location:
  `/home/graneple/git/autononous_ai/aichallenge-racingkart/output/teacher_recovery_v3/50dc441/valid_candidates/mpc_random_left_far_seed103`

The official start service returned success, all requested bag topics were
subscribed, and the recorder finalized with exit code 0. The teacher again
reached `front_obstacle_inside_stopping_distance`; the vehicle was reset to
`Grounded`, and both teacher processes were stopped after collection.

The two-run combined audit remains `FAIL`, which is expected and prevents this
partial set from being promoted directly into training. It now contains 2,087
sampled states. Left-far coverage reached 1,927 samples across two runs and
heading-left coverage reached 1,809 samples across two runs, but both still
miss their five-run and twenty-episode gates. Left-curve coverage remains zero.
The combined report SHA-256 is
`4df7888731541929c091d256a8a07aa146150a951ee58010849d11b3e860c0c5`;
the gap report SHA-256 is
`67f5be220fdc2b285524508b514317a848f6ca3c532bb7de06746c60a4e12548`.

## Automated seeded recovery batch

Three additional 120 s official-MPC runs were captured with deterministic
AWSIM start seeds. Each run used the independent Safety Supervisor, passed the
single-publisher recorder preflight, reached an authoritative one-shot Start,
finalized its rosbag metadata, and was followed by official-Start cancellation
and an admin reset to `Grounded`:

| run | seed | compressed MCAP SHA-256 |
|---|---:|---|
| `mpc_random_seed102_retry2` | 102 | `ba27dd5e53d5940673dce6ecf0cbc3929570d384609535c26e50e3bc46b4b2d2` |
| `mpc_random_seed100` | 100 | `3efe482e6b700541c05a8d2c85bd9e76fc277483e4f23133078401b1d33de1c8` |
| `mpc_random_seed099` | 99 | `2d917529bbaff99f4326c5b45f4930278804b29b837ed31c33336b086c7c9b0a` |

The accepted set now contains five runs and 5,204 sampled audit states. The
combined audit passes `right_curve` (4,844 samples, 5 runs, 10 episodes) and
`stopped` (4,542 samples, 5 runs, 15 episodes). Overall coverage remains
`FAIL`: `left_curve` is still zero, `offset_left_far` has enough samples and
runs but only 9/20 episodes, and the right-offset, heading-right, launch,
stop-approach, straight, and recovery gates remain incomplete. Repeating the
same start region is therefore not an efficient way to close the remaining
gaps.

- Combined report SHA-256:
  `03a20c4bf410d13b17a3a0a190fe515f22c15120825ba5428a6a2d87454cfda2`
- Combined gap report SHA-256:
  `e31b7016bf8aabc5d62e14d4fba1da45576465b0c6c2eaea378b6b41571c9264`
- Remote report:
  `/home/graneple/git/autononous_ai/aichallenge-racingkart/output/teacher_recovery_v3/50dc441/reports/combined_5run_seed099_100_102.coverage.json`

Two interrupted setup attempts (`mpc_random_seed101` and
`mpc_random_seed102`) and one non-grounding seed attempt
(`mpc_random_left_far_seed105`) were isolated under `raw`; they are not part of
the accepted five-run audit.

The collection exposed a cross-domain ROS discovery bug in
`record_dataset_v3.py`: topic discovery bypassed the ROS daemon, but publisher
count discovery did not. The recorder now uses `ros2 topic info --no-daemon`
with an explicit spin time, preventing a domain-0 daemon from falsely
reporting zero publishers for a domain-1 teacher.

## Verified right-far capture

The `0.80 m` randomization range could not produce a right-far initial pose
because the fixed grid pose is already +0.664 m left of Reference point 59.
Seed 109 was therefore rerun with a bounded `1.50 m` lateral range:

```text
--start-random=true --start-random-seed=109 \
--start-random-range=1.50,0.00 --start-random-min-separation=0
```

Before recording, a fresh Autoware initialization reported `WaitStart`,
`Grounded`, and `initialization_ready=true`. Projection of the live pose was
-0.5894 m, and the first synchronized pose read back from the finalized bag
was also -0.5894 m relative to Reference point 59. The run is therefore a
measured right-far case rather than a label inferred from the requested range.

- Run: `mpc_random_right_far_seed109_r150`
- Scenario: `d1_sim_recovery_right_far`
- Duration: 120 s
- Sampled audit states: 1,040
- Recorder result: exit code 0 with finalized metadata
- Compressed MCAP SHA-256:
  `53df5f58089151a7542f06dfbafeb9b0741d34b553c67ff5fc5e8b1d4fdbf2a5`
- Run report SHA-256:
  `bdcabaa9c99cdfd94848dd61c311d7b1766ace8401542194eb06a78ff01b651c`

The six-run combined audit contains 6,244 states and remains `FAIL`.
`offset_right_far` improved from 207 to 337 samples and reached 20 episodes,
but still needs 163 samples to meet its 500-sample gate. `right_curve` and
`stopped` remain `PASS`; `left_curve` remains at zero. This capture is recovery
training evidence only. No retraining or M3 closed-loop rerun has yet used it.

- Combined report SHA-256:
  `b3715b8e6668f69ee99faad2d172d14fe3e8775bc9b7167e77d70ee6528c5eb0`
- Combined gap report SHA-256:
  `7474c5f6529ca21b0fb15f8a7ac4e8dde720d6054e164cd014928c2c93d756d9`
- Remote report:
  `/home/graneple/git/autononous_ai/aichallenge-racingkart/output/teacher_recovery_v3/50dc441/reports/combined_6run_seed109_r150.coverage.json`

### Speed-cap defect found by bag readback

Although these teacher launches configured the Safety Supervisor with
`max_speed_mps=0.75`, synchronized velocity readback showed per-run maxima of
5.81--5.90 m/s. The envelope clamped the outgoing command's speed field, but
the Safety decision did not override positive acceleration after ego speed
crossed the limit. The six runs therefore prove recovery geometry and recorder
operation, but they are not evidence of a 0.75 m/s teacher trial and must not be
used as such.

The first fix made the Safety core return `speed_limit_exceeded` with bounded
steering and `min_accel_mps2` braking whenever measured ego speed was above the
configured maximum. A fresh 30 s Graneple probe at source commit `92b37d6`
confirmed that this code executed (`speed_limit_exceeded` was reported), and
the bag finalized normally, but the result still failed the speed gate:

- Run: `safety_speed_cap_probe_92b37d6`
- Sampled states: 254
- Mean speed: 0.4636 m/s
- P99 speed: 0.8828 m/s
- Maximum speed: 0.8987 m/s
- Acceptance bound: <= 0.85 m/s
- Compressed MCAP SHA-256:
  `d7daf27d5416eca3a845330d0cbc5b05c7ce98ed44ed12f97b7ea0196a82dfd2`

The measured overshoot shows that braking only after crossing 0.75 m/s is too
late for the AWSIM actuation path. Safety therefore now has an explicit
`speed_limit_guard_margin_mps` (default 0.1 m/s). When positive acceleration
is requested at or above `max_speed_mps - speed_limit_guard_margin_mps`, it
returns `speed_limit_guard` with bounded steering and
`min_accel_mps2` braking. Existing deceleration is not replaced by the guard,
and negative or non-finite margins are tested.

A fresh `ef56060` Graneple probe passed the bounded speed check: maximum speed
was 0.7887 m/s, P99 speed was 0.7475 m/s, and the 30 s bag finalized with
recorder exit code zero. Its compressed MCAP SHA-256 was
`03204b3c2ea323f6a6c09ed2173ec8a2a866c430ba5528a1fc29c57314db5e11`.
This verifies the 0.75 m/s Safety cap with the declared 0.10 m/s tolerance; it
does not by itself pass the M3 model closed-loop gate.

## Generated recovery Reference pilot

Commit `e9dd85d` added the clearance-checked recovery Reference generator and
commit `38f5c67` bound collection coverage to the exact official MPC baseline.
The current configuration automatically selected four disjoint cases:

| case | side/offset | base-route interval |
|---|---|---:|
| `right_near_left_curve` | right, 0.35 m | 89.74--111.59 m |
| `left_near_right_curve` | left, 0.35 m | 174.10--196.18 m |
| `right_far_right_curve` | right, 0.55 m | 268.87--289.63 m |
| `left_far_left_curve` | left, 0.55 m | 313.39--334.73 m |

Each case has separate approach, hold, and return-to-baseline intervals. The
generated path passed the configured 1.40 m minimum vehicle-centre clearance
check. Artifact hashes are:

- generated MPC Reference:
  `1e06af3a0da492613ab5af5ae69b74d6bdc23b392e8d2b2ef4bf51cbc9e4229a`;
- phase intervals:
  `b76321e13fbb57ce0d8ad1d3007e82307c3f6b74fb7e2f210acbc3349e2cf827`;
- exact baseline collection Reference:
  `8bfa666213eac167fa648ebee8076109bad44ed97ea60f661f527effa4eba594`.

Two 180 s Safety-bounded official-MPC pilots were retained as operational
evidence but rejected as recovery teacher data. The first used the original
short approach/hold profile. The second used the longer profile above, but its
`right_near_left_curve` hold stayed at +0.0004 m mean lateral offset instead of
the requested -0.35 m. Static inspection confirmed that the MPC loaded a
different path, but the executed vehicle did not follow its lateral excursion.

### Accepted partial Pure Pursuit capture

The same generated Reference was then published as the planning trajectory and
tracked with `simple_pure_pursuit`. Its command was the sole publisher on
`/nominal_control_cmd`; the independent Safety Supervisor was the sole final
publisher on `/control/command/control_cmd`. A 180 s full sensor capture passed
recorder preflight and finalized normally:

- run: `pure_pursuit_reference_pilot_003`;
- teacher: `simple_pure_pursuit_recovery_reference_v3`;
- captured messages: 90,606, including 1,707 Camera images, 3,584 LiDAR scans,
  8,962 poses, and 5,121 velocity reports;
- compressed MCAP size: 199,759,183 bytes;
- compressed MCAP SHA-256:
  `2e0477478bc2710802fb7324c4fc2d3325d4dc77e732ca97f0bda6cc57fb85d0`;
- remote location:
  `/home/graneple/git/autononous_ai/aichallenge-racingkart/output/teacher_recovery_v3/38f5c67/pure_pursuit_reference_pilot_003`.

Bag-derived phase audit for `right_near_left_curve` measured a -0.3884 m hold
mean against the -0.35 m target, with a -0.0405 m mean signed error from the
generated segment. The post-recovery tail crossed back to the baseline side;
its mean was +0.1346 m and its mean absolute offset was 0.1552 m. Maximum speed
was 0.3965 m/s, P99 was 0.3904 m/s, and no velocity sample exceeded 0.85 m/s.
This run is accepted as partial right-near recovery teacher data.

The generic coverage audit still reports `FAIL`, as required for a single run:
1,563 sampled states, 254 right-near samples, 96 right-far samples, and one
right-recovery episode. It must not be treated as a coverage-complete training
set. The remaining acquisition order is left-near, right-far, and left-far,
with multiple independent repetitions of every case. After capture the
official Start was cancelled, AWSIM was reset to `Grounded`, and both temporary
teacher processes were stopped.

## Remaining recovery capture batch

The remaining lateral-recovery cases were collected on Graneple on 2026-09-04.
The initial combined Reference could not be used for the later cases because
the vehicle physically stopped progressing near route distance 135 m even
though the teacher still requested positive acceleration and Safety reported
`normal`. Those two operational attempts are retained under `rejected` and are
not training inputs. Instead, each missing case received a separate early,
clearance-checked Reference so every repetition starts from a fresh initialized
Autoware state and reaches the intended excursion in one bounded run.

The accepted batch contains five independent runs for each of the three missing
cases:

| case | duration/run | requested hold | measured hold mean | post-recovery absolute offset mean | maximum speed |
|---|---:|---:|---:|---:|---:|
| `offset_left_near` | 150 s | +0.35 m | +0.3484 m | 0.1432 m | 0.4069 m/s |
| `offset_right_far` | 150 s | -0.55 m | -0.5536 m | 0.0625 m | 0.4070 m/s |
| `offset_left_far` | 330 s | +0.55 m | +0.5469 m | 0.1260 m | 0.4577 m/s |

All 15 runs passed the batch acceptance policy: complete manifest, one nominal
teacher publisher, recorder exit code zero, bag metadata present, absolute hold
target error no greater than 0.08 m, post-recovery mean absolute offset no
greater than 0.20 m, and no speed sample above 0.85 m/s. The aggregate contains
1,585,831 messages and 3,568,317,904 bytes of compressed MCAP. Required sensor
and command totals include 29,882 Camera images, 62,737 LiDAR scans, 156,859
poses, 89,628 velocity reports, 313,763 nominal commands, and 62,767 final
commands. Every individual run contains all of these required topic classes.

The final left-far Reference required an evidence-driven correction. A 4 m
return profile reached the requested +0.55 m hold but left 0.3730 m of mean
absolute offset after recovery, so that run was rejected. The accepted version
uses a 16 m return profile at base-route distance 79.36--113.36 m. It passed the
same 1.40 m minimum centre-clearance check and has these hashes:

- Reference CSV:
  `85ff0ec5f3dd98e8d059bf241bc5713c84a67f476ef417e971c0e12cfa063fdc`;
- phase intervals:
  `a47ef07113371742a02c1112af5f7831d52ca1150e92c735c03d21a57408c729`.

The batch-wide coverage audit evaluated 27,387 sampled states. Every lateral,
heading, curve, stopped, and recovery bucket passed. In particular,
`offset_left_near` passed with 4,403 samples and 70 episodes,
`offset_right_far` with 2,072 samples and 25 episodes, `offset_left_far` with
5,538 samples and 30 episodes, `recovery_left` with 2,128 samples and 30
episodes, and `recovery_right` with 1,056 samples and 21 episodes. The overall
coverage status remains `FAIL` only because two separate non-recovery targets
remain: `launch` needs another 489 samples and 2 episodes, and `stop_approach`
needs another 572 samples and 17 episodes. This does not invalidate the
completed recovery acquisition, but it must be resolved before claiming the
entire Dataset V3 coverage contract is complete.

The durable remote artifacts are stored at:

- accepted bags and manifests:
  `/home/graneple/git/autononous_ai/aichallenge-racingkart/output/teacher_recovery_v3/38f5c67/recovery_remaining_20260904/accepted`;
- `quality_summary.json`, `coverage.json`, and `coverage.gaps.json`:
  `/home/graneple/git/autononous_ai/aichallenge-racingkart/output/teacher_recovery_v3/38f5c67/recovery_remaining_20260904`;
- case-specific References:
  `/home/graneple/git/autononous_ai/aichallenge-racingkart/output/teacher_recovery_v3/38f5c67/recovery_case_references_20260904`.

The quality-summary, coverage, and gap-report SHA-256 values are respectively
`1fa2e4fd3ea7f0dffd58bb607988a408f1cad8090dbfb9e58d0b7dd057d124d6`,
`7a27b5c2e1bf5063c2c9a9568000869a7af723d16b9ffebf29a579b03bb6f440`,
and `00a953ef9cd5b59b3301643330c59efa448fc939ab6d6b82098f52cc329a2da6`.
All 15 MCAP files passed their stored SHA-256 checks and byte-for-byte comparison
between the runtime and durable copies. Rejected and interrupted probes remain
isolated under `rejected`; none are part of `accepted`.

After the audit, official Start was cancelled, AWSIM was confirmed `Grounded`
at 0.0 m/s, the temporary teacher and Safety processes were absent, both nominal
and final command topics had zero publishers, and the standard official MPC
Reference parameter was restored. No bag, weight, or generated data was added
to Git. The host had 7.3 GiB free after retaining both verified copies, so the
runtime duplicate should only be removed after an explicit retention decision.

## Canonical conversion and mixed-data training

The 15 accepted remaining-case bags were transferred byte-for-byte to WSL at
`/home/thistle/e2e_autonomous/raw/teacher_recovery_v3_20260904/accepted`.
All stored SHA-256 values matched after transfer. Canonical Dataset V3 conversion
produced 15 runs and 23,751 samples at
`/home/thistle/e2e_autonomous/datasets/recovery_20260904_v3`; its manifest
SHA-256 is
`ae4cbd76b0397057638a1a84a43187cc2f69c0e46f88a397eb5e96d0e1568efa`,
and the Dataset V3 audit passed.

The recovery phase view contains 3,372 approach, 2,329 hold, 2,799 recovery,
and 15,251 baseline samples. Hold plus recovery therefore contributes 5,128
direct recovery examples. Maximum/mean source alignment error was
24.999/5.985 ms against the 50 ms limit. All 23,751 samples have a valid
0.75 m/s nominal speed target; future-label invalid fraction was 0.007605 and
LiDAR validity was 0.95348. Two implausible raw yaw-rate values, 1077.06 and
179.46 rad/s, were excluded from anchor eligibility by the explicit 5 rad/s
ego-feature limit. Their runs were assigned to test/validation, not training.

The recovery split uses seed 4904 and places three/one/one runs from each of
`offset_left_near`, `offset_right_far`, and `offset_left_far` into
train/validation/test. Its manifest SHA-256 is
`12f23492bf75ef36359842696390b6db2d975e20f378ed784e5c4a952e9dc563`.
The earlier single `right_near_left_curve` pilot remains a separately retained
partial capture and is not one of these 15 converted runs. It should be added
through a new run-balanced data revision rather than silently mixed as a lone
case.

The streaming hard-link merge with the 11-run normal-lap dataset produced 26
runs and 72,697 samples at
`/home/thistle/e2e_autonomous/datasets/d1log_recovery_mixed_20260904_v3`.
Its manifest SHA-256 is
`181cf909b80589110574859990b0885005b7f9a0bb07cff1c24f38d6b090f388`.
Recovery runs are 32.7% of natural samples, while hold plus recovery phases are
7.1% of effective samples. The mixed Dataset V3 and Behavior View V1 audits
passed. The preserved normal and recovery run-level splits contain 16/5/5
train/validation/test runs with no leakage.

The one-step CUDA smoke run completed at
`/home/thistle/e2e_autonomous/runs/recovery_20260904_smoke_daeae6c`.
The full waypoint-authoritative command was then run under the WSL training
lock (paths abbreviated only for line length):

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  -m aic_transfuser_lite.cli train \
  --config configs/models/trajectory_authoritative_finetune_v3.yaml \
  --dataset-root /home/thistle/e2e_autonomous/datasets/d1log_recovery_mixed_20260904_v3 \
  --split-manifest /home/thistle/e2e_autonomous/datasets/d1log_recovery_mixed_20260904_split_manifest.json \
  --view-config configs/data/view_temporal_v3.yaml \
  --behavior-view /home/thistle/e2e_autonomous/datasets/d1log_recovery_mixed_20260904_behavior_v1 \
  --output /home/thistle/e2e_autonomous/runs/d1log_recovery_mixed_20260904_waypoint_e02b804 \
  --epochs 5 --batch-size 2 --device cuda --checkpoint-every-steps 500 \
  --resume --resume-initialization-checkpoint \
  /home/thistle/e2e_autonomous/runs/m3_trajectory_authoritative_finetune_v3_19a0748/last.pt
```

The run completed exactly 14,110 optimizer steps. Epoch 4 was selected with
trajectory ADE 0.129715 m and speed-profile MAE 0.111101 m/s over 198,382 valid
validation waypoints. Epoch 5 was not promoted (0.129976 m and 0.109913 m/s).
The selected checkpoint contains 219 finite tensors and has SHA-256
`9dea8c47f7b446c10661fb38090a377457b639e762ffe6cfe80ed061df0b6d19`.
The runtime artifact SHA-256 is
`948fe1c3810023e0aef7166bc2463da7024013edf9b0caf20945e60b0a1fbab4`.

An exact same-split comparison against the initialization checkpoint measured
ADE 0.198042 -> 0.129715 m (34.50% improvement) and speed MAE
0.224224 -> 0.111101 m/s (50.45% improvement). The offline regression gate
therefore passes. This result does not imply an M3 closed-loop pass; the three
independent Graneple trials and their failure mode are recorded in
`docs/v3_m3_limited_odd.md`.
