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
