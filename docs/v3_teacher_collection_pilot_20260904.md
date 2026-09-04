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

AWSIM logged the enabled seed and the jittered `GoKart1` position. The first
localized pose was displaced 0.778 m from the earlier fixed-start M3 pose; its
projection onto the route Reference left normal was +0.752 m. This is a
verified left-far start, rather than a case label inferred from the requested
arguments alone.

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
