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
