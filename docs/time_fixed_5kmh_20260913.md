# TimePath B0: Pure Pursuit + fixed 5 km/h AWSIM trial

## Active request and state

The user requested Pure Pursuit with a fixed 5 km/h target. The model remains
TimePathV1 B0, command history OFF, epoch 10, checkpoint SHA256
`e857db4b67d3d9f6a77c2865cfc1fa53e9903e7a1d2d8a371407fbe009a1a44f`.
Previous evidence: [origin guard trial](time_origin_guard_20260913.md), trial04.
Its 0.25 m/s commanded target reached 0.153 m/s measured maximum. No full lap
or lane-tracking accuracy claim follows from that test.

First false: the existing controller derives target speed from time-point
spacing and caps it at 0.25 m/s; it cannot command the requested 5 km/h.
Owner: the time trial longitudinal target policy, carried through controller
startup configuration and recorded replay. No model or actuator retuning.

## Necessary change and acceptance

- Add an explicit opt-in `fixed_5kmh` speed policy: 5 / 3.6 m/s normal target,
  6 / 3.6 m/s overspeed stop. Keep the default source-speed 0.25 m/s trial
  compatible, including replay of previous recordings.
- Pure Pursuit follows the same raw 30 XY points in metres. Preserve observation
  time, age alignment, map/base_link/rear_axle transforms, geometry checks,
  acceleration +/-1 m/s^2 and steering/rate limits. Teacher inputs stay separate.
- Fixed speed is an experimental external longitudinal policy, not a learned
  speed/stop decision. Stationary/unresolved, stale, infeasible and insufficient
  stopping-reference predictions cannot authorize fixed-speed motion.
- Existing stopping-distance checks use measured speed: at 5 km/h the nominal
  reference requirement is 1.759 m and scan corridor clearance is 2.059 m
  (reaction 0.5 s, braking 1 m/s^2, respective margins 0.1/0.4 m). These checks
  remain active; the forward scan corridor is not a swept-footprint proof.
- Configuration must agree with implemented speed and duration limits; log the
  selected policy and config SHA so WSL can replay the actual calculations.
- Verify fixed target independently of model point spacing, old mode behavior,
  overspeed, invalid config, short/stationary path and speed-dependent clearance.
- Bounded execution: `graneple@192.168.3.10`, same selected AWSIM scene, normal
  RViz, one new run ID, 10 simulation seconds drive / 30 wall seconds drive,
  120 wall seconds total including shutdown. Stop then confirm measured zero.
  Preserve historical projects; stop only this owned run. No automatic retries.
- Success for this task is correctly commanded 5 km/h on admitted model paths,
  recorded measured response and classified safety rejections, not an invented
  assertion that the measured speed is exactly 5 km/h or a full lap was passed.

The change is necessary at the target calculation, not at the trained weights.
An opt-in policy is smaller and reversible by selecting the unchanged old
configuration. Tests and recorded replay isolate its effect; the host trial
will establish the measured response. Current state: opt-in policy, matching
JSON/ROS launch, policy-aware replay, speed-response metrics and synthetic ROS
overspeed brake check implemented. WSL and runtime verification pending.

## Commands

Windows commit, then `./tools/sync_to_wsl.ps1 -CheckOnly` and sync. In native WSL:

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
```

Deployment-specific commands and results will be recorded after verification.
