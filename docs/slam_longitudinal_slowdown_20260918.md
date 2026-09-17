# SLAM front-obstacle longitudinal limiter

Use the existing E2E model/path/tracking steering with an explicit speed-only
limiter. No lateral avoidance, route regeneration or obstacle-driven steering.
Configuration: `configs/control/time_path_slam_slowdown.json`; ordinary
`time_path_dev.json` retains its previous behavior.

The detector uses LiDAR plus wheel-speed/steering odometry, no GNSS/IMU/global
map pose. Current occupied returns overlapping the forward E2E path supply a
distance from the observed base position. The controller subtracts 1.8 m for the
front body, 1 m stopping margin and travel during observation age. It derives a
speed cap assuming 1 m/s² braking and a further 0.5 s response allowance:
`v_cap = sqrt(0.5² + 2 * max(0, remaining_distance_m)) - 0.5`.
These are explicit simulation assumptions; this is not a measured guarantee.
Speeds below 0.05 m/s become a stop. Reductions apply immediately and release
after a restriction is limited to 0.5 m/s². Ordinary E2E target changes on a
continuously clear road pass unchanged. Existing target speed and acceleration
can only decrease.
While closing on an obstacle, acceleration also anticipates the falling cap
(`d(v_cap)/dt = -v / sqrt(0.25 + 2 * remaining_distance_m)`) rather than waiting
for speed error alone. This removed a 0.1 m margin overrun in the ideal-plant
unit test; it still requires actual AWSIM validation.
The selected steering command is retained even while the limiter requests zero.

Input admission checks the publisher, run ID, local frame, valid path, original
scan time (350 ms maximum), local receipt time (350 ms) and plan observation time
(500 ms). Missing, malformed, stale or unmatched observations request braking;
they do not declare the road clear. Recovery is possible on fresh valid input.
Model/vehicle/clock faults retain the existing controller's separate handling.
The usual finite-run supervisor still ends a prolonged standstill.

```bash
# After committing Windows source and safe native-WSL synchronization:
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
# ROS_DOMAIN_ID=93, --network none, official prepared ROS/Cartographer image:
python3 tools/check_slam_slowdown_ros.py --output <new-output-directory>
# Normal finite AWSIM lap, with speed limiting and recorded RViz:
timeout --signal=TERM --kill-after=10s 710s python3 "$DEPLOYMENT/source/tools/run_time_path_awsim_trial.py" \
  --deployment "$DEPLOYMENT" --run-id codex-time-slam-slowdown01 --display :0 \
  --config configs/control/time_path_slam_slowdown.json --ros-launch \
  --max-speed-kmh 20 --corner-max-speed-kmh 10 --npcs 0 --pp-vehicles 0 \
  --slam-obstacles --record-video
# Physical three-box barrier, finish after measured obstacle-induced stop:
# Add --slam-stop-box-test and use a new run ID.
```

The box fixture is scenario setup only. Coordinates never enter inference,
SLAM perception or control. It preserves the normal ego spawn and uses the
reference s=44 m object placement from the existing scenario tool. Object identity,
moving-vehicle speed, true-pose accuracy and collision-free operation are not
implied by a speed-cap activation. Phantom LiDAR returns and walls can also
cause conservative slowing.

Runtime source: `879657928c42eb9d243093146a2b0ea6946d22b1`.
Remote deployment: `/home/graneple/e2e_autonomous/time_slam_slowdown_20260918_r4`.

- Official ROS/Cartographer image, 251 installed Python sources byte-matched:
  **PASS**. Production controller plus real Cartographer, synthetic sensor/path
  inputs, shadow commands only. Observed 379 guarded commands, 77 slowdown,
  58 close-obstacle stop, 78 release and 20 stale-input braking commands.
  The production steering command matched the pre-limiter command in all 379;
  no vehicle command publisher was created. This is not AWSIM driving evidence.
- The first ROS test used a curved path that itself reached the corridor wall.
  The wall correctly remained an obstacle after removing the box. The corrected
  test uses a gentle curve entirely within the corridor to isolate release.

Native WSL full suite at `a9d5939efe1312af986fe624129abc3c6a3a2b8c`:
**3103 passed, 4 skipped**, 84 warnings, 113.95 s. This descendant includes
concurrent vendor-package changes; the limiter runtime is unchanged since
`8796579`. Existing skips are unavailable OSQP, Draft2020 validator, jsonschema,
and the optional official tiny-LiDAR package. Evidence is under
`docs/evidence/slam_slowdown_20260918/`.

## AWSIM results and remaining limits

Both final runs used real model inference, LiDAR + speed/steering odometry,
Cartographer, the production controller, zero NPCs and zero PP vehicles.

| Run suffix | Scenario | Maximum measured speed | Limiter-evaluated commands | Obstacle-limited commands | Invalid-input braking commands | Outcome |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| `box03` | Three physical boxes | 1.3062 m/s | 75 | 0 | 2 | `STOPPED_NO_LAP / PROGRESS_STALLED` |
| `lap03` | Normal course | 0.2556 m/s | 48 | 0 | 7 | `STOPPED_NO_LAP / PROGRESS_STALLED` |

Run IDs are `codex-time-slam-slowdown-<suffix>`. In `box03`, the existing
`STEERING_FEASIBLE_LOOKAHEAD_MISSING` check requested braking before an
obstacle-induced SLAM cap was observed. In `lap03`, existing checks reported
`TIME_PATH_INITIAL_DIRECTION` and `STEERING_FEASIBLE_LOOKAHEAD_MISSING`;
seven limiter evaluations also rejected stale/unmatched input. The run ended
after prolonged standstill. These observations do not isolate all causes of
the poor launch behavior. They do not demonstrate successful obstacle braking,
collision avoidance, the assumed stopping margin, or normal-course completion.
Do not interpret an ordinary stopped vehicle as a SLAM obstacle-stop success.

For all 123 limiter-evaluated AWSIM commands, issued steering exactly matched
the command selected before the limiter, and target speed/acceleration never
increased. The observed ROS subscription graph contained no GNSS, IMU or
global-localization input. This proves integration boundaries, not closed-loop
obstacle performance. No lateral avoidance was added.

Earlier box fixtures and an interrupted normal-course run are not acceptance
evidence. They preceded the final clear-road release-state fix. That fix and
the finite test stop-reason registration have regression coverage.

Artifacts on the Windows source checkout:

- `artifacts/slam_slowdown_20260918/box03/`: raw control/perception logs, audit,
  AWSIM and RViz recordings.
- `artifacts/slam_slowdown_20260918/lap03/`: corresponding normal-course evidence.
- `artifacts/slam_slowdown_20260918/smoke/final_summary.json`: synthetic-input
  ROS smoke result. The model is not part of this smoke test.

Small audit summaries, hashes, installed-source verification and test results
are committed under `docs/evidence/slam_slowdown_20260918/`; raw logs/videos are
excluded. The final normal-course recordings decode successfully and the RViz
frame shows the local occupancy, LiDAR and E2E path. The ordinary development
Makefile hash is unchanged, the original RViz configuration is restored, and
no trial containers remain running.

The dedicated configuration is ready for further validation. It is not enabled
in the ordinary development entrypoint. Remaining work is to resolve the
observed launch/path-admission and input-timing behavior, then demonstrate a
moving approach, obstacle-induced slowdown/stop and clearance recovery in
AWSIM without substituting a classical avoidance path for the E2E output.
