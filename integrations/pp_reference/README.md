# Existing Pure Pursuit reference routing

Stage 1: existing reference trajectory -> existing simple_pure_pursuit.
Stage 2 (NOT implemented): V4-updated path -> validated adapter -> PP.

Scoped remote changes: Makefile CONTROL_METHOD default becomes pure_pursuit;
reference.launch.xml retains the existing overtake/wall planner for that branch,
routes PP tracking feedback to it, disables MPC horizon use, and requires fresh
planner override. Other control branches and their settings remain unchanged.
The planner's existing inactive-MPC logic is not bypassed or retuned here.
Only one control_method branch is selected; no MPC solver is launched for PP.

Input: /planning/scenario_planning/trajectory from simple_trajectory_generator,
existing traj_mincurv_manual.csv; pose /localization/kinematic_state.
The reference generator speed cap is min(requested cap, 20/3.6 m/s) for PP.
PP consumes that generation's speed; use_external_target_vel=false.
Important: the EXISTING buildExecutionProfile uses one speed equal to the minimum
of the configured cap and every source-point speed, not a per-corner speed plan.
A zero source speed rejects that profile rather than preserving timed stop points.
This task does not redesign that generator or claim curve/stop-label fidelity.
No V4 model inference or MPC-reference substitution.

Reuse PP longitudinal proportional control and its existing stop path (-1.5 m/s2),
not MPC acceleration/filter settings. Existing PP steering gain is 1.54 (MPC's
1.639 must not be reused in a PP observer). Stale-input/planner guards remain.
This does not prove actuator braking or a successful lap. The PP proportional
acceleration helper itself is not an acceleration clamp; live limits and final
consumer must be checked before driving. No new controller implementation.

Non-driving tests: Windows commit -> CheckOnly/sync -> WSL worktree lock:
`python -m pytest -q tests/test_pp_reference_switch.py`.
Use installed ROS launch argument expansion in network-none Docker to verify
the applied launch can be parsed, without starting nodes or AWSIM.

Future finite-run command (NOT authorization to execute unattended):
`V4_SHADOW_ENABLED=false make dev CONTROL_METHOD=pure_pursuit CAPTURE=false ROSBAG=false AWSIM_LAPS=1`.
Existing MPC run11 runner/observer must NOT be reused unchanged: it hardcodes
MPC node identity, solver failure counters and MPC steering gain.
Before a new live run, bind its observer to /simple_pure_pursuit_node, verify
current consumer/stop/authority and carry forward previous budgets. No new
driving budget has been requested or consumed for this routing-only task.

Applied to the remote dirty checkout with git apply --check then focused patch;
unrelated changes preserved. Windows implementation b086707: WSL lock tests
3 passed / 0.03s. Installed reference.launch.xml SHA256 matches the local copy:
2fe413aeefce6737d3057d247afb13349c75f716f1fc1225d6ceda57253fdd72.
Network-none ROS launch --show-args and actual substitution evaluation passed
(10 m/s requested -> 20/3.6, 2 m/s requested -> 2). Existing PP executable found.
Initial verification script imported a module as a function; corrected its
Humble import path and reran successfully. No ROS nodes or AWSIM started.
Actual PP binary behavior, control topic exclusivity, fresh planner overrides,
speed/steering/clamp/stop behavior and one-lap completion remain NOT_RUN.

## Later live results (2026-09-09)

The preceding paragraph records the original preflight, not current live status.
Run12 drove 91.93 m but requested acceleration above the simulator's 3 m/s²
input contract and remained near 5 km/h. Apply `speed20_gain.patch` AFTER
`racingkart.patch`; the current XML includes that additional PP-only gain 0.5.
Run13 reached 17.39 km/h, maximum observed acceleration request 2.7622 m/s²,
and 277.55 m before its host time limit. No Finish; no braking-stop acceptance.
Target remained 19.887 km/h; actual 20 km/h holding is NOT achieved.
Full records: `docs/v4_normal_dev_start_split.md`, local `tmp/pp_speed20_lap_13`.
AWSIM was not modified, and V4/MPC were off in both PP trials.
