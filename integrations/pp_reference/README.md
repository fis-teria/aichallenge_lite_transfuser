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
PP consumes that generation's speed, including lower curve/stop values;
use_external_target_vel=false. No V4 model inference or MPC-reference substitution.

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
