# MPPI recovery controller

`control_method=mppi` uses this adapter as the sole final control publisher.
Ordinary CMA PP commands are forwarded unchanged. During a stuck-recovery
episode, the adapter publishes MPC recovery control and gear commands.

The adapter imports MPC's `StuckRecovery`, `RecoveryExecutionState`, directional
wall assessment, steering selection, race-start latch and reverse publication
semantics. Recovery constants match MPC; configurable values are read from
`multi_purpose_mpc_ros/config/config.yaml` through `mpc_config_path`.

Input geometry is the MPPI global reference and physical occupancy map. Their
left/right corridor bounds supply MPC's existing clearance checks. Forward
alignment uses MPC's base-reference lookahead branch; its MPC avoidance planner
is not started. The unmodified reference speed supplies forward intent even if
MPPI rejects all moving candidates and commands zero speed.

The adapter pauses MPPI and CMA through opt-in `set_recovery_active` services.
MPPI clears retained execution, rejects pre-reset worker batches and resets warm
starts. CMA clears tracking history and rejects trajectories stamped before its
resume. The adapter resumes forwarding only after both reset replies and a fresh
CMA command, excluding commands already queued before handback.

The existing AWSIM Ready/Start latch enables recovery detection; the remapped
`input/stop_request` disables control. Fresh odometry is required for recovery
commands. Missing/stale input or an incomplete reset produces a brake command.
Normal tracking retains CMA's input timeout behavior.

With `simulation=false`, the adapter subscribes to the real control-mode and
gear reports instead of AWSIM state. Unknown/MANUAL mode blocks forwarding and
clears recovery state. On each AUTONOMOUS entry it requests DRIVE and waits for a
new DRIVE report, then permits normal tracking. Recovery remains disarmed until
a forwarded forward command is followed by measured forward motion. It reuses
the existing 1.0 m/s minimum forward command and 0.20 m/s stopped-speed threshold,
with the normal input timeout. MANUAL clears this latch and all recovery timers;
the reference node's lap and route selection are unaffected. Real recovery motion
also waits for its requested gear to be reported. A car that never starts moving
requires manual intervention; it cannot trigger automatic start-line recovery.

Private inputs/outputs and services are wired by `control/mppi.launch.xml`.
Other controller launch profiles leave the recovery services disabled.

When a rear vehicle is within 3 m, recovery uses short strokes within the same
reverse/stop/forward cycle. Each stroke is at most 0.6 m and 0.5 m/s; available
travel is shortened using the physical occupancy map and vehicle body geometry.
The speed budget includes 0.20 s of pending propulsion and braking at 0.8 m/s².
Reverse-to-drive still requires measured speed below 0.05 m/s for 0.20 s.
Reverse and forward use opposite steering to align with the local road tangent
across a cycle; ordinary recovery retains its lookahead steering.
Each short stroke holds zero target speed for 0.70 s while steering changes;
AWSIM bag measurements showed about 0.60 s for a full left/right reversal.
The adapter measures heading/lateral improvement before counting a cycle as
unsuccessful. These short-motion settings live in `RecoveryConfig`; ordinary
MPC callers retain their existing recovery defaults.

V2X supplies opponent positions without heading, so clearance encloses each
opponent body in a circle. The motion preview includes both a bicycle arc and
the initial straight response during steering delay. This is a sampled physical
map check, not a guarantee of clearance under localization error or every
contact configuration. Reference heading is derived from point coordinates,
matching MPPI's path direction rather than the CSV quaternion convention.
# Asynchronous reference geometry

Wall-distance construction runs in one spawned worker process. The ROS node
keeps command forwarding and recovery authority; it never waits for a geometry
build in a subscription or timer callback. Until the current reference is ready,
normal commands continue to forward and recovery does not use a previous route.

Completed references are cached by map generation and reference samples (including
speed), with up to eight entries. A→B→A switches reuse A. At most one build runs;
subsequent requests coalesce to the latest reference. A completed older route can
warm the cache, but a result from an older wall map is discarded. Identical map
messages with new timestamps do not invalidate the cache. Worker shutdown joins
at node destruction, outside the control loop. `[MPPI_REFERENCE_CACHE]` reports
build counts, cache hits, worker PID and build duration.
