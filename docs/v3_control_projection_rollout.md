# V3 control projection and rollout consistency

V3-020 adds ROS-independent, fail-closed projection and consistency gates. It
does not add a control publisher or grant model authority.

## Bounded projection contract

`project_control_sequence` accepts finite raw `[H,2]` values ordered as
steering-rate and jerk. It applies bounded `tanh` decoding and integrates a
physical `[H,3]` sequence ordered as steering (rad), speed (m/s), and
acceleration (m/s^2). Steering angle/rate, non-negative speed, acceleration,
and asymmetric jerk are bounded at every step.

Every limit must be finite and physically ordered. Positive steering, rate,
speed, and time-step magnitudes, negative/positive acceleration limits, and
negative/positive jerk limits are mandatory. The caller must also provide a
non-empty authoritative limit source. Missing, zero, unmeasured, or explicitly
non-authoritative limits reject projection; they are never interpreted as
unlimited.

The request carries the positive source-observation stamp, current time,
maximum observation age, command lifetime, and future-stamp tolerance. Future,
stale, or already-expired requests fail before projection. The result records
both source stamp and valid-until time.

## Actuator and bicycle rollout

`rollout_actuator_bicycle` consumes only a projected sequence and a versioned
`aic_actuator_calibration_v1` artifact. Steering, drive, and brake fits must all
pass their individual quality gates. Speed outside the corresponding recorded
applicability range fails closed.

Pure delay remains separate from first-order lag. Fractional delay is sampled
on the command grid by interpolation, and the first-order response uses the
stable exact discretization of the fitted continuous lag. Drive and brake use
separate gain, bias, delay, and lag parameters; a declared acceleration
hysteresis retains the previous mode around zero. The resulting actual
steering and acceleration feed a kinematic bicycle rollout with an explicit
positive wheelbase and time step.

## Consistency decision

`evaluate_rollout_consistency` compares the actuator rollout only with the
same selected model trajectory and matching speed profile. It records mean and
maximum position error, tangent-relative lateral error, wrapped heading error,
speed error, and endpoint error. Every threshold is explicit, finite, and
positive. Failed metrics are returned as ordered structured reason strings;
the evaluator never switches to another candidate.

Heading error is applied only where the model speed profile meets the explicit
minimum heading-observability speed. A stationary or near-stationary path has
no reliable tangent direction, but it still must pass all position, lateral,
speed, and endpoint checks. A moving backwards-oriented trajectory still
fails the heading gate.

Run unit and negative tests with:

```bash
python3 -m pytest -q \
  tests/test_control_projection_v3.py \
  tests/test_rollout_consistency_v3.py
```

## Historical V3-020 boundary

The recorded `d1log_0902` calibration candidate is not individually valid for
steering, drive, or brake, and therefore is rejected by the rollout entry
point. Authoritative vehicle limits and rollout-consistency thresholds also
remain unmeasured. V3-020 can be verified with deterministic synthetic unit
fixtures, but must not enable `full_control` from those fixtures.

No ROS 2 runtime, Safety Supervisor connection, vehicle command, or AWSIM
closed-loop test is part of this task. V3-021 and later authority changes must
remain separate tasks.

## Current promotion boundary

Subsequent tasks produced a stable calibration artifact and advanced it to
`shadow`, then actually ran both the ROS shadow graph and an M11 limited
full-control attempt. The trial did not establish forward motion, so the
artifact remains `shadow`; successful launch, route progress, collision
avoidance, and finish are not claimed. See
`docs/v3_m11_limited_odd_report.md` for the current evidence.

## Verification record

Commit `7cb4aa1` passed the focused projection/rollout unit and negative suite
in WSL (`26 passed`) and the combined repository plus ROS source suite
(`388 passed`). The exact tracked archive, SHA-256
`c2afba00742f73e25c0efeb17108ed645e4c7ff57bb1b7c31722c91746298341`,
then passed the projection, rollout, and calibration tests in
`aichallenge-2025-dev:latest` on Graneple (`33 passed`). The container's only
warning was inability to write pytest cache because the snapshot was mounted
read-only.

No ROS graph or AWSIM test was executed for V3-020, and no closed-loop success
is claimed.
