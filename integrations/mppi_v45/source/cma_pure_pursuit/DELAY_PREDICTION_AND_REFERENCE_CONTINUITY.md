# Delay prediction and Reference continuity

## Runtime evidence

In DEV3 `20260821-115335/d1`, the first final-corner overtake path remained the
same accepted path from 57.034 s through 58.033 s. Its nearest anchor stayed
within 0.246 m and 0.161 rad. The Pure Pursuit delay prediction nevertheless
alternated between opposite predicted poses while the trajectory stamp was
unchanged. Only after that oscillation did the accepted-path anchor exceed its
contract and the Planner select `direct_overtake_replan_current_d_hold`.

Therefore the current incident is not an initial Planner Reference discontinuity.
The later Reference change is a downstream recovery response to tracking loss.

## Steering contract

- Planner steering requests remain passthrough commands.
- Delay prediction starts from the fresh measured steering report.
- Only the internal bicycle model target is bounded by the configured AWSIM
  steering angle, steering rate and first-order lag.
- The prediction bound must never be reused to reject, reshape or clamp a
  Planner trajectory or the command sent to the vehicle.
- Debug output reports requested, bounded-target and final predicted steering so
  an internal-model mismatch can be distinguished from command limiting.

## Reference continuity contract

- A newly admitted OVERTAKE candidate is already joined from the measured pose
  and passes the complete wall, all-opponent and PP-trackability validators.
- Once admitted, the immutable accepted-path continuation remains the preferred
  geometry while its anchor contract is valid.
- Do not geometrically blend an accepted pass path with FREE/FOLLOW or current-d
  paths: an interpolated path has not passed the original safety validators.
- `current-d hold` remains a validated fallback only after the accepted-path
  anchor becomes invalid. It is not an entry smoothing mechanism.
- Reassess Planner-side entry shaping only if a new run with the corrected delay
  model shows a command discontinuity while the accepted-path anchor remains
  valid. The required evidence is trajectory stamp, anchor distance/yaw,
  requested steering, bounded prediction target and measured steering.
