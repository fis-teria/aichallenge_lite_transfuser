# V3 M0 Plan Contract / Executable Reference

## Scope

This M0 patch adds ROS-independent contracts and shadow diagnostics for the
A-prime trajectory-authoritative design. It does **not** change runtime control
authority, Safety Supervisor behavior, checkpoints, or V1 code.

The model motion intent is represented by
`AuthoritativePlanV3`:

- `trajectory_xy_m`: ego-frame future path, shape `[N,2]`, metres
- `speed_profile_mps`: non-negative speed at each waypoint, shape `[N]`, m/s
- `waypoint_times_sec`: strictly increasing time from the observation, shape `[N]`, s
- `observation_stamp_sec`: source observation time, s
- `frame_id`: explicit frame name
- `stop_probability`: optional during the pre-Stop-Head migration only, within `[0,1]`

`require_stop_probability=True` makes a missing Stop Head fail closed. The temporary
`False` setting must remain explicit until the V3 Stop Head is trained and deployed.

## E_plan

`evaluate_plan_consistency_v3` computes geometric speed from the polyline segment
length, including the segment from ego origin to the first waypoint:

```text
v_geom[k] = segment_length[k] / segment_dt[k]
v_trap[k] = (v_previous[k] + v_predicted[k]) / 2
residual[k] = v_geom[k] - v_trap[k]
```

It returns field-level arrays, mean/max absolute error in m/s, and a
speed-scale-normalized Huber mean. It does not define an authority threshold. M0
must first collect held-out distributions and relate them to controller commit
horizon, calibration uncertainty, and track clearance.

## Executable Reference transformation

`build_executable_reference_v3` performs these pure transformations:

1. validate shape, units, finite values, time order, frame, and Stop Probability;
2. trim only leading non-forward points contained within a 0.05 m radius of
   ego when at least two forward points remain, then require path length;
3. convert the trajectory polyline to cumulative arc length;
4. estimate discrete absolute curvature;
5. apply ODD, curvature, and optional upstream safety speed caps;
6. retime every spatial segment using trapezoidal executable speed;
7. assign a deterministic SHA-256 reference ID and immutable output arrays.

The builder does not apply a hidden launch-speed floor. A non-zero path whose
trapezoidal speed is below `minimum_retime_speed_mps` returns a STOP decision with
`non_executable_speed`. Invalid plan data and an asserted model stop also return a
structured STOP decision and never yield a controller reference.

The bounded trim is an executable-reference normalization, not a Safety bypass.
It is reported as `trimmed_initial_nonforward_noise`. A leading point outside
the configured radius, no later forward point, or fewer than two retained points
still returns `initial_waypoint_not_forward` and STOP.

## Shadow runtime diagnostic

Every V3 runtime profile publishes a compact JSON record on
`plan_diagnostics`. It contains the raw Plan, `E_plan`, executable-reference
decision, transformations, and retimed reference. The external-controller shadow
profile also includes its preview target and command in the same observation
record. This topic is explicitly `shadow_diagnostic_only`; it neither publishes
nominal control nor changes Safety behavior.

The ROS-independent payload is implemented in
`src/aic_transfuser_lite/runtime/plan_diagnostics.py`. Invalid plan diagnostics
are reported on `runtime_status` and do not interrupt the existing inference
status path.

## Exact control-sequence teacher time grid

The full-control training view now selects each sequence target by the exact key
`(run_id, segment_id, anchor_grid_stamp_ns + k * control_dt_ns)`. A later CSV row
is never shifted into a missing 100 ms slot. Missing timestamps are zero-filled,
masked on all three control fields, and marked
`missing_exact_timestamp`. Each batch carries `[B,H]` target times in seconds and
`[B][H]` provenance.

The required alignment identity is
`control_sequence_alignment: exact_grid_timestamp_only` in both the temporal view
and full-control model config. Because the temporal view file is part of the
experiment identity hash, a checkpoint produced with the former next-row
semantics cannot be resumed silently. Existing runtime artifact contract hashes
remain unchanged.

The read-only Graneple M0 graph check confirmed the installed Drive value `2`,
Autonomous value `1`, AWSIM's direct final-command subscription, and the official
Start/control-mode request path. See `docs/v3_m2_external_controller.md` for the
observed commands and remaining dynamic-response boundary.

## Example

```python
import numpy as np

from aic_transfuser_lite.control.executable_reference import (
    AuthoritativePlanV3,
    ExecutableReferenceConfigV3,
    build_executable_reference_v3,
)
from aic_transfuser_lite.runtime.plan_consistency import evaluate_plan_consistency_v3

plan = AuthoritativePlanV3(
    trajectory_xy_m=np.array([[0.1, 0.0], [0.2, 0.0]]),
    speed_profile_mps=np.array([1.0, 1.0]),
    waypoint_times_sec=np.array([0.1, 0.2]),
    observation_stamp_sec=10.0,
    stop_probability=None,
)
metrics = evaluate_plan_consistency_v3(plan, current_speed_mps=1.0)
decision = build_executable_reference_v3(
    plan,
    current_speed_mps=1.0,
    config=ExecutableReferenceConfigV3(
        odd_speed_cap_mps=0.75,
        max_lateral_acceleration_mps2=1.0,
        require_stop_probability=False,
    ),
)
assert metrics.normalized_huber_mean == 0.0
assert decision.reference is not None
```

## Verification

Focused unit and negative tests:

```bash
PYTHONPATH=src python -m pytest -q \
  tests/test_executable_reference_v3.py \
  tests/test_plan_consistency_v3.py \
  tests/test_plan_diagnostics_v3.py \
  tests/test_cli_full_control_train_v3.py
```

WSL validation must hold the shared worktree lock. After committing and syncing the
Windows source according to `docs/windows_codex_wsl_training_workflow.md`, run:

```bash
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
```

ROS 2 and AWSIM execution have not been performed for this patch. They must not
be reported as successful until run in the designated official environment.
