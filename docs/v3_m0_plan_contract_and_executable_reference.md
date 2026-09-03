# V3 M0 Plan Contract / Executable Reference

## Scope

This M0 patch adds ROS-independent contracts and diagnostics for the A-prime
trajectory-authoritative design. It does **not** change runtime control authority,
Safety Supervisor behavior, ROS topics, launch files, checkpoints, or V1 code.

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
2. require the initial waypoint to be ahead of ego and the path to have length;
3. convert the trajectory polyline to cumulative arc length;
4. estimate discrete absolute curvature;
5. apply ODD, curvature, and optional upstream safety speed caps;
6. retime every spatial segment using trapezoidal executable speed;
7. assign a deterministic SHA-256 reference ID and immutable output arrays.

The builder does not apply a hidden launch-speed floor. A non-zero path whose
trapezoidal speed is below `minimum_retime_speed_mps` returns a STOP decision with
`non_executable_speed`. Invalid plan data and an asserted model stop also return a
structured STOP decision and never yield a controller reference.

The current implementation is deliberately not wired into
`inference_node_v3.py`. Runtime wiring belongs to M1 after M0 confirms the exact
AWSIM command semantics, gear/autonomous state, command routing, and timestamp
alignment.

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
  tests/test_plan_consistency_v3.py
```

WSL validation must hold the shared worktree lock. After committing and syncing the
Windows source according to `docs/windows_codex_wsl_training_workflow.md`, run:

```bash
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
```

ROS 2 and AWSIM tests are not part of this M0 patch. They must not be reported as
successful until run in the designated official environment.
