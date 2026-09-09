# Existing MPC solution guard (local implementation; NOT DEPLOYED)

Goal: MPC-only lap completion. This change addresses acceptance of invalid
solutions and over-limit raw steering, not path/model/solver redesign.
AWSIM and V4 are unchanged. No new driving authorization is implied.

The two files are patched copies of the current racingkart source under
`aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/`:

- `MPC.py` -> `multi_purpose_mpc_ros/core/MPC.py`
- `mpc_controller.py` -> `multi_purpose_mpc_ros/mpc_controller.py`

Baseline remote commit: `4af395eee10f928c7fc7225760adfa04c4c07ff4`, dirty preserved.
Baseline SHA256:

- MPC.py: `78f7bc85734d231330555219d53aee8b18e23f20fc7417a520e62a2388316155`
- mpc_controller.py: `27eb4132c071f00d01030510db4378834c846eac53d81e12cec60b097455427e`

Only OSQP `solved` (status_val=1) is admitted. `solved inaccurate`, absent,
wrong-sized or nonfinite vectors and violation of assembled constraints by
more than 1e-5 absolute numerical units are rejected. This tolerance does not
expand vehicle/wall margins. Its effect on real solve acceptance is NOT_RUN.
Zero curvature is a normal solution; there is no safety-margin retry.
On rejection, old predicted controls and prediction metadata are cleared.
The returned zero speed request holds bounded previous raw steering. The node
overrides feedforward/filter/disabled-control ramp with configured braking.
Braking is a REQUEST, not verified stopping or a collision-free stop path.

Raw steering absolute/rate checks happen before the existing gain conversion.
The final message limit is therefore raw limit times existing gain, not the
same numeric angle. Consumer/actual tire-angle calibration is not changed.
The first rate-limited command and the solved prediction remain distinct;
prediction is not claimed to be the executed rollout. Existing QP adjacent
curvature-difference rate formulation is not redesigned in this scope.

Tests: Windows commit, unchanged CheckOnly/sync, then WSL worktree lock:

```sh
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q \
 tests/test_mpc_solution_guard.py tests/test_mpc_speed20_fixture.py
```

Tests use synthetic solver results, one synthetic OSQP problem, and AST-extracted
actual node methods without importing ROS or publishing. They do not establish
full ROS integration, simulator braking, successful lap, or real-vehicle safety.

Remote application/install remain pending. External review is optional under
the user's 2026-09-09 policy revision and no longer blocks this work. Do not overwrite
a dirty target with these copies: compare baseline hashes and use a reviewed
focused patch. Do not automatically submit the historical queued review.
No automatic push, live launch, inference, collection, or driving.

Validation at ff04c19: WSL 27 passed / 1 skipped (OSQP unavailable).
The actual deployed MPC venv OSQP 0.6.7.post1 passed the two synthetic QPs in
`smoke_osqp.py`, run with `--network none` and read-only source mounts.
The venv has no pytest; the standalone smoke does not require package installs.
Full pytest and ROS integration remain NOT_RUN. Evidence is under
`tmp/mpc_solution_guard_results/`. `git apply --check` exited 0 on the remote
baseline; retain the node's existing executable file mode when applying.
