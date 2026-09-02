# V3 bounded residual contract

V3-022 adds a ROS-independent residual blender. The external controller stays
the primary proposal source. The model contributes only the difference between
its debug proposal and that external proposal, independently hard-clipped in
steering (rad), speed (m/s), and acceleration (m/s^2).

When residual mode is disabled, the function returns the exact external
command object without inspecting or numerically rebuilding it. This preserves
the baseline bit pattern and permits disabled operation before model or limit
artifacts exist.

Enabling residual mode requires finite, positive, explicitly authoritative
per-field residual limits with a non-empty source. Missing, zero, infinite, or
unreviewed limits fail closed. A model proposal marked authoritative is also
rejected. The result continues to mark the external controller as primary and
Safety Supervisor as mandatory downstream authority.

Run unit and negative tests with:

```bash
python3 -m pytest -q tests/test_bounded_residual_v3.py
```

No measured authoritative residual limits currently exist, so residual mode
must remain disabled outside deterministic tests. This task adds no ROS
publisher or launch profile, does not connect a command to Safety Supervisor,
and does not execute AWSIM. V3-023 full-control gating remains a separate task.

## Verification record

Commit `12fab54` passed the focused WSL suite (`12 passed`) and the combined
repository plus ROS source suite (`411 passed`). Its exact tracked archive,
SHA-256
`4ce71f4aa371d57f2849d31d10162fce444d58f66449db5fbf95d60e87a6e26e`,
passed the same 12 tests in `aichallenge-2025-dev:latest` on Graneple. The sole
official-container warning was the expected read-only pytest-cache warning.

No ROS 2 or AWSIM execution was performed for V3-022, and residual control was
not enabled on a vehicle.
