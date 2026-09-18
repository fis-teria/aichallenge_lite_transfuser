# partial05_first_failure.npz

2,105-byte regression fixture from AWSIM run `codex-time-slam-mppi-partial05`
(2026-09-19), first `MPPI_NO_FEASIBLE_PATH`, scan stamp 29140413475 ns.
Contains the exact SLAM occupancy grid (-1 unknown, 0 free, 100 occupied),
0.2 m resolution, metre-valued reference/origin, and base pose [m, m, rad].
Source implementation commit: `39d566cc968441b6257f90fa07643f321dc1efe2`.

The original lateral-offset family failed here. Curvature rollouts must find
a valid path without changing the grid, steering limit or footprint clearance.
This fixture proves local planning feasibility, not closed-loop AWSIM success.
