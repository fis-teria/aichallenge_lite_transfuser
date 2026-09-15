# Evidence index

- `diagnosis/summary.json`: the one diagnostic drive, both clearance profiles replayed on saved scans, and two measured normal-reference comparisons.
- `evaluation/summary.json`: official judge outcome and command/vehicle-model replay.
- `deployment/`: exact source and ROS readiness receipts.
- `trial_config.json`: explicit AWSIM-only, one-trial profile; fixed target 5 km/h and unchanged A checkpoint.
- `shipping.json`, `transfer_verification.json`: the sealed raw archive and complete native WSL verification.
- `post_environment.json`: original remote workspace, RViz and historical containers preserved.
- `wsl_transport_recovery/`: saved process inventory and one exited transport cleanup before the confirmed-idle WSL distribution was restarted. No training, evaluation or user process was active at that point.
- `reproduce/`: task-specific sealing, verification and analysis scripts. Native evaluation and analysis require the repository worktree lock wrapper. Use fresh output paths for a repeat; the original evidence must not be overwritten.

`transfer_manifest.json` is the harvest receipt. `artifact_manifest.json` covers the final directory except itself. `.gitattributes` preserves exact evidence bytes through Git.
The raw archive and full attribution remain in `/home/thistle/e2e_autonomous/runs/time_near_limit_diagnostic_20260915`.
The smaller-clearance result is separate from the six standard trials. A ray-distance margin is not a measured vehicle collision distance.
