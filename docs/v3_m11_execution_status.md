# V3 M0-M11 execution status

This milestone map is the bounded path from the existing V3 trajectory model
to a gated full-control AWSIM trial. M11 does not include V3-024 or online
adaptation.

| Milestone | Deliverable and exit gate | Current status |
|---|---|---|
| M0 | Read repository instructions/specs; record Git, V1 freeze, WSL, and remote boundaries | complete |
| M1 | Versioned safe excitation plans with preflight, stop hold, and abort conditions | complete |
| M2 | Independent moving steering, drive, and brake runs captured in AWSIM | complete: 18 selected complete runs |
| M3 | Dataset V3 conversion, checksum transfer, and audit | complete: manifest `462015e2...07f4` |
| M4 | Separate steering, drive, and brake identification with quality/applicability ranges | complete: candidate `838ff71c...496b2` |
| M5 | Cross-run stability gate and rejected-cohort negative evidence | complete; calibration remains candidate until shadow |
| M6 | Ten-step physical control-sequence contract and bounded decoder | complete in source/tests |
| M7 | Same-run/same-clock future command labels with explicit tail masks | complete in source/tests |
| M8 | CUDA training artifact, per-SI-field offline metrics, and trajectory non-regression boundary | in progress; train-only Dataset has no independent run split |
| M9 | ROS shadow deployment with model trajectory/control diagnostics and RViz | pending execution on Graneple |
| M10 | Authoritative projection, calibrated rollout consistency, same-trajectory fallback, and Safety wiring | complete: WSL tests plus official-container ROS build/launch parsing passed |
| M11 | Stopped-start, 0.8 m/s limited-ODD full-control AWSIM trial and promotion report | pending M8/M9/M10 gates |

## M11 execution order

1. Finish the frozen-backbone sequence-head training and hash the checkpoint,
   runtime manifest, training manifest, and offline evaluation.
2. Run the full unit/negative suite in WSL. Build the ROS package in the
   official container; do not infer ROS success from Python tests.
3. Copy a tracked source archive and the WSL checkpoint/artifacts to Graneple.
   Do not push Git from WSL or Graneple.
4. Start AWSIM and Autoware through the official helper. Reset the vehicle and
   verify it is stopped before changing authority.
5. Run trajectory/model-control shadow with RViz. Record topic rates, sensor
   skew, inference status, Safety reason, predicted path, and commands. No
   shadow output may own the final command topic.
6. If shadow is clean, create a new hash-addressed calibration artifact in
   `shadow` state. Keep the candidate immutable.
7. Start the limited full-control launch. The model publishes only nominal
   commands; Safety Supervisor exclusively publishes
   `/control/command/control_cmd`. Apply the 0.8 m/s cap regardless of whether
   control comes from the model sequence or same-trajectory fallback.
8. Abort on sensor timeout, non-finite output, stale command, calibration range
   failure, Safety exception, or unexplained command publisher duplication.
9. Record route progress, collision/contact, intervention, fallback reasons,
   Safety reasons, maximum speed, and whether the finish was reached.
10. Produce the limited-ODD report. Promotion requires actual pass evidence;
    an unexecuted or failed trial leaves M11 incomplete and the artifact no
    higher than `shadow`.

## Explicit evidence boundary

The current source-level implementation and unit tests do not by themselves
prove ROS 2 graph correctness, AWSIM motion, collision avoidance, or course
completion. Those results are added here only after their commands have
actually run and their logs have been retained.
