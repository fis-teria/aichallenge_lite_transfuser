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
| M8 | CUDA training artifact, per-SI-field offline metrics, and trajectory non-regression boundary | complete with boundary: artifact and metrics retained; the one-run Dataset has no independent run split |
| M9 | ROS shadow deployment with model trajectory/control diagnostics and RViz | complete: actual Graneple ROS graph published trajectory/control diagnostics at about 9 Hz with RViz and zero final-command publishers |
| M10 | Authoritative projection, calibrated rollout consistency, same-trajectory fallback, and Safety wiring | complete: WSL tests, official-container ROS build, and live Safety-owned command wiring passed |
| M11 | Stopped-start limited-ODD full-control AWSIM trial and promotion report | attempted again with causal-history checkpoint, incomplete: Safety stayed normal but rollout gate used fallback for every decision and did not establish launch or route progress; artifact remains `shadow` |

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

## Execution result

M8-M10 are complete under the evidence boundaries above. The M11 launch was
actually run against AWSIM on `graneple@192.168.3.10` with RViz and Safety as
the sole final-command publisher. The final 30 s observation recorded a
maximum speed of `0.012603 m/s`, mean speed of `0.002424 m/s`, and displacement
of `0.039785 m`. Safety was `normal` for all 600 sampled states, but the
longitudinal command did not continuously overcome the vehicle's launch
deadzone. The trial was stopped by the operator; final command publisher count
returned to zero and measured speed returned to `0.001570 m/s`.

This is not a route-progress or course-completion pass. No result, collision,
or contact topic/file was available in the observed ROS graph, so collision
avoidance is `NOT_EVALUATED`, not successful. See
`docs/v3_m11_limited_odd_report.md` for commands, hashes, and the remaining
training-data gate. V3-024 was not started.

A later `causal_previous_only` checkpoint (`6e8fc01b...`) removed the learned
maximum-brake lock and produced a forward shadow trajectory. During the actual
30 s Graneple re-trial at a 0.75 m/s cap, Safety was `normal` for all 600
samples, but all 270 model decisions failed rollout position/endpoint
consistency and used the same-trajectory fallback. Maximum vehicle speed was
only `0.002166 m/s` and displacement was `0.029100 m`. The graph was stopped,
the final-command publisher returned to zero, and AWSIM was reset to
`WaitStart`. Therefore M11 remains incomplete; this is not a launch or course
completion pass.

## Explicit evidence boundary

The ROS shadow and limited full-control graphs were actually started against
AWSIM. They prove topic wiring, inference publication, Safety ownership, and
the failed stopped-start attempt described above. They do not prove meaningful
AWSIM motion, collision avoidance, route completion, or generalization beyond
the single training run.
