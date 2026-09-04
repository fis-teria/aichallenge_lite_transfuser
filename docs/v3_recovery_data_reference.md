# Dataset V3 recovery-data Reference and automated collection

## Purpose

The first closed-loop M3 trial did not fail because the existing Dataset V3 had
no curves. The current 48,946 canonical samples contain 7,190 straight,
24,283 right-curve, and 17,473 left-curve samples. However, all 11 runs share
one `scenario_id`, and the canonical audit found only 281 stop-to-launch
candidates and 479 stop-approach candidates. The dataset also has no explicit
gate for lateral displacement, heading error, or successful recovery.

This Reference adds those gates. It is an offline teacher/debug contract. The
Reference pose, global pose, projection error, and coverage bucket must never
become inference inputs.

## Coordinate and sign contract

`capture_raceline_reference_v3.py` reads the transient-local
`/heading_pose_initializer/raceline_markers` `MarkerArray`. Only Arrow markers
in namespace `heading_arrows` are accepted. The Arrow tail is `(x_m, y_m)` and
the tail-to-head direction is `heading_rad` in the marker frame.

For each measured pose, the audit projects onto the nearest circular route
segment and interpolates the adjacent Arrow headings. This is required because
the captured course Reference is sparse; nearest-point projection on a curve
would misclassify along-track spacing as lateral error. The displacement is
then projected onto the interpolated heading's left normal:

- positive lateral offset: vehicle is left of the Reference;
- negative lateral offset: vehicle is right of the Reference;
- positive heading error: vehicle yaw is counter-clockwise from the Reference;
- position is metres, heading is radians, time is seconds/nanoseconds.

The generated manifest sets `teacher_debug_only: true` and records the CSV
SHA-256. The initial numerical gates live in
`configs/data/recovery_collection_reference_v3.yaml`. They are conservative,
versioned engineering targets, not evidence that a particular count guarantees
closed-loop quality.

## Coverage gates

Every bucket requires a minimum sample count, distinct run count, and episode
count. The initial contract checks:

- stopped, stop-to-launch, and moving-to-stop approach;
- straight, left curve, and right curve;
- left/right lateral offset in 0.25--0.50 m and at least 0.50 m bands;
- left/right heading error of at least 5 degrees;
- left/right successful recovery, defined as reducing absolute lateral error by
  at least 0.15 m within 3.0 s.

An episode is separated from the previous same-bucket sample by another run or
at least 0.5 s. This prevents one long segment from being reported as many
independent recovery cases.

## Automated workflow

Run these commands in the official Linux/ROS environment. Do not record or
train under `/mnt/e`.

When an external teacher is routed through the independent Safety Supervisor,
run the teacher-only safety launch below. The teacher publishes
`/nominal_control_cmd`; Safety is the only publisher of
`/control/command/control_cmd`.

```bash
ros2 launch aic_e2e_runtime teacher_capture_safety_v3.launch.py \
  use_sim_time:=true \
  maximum_speed_mps:=0.75
```

`use_sim_time:=true` is mandatory. If the teacher stamps commands from the
simulation clock while Safety uses the wall clock, every teacher command is
correctly rejected as `nominal_command_timeout`. Before recording, verify one
publisher on each command topic and verify that `/safety_reason` is not a
continuous `nominal_command_timeout`.

For the official MPC, start the teacher in the Autoware environment with:

```bash
ROS_DOMAIN_ID=1 tools/run_official_mpc_teacher_v3.sh \
  /path/to/multi_purpose_mpc_ros/config/config.yaml \
  /path/to/multi_purpose_mpc_ros/config/ref_vel.yaml
```

The runner also remaps the MPC's relative
`control/control_mode_request_topic` subscription to the simulator's
`/awsim/control_mode_request_topic`. Without that remap the MPC can calculate a
non-zero target speed while continuing to publish zero commands.

### Recovery Reference generation

Keep two References separate throughout collection:

- the captured raceline is the fixed teacher/debug-only axis used by the
  coverage audit to measure lateral and heading error;
- the generated recovery Reference is supplied only to the official MPC so it
  deliberately moves away from that axis and returns.

`generate_recovery_reference_v3.py` reads the official MPC CSV and occupancy
grid. It automatically selects disjoint left-curve and right-curve intervals
whose complete circular vehicle footprint remains free. The initial contract
uses a 1.40 m centre-to-wall clearance: half of the official effective 2.30 m
width plus the configured 0.25 m wall margin. Clearance is checked between
source waypoints at occupancy-grid resolution, not only at CSV points.

Each selected episode is a C2-continuous lateral profile. The first low-speed
pilot showed that a 6 m approach and 3 m hold ended before the MPC/vehicle
lateral response settled, so the versioned collection config now uses:

- 10 m approach from the base MPC line to the requested offset: record for audit,
  but exclude from training;
- 8 m hold at 0.35 m (near) or 0.55 m (far): eligible after post-run checks;
- 4 m return to the base MPC line: eligible after post-run checks.

At the 0.75 m/s collection cap, 4 m is short enough to produce at least
0.15 m improvement inside the existing 3 s recovery gate. Eligibility is
written to the adjacent `*.intervals.csv`; generation alone does not make a
frame training data. The post-run pose, sensor synchronization, Safety reason,
and successful return still have to pass.

Generate into a native Linux data/artifact directory (the Windows example
below is only the Codex source-of-truth dry run):

```bash
PYTHONPATH=src python3 tools/generate_recovery_reference_v3.py \
  --base-reference /path/to/traj_mincurv_manual.csv \
  --occupancy-map-yaml /path/to/occupancy_grid_map.yaml \
  --config configs/data/recovery_reference_generator_v3.yaml \
  --output /artifacts/recovery_reference_v3/recovery_reference_v3.csv
```

The tool refuses overwrite and emits a hash-bound manifest plus the interval
CSV. For the inspected official MPC, render a copied config rather than
modifying its package or source checkout:

```bash
PYTHONPATH=src python3 tools/render_official_mpc_recovery_config_v3.py \
  --base-config /path/to/official/config.yaml \
  --reference-container-path /artifacts/recovery_reference_v3/recovery_reference_v3.csv \
  --output /artifacts/recovery_reference_v3/official_mpc_recovery_config.yaml
```

The renderer accounts for the controller's package-share path concatenation,
forces circular/reference-path mode, and disables dynamic path/border updates.
Use the copied official `ref_vel.yaml` with `run_official_mpc_teacher_v3.sh`.
The independent Safety Supervisor remains authoritative; changing the path is
not permission to bypass the measured 0.75 m/s speed cap.

Build the coverage axis from the exact generated-path source CSV, rather than
from the sparse visualization markers:

```bash
PYTHONPATH=src python3 tools/convert_mpc_reference_to_collection_reference_v3.py \
  --input /path/to/traj_mincurv_manual.csv \
  --output /artifacts/recovery_reference_v3/base_mpc_collection_reference.csv
```

Its manifest binds the source CSV SHA-256. The marker capture remains useful
for course/version diagnostics, but its roughly 5.7 m spacing is too sparse to
be the primary numerical offset axis on tight curves.

1. Capture the course Reference once per course/version:

   ```bash
   python tools/capture_raceline_reference_v3.py \
     --output /home/thistle/e2e_autonomous/references/aic_course_v1.csv
   ```

2. Ask the recorder for a read-only preflight plan. This validates exact topic
   types, required roles, Reference existence, and the absence of a conflicting
   E2E inference node. It creates no bag without `--execute`:

   ```bash
   python tools/record_dataset_v3.py \
     --topic-profile configs/data/topic_profile_recovery_collection_v3.yaml \
     --output-root /home/thistle/e2e_autonomous/datasets/raw_v3 \
     --run-id recovery_left_far_001 \
     --scenario-id d1_sim_recovery_left_far \
     --collection-case-id offset_left_far \
     --reference /home/thistle/e2e_autonomous/references/aic_course_v1.csv \
     --teacher-controller-id official_pure_pursuit \
     --teacher-command-role final_command \
     --duration-sec 120
   ```

3. After checking exclusive control authority and the requested expert/baseline
   controller, repeat with `--execute`. Each bag gets a sidecar manifest with
   `scenario_id`, `collection_case_id`, exact topics, and Reference hash. The
   path is never overwritten. Official MPC/Pure Pursuit publish directly to
   `final_command`; select `final_command` for those runs. When an expert is
   explicitly routed through the independent Safety Supervisor, select the
   pre-Safety `nominal_command`. The selected teacher topic must have exactly
   one publisher or preflight fails.

4. Audit all raw bags without decoding Camera/LiDAR payloads:

   ```bash
   python tools/audit_collection_coverage_v3.py \
     --input-root /home/thistle/e2e_autonomous/datasets/raw_v3 \
     --topic-profile configs/data/topic_profile_recovery_collection_v3.yaml \
     --reference /home/thistle/e2e_autonomous/references/aic_course_v1.csv \
     --criteria configs/data/recovery_collection_reference_v3.yaml \
     --output /home/thistle/e2e_autonomous/reports/recovery_coverage.json
   ```

The audit returns exit code 0 only when every gate passes. On a gap it returns
2 and writes `recovery_coverage.gaps.json`, including additional samples, runs,
episodes, and a collection instruction for each deficient bucket. Codex can use
that file as the next bounded acquisition plan instead of blindly adding laps.

After collection, run the normal canonical conversion, run/scenario split,
Dataset V3 audit, training, and closed-loop evaluation. Coverage PASS is a data
gate; it is not deployment proof.

## Current automation boundary

The official `/set_initial_pose` service is `std_srvs/srv/Trigger`. It
initializes from the simulator's latest GNSS and does not accept a requested
`(x, y, yaw)` offset. However, the inspected AWSIM build also exposes a
deterministic process-start randomizer. A Graneple capture verified that the
following arguments reached AWSIM and displaced the first synchronized
bag-derived pose by 0.275 m from the fixed-start teacher pilot. The signed
lateral offset relative to Reference point 59 increased from +0.664 m to
+0.937 m, so the resulting capture is a verified left-far case:

```text
--start-random=true --start-random-seed=103 \
--start-random-range=0.80,0.00 --start-random-min-separation=0
```

Do not validate a randomized start from a localization sample retained across
an AWSIM restart. Restart Autoware, wait for initialization readiness, and use
the first synchronized pose stored in the finalized bag as the evidence value.

This makes bounded lateral and longitudinal start-position acquisition
automatable by restarting AWSIM with a versioned seed. The recorder must still
measure the actual pose against the Reference before assigning a left/right
near/far case; the requested range is not evidence of the realized offset.
Use course-safe ranges, return AWSIM to `Grounded`, and stop the teacher after
each capture.

The randomizer preserves the base vehicle orientation, so it does not provide
controlled heading-error examples. Those cases still require a verified
scenario/pose mechanism, a safe human/test harness, or an expert DAgger
controller. Recording, provenance, coverage evaluation, and next-gap
generation are automated for every established start.

The generated MPC recovery Reference removes that heading-error limitation for
bounded lateral recovery episodes: its smooth approach and return create
measured offset and heading error while the official controller remains the
teacher. Start randomization is no longer the primary source of the recovery
label. It remains useful only to place a short pilot before a selected segment;
the realized bag pose and the fixed raceline audit still decide the label.

Do not run this recorder beside trajectory-authoritative/full-control E2E
inference. Recording `/nominal_control_cmd` and
`/control/command/control_cmd` adds subscribers and can violate its strict ROS
graph preflight. Use an official expert/baseline control run or separately
designed mirrored diagnostic topics.
