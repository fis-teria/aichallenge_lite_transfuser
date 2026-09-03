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

For each measured pose, the audit chooses the nearest reference Arrow and
projects its displacement onto the Arrow's left normal:

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
     --duration-sec 120
   ```

3. After checking exclusive control authority and the requested expert/baseline
   controller, repeat with `--execute`. Each bag gets a sidecar manifest with
   `scenario_id`, `collection_case_id`, exact topics, and Reference hash. The
   path is never overwritten.

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

The inspected official `/set_initial_pose` service is
`std_srvs/srv/Trigger`. It initializes from the simulator's latest GNSS and does
not accept a requested `(x, y, yaw)` offset. Therefore the repository does not
pretend to generate perturbed starts automatically. Until a verified simulator
reset/teleport API or an expert DAgger collection controller is available, a
human/test harness must safely establish each `collection_case_id`; recording,
provenance, coverage evaluation, and next-gap generation are automated.

Do not run this recorder beside trajectory-authoritative/full-control E2E
inference. Recording `/nominal_control_cmd` and
`/control/command/control_cmd` adds subscribers and can violate its strict ROS
graph preflight. Use an official expert/baseline control run or separately
designed mirrored diagnostic topics.
