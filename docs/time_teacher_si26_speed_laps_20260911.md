# SI26 time-based teacher collection: speed bands and complete laps

## Accepted collection policy

The user explicitly accepted the current AWSIM position information on 2026-09-11.
Use existing GNSS/IMU/localization and measured velocity/steering. Further position
accuracy improvements are not a prerequisite for collection. Keep its provenance
as recorded; localization estimates are not relabeled as simulator ground truth.

The initial 180-second proposal was corrected by the user: collect through one
complete lap, with approximately 30 minutes as an upper limit, so low-speed runs
include all corners. The lap batch uses speed ceilings of 3, 5 and 8 km/h, one
vehicle at a time. Measured speed is reported separately from the configured cap.
No learning-model control and no new training are part of this collection task.

## Execution and stop contract

Host: `si26-pc008@192.168.3.13`, workspace
`/home/si26-pc008/git/autonomous_ai`.
Raw lap batch: `ai-work/raw/time_teacher_20260911_laps01/` beneath that workspace.
Each speed has a separate `Nkmh_run01/` folder and owned Docker container
`time-teacher-20260911-l01-Nkmh`.

Existing Pure Pursuit follows `raceline_awsim_15km.csv`; both
`external_target_vel` and `reference_execution_speed_cap_mps` are set at launch
to the selected cap in m/s. Runtime external-speed changes are not used because
the prior pilot showed a cached parameter. Only the existing speed caps differ
between the three lap runs.

`/awsim/status.data[1]` is the current lap counter. The existing MPC callback at
`multi_purpose_mpc_ros/mpc_controller.py` interprets an increase beyond 1 as a
completed lap. Accordingly, 0 -> 1 is only the initial start-line crossing;
the collector requires 1 -> 2, records four more simulation seconds for future
teacher coverage, then interrupts its own Pure Pursuit process and publishes
timestamped brake commands. It confirms measured speed below 0.03 m/s for three
simulation seconds, records another four wall seconds, flushes the bag and stops
only its own container.

Bounds: 1,800 simulation seconds, collector 1,860 wall seconds (including startup),
host supervisor 1,950 wall seconds, owned container lifetime 1,980 seconds.
Missing/stale required sensors, recorder failure, excessive measured speed or an
unplanned standstill cause a failed attempt to be retained separately. A failed
attempt without confirmed normal stopping prevents automatically starting the
next speed. The 30-minute bound is a timeout, not the normal stopping condition.

Exact runtime scripts/argv are preserved per run as `collect.py`, `boot.bash`,
`docker_command.json`, `collection_config.json`. Host batch entry:

```bash
python3 /home/si26-pc008/git/autonomous_ai/ai-work/raw/run_laps.py
```

The entry creates a fresh root exclusively and cannot overwrite these results.
Source application files are mounted read-only and existing unrelated changes
are preserved. Batch source HEAD/diff are recorded and checked after execution.
No remote Git push. The unrelated ROS-bag cleaner container is not stopped.

## Preliminary short recordings

`ai-work/raw/time_teacher_20260911_batch01/3kmh_run01` and `5kmh_run01` completed
180-second windows with timestamped braking and confirmed stopping. They were
already launched when the user corrected the stopping policy. The old batch was
cancelled after the 5 km/h recorder completed; no 8 km/h short run was launched.
These are partial-course supplemental recordings, not complete laps.

## Validation

Collection scripts passed Python syntax compilation on the host. Bag evaluation
runs in native WSL under the shared worktree lock. Small audit source is preserved
at Windows `tmp/time_teacher_si26_20260911/audit.py`. No application package code
was changed; full pytest/training was not run for the collection-only task.

First lap batch (`laps01`) aborted after about 282 driving seconds because the
SQLite recorder raised `database is locked`. A read-only in-run database query
used to extract a camera preview likely contended with its writer. This was an
inspection error, not insufficient host storage or an OOM (both were checked).
The failed bag is preserved; it is not a completed lap. Do not read an active
SQLite bag, including for previews. Inspect only after recorder closure.

The restarted batch is `ai-work/raw/time_teacher_20260911_laps02/`, with Docker
names `time-teacher-20260911-l02-Nkmh` and entry `ai-work/raw/run_laps02.py`.
Live monitoring reads only the separate probe log; bag inspection waits for closure.
`laps02/3kmh_run01` subsequently stopped after about 577.9 driving seconds with
`unplanned standstill`. This was not a time-limit stop. The recorded movement was
210.10 m; one lap was not completed. Its closed bag was retained and audited in
WSL as partial-course supplemental/diagnostic data, not a completed lap. The
failure path records final speed zero but does not set the normal-stop-confirmed
flag; do not reinterpret that flag as a successful normal completion.

The next bounded batch `ai-work/raw/time_teacher_20260911_laps03/`, entry
`ai-work/raw/run_laps03.py`, ran the remaining 5 and 8 km/h settings. Both completed
one lap and confirmed braking. All owned simulator/controller containers are
stopped. Source HEAD remained `ee37e2051bd86211d4d459695bbdc11eb048718c`; the existing
dirty source diff remained SHA256
`9f3068e58b332d64ea0c8929f2cfa68a41f1f9a4ea7f286453d06773c56013a3`.

## Completed measurements

| Configured cap | Result | Lap duration | Moving-speed median | Max measured speed | Speed-integrated distance | Camera frames | LiDAR scans | Camera anchors with full 3-s pose support |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| 3 km/h (`laps02`) | Partial, unplanned standstill | N/A | 0.402985 m/s | 0.884836 m/s | 210.104 m | 5,567 | 11,691 | 5,539 |
| 5 km/h (`laps03`) | One lap completed and stopped | 373.405 s | 0.977677 m/s | 1.497675 m/s | 378.633 m | 3,907 | 8,207 | 3,879 |
| 8 km/h (`laps03`) | One lap completed and stopped | 202.805 s | 1.784471 m/s | 2.250171 m/s | 384.150 m | 2,213 | 4,649 | 2,185 |

Moving median includes measured speeds above 0.05 m/s. Distances include initial
approach and end-of-collection movement, not only the timed lap. Configured speed
caps are controller parameters, not an absolute bound on measured physical speed.
The completed-lap pair totals **6,120 camera frames / 12,856 LiDAR scans / 6,064
camera anchors with complete three-second position interpolation support**.
Their bag directories occupy approximately 2.02 GB in total. These are raw data
inventory/support counts, not final accepted training samples or independent scenes.

WSL independently decoded the saved `/awsim/status` values and verified the
0 -> 1 -> 2 progression for both successful runs. Both recorded final speed zero,
positive-timestamp brake commands, and confirmed stopping before simulator shutdown.
The camera cadence is approximately 9.52 Hz, LiDAR approximately 20 Hz, and
localization approximately 50 Hz. Camera/LiDAR nearest-stamp skew p95 was 23.91 ms
at 5 km/h and 23.99 ms at 8 km/h. Maximum skew was 32.14/29.17 ms respectively;
do not claim every pair is below 30 ms. Decoded mid/end camera frames were visually
checked after recorder closure, including a curve and the return to the start area.
This verifies collection and lap completion, not collision-free driving quality.

Command streams contain repeated timestamps; preserve arrival order and handle
duplicates explicitly in any causal history construction. Collection-end braking
is an intervention with a recorded `brake_sim`, not an environmental stop label.
Canonical conversion should separate that intervention and its preceding future
horizon from ordinary cruising teachers. Existing AWSIM position data is accepted
per the user's instruction; it is not a reason to block further use or collection.
No time-based model was trained in this collection task. Run splits remain to be
assigned; two completed laps do not establish broad scenario generalization.

## WSL preservation and reproduction of the audit

Root: `/home/thistle/e2e_autonomous/runs/time_teacher_si26_20260911/`.
Closed copies: `laps02/3kmh_run01`, `laps03/5kmh_run01`, `laps03/8kmh_run01`.
The earlier interrupted `laps01` bag remains separately on the host.
Host and WSL bag SHA256 matched:

| Run | `bag_0.db3` SHA256 |
|---|---|
| 3 km/h partial | `8abc4bd8ed394221061be94e5451dfecf46be9e13d069a63374ec144c0f6e581` |
| 5 km/h complete | `27e9ed05ab4c453bb7be689e3996fdf4daf8420858cb028524910b355fa56d84` |
| 8 km/h complete | `ae46465bd1cad9eedf28dd6825aad20b11f81a29156418dfb912c82e0403df24` |

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
bash tools/with_wsl_training_lock.sh .venv/bin/python \
 /home/thistle/e2e_autonomous/runs/time_teacher_si26_20260911/audit.py \
 /home/thistle/e2e_autonomous/runs/time_teacher_si26_20260911/laps03/5kmh_run01
# Repeat for laps03/8kmh_run01 and laps02/3kmh_run01, only after recorder closure.
```

Audit outputs beside each closed bag: `data_audit.json`, `audit_arrays.npz`,
decoded camera PNGs. Small Windows copies are in
`tmp/time_teacher_si26_20260911/`. Raw bags were not added to Git.
