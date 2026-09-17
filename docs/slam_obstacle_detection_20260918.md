# LiDAR + wheel-odometry local SLAM obstacle detection

Base-map wall matching is suspended. This separate mode runs the existing
Cartographer installation with scan + current E2E wheel odometry only. No
preloaded Lanelet/map, GNSS, IMU, reference/global pose or simulator objects enter
perception. The E2E model and motion controller remain unchanged.

The scan-time SLAM pose aligns current occupied surfaces and a recent occupancy
grid (50 m square, 0.2 m cells, 2 s TTL). Free rays clear previous hit locations;
unknown remains unknown. Detection always uses current returns, including
stationary obstacles already present in the grid. No subtraction of newly
learned "background" can erase a stopped car. Surface IDs are tentative; this
does not classify vehicles or estimate their true speed.

Fresh E2E waypoints are transformed at their original observation-time SLAM pose.
The detector reports observed points overlapping a 0.85 m half-width path
corridor plus a 1.8 m front extension. Without a matched fresh plan,
`path_valid=false` and `path_blocked=null`, never an empty-road assertion.
This diagnostic corridor is not a certified vehicle swept footprint. No brake,
steering or target-speed command is published by the SLAM/detection sidecar.

The original AWSIM scan is preserved. Cartographer's copy and the RViz display
copy use the verified snapshot interpretation (`time_increment=0`) and dedicated
frame `time_slam_lidar`. Cartographer's copy also replaces invalid/saturated
returns with +inf. Mount: base forward 1.65 m, z 0.0377 m.
Wheel odometry is converted from base origin to LiDAR origin for Cartographer.
SLAM local matching is active; global loop-closure optimization is disabled.
Cartographer's TF subscriptions are isolated from the normal Autoware TF tree.
Its installed `time_conversion.cpp` rounds input time to 100 ns ticks:
`((stamp_ns + 50) // 100) * 100`. Scan/pose association accounts for this exact
representation conversion (at most 50 ns), preserving the original scan time.
It does not allow a different scan to substitute for a missing pose. The ROS
smoke includes non-100-ns timestamps to exercise the actual library conversion.

Outputs:

| Topic | Meaning |
|---|---|
| `/time_path/slam/scan` | Scan displayed at its own SLAM observation pose |
| `/time_path/slam/pose` | Local SLAM base pose, no global-map alignment |
| `/time_path/slam/recent_occupancy` | Recent observed hit/free/unknown cells |
| `/time_path/slam/obstacles` | Occupied surfaces, confirmation, path overlap |
| `/time_path/slam/markers` | Yellow surfaces; red overlaps predicted path |
| `/time_path/slam/path` | Same E2E output expressed in the local SLAM frame |
| `/time_path/slam/status` | Validity, missing/stale inputs, counters and graph |

Dedicated RViz uses `time_slam_map`; it does not overlay the suspended base map.
Scan/SLAM timeouts clear markers and mark results invalid. Backwards clocks
require a restart of the coupled SLAM session. Display availability is not
localization accuracy, and phantom LiDAR returns can still appear as occupancy.

```bash
# Native WSL validation, after Windows commit/sync:
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
# Official ROS image with prepared Cartographer and matching runtime install:
python3 tools/check_slam_obstacles_ros.py --output <new-smoke-directory>
# Dedicated AWSIM host deployment, same finite E2E runner:
timeout --signal=TERM --kill-after=10s 710s python3 "$DEPLOYMENT/source/tools/run_time_path_awsim_trial.py" \
  --deployment "$DEPLOYMENT" --run-id codex-time-slam-obstacles01 --display :0 \
  --config configs/control/time_path_dev.json --ros-launch \
  --max-speed-kmh 20 --corner-max-speed-kmh 10 --npcs 1 --pp-vehicles 0 \
  --record-video --slam-obstacles
```

The runner requires a prepared install at
`/home/graneple/e2e_autonomous/cartographer_extrap_build_20260910`, records the
actual binary/config hashes, checks fresh SLAM output and RViz subscription
before motion, owns all child processes, and restores normal RViz afterwards.
`--slam-obstacles` and `--lidar-map-comparison` are mutually exclusive.

Validation at source commit `14a1e113484947a34a7116dc327dad482ffd8c4e`:

- Native WSL full suite: **3077 passed, 4 skipped**, 84 warnings, 130.62 s.
  Existing skips: unavailable OSQP, Draft2020 validator, jsonschema, optional
  official tiny-LiDAR package. No dependencies installed to hide those limits.
- Official ROS image: all **250 installed Python sources** byte-matched the
  committed archive. Existing Cartographer binary/config identity is recorded.
- Real Cartographer + synthetic ROS scan/wheel/path test: **PASS**, 129 processed
  scans, 38 stopped-obstacle overlap scans. Checks cover stationary obstacle
  confirmation, removal without a ghost obstacle, missing scan, paused clock,
  absence of GNSS/IMU/global-pose inputs and absence of command publishers.
- Deployment: `/home/graneple/e2e_autonomous/time_slam_obstacles_20260918_r4`.

Static obstacle detection, moving vehicle classification, stopping/avoidance
control and true-pose accuracy are separate claims and must be supported
individually.

## AWSIM trial, 2026-09-18

`codex-time-slam-obstacles03`, same source/deployment above, **NPC0**, requested
speed caps 20 km/h and corner 10 km/h, `--slam-obstacles --record-video`:

| Measurement | Result |
|---|---|
| Official ordered-section lap | 1 lap, 132.1934 s |
| Judge penalties | 0 (crash / wall / over all 0) |
| Processed scans, including pre-start and stopping | 1565 |
| Rejected / dropped scans | 0 / 0 |
| Scans in active E2E tracking interval | 1361 |
| Fresh plan association in that interval | 1359 / 1361 = 99.85% |
| Scan observation age at processing | median 81.3 ms, p95 102.1 ms |
| Maximum gap between processed observation times | 152.0 ms |
| Sampled status while tracking | 557 / 557 OK |
| Scans reporting predicted-path overlap | 31 |

The tracking interval is the first through last `TIME_PATH_TRACKING` command,
sim time 17.880–158.990 s. It excludes stopping and deliberate simulation freeze
during cleanup; those produce the expected `CLOCK_STALE`. The 99.85% is temporal
association availability, **not object-detection accuracy or localization
accuracy**. The two unavailable plans yielded `path_valid=false`, not clear-road
claims. Of 32 overlapping surface instances in those 31 scans, 31 had observed
extent greater than 3 m; walls are included. NPC0 has no vehicle-recall
denominator. A visible/occupied surface is not a semantic vehicle identification.

Both SLAM nodes and the active E2E controller/inference graph have no GNSS,
IMU or `/localization/*` subscriptions. SLAM nodes publish no motion commands.
The normal Autoware startup stack still runs its own localization initialization;
its GNSS-dependent readiness is separate from E2E control and SLAM perception.

RViz shows current cyan LiDAR, yellow occupied surfaces (red for path overlap),
magenta E2E prediction and a green SLAM pose. The recent grid remains distinct
from the suspended base map. Recordings are H.264, 10 fps, no audio, and both
decoded fully without errors:

- `artifacts/slam_obstacles_20260918/awsim_npc0/rviz.mp4`: 161.8 s, 1280×786.
- `artifacts/slam_obstacles_20260918/awsim_npc0/awsim.mp4`: 161.2 s, 1280×960.
- Small metrics/hashes: `docs/evidence/slam_obstacles_20260918/awsim_npc0_audit.json`.
- Judge detail: `docs/evidence/slam_obstacles_20260918/awsim_npc0_judge.json`.

NPC1 attempts `01` (before timestamp fix) and `04` (final source) both failed
**before driving**: the official startup helper observed initialization readiness
false and hit the runner's 20 s timeout. NPC spawn count 1 was verified, but no
NPC driving/detection-performance claim is supported. This startup condition was
not bypassed. Attempt `02` exited before launch because another task owned active
containers; its containers were preserved. Final-source NPC0 is the completed
driving trial. The synthetic stopped-box test is the stationary-obstacle evidence.

All owned trial containers were stopped. The normal RViz file was restored
byte-for-byte (SHA256 `981c28955e9bceedb8845ec8d59e57c4f47805040cd84c5e09f2f1430c7bb3f1`),
and the normal dev Makefile hash remained
`5f4fd6f292f1741ed2c6bb1f6919d5a0cc7b1437a1a54d3e2f3954732ddf5cf6`.
This feature is explicitly enabled by `--slam-obstacles`; it is not a permanent
replacement of the standard dev launch or a new avoidance/braking controller.
