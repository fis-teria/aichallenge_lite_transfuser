# SI26 independent lap collection: 10 runs per speed

## Scope and state

User requested ten separately recorded laps per speed on 2026-09-11.
The target is ten total successful runs at each configured cap, 5 and 8 km/h:
existing `time_teacher_20260911_laps03/{5,8}kmh_run01` plus nine new runs each.
3 km/h is excluded because the previous attempt did not complete a lap.
This document records launch, not completion. Read live results before claiming completion.

## Execution

Host: `si26-pc008@192.168.3.13`.
Root: `/home/si26-pc008/git/autonomous_ai/ai-work/raw`.
Runner: `run_laps04.py`; collector: `collect_batch.py`.
Launched via Python `subprocess.Popen` with `start_new_session=True` and a file log,
so collection continues independently of the SSH connection.
PID at launch: `609954`; check command identity before any process action.

Equivalent execution command (already launched; do not rerun over this destination):

```bash
python3 /home/si26-pc008/git/autonomous_ai/ai-work/raw/run_laps04.py
```

Output: `time_teacher_20260911_laps04/{5,8}kmh_run02` through `run10`.
Sequence alternates caps: 5/run02, 8/run02, 5/run03, 8/run03, etc.
Each run starts a fresh simulator/container and records a distinct bag.
Each run saves configuration, exact collector and boot script, ROS parameter evidence,
probe summary, sensor bag, and a SHA-256 file after the recorder/container stops.
Runner progress: `time_teacher_20260911_laps04_runner.log`.
Completed-run summaries: `time_teacher_20260911_laps04/results.json`.

```bash
tail -n 5 /home/si26-pc008/git/autonomous_ai/ai-work/raw/time_teacher_20260911_laps04_runner.log
cat /home/si26-pc008/git/autonomous_ai/ai-work/raw/time_teacher_20260911_laps04/results.json
```

## Collection rules

Keep existing AWSIM localization, rendered camera and LiDAR, one ego vehicle,
no NPCs, and the previously validated Pure Pursuit/AWSIM reference route.
Stop normally after status lap 1 to 2, plus four simulated seconds of future support.
Keep the 30-minute driving upper bound and required-stream/speed/standstill guards.
Stop the batch on a collection fault, missing lap completion, or unconfirmed stop.
Never open an active SQLite bag. Preserve all failed runs separately.
Only stop containers owned by this collection; preserve the existing rosbag cleaner.
No application source changes, training, or model evaluation are part of this launch.

## Storage and validation

Preflight: host 177 GiB available; Windows D: 7.17 GiB available.
Raw data stays on SI26; do not bulk-copy the expected additional ~18 GB to WSL's D: VHD.
Training and model evaluation must run in native WSL under the shared training lock.
Both existing run01 summaries were checked for no fault, completed lap, and confirmed stop.
Runner passed Python compile checks locally and on SI26.
The source diff SHA-256 matched the prior validated collection configuration:
`9f3068e58b332d64ea0c8929f2cfa68a41f1f9a4ea7f286453d06773c56013a3`.
No package changes; full pytest was not run for this temporary experiment runner.

## Interpretation and follow-up

Independent restarts prevent frame-level train/validation mixing, but do not add
scenario diversity: route and initial conditions remain the same.
Do not claim generalization from these repeated laps alone.
Assign splits by whole run before fitting, and keep final test runs untouched.
End-of-collection braking is an intervention; exclude it and affected future targets
from ordinary cruising labels. Count usable labels only after timestamp/pose quality checks.
Collection completion, per-bag audit, canonical conversion, and training remain separate states.
