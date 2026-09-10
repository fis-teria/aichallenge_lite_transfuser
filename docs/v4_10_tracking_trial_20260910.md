# V4-10 AWSIM tracking evaluation

Use completed SpatialPathTenV4 epoch2, SHA256
`32752af8d3ebd023382ec405a0f72d529b4120471adabf4f12469c6baf7be22e`.
Explicit `ten_model` config selects36 raw points; ambiguous long/ten config rejects.
Checkpoint keys, shapes, model type, grid, epoch and file hash must match.

First run37 reproduces V4-20 run34 conditions: target0.25 m/s, drive10 sim seconds,
then brake; observation up to14 sim seconds, host wall180 seconds. Independent
50 ms controller watchdog, speed0.45 limit, plan TTL0.5 s, sensor TTL0.3 s,
LiDAR stopping corridor, steering angle/rate and prefix checks are unchanged.
Pure Pursuit uses the contiguous near3 m prefix and1 m lookahead. Raw36 points are
saved without correction. Sole sender `/v4_10_controller`; actual sent past commands
feed the model. Map/EKF data is not a model/control input. Simulation only.

The host's existing `CONTROL_METHOD=v4_20_external` is retained solely as the
controller-free make-dev launch mode; the actual dedicated controller/model are
V4-10 and are independently checked in the ROS graph. No baseline route PP.
Normal Autoware RViz gains a stock Path display `/visualization/v4_10/raw_path`;
no private RViz. Original RViz config backed up before adding the new display.

Run on the idle192.168.3.10 host in a new owned source/runner directory. Do not reuse
evidence paths or affect unrelated containers. Official Start precedes bounded
drive authorization. Preserve all failed attempts and independently verify braking
before host simulator freeze/kill. A short low-speed result is not a lap, collision
avoidance or general tracking proof. Maximum initial budget: two short trials,
second only for reproducibility or resolving a concrete startup issue.

```powershell
.\tools\sync_to_wsl.ps1 -CheckOnly
.\tools\sync_to_wsl.ps1
```

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
bash tools/with_wsl_training_lock.sh env OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 .venv/bin/python -m pytest -q
```

Host command for the first dedicated trial:

```bash
CARTOGRAPHER_BUILD_ROOT=/home/graneple/e2e_autonomous/cartographer_extrap_build_20260910 \
CARTOGRAPHER_TEST_PROJECT=codex-v4-10-tracking-37 CARTOGRAPHER_V4_EXTRAP_COMPARE=1 \
V4_SLAM_SHADOW_ROOT=/home/graneple/e2e_autonomous/v4_10_tracking_37 \
python3 /home/graneple/e2e_autonomous/cartographer_v4_10_tracking_37/run_moving.py
```

Transfer the completed evidence to native WSL and evaluate under the shared lock.
Report raw path quality, accepted prefix, command replay, measured movement/speed,
PLAN_STALE duty and brake-stop verification separately. Compare V4-20 run34 only
within the same low-speed short-trial boundary; updated predicted paths are not a
fixed map reference, so instantaneous model-path proximity is not lane accuracy.
Record commands and verification/results after execution. No remote git push.
