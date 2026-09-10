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

## Completed trials37 and38

Execution source `b764e7d8b0d1573654435012ad34025615b24ec8`.
WSL full pytest **1,853 passed / 4 skipped / 53 warnings,80.39 s**. Host Humble
package build succeeded (1 package,1.61 s). Source archive Windows/host SHA256
`1ca23bcce542b57decf989c39f00e256ab4b7bc71eff19247beefcb09ae05349`.
Both checkpoint copies and runtime strict load matched the declared epoch2 SHA.

| Metric | V4-20 prior run34 | V4-10 run37 | V4-10 run38 |
|---|---:|---:|---:|
| Drive authorization | 10 sim s | 10 sim s | 10 sim s |
| Speed-integrated distance | 1.254569 m | 1.266774 m | 1.255946 m |
| Maximum observed speed norm | 0.162124 m/s | 0.153393 m/s | 0.159943 m/s |
| Full model plans | 221 | 220 | 221 |
| Plans actually used for tracking | 65 | 62 | 70 |
| Tracking commands / replay matches | 176 / 176 | 180 / 180 | 180 / 180 |
| PLAN_STALE cycles while authorized | 25 | 21 /201 | 20 /200 |
| PLAN_STALE episodes | approximately5 | 5 | 6 |
| Prefix point counts | 13 only | 12:11,13:6,16:163 | 12:21,13:7,16:152 |
| Independent stop confirmation after authorization | 11.285 s | 11.220 s | 11.225 s |
| Host wall time | 69.649 s | 68.001 s | 68.476 s |

The independent speed stream confirmed <0.03 m/s for1 sim second after scheduled
braking and before host freeze/kill. Both trials had host_error=null,
probe_fault=null, cleanup=[] and sole `/v4_10_controller` command sender. Existing
wheel helper shutdown still has the known rclpy shutdown traceback; do not claim
every child exited without warnings. No simulator/controller/RViz processes or
running containers remained after either trial. No full lap/collision claim.

**Short low-speed tracking and braking were reproduced, but no clear improvement
over V4-20's short driving result was demonstrated.** Target0.25 m/s was not reached.
Approximately10% of drive-window command cycles requested braking for stale plans.
Accepted plan age median0.225/0.215 s, max0.500/0.475 s, with TTL0.5 s.
All360 tracking cycles used a prefix cut before a foldback; none used the raw36-point
path in full. Nominal grid labels1.2/1.3/1.6 m are not measured geometric path length.
Instantaneous local following is not fixed-route lateral RMSE or lane accuracy.

Raw36-point self-intersections appeared in6/62 and11/70 used model predictions.
Their final-point distance from the observation origin was5.147..5.940 m and
5.103..5.968 m, despite the output head's nominal10 m horizon. These geometry facts
are not teacher error measurements; ground-truth10 m trajectories were not acquired.

## Normal RViz and remaining failure mechanisms

Standard Autoware RViz subscribed to `/visualization/v4_10/raw_path`. For218 and221
received nonempty paths, all36 points/frame/stamps matched the recorded raw model
outputs under the specified root->lidar transform; max coordinate difference0 m.
The existing RViz config equals its `.before-v4-10` backup plus exactly the new
standard Path display. No private RViz process was launched. Run37 screenshot shows
the V4-10 display and Global Status: Ok in normal Autoware RViz.
Remaining path disorder therefore already exists in raw model predictions.

WSL rechecked the training selection, teachers, input provenance and canonical
samples hashes, then measured current ego speed for all12,661 train anchors with
observed10 m teacher support:

| Statistic | Current speed |
|---|---:|
| Minimum | 1.922498 m/s |
| 25th percentile | 6.523389 m/s |
| Median | 6.885400 m/s |
| 75th percentile | 7.508120 m/s |
| Maximum | 9.790764 m/s |
| Anchors at <=1 m/s / <=0.25 m/s | 0 / 0 |

Thus the added10 m teachers do not directly supervise long paths at this trial's
~0.15 m/s input speeds. This measured speed-support gap is a plausible contributor,
not proof of the sole cause. The model still has no predicted valid-length head
or explicit shape constraint. Offline improvement alone did not resolve low-speed
far-path reliability. Do not respond by simply raising test speed to match training;
first address low-speed horizon coverage/validity and investigate update stalls.

## Reproduction and evidence

Run38 repeats the command above with project/runner/source suffix38. Its source was
copied from37 with the instance ID/weight mount updated, preserving model and policy.
Runner37 SHA256 `0453eadb42374f040ea091a040af884572b70390930cda234ff3812960358fb9`;
runner38 `7a643a752a2105c2fb809eaa77ff45ffa4a25da6f93e11fd6d477e3137dbd4fc`;
shared probe `2f6405521fc466e525a6eb4355c72617815b11b1fd1589d24d3f88337f4dd463`.
Host evidence remains in each dedicated runner's `evidence/`.
Full copies are under native WSL
`/home/thistle/e2e_autonomous/runs/v4_10_tracking_20260910/run37` and `run38`.

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python tools/evaluate_ten_tracking_v4.py /home/thistle/e2e_autonomous/runs/v4_10_tracking_20260910/run37 /home/thistle/e2e_autonomous/runs/v4_10_tracking_20260910/evaluation37
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python /home/thistle/e2e_autonomous/runs/v4_10_tracking_20260910/diagnose.py /home/thistle/e2e_autonomous/runs/v4_10_tracking_20260910/run37 /home/thistle/e2e_autonomous/runs/v4_10_tracking_20260910/evaluation37
# Use suffix38 for the repeated trial; evaluator output must be a new directory.
```

Small summaries/plots, runner/preparation/diagnostic scripts and source archive are
preserved in Windows `tmp/v4_10_tracking_37/`. WSL also contains
`training_speed_support.json`. No data/checkpoint/bag added to Git, no push.
