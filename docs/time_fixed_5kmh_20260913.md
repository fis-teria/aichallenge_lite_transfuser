# TimePath B0: Pure Pursuit + fixed 5 km/h AWSIM trial

## 結果

固定5km/hを指令する設定を実装・適用し、指定ホストで10秒間のAWSIM追従試験
`codex-time-trial-05` を完了。目標速度は全201回の制御で5km/h、実測の最高速度は
4.687km/h、終盤3秒の中央値は4.623km/h、走行距離は11.238m。
停止指令後の実測停止も確認した。周回完走・回避・車線追従誤差の評価は未実施。

これは **目標速度の固定化** の確認であり、実速度が正確に5km/hを維持したという
結果ではない。終盤の加速度指令中央値は0.419m/s^2で、速度偏差と正の加速度指令が
残る。現行の比例制御 `clip(4 * (目標速度 - 実測速度), -1, 1)` で負荷依存の偏差が
残ることとは整合するが、AWSIM側の駆動変換・走行抵抗を個別同定した結果ではない。
次の速度追従調整はこの偏差を対象とする。今回、ゲイン・アクチュエータ・重みは変更していない。

- [実行・WSL再計算結果](evidence/time_fixed_5kmh_20260913/summary.json)
- [速度・加速度・操舵の時系列](evidence/time_fixed_5kmh_20260913/control_timeline.png)
- [モデルの未加工時間経路](evidence/time_fixed_5kmh_20260913/raw_time_paths.png)
- [固定5km/h設定](../configs/control/time_path_fixed_5kmh_awsim_20260913.json)

## Active request and state

The user requested Pure Pursuit with a fixed 5 km/h target. The model remains
TimePathV1 B0, command history OFF, epoch 10, checkpoint SHA256
`e857db4b67d3d9f6a77c2865cfc1fa53e9903e7a1d2d8a371407fbe009a1a44f`.
Previous evidence: [origin guard trial](time_origin_guard_20260913.md), trial04.
Its 0.25 m/s commanded target reached 0.153 m/s measured maximum. No full lap
or lane-tracking accuracy claim follows from that test.

Initial first false: the existing controller derives target speed from time-point
spacing and caps it at 0.25 m/s; it cannot command the requested 5 km/h.
Owner: the time trial longitudinal target policy, carried through controller
startup configuration and recorded replay. No model or actuator retuning.

## Necessary change and acceptance

- Add an explicit opt-in `fixed_5kmh` speed policy: 5 / 3.6 m/s normal target,
  6 / 3.6 m/s overspeed stop. Keep the default source-speed 0.25 m/s trial
  compatible, including replay of previous recordings.
- Pure Pursuit follows the same raw 30 XY points in metres. Preserve observation
  time, age alignment, map/base_link/rear_axle transforms, geometry checks,
  acceleration +/-1 m/s^2 and steering/rate limits. Teacher inputs stay separate.
- Fixed speed is an experimental external longitudinal policy, not a learned
  speed/stop decision. Stationary/unresolved, stale, infeasible and insufficient
  stopping-reference predictions cannot authorize fixed-speed motion.
- Existing stopping-distance checks use measured speed: at 5 km/h the nominal
  reference requirement is 1.759 m and scan corridor clearance is 2.059 m
  (reaction 0.5 s, braking 1 m/s^2, respective margins 0.1/0.4 m). These checks
  remain active; the forward scan corridor is not a swept-footprint proof.
- Configuration must agree with implemented speed and duration limits; log the
  selected policy and config SHA so WSL can replay the actual calculations.
- Verify fixed target independently of model point spacing, old mode behavior,
  overspeed, invalid config, short/stationary path and speed-dependent clearance.
- Bounded execution: `graneple@192.168.3.10`, same selected AWSIM scene, normal
  RViz, one new run ID, 10 simulation seconds drive / 30 wall seconds drive,
  120 wall seconds total including shutdown. Stop then confirm measured zero.
  Preserve historical projects; stop only this owned run. No automatic retries.
- Success for this task is correctly commanded 5 km/h on admitted model paths,
  recorded measured response and classified safety rejections, not an invented
  assertion that the measured speed is exactly 5 km/h or a full lap was passed.

The change is necessary at the target calculation, not at the trained weights.
An opt-in policy is smaller and reversible by selecting the unchanged old
configuration. Tests and recorded replay isolate its effect; the host trial
will establish the measured response. Current state: opt-in policy, matching
JSON/ROS launch, policy-aware replay, speed-response metrics and synthetic ROS
overspeed brake check implemented and verified. Trial05 completed and replayed.
The requested fixed target is established; exact measured-speed regulation and
longer/corner/lap evaluation remain separate, unexecuted work.

## Verification before trial05

- Runtime commit `c452e42b9bdd441bd86ff8dd1c73d2d8e1a598f9`; Windows committed
  and native WSL synchronized cleanly. Focused tests: 56 passed. Full WSL
  `pytest -q`: 2057 passed, 4 skipped, 63 warnings, 99.94 s.
- Previous trial04 replay passed with 201 matched commands. New policy does
  not change those prior calculations.
- WSL evidence root: `/home/thistle/e2e_autonomous/runs/time_fixed_5kmh_20260913`.
- Deployment root: `/home/graneple/e2e_autonomous/time_fixed_5kmh_20260913`;
  source `source_c452e42`, archive SHA256
  `a3f86c75d82432dabe9a470b5c717ae78733d834497ffa7904489f6e37e5b17b`.
- Existing Humble image build: 1 package, 1.16 s. Five control modules and ROS
  controller source/install hashes match (`installed_identity.json`). Fixed-5
  config SHA256 `3509ab7e65950dafb850f5c00370a0d703ef484f95356a7c980ecf19b9bc5f58`.
- Isolated Humble smoke: actual checkpoint raw Path matches 6; synthetic
  fixed-5 PP positive shadow commands 110; stale-plan braking 11; paused-clock
  braking 7; overspeed braking 12; zero vehicle command publishers. Both child
  exit codes 0. This is ROS integration evidence, separate from AWSIM behavior.
- Host GPU RTX 4060 Laptop / 595.91.07 healthy; no active runtime at preflight.
  Existing host checkout modifications and historical projects are preserved.

## Commands

Windows commit, then `./tools/sync_to_wsl.ps1 -CheckOnly` and sync. In native WSL:

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
```

After the existing isolated Humble build/smoke and install identity checks,
execute on the specified host (run ID is consumed once):

```bash
cd /home/graneple/e2e_autonomous/time_fixed_5kmh_20260913
timeout --signal=TERM --kill-after=10s 110s python3 \
  source_c452e42/tools/run_time_path_awsim_trial.py \
  --deployment /home/graneple/e2e_autonomous/time_fixed_5kmh_20260913 \
  --run-id codex-time-trial-05 --display :1 \
  --config configs/control/time_path_fixed_5kmh_awsim_20260913.json </dev/null
```

The recorded trial used this command without `</dev/null`; the SSH exec
completion returned exit code 0 and the host recorded `COMPLETE_BOUNDED_TRIAL`.
The shell's intended post-timeout receipt file was absent. Redirecting stdin in
the reproduction command prevents child processes consuming streamed shell
input. This did not require rerunning the consumed trial. The observed SSH
completion, rather than an invented separate child exit receipt, is recorded
in `final_environment.json`.

## Trial05 evidence and boundary

- Actual controller/host both selected `fixed_5kmh`; recorded config SHA matches
  the source config. Normal Autoware RViz reused, official AWSIM Start succeeded.
- All 201 active commands were `TIME_PATH_TRACKING`; all used 5/3.6 m/s target.
  The 88 active observation predictions passed the unchanged geometry policy.
  No active safety rejection; all 30 raw points remained unmodified.
- WSL replay matched all 201 calculations (geometry and PP before steering-rate
  clamp), maximum numerical difference `8.881784197001252e-16`, tolerance 1e-9.
- 201 measured poses; path travel 11.238270308 m, net displacement 11.224178840 m.
  Final 3 s measured speed min/median/max: 4.556/4.623/4.687 km/h. No active
  sample was within +/-0.25 km/h of 5; do not call this exact speed tracking.
- Model inference p50 52.147 ms / p95 99.848 ms / max 142.882 ms. Four initial
  missing-input records were explicit rejections; no inference fault occurred.
- Drive ended at the 10 s simulation deadline; measured speed stayed below
  0.03 m/s for >=1 simulation second before cleanup. Host wall time 55.259 s,
  cleanup errors zero, no owned active/stopped containers remain. All 39
  historical Compose project names preserved. GPU driver stayed healthy.
- Host raw evidence: deployment root `codex-time-trial-05/`. Its 21 top-level
  records were packed into `trial05_records.tar` (12,503,040 bytes), archive SHA
  `b366190ce669513b0af78d3ff92028402aae4d0a19bcf47944ac063bc4800810`.
  Windows and native WSL archive hashes match; each extracted file hash verified.
- Native WSL copy and evaluation are under the evidence root above:
  `codex-time-trial-05/`, `evaluation05/`, `speed_response.json`.
  Raw records, weights and archive remain outside Git; selected small reports,
  checks and plots are under `docs/evidence/time_fixed_5kmh_20260913/`.

WSL evaluation command (output directory must be new):

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  tools/evaluate_time_awsim_trial.py \
  --run /home/thistle/e2e_autonomous/runs/time_fixed_5kmh_20260913/codex-time-trial-05 \
  --output /home/thistle/e2e_autonomous/runs/time_fixed_5kmh_20260913/evaluation05
```

Rollback selects `configs/control/time_path_awsim_trial_20260913.json` explicitly
with `--config` for a new, separately bounded run. The old default source-speed
policy and prior deployments/evidence are preserved. This trial does not validate
learned stopping behavior, obstacle avoidance, complete swept-footprint clearance,
corner coverage or lap completion. No retraining was needed for target selection.
