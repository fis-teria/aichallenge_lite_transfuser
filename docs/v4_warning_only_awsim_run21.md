# 警告のみの局所Odometry：AWSIM試験21

2026-09-09。**既存Pure Pursuitで1周Finish、局所OdometryとV4 shadowの更新継続を確認。**
V4経路で制御した完走ではない。PPの既存Reference/EKF依存は残り、E2E部門適合走行の証明ではない。

## 配布・確認

- Windows正本branch `codex/windows-wsl-training-sync`、実行commit
  `13fd9be86f37d38c1a50f2904152506bb8cca7b9`。開始時clean。
- 既定CheckOnly/同期成功。Dataset固定rootの存在確認のみ実施、内容未読。
- `graneple@192.168.3.10` の `/home/graneple/e2e_autonomous/v4_pp_shadow_21` に専有配布。
  既存dirty checkout、AWSIM、scene、sensor、PP設定は変更なし。
- source.tar SHA256 `997ceb138767e02ee7cacf6c1036391c3dac1ca8d24fe77e590e53316b9cbd4a`。
  Windowsとremoteで一致を確認。
- 固定Humble環境、network noneでcolcon build 1 package/1.24s、CONFIG_OK_NO_INFERENCE。
  同隔離ROSの合成11件試験も成功。実モデルは走行試験でのみload。
- 開始前に稼働container/ROS/AWSIMなし、/dev/vcu・/dev/gnssなし、現DDS設定lo限定を確認。
  同一試行のinstance mount/device/network情報は `evidence/instance_inspect.json` に保存。

```bash
timeout --signal=TERM --kill-after=15s 300s python3 /home/graneple/e2e_autonomous/v4_pp_shadow_21/run.py
# wrapper内
make dev CONTROL_METHOD=pure_pursuit CAPTURE=false ROSBAG=false AWSIM_LAPS=1 \
  RUN_ID=v4_pp_shadow_21 OUTPUT_HOST_ROOT=/home/graneple/e2e_autonomous/v4_pp_shadow_21/evidence
```

1試行、外側300wall秒/駆動240sim秒以内、1周Finishで終了。
旧budget/usedを保持し新runへ累積引継ぎ。forward10000の保守予約と実推論767は別。

## 走行・終了

- AWSIM `/awsim/state` のfinish: **101.594997729sim秒**。
  Start再通知19.824999556から約81.770sim秒。開始位置への近接ではなく公式stateで確認。
- 最大前後速度4.8411m/s（約17.43km/h）。
- 終了理由SIMULATOR_FINISH、実wall131.2113秒。
  所有AWSIMをfreeze/KILL、Autowareも終了。
- raw resultはFAILED/OBSERVER_EXITのまま保持。終了処理との競合であり公式Finishとは区別。
  自然制動・ブレーキ停止成功、無接触・無ペナルティ、V4経路品質は未確認。
- 終了後の独立確認で当該project container、全稼働container、ROS/AWSIM process残存なし。

## 局所Odometry・V4

- 局所Odometry **2909件**、最終source101.814997724sim秒。
  専用v4_odom/v4_base_link、V4にはこのposeを入力。GNSS/IMU/EKF poseをV4へ結合しない。
- FORWARD_STARTED **767**、PLAN **766**。全PLANは未補正[20,2]有限値。
- 1件の拒否 `ValueError:current ego features must be valid` も保存。
- 成功PLANのcamera受信→join平均 **28.81ms**、推論呼出し平均 **33.16ms**。
- 実効PLAN更新 **7.55Hz**、最大wall間隔 **.706秒**。
  旧run17は110.51ms/3.68Hz/2.715秒。現在はpose源とjoinが変わった比較であり、
  モデル単体の高速化や同一条件での改善率とは扱わない。
- camera grid不適合192件、SUPERSEDED_BY_READY10件。JOIN_DEADLINEは0件。
  100ms gridの契約は維持され、全受信画像を推論したわけではない。

## 警告値（今回記録できたもの）

| source時刻sim s | 前後速度m/s | 横速度m/s | yaw rate rad/s |
|---|---:|---:|---:|
| 52.639998823 | 4.138926506 | -0.025059029 | 1256.332397461 |
| 60.094998656 | 4.563657284 | -0.058465369 | 1255.904296875 |
| 72.974998368 | 4.580586910 | 0.067647584 | -1255.806640625 |

3件とも `yaw_rate_exceeded=true`、`speed_exceeded=false`、
`action=INTEGRATED_UNCLIPPED`。停止・クリップせず、その後も更新した。
これは今回run21の超過値であり、記録不足だったrun20の瞬間値を遡及確定したものではない。

約1256rad/sは局所pose精度上の重要な課題。
前回静的確認したAWSIMのEuler角差分生成と、2π/.005s≈1256.64rad/sから、
角度折返しの可能性があるが、今回元rotationや全velocity streamは記録しておらず原因断定はしない。
停止しなくなったことと、正しい相対poseを得られたことは別。
この結果だけでLiDAR SLAMの必要性も確定しない。

## 保存

Windows `tmp/v4_pp_shadow_21/`、remote同名runにsource/config/run.py/budgetと全evidenceを保存。
主要ログは `shadow.jsonl`、`local_odometry.jsonl`、`vehicle_states.jsonl`、`motion.jsonl`、
`v4_pp_shadow_21/d1/autoware.log`（超過値JSON）、`result.json`。
実画面動画は未収録。今回追加再試行・自動push・AWSIM改変は行っていない。
