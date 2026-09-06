# 未改変AWSIM: RUN/dispatch継続修正

開始HEAD a04fa39c5b6dda014baab338ee5e7810d4fb1d46。前段V4 60 / HOLD MPC1 / MPC送信0を変更しない。
ユーザーの2026-09-07依頼scope内。AWSIM/元起動script/元racing-kart Makefileを変更しない。

| 既存不整合 | 今回の限定変更 |
|---|---|
| host監視の初回powered heartbeat待ち | 送信前ARMED handshake、powered=falseでも期限監視 |
| cleanupのunpause | 実host専用helperでpaused→KILL→exitedを確認、解凍せず終了 |
| cleanup失敗で後続停止が飛ぶ | sim/runtime/logの終了試行を分離、first/cleanup error別記録 |
| 105ms cameraを100ms gridへ入れるgap reset | SIM_GRID_MISSING_V2、40msは維持、欠損slotの全mask false、command保持 |
| 真のepoch/gap | epoch/clock回帰/実camera途絶>1sは両履歴reset |
| pose右bracket遅着 | raw/forward/cutoff固定、control-only後結合、元200ms deadlineを延長しない |
| bundle時のdt=0 | 単一sim送信schedule、model dt/次区間/実送信間隔を分離、publisher側で一度だけrate積分 |
| 最新queue IDとの一致 | epoch/age/採用順/state整合/重複operationで判定、queue更新は採用更新ではない |
| 停止近傍の数値負速度 | raw/model入力不変。Drive実送信＋連続1s停止観測、-1mm/s以内のみcontroller初期v=0 |
| body/frameの固定false | 同じscene/colliderの境界と取付けへ結合。旧幅1.3mより保守的な約1.536m |
| scanで現在全車体を見る要求 | 旧scan診断を保持し、static AABB exclusionと現在車体を別判定。AABB内はUNKNOWN |
| 再起動ごとの定数budget | 指定の累積JSONへ予約/終了、未回収counterは予約上限を課金、active未解決なら次回拒否 |
| 未消費Queueによる終了待ち | worker停止後cancel_join_thread/close。未消費推定数と打切りを記録 |
| ROS終了処理中のheartbeat途絶 | 終了要求→host pause実確認→有限runtime終了待ち。pause前に監視を無効化しない |

SIM_GRID_MISSING_V2はsim限定の意味差であり、学習時完全parityとはしない。
欠損slotのsourceはgrid_slotsでnull、内部transport再利用はmask=falseの保管用で有効複製ではない。
sensor_dtは有効slotのみcamera-grid/lidar-camera、欠損は0かつmask=false。
startup paddingと欠損slotは別。11枠を実時間1sで覆ったstable_historyを記録し、その代表tensorを最大2個保存する。
commandのstrict past/50ms age/availabilityは変更しない。

poseの限定誤差根拠はGNSS設定delay=0、20Hz bracket≤50ms、5ms physics step。
source acquisition自体はUNKNOWNであり、物理時刻証明ではない。
現在stateはpose/v/steeringの同時刻補間。dispatchで最新stateとplan初期stateを物理移動上限＋小さな数値許容で再照合する。

現在のscene境界抽出は障害物2個のAABB。mesh内部・路面/壁の面分類と動的actorの全監視は未成立。
AABB包含をfreeにしない。外部判定のUNKNOWNを衝突なしや内部judgeと同等にしない。
RUN条件が成立しない場合は40 forwardで有限終了し、HOLDを大量追加しない。
RUN参照が成立した場合のみ最大240 forward/60sim秒、累積残量の上限内。

検証command（全pytest/学習testは禁止、同一Windows commitを既定sync後WSL lock下で実行）:

```bash
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q \
  tests/test_spatial_run_continuation_v4.py tests/test_spatial_dev_connection_v4.py \
  tests/test_spatial_sim_e2e_v4.py
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q \
  tests/test_spatial_runtime_v4.py tests/test_spatial_bootstrap_v4.py \
  -k 'receipt or policy or first_error'
```

既定syncのDataset操作は固定root存在確認のみ。Dataset内容は読まない。
前段asset metadata/指定run/logと今回許可された固定checkpoint/新sim観測は別scopeとして読む。
実hostの新規出力を使う。終了済みhelperはID 7233fb59ff33756d3d471c8cdc2c90c437bb31672f7516d6630ff4f7f9961cfc。

```bash
make dev SIM_REPO=/home/graneple/git/autononous_ai/aichallenge-racingkart \
  V4_CHECKPOINT=/home/graneple/e2e_autonomous/spatial_sim_e2e_20260906/fixed_final.pt \
  XVFB_ROOT=/home/graneple/e2e_autonomous/spatial_sim_e2e_20260906/xvfb-root \
  V4_OUTPUT=NEW_ABSOLUTE_TASK_OUTPUT V4_COMMIT=EXACT_40_CHAR_COMMIT \
  V4_BINDING=ABSOLUTE_SELECTED_BINDING_JSON V4_BUDGET=ABSOLUTE_CUMULATIVE_BUDGET_JSON \
  V4_PHASE=run V4_WALL_SECONDS=60
```

実行値と試験結果は新規report/packetへ記録する。実装や合成passを走行成功へ昇格しない。pushしない。

最終実行版af1c7f12d99b1889fb8ea4ead395a4b6458c1b45。
実測結果は `spatial_path_v4_run_continuation_result.md`。
