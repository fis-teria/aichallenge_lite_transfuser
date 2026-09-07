# 更新V4経路 → Pure Pursuit：有限AWSIM試験

開始版7cc5f4849dc77bd80ad443f7c2548ddf8bee2414、Windows clean。
ユーザー承認：駆動4回追加(7→11)、各回制動込み10sim秒、旧09:50 cutoffを更新。
準備/合成検証完了後の最初の起動予約で30分deadlineを一度だけ台帳に固定。
wall3600s / 共通forward7100 / sim320s / snapshot16 / log512MiBは据置。
Tiny5711回とV4 124回を共通上限で照合し、別counterを維持。

既存Lite側make dev、既存未改変AWSIM、隔離コンテナを使用する。
今回の既存V4起動経路はXvfbであり、デスクトップGUI/RVizの確認とは呼ばない。
元racing-kart Makefile、AWSIM設定/asset、既存dirty checkoutは変更しない。

1. 停止中の新規観測で固定step500を最大40forward。学習なし。
2. 未補正20点を保存し、ENDPOINT_NORMALIZED_LENGTH_V1で参照生成。
3. 観測時poseからworldへ変換、現在rear stateを後結合。
4. 既存PP(lookahead0.5m)/速度P制御(目標0.2、上限0.3m/s)。MPC solveなし。
5. 操作expiry200ms、state/clock/採用順/jerk/footprint/競合publisher/host watchdog維持。
6. 停止中結果が成立したら最大4run。6sim秒で制動に入り、9秒時点はhost freeze予備停止。
   1回の予約は制動込み10sim秒。実時計尾部の過小計上を避け駆動runは最低10秒を保守課金。
7. 新しい観測ID→forward→参照→PP→送信→速度/poseの実ログで更新追従を判定。
   HOLD/静止、freeze/KILL、移動後制動を区別し、未成立gateは外さず終了する。

固定経路41件のoffline成功は今回の更新経路成功を保証しない。
PP用rolloutは固定操作の合成自転車予測であって実走行証拠ではない。
衝突judge/全周回/実車安全性は今回の成功条件外、UNKNOWNをfreeにしない。

## コマンド

Windows commit後に既定tools/sync_to_wsl.ps1 -CheckOnly / 通常同期。
同期scriptのDataset操作は固定rootのtest -dのみ。内容探索はしない。

```bash
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_spatial_live_pp_v4.py tests/test_spatial_pure_pursuit_v4.py tests/test_spatial_run_continuation_v4.py tests/test_spatial_dev_connection_v4.py tests/test_spatial_reference_length_v4.py
# 専有hostへ固定commitのgit archiveを新規directoryに展開後:
make dev V4_PURE_PURSUIT=1 V4_PHASE=stationary V4_FORWARD_LIMIT=40 V4_WALL_SECONDS=60 \
 SIM_REPO=/home/graneple/git/autononous_ai/aichallenge-racingkart \
 V4_CHECKPOINT=/home/graneple/e2e_autonomous/spatial_sim_e2e_20260906/fixed_final.pt \
 XVFB_ROOT=/home/graneple/e2e_autonomous/spatial_sim_e2e_20260906/xvfb-root \
 V4_BINDING=/home/graneple/e2e_autonomous/spatial_run_continuation_20260907/binding.json \
 V4_BUDGET=/home/graneple/e2e_autonomous/spatial_run_continuation_20260907/budget.json \
 V4_OUTPUT=NEW_ABSOLUTE_OUTPUT V4_COMMIT=EXACT_COMMIT
# stationary成立後のみ同じ構成でV4_PHASE=run / V4_FORWARD_LIMIT=240 / 新規出力。
```

実行版・全attempt・停止確認・残予算は結果追記へ。自動pushなし。
