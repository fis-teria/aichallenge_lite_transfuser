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

## 実行結果：停止中入力更新まで、駆動は未実施

実行commit `40b879b16c33ccd22d1bec5c2fc3647e80740069`。
2026-09-07 10:25:48 JSTに台帳へ承認を一度だけ追記し、cutoffを10:55:48 JSTに固定。
実行host `graneple@192.168.3.10`。
専有source `/home/graneple/e2e_autonomous/spatial_live_pp_20260907/source_40b879b`。
出力は同rootの `stationary_40b879b_01`。実行は上記コマンドのstationaryのみ。
元repo HEAD `4af395eee10f928c7fc7225760adfa04c4c07ff4`、既存dirty状態の
porcelain SHA256 `0b9af671098d0d60fd59457c6aba36e75c88b17ff6a04c1045622ea5617eb9a3` は前後不変。
配布tar SHA256 `77bc470b6e22b8e087abd0a496675bbe196d7202477160a7aaefe59d2c3f15b6` は転送前後一致。

| 段階 | 実測 |
|---|---:|
| 新規camera観測由来V4 forward | 40（観測時刻40種類、raw hash40種類） |
| stable history | 35 / 40 |
| 可変長参照受理 | 35 / 40 |
| 初期state limit拒否 | 5 / 40 |
| pose後結合期限切れ | 23 / 40 |
| 後結合・PP要求生成到達 | 12 / 40、全件HOLD |
| PP送信 / MPC solve | 0 / 0 |
| 駆動 / 移動中経路更新追従 | 0 / 未実施 |

12件はHOLD用要求であり、RUN操舵の実送信確認ではない。
全12件でclearance=false、collision_monitor=false、最終motion_rejectionは
`ROLLOUT_FREE_SPACE_UNVERIFIED`。静的scene側は
`STATIC_MESH_INTERIOR_UNRESOLVED`（コース全体を覆うAABB内、dynamic_coverage=false）。
LiDAR全車体判定も`UNKNOWN_FREE_SPACE`であり、見えない領域をfreeとはしていない。
これは実衝突を検出した意味ではなく、許可に必要な観測根拠がない意味。

独立senderのdispatchはEXPIRED 10、NO_ACCEPTED_MPC_REQUEST 28、
ROLLOUT_FREE_SPACE_UNVERIFIED 2。MPCという拒否名は共通既存gateの名称でありMPC実行数ではない。
23件はpose join開始/完了が元camera受信から200msの有効期限に収まらず
`POSE_JOIN_EXPIRED`。入力待ち・fit・後結合それぞれの寄与は未断定。
期限延長、gate無効化、同一失敗の4回反復はしなかった。

control publishはHOLD125 + STOP20 =145回、全加速度0m/s²。
mode/gear要求は各1回。最大観測速度0.0001562778m/sで実質静止。
stationary_stop_confirmed=trueは**初めから静止**の確認で、移動後制動成功ではない。
host_armed=true、watchdog_faultなし、runtime exit0。
その後に所有simをfreeze→KILL、runtime終了を確認。unpauseなし、cleanup errorなし。
AWSIM binary/vehicle.yaml/DLL/scene/asset/元起動scriptの検証hashは前後一致。
終了後docker psは空（他者process停止なし）。AWSIMの変更なし。

## 検証と保全

限定合成テスト60 passed。最初8.71s、JUnit/生ログ保存用再実行6.26s。
実sim試験の再実行はしていない。全pytest/学習テストは実施しない。
既定同期によるDatasetルートの存在確認をCheckOnly・通常同期で実施。
Dataset内容・既存rawの読取りは未実施。
今回承認された固定checkpoint読取と新sim sensor観測は実施し、全parameterの前後不変を記録。
新学習/optimizer/Tiny推論/追加sensor tensor保存/自動pushは未実施。

Windows証拠: `tmp/spatial_live_pp_20260907/` の
worker.jsonl、supervisor.jsonl、worker/supervisor/host summary、authorization、budget_after、
resolved_config、instance_inspect、compose_resolved、checkpoint_load_map、host.log、
validation.log、tests.xml。元hostにも全ログを保全。

| ログ | SHA256 |
|---|---|
| worker.jsonl | 170658070ba92d4e0ce7c92f72ea69f6ad5dca467c77c38e27f46dc2b79bab0a |
| supervisor.jsonl | 459961168eb90f72e72f4046343fbe001f24f51bf29d736712b9cbaac662c828 |
| host_summary.json | 5f905ca691f24eee5d51ebd843456ef4b4788f33594224d7e620854066be3b3c |

累積：wall1565.665976s、V4 forward164、Tiny5711（共通5875/7100）、MPC1、
snapshot11、powered7/11、powered sim269.989995/320s、log91388595bytes、active=null。
追加4回はすべて未消費。今回のwall27.395917s、V4 forward40を正確に追記。
cutoffは自動延長しない。

## 未完了と次に必要な作業

「走行中の新観測→V4更新→PP実送信→移動後停止」は未完了。
先に、未改変AWSIMで実際に得られる情報へ結合した周辺監視方式を整備する必要がある。
現AABBの包含判定や前方LiDAR全車体可視条件を、そのまま緩和する案ではない。
独立した接触/逸脱/可動物体監視と停止根拠を実装する範囲・情報源を決める必要がある。
同時に新観測→fit→pose join→senderの期限内完了を診断し、期限を延ばさず処理順/負荷を見直す。
今回のPP接続だけでこれらの監視が完成したとは扱わず、駆動段階は保留。
