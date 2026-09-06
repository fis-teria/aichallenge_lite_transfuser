# Lite TransFuser V4 + NMPC: make dev接続

2026-09-07継続版の実行結果は `spatial_path_v4_run_continuation_result.md`、
入力policy/終了処理の差分は `spatial_path_v4_run_continuation.md` を参照。
新観測64 forward（stable 46）を追加したが、参照受理/MPC送信/走行は未成立。

## 変更範囲

AWSIMのbinary/DLL/scene/asset/設定ファイルは変更しない。
このrepositoryにLinux用の **専用 `make dev`** を追加した。
SSH先racing-kart repositoryのMakefileそのものは変更・実行せず、
既存 `aichallenge/run_simulator.bash dev` と同じ既存AWSIM実行物を使用する。
したがって「従来のmake devをそのまま実行した」とは報告しない。

理由: 従来のtargetは既存classic controller/自動Startを起動するほか、
capture_run_fingerprintがsource/build/install全treeをhashする。
今回許可されていない別方式の重みやデータを読む可能性のある全tree走査を避け、
V4の固定ソース・固定checkpoint1本・選択simulatorの小さなidentityへ範囲を限定する。
既存の稼働構成・dirtyなSSH checkoutは上書きしない。

## 接続と安全境界

`make dev` → 所有Docker Compose project → 未改変AWSIM + 外部V4/MPC runtime。

- simulatorはnetwork none、runtimeはそのnetwork namespaceだけを共有する。
  host network、物理CAN/serial、remote車両bridge、Docker socketを渡さない。
  GPUだけを共有する。両containerは非privileged、cap-drop ALL。
- 原本simulator/scriptとV4コード・固定checkpointはread-only bind mount。
  専用Xvfbを使い、hostの画面・入力deviceへ接続しない。
- runtime supervisorが唯一の制御publisher。実awsim_d1 subscriberと競合publisherを確認した後だけ
  HOLD/gear/modeを送信する。pure_pursuit等の別controllerは起動しない。
- ML/MPCは別process。親は推論/solverを待たず、期限切れ時は過去の正加速度を再送しない。
  powered後の故障はラッチする。host runnerも親heartbeatを監視し、
  最初の送信前にARMEDを確認する。旧powered=falseのheartbeatも監視する。
  故障時および終了引渡し時は所有simulatorだけをpauseし、解凍せず終了する。
  正常終了はHOST_FREEZE_REQUESTED→hostのpause実確認→最大5秒runtime終了待ち。
  Docker pauseは自然制動による停止成功としない。
- 終了時には速度観測を続け、0.03m/s以下が1 sim秒続いたかを別記録する。
  これが成立しても、元々動いていなければ低速走行・停止成功へ昇格しない。

## 入力・経路

既存の固定step500 strict loader、9 tensors、SpatialInputV4、制約付き参照、
rolling speed、SpatialMPCを再利用する。旧private shadowは変更しない。
新scopeは `SIM_E2E_CONTROLLED_TEST`。

Camera 100ms grid / nearest LiDAR 30ms / ego bracket 50msは既存converterの
nearest/linear/angle interpolationを使用する。headerと受信・利用可能時刻は別保存。
750本だけでなく角度・rangeも確認する。V4全学習runとの角度parityは未証明。

command履歴は本試験の実送信receiptだけを `SIM_ONLY_POLICY_CHANGED` として使う。
nominal/final/appliedと同一視せず、過去性と利用可能時刻を保持する。
AWSIMが無視するspeed fieldはdesired-speed referenceと明記。
起動時のpaddingはmask=false。送信していないzeroを作らない。

GNSSの現在位置をUTM54Nへ変換し、現在IMUのorientationとscene外部パラメータを結合する。
heading_reference.csv、正解route、future pose、teacher maskは読まない。
poseは同じ原本のdelay設定/取付けに結合したsim限定誤差仮定とする。
取得物理時刻はUNKNOWN。forward後の右bracketは元期限内の制御用後結合だけに使用する。
raw float32[20,2]の160bytes/hashは保持し、参照は別field。
MPCへは観測時点で一度world化した参照を渡す。近傍の参照頂点によるprogressは
速度計画専用で、raw対応や実走距離の証拠にはしない。

## 現在の未成立gate

`V4_PHASE=run`だけでRUN根拠は成立しない。車体の保守的境界とposeの限定仮定は
同じ原本の静的資料へ結合した。現在車体と参照/rolloutの検査を分けたが、
障害物AABB内部の路面/壁分類、可動物監視、停止swept footprintは未成立。
衝突heartbeat/適用確認はMISSING/UNKNOWNのまま。閉ループ実装完了とは扱わない。
今回の実試験ではこの段階より先に通常履歴の制約付き参照が全件拒否された。

前方180度LiDARに対し車体の全矩形を点検し、未知領域をfreeにしない。
現在の車体領域自体を既知freeとする抜け道もない。
この保守的な検査は未知領域による拒否を起こし得る。
追加のsimulator改変・Safety条件緩和・走行許可をこの実装から推定しない。

## 実行

Windowsでcommit → 既定sync CheckOnly → 通常sync → 同一SHAをWSL lock下で限定test。
同期scriptは変更しない。Dataset操作は既存の固定root存在確認だけ。

```bash
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q \
  tests/test_spatial_run_continuation_v4.py tests/test_spatial_dev_connection_v4.py \
  tests/test_spatial_sim_e2e_v4.py
```

実simulator hostの固定ソース展開先で:

```bash
make dev \
  SIM_REPO=/home/graneple/git/autononous_ai/aichallenge-racingkart \
  V4_CHECKPOINT=/home/graneple/e2e_autonomous/spatial_sim_e2e_20260906/fixed_final.pt \
  XVFB_ROOT=/home/graneple/e2e_autonomous/spatial_sim_e2e_20260906/xvfb-root \
  V4_OUTPUT=/home/graneple/e2e_autonomous/spatial_sim_e2e_20260906/NEW_UNIQUE_RUN \
  V4_COMMIT=FULL_40_CHARACTER_SOURCE_COMMIT V4_PHASE=stationary V4_WALL_SECONDS=60 \
  V4_BINDING=ABSOLUTE_SELECTED_BINDING_JSON V4_BUDGET=ABSOLUTE_CUMULATIVE_BUDGET_JSON
```

`NEW_UNIQUE_RUN`とcommitは実値へ置換。既存出力を上書きせず、存在したら拒否する。
固定checkpoint期待SHA:
`0316692543a901d9d6b96718c5651d6739367aa851f83fb3a133c126ccc8919f`。
この1本のみを使用し、Windowsへ重み本体を保存しない。

## 証拠・予算

resolved config、compose spec/inspect、host/supervisor/workerログ、raw20点、
入力tensor hash/mask/履歴source、参照・MPC、送信receipt、GNSS/速度観測、
strict load map、全state前後hash、最大2 stable tensor snapshotを別runへ保存。

1回の入口はwall最大120秒＋host終了余裕。forwardはstationary40/run240、
RUN参照0のままなら40で終了。明示V4_FORWARD_LIMIT=1..240も可。
累積JSONのreservation/finishを全attemptで継続し、未解決予約なら次回拒否する。
欠落した終了counterは予約上限を計上する。現在の累積値は新しい結果文書参照。
元のpowered最大3/180sim秒、fixed forward3000、MPC6000、logs512MiBは引き続き上限。
固定値の許可fieldだけを動作証拠としない。今回の7判定は実ログから別報告する。

旧c48f823実行結果は `docs/spatial_path_v4_make_dev_result.md`（履歴）を参照。
実車・競技全般・Safety認証は未検証。
