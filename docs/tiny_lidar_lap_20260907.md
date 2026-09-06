# TinyLidarNet：未改変AWSIM・固定配布重みの有限試験

## 目的と起点

期限は2026-09-07 10:00 JST。09:30以降は構成固定、09:50までに終了へ進む。
Windows正本 `E:\workspace\e2e_lite_transfuser`、branch
`codex/windows-wsl-training-sync`、開始HEAD
`16dada746c2946b8a3287899077fcba7426c7656`、開始working tree clean。
originは `https://github.com/fis-teria/aichallenge_lite_transfuser.git`。
V4/S1/参照fit/MPC/学習は休止。これらのモデル入力・gateはTinyに流用しない。
自動pushなし。旧結果、dirtyな別repo、他者processは変更しない。

## 公式版・推論経路

[公式手順](https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/en/ml_sample/tiny_lidar_net.html)
を参照するが、元Makefileやsensor変更手順は実行しない。
公式 `AutomotiveAIChallenge/aichallenge-racingkart` の
`1f54dff995d02625566341f9e1be1c39369224f2` にある
`aichallenge/workspace/src/aichallenge_submit/tiny_lidar_net_controller` を使用。
既存hostの同packageはこの公式Git tree/blobと一致し、package内のdiffなし。
元repo HEADは `4af395eee10f928c7fc7225760adfa04c4c07ff4`、151件の既存dirtyを保持。

重みは `ckpt/tinylidarnet_weights.npy`、602491 bytes、SHA256
`7a3f2702fe652a14970710aefde775d5328105d1a5370d4abf1fc88611043963`、
Git blob `c5d09d84048a37480041f51e208bb6b6cfe7aa81`。
weight最終変更commit `b0ddb3f8806a50188e9ee9f1780c7c1cc543ca47` は180度化。
公式packageを専有出力へコピーしread-only mount。モデルファイルをGit追加しない。

`OfficialTiny` は固定SHA確認後のみ公式object-npyを読む。
公式`TinyLidarNetNp(750,2)`の全18 parameter key/shape、float32/finiteを照合し、
公式loaderで全配列がそのまま代入されたことも照合する。推論は別processの
公式`TinyLidarNetCore.process`。独自network/再学習/部分loadなし。
official coreの`fixed acceleration`のみ0へ明示設定し、返る加速度を使用しない。

| 実field/実装 | Tiny契約・送信先 |
|---|---|
| `/sensing/lidar/scan` / `LaserScan.ranges` | float32[750], angle_min=-1.5666074753rad, increment=+0.004188789986rad, sensor range=[0,25]m。frameは実観測を記録し途中変更は拒否 |
| 公式`_preprocess_ranges` | NaN→0、±inf→30、[0,30]clip、750点へ公式index選択/pad、30で正規化。既存scan750なのでresizeなし。独自前処理なし |
| 公式`process`戻り値 | (未使用fixed加速度, steering rad)。出力操舵を補正/平滑化しない。絶対値30deg超・非有限なら停止 |
| `VelocityReport.longitudinal_velocity` | base_link, m/s。速度目標2.0、上限2.4m/s。`clip(2*(target-v),-1,0.6)` m/s²で実加速度を送る |
| `AckermannControlCommand.lateral.steering_tire_angle` | Tinyのradを同符号で送信。AWSIM内部は -rad×57.29578でUnity角度へ変換 |
| `longitudinal.acceleration` | AWSIM `VehicleRosInput.TryGetAckermannLongitudinalInput` がSI加速度として解釈。consumer上限[-3,1.37]より狭くする |
| `longitudinal.speed` | consumer不使用。0を入れるが停止/速度制限の証拠にはしない |
| `/awsim/control_mode_request_topic`, `/control/command/gear_cmd` | Bool true, GearCommand 2。単独supervisorのみが送信 |
| `LapCount.OnHitEnter` | `Section line hit`の順序と`Lap completed`の実ログ。位置近接・GNSS累積を完走条件にしない |

2m/sは静的車両上限より十分低く、停止加速度1m/s²なら理想停止距離2mに入力/処理遅延分を加える必要がある。
実測停止が成立するまでは安全性を保証しない。scan前方しかないことは未知領域freeの意味ではない。
GNSS/正解route/classic steering/MPCを操舵生成に使わない。

## 隔離・停止

専有simulator host `graneple@192.168.3.10`。既存imageを使用し、AWSIM本体、DLL、
level/asset、vehicle yaml、元起動scriptのhashを前後確認。元scriptをread-only mountし
`run_simulator.bash dev`を実行。sensor/scene/physics値は変更しない。
新composeはsimulator network=none、Tinyはそのnetwork namespaceのみを共有。
privilegedなし、ALL cap drop、private ipc、no-new-privileges、GPU以外deviceなし。
実inspect・現在hostname・loのみ・routeなし・serial/CANなしを起動前確認。
実consumer `awsim_d1` のGID/QoS、control/mode/gearの競合publisherなし、
actuation/emergency別経路のpublisherなしを確認し、稼働中も再確認する。

唯一の制御publisherは`run_tiny_lidar_dev.py`。model workerにはROS/controller接続なし。
host監視のARMを最初の送信前に要求し、heartbeat750ms失効、scan/state欠損、推論失敗、
操作wall500ms/sim350ms失効、overspeed、異常操舵、clock reset、logger失敗で停止。
modelが固まってもsupervisorは独立。supervisor停止でもhostが所有simだけpause→KILL。
以前の同host pause後KILL検証を再利用し、**unpauseは一切しない**。
停止は実速度の新規報告が0.03m/s以下で0.5sim秒以上続くことを観測。
初めから静止、移動後制動停止、host pause/KILLを分ける。

## 予算と進行

既存共通budgetをそのままロックして使用。開始消費はwall1119.29052884/3600秒、
V4 forward124/3000、MPC1/6000、snapshot7/16、powered0/3、powered sim0/180秒、
log53527759/536870912bytes。過去のV4 forward124を書き換えず`tiny_forward`を別記。
保守的にV4+Tiny合計も既存3000以下に制限。不明消費は予約上限で課金。
各駆動は既承認60sim秒上限を維持。周回は54sim秒で未完なら制動へ移り残6秒を確保。
60秒以内に一周できるとは主張しない。不足なら必要な予算差分を報告し追加承認が必要。
stationary→short(8sim秒+停止)→lapの順。失敗時は直接関係する修正のみ、旧attempt保持。

## 再現コマンド

Windowsで今回差分commit後に、未改変`tools/sync_to_wsl.ps1 -CheckOnly`と通常同期。
同期のDataset操作は固定root `datasets/processed/aic_real_dataset_v2` の`test -d`のみ。
Dataset内容/raw/V4 checkpointの読取なし。限定検証はWSL native repoから
`bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_tiny_lidar_sim.py`。
`TINY_OFFICIAL_PACKAGE`を専有コピーへ設定すると、公式全parameter/前処理だけ追加確認しforwardしない。
全pytestはV4/学習系へ対象を広げるため実行しない。

Windows git archiveをSHA別の新規remote専有directoryへ展開して実行する。既存dirty checkoutへ上書きしない。
以下の値は実行時に確定して全commandを保存する（元racing-kart Makefileではない）。

```bash
make dev DEV_CONTROLLER=tiny TINY_PHASE=stationary TINY_WALL_SECONDS=120 \
  SIM_REPO=/home/graneple/git/autononous_ai/aichallenge-racingkart \
  TINY_PACKAGE=/home/graneple/e2e_autonomous/tiny_lidar_lap_20260907/official/tiny_lidar_net_controller \
  XVFB_ROOT=/home/graneple/e2e_autonomous/spatial_sim_e2e_20260906/xvfb-root \
  TINY_OUTPUT=/home/graneple/e2e_autonomous/tiny_lidar_lap_20260907/ATTEMPT_UNIQUE \
  TINY_COMMIT=WINDOWS_COMMIT_SHA \
  TINY_BUDGET=/home/graneple/e2e_autonomous/spatial_run_continuation_20260907/budget.json
```

## 最小証拠・未検証

現時点でこの文書は実装契約であり、走行成功の報告ではない。
実行後に別results文書へ全attempt、SHA、予算、失敗/変更、停止、lap結果を追記する。
生ログは保存し、入力ID/header/受信→forward→要求/送信→速度/操舵stateを結べる形にする。
sensor/tensor全保存は既定にせずhashとmetadataを残す。hashだけで入力再現可能とはしない。
Judge/実画面の有無、接触telemetry、逸脱、pause/reset/介入を明示。診断図を実画面と呼ばない。
レビューZIPはREADME_REVIEWとmanifest、固定コード/設定、全attemptログ、出典を含める。
独立監査済み・実車安全性・競技採用保証を意味しない。
