# E2E Path-to-Control Bridge: 現行監査とshadow実装

## 版・権限

起点 `40c77fd71806fe9e3330de4d3084cbbf16b28cd0`、Windows clean。
origin https://github.com/fis-teria/aichallenge_lite_transfuser
branch `codex/windows-wsl-training-sync`。
実装実行版 `d02f06fdec8c5245b6954687689a291b1cd80dd8`。
既定Windows commit→CheckOnly→同期→WSL lockの順に限定検証。
README_ROS2.mdはHumble手順を記載。ただし今回WSLのROS_DISTROは未設定、
`/opt/ros/humble`の存在確認でも確認できず、現在有効なROS distroはUNKNOWN。
sim hostには接続せず、現在の外部node graph/制御権限はUNKNOWN。
今回追加のコードにはROS依存・publisher・actuator・engageが存在しない。

## 現行契約の監査

| 対象 | 現行file/型/fieldと確認内容 |
|---|---|
| `/shadow/e2e/path_raw_lead` | checkout内に生成/publish/remapなし。message・後処理・時刻契約UNKNOWN |
| `/shadow/e2e/path` | 同上。別プロジェクトにあると仮定せず、今回は接続しない |
| V4実出力 | `runtime/spatial_runtime_v4.py` の `SpatialRuntimeV4`相当のforward処理が `[1,20,2]`をsnapshot。`output.model_xy_m`、`output_id`、frame=`base_link`、pose_reference_point=`BASE_LINK_ORIGIN` |
| V4形状 | `models/spatial_path_diagnostic_v4.py` のforwardは `[B,20,2]` metres。`runtime/spatial_sim_worker_v4.py`でnominal_s=0.1～2.0mを20点へ対応。配列の順序を維持。実際の点間隔・実弧長は等距離とは限らない |
| V4速度/姿勢/時刻 | rawはXYのみ。速度・加速度・到達時刻・予測姿勢なし。名目sから速度・時間を作らない。原点を先頭へ追加しない |
| V4観測時刻 | `runtime/spatial_recording_v4.py`の `p['output']['t_obs']` は最新sensor frameのcamera header。取得時刻の真実性は別。各点に到達時刻なし |
| V4生成時刻 | `timing.inference_start/inference_api_return/snapshot_ready` はmonotonic系。`snapshot_ready`とt_obsのclockを混ぜない。別domain間の差を遅延として計算しない |
| V4公開 | `ros2_ws/src/aic_e2e_runtime/aic_e2e_runtime/spatial_path_shadow_node_v4.py` は入力専用wrapper。経路publisherなし。`config/spatial_path_shadow_v4.param.yaml`はenabled=false、ROS environment UNKNOWN |
| 既存V3 Path | `inference_node_v3.py`がnav_msgs/Pathを相対topic `predicted_trajectory_path`へpublish。headerと全PoseStampedのstampはimage.header.stamp、orientation.w=1。姿勢推定ではない |
| V3速度 | 同nodeの `trajectory_speed_publication` が同forwardのtrajectory_xy/trajectory_speed_mpsを整形し、Float32MultiArray `predicted_speed_profile`へ。標準のPathと別topicでplan ID/stamp結合は保証されないので新Bridgeは購読しない |
| V3後処理 | `trajectory_path_publication`を介した点列生成。V3 timed planとV4 spatial planを混同しない。今回V3契約への一般対応は対象外 |
| state/TF | `tools/run_spatial_sim_dev_v4.py`のvelocity/steering/IMU/fix入力、`runtime/spatial_sim_adapter_v4.py`のbase_pose_from_gnss_imu/await_control_join。GNSS/IMUからposeを作るsim経路。新BridgeはOdometry topicを推測せず時刻付きVehicle/PathPoseを受ける |
| Pure Pursuit | `control/waypoint_controller.py: control_from_waypoints`はtire steering radと加速度m/s²。これを明示Limitsで再利用、既定寸法は使わない |
| 既存PP統合 | `control/spatial_live_pp_v4.py: propose`はsim用。独自RUN/HOLD gateやrolloutと結合されているため、新Bridgeへ丸ごと流用しない |
| MPC/縦制御 | `control/spatial_mpc_v4.py`、`spatial_speed_profile_v4.py`、PPの比例速度制御が実在。新規MPC開発なし。新Bridgeの速度は明示試験方針＋曲率＋残長制限、P制御を再利用 |
| Supervisor | `spatial_sim_guard_v4.py: MotionEvidence/OperationLease`、`sensor_local_monitor_v4.py`が実在。今回は入力追従妥当性のみ、障害物監視を重複実装せず既存にも接続しない |
| command publisher | V3のFULL_CONTROL/TRAJECTORY_AUTHORITATIVE条件で `nominal_control_cmd`、EXTERNAL_CONTROLLER条件で `shadow_external_control`。sim runnerにはAckermann送信がある。今回これらは変更・起動しない |
| launch | spatial_path_shadow_v4.launch.py、V3 external_controller_shadow.launch.py等は既存のまま。新launchなし |
| replay/tests | 専用pytestの人工Planで実行可能。実bag内容の探索/読取なし。実bagの有無・契約適合はUNKNOWN、bag replayはNOT_RUN |

上表のsrc相対pathは `src/aic_transfuser_lite/` 起点（ROS/toolを除く）。
requested topicが見つからないのでROS Path Bridge完成とは報告しない。

## 新規実装と利用契約

新規 `src/aic_transfuser_lite/control/path_control_bridge.py` と
`tests/test_path_control_bridge.py`。既存model/raw/ShadowMode/controller/supervisorは未変更。

`from_v4_record(record, expires_s=...)` → immutable `Plan` →
`ShadowBridge.accept(plan, PathPose, now_s=...)` → 周期的 `tick(..., Vehicle)` の順。
rawのdict/listへ書き込まない。model推論を呼ばない。

- Plan: source/id、観測clock/epoch/stamp、生成monotonic、expiry、frame、原点、XYを保持。
- PathPose: planと同じID/stamp/clock/epochのbase→固定local pose。
- Vehicle: 時刻付きbase pose、速度m/s、tire steer rad。
- Limits: 出典、fixture専用区分、wheelbase、rear/base差、制約、期限等を全て明示。
  実設定の既定値なし。fixture_onlyを非fixtureモードへ流用すると拒否。
- XY軸はlocal/baseのx前方・y左を前提とする明示契約。実topicとの一致は未検証。
- rear_x_in_base_mで追従器の原点を後輪軸中心へ移す。steering wheel角へ変換しない。

等時間軌道は `TIMED_TRAJECTORY_UNSUPPORTED` で無効化。停止の重複時系列点を削らない。
幾何経路のみ連続重複を除き、線分補間で再サンプル。平滑化・lane吸着・fitなし。
元頂点で接線方向・曲率を計算し、不連続・折返し・操舵不可能を補間前に拒否。
raw/参照の双方向点→折線最大距離を記録する。補間が角を跨ぐ差も測る。

V4には速度がないので `EXPLICIT_TRIAL_POLICY` を明記。速度capから、曲率横加速度と
`v*delay+v²/(2b)<=remaining` による残長制限を適用。定数bのモデル内制動方針であり
実制動性能や停止保証ではない。明示speed=0とsource/idがあれば停止目標として扱う。
モデル速度の配列対応・時系列速度は未実装。scalarのPlan一体速度のみ対応。
横は既存PP、縦は既存比例速度制御。操舵速度は最新状態からの明示上限で制限し記録。

無効入力はplan/cacheを破棄し、`valid=False, command=None, expires_s=now` を返す。
直前指令へfallbackしない。これは実車ブレーキ送信ではなく、shadow指令の失効記録。
時計巻き戻り/epoch変更時はcacheと順序情報を破棄し、改めて新Planが必要。
入力更新とtickは独立。毎tickでstate・plan・周期・制約を再判定する。
core停止時は次の無効イベント自体が出ないので、将来のconsumerにはexpiry確認と
独立watchdogが必要。今回はconsumer/publisherはない。

## 検証・再現

Windows commit後、既定syncのCheckOnlyと通常syncを実施。
固定Datasetルートの存在確認のみ実施。Dataset内容・raw・checkpointは読取なし。

```powershell
.\tools\sync_to_wsl.ps1 -CheckOnly
.\tools\sync_to_wsl.ps1
```

WSL（毎回新しい出力ディレクトリを使う）:

```bash
mkdir -p runs/path_bridge_02
bash tools/with_wsl_training_lock.sh \
 env PATH_BRIDGE_TRACE_DIR=/home/thistle/e2e_autonomous/e2e_lite_transfuser/runs/path_bridge_02/traces \
 .venv/bin/python -m pytest -q tests/test_path_control_bridge.py \
 tests/test_spatial_live_pp_v4.py tests/test_sensor_local_monitor_v4.py \
 --junitxml=runs/path_bridge_02/junit.xml \
 > runs/path_bridge_02/stdout.log 2> runs/path_bridge_02/stderr.log
```

初回7270f58: 20 passed / 1 failed、0.57s。
横偏差15cm/方位0.05rad、lookahead0.5m、wheelbase1mのfixtureが要求する操舵が
上限0.6radを超えた。coreを緩和せず拒否testを残し、正常fixtureを5cm/0.02radへ変更。
最終d02f06f: **78 passed / 3.20s**（Bridge23件、既存PP/局所core55件）。
正常: 直線/左右円弧/S、古いbody変換、複数tick、停止、偏差ありモデル内閉ループ。
異常: 空/NaN/短horizon、stale/future/order/reset、TF/state欠損、jump、曲率、速度世代不一致等。
全pytestは今回未実施。ROS/bag/simulatorはNOT_RUN。

`tmp/path_bridge_results/path_bridge_01/`と`path_bridge_02/`に生ログ、JUnit、fixture trace。
閉ループtraceには各tickの横偏差・方位偏差・操舵・速度・age・処理時間・期限を記録。
人工運動学モデル内の評価であり、実bag軌跡との差や実車の追従誤差ではない。

## 不足と次段階

1. 実際に接続したい2topicの生成repo/型/時刻・原点・後処理を特定。
2. 実車両のLimitsと出典、state/TF経路、生成clock間の対応を確定。
3. 必要ならPlan/速度の同世代結合とROS shadow専用sinkを追加。独立周期とexpiryを維持。
4. 既存局所停止監視・独立supervisorとの接続は別作業。入力妥当性を非衝突証明にしない。
5. 実bag/無駆動shadowを確認し、専用simulatorの有限予算を別に設定して閉ループへ進む。

今回の成果は確認できたV4レコード→参照→PPのin-memory shadow Bridgeと合成検証。
モデル再学習、新規MPC、実車接続・engage・制御権限変更・自動走行は未実施。
