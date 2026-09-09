# 車速・操舵Odometry有効化: AWSIM試験22

今回のユーザー依頼により、新方式を有効にして既存Pure Pursuit走行＋V4 shadowを1試行する。
外側300wall秒・駆動240sim秒以内、公式1周Finishで終了。前run21の使用量を保持して引き継ぐ。
V4制御ではなく、既存PPのみが制御する。AWSIM本体・scene・既存dirty checkoutは変更しない。

## 有効化根拠

2026-09-10、SSH先の現level1を既存隔離UnityPyで読取。
SHA256 `9ab2e1e8865c02885594e0bbdde372302530f047b5a2e89c455be6a18f3e090b`。
GoKart1のWheelCollider 1227/1232の後輪中心はroot座標で横x=±0.635、前後z=−0.484m。
前輪1229/1231は平均横x=0、z=0.603m。rootまでの中間Transformは回転単位・scale1、横並進0。
したがって後輪中心線に対するVehicle rootの横offsetは0m、前後輪間隔は約1.087m。
Vehicle.LocalVelocityはrootのInverseTransformDirectionで得られ、publisherはこれをbase_linkとして報告する。
root=後輪中心とは主張しない（rootの縦offset約0.484m）。測定vyを保持するため縦offsetに伴う横速度を捨てない。
静的設計寸法でありタイヤ滑りや動的校正まで証明しない。

`tmp/v4_pp_shadow_22/geometry.json`と読取scriptに証拠を保存。
同runのlive.template.jsonにwheelbase_m=1.087、reference_left_offset_m=0、証拠IDを設定。
他の既定configへ無条件に適用しない。ROS合成smokeも車速＋操舵入力へ更新する。

## 実行手順

Windows commit → 既定CheckOnly/WSL同期 → lock付き限定テスト。
Windows git archiveをSSH先専有`/home/graneple/e2e_autonomous/v4_pp_shadow_22`へ配布。
Humbleのnetwork noneでbuild/合成ROS確認の後に、同runのrun.pyで通常make devを実行する。

```bash
timeout --signal=TERM --kill-after=15s 300s python3 /home/graneple/e2e_autonomous/v4_pp_shadow_22/run.py
# 内部: make dev CONTROL_METHOD=pure_pursuit CAPTURE=false ROSBAG=false AWSIM_LAPS=1 RUN_ID=v4_pp_shadow_22
```

試験結果・実行版・停止確認は後記。自動pushなし。

## 実行結果

- 実行commit `d23958f8a77630c837c24a8c80d521adf80f8326`、Windows archive SHA256
  `dbc1198b3260eb32c17e2a8b914c9253a5bd31b567f7c5ecc55b29cac7c32d27`、SSH先と一致。
- 既定同期CHECK_OK/SYNC_OK。Dataset固定ルート存在確認を実施、内容の確認はしていない。
- WSL lock限定pytest 35 passed / 4.22s。前実装版全pytest 1776 passed/4 skipped。
- Humble build 1package / 1.20s、network noneの合成ROSで11出力、CONFIG_OK_NO_INFERENCE。
  初回build後の合成ROSはCycloneDDS設定mount不足で失敗。既存lo設定をread-only mountして再実行し成功。実装やAWSIMを変更して隠していない。
- make devで実走行。公式finish **100.129997761sim秒**、最後のStart 18.204999593から**81.925sim秒**。
- 最大車速4.8341m/s（約17.40km/h）。PPだけの完走であり、V4制御・無接触・E2E部門適合の証明ではない。
- local odometry/入力traceとも**2868件**、local fault 0。
- FORWARD_STARTED **749**、PLAN **747**。推論継続は確認できたが追従採用ではない。
- 元heading異常値が34.019999239秒で−1255.8997rad/s、58.589998690秒で+1255.9104rad/s。
  同時刻の新推定はそれぞれ+0.82563rad/s、−0.82832rad/s。元headingは積分せずログに保存できた。
- 全local poseの最大単一更新yaw変化は**0.03684rad（約2.11度）**。以前の約179度ジャンプはこの試行では出なかった。
- V4のraw ego heading入力は今回変更していない。上記2時刻に`current ego features must be valid`で2推論拒否が残った。局所pose対策だけでV4入力全体の問題が解消したとは言わない。

## 精度: そのまま採用できない

今回保存したEKF yaw/XYで5.004999888sim秒の初期座標を合わせ、header時刻を補間した2718組では
位置差RMSE **16.79m**、最大28.00m、終端18.98m。
さらに前回と同じく20–22秒の初期移動だけで回転を合わせる診断でもRMSE **15.96m**。
したがって今回の大きな差は初期方位合わせだけでは説明できない。

飛び値を除いた走行中（|vx|>1m/s、|raw heading|<=2rad/s）の2502入力でも、推定yaw rateと元報告値の差はRMSE0.05569rad/s。
元報告・EKFは真値ではないが、bicycle近似と車両の実際の旋回に差がある可能性を検討すべき結果。
タイヤ滑り・操舵から旋回への関係・車速基準点/時間対応の寄与は未確定。推定値への係数合わせや追加試行はしていない。
**接続・異常値非積分は確認できたが、局所Odometryの精度改善は達成していない。運用採用へ昇格しない。**

## 終了と保存

停止理由SIMULATOR_FINISH、wall130.092s。所有AWSIMをfreeze/KILL、Autoware終了。
result.jsonは既存終了処理競合によりFAILED/OBSERVER_EXITのまま保全し、成功へ書き換えていない。
終了時TRANSPORT_CLOSEDも記録。ブレーキによる停止成功とは扱わない。
終了後docker ps、ROS/AWSIM/RViz processで残存なし。level1/DLLの前後hash一致を確認。

`tmp/v4_pp_shadow_22/`にgeometry、source、config、run.py、budget、全evidence、summary.json、解析scriptを保存。
旧予算を引き継ぎ、今回は追加再試行なし。実画面動画は未取得。
解析再現: `C:/Python310/python.exe tmp/v4_pp_shadow_22/summarize.py`。
