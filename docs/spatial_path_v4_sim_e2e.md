# V4 simulator integration — current bounded work

開始HEAD: `3e7662d1874ca56f3ee2f552a1ef890cfea461ce`。
依頼: 固定V4→制約付き参照→既存NMPC→新観測。現時点で実装完了・走行成功とはしない。
ユーザー指定hostは `graneple@192.168.3.10`。自動pushなし。

## 実装

- `constrained_reference_v4.py`: raw float32[20,2]/160 bytesと旧gate結果を保持。
  7舵角knotのSLSQP shooting一方式、最初のcuspで一度だけprefixを選ぶ。
  raw arc上の単調対応。点間も含む偏差上界は標本誤差+最大検査間隔で0.10m以内。
  初期接続は別項目。現在rear pose/yaw/steeringを初期条件とする。
  これは幾何参照のみ。current LiDAR/footprint未確認なら制御採用しない。
- `rolling_horizon`: 現v/前回a/遅延/進行を引継ぎ、path更新をrest-to-rest開始時刻0にしない。
  permission/Safetyをfinal capへ含める。旧HOLDのfinal cap表示も0へ修正。
- `sim_dispatch_v4.py`: 出力単位、一度だけのrate積分、送信後のみの履歴receipt、
  snapshot/epoch/時刻/記録/通信/監視/clearance再検査の純粋helper。
  **実publisher、独立watchdog、実入力→制御wrapperの代用ではない。**
- `probe_spatial_sim_v4.py`: network-none専用containerの有限・無制御probe。
  最大20 wall秒、各role最初4messageの小metadata/最大256 timestampだけ保存。
  checkpoint、Dataset、教師、画像/range payloadの保存なし。

## 確認済み境界

既存host-network/privileged AWSIMは当初無操作で保護。
追加指示「SSH先のAWSIMを閉じてから続ける」により、確認済みcontainer内PID79の
AWSIM executableだけへSIGTERMを送り、2026-09-06 12:13:29 UTCにcontainerの終了を確認。
Autoware/RViz停止指示は出していない。

当タスク専用container `codex-v4-e2e-20260906` は network none / private IPC /
cap-drop ALL / no-new-privileges / GPUのみ。外部route、CAN/serial、host X11/socket mountなし。
旧AWSIM実行物をread-only mount。作業記録のみ別bind mount。
最初の無操作起動はVulkan初期化後SIGSEGV。小さなXvfb依存を当タスクdirectoryへ取得・展開し、
既存image/ホストpackageをupgradeせず隔離画面を作った。
2回目は画面初期化を通過したが、ROS_LOCALHOST_ONLY=1と明示lo指定が重複しDDS失敗。
失敗ログを保持。旧版の検証済み結果で補完しない。

実行物 `Assembly-CSharp.dll` SHA256:
`859e5560dbffd7d0833cd1fe5eb1f36b87fa8d22f6e45eb0e1203d04a1ea6d13`。
ILSpy 9.1.0.7988による静的読解でVehicleRosInputのAckermann consumerを確認。
舵角rad→Unity符号反転deg、longitudinal accelerationを[-3,1.37]へclamp。
speed fieldはこのconsumerで適用されないため、将来利用してもdesired reference意味のみ。
vehicle.yamlはmaxSteerAngle=30deg、URDF側0.64radより厳しい。
base_linkと後輪中心の対応、scene内footprint・sensor extrinsics、学習時の角度配置との一致は
現時点でUNKNOWN。configのnullを推定値で埋めて制御しない。

## 有限検証手順

Windowsで今回変更をcommit後、既定 `tools/sync_to_wsl.ps1 -CheckOnly` →通常sync。
既定syncは固定Dataset root存在判定だけ。Dataset内容・rawは読まない。

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q \
  tests/test_spatial_sim_e2e_v4.py tests/test_spatial_mpc_v4.py \
  tests/test_spatial_tracking_sim_v4.py
```

probeは実際のcontainer inspectの固定IDと照合し、専用container内の同一sourceでのみ実施。
`ROS_LOCALHOST_ONLY=0`でもnetwork none/loopback以外なしを再検証する。

```bash
python3 /evidence/probe_spatial_sim_v4.py \
  --inspection /evidence/container_inspect.json --container-id <actual-full-id> \
  --output /evidence/probe_03.json --seconds 12
```

未完了: current-space footprint、実rear transform、versioned同期/実sent履歴接続、
独立停止watchdog、実sim controlled wrapper、固定checkpoint strictload/新forward/走行・停止。
幾何単体testやhelperの存在からこれらを完了にしない。
現在の既定configはdisabled。実車/本番昇格・学習・Dataset化は未許可。
