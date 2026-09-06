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

その後ユーザーの追加指示でAutoware/RVizも終了。
Autowareは親run scriptへSIGINT→既存のlaunch協調終了、RVizは確認済みPIDへSIGINT。
2026-09-06 12:17:31 UTCにAutoware container終了、残るRViz専用の空containerも停止。
削除・自動再起動設定の変更はしない。

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

固定12配列の追加確認（前回のrow集合を固定。選び直しなし）:

```bash
tools/with_wsl_training_lock.sh .venv/bin/python tools/evaluate_constrained_reference_v4.py \
  --packet /mnt/e/workspace/e2e_lite_transfuser/tmp/spatial_v4_validation_review_20260906_153a22a.zip \
  --config configs/control/spatial_sim_e2e_v4.yaml \
  --output /home/thistle/e2e_autonomous/runs/spatial_sim_e2e_20260906/<commit>/fixed12
```

この確認のrear=base原点は合成前提であってlive frameの補完ではない。
既存receipt/policy/first_error回帰は runtime/bootstrap の2fileを該当`-k`に限定する。

## 今回の実測結果・未完了境界

最終コード版: `b3b90b91ee3e82d5b6d0406fd6546528d5db1d0b`。
新run: `/home/thistle/e2e_autonomous/runs/spatial_sim_e2e_20260906`。
remote evidence: `/home/graneple/e2e_autonomous/spatial_sim_e2e_20260906`。
Windows小成果物: `tmp/spatial_sim_e2e_20260906`。

| 判定 | 結果 |
|---|---|
| IMPLEMENTATION_COMPLETE | false。幾何・速度・dispatch純粋helperまで。live controlled wrapperは未完了 |
| SIM_ISOLATION_VERIFIED_FOR_SELECTED_INSTANCE | true、無操作probeの当該instance/時点限定 |
| LIVE_V4_INFERENCE_EXECUTED_IN_SIM | false、固定forward 0 |
| MPC_CONTROL_PUBLISHED_TO_SIM | false、MPC操作送信0 |
| E2E_SIM_CLOSED_LOOP_EXECUTED | false |
| LOW_SPEED_RUN_AND_STOP_PASSED | false、powered episode 0 |
| REALTIME_AT_1X | NOT_TESTED。sensorの時刻だけでV4/MPC実時間性を認定しない |
| 実車/Safety認証/競技全般 | 未検証・未許可 |

無操作AWSIM起動を3回実施した。これはpowered episodeではない。
1回目は画面なしVulkan初期化直後のSIGSEGV。
2回目は専用Xvfbで画面初期化を通過したが、DDSのlo二重選択エラー。
3回目はcontainer network-noneを維持し、ROS_LOCALHOST_ONLY=0で成功。
同一実行物、scene `AIChallenge2026` / `GoKart1` / Unity `2022.3.49f1`。
timeout上限は各50/50/70 wall秒。停止後に設定を切替え、失敗ログを残した。

3回目の無操作probeはコード`df55c99950eb8a92704f996fcf0d17d991c7dcac`、
計測wall 12.2741秒でCamera 114、LiDAR 240、velocity/steering各343、clock 3600 callback。
payload保存0、checkpoint読取0、固定forward0、制御/モードpublish0。
Cameraは384×256/bgr8/camera_optical_link。LiDARは750点/lidar frame、
angle_min=-1.5666074752807617 rad、increment=0.004188789986073971 rad、0..25m。
旧V1設定の角度値とは一致するが、今回許可した小資料だけからV4全学習runの角度parityは証明していない。
`/control/command/control_cmd`は実awsim_d1 subscriber 1、publisher 0。
`/awsim/control_mode_request_topic`も実awsim_d1 consumerを確認。
依存実値: numpy1.26.4、torch2.3.1+cu121、scipy1.15.3、RTX4060 Laptop 8GiB。

**現時点で走行を止める直接の境界は、時刻付きego poseと後輪基準frameの未結合。**
選択instanceの`/awsim/ground_truth/vehicle/pose`にpublisherなし。
domain191全topicと同じ隔離namespaceのdomain0も照会したが、このpose/TFは見つからない。
V2X代替も静的確認：`ShouldIncludeInDomain`は自車domainから自車を除外し、
messageはpositionのみでorientationを持たないため、そのまま自車poseへ使えない。
GNSS/IMUは存在するが、simulator vehicle root→base_link→後輪中心、センサー外部パラメータの
実frame結合が未証明。URDFの寸法と見た目のmesh原点を後輪中心の証明として流用しない。
さらに想定`/awsim/ground_truth/on_collision`のpublisherもなく、衝突なしとは判定できない。
`/awsim/status`はLapCount.UpdateSimulatorStatusの7値（時間/周回/区間/timeScale/boost）であり、
poseやcollisionの代用にはならない。
こうした境界を解消する既存設定・ビルドの指定、またはsim側の小規模計測追加方針の確認が必要。

固定12の幾何passは同じ集合で2回（fa058d5、最終b3b90b9）。共に6受理/6拒否。
受理6件の偏差上界0.02957..0.04190m、参照長1.99299..2.00556m。
最終版のfit時間0.01351..0.09454 wall秒。NMPC/forwardを含む時間ではない。
拒否6件のうち境界値0.10000000000000096/0.10000000000008125mも厳密に拒否した。
閾値緩和、追加scene選択、再学習、補正rawの上書きなし。
幾何受理は空間/車体/追従/停止の合格ではない。

限定test: df55c99でcontrol関連42 passed、runtime/bootstrapのreceipt/policy/first_error43 passed。
340f236のstop-margin修正後43 passed、最終b3b90b9はcontrol関連45 passed。
累計173 test executions（再実行を含む）。異なる版・再実行を別の独立合格件数にしない。
停止中の正加速度+遅延を停止距離0としない修正も含む。
全pytest・学習test・実ROS controlled wrapper testは未実施。
unit内のfake forward/solver正確回数はinstrumentationなしのためUNKNOWN、実fixed forwardとは別。

実移動距離/到達最大速度/連続停止1秒/衝突/逸脱: UNKNOWNまたはNOT_EXECUTED。
保存された最初4 velocity値は0m/sだが、これを試験全体の停止完了証明にしない。
MPC cycle traceはunit以外0、要求/送信/適用の走行traceなし。動画なし。
代表図は保存raw→reference図であり、実simulator画面ではない。

予算は全attempt共通。powered0、powered sim秒0、fixed forward0、offline control cycle0、
unit以外MPC solve0、幾何reference optimizer24。
sim試験wall予算は専用container全生存時間1005.284秒を保守的に計上（起動待ち・offline待ちを含む）。
実sim秒の全attempt合計はUNKNOWN。powered0から無操作sim稼働まで0秒としない。
上限3600 wall秒/180 powered sim秒/3000 forward/6000 MPC solveを超えていない。

既定同期によるDatasetルートの存在確認を実施（CheckOnly/通常sync各4回）。
Dataset内容・raw・既存sensor・checkpoint本体の読取りは未実施。
新simulatorの許可されたsensor callbackは上記の通り実施。sensorアクセス完全0とは記載しない。
固定checkpoint strictload map/全state前後hashは未実行なのでUNKNOWN。期待hashだけを成功証拠にしない。
新forwardは入力/制御frame gate未完了のため実施しなかった。

12:25:43 UTCに専用probe container/Xvfbも終了。確認時点でAWSIM/Autoware/RVizプロセスと
稼働Docker containerはなし。既存container・ソース・出力を削除せず、再起動可能な状態で残した。
Git pushなし。今回をE2E走行完了として閉じず、境界付きの部分成果として提出する。
