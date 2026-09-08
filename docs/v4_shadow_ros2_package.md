# V4 shadow ROS2 node

既存ament_pythonパッケージ `aic_e2e_runtime` に `v4_shadow_node` と
`v4_shadow.launch.py` を追加した。別の独自runnerを起動窓口にしない。
既存MPC/controller launchは変更せず、並列のROS nodeとしてinclude/起動する。

```bash
colcon build --packages-select aic_e2e_runtime
source install/setup.bash
ros2 launch aic_e2e_runtime v4_shadow.launch.py config_file:=/absolute/reviewed.json
```

設定雛形はshare/aic_e2e_runtime/config/v4_shadow.example.json。
enabled=falseで配布する。実node名・Odometry topic・pose frame/根拠・command意味・
一意の出力ファイル・有限envelopeを確認して指定する。UNKNOWNを仮値で有効化しない。
既存固定step500 loaderのcheckpoint配置を再利用し、ランダム重みへfallbackしない。
ROS/PyTorch/既存Python依存は実行環境側に必要。環境更新は行わない。

親ROS processはsubscription・graph監視を担当し、別spawn childが既存
SpatialInputV4 / ShadowObservationJoin / ShadowSession / fixed loaderを再利用する。
queueは各64件。満杯・graph fault・reset・期限切れ・worker終了で有限sessionを終了。
終了時は所有childだけをjoin→TERM→KILL。ROS受信をmodel forwardで塞がない。
親でfaultを検出した後のchild結果は採用せず、node再起動で勝手に期限更新しない。
このnodeからAWSIM/controller/gear/modeを起動・変更・送信しない。

出力はJSONLの未補正V4経路と処理状態。車両制約が未設定なのでShadowBridge(None)を
使用し、追従commandを有効化しない。MPCへV4経路を入力する構成ではなく、
MPCが別途走行している間のV4 shadow記録用nodeである。
通常ROS timerのgraph監視に加え、既存monotonic freshness checksを維持する。
外側の120秒上限・AWSIM停止・累積試験予算は試験起動側の責任であり、本nodeの
追加だけで車両停止確認済みとはしない。

現AWSIM単独ではcommand publisherとOdometryがないことを前回実測済み。
外部制御器を含むlaunch側で提供する必要がある。GNSS/IMUによるpose生成は別途
確認が必要で、単なるtopic remapでOdometryに見せかけない。

限定tests：tests/test_v4_shadow_package.pyと既存transport/join/session/bridgeテスト。
実checkpoint読取・推論・AWSIM走行は今回のパッケージ整備では行わない。

## 検証結果

実行commit: `0b9c848ed41ec7c996f2db02bf729cc796affa8f`。
Windows commit→既定CheckOnly/sync→WSL lockで限定6ファイルを実行、
**64 passed / 4.08s**。
```bash
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q \
 tests/test_v4_shadow_package.py tests/test_shadow_ros2_transport_v4.py \
 tests/test_shadow_observation_join_v4.py tests/test_passive_controller_command_v4.py \
 tests/test_publisherless_shadow_v4.py tests/test_path_control_bridge.py \
 --junitxml=runs/v4_ros_package_01/junit.xml
```
固定Humble image（sha256:8c650c13157ffabbc3a72bab08865ccff8338f9025b43a8f4405b4c6b96d1ba7）、
network none専有コンテナでcolcon build成功（1 package / 1.22s）。
ros2 pkg executablesでv4_shadow_nodeを確認、launch --show-argsでconfig_file引数を確認。
インストール先のros2 runにdisabled雛形を渡し、V4_SHADOW_DISABLEDでexit1を確認。
これは期待どおりの起動拒否であり、実モデル動作PASSではない。
ログ/JUnitはWindows `tmp/v4_ros_package_01/`。
既定同期のDataset root存在確認は実施、内容・checkpoint読取は未実施。
enabled実モデルのend-to-end・親子queue負荷・AWSIM/MPC同時起動はNOT_RUN。
既存制御器へのlaunch includeやmake dev既定変更はしていない。

## 通常make devでの入力確認（2026-09-08）

通常環境の起動・実topic受信まで実施した。V4 nodeのenabled起動、モデル推論、
走行Startはまだ実施していない。上のパッケージ試験とは別の実測である。

- Windows参照HEAD: `ee25918`（開始時clean）。
- SSH: `graneple@192.168.3.10`。
- 実行repo: `/home/graneple/git/autononous_ai/aichallenge-racingkart`。
- Remote HEAD: `4af395eee10f928c7fc7225760adfa04c4c07ff4`、既存dirtyのまま保全。
- Image: `aichallenge-2025-dev`、ID
  `sha256:8c650c13157ffabbc3a72bab08865ccff8338f9025b43a8f4405b4c6b96d1ba7`。
- 実行overlay: `/aichallenge/workspace/install/setup.bash`。
  `/aichallenge/install`ではない。通常composeのhost workspace mountを使用。

```bash
# 専用COMPOSE_PROJECT_NAME、既存GUIのDISPLAY/XAUTHORITYを指定。
# 記録コマンドであり、外側の有限停止処理なしに再実行しない。
make dev DEV_AUTO_START=false CONTROL_METHOD=mpc CAPTURE=false ROSBAG=false \
 RUN_ID=normal_dev_20260908_01 \
 OUTPUT_HOST_ROOT=/home/graneple/e2e_autonomous/normal_dev_20260908_01/evidence \
 AWSIM_START_MODE=sync
```

make exit=0。AWSIM、Autoware、MPCが起動し、RVizはOpenGL初期化ログを確認。
画面動画・目視確認は未取得。25.001秒の実ROS購読で以下を受信した。

|入力|topic|実publisher|受信数|
|---|---|---|---:|
|Camera|`/sensing/camera/image_raw`|`/awsim_d1`|96|
|LiDAR|`/sensing/lidar/scan`|`/awsim_d1`|203|
|速度|`/vehicle/status/velocity_status`|`/awsim_d1`|290|
|操舵状態|`/vehicle/status/steering_status`|`/awsim_d1`|290|
|Odometry|`/localization/kinematic_state`|`/localization/ekf_localizer`|498|
|MPC command|`/control/command/control_cmd`|`/mpc_controller`|375|
|Clock|`/clock`|`/awsim_d1`|2020|

Odometry header frameは`map`。child frame、時刻整列の成立、V4 command履歴の
採用可否までこのprobeでは判定していない。Camera/LiDARはBEST_EFFORT、
残りはRELIABLE、全てVOLATILE。単一publisherのgraph snapshot確認であり、
各messageの送信者認証ではない。raw commandも同じMPCから375件受信。

既存DDS設定はloインターフェース。通常make devはdriver/zenohを起動せず、
simulation=trueによりlaunch_vehicle_interface=falseとなる既存launchを確認。
Start要求なし、race_not_armedログあり。ただし本probeは速度値を保存しないので、
ゼロ速度実測や自然制動成功とは呼ばない。MPCからのcommand publishは実施されている。
V4からの制御publish・forwardは0。AWSIMの実行物やsensor設定は変更していない。

全体38.032秒、75秒host watchdogは発火せず。所有simulatorをpause→KILL、
所有Autowareをstop、正確な専用projectをdown。cleanup errorなし、所有container残存なし。
旧停止済みprojectは保全。これはhost終了であり車両制動試験ではない。

既存累積予算を保持しwall/logを加算。MPC内部solve数は未測定なので、予約4000回を
保守的に消費計上（exact=false）。4000回実測とは報告しない。forward=0、
powered試行=0の計上。走行試行のready/deadlineは未発行、予算の自動拡張なし。

成果物: Windows `tmp/normal_dev_20260908_01/evidence/`、
remote `/home/graneple/e2e_autonomous/normal_dev_20260908_01/evidence/`。
生make/Autoware/AWSIMログ、graph_probe.json、result.jsonを保存。
実行指令の補助scriptは同runの`run.py`（製品runtimeへの追加ではない）。
通常imageでtorch 2.3.1+cu121等のimportも成功。GPU非公開の読取確認containerで
cuda=falseだったことは通常dev環境のGPU可否判定に使わない。

次は実測node名をV4設定へ結合し、pose child frameとcommand単位・実際の時刻支持を
確認した上で、固定checkpointによる停止中shadowを行う。続く短距離走行はその結果、
既存停止手順、残予算を確認してからとする。今回不足が解消したのは実topicの存在・
受信であり、V4入力組立→forward→更新経路記録のend-to-endはNOT_RUN。
trajectoryのQoS不一致WARNも保存しており、走行前に対象subscriberを確認する。

## 停止中shadowの準備

モデル読込中は親が入力subscriptionをまだ作らず、MODEL_LOADEDを待つ。
従来はモデル初期化中に64件の入力queueが満杯になり得たため、起動順のみ修正。
元のsession deadlineに読込時間を含め、読込失敗・期限切れを明示して終了する。
読込後も既存単一publisher・graph freshness条件は変更しない。
過去入力の再stamp、queue拡大、許容遅延の緩和は行わない。
tests/test_v4_shadow_package.pyで遅延ready・失敗・期限切れを合成検証する。
MPCは通常command送信前にsteering gainを掛けるため、V4にはゲイン適用前の
`/control/command/control_cmd_raw`をnominalとして選択する。

### 停止中shadowの実試験結果（2026-09-08）

実行版 `6ad1ae130d3d3e899358a6f4741340e824295561`。
Windows commit→既定CheckOnly/sync→WSL lock付き限定6ファイル:
**69 passed / 4.18s**。Datasetルート存在確認を実施、内容は未読取。
全pytestはNOT_RUN。Humbleでcolcon build成功（1 package / 1.22s）。
最初のbuildは`--log-base`をbuildの後へ置いたためCLIエラー、
`colcon --log-base /v4/log build ...`へ引数位置を直して成功した。

試験02: 試験用COMPOSE_FILE指定から通常`.env`のdocker-compose.gpu.ymlが
抜けてしまい、AWSIMがexit1。Player.logにlibnvidia-ml.so.1不足、Vulkan detection=0、
Forced renderer not supportedを記録。INPUT_READY_DEADLINE、38.535秒で終了。
観測count={}、MPCはclock待ち、V4 dispatch未到達。
元の予約計上を残した上で、この試験だけMPC -1500 / forward -6の精算履歴を
入力/実行ログhash付きで追記した。wall/log消費、旧試験の計上、承認上限は変更しない。

試験03: 原因に直接関係するCompose指定だけを直して1回再確認。
`docker-compose.yml:docker-compose.gpu.yml:<run>/overlay.json`とし、通常GPU設定を保持。
追加mountはV4 package/run成果物と固定checkpoint（read-only）。AWSIM本体未改変。
専用project=`codex-v4-shadow-live-03`、make devはDEV_AUTO_START=false、CONTROL_METHOD=mpc。
V4は最大6forward、25秒session、外側は65秒watchdog、20sim秒で終了要求。
観測速度の絶対値が0.05m/sを超えた場合も終了要求する、停止中確認専用条件。
これは車両停止距離や走行用安全閾値ではない。

実測:
- camera=256x384 bgr8、Odometry=map/base_link、raw command=/mpc_controller。
- guardで観測した最大絶対速度=2.861155223854439e-7 m/s。
- MODEL_LOADED=1。固定checkpoint SHA256
  `0316692543a901d9d6b96718c5651d6739367aa851f83fb3a133c126ccc8919f`、
  before/after一致、strict load。checkpoint読取はこの試験で実施した。
- JOIN_READY=1。camera t=6.084999863s、LiDAR t=6.089378534s、
  ego/steeringは6.054999864～6.089999863sのbracket、poseはcameraと同時刻。
- FORWARD_STARTED=1、未補正経路の記録=0。
- SESSION_END=`INPUT_QUEUE_FULL`。最初のforward中、親の入力/tick投入が
  子の消費を上回り64件queueのput_nowaitが失敗した。
- ROS process exit=0でも試験成功ではない。経路生成・継続更新は**未確認**。
- 観測時刻対応はこの1件の成立であり、全frameや学習LiDAR幾何との一致を意味しない。
  provenanceのtraining_lidar_geometry_parity=NOT_FULLY_PROVENを維持する。

全体32.000秒。Start要求なし、V4制御publishなし。所有simulatorをpause/KILL、
Autowareをstop、専用projectだけdownし、cleanup error/残存containerなし。
ここまでの観測は停止状態であり、走行追従や自然制動の成功ではない。
試験03の内部実行終了総数を断定できないためforward=6、MPC=1500を保守的に計上。
記録されたforward開始は1回で、6回完了したという意味ではない。
累積MPC計上5501/6000（残499）、V4 forward計上210、Tiny forward5711、
wall=1744.936s、powered試行7/11とpowered秒269.990/320は据え置き。

Windows成果物:
`tmp/v4_shadow_live_02/evidence/`（失敗・Unityログを含む）、
`tmp/v4_shadow_live_03/evidence/`、同runのlive.json/JUnit。
remoteは対応する`/home/graneple/e2e_autonomous/v4_shadow_live_02/`と`03/`。
試験指令は各run.pyに固定。自動再試行・自動pushはしない。

次の修正対象は推論中の入力/tick配送。workerが捨てるclockまでqueueへ渡し、
各spinでcommand snapshot付きtickも加える現実装を見直す必要がある。
有限buffer・入力時刻・欠損記録・停止条件を維持し、容量や許容期限だけを増やして
隠さない。今回の起動順修正は初期load中の詰まりに限定して有効であり、
実forward中の配送問題を解決済みとはしない。追加live試行は未実施。
