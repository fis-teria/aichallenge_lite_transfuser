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

## 配送ボトルネック修正（2026-09-09）

原因: workerが無視するclockもIPCに送り、各spinの後に同じ最大64件のcommand履歴を
添えたtickを追加していた。推論中はworkerが消費できず、不要な通知まで64件FIFOを
占有する。前回ログはFORWARD_STARTED→INPUT_QUEUE_FULLであり、forward完了時間を
保存できていないため「モデルそのものの速度が原因」とはまだ断定できない。

修正範囲は`shadow_delivery_v4.py`、V4 node、joinの配送境界と限定tests。
clockは親の従来transportで時刻/reset/graph監視し、workerへは送らない。
親はsensorイベントを最大64件、既存joinと同じ300ms期限で保持する。
IPCは最大1便。workerのBATCH_CONSUMEDを受けるまで次の便・command履歴を作らない。
空tickを蓄積せず、最短20ms間隔（最大50Hz）で必要な最新のtickをbatchに付ける。
これはカメラ・scan自体の間引きや、各streamのlatest-only置換ではない。

期限切れイベントはrole/epoch/header/元received時刻とDELIVERY_DEADLINEで記録。
期限内のburstで64件を超えた場合は引き続き終了し、容量・期限を緩めない。
待機bufferと処理中batchはそれぞれ最大64イベント、IPC未完了batchは1件に限定。
画像は期限内のものだけで、期限の更新や再stampはしない。欠損をFREE/有効入力としない。

workerは実消費時のmonotonicを使って配送期限を再検査し、joinにもその時刻を渡す。
1回のjoin.tickでforward候補は最大1件。遅いforwardの後に同じ古いcutoffで次の画像を
採用しない。次便の実時刻で期限・command availabilityを再検査する。
ROS時刻は別fieldの受信済みsnapshotのまま。monotonic差分でROS時刻を外挿しない。
元のcontroller command source/世代・受信時刻・単位・観測horizonは変更しない。
reset/fault後は待機を破棄し、旧ackで配送を再開しない。親のgraph freshness検査を
維持し、判定失効後のworker結果は採用しない。ROS制御出力は追加しない。

DELIVERY_SENTとBATCH_CONSUMEDにbatch ID、入力数、command数、queue_wait_nsと
worker_ns（join/forwardを含む）を記録する。実モデル単独forwardの計測とは区別する。
合成500ms worker停止で旧方式の64件超過、新方式の有限待機・期限切れ・再開を比較。
実spawn IPCのack、元stamp保持、期限切れ、reset、容量超過、遅いforward後の
cutoff再利用防止も限定テストする。

```bash
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q -s \
 tests/test_shadow_delivery_v4.py tests/test_v4_shadow_package.py \
 tests/test_shadow_ros2_transport_v4.py tests/test_shadow_observation_join_v4.py \
 tests/test_passive_controller_command_v4.py tests/test_publisherless_shadow_v4.py \
 tests/test_path_control_bridge.py \
 --basetemp=runs/v4_delivery_20260909_01/pytest_tmp \
 --junitxml=runs/v4_delivery_20260909_01/junit.xml
```

今回の検証は合成限定。実ROS・AWSIM再試験、checkpoint読取、新規モデル推論、走行は
実施しない。liveで経路が記録できることを合成結果でPASS扱いにしない。

### 配送修正の検証結果

実行commit: `38d4131d1b800a07c79fcffcf4edf2a714a7f7aa`。
既定CheckOnly/sync後、WSL worktree lock付きで上記7ファイルを実行:
**77 passed / 4.19s**。標準出力・標準エラー・JUnitを保存。
合成traceでは500msのworker停止に対して旧方式は90msで64件を超過
（500ms全体で348 enqueue）。修正後はack前1便、待機最大42件、
期限切れcamera等を理由付きで記録し、ack後に期限内イベントだけを配送。
これは同一の人工trafficモデルの比較で、実ROS帯域や実forward時間の測定ではない。
実multiprocessing spawn/Queueを使った合成IPC試験も成功。

Windows成果物: `tmp/v4_delivery_20260909_01/`。
WSL成果物: `runs/v4_delivery_20260909_01/`。
`pytest_tmp/test_blocked_worker_does_not_a0/traffic_trace.json`に合成失効traceを保存。
`git diff --check`成功。変更はV4専用配送/node/join/tests/docsに限定し、
既存controller、Safety、入力の数値前処理、checkpointは変更していない。
同期に伴う既定Datasetルート存在確認は実施、Dataset内容・checkpoint読取は未実施。
全pytest、実ROS接続、公式環境での再build、AWSIM、実モデル推論はNOT_RUN。
次はこの版のROS packageを再buildして、停止中の有限shadowで経路と配送計測を確認する。

## 配送修正版のAWSIM再試験（2026-09-09、試験04）

ユーザーの明示承認でMPC累積上限のみ6000→7001へ変更し、1回実施。
実行commit=`9af0e8dba4648dd2c3806a34a8744aa76d57db80`。
Windows clean→既定CheckOnly/sync。固定Humble imageでcolcon build成功
（1 package / 1.29s）。Datasetルート存在確認を実施、内容は未読取。
固定checkpointはread-only mountし、実推論のため読取。
SHA256=`0316692543a901d9d6b96718c5651d6739367aa851f83fb3a133c126ccc8919f`、
before/after一致、strict=true、missing/unexpected=[]。

通常repoのHEADは`4af395eee10f928c7fc7225760adfa04c4c07ff4`。
既存dirtyを保持し、通常GPU composeを含める。実行コマンド:

```bash
# 各run.pyに記録した専用project・有限watchdogと共に実行。
make dev DEV_AUTO_START=false CONTROL_METHOD=mpc CAPTURE=false ROSBAG=false \
 RUN_ID=v4_shadow_live_04 \
 OUTPUT_HOST_ROOT=/home/graneple/e2e_autonomous/v4_shadow_live_04/evidence \
 AWSIM_START_MODE=sync
```

**停止中の未補正経路記録に成功**。JOIN_READY=6、FORWARD_STARTED=6、PLAN=6。
全6件のraw_xy_mは[20,2]かつ全要素finite、異なるinput/output IDを持つ。
観測時刻は5.979999866～6.609999852 sim秒。SESSION_END=FORWARD_LIMIT。
INPUT_QUEUE_FULLなし。配送15便、ack15件。queue待ち時間最大13.928ms。
worker処理（join/forwardを含む）は初回forward便379.581ms、
後続forward便24.592～43.760ms。

|forward|観測sim時刻(s)|記録上のinference(ms)|command履歴の実採用slot数/10|
|---|---:|---:|---:|
|1|5.979999866|362.627|0|
|2|6.189999861|20.038|1|
|3|6.294999859|19.691|2|
|4|6.399999856|22.313|3|
|5|6.504999854|19.406|4|
|6|6.609999852|32.603|5|

起動直後の履歴paddingを含む短い試験。全履歴充足後の定常評価ではない。
時計受信前のINPUT_REJECTED=3、期限切れ配送13件（image1/lidar2/odom4/
steering3/velocity3）、終了時破棄5件も記録。欠損を隠していない。
最後のTRANSPORT_CLOSEDは終了後のclose記録であり運転中の新規faultではない。

全PLANのaccepted=false、reason=VEHICLE_CONTRACT_UNKNOWN。
これはShadowBridge(None)による追従入力の無効化で、経路が記録できなかった意味では
ない。一方、20点がfiniteというだけで追従可能・安全・経路品質良好とはしない。
V4からPP/MPCへの制御接続、走行、制動成功は未検証のまま。

Start要求なし、V4 control/gear/mode publishなし。
guardの最大絶対速度=2.8425111509022827e-7 m/s。
全体32.862秒、所有simulator pause/KILLとAutoware stop、専用project down完了。
cleanup errorなし、所有container残存なし、確認時の稼働containerなし。AWSIM未改変。

成果物: Windows `tmp/v4_shadow_live_04/evidence/`と`live.json`、
remote `/home/graneple/e2e_autonomous/v4_shadow_live_04/`。
生stdout/stderr、shadow.jsonl、guard.json、make/Autoware/AWSIMログ、終了記録を保存。
承認履歴と今回の予約1500 MPCを台帳へ保持。累積MPC計上7001/7001で残0
（実測solve数ではなく保守的計上）、V4 forward216、Tiny5711、wall1777.798秒。
追加試行は行わない。次の試験には予算判断が必要。自動pushなし。

## MPC走行＋V4 shadow試行（2026-09-09、試験05）

ユーザー承認によりMPC上限8501（+1500）で1回実施。
実行版`2f104fce72c09b535f0c73d063175c598296299a`。既定CheckOnly/sync後、
Humble colcon build成功（1 package / 1.15s）。固定Datasetルート存在確認は実施、
内容未読取。固定checkpoint読取・推論を実施。AWSIM本体・既存controllerは未改変。

専用project=`codex-v4-mpc-drive-05`。通常make devをDEV_AUTO_START=false、
CONTROL_METHOD=mpc、RUN_ID=v4_mpc_drive_05、通常GPU compose付きで起動。
V4の初回PLAN後に、Autoware container内で既存
`ROS_DOMAIN_ID=0 AWSIM_READY_DOMAINS=1 bash /aichallenge/request_awsim_start.bash`
を一度だけ起動（絶対monotonic deadline付き）。新しいcontrol publisherは追加しない。
親watchdogはStart要求前に有効化。Start要求時点から8sim秒または15wall秒で終了要求し、
10sim秒承認枠に余裕を残す。V4終了・入力/graph異常でも所有instanceをfreeze/終了。
V4最大40forward、全体95秒watchdog、外側115秒＋kill grace5秒。

**結果: DRIVING_NOT_ESTABLISHED（走行成立せず）。**
- 既存Start helperはReady/WaitStartを観測し、Start pulseを1回送信。
  vehicle state=startまで確認したが、helperの完了は未確認のまま終了。
- 既存Autostartログ:
  `overtake race arm deferred: state=Start waiting_for_neutral=Ready`。
  overtake側はrace_not_armedを記録。Start送信だけで制御開始済みとはしない。
- V4は40forward/40PLAN、全て[20,2]のfiniteな未補正XYを記録。
  観測時刻5.979999866～10.914999756sim秒、FORWARD_LIMITで終了し、hostも終了。
  INPUT_QUEUE_FULLなし。走行中の経路記録ではなく、ほぼ停止状態での記録。
- guard最大絶対速度2.8891219017168623e-7m/s。601件のpose記録の始終差は
  dx=-7.559e-6m、dy=8.688e-5mで、移動・追従成立の証拠ではない。
- guardの最終INPUT_STALEはfreeze後の欠損も含む最終状態。
  hostの終了理由はV4_ENDED、nodeの終了理由はFORWARD_LIMIT。

全体37.923秒。Start要求時6.314999858sim秒、最終観測12.254999726sim秒。
所有simulator pause/KILL、Autoware stop、専用project down完了。
cleanup errorなし、所有container残存なし。車両の自然制動成功・無接触走行は未確認。
V4のaccepted=false/VEHICLE_CONTRACT_UNKNOWNは維持し、V4操舵には接続していない。

Windows成果物: `tmp/v4_mpc_drive_05/evidence/`、`live.json`、`evaluation.json`。
remote: `/home/graneple/e2e_autonomous/v4_mpc_drive_05/`。
motion.jsonlで観測時刻/pose/速度/raw・final commandを対応付け、Startログも保存。
run.pyのV4_ATTEMPT_FINISHEDは処理終了の意味であり走行PASSではない。
予算履歴は保持し、MPC1500、forward40、powered試行1/秒10を保守的計上。
累積MPC8501/8501、V4 forward256、Tiny5711、wall1815.721秒、
powered試行8、powered秒279.990。駆動10秒の実測という意味ではない。

追加試行なし。次は既存Start→neutral Ready確認→race armの成立条件を確認する。
今回の試験起動順では、その完了前にV4の40回枠を使い切ることも分かった。
上限を増やすだけで解決済みとせず、走行準備と推論開始の順序を見直す必要がある。
安全gateの解除・Readyの偽装は行わない。次回liveには改めて予算判断が必要。

## 開始順序の見直し（2026-09-09、静的確認・設計）

対象は試験05のStart未完了と、走行前のV4予算消費。
追加ROS起動・Start・checkpoint読取・推論・走行は実施していない。
remoteの既存dirtyコードとSafety/Start helperは変更しない。
参照repo HEADは`4af395eee10f928c7fc7225760adfa04c4c07ff4`（dirty）。
この節は次の実装修正仕様であり、実装・走行済みという意味ではない。

### コードで確定した条件

以下のパスはremoteのaichallenge-racingkart基準。

|箇所|確認した意味|
|---|---|
|`aichallenge/workspace/src/aichallenge_system/autostart_orchestrator_py/autostart_orchestrator_py/autostart_orchestrator_node.py` `_RaceArmLatch.observe_state`|Ready未観測のStartはarm_state_before_neutralで拒否。Readyは車両状態名で、操舵中立値の要求ではない|
|同 `_RaceArmLatch.observe_official_start`|同一reset世代でReady観測と初期化完了が必要。terminal/reset後や初期化前の呼出しを拒否|
|同 `_on_vehicle_state`|上の拒否理由をwaiting_for_neutral=Readyとして表示|
|同 `_on_official_start`|SetBoolの結果をsuccess/messageとして返し、race_armをpublish|
|同packageの`config/autostart_orchestrator.param.yaml`|arm=Start、neutral=Ready、disarm=Spawned/Grounded/Finish|
|`aichallenge/request_awsim_start.bash` `publish_official_one_shot_start`|GroundedまたはReady＋初期化済み＋race_arm=false等を確認して1回だけStart pulseを送る|
|同 `wait_for_one_shot_start_barrier`|車両のStart履歴だけでなく、watermark後のadmin Startも要求|
|同 `arm_after_authoritative_start`|履歴・初期化・rollback余裕を再確認してofficial_start serviceを呼び、race_arm=trueと最終状態を確認して完了|

したがって、車両domainのStartログだけをもって、admin Start成立やrace_arm成立と
扱えない。Ready前Startの待機ログそのものは故障の証拠でもない。
試験05にはhelperの完了ログやofficial_start service成功ログがなく、40forward終了で
途中終了した。Readyが恒久的に来ない、既存gateが壊れている、という断定は取り消す。
カウント時間の実値、Ready/admin Startがその後成立したかはUNKNOWN。

### 試験起動側の問題

`tmp/v4_mpc_drive_05/run.py`は最初のPLANを待ってStart helperを起動するが、
その待ち時間にもV4は連続推論する。40回終了でhelperとsimを止めるため、
Start handshakeの完了とshadowの残り回数を両立する段階分けがない。
これは配送queueの問題とは別であり、MPC/モデル再学習の根拠にはならない。

### 採用する修正仕様（未実装）

1. PREPARE: 固定モデル読込・入力購読・graph監視・有限履歴の準備を先に行う。
   forwardの実行許可は閉じておく。準備中も元の時刻/欠損/resetを記録し、
   古い入力を復帰時にまとめて再推論しない。既存のworktree/試験deadlineを延長しない。
2. START_HANDSHAKE: 既存Start helperを1回だけ実行する。Readyやrace_armの偽装、
   neutral条件の変更、別経路からのengageは行わない。
3. RUN_SHADOW: helperの正常完了に加え、選択した同じinstance/世代の
   initialization_readyとrace_armの新鮮な証拠を確認してforwardを許可する。
   残るwall/sim/forward予算が不足なら走行試験を打ち切る。
4. END: 既存監視の失効・worker終了・上限で所有simを停止する。
   自然制動とhost freeze/KILLを別記し、停止できたことを走行成功に置換しない。

V4側の許可は「shadow forwardを開始してよい」だけで、車両制御権限ではない。
この許可をrace_arm/Startへ逆送しない。新しいactuator publisherやcontrollerは不要。
標準ROS2パッケージ内の最小の推論待機機能として実装し、別の走行runnerへ
モデル本体を移動しない。親のgraph監視・期限・worker終了監視は待機中も有効とする。

重要: 既存helperが走行許可を出した瞬間に車両が動く可能性がある。
RUN_SHADOW開始を理由に駆動タイマーを0へ戻さない。Start要求前からの保守的な
時間計測と、許可後/実速度発生後の計測を併記する。準備時間と駆動時間を区別しても
全体120秒・駆動10sim秒の既存上限を延長する意味にはしない。
正常helper完了が駆動枠内に収まらない場合は、原因・必要時間を示して別承認を求める。

### 実装時の限定テスト

- Ready未到着、早いStart、初期化未完了ではforward=0、待機が有限で終了する。
- helper成功だけ、retained race_arm=trueだけでは開始しない。現在世代・freshnessを要求。
- helper完了＋新鮮な許可後にだけforwardを開始し、実際の履歴maskを保持する。
- reset、race_arm=false、入力/graph stale、期限切れで待機/推論許可を失効させる。
- 前段でforwardがあれば総上限から差し引き、フェーズ変更・再起動で回数をresetしない。
- 失効後の旧worker結果を採用しない。既存Start helper/Safetyの条件は変わらない。

今回は静的見直しと文書化まで。上記待機機能・テスト・追加走行はNOT_RUN/未実装。
現在の累積MPC予算8501/8501は変更していない。実装の合成検証は走行予算を消費しない。

## 推論待機の実装（2026-09-09、合成検証対象）

上節の「未実装」は設計時点の記録。今回、標準`v4_shadow_node`に
`start_gate`を任意設定として追加した。既存の設定（キーなし）は停止時試験との
互換のため従来どおり即時推論。駆動試験には必ず新しいgate設定を指定する。
exampleは引き続きenabled=false、instance/receipt未設定で起動不可。
これはモデル・PP・MPC・Start/Safetyを変更する機能ではない。

- `shadow_start_gate_v4.py`: ROS非依存のStartGate/ForwardPermitと、読取専用の
  ROSStartObserver。`std_msgs/Bool`をreliable/transient-localで購読する。
  現環境の実QoS互換性は今回NOT_RUN。送信元は単一の`/autostart_orchestrator`を
  100ms周期でgraph確認し、500ms以内の確認を要求する。
- `/overtake/race_armed`: この購読開始後のfalse、その後のtrueを要求。
  初期retained trueだけでは開かない。`/autostart/initialization_ready`もtrueが必要。
  Boolにepoch/headerはないため同一instanceの隔離と既存helperの世代検査を信頼する。
  graphは個々のmessageを認証しない。起動中のpublisher入替えを検出し切る保証もない。
- helper成功証拠は同一hostのmonotonic nsを使う小さなJSON（上限4096byte）。
  起動時に存在するファイルは拒否する。session/instance/epoch=0、exit_code=0、
  helper開始・終了時刻を照合する。false観測はhelper開始より前、true観測は開始以降。
  false/trueだけからhelperの成功を捏造しない。
- Boolは更新時通知なので受信時刻をheartbeatとしてTTL更新しない。
  開始後はfalse、初期化取消、graph失効、clock reset、既存入力監視の故障で終了。
  workerへは最大500msの期限付き許可を渡し、forward前後で検査する。
  親もPLAN採用前に再確認。イベント配送には遅延があり、既に実行中のforwardを
  即時キャンセルする保証はないが、失効結果を有効PLANとして採用しない。
- PREPAREでも実入力・外部command履歴を既存adapterへ追加し、tensor buildとforwardは
  行わない。許可前に受信したcameraを許可後に再推論せず、PRE_START_OBSERVATIONとして
  履歴だけに使用する。履歴maskを全有効へ置換しない。
- 全候補（PREPARE含む）200件上限、総wall/forward/log上限を変更しない。
  したがってStartが遅ければCANDIDATE_LIMITで終了し得る。開始時に予算をresetしない。
  今回の実装が既存の有限時間内にStartを完了できるという実証はまだない。

### 次回の既存試験起動側との受け渡し

旧test05の「最初のPLANを待つ」はgate有効時には使わない。既存の専有試験起動側で
PREPARE_STARTEDと少なくとも1件PREPARED（入力組立済み）を確認し、さらに
START_EVIDENCEのarm=falseを確認してから、**既存Start helperを一回だけ**起動する。
helperの正常終了を待った呼出元が次を実行する。以下は接続例であり実行ログではない。

```python
# helper_started_ns は既存helperを起動する直前に記録。
# returncode は既存helper process.wait() の実測値。固定値0で代用しない。
from pathlib import Path
from aic_transfuser_lite.runtime.shadow_start_gate_v4 import save_helper_completion
if returncode == 0:
    save_helper_completion(Path(config['start_gate']['receipt_file']), {
        'session_id': config['envelope']['session_id'],
        'instance_id': config['start_gate']['instance_id'], 'epoch': '0',
        'helper_started_ns': helper_started_ns,
        'helper_completed_ns': helper_completed_ns, 'exit_code': returncode,
    })
# 失敗時はreceiptを作らず、既存の所有sim停止・終了処理へ。
```

receiptは信頼する試験起動側からのローカル証拠であり署名/認証ではない。
同一host、専有run、共有した絶対パスを使用し、他者書込不可のディレクトリに置く。
保存はpendingのexclusive作成→hard linkによる原子的公開で、既存ファイルを上書きしない。
helper/Start操作はnode側に追加しない。元Start helperは変更しない。
今回、旧試行run.pyやremote環境は変更していない。次回実試験の起動側への組込、
実QoS/送信元/Ready遷移確認、停止、追加駆動予算承認は残課題。
Start要求前からの既存駆動タイマーを維持し、RUN_SHADOWでresetしない。

### 限定再現コマンド（固定モデル・ROS不要）

Windows commit後、既定`tools/sync_to_wsl.ps1 -CheckOnly`、通常syncを使用する。
WSLにて以下の限定テストのみを実行する（runsは新規run名を使う）。

```bash
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q -s \
  tests/test_shadow_start_gate_v4.py tests/test_shadow_delivery_v4.py \
  tests/test_v4_shadow_package.py tests/test_shadow_ros2_transport_v4.py \
  tests/test_shadow_observation_join_v4.py tests/test_passive_controller_command_v4.py \
  tests/test_publisherless_shadow_v4.py tests/test_path_control_bridge.py \
  --basetemp=runs/v4_start_gate_20260909_01/pytest_tmp \
  --junitxml=runs/v4_start_gate_20260909_01/junit.xml
```

合成traceはpytest_tmp下の`synthetic_start_trace.json`。実センサ/実forwardではない。
ROS実購読・checkpoint読取・モデル推論・AWSIM走行・全pytestはNOT_RUN。
既定syncのDataset操作は固定ルートの`test -d`のみ（静的確認済み）。
Dataset内容・raw・sensor・checkpoint読取を許可する変更は加えていない。
