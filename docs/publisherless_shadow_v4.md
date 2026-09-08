# Publisherless V4 shadow runner — 合成限定実装

## 結果と未完成境界

起点4af1e2650848cd6b25fb73120f1fac9b5596cd6a、Windows clean。
実行commit2914a21f8a81d07d8c191da68236cfdd352b1305。
今回追加したのは送信非接続のsession core、有限子process harness、入力専用node構築hook、合成tests。
**実AWSIMへ接続するrunner全体は未完成**。CLIはfixtureだけを許可し、実入力モードは
`LIVE_BLOCKED_PASSIVE_COMMAND_POSE_AND_ISOLATION_BINDINGS_REQUIRED` で起動前に拒否する。

ユーザーが承認した1回/120秒/40推論/送信0という試験方針に対して、上限と送信非接続を
独立に実装・検証した段階。実ROSの完成やAWSIM試験PASSとは扱わない。

## 変更

- `src/aic_transfuser_lite/runtime/publisherless_shadow_v4.py`
  - `ShadowSession`: adapter→V4 callback→raw20点→既存Bridge。既存送信runnerはimportしない。
  - `fixed_infer_factory`: 明示呼出し時だけ既存の厳密固定loaderへ接続。今回呼出していない。
  - 欠損commandを0で補わない。shadow出力をcommand履歴へ戻さない。
  - epoch変更で履歴とBridgeを無効化、重複候補を拒否、forward前に試行数を加算。
  - late forwardはplan化しない。最大40試行、最大200候補。
  - `control_tick`は観測受付とは別のAPI。ただし現段階の同一process内同期infer中には
    tickを実行できない。実transport版では別processの推論と独立周期の制御評価へ接続が必要。
  - `create_input_only_node`: 7種類の受動subscriptionだけを作る注入型hook。
    ROS middleware内部のparameter-event publisher等は未実測。全DDS publisherゼロは未証明。
    control/gear/mode publisherは新コードに存在しない。
- `tools/run_v4_publisherless_shadow.py`
  - fixture専用CLI。新規output必須、上書き禁止、自動retryなし。
  - 120秒のうち5秒を終了処理へ確保。所有childのみterminate→kill→reap確認。
  - 同期modelが止まっても親processが終了させる設計。OS停止不能なら成功扱いしない。
  - 合成fixtureの生成時だけ人工値を使用。実モードの承認期限や車両値を自動生成しない。
- `tests/test_publisherless_shadow_v4.py`: 11件。既存Bridge23件と合わせ34件。

## 入力の実在問題

既存 `SpatialInputV4.build()` はcommand bindingと利用可能な過去commandを要求する。
旧sim runnerはHOLD等を実際に送信して履歴を作っていた。送信ゼロの試験で同じ履歴を
捏造するとモデル入力契約を変えてしまう。
このため、新sessionは本当に受動取得したcommandがない場合に
`PASSIVE_COMMAND_MISSING` としてforwardを呼ばない。
静止していることだけからcommand=0とは推定しない。
ただし「送信ゼロ」はV4 shadow側に対する条件。別制御器MPC/PPが唯一の実制御を
担当する構成では、その指令を受動取得する。システム全体の制御送信ゼロは要求しない。

## 外部制御器commandの受動取得（2026-09-08追記）

起点 `3184ddc1474002a35d7ac3ec651ac28b0a4baaea`。新規
`runtime/passive_controller_command_v4.py` の `ControllerCommandBinding` は
選択topic・外部publisher identity・契約根拠・field意味を必須とする。
`decode()` は AckermannControlCommand の `stamp`、
`lateral.steering_tire_angle` [tire rad]、`longitudinal.speed` [target m/s]、
`longitudinal.acceleration` [m/s^2] を既存PassiveCommandへ変換する。
既存 `spatial_bootstrap_v4.py` の型/単位契約を使用。実速度・実操舵reportで代用しない。

`create_input_only_node(..., command_binding=binding)` のcommand roleを選択topicへ
接続し、受信callbackがpublisher identityを照合してdecodeする。受信/利用可能monotonic
時刻とmessage stampを分離して渡し、既存observationのpassive_commandsへ渡す。
adapterは `command_binding_known=True`、final採用時に限って根拠確認後
`final_fallback_verified=True` とする。実producerの同定を文字列指定だけで証明済みにしない。

学習selector `dataset_view_v3._selected_command` に合わせnominal優先、確認済みfinal
のみfinal_fallbackで採用。finalをnominalへ読み替えない。nominalは選択制御器の
actuation前要求であり、最終送信と同一とは主張しない。final topicの例は
`/control/command/control_cmd` だが現在graphのproducer・意味・競合は未確認。
shadow自身の生成指令をこのcallbackへ戻さない。既存50 ms過去slot対応、利用可能時刻、
epoch、64件上限を再利用し、現在時刻へのrestampや期限延長は追加しない。
PLAN記録には選択されたcommandとstamp/sourceのprovenanceを保存する。

外部制御器の起動、実ROS subscription組立、publisher GIDの現在graph照合は今回未実施。
実transport runner未完成のためlive CLIの拒否は維持。MPC/PP起動・AWSIM操作・走行・
実推論・実入力収集はNOT_RUN。これは外部指令の受動取得hookと合成検証の変更。

追加検証（Windows commit→既定CheckOnly/sync→WSL lock）：
```bash
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q \
 tests/test_passive_controller_command_v4.py tests/test_publisherless_shadow_v4.py \
 tests/test_path_control_bridge.py --junitxml=runs/passive_external_command_01/junit.xml
```

## 実行結果とコマンド

Windows commit→既定CheckOnly/sync→WSL lock。固定Dataset rootの存在確認を実施。
Dataset内容/raw/sensor/checkpointの読取なし。sim hostへの接続・変更なし。

```bash
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q \
 tests/test_publisherless_shadow_v4.py tests/test_path_control_bridge.py \
 --junitxml=runs/publisherless_shadow_01/junit.xml

bash tools/with_wsl_training_lock.sh .venv/bin/python tools/run_v4_publisherless_shadow.py \
 --fixture --session-id synthetic-publisherless-01 \
 --output runs/publisherless_shadow_fixture_01 --wall-seconds 120 --forward-limit 40
```

限定tests: **34 passed / 0.41s**。
fixture process実行: exit0、child回収済み、wall約0.214s、人工出力callback40回。
これはV4モデルforward40回ではない。実モデルforward0、実scan0、AWSIM起動0、制御/gear/mode送信0。
fixtureログ中のFORWARD_STARTEDも人工callbackの試行であり、`fixture_only=true`で区別する。
上限到達後のcontrol_tickは指令無効として記録する。
trace.jsonl、summary.json、stdout/stderr、JUnitをWindows `tmp/publisherless_shadow_results/`へ保存。
旧live台帳の使用量・期限・active等は変更していない。今回はlive予算未消費。

ASTによる新コードのpublisher/client/action呼出し不在検査と、fake Nodeがpublisherを
作らない構築試験を実行。実ROS graphでの確認はNOT_RUN。
全pytest、ROS/AWSIM、実checkpoint推論、走行、pushはNOT_RUN。

## 実試験へ残る作業

1. 選択した別制御器のcommand topic・publisher identity・意味を現在graphで確認し、上記受信hookへ結合。
2. 現simのpose生成経路と観測時刻を結合する入力受信部。任意のOdometry topicを仮定しない。
3. 車両Limitsと出典、current-container隔離の読取検証、実ROS内部endpoint監査。
4. 推論childと独立control tick、queue/失効/欠損を実transportへ接続。
5. 新しい有限試験枠を旧使用量へ追加する既存budget連携と、所有AWSIMの外側監視・終了。

これらを接続するまではlive CLIを開放しない。既存HOLD送信runnerを裏で使う回避も禁止。

### 外部指令hookの合成検証結果

実行commit `919481c3b8ee8d3721e13a46547c05fd880289b2`。
上記3テストファイル: **46 passed / 8.62s**。source維持、未確認final拒否、
nominal優先、future/stale/availability/epoch不一致不採用、非有限値・restamp拒否、
shadow自己出力拒否、選択topicのsubscriptionのみの構築を合成検証。
生stdout/stderrとJUnit: Windows `tmp/passive_external_command_01/`。
既定同期によるDatasetルートの存在確認を実施。
Dataset内容・raw・sensor・checkpointの読取りは未実施。
ROS・AWSIM・実推論・実制御・走行・全pytest・pushはNOT_RUN。
実環境の制御器稼働、QoS、producer identity、final指令意味はこのfixture結果では確定しない。

### 2026-09-08: ユーザー承認済みの試験枠更新

`configs/control/v4_external_controller_shadow_authorization_20260908.json` に承認を記録。
1試行、終了処理込みwall120秒、制動込み駆動10sim秒、V4推論試行40回。
準備完了時を一度だけ記録し、そこから30分以内に終了する。準備前の現在時刻を
仮のready時刻として書かない。承認待ちではなく `AUTHORIZED_PENDING_PREPARATION`。
累積上限・使用量は変更せず、旧走行期限はこの1試行の有効化時に更新する。
他試行や旧プロファイルの一括再承認ではない。

現行 `PPBudget.reserve` はmpc予約を0に限定し、V4から直接PPへ送る旧4試行用。
この旧gateを無効化して外部MPC走行を通さない。外部制御器の実計算を含めた予約と
上記1試行制限の接続、実ROS受信・隔離・停止確認が完了するまでdispatchしない。
承認ファイルのみでruntime enforcement済みとは扱わない。
現時点でremote budgetのready/期限変更・試行予約・AWSIM起動は未実施。

### ROS2受信部の準備（2026-09-08）

実装 `runtime/shadow_ros2_transport_v4.py`、実行commit
`49eb9cba48ae1d978fc43a186224978106554720`。
`ros2_types()` はImage/LaserScan/VelocityReport/SteeringReport/
AckermannControlCommand/Odometry/Clockを遅延importし、ROSを自動初期化しない。
`ShadowROS2Transport` は既存nodeに7種類のsubscriptionのみを作る。
topic/QoS/clock IDと全入力publisher GIDを明示指定し、受信MessageInfoの
publisher_gid.hexと比較する。GIDは実graphから取得し、自身や競合producerを
選ばないこと。topic名の一致だけでは外部制御器の同定にならない。

commandは既存decoderへ接続し、最大64件で保持する。上限超過はevictを記録。
`command_snapshot()` を入力確定時に既存sessionのpassive_commandsへ渡す。
保持値の時刻は元stampのまま。選択の過去性・availability・50ms制限は既存adapterの責任。
clock未受信は拒否。clock巻戻りでepoch更新・command消去・on_reset通知。
送信元不一致やdecode例外でもcommand消去・on_resetを通知し、理由をemitする。
closeは所有subscriptionのみを解除する。node/context/executor自体は呼出元所有。

`on_input(role,message,received_monotonic_ns,epoch)` は未整列の実messageを渡す境界。
ここから観測時刻でのego/pose結合とGridObservation構築を行う組立は残っている。
on_resetはその組立側のsensor queue・pose・planも失効させる必要がある。
既存SpatialPathShadowWrapperV4は独自runtime/Recordsとnominal専用経路へ結合されており、
新ShadowSessionへそのまま接続済みとは扱わない。新しい実行CLIはまだ開放しない。
この受信部だけで「実ROS接続準備全体完了」や「走行可能」とは主張しない。

検証コマンド（Windows commit→既定CheckOnly/sync→WSL lock）：
```bash
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q \
 tests/test_shadow_ros2_transport_v4.py tests/test_passive_controller_command_v4.py \
 tests/test_publisherless_shadow_v4.py tests/test_path_control_bridge.py \
 --junitxml=runs/shadow_ros2_transport_01/junit.xml
```
結果 **51 passed / 6.51s**。fake Node/MessageInfoによる受信・source維持・GID不一致・
clock未受信/巻戻り・有限buffer・解除を検証。ASTで制御publisher等の不在を確認。
生stdout/stderr/JUnitはWindows `tmp/shadow_ros2_transport_01/`。
実rclpy/DDS・message package import・実センサ購読・実モデル推論・AWSIM・走行はNOT_RUN。
既定同期によるDatasetルート存在確認は実施。Dataset内容・raw・sensor・checkpointの
読取りは未実施。試験期限は未開始、台帳・AWSIM・既存制御器は変更していない。

### 観測時刻結合と実ROS graph probe（2026-09-08、部分完了）

`shadow_observation_join_v4.ShadowObservationJoin` がtransportのon_input/resetと
ShadowSession.observationを接続する。RGB/BGR Image、LaserScan、Velocity/SteeringReport、
明示base_link childのOdometryをSampleへ変換する。poseの固定frameと根拠は必須。
既存align_observation/interpolateを再利用し、camera基準ego/poseをbracket補間する。
LiDARは30ms以内のnearestであり同時刻へ再stampしない。ego/poseは50ms支持、
外挿なし。最大64件/stream、待機camera16件、300msの固定期限。欠損はWAITから
DEADLINEへ終了し、resetで待機・履歴・planを消す。推論呼出しはROS受信callbackではなく
別workerからtickする必要があり、worker分離の実組立は未検証。
実AWSIMにOdometryがあるとは仮定しない。GNSS/IMUからのpose生成は自動追加していない。

合成tests: commit `61602c917e5dee830d60c6595b9004cf3d2d28f8`、**54 passed / 4.16s**。
前節の限定コマンドに `tests/test_shadow_observation_join_v4.py` を追加し、
JUnit出力先を `runs/shadow_join_02/junit.xml` とした。
生ログ/JUnitは `tmp/shadow_join_live_20260908/tests/`。
既定CheckOnly/同期による固定Dataset root存在確認のみ。内容/checkpoint読取なし。

実host `graneple@192.168.3.10`、既存固定image
`sha256:8c650c13157ffabbc3a72bab08865ccff8338f9025b43a8f4405b4c6b96d1ba7`、ROS Humble。
`tools/host_shadow_graph_probe_v4.py` で旧専用composeのsimulatorだけを新規所有projectへ
複製。network=none、非privileged、AWSIM/read-only mount、制御nodeなし、外側110秒
TERM+5秒KILL。既存バイナリ/scene変更なし。旧checkout・旧コンテナを変更していない。
実行は各専有ディレクトリで `timeout --signal=TERM --kill-after=5 110 python3 host_shadow_graph_probe_v4.py`。

2試行はともにFAILED。第1は9.245秒、第2は7.377秒、制御送信0、推論0、駆動要求0。
各所有AWSIMはhost pause→KILL→scoped compose downで終了。自然制動の成功ではない。
final_containers空・cleanup errorなし。生ログを保全し、wall/log消費を既存budgetへ追記。
旧累積使用量をリセットせず、承認された駆動1試行とそのready期限は未開始。

観測できた実graph：/clock は awsim_d1、RELIABLE/VOLATILE/KEEP_LAST depth10。
/sensing/lidar/scan は awsim_d1、BEST_EFFORT/VOLATILE/KEEP_LAST depth5。
各endpoint GIDをgraph_probe.jsonへ保存。
/control/command/control_cmd は型を確認したがpublisher=0（このprobeはsimulatorのみ）。
camera/ego/Odometryの網羅確認は未完了。receiptsは空で、受信成功とは扱わない。

初回エラーはcallbackにinfoが渡らないTypeError。2引数closureへ修正して再試験したが
同じ失敗が再現し、以降のAWSIM再起動は中止。実image内の
`/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py` を静的確認したところ、
`_take_subscription` がtake_messageの第0要素だけを返し、`_execute_subscription` は
callback(msg)のみを呼ぶ実装だった。2引数への修正だけではこの版に対応できない。
**実ROS transportはこの版で未対応**。graphのGIDを受信messageのGIDだと偽装しない。

次に必要なのは、同じ固定imageの低レベルtake_messageが返すmetadataの実契約確認と、
MessageInfo/GIDを保持できる受信方式の合成ROS検証。それができるまで本transportを
liveへ昇格しない。環境全面更新・GID照合解除・実制御開始はしていない。
外部制御器の起動とego/pose送信元の確認も残る。今回の実ログは
`tmp/shadow_join_live_20260908/attempt01/` と `attempt02/`。
host側正本は `/home/graneple/e2e_autonomous/shadow_graph_20260908_01/evidence/` と
`shadow_graph_20260908_02/evidence/`。失敗時も当時host wrapper自体のexitは0だったため、
result.json/statusとstderrを判定正本とした。後続修正で失敗時exit1を追加（実再試験なし）。

### メッセージ送信元受信の修正（部分完了）

実行commit `eb81be190e3d4d36847c4ee6a000c7165faa0421`。
固定image内のrclpy take_messageを隔離合成ROSで確認した結果、metadataはdictで
source_timestamp/received_timestampのみ。publisher_gidは提供されない。
`integrations/ros2_gid_receiver_v4/receiver.cpp` を追加し、rclcppの
SubscriptionBase.take_serialized(message, MessageInfo)から同じtakeのserialized bytesと
publisher_gidを取得する方式を実装。ROS publisher/client/engageは作らない。
Node parameter events/rosoutも無効。QoSは明示SensorDataQoS、最大7入力、25秒、2000件、
1MiB/message。stdoutはnonblocking、200msでbackpressure終了。外側監視は引き続き必須。
role/gid/受信CLOCK_MONOTONIC ns/serialized hexを1行で渡す。rawセンサ全保存は不要。

Python `ShadowROS2Transport(..., serialized_receiver=True)` はrclpy subscriptionを
作らず、`ingest_wire`が同一hostの専有child pipeから取得した有界行をdeserializeして
既存receiveへ渡す。GID不一致・不正行・未来monotonicは履歴失効。EOF/child停止も
呼出元がresetする必要がある。実runnerへのprocess/pipe組立は未完了。

固定ROS image内でCMake build成功。隔離された/fixture/gidのString publisher2個から
異なるメッセージGIDを取得しCDRをPythonへ復元できた。ただしgraph endpoint GIDとの
一致assertは失敗した。例：fixture0のmessage GIDは
`9b64e5fa06ad67f000000000000000000000000000000000`、graphは
`01101fa4953d3b6574e3dc22000007030000000000000000`等で、同一表現ではない。
このRMW版における対応付けはUNKNOWN。graph GIDをmessage GIDとして使用したままでは
live入力は拒否される。照合を解除せず、graph/source同定がPASSとは報告しない。
2回の一致検証失敗後は同じ試験を繰り返していない。

追加Python unit testsを含む限定5ファイルは58 passed。WSL lock実行：
```bash
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q \
 tests/test_shadow_ros2_transport_v4.py tests/test_shadow_observation_join_v4.py \
 tests/test_passive_controller_command_v4.py tests/test_publisherless_shadow_v4.py \
 tests/test_path_control_bridge.py --junitxml=runs/gid_receiver_fix_01/junit.xml
```
CPP/合成ROS再現（固定image、network none、専有/probe mount内）：
```bash
source /autoware/install/setup.bash
cmake -S /probe -B /probe/build
cmake --build /probe/build -j2
env -u CYCLONEDDS_URI timeout -k 2 20 python3 /probe/test_humble_gid_receiver_v4.py /probe/build/v4_gid_receiver
```
ROS_LOG_DIR=/probe/ros_logs、user1000:1000で実施。既定imageに存在しない
Cyclone設定パス、初回出力権限/ログディレクトリの問題を当該隔離コンテナだけで訂正。
ホスト/既存ROS環境は変更していない。build.log/test.log/test2.log/test3.logと
JUnit/stdout/stderrをWindows `tmp/gid_receiver_fix_01/` に保存。
このtest scriptの最終結果はFAILのまま保全し、期待値を書き換えてPASSにしていない。
今回AWSIM起動・制御topic送信・走行・実センサ/モデル推論なし。
既定同期のDataset root存在確認のみ実施、内容は未読取。pushなし。
残る一点はmessage GIDと選択graph endpointを結び付ける根拠ある方式の確定。
実ROS接続全体の完了や走行可能を意味しない。
