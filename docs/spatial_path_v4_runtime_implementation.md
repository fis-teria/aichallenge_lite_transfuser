# 固定step500 V4 runtime：実装と限定検証

## 現在の変更：単独bootstrap・受動binding（基準4aae81e、合成限定）

以下の履歴にある「mainが無条件BLOCKED」は前段の状態。今回、default disabledを保ちつつ、
正しいconfigと別途承認manifestが揃えば専用bootstrapへ到達する実装に変更する。
配布configは未許可・未解決であり、実ROS・checkpoint・sensorを今回使用しない。

### 接続と検査

- `spatial_bootstrap_v4.check_config`: 明示した小configだけを解析し、code/config/binding/input/checkpoint hash、
  ROS domain/namespace、出力先、全上限、producer根拠file/commit/basisと別承認manifestを照合する。
  hash参照先のcheckpoint/Datasetを開かない。期待contractの設定と実graph観測は別状態。
- `spatial_bootstrap_v4.run`: 新規private directory→既存固定loader→Records/writer/adapter/core→
  自分のcontext/node/wrapper/executor→bounded spin_once/monotonic tick→共通終了。
  固定loader自体は変更なし。device=cpu、同じ9 tensorsと未補正snapshot。許可はretry/resetで増やさない。
- `BindingGuard.check/ready`: header整数・frame・画像shape/stride・750点と角度配置/range metadata・
  ego/commandのfinite fieldを検査。全roleの内容確認と過去nominal受信前はforwardしない。
  header時刻は取得時刻・monotonic receipt/availabilityとは別。内容一致でもproducer意味や実input parityの証明ではない。
- wrapper: optional guardでcallback前の受付上限とforward前の停止条件を確認。stopは新受付を止めて
  pendingを同じIDでterminal化し、shutdown中の入力再構成/forwardを行わない。
- 終了: executor→node→所有context→writerの有限処理。元例外とcleanup errorを分離。
  healthy時だけhealth/endを試行。FAILED/CLOSEDへ再書込みしない。I/O/forwardのhard realtime中断は保証しない。
  shutdown_graceはexecutor待機と終了超過検出の予算であり、同期I/Oを強制cancelする上限ではない。

### 受動入力の対応（実graphは全role NOT_OBSERVED）

| role | topic | message / field・単位 | frame・source clock・QoS | producer/根拠 |
|---|---|---|---|---|
| image | MISSING | sensor_msgs/Image; rgb8,height,width,step,data | frame/実shape UNKNOWN; ROS_SIM期待、SENSOR_DATA | wrapper.receive/tick (基準60b3da3)、producer UNKNOWN |
| lidar | MISSING | sensor_msgs/LaserScan; ranges[m],angles[rad],range_min/max[m] | frame/実角配置 UNKNOWN; ROS_SIM期待、SENSOR_DATA | wrapper.tick/SpatialInputV4.append (60b3da3)、producer UNKNOWN |
| velocity | MISSING | autoware_auto_vehicle_msgs/VelocityReport; longitudinal/lateral_velocity[m/s],heading_rate[rad/s] | frame UNKNOWN; ROS_SIM期待、RELIABLE/VOLATILE/depth10 | wrapper.tick (60b3da3)、producer UNKNOWN |
| steering | MISSING | autoware_auto_vehicle_msgs/SteeringReport; steering_tire_angle[rad] | frame UNKNOWN; ROS_SIM期待、RELIABLE/VOLATILE/depth10 | wrapper.tick (60b3da3)、producer UNKNOWN |
| nominal | MISSING | autoware_auto_control_msgs/AckermannControlCommand; stamp,lateral.steering_tire_angle[rad],longitudinal.speed[m/s],acceleration[m/s²] | 意味上のframe UNKNOWN（messageにframeなし）; ROS_SIM期待、RELIABLE/VOLATILE/depth10 | passive nominal before actuationを要求。実producer UNKNOWN、final/appliedへのalias禁止 |

実環境用evidenceはfile/40桁commit/basisの明示参照が必要。チェック時にそのpathを辿らない。
boolのverified flagでは代用しない。testのFAKE根拠をlive用に昇格しない。
gridは既存100ms/ROS epoch原点へのnearest、egoはcamera exact、LiDARは±30ms内nearestという
コード上の期待。実grid phase適合の根拠はUNKNOWN。新しい補間は導入しない。
sensor_dt[4,2]はcamera_header-gridとlidar_header-camera_headerを維持。
LiDARのrange validityはsensor min/max、正規化は0..25mであり別物。750点だけを正しいscanと認定しない。
final_fallback_verified=false固定、command恒常欠測をwarm-up完了にせず、controllerを呼ばない。

### mode・package・無制御性

旧design-v2/runtime-v1 schemaを変更せず、新live-passive envelopeを追加。
LIVE_PASSIVEとLIVE_PASSIVE_FIXTUREを分離し、fixtureでは実ROS/checkpoint実行はfalse。
scopeのauthorization/config/binding/code/checkpoint hashと承認者を記録し、payloadは既存design-v2 eventを使う。
実施結果はattemptと成功/UNKNOWNを分ける。control/training/promotionを有効にする設定は追加しない。
entry pointは既存spatial_path_shadow_node_v4を使用。setupに3 schemaの静的install登録を追加。
source checkout/標準ament_python layoutからV4 canonical codeを選び、V3 vendorへfallbackしない。
launchはenable=false、明示configとauthorizationをCLIへ渡す。ROS topic/namespace/parameter remapは拒否し、
launchが追加する固定node名だけを冗長引数として許す。構成対象nodeはuse_global_arguments=false。
アプリはsubscriberのみで制御/Path/speed/preflightのpublisher/client/service/actionを作らない。
ROS Humble Node内部のparameter-event publisher等までmockでゼロと証明しない。
参照: https://raw.githubusercontent.com/ros2/rclpy/humble/rclpy/rclpy/node.py
（Node constructor）、同context.py（所有Context.init/try_shutdown）、executors.py（SingleThreadedExecutor）。
実ROS import/build/launch/graph確認はNOT_EXECUTED。

### 今回だけの検証手順

Windowsで今回変更をcommit後、既定sync CheckOnly→通常sync、同じSHAで共有lock付きの次の3ファイルのみ。

```sh
tools/with_wsl_training_lock.sh env V4_BOOTSTRAP_TRACE_DIR=NEW_RUN/bootstrap V4_TRACE_DIR=NEW_RUN/runtime .venv/bin/python -m pytest -q tests/test_spatial_bootstrap_v4.py tests/test_spatial_runtime_v4.py tests/test_runtime_input_history_v3.py --junitxml=NEW_RUN/junit.xml
```

check-configは明示configを使って実行可能だが、ROS/weightsへ触れる起動コマンドは今回実行禁止。
new testはRealDependencies/load_fixed/ROS import/checkpoint・Dataset path open/statを失敗sentinelへ置換する。
fake許可fixtureだけを使用し、実環境の承認ファイルは生成しない。
同期scriptのDataset操作は既定ルートへのtest -dのみであることを静的確認。ユーザーの訂正に従い、
CheckOnly/通常syncでこの存在確認だけを許可する。script自体は変更しない。
「既定同期によるDatasetルートの存在確認」と「Dataset内容・raw・sensor・checkpoint未読取」を別記する。
結果件数・実行版・trace/export版・文書版は今回実行後の成果物に記載。旧82passを今回結果に流用しない。

REAL_ROS_RUNTIME_TESTED=NOT_EXECUTED、REAL_CHECKPOINT_READ_PERFORMED=false、
LIVE_INPUT_INFERENCE_TESTED=NOT_EXECUTED、CONTROL_CONNECTION=NOT_IMPLEMENTED。
実装と合成確認はlive起動/取得/走行の承認ではない。geometry教師/Safety/controller oracleは別gateのまま。

## 候補処理修正（合成限定、基準19c6be9/結果f7c9f39）

今回のHEAD確認はf7c9f39b6c7071cbcee502b57ecbb86ec7380894、working tree clean。
19c6be9との差はexporter1件と結果/依頼文2件のみ。旧26passは今回の結果に代用しない。
checkpoint loader/model/前処理数値/9 tensors/既存V3/design-v2/disabled launch/main BLOCKEDは変更しない。

| 修正 | 実装対応 |
|---|---|
| A 受付 | camera callbackでCandidateContextを一度作成。core.infer(candidate=...)が引継ぎ、terminal/claimedで再forward禁止 |
| B 有限待ち | pending候補上限16、max_sync_wait_ns=300000000（合成policy、校正なし）。pure tickで期限以上をDROP後に次候補へ進む |
| C 時刻 | context.selection_cutoffは選別境界。input_finalizedはcoreの9 tensor freeze/device copy完了後。event_observedはterminal処理状態を観測した時刻 |
| D 終了 | core.finishで一度terminal化してから保存。拒否DROPもhealthy writerへ。記録失敗時にも同context/output/forward countを保持 |
| E padding | adapterのsensor/ego選択mask（構造上の実在）とcommand_padding/command_framesをtransport provenanceへ。tensor maskは変更しない |

同期待ちの期限はcamera受信monotonicから評価し、期限時刻ちょうどでもDROPする。
late ego/scanをbufferへ追加する前にtickするため、期限切れcameraは復活しない。
FIFOを維持し、先頭欠損は期限までだけ待つ。pollが呼ばれることは将来bootstrap側の責任。
実ROS timerは追加/起動しない。受信停止時はpure tickをfake clockで検証する。
header逆行/monotonic逆行時は待機全候補を理由付きterminal化してからcommand含めreset。
queue overflowも旧cameraを明示DROP。非camera callbackの拒否は候補母数に加算しない。

選別cutoffはCandidateContextの独立traceとhistory.reasonに記録する（design-v2にfield追加なし）。
同じclock_id/epoch/domainの比較根拠がないdurationはUNKNOWN。別clock callableは明示指定なしに
Records時計のIDへ付け替えない。取得済み受付時刻を構築時刻へ変更しない。
input_finalizedは推論入力freeze完了境界で、後続descriptor生成の終了時刻ではない。

writer.stateはOPEN/CLOSED/FAILED。正常closeも新forwardを拒否、closeはidempotent。
最初の失敗reasonを保持。context.persistenceでdata_writeとreceipt_writeを独立記録し、
receipt失敗だけで既知の対象writeを取り消さない。write完了はdurable/fsync保証ではない。
メモリtraceには保存不能でも終端と出力を残す。bounded completed64/results4の範囲で保持し、
永続障害時の全履歴復元は保証しない。既存7段階counterは維持し、再送で増やさない。

真のwarm-upはslot.reason=WARM_UP_PADDING、存在する無効/欠損slotはPRESENT_INVALID_OR_MISSING。
commandが欠けてもpast frameのsample IDを残す。ego全feature無効でもpaddingにはしない。
tensor-onlyでprovenanceがない場合はreason=PADDING_UNKNOWN。
旧design-v2のpaddingはbooleanしかないためfalseを互換sentinelとし、その場合は実在の証拠と解釈しない。
UNKNOWNの根拠はreasonに残し、旧schemaを変更して未知を解決したふりをしない。

合成反例とtrace出力はtests/test_spatial_runtime_v4.pyに同居し、別exporter版は不要。
既定テストはファイル出力しない。V4_TRACE_DIRを新規directoryへ明示した時だけ実行中のfake eventを保存。
使用する限定コマンド例（COMMITは今回commit、既存出力は上書きしない）：

```sh
tools/with_wsl_training_lock.sh env V4_TRACE_DIR=/home/thistle/e2e_autonomous/runs/spatial_candidates_COMMIT/traces .venv/bin/python -m pytest -q tests/test_spatial_runtime_v4.py tests/test_runtime_input_history_v3.py --junitxml=/home/thistle/e2e_autonomous/runs/spatial_candidates_COMMIT/junit.xml
```

raw_execution_authorized=false、live_sensor_connection_authorized=false、shadow_connection_authorized=false、
control_connection_enabled=false、training_authorized=false、fixed_checkpoint_replay_authorized=false、runtime_promotion_authorized=false。
approval_gate=PENDING_EXPLICIT_AUTHORIZATION_FOR_LIVE_SHADOW。
今回、固定weight本体/旧fixture/raw/Dataset/実ROSへのアクセスやfixed forwardは行わない。
下のofflineコマンドは前段履歴であり、今回の実行対象ではない。

### 今回の合成確認結果

実装・テスト・trace生成版: 99892cd8f72a1d0dabf7da446a8808dbd8887dc6。
限定pytestは39 passed / 1 skipped（4.95s）。skipは既存venvにDraft2020 validatorがないためで、installなし。
初回b099c2bは37 passed / 1 skipped、その後monotonic epoch反例を追加した。旧runtimeの26passは流用しない。
traceは同じpytestから9ファイル・20候補。代表trace内のfake forward合計7回、各候補0または1回・全terminal。
これは全pytest内のmodel呼出し総数ではなく、選んだ合成traceの実数。
期限M0/M1、late source、header欠損/重複/overflow/reset、時刻境界、callback/記録例外、
close/receipt失敗、非padding欠損を新規確認した。実checkpoint読取0、fixed forward0、学習0、実ROS0。
schema全文validationはNOT_EXECUTED（skip）。今回Draft7互換checkerも再実行していない。
field間意味検証・既存のraw bits/6状態/幾何/receiptのテストは今回の限定実行に含まれる。
実装完了と合成確認は上記範囲。liveはNOT_EXECUTED、standalone bootstrapは引き続きBLOCKED/未完成。
結果追記commitとZIP梱包時HEADはpacket manifestへ記録し、repo/はテスト版へ固定する。

参照HEAD/design-v2: da8f5dc90e45f2bc80dce6651d84b509a9441042。
学習f33b197→validation153a22a→注釈2386f0f→結果7acc1ef→残差9b01b1e→記録設計da8f5dc。
モデル/入力の既存コードは変更しない。design-v2原本も変更しない。
依頼文の$id省略形ではなく、実在する urn:aic:spatial-path-v4:shadow-record:design-v2 を参照する。

## 構成

- spatial_input_v4: grid観測・受動command、4/4/10/10、availability/epoch、既存RGB/ego/command/LiDAR前処理。
- spatial_runtime_v4: 明示final.ptのみweights_only/strict restore、1 snapshotに1 forward、6状態、全state inventory。
- spatial_recording_v4: design-v2 event payload共用、SYNTHETIC/OFFLINE_TENSOR_REPLAY envelope、private bounded writer/receipt。
- SpatialPathShadowWrapperV4: V3を継承せず入力subscriberのみを注入可能。専用launchはdefault false。

旧design-v2のauthorizationは変更しない。実装版envelopeには実forward回数を記録し、live/controlはfalse。
実入力bootstrapはbindingとlive envelope承認待ちで明示的に停止する。mock wrapperの実装を実DDS起動済みと扱わない。
ROS/parameterサービスは本タスクで起動しない。wrapperはpublisher/client/service/action/parameter更新を持たない。

## 現在の入力制約

camera_delta=(camera-header−grid)/1e9、lidar_delta=(lidar-header−camera-header)/1e9。
元converterのgrid phase・画像JPEG化の影響・実beam geometry・ego補間は実sensor parity未検証。
wrapperはrgb8/750 beamsとcamera時刻に一致するegoだけを受理し、未証明の補間をしない。
nominal producerはV3 parameterの候補だけで実在未確認。final fallbackは未承認で無効。
供給元不明はBLOCKED、有効0で埋めない。warm-upだけはfalse masks。
queue4、record128KiB、保存8MiBは合成用の有限設定で、実負荷で適切という実測はない。
保存失敗は新推論を停止するだけで車両へ停止commandを送らない。

## 許可された検証コマンド

Windowsで今回変更をcommit後、tools/sync_to_wsl.ps1 -CheckOnly → 同script通常sync。
このscriptの既定Dataset存在確認はdirectory存在チェックのみで、Dataset内容は読まない。
WSLで同一commit、worktree lock保持下で以下のみ（全pytest・学習tests禁止）：

```sh
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_spatial_runtime_v4.py tests/test_runtime_input_history_v3.py --junitxml=/tmp/v4-runtime-junit.xml
tools/with_wsl_training_lock.sh .venv/bin/python tools/check_spatial_runtime_v4_offline.py --output /home/thistle/e2e_autonomous/runs/spatial_runtime_v4_UNIQUE
```

実際の出力先・コマンド・版は新規runのlogs/provenanceに保存する。例のUNIQUEを既存runへ置換して上書きしない。
offlineはinput_0/1と対応selection/history/predictionだけ、batch1各1回、固定6forward上限。
rtol1e-5/atol1e-6m固定。未達でも入力・重み・precisionを選び直さない。
pure testではin-memory synthetic assetだけでoffline build_inputsを比較する。

## 完了判定と残る承認

実測結果は後続報告とexecution manifestに記載する。現時点では未検証を合格としない。
live入力/実時間性能/実graph/Safety/走行可能性はNOT_EXECUTED。
CONTROL_CONNECTION=NOT_IMPLEMENTED。MPC完成や全raw監査を前提にしない。
次段には受動source、時刻/epoch、grid phase、ego整合、beam geometry/QoS、容量・停止条件、live envelope/bootstrapの別承認が必要。
将来MPCには基準点、現在時刻への変換、速度参照、必要経路長・validityの別interface検証が必要。今回実装しない。

## 候補処理の残存2点修正（2026-09-06、合成限定）

基準99892cd、開始HEAD 2026941（差分は前回結果文書のみ、作業開始時clean）。
修正前は静的反例として確認し、修正前コードの新反例testは実行していない。

- wrapper.__init__: min比較を廃止し、grid_period_ns/max_sync_wait_ns/candidate_capacityを
  それぞれtype(value) is intかつ正数へ限定。bool/float/NaN/Infinity/string/Noneを副作用前にValueError。
  既定100000000/300000000/16、同期選択・期限・容量アルゴリズムは変更なし。上限の追加やlive校正なし。
- PrivateWriter.drain: 主recordの書込み完了が確認済みなら、後続receipt失敗時のfailにcandidateを渡さない。
  savedは維持し、既存の処理DROP所属も消さない。主record失敗と未保存queueは従来どおりdropped。
  receipt_writeは検証/encode/size確認後、実write直前にUNKNOWNへ移す。準備失敗はNOT_ATTEMPTED。
  主record完了のsaved加算は後続の時計観測より先。OS write完了はdurable保証ではない。
- tests: 3引数×11不正値、既定/正整数、FRAME/DROP×receipt成否、部分主record失敗、
  receipt失敗と未保存queue、receipt準備失敗を検証。7段階candidate ID集合、first error、
  terminal/forward/persistenceを段階付きで既存trace exporterへ保存し、実際の一時JSONL bytesも別保存。

今回の実行対象は次の2ファイルのみ（上記の過去offlineコマンドは今回実行禁止）。

```sh
tools/with_wsl_training_lock.sh env V4_TRACE_DIR=NEW_VERSIONED_RUN/traces .venv/bin/python -m pytest -q tests/test_spatial_runtime_v4.py tests/test_runtime_input_history_v3.py --junitxml=NEW_VERSIONED_RUN/junit.xml
```

Windows局所commit後、既定CheckOnly→通常sync→同一SHAのWSL lock下で限定検証する。
生stdout/stderr、exit code、環境・実際のコマンド・SHAは新規runへ保存。結果は実行後追記する。
model/loader/9 tensors/未補正bits/design-v2/V3/Safetyは変更しない。
固定checkpoint読取/固定forward/実Dataset内容/raw/学習/ROS接続/走行/pushは禁止。
LIVE_BOOTSTRAP_BLOCKED、disabled launch、PENDING_EXPLICIT_AUTHORIZATIONを維持。

### 今回の限定実行結果

実装/test/trace版: `60b3da378b1dee37bfcd3d8841b519b1d07e8d0c`。
CheckOnly=CHECK_OK、通常sync=SYNC_OK。WSLは同一SHA・clean・共有lock下で実行。
出力: `/home/thistle/e2e_autonomous/runs/spatial_two_fixes_60b3da3`。
`82 passed, 1 skipped, 1 warning in 5.97s`、pytest終了コード0。
Python3.10.12、torch2.7.1+cu128、numpy2.2.6、pytest9.1.1、Pillow12.2.0。
skipはjsonschema未導入。完全Draft2020検証と別途Draft7互換checkerは今回未実行。
前回39 passedの流用ではない。修正前反例は静的確認のみ。
今回生成したtrace JSON49（うち不正設定33）、一時JSONL8（うち17bytesの部分書込み1）。
代表候補trace30・fake forward13（選択trace内のみ。pytest全体や固定checkpoint推論総数ではない）。
receipt-only失敗直後はsaved={0}, dropped={}。その後の拒否候補1のみdroppedへ加算。
処理DROPのreceipt失敗はsaved={0}, dropped={0}を維持。
部分主record失敗直後はsaved={}, dropped={0}、data=UNKNOWN/receipt=NOT_ATTEMPTED。
未保存queue付きreceipt失敗直後はsaved={0}, dropped={1}、queue空、候補1のdata/receiptはNOT_ATTEMPTED。
receipt準備失敗はdata既知/receipt=NOT_ATTEMPTED、saved={0}, dropped={}。
各集合は別test/session。first error・失敗後forward禁止・再finish不変をassertした。
自己点検/合成確認であり独立レビュー合格ではない。固定checkpoint読取/固定forward/実接続/学習/走行0、pushなし。
