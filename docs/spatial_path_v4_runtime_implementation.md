# 固定step500 V4 runtime：実装と限定検証

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
