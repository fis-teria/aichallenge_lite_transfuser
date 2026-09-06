# 固定step500 V4 runtime：実装と限定検証

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
