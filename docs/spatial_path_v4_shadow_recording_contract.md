# Spatial Path V4 無制御shadow最小記録契約 — 設計のみ v1

## 承認状態・参照版

```yaml
design_only: true
new_inference_authorized: false
new_collection_authorized: false
shadow_connection_authorized: false
control_connection_enabled: false
raw_execution_authorized: false
runtime_promotion_authorized: false
approval_gate: PENDING_EXPLICIT_AUTHORIZATION
```

Repo: https://github.com/fis-teria/aichallenge_lite_transfuser

Branch: `codex/windows-wsl-training-sync`。読取時HEADは`41c7bcf92c62c6081028fde5072a63747712c2ef`、working treeはclean。
残差実行`9b01b1e2ed6fc89434316c7db409d376e56aeb03`→結果`21c132bf3be050e00081cbc89e7effc63a5f4400`
の差分は結果文書1件、→HEADの差分はレビュー依頼と梱包tool2件。残差コードの再解析なし。
固定残差成果物の小さな`provenance.json`と現行コードを静的参照した。
今回の作成物は本書と`schemas/spatial_path_v4_shadow_record_v1.schema.json`だけ。
コード実行、JSON Schema検証、test、推論、checkpoint/Dataset/raw/sensor読取、WSL同期、commit/pushはしない。

Schemaは本設計のsynthetic envelopeを記述する。`authorization`のfalseを反転して稼働許可にしてはならない。
将来の実装・起動・記録は別承認manifestと承認されたschema版が必要。本仕様の存在は承認ではない。
geometry教師採用、stop/launch labels、motion permission/Safety、controller oracleは別gate。

## 実在実装との対応表（パスはrepo相対、静的確認）

| 記録対象 | 実在file/function/field | 状態と設計上の差分 |
|---|---|---|
| 未補正20点 | `src/aic_transfuser_lite/models/spatial_path_diagnostic_v4.py` / `SpatialPathDiagnosticV4.forward` / `result` [B,20,2] | EXISTS。`targets=None`に置換、Tensorを返す。ROS用`ModelOutputV3`ではない |
| 4/4/10/10入力 | `src/aic_transfuser_lite/data/spatial_diagnostic_inputs_v4.py` / `build_inputs`, `INPUT_FIELDS`, `INPUT_CONTRACT` | EXISTS、offline専用reader。これをruntimeで呼ぶ設計ではない |
| input provenance | 同上 / `provenance.sensor_ids`, `sensor_stamps_ns`, `ego_ids`, `command_ids`, `command_stamps_ns`, `command_sources` | 部分的EXISTS。受信・実利用可能時刻/clock domainはMISSING |
| 因果command | 同上 / `select_epoch_history_before_anchor`, `BOUNDS`, `command_mask` | offlineは厳密に過去grid slot。実利用可能時刻の証明はUNKNOWN |
| tensor masks | `src/aic_transfuser_lite/contracts/model_batch_v3.py` / `ModelBatchV3`のimage/lidar/ego_feature/command masks | EXISTS。teacher maskと区別する。tensor ID/hashは新規設計 |
| reset/履歴 | `src/aic_transfuser_lite/runtime/input_history_v3.py` / `RuntimeObservationTensorV3.stamp_sec`, `RuntimeObservationHistoryV3.append/reset` | 時刻非増加/gapでreset、既存理由文字列あり。session/reset ID・clock epoch台帳はMISSING |
| runtime padding | 同上 / `_left_padded_observations`, `build_runtime_temporal_batch_v3` | 先頭反復false mask、commandゼロpadding。4/4/10/10完全parityを今回保証しない |
| command runtime型 | `src/aic_transfuser_lite/runtime/residual_control.py` / `ExternalControllerCommand` | steering/speed/accelerationのみ。stamp、receipt、available時刻はMISSING。型の参照のみでcontroller呼出しなし |
| command履歴の供給 | `ros2_ws/src/aic_e2e_runtime/aic_e2e_runtime/inference_node_v3.py` / `nominal_command_history`, `_publish_trajectory_authoritative`, `_publish_gated_full_control` | 制御publish側でappendする既存経路。無制御V4のcommand入力源としてそのまま利用不可 |
| sensor時刻/同期 | 同node / `_add_sensor`, `_on_image`, `_drain_camera_queue`, `ReadyObservation.role_stamps_sec` | header-derived時刻と同期差はEXISTS。sensor物理取得時刻との一致、全sensor receiptはUNKNOWN/MISSING |
| ready/drop/status | 同node / `_queue_ready_observation`, `_drain_ready_observations`, `runtime_status`, `runtime_sync_debug` | overflow等の文字列出力あり。永続sequence gap、logger状態、失敗frameの構造記録はMISSING |
| 入力/forward境界 | 同node / `_process_observation`, `_make_batch` | `self.model(batch)`の1回呼出し境界あり。ただしV3はtrajectory+speed出力を要求。V4を差すだけでは不適合 |
| clock | 同node / `_receipt_time_sec` = `self.get_clock().now()` | ROS clock。monotonicではない。推論/保存のmonotonic境界はMISSING |
| timingの意味 | V4 `INPUT_CONTRACT.sensor_dt_meaning_s` とV3 node `_make_batch.sensor_dt` | V4はcamera_delta_ms/lidar_delta_ms、nodeは[0, lidar−camera]。名前一致は意味の一致ではなく、runtime adapter parityは未検証 |
| 0.3m弦/点間隔 | `tools/analyze_spatial_fixed_residuals_v4.py` / `tangent_window`, `summarize` / `GRID`, `WINDOW_STEPS`, `DEGENERATE_EPS_M` | 既存offline定義だけ参照。runtime実装なし。maskによるteacher比較部分は持ち込まない |
| 元frame | V4 teacher/input記録の`base_link@t_obs`、V3 `trajectory_frame_id`, `PathMessage.header` | 原点はbase_link。後輪中心一致UNKNOWN。TF変換証拠/隣接drift記録はMISSING |
| 既存shadow launch | `ros2_ws/src/aic_e2e_runtime/launch/transfuser_lite_v3_shadow.launch.py` / `generate_launch_description` | `runtime.v3.shadow.param.yaml`: `runtime_profile=shadow_control`。XY記録専用ではない |
| external shadow launch | 同launchディレクトリ`transfuser_lite_v3_external_controller_shadow.launch.py` | `runtime_profile=external_controller`。controller計算/`shadow_external_control`を伴うので今回流用不可 |

本表は見た範囲の静的対応で、全依存閉包や実node graphの監査合格ではない。
既存V3 nodeの説明文に「without a control publisher」とあっても、実際の分岐を優先する。

## 識別とレコード単位

初版は1入力候補につき1frame event、batch size=1を設計上固定。候補受付時にsequenceを採番し、
session内でreset後も再使用しない。session_id/process_boot_id/reset_id/clock epochを別々に持つ。
`record_id`はframeだけでなくDROP/RESET/LOGGER_HEALTH/SESSION_ENDにも一意。
input_builder_idは入力snapshot確定時、forward_invocation_idは呼出し開始時、output_idは戻り値取得時に一度発行。
未構築/未推論/例外はそれぞれnullと理由、成功を装うダミーID/ゼロXYは不可。

hashはSHA-256。code=実装artifact bytes manifest、model=architecture/forward仕様、checkpoint=固定重み、
config=解決済み設定、input/output contract、schemaを別々に記録する。Git SHAとSHA-256を混ぜない。
KNOWNには実値と取得根拠、MISSING/UNKNOWNにはnullと理由。セッション開始時のimmutable manifestで
共通hashを定義し各recordへ同じ値を展開する。未検証の文字列をKNOWNとして埋めない。
今回checkpoint本体を読まない。既存metadataに期待hashがあっても現物照合済みとは記載しない。
runtime output contract専用hashとinput_builder ID方式はMISSING（将来の仕様確定時に生成）。

## 同じforwardの未補正出力と派生幾何

承認後の構成案：同じforward戻り値→immutable snapshot→bounded非同期logger。logger用再forward禁止。
同期copyでGPU待ちが起きるならその待ち時間も境界へ含め、遅延を隠さない。snapshot時点のdtype/device/shape、
40 scalarのfloat32 little-endian bit列（160 bytes、hex320文字）とtensor hashを保存する。
`model_xy_m`は[20,2]、nominal_s_mは0.1…2.0m。NaN/InfはJSONの不正numberではなく指定文字列にし、
raw bitsでNaN payloadや−0も保持。丸め、終点複製、原点挿入、bias補正、平滑化、距離伸縮はしない。
shape不適合時はactual_shape/dtype/理由を残し、model_xy_m=null。巨大な不正tensorの全dumpはしない。

有限性/shapeの成功ラベルは`SHAPE_FINITE_ONLY`。経路valid/safe/go等の意味を付けない。
NONFINITEでは元bit列を残す。派生値は非有限に依存する部分をnull/理由、別の有限点へ橋渡ししない。
速度・control要求・teacher mask・future由来shapeをevent/forwardに含めない。Schemaのeventは追加field禁止。

- 点p0…p19、o=(0,0)。spacing[0]=|p0−o|、spacing[k]=|pk−p(k−1)|。
  cumulative[k]=spacing[0…k]の和。index0だけorigin_to_first、残りは実点間隔。
- chord j=0…16: pj→p(j+3)。start_s=grid[j], end_s=grid[j+3]、窓0.3mは名目sで時間でも実弦長でもない。
  長さLとatan2方向radを保存。原点不使用、j順・17窓を守る。
- L<=既存数値定義1e-12mはUNKNOWN_DEGENERATE、非有限依存はUNKNOWN_NONFINITE。
  これは数値上の方向不定判定で、teacher/tier/安全thresholdを変えない。
- 派生幾何はraw snapshotだけから再計算できる。teacher maskなしで全20点を扱うが、利用可能性は予測しない。

## clock・history・未推論frame

時刻はns整数をJSON文字列で保持し、ROS_SIM/ROS_SYSTEM/MONOTONIC/DEVICEを明示。
同domainでもclock_id/epoch_idが違えば直接差分しない。ROS use_sim_time変更、巻戻り、jumpでepochを更新。
monotonic時計もprocess boot/host識別が必要。device/header/receipt時刻を同一と推測しない。
入力確定/推論開始終了/enqueue/commitは同じmonotonic基準を優先。t_obsは元観測基準のROS/device時計。
cross-domain end_to_end_ageはclock対応証拠なしならUNKNOWN。負ageを0にclipしない。

camera4/lidar4/ego10/command10の各slotにsample ID、source、padding、実際のmask、ego feature mask、
取得/header/ROS受信/monotonic受信/利用可能時刻、input確定時ageを残す。paddingは元参照IDを残せるが
新しい実測sampleとして数えない。input未確定はstatus=NOT_BUILTで利用可能なslotだけ（最大長まで）。
欠けた時刻はMISSING/UNKNOWN。受信時刻で取得時刻を埋めず、hashで時刻対応が証明されたとしない。

commandはsource timestamp<t_obs（同時計または対応証拠）、かつ利用可能monotonic<=input確定を別条件とする。
前段offline gridが過去でもreceiptが未来なら実運用の因果性は証明されない。出典nominal/final fallback、
選択したcommand ID/値のhash、maskを保持。現在のExternalControllerCommandにはこれらの時刻がなくMISSING。
既存controllerを呼んで履歴を満たしたり、将来commandを遡及挿入したりしない。入力源は別設計承認が必要。

warm-up/stale/duplicate/clock reset/例外でも受付sequenceを記録。既存の処理方針をこの記録仕様で変更せず、
推論しなかった場合NOT_INFERRED、同理由を残す。resetでセンサとcommand履歴を一緒に区切ることを要求。
成功frameだけのレイテンシ平均は不可。受付/構築/呼出し/戻り/保存/欠損の各母数、未観測時間区間を併記。

## frame変換とoffline sidecar

rawは`base_link` at t_obs、その元原点のまま保持。base_link=後輪中心とは仮定しない。
変換済み診断はtransforms[]に分離し、source_output_id、source/target frame、双方の基準時刻、
4×4 T_target_from_source、取得元、補間/外挿の有無、age、clock対応証拠を必須にする。
変換に根拠がないなら配列を捏造せずtransforms=[]、drift=UNKNOWN/TF_MISSING等の理由。
変換後の元原点はT*[0,0,0,1]で、現在ego原点(0,0,0)へ置換しない。
回転の正規性、行列の向き、元t_obsとtarget時刻を将来の意味検証で確認する。
隣接出力driftは共通frame/共通比較時刻・両出力ID・各変換証拠・clock連続性が揃った時だけ
`ELIGIBLE_FOR_SEPARATE_DIAGNOSTIC`。本v1にdrift数値の算出処理はなく、TFなし/reset跨ぎはUNKNOWN。

offline_join_sidecarは別ファイル/別経路。join keyはsession/reset/sequence/input_builder_id/output_idに
元record hashを加え、時刻の最近傍だけで一意joinとしない。teacher contract/hash、20点mask、切断理由、
annotation根拠hash、後処理code hash、join状態を保存する。UNKNOWN/KNOWN_ZEROを区別し、未知tailを
ゼロ教師として採点しない。重複/曖昧joinはAMBIGUOUSで自動解決しない。未来情報をinput builderへ逆流させない。
今回はsidecarの仕様だけで、future生成/収集/教師変更はしない。

## 無制御性の構成と証拠

将来の最小構成は「承認された受動入力→V4同一forward→private保存queue→ファイルwriter」のみ。
最初の段階ではXYを既存trajectory/speed/control topicへpublishせず、controller/Safetyの関数も呼ばない。
実nodeやtopic名、launch/remapはMISSING（今回追加なし）。入力subscriberも別承認。
loggerが吐く処理状態も初版はprivate記録のみとし、共有preflight/arming経路へ出力しない。

既存V3 nodeには以下の到達経路があるため、そのlaunchを名前だけで流用しない。

| 種別 | 静的に確認した経路 | 将来の遮断/証拠 |
|---|---|---|
| XY/speed publish | `__init__`のpredicted_trajectory/path/speed_profile、`_process_observation`のpublication | V4 recorderから到達不可。デバッグpathをcontrollerが購読しない解決済みgraphが必要 |
| shadow制御publish | shadow_external_control、shadow_model_control、shadow_model_control_sequence | 全部非生成・非呼出し。Ackermann型でもdebug名なら許可、とはしない |
| 実制御経路 | FULL_CONTROL/TRAJECTORY_AUTHORITATIVEのnominal_control_cmd | 到達不可。`_publish_gated_full_control`等のguard任せで許可しない |
| controller計算 | `_publish_shadow_external_control`, `_external_control_from_trajectory`, `_publish_trajectory_authoritative` | 計算だけでも今回対象外。callgraphで不使用を示す |
| preflight | `_control_preflight`とgear/control_mode/awsim/race_armed subscriptions | recorderからarming/parameter/service等への作用経路なしを確認 |
| parameter/service/action | nodeにdeclare_parameterを確認。今回の検索範囲ではcreate_client/service・ActionClient/Server・set_parametersの直接呼出しは見つからない | 依存/ROS既定parameter serviceを含む実endpointはUNKNOWN。検索不在を無書込みの証明にしない |
| launch/remap | 両shadow launchのparam_file/model_path/use_sim_time、image/scan/velocity/steering remap、任意RViz | 解決後の全引数/namespace/remap/parametersと起動graphをhash記録。外部param_file差替えにも注意 |

実装承認後に必要な証拠は、(a)実行版と依存閉包のwrite/callgraph、(b)解決済みlaunch/remap、
(c)publisher/subscriber/service/action/parameter endpoint graph、(d)権限deny/allow設定、
(e)正常/異常/終了全経路のwrite試行監査。制御topic、制御要求service/action、parameter set、
preflight/arming、間接bridgeにも到達しないことを確認する。必要なROS内部通信の範囲は別承認で限定。
`publish_count=0`や一時点graphだけでは、無購読時・異常時・動的remap後の無制御性は証明できない。
no_control_evidenceは初期NOT_VERIFIED、各証拠はMISSING。証拠を得ても安全/走行gateへ昇格しない。

## 負荷・logger停止

queue容量/byte予算/drop方針/保存期限/ローテーションは実装承認時に有限値で固定（現時点MISSING）。
logger待ちでセンサ処理や他controllerをblockしない構成を要求。queue深度、drop累計/sequence範囲、
backpressure状態、保存失敗、last committed sequence、health証拠を記録。overflow時にもdrop記録が
同queueで失われ得るため、別のbounded health経路/セッション終端集計が必要。
logger停止後にlogger自身が正常ログを残せるとは仮定しない。別監視のheartbeat/exit情報がない場合は
最後の確定sequence以降をINCOMPLETE/UNKNOWNとし、欠損0や成功終了を補作しない。
record_committedは書込み前の予測時刻ではない。writer receipt/後続healthイベントで確定させ、
未確定recordではMISSING。元record bytesの書換えで自己hashや確定時刻を循環させない。
sensor/input tensorの全保存は既定禁止。hashだけでは元sensorや前処理の独立再現はできない。

## 合成例と将来の検証表（記述のみ・未実行）

以下は実測でも実行可能testコードでもなく、将来実装者が作るfixtureの設計である。

| 合成例 | 想定record/期待結果 |
|---|---|
| 正常 | 同clock epoch、4/4/10/10、過去かつ利用可能command。1 forward IDからfloat32[1,20,2]取得。17弦と20spacing、SHAPE_FINITE_ONLY。制御安全を主張しない |
| warm-up | 最初の観測をpadding反復してfalse masks。未推論方針ならrecord.output.model_xy_m=null、record.output.status=NOT_INFERRED、record.status=WARM_UP。推論可否の既存方針を記録仕様で決め直さない |
| stale | 受付sequenceあり、既存timing判定理由付きSTALE/NOT_INFERRED。推論durationはUNKNOWN/未実施、レイテンシ母数に受付として残る |
| clock reset | ROS_SIM 10s→2s、monotonicは増加。reset_id/ROS epoch更新、history断絶、drift UNKNOWN。−8sを処理時間としない |
| TFなし | raw20点は保持。transforms=[]、drift UNKNOWN/TF_MISSING。現在ego原点へ付け替えない |
| 非有限 | 1要素NaNのbit列とtag保持、NONFINITE。該当弦/累積の不定をnull、ゼロ埋め/再推論なし |
| 方向不定 | p0=p3等のzero chord、length=0/direction=null/UNKNOWN_DEGENERATE。正しい直進角0とは区別 |
| logger停止 | 最終commit sequence=42、以降の監視断絶。health証拠なしならINCOMPLETE、欠損数UNKNOWN。43以降の成功frameを捏造しない |

| 将来test項目 | 検証仕様 |
|---|---|
| teacher漏洩なし | event schemaでteacher field拒否、sidecarからforwardへ到達不可。teacher変更が入力snapshot/outputを変えないことを別承認下で確認 |
| 時刻・履歴対応 | slot ID/実mask/feature mask、padding、利用可能時刻、clock epoch、reset時command断絶を照合。違反をUNKNOWNで隠さずVIOLATION |
| frame変換 | 既知回転/並進でmatrix方向・両時刻・変換後元原点を照合。TF欠落/reset跨ぎはdrift不成立 |
| 未補正値不変 | 同forward戻り値とraw snapshot bit列/hash一致、記録経路の再forward0、NaN/−0保持、shape例外/巨大tensor上限 |
| geometry | j順0…16、end=start+0.3、弦長/角差/累積、非有限伝播、原点区間の区別。schemaだけでは数式一致を保証しない |
| 制御書込みなし | 正常/例外/reset/overflow/shutdown/param差替えの全分岐でcallgraph・endpoint・write試行・権限監査を突合 |
| 負荷/欠損 | bounded queue overflow、disk full、writer kill、heartbeat欠落、sequence gap/重複を合成し、成功のみ集計を拒否 |
| sidecar join | 元record hash/ID完全一致、別session同時刻/重複候補を拒否、UNKNOWNとknown zero区別 |

JSON Schemaは構造と一部条件を表現するだけ。hexとXYの一致、hash計算、slot index順、時計の比較可能性、
mask因果性、行列正規性、17弦の数式、field間status整合、無制御性は上記の将来意味検証が必要。
本タスクではschema parse/validationを含めテスト未実行。既存情報の不足を合格扱いにしない。

## 別承認が必要な段階

まず仕様/未知fieldをレビューし、runtime用input adapterの時刻意味・command入力源とparity方針を確定する。
次に専用の無制御node/logger/clock/graph証拠収集を実装する承認、合成testの承認を得る。
その後に対象入力・固定checkpoint照合・実行環境・容量/保存期間を限定した推論/記録/接続開始を別承認する。
controller要求計算、速度参照、publish、Safety/MPC連携、走行はさらに別gateで、今回の仕様から派生承認されない。
現runtimeを縦横MPC完成済みとせず、残差解析の再合格待ちへ戻さない。
