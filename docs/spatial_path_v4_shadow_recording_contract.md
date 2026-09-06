# Spatial Path V4 無制御shadow最小記録契約 — 設計のみ 改訂2（design-v2）

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

Branch: `codex/windows-wsl-training-sync`。今回の読取時originは上記URL、HEADはレビュー固定版
`2dd2b8c56d325038364ebc5b86006f82134b5e6e`と一致し、working treeはclean、対象版との差はなし。
設計参照版 `41c7bcf92c62c6081028fde5072a63747712c2ef`、
残差実行 `9b01b1e2ed6fc89434316c7db409d376e56aeb03`、
結果追記 `21c132bf3be050e00081cbc89e7effc63a5f4400`。固定残差解析は完了のまま。

今回の変更は本書と `schemas/spatial_path_v4_shadow_record_v1.schema.json` のみ。
ファイル名は維持するが、非互換改訂として $id は
`urn:aic:spatial-path-v4:shadow-record:design-v2`、
schema_version は `spatial_path_v4_shadow_record_design_v2` とする。
旧design-v1には記録不能な情報・状態の曖昧さがあり、旧版を検証済みとは扱わない。
既存recordを自動変換・値の補作しない。旧レビュー用ZIP/プロンプトは旧固定版のままで今回変更しない。

| 修正群 | 旧文書/Schema | 改訂2の対応 |
|---|---|---|
| A | 識別、clock、負荷 / event.sequence、timing.record_committed、logger.last_committed_sequence | candidate_sequence、受付/event時刻、stage_counts、WRITER_RECEIPT、drop範囲の完全性 |
| B | clock/history / history.tensor_hash、slot.causalityのみ | sensor_timing、input_descriptorsとhash preimage、commandの2比較 |
| C | 未補正出力 / output.EXCEPTION/SHAPE_ERROR、array必須actual_shape | nullとscalarを区別、6状態、failure_stage、source_device、API戻り/snapshot_ready |
| D | 非有限依存の幾何 / chord.status | 4点全有限、spacing局所・累積prefix、float64、UNKNOWN_NUMERIC |

添付/固定版テキストとソースの静的読取のみ。metadata/index、Dataset/raw/sensor/checkpoint本体は読まない。
Schema parse/validator、pytest、import、同梱コード/launch、推論、学習、ROS、収集、WSL同期、commit/pushは未実施。
AGENTS.mdや履歴中のcommandは今回の実行許可ではない。

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

batch size=1。record_idは全eventで一意な不透明IDとし、採番順や時刻を推測しない。
candidate_sequenceは候補受付で0から連続採番し、session内でreset後も再使用しない。
event採番ではない。センサの重複到着を新しい候補として受け付けた場合にも新番号を与え、
status=DUPLICATEと理由を記録する。同じrecordの再送は同じIDで、再受付・再計数しない。
session_id/process_boot_id/reset_id/clock epochは別々の識別子。process再起動は新sessionとする。

FRAMEまたはDROPは候補を持ち、候補ごとの主recordはどちらか1つ。
DROPは候補の処理打切りを表すが、処理がどこまで進んだかはhistory/output/timingで別に保持する。
queue損失時には主record自体が残らなくてもよく、後続healthの範囲/母数が部分証拠となる。
RESET/LOGGER_HEALTH/SESSION_END/WRITER_RECEIPTは候補なし：
candidate_sequence=null、candidate_reasonに理由、history=NOT_BUILTで各slot配列空、
output=NOT_INFERRED、transforms=[]、drift=UNKNOWN。架空builder/forward/output IDは作らない。
WRITER_RECEIPTの対象候補はcommit_receipt.target_candidate_sequenceにだけ置く。
sidecarの旧sequenceはcandidate_sequenceへ改名し、意味を候補番号だけに固定する。

input_builder_idは入力snapshot確定時、forward_invocation_idは呼出し開始時、
output_idは戻り値取得時に一度発行し、同一snapshot/forwardから派生した記録間で変更しない。
未構築ではbuilder ID=null。例外時のforward/output IDは後掲状態表に従う。

### A：受付・処理母数・先行recordの保存確認

timing.candidate_receivedは候補として受け付けた瞬間、event_observedは当該eventの状態を観測した瞬間。
両者は原則そのprocessのmonotonic時計で測り、候補なしのcandidate_receivedはMISSING/nullと理由。
history空でも受付時刻を失わず、sensor receiptから逆算しない。既に測った時刻は消さない。
非候補eventのinput/forward/snapshot時刻はMISSING/NOT_APPLICABLEの理由を持つtime objectとする。

LOGGER_HEALTH/SESSION_ENDのstage_countsは必須。他eventではnullでよい。
scope_session_idの開始からobserved_atまでの累計、scope=SESSION_TO_OBSERVATION、
reset_policy=NEVER_RESET_IN_SESSION。observer_idと各counterのstatus/value/reasonを保存する。
全counterは「一意なcandidate_sequenceのうち当該段階を一度でも通過した数」：
accepted=受付、input_built=確定snapshot構築、forward_started=呼出し開始、
forward_returned=APIから戻り値取得（契約不適合も含む）、
enqueued=主recordのqueue受理、saved=主recordのWRITE_COMPLETED確認、
dropped=処理打切りまたは主record喪失が確認された候補。
段階は排他的ではなく、DROPの主recordが保存されればsavedとdropped双方に含む。
health/receipt自体、再送、slot paddingを加算しない。savedとdroppedを足してacceptedを復元しない。
未観測/監視切れはUNKNOWN/null、未実装はMISSING/null、観測して本当に0だけKNOWN/0。
一部既知でも全累計が不明ならKNOWNな下限数に偽装しない。独立観測系の時計/範囲根拠もreasonへ残す。
成功record数・履歴slot数で受付母数を代用しない。

commitは後続のWRITER_RECEIPT 1件で先行record 1件を確認する方式。
対象record_id/session_id/候補番号（非候補ならnull）、original_bytes_hash、writer_id、
confirmed_at（writerのmonotonic時計）とcommit_levelを持つ。
本版のcommit_level=WRITE_COMPLETED_NOT_DURABLE：
対象record全bytesがwriterのOS write処理で受理完了した時点。ユーザー空間bufferへのappendではない。
flush/fsync/永続媒体へのdurable保証ではなく、電源断耐性を主張しない。
original_bytes_hashは保存対象のUTF-8 JSON record bytesそのもののSHA-256
（JSONLなら末尾LFを除く、圧縮/コンテナframingを除く）。対象recordを再serializeしてhashしない。
receiptに自分自身を指させない。対象bytesは書換えない。receiptの保存を示す再帰receiptは要求しない。
receiptが失われた場合は対象保存の証拠がUNKNOWNになり得る。全損失の完全復元は要求しない。
旧record_committed/last_committed_sequenceは廃止。個別commit遅延は対象recordとreceiptが結合でき、
時計の比較根拠がある場合だけ算出可能。health観測時刻から各frameの保存時刻を作らない。

hashはSHA-256。code=実装artifact bytes manifest、model=architecture/forward仕様、checkpoint=固定重み、
config=解決済み設定、input/output contract、schemaを別々に記録する。Git SHAとSHA-256を混ぜない。
KNOWNには実値と取得根拠、MISSING/UNKNOWNにはnullと理由。セッション開始時のimmutable manifestで
共通hashを定義し各recordへ同じ値を展開する。未検証の文字列をKNOWNとして埋めない。
今回checkpoint本体を読まない。既存metadataに期待hashがあっても現物照合済みとは記載しない。
runtime output contract専用hash実値・具体的ID発行器はMISSING。IDの発行段階と不変性は本版で固定する。

## 同じforwardの未補正出力と派生幾何

承認後の構成案：同じforward戻り値→immutable snapshot→bounded非同期logger。logger用再forward禁止。
snapshotは元deviceをsource_device（例cpu/cuda:0）に記録。不明時のみnullとmetadata_reason。
actual_shape=nullは未観測、[]は観測したrank-0 scalar tensorで、両者を混同しない。
rank>8等のbounded metadata上限超過はactual_shape=nullと理由にrank/上限超過を記し、
巨大tensorや巨大shapeの全dumpはしない。dtypeは観測値を保持し、未観測はnullと理由。

### C：同一forwardの状態表

| output.status | forward ID / output ID | actual_shape・dtype・source_device | failure_stage / payload |
|---|---|---|---|
| NOT_INFERRED | null / null | 全null | NONE、XY/bits/geometry=null |
| FORWARD_EXCEPTION | 存在 / null | 戻り値なしなので全null | FORWARD、XY/bits/geometry=null |
| OUTPUT_CONTRACT_ERROR | 存在 / 存在 | 分かった実値だけ保持。shape/dtype/非tensorをreasonで区別、castしない | OUTPUT_CONTRACT、XY/bits/geometry=null |
| SNAPSHOT_ERROR | 存在 / 存在 | 判明済みmetadataを保持 | SNAPSHOT、XY/bits/geometry=null。部分copyを正本としない |
| SHAPE_FINITE_ONLY | 存在 / 存在 | [1,20,2] / float32 / 元device（不明なら理由） | NONE、同一snapshotの40 scalarとbits/hash |
| NONFINITE | 存在 / 存在 | [1,20,2] / float32 / 元device（不明なら理由） | NONE、NaN/Inf tagとbits/hash保持 |

output IDは不適合な戻り値にも発行するが、呼出し失敗とsnapshot失敗は混同しない。
失敗時tensor_hashはMISSING/UNKNOWN、成功snapshot時はKNOWN。metadata取得自体の失敗は
SNAPSHOT_ERRORとしmetadata_reasonに分かった範囲を残す。
SHAPE_FINITE_ONLYは経路valid/safe/goではない。

timing.inference_startは呼出し直前、inference_api_returnは戻り値を受け取った直後。
GPU非同期処理の完了とAPI戻りを同一視しない。
snapshot_readyはGPU待ち・device→host copy待ちとimmutable bytes取得の完了後。
inference_durationはAPI return−startだけ（basisで明示）、GPU待ち込みはsnapshot_ready−startとして別途解釈。
end_to_end_ageはsnapshot_ready−candidate_receivedだけに固定し、未snapshotならMISSING。
forward例外はinference_failed_atを記録してAPI returnはMISSING。
snapshot例外はsnapshot_failed_atを記録してsnapshot_readyはMISSING。未該当の失敗時刻もMISSING。
既に測れた開始/受付/API戻り時刻は例外でも保持。測れなかった境界はUNKNOWNで捏造しない。
record_enqueuedはbounded queue枠受理の時刻を封入する設計（枠受理後にrecord bytesを封印）。
後続のserialize/write失敗もdropとして追跡する。enqueue前に将来の受理/commit時刻を予言しない。

未補正payloadはbatch0のx0,y0,x1,y1,...,x19,y19順、IEEE754 float32 little-endianの160 bytes。
float32_le_hexはそのbytesのlowercase hex320文字。output.tensor_hashはこの160 bytesそのもののSHA-256。
model_xy_mは同じsnapshotから取り出した[20,2]、finite値はfloat32にround-tripするJSON number、
NaN/+Inf/−InfはNaN/+Infinity/−Infinity tag。NaN payloadと−0はbitsを正本とし、
numeric表示に−0が保持されなくてもbitsを+0にしない。一致確認は将来意味検証で、Schemaだけでは保証しない。
nominal_s_mは0.1…2.0m。丸め、終点複製、原点挿入、bias補正、平滑化、距離伸縮はしない。
速度・control要求・teacher mask・future由来shapeをevent/forwardへ含めない。

### D：派生幾何の一意な有限性規則

- p0…p19、o=(0,0)。spacing[0]=|p0−o|、spacing[k]=|pk−p(k−1)|。
  index0だけ原点→先頭。各spacingは必要な2点が全成分有限の場合だけ定義する（原点は有限）。
- cumulative[k]はspacing[0…k]が全て定義済みの場合だけ、その順の左からの和。
  一度nullなら後続累積もnull。非有限区間後にゼロから再開しない。
- chord j=0…16はpj→p(j+3)、4点pj,p(j+1),p(j+2),p(j+3)の全成分有限が前提。
  中間点がNaNでも端点だけで橋渡ししない。1点でも非有限ならUNKNOWN_NONFINITE、length/direction=null。
  raw有限性を使いteacher maskは使わない。
- 4点有限なら長さL=|p(j+3)−pj|とatan2方向rad。L<=1e-12mなら
  UNKNOWN_DEGENERATE、length=L、direction=null。thresholdは既存のまま。
- start_s=grid[j], end_s=grid[j+3]、名目0.3mは実弦長/実弧長/時間ではない。
  原点不使用、j順・17窓。将来方向差はatan2(sin(delta),cos(delta))の最短方向差とする。
- 計算はraw float32を値不変でfloat64へ展開、float64の差・hypot・atan2・左結合累積。
  計算結果が非有限なら当該値はnull、弦はUNKNOWN_NUMERIC（両値null）、
  geometry.status=PARTIAL、reasonに該当index/演算を残す。NOT_COMPUTEDは未実施理由付きnull。
  数値的失敗も累積prefixを切り、非有限を有限値へclipしない。
- 後段で独立して有限なspacing/4点弦は記録できるが、原点から連続した走行可能経路を意味しない。
  geometryは計算版をderivation_versionへ記録し、20点の有限性をオンラインvalidityに変換しない。

## clock・history・未推論frame

時刻はns整数をJSON文字列で保持し、ROS_SIM/ROS_SYSTEM/MONOTONIC/DEVICEを明示。
同domainでもclock_id/epoch_idが違えば直接差分しない。ROS use_sim_time変更、巻戻り、jumpでepochを更新。
monotonic時計もprocess boot/host識別が必要。device/header/receipt時刻を同一と推測しない。
受付/入力確定/推論境界/snapshot/enqueue/receiptはmonotonic基準を用いる。t_obsは元観測基準のROS/device時計。
end_to_end_ageもclock_id/epoch_idが違えばUNKNOWN。負ageを0にclipしない。

camera4/lidar4/ego10/command10の各slotにsample ID、source、padding、実際のmask、ego feature mask、
取得/header/ROS受信/monotonic受信/利用可能時刻、input確定時ageを残す。paddingは元参照IDを残せるが
新しい実測sampleとして数えない。input未確定はstatus=NOT_BUILTで利用可能なslotだけ（最大長まで）。
欠けた時刻はMISSING/UNKNOWN。受信時刻で取得時刻を埋めず、hashで時刻対応が証明されたとしない。

command slotでは二つのcausalityCheckを独立に保持する：
command_source_pastはsource timestamp < output.t_obs、
command_available_before_inputはavailable_monotonic_time <= timing.input_finalized。
lhs_time/rhs_timeに実際の比較時刻、lhs_reference/rhs_referenceにsource field/record/slotの参照、
clock_mapping_evidenceに同clock/epochの根拠または変換証拠、statusにPROVEN/UNKNOWN/VIOLATIONを残す。
不一致が確認できたらVIOLATION、比較根拠なしはUNKNOWN。両方PROVENの時だけ要約causalityを
PROVEN_PAST_AND_AVAILABLEとする。一方VIOLATIONならVIOLATION、それ以外はUNKNOWN。
非command/paddingは各checkをNOT_APPLICABLEと理由にし、因果性証明の母数へ入れない。
sourceラベル/hashだけでPROVENにしない。出典nominal/final fallback、ID/hash/maskは別に保持。
現在のExternalControllerCommandには時刻がなくMISSING。既存controller呼出しや未来commandで埋めない。
command入力源実装の選択・clock対応実値は別承認のまま。

### B：実入力のtiming8とinput hash preimage

history.sensor_timing.values_sはforwardへ渡したsensor_dt_sec [1,4,2]のbatch0、
固定[4,2]・8 scalarを同じ確定input snapshotから記録する。header/receiptから再計算して代用しない。
BUILTは実値、同じinput_builder_id、input_contract_version/hash、components[4][2]を必須とする。
NOT_BUILTはvalues/components/builder/version=null、理由付き。未知contract hashはMISSINGのままでよい。
componentsの各slot/成分はcomponent_index、meaning、operation、lhs_time/rhs_time、
referenceとclock_mapping_evidenceを持つ。時刻object自体にclock_id/epoch_id/domainを含む。
LHS_MINUS_RHS_SECONDSなら秒換算差、CONSTANT_SECONDSなら定数の根拠と基準slotをreferenceへ、
UNKNOWNなら不明理由とtimeのUNKNOWNを残す。値が実測できても意味の根拠はUNKNOWNになり得る。
offlineのcamera_delta_ms/1000、lidar_delta_ms/1000とruntimeの[0,lidar−camera]を混同しない。
componentsは意味の記録であり、values_sの作り直しを要求するものではない。

payload_policyは旧NO_SENSOR_OR_INPUT_TENSOR_FULL_DUMPから
NO_FULL_INPUT_DUMP_EXCEPT_TIMING8へ改名。固定8 scalarのtiming metadataだけ明示的例外。
camera/LiDAR/全ego/全command tensor dumpは追加しない。hashだけでsensorを再現できるとはしない。

history.input_descriptorsは次のINPUT_FIELDS順の9件。各descriptorはfield、dtype、
batch次元を含むshape、byte_order=little、bytes_sha256。boolは1要素1byteの0/1、
float32はIEEE754のbit保持、C順の論理要素順に展開し、stride/padding/device pointerは含めない。
NaN payload/−0を正規化しない。数値castせず、承認input contract不適合ならBUILTを捏造しない。
本契約のbatch1ではfloat32のimage[1,4,3,224,384]、lidar[1,4,2,750]、ego[1,10,4]、
command_history[1,10,3]、sensor_dt_sec[1,4,2]、boolのimage_mask/lidar_mask[1,4]、
ego_feature_mask[1,10,4]、command_mask[1,10]。dtype/shapeの整合は将来意味検証とする。

input tensor_hashのpreimageは以下の固定ASCII文字列（UTF-8、BOMなし）。
先頭は `v4-input-snapshot-v2\n`、続いて各descriptorを順に
`field|dtype|dim0,dim1,...|little|bytes_sha256\n` として連結する。
ここで `\n` は2文字ではなく1byteのLFを意味する。
9件の順序はimage,image_mask,lidar,lidar_mask,ego,ego_feature_mask,command_history,command_mask,sensor_dt_sec。
dimは先行0なし10進整数、空白なし、hashはlowercase64桁、改行はLFのみ、最後にもLF。
history.tensor_hashはこのpreimageのSHA-256であり、生tensor連結やJSON serializeのhashではない。
input_builder_id/contract hashは別fieldで結合しpreimageには混ぜない。
BUILTでは9 descriptorとKNOWN tensor_hashが必要、NOT_BUILTではdescriptor=null/hashはMISSINGまたはUNKNOWN。
今回これらのhashを実計算せず、MISSINGな実値も埋めない。将来の同一snapshot・bit一致検証が必要。

warm-up/stale/duplicate/clock reset/例外でも受付candidate_sequenceを記録。既存の処理方針をこの記録仕様で変更せず、
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
`ELIGIBLE_FOR_SEPARATE_DIAGNOSTIC`。本design-v2にdrift数値の算出処理はなく、TFなし/reset跨ぎはUNKNOWN。

offline_join_sidecarは別ファイル/別経路。join keyはsession/reset/candidate_sequence/input_builder_id/output_idに
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
queue_capacity/depth、backpressure、write_failureとhealth証拠を記録する。
logger待ちでセンサ処理や他controllerをblockしない構成を別途具体化する。
drop_candidate_rangesは当該session開始からevent_observedまでに確認された候補損失の、
昇順・非重複・両端包含範囲を最大64件記録する。
全範囲を列挙できればCOMPLETE、上限超過等で一部ならPARTIAL、観測自体不十分ならUNKNOWNと理由。
[]かつUNKNOWNは欠損0ではない。母数はstage_counts.droppedの独立したstatus/valueで示す。
overflow時はdrop記録も同queueで失われ得る。別のbounded health経路/終了観測は将来設計・実装承認事項。
logger停止後にlogger自身が記録を残せるとは仮定しない。
監視/receipt断絶以降はINCOMPLETE/UNKNOWN。最終receiptの候補番号から連続保存を仮定しない。
記録の順序逆転・穴を許容し、receiptは対象ID/hashで後結合する。全損失の復元は保証しない。
sensor/input全dumpは禁止で、限定timing8以外はhashのみ。元sensorの独立再現は不可能な場合がある。

## 合成例と将来の検証表（記述のみ・未実行）

以下は実測でも実行可能testコードでもなく、将来実装者が作るfixtureの設計である。

| 合成例 | 想定record/期待結果 |
|---|---|
| 正常 | 同clock epoch、4/4/10/10、過去かつ利用可能command。1 forward IDからfloat32[1,20,2]取得。17弦と20spacing、SHAPE_FINITE_ONLY。制御安全を主張しない |
| warm-up | 最初の観測をpadding反復してfalse masks。未推論方針ならrecord.output.model_xy_m=null、record.output.status=NOT_INFERRED、record.status=WARM_UP。推論可否の既存方針を記録仕様で決め直さない |
| stale | candidate_sequenceあり、既存timing判定理由付きSTALE/NOT_INFERRED。推論durationはMISSING/未実施、レイテンシ母数に受付として残る |
| clock reset | ROS_SIM 10s→2s、monotonicは増加。reset_id/ROS epoch更新、history断絶、drift UNKNOWN。−8sを処理時間としない |
| TFなし | raw20点は保持。transforms=[]、drift UNKNOWN/TF_MISSING。現在ego原点へ付け替えない |
| 非有限 | 1要素NaNのbit列とtag保持、NONFINITE。該当弦/累積の不定をnull、ゼロ埋め/再推論なし |
| 方向不定 | p0=p3等のzero chord、length=0/direction=null/UNKNOWN_DEGENERATE。正しい直進角0とは区別 |
| 未構築DROP | candidate_sequence=7、受付時刻は既知、history空/NOT_BUILT、forward/output IDとshape/dtype/deviceはnull。drop母数に入り、builderを補作しない |
| forward例外 | forward IDあり/output IDなし、inference_startとinference_failed_at、FORWARD_EXCEPTION。戻り時刻を作らない |
| dtype/shape不正 | float64[1,20,2]またはfloat32[]を実metadataとしてOUTPUT_CONTRACT_ERRORに記録。[]はscalarでnullではない。cast/dumpなし |
| GPU待ちsnapshot | API戻りが先、GPU/copy完了後にsnapshot_ready。両境界を残し、API時間だけを全推論負荷と表示しない。copy失敗はSNAPSHOT_ERROR |
| 内部NaN窓 | p0,p3有限でもp1にNaNならj=0弦はUNKNOWN_NONFINITE、長さ/方向null。spacing後段は独立に定義可能でも累積は再開しない |
| logger停止/receipt | 先行record R42のbytes hashをwriter WのWRITER_RECEIPTが確認。R41保存は推測しない。以降監視断絶なら欠損数UNKNOWN、自己commitを書換えない |

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

| 修正群 | Schema構造で表現すること | 将来意味検証/実行証拠に残すこと |
|---|---|---|
| A | 候補番号nullable、非候補eventのID/出力不在、health/end母数必須、counter status/null、receipt必須参照 | ID一意性、母数重複排除、対象bytes hash、writer実書込み、clock比較、停止後の損失 |
| B | timing8の固定shape/BUILT条件、9 descriptor、2 causalityCheck | descriptor順/serialization/hash、同一input snapshot、実時刻・因果性判定/clock対応 |
| C | 6状態のID/null/failure_stage、metadata nullable、float32[1,20,2]/hex320 | metadata真実性、device/GPU境界、bit/numeric/NaN/−0/hashの一致 |
| D | 17弦、statusごとのlength/direction null、退化閾値 | 4点有限性、spacing/cumulative式と非有限伝播、float64演算、j順/方向差 |
| 維持 | sidecar分離、追加field拒否、承認flag固定 | teacher漏洩、行列正規性、実graph/無制御性、物理教師採用の別gate |

JSON Schemaは構造と一部条件を表現するだけ。hexとXYの一致、hash計算、slot index順、時計の比較可能性、
mask因果性、行列正規性、17弦の数式、field間status整合、無制御性は上記の将来意味検証が必要。
本タスクではschema parse/validationを含めテスト未実行。既存情報の不足を合格扱いにしない。

## 別承認が必要な段階

まず仕様/未知fieldをレビューし、runtime用input adapterの時刻意味・command入力源とparity方針を確定する。
次に専用の無制御node/logger/clock/graph証拠収集を実装する承認、合成testの承認を得る。
その後に対象入力・固定checkpoint照合・実行環境・容量/保存期間を限定した推論/記録/接続開始を別承認する。
controller要求計算、速度参照、publish、Safety/MPC連携、走行はさらに別gateで、今回の仕様から派生承認されない。
現runtimeを縦横MPC完成済みとせず、残差解析の再合格待ちへ戻さない。
