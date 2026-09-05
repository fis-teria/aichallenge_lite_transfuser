# 保存済みh30 futureの適格性・停止文脈監査

## Scopeと版

開始HEAD `cac81386b89ed508365adfa81b7c98338b3e1926`、Windows正本はclean、
origin `fis-teria/aichallenge_lite_transfuser`、branch `codex/windows-wsl-training-sync`。
指定保存版/実験A/判定名修正/前段開始版/前段実装版のobjectと祖先関係を再確認。
前段実行版 `2a2558749706e7554362d3d49613d92fbe3030f6` との差は前段報告書の追記のみ。
今回、既存V3/前段監査の意味を変更しない。新規module/reader/CLI/tests/documentのみ追加。
モデル・loss・runtime・controller・Safety・設定は未変更、実データ学習/推論/走行/push禁止。

## Q1：validの保証と非保証

`canonical_converter_v3._convert_observation` はimage header時刻を観測原点に使用。
`select_regular_grid` の `delta_ns = camera_source_stamp - grid_stamp` がcamera_delta_msとなり、
今回の観測時刻は `grid_stamp_ns + camera_delta_ms*1e6` で復元する。
Camera payloadそのものの再decodeとは区別する。

`_dense_future_state` のtargetは観測時刻＋0.1秒×1..30。
`_interpolate_pose_indexed` は両source endpointがtargetから各50ms以内、
endpoint間frame/child frame一致、XY線形・yaw最短wrap補間を要求する。
最大補間間隔は通常100ms以下。ただし50msはendpoint片側の許容値であり、
「最大gap50ms」とは異なる。速度は別 `/vehicle/status/velocity_status` の3成分線形補間。
XYはanchor yawで世界差分を回転、yawは相対角rad、float32 `[30,8]` 保存。
invalidはtimeを除く数値列NaN、mask=0。schemaのdocstringより実validate実装を優先する。

valid生成の条件は、clock epoch終端を越えず、poseとvelocity補間が成立すること。
schemaが有限値を検査する。validにはSafety・障害物・走行許可・route意図・
後輪中心・車体限界の検証は含まれない。補間内frame一致とanchorからfutureへのframe一致も別。
converterのstream indexはrun全体に作られ、時刻dedupを含むため、
validだけでclock reset前後のsource取り違えが完全に排除済みとは断定しない。
今回のreaderはbag記録時刻とmessage header時刻を両方保存する。

source再現は保存float32に対する暫定許容値：位置20µm、yaw10µrad、速度10µm/s、
保存相対時刻250ns。センサの実精度や車体実現可能性を保証する校正値ではない。
位置飛び20m/sの粗い検査に合格しても車両運動学はUNKNOWN。

## Q2/Q3：代表例と停止文脈

前段ledger全anchorのstable IDを追跡し、val stopped-commanded 530を落とさない。
追加調査は既定8 groups/32 anchors、test選択なし。
run coverageを優先し、h30到達停止/先頭欠損、短停止/censored/recovery増分/offset holdを選ぶ。
同点は時刻・ID順で決定。既存推定stop episodeと補足growth診断windowを別種として明記。
選択表には未収録の存在sliceとright-near欠落を保存する。
4 anchors/groupは代表抽出の負荷制限で、独立episode数の保証ではない。

nominal/finalを両方保存し、正commandは明示走行要求だけを意味する。
過去350ms以内のSafety/state記録をsource/hash/時刻/元値付きで保存する。
正command、後で動く、Safety理由未記録、replay preflight=Trueから許可を作らない。
offset-holdは横offset保持のphaseであり停止intentではない。
明示許可・意図停止・actuator故障の原因が不明ならUNKNOWNを残す。
first-future欠損は各targetのpose/velocity補間可否と照合し、
補間gap/観測window外/保存maskの未解決要因を区別する。欠損を橋渡ししない。

## source readerと予算

先にdry-runでrun/topic/window/予算を保存し、実行時は同一plan identityを要求。
通常MCAPはrosbagsのmetadata/index読取をreuseし、chunk展開は独自のbounded wrapper。
indexがなければ無制限meta scanにfallbackせずBLOCKED。
file-zstdはseek不可としてforward stream。最初のschema chunkと必要windowをdecode、
bag記録時刻の単調性を仮定してwindow後で停止する。run全体のreset不存在の証明ではない。
Camera/LiDARが同居するchunkは圧縮解凍byte予算に含めるが、sensor messageはdecode/出力しない。

既定の全実行合計：source読取256MiB、expanded1GiB、decoded50,000 messages、180秒、
一時disk0 bytes、単一record/chunk64MiB。CLIで明示変更可能、自動拡大しない。
expandedはfile-zstd出力とnested chunk展開の各段を計上し、二段処理は二段分数える。
圧縮stream内部の小さなread-aheadはsource読取量に含む。
上限停止はBUDGET_EXCEEDED/PARTIALとして取得済みevidenceを保持する。
bag-header margin既定250msを実recordで検査し、不一致は別時刻を推測して再探索しない。
raw窓の完全性が未確認ならsource再現の一部が一致してもtierは上げない。

## 同一anchor h15/h30比較とtier

A/Bは同じ保存futureの先頭15/30点。3秒超復元、他anchor接続、controller oracleではない。
rawは前段関数をそのまま使用して前段ledgerともanchorごとに比較。
厳格診断は両horizon共通の因果規則：mask/finite、時間gap、粗いteleport、reverse、
速度観測に基づく0.5秒holdで打切り、最後の採用位置から5mm蓄積して低速移動を保持する。
0.1m共通距離gridで同一prefixの誤差を比較。共通prefix一致は当然期待される性質で、モデル改善ではない。
旧ノイズ規則との差は別名strict_diagnosticとして保存し、前段結果を上書きしない。

OBSERVED_ONLY：保存幾何はあるがsource/time/frame/境界の追加根拠が不足。
GEOMETRY_VERIFIED：raw window完了、全保存valid点のsource再現、header-bag対応、
frame、局所pose/clock境界検査がPASS、厳格prefixに0.1m以上支持がある。
tierの対象は指定した暫定ノイズ/数値条件下の連続prefixであり、3秒全域の安全性ではない。
PATH_SUPERVISION_REVIEWED：今回の既定処理では自動付与しない。意図/環境/教師利用方針が別途必要。
clearance未知だけを理由に幾何存在を消さず、未知をfalse/safeへ変換もしない。
pathの短さ/hold/終端からnegative continuation、安全停止endpointを生成しない。

## Reference

前段のsource inventoryと、選択runのmetadata/Reference/interval/base SHAを再照合する。
phaseはその当時の近傍poseをrouteへ投影して作ったannotation。
source/hashによる関連は確認できても、同名run・mtime・形状の近さだけで
Reference生成時刻/frame/現在anchorの明示route意図が確定するわけではない。
planned Referenceはteacher/debug-only別source。今回futureとの平均化・置換・教師化はしない。
推論時route入力、後輪extrinsic、車体輪郭、ego接続、clearanceは別条件として未確認を維持。

## 実行方法

Windowsで今回の新規ファイルだけcommitし、`tools/sync_to_wsl.ps1 -CheckOnly` → 通常sync。
WSLでは同一commitとworktree lockで実行する。

```bash
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_spatial_evidence_v4.py
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
tools/with_wsl_training_lock.sh .venv/bin/python tools/audit_spatial_evidence_v4.py \
  --dataset-root /home/thistle/e2e_autonomous/datasets/d1log_recovery_mixed_20260904_v3 \
  --split-manifest /home/thistle/e2e_autonomous/datasets/d1log_recovery_mixed_20260904_split_manifest.json \
  --previous-audit /home/thistle/e2e_autonomous/runs/spatial_v4_coverage_full_v2_20260905 \
  --output /home/thistle/e2e_autonomous/runs/spatial_v4_evidence_dry_20260906
```

dry-run結果を確認後、同じ引数でoutputを新規 `spatial_v4_evidence_run_20260906` にし、
`--execute-raw --plan /home/thistle/e2e_autonomous/runs/spatial_v4_evidence_dry_20260906/raw_read_plan.json` を追加。
日付名は実行識別子、既存directoryは拒否する。Windows/WSL論理結果は時刻/pathを除き比較する。
source内部identityとファイルSHAは別々に検証・保存する。

## Q4：次段gate

geometry-only converter提案schemaをJSONで保存する（Dataset生成ではない）。
GEOMETRY_VERIFIED prefixがあれば、非実行用のversioned幾何変換へ限定的に進める。
停止教師設計は明示意図/許可/Safetyの不足、controller/MPC oracleは環境・車体・
入力契約不足で、それぞれ独立してBLOCKEDと判定する。
controller/bootstrap/command入力/stop headの変更は別タスクであり今回行わない。

## 実測結果

### 1. 実行版・差分

実raw監査・全pytest実行版は `7ae8298b71aa7bdbacf1d1757798bf0de66bcfc4`。
その後の本節追記は文書のみ。Windows named branchからcommitしてWSL detached HEADへ通常同期した。
新規ファイルは本書、`spatial_evidence_v4.py`、`spatial_source_reader_v4.py`、
`tools/audit_spatial_evidence_v4.py`、`tests/test_spatial_evidence_v4.py` の5つ。
既存V3、前段coverage、model/loss/runtime/controller/Safetyのファイル差分はない。pushしていない。

### 2. identity

指定されたDataset内部identity `181cf909…f388` をmanifestから検証。
manifestファイル `d625f42c…be1de`、split `7d0e433d…27b4f`、
前段manifest `458f6ebb…5ed13`、ledger `35781616…5e35` は指定SHAと一致。
省略なしの値は実行manifestに保存した。前段source inventoryから選択source/phaseの19ファイルを再hashし全件一致。
処理後もこれらのhashは不変。選択trajectoryは個別にcanonical manifest SHAと一致。
raw大容量ファイル全体を追加hash走査してはいない。metadata hash、選択message payload hash、
raw size/mtime前後一致を根拠として保存し、全raw byteの不変証明とは区別する。

最終出力root（2026-09-05 UTC実行、末尾20260906はdirectory識別子）：
`/home/thistle/e2e_autonomous/runs/spatial_v4_evidence_run_v3_20260906`

| ファイル/identity | SHA256 |
|---|---|
| execution_manifest.json | `4261af73079fdaafa7b2478b94447ab904fbd9bfcd61b7ed8529bd0c449d340b` |
| anchor_evidence.json | `98399b988b6c69fd7c47668f91d1388a62d46f3cb03d7a1e442ea8582db5c7df` |
| raw_window_evidence.json | `97d2ccd5992aacf7c20b894f30afc03de283ded61dd4e15cda3ea2b28d853d75` |
| read plan内部identity | `034d33a1f6545604af473196248bcf30bfe17e35b703b9b62ca7ca27637035b8` |

### 3. scope・budget・未検査

全72,697 IDを保存し、val stopped-commandedは530全件追跡。
代表29 anchors = stopped-commanded 17（normal 2 runs、5推定episodes）＋
recovery 12（3 runs、3補足diagnostic windows）。8 groupsは独立停止episode8件という意味ではない。
停止cohortの残り513、test13,866は追加raw調査なし。復帰12件も対象時間窓のpayloadを取得できずUNKNOWN。
存在する選択tagは全収録、right-near valは元集合にない。停止recoveryそのものの追加raw確認は未実施。

| 最終試行のrun | 読取source bytes | 展開bytes | decode messages | 状態 |
|---|---:|---:|---:|---|
| 20260902-131505 | 43,254,858 | 42,819,672 | 4,971 | COMPLETE |
| 20260902-132822 | 14,419,763 | 13,984,412 | 1,629 | COMPLETE |
| recovery_left_far_early_r03 | 202,248,743 | 533,350,210 | 0 | BUDGET_EXCEEDED |
| recovery_left_near_early_r01 | 18 | 0 | 0 | BUDGET_EXCEEDED |
| recovery_right_far_early_r04 | 18 | 0 | 0 | BUDGET_EXCEEDED |
| 合計 | 259,923,400 | 590,154,294 | 6,600 | PARTIAL |

最終予算はsource260,046,848 bytes、expanded1,065,353,216 bytes、49,980 messages、170秒、temp0、
record/chunk/zstd window64MiB。raw各reader時間合計約1.425秒、manifestのraw開始後処理含む時間2.764秒。
metadata/manifest/canonical検査時間はraw予算時間と別。sensor decode/保存と一時展開ファイルは0。
通常MCAPはindexで窓を読み、復帰はfile-zstd前方stream（実window要求2MiB）。
left-farの対象時刻へ到達する前にsource byte予算停止。near/farの順を変えたり予算を増やして再探索しない。

初回/e8b77f1はIDL未対応とzstd例外未捕捉、2回目/25b8952はprogressのimmutable書込衝突で中断。
失敗directoryも残し上書きしていない。各normal読取は合計5,660,300 bytes、expanded3,919,226 bytes、
8 decoded messagesとログで確認。初回zstd失敗直前のcounterは保存されず、その読取量の厳密実測は欠ける。
再試行では当初上限から計8MiB source/expanded、20 messages、10秒を控除した。
全試行の厳密なbyte合算が完全保存できたとは主張しない。最終readerは各fileのimmutable progressを保存する。
IDLとzstdの修正はreader互換性修正であり、geometryの採用条件・暫定閾値は緩めていない。

最終dry-run/executeで上の実行例へ共通追加した引数：

```text
--max-source-bytes 260046848 --max-expanded-bytes 1065353216 --max-messages 49980 --max-seconds 170
```

dry outputは `spatial_v4_evidence_dry_v3_20260906`、execute outputは `spatial_v4_evidence_run_v3_20260906`。
executeのplanはdry側 `raw_read_plan.json`。選択/windows/topic/config/source identity一致を検査後に読んだ。

### 4. h30実source照合

normal17 anchorsの保存valid493点がすべて再現PASS。位置残差最大1.210e-7m、yaw2.768e-8rad、
速度9.544e-8m/s。保存相対timeの最大誤差95.37ns。照合対象pose補間gap最大75.0ms、
header対bag差最大35.0ms。frame `map`、child `base_link` を観測した。Camera header自体はdecodeせずmetadataから復元。
一定速度の相対pathは時刻の一様shiftでも一致し得るため、数値再現単独を時刻対応の独立証明にしない。

ただし17件とも局所windowの同一pose stamp・異なるXYを検出しboundary FAIL。
この重複判定は差>1e-8mの保守的・未校正ルール。微小差も含み、全17件が車両に危険な跳躍という意味ではない。
normal131505の抽出窓全体では差1.063e-8〜0.131172mの重複111組（同一stampの先頭対後続比較）。
最大例はstamp/bag共に9,429,999,789ns、XY=(89630.3050756879,43132.55743654385)と
(89630.28039531305,43132.686265950106)。payload hashは
`53534dc800b49b841fd5ff27bfc7459bf222b4b448b3af3bd4933dc2e223fd0f` と
`2dd19cca28a31a03b0ee1d099c7b2037df53c347002d843e14a170ca7d8ac3ef`。
最大例は抽出窓全体の値で、全anchorのwindow内最大と混同しない。
converterのlast-record dedupを再現できても、一意の物理poseの確定とはならない。
局所clock逆行/gap/teleport/frame変更flagは検出なし。run全体のreset、route変更、車体実現可能性はUNKNOWN。

先頭future欠損2例はpose補間の片側50ms条件未達をsourceで確認：
`20260902-131505__epoch0000__6192918933` は最大endpoint差54.218524ms、
`20260902-131505__epoch0000__1492918933` は50.736653ms。velocity補間は成立。
後続validがあっても原点へ橋渡ししない。これは一律「停止だから欠損」ではない。

### 5. 停止文脈

調査停止17/530には、3秒後のmotionがある例と、観測支持が短い例が両方ある。
例 `20260902-131505__epoch0000__5292918933` は正final command約9.72m/s、raw h30弧長2.24m。
一方、観測速度による初期0.5秒holdでstrict prefixは0m。後から動いた事実はanchor時点の発進許可ではない。
`…__253892918933` はfinal2.520701m/s（stamp253864994325ns）、h15約0.58m→h30約2.07m。
GearReport report=2のtimestamped記録はあるが、完全なpreflight、停止意図、Safety原因、faultは確定しない。
normalのmetadataにはSafety理由topicなし、recoveryにはtopicがあるが今回window payload未取得。
全29件でSafety/走行許可/待機意図/actuator原因はUNKNOWN。offset-hold4件は横offset保持であり停止意図でない。
全530に対する停止理由率や、前段450/43/37の分類を今回確定した割合として出さない。

### 6. 同一anchor比較

以下は今回選択29件のみ。raw prefix既知27、先頭欠損UNKNOWN2。
strict diagnosticは保存値への共通規則適用であってsource検証済みcoverageではない。

| 距離 | raw h15 / h30（既知27） | strict diagnostic h15 / h30（29、欠損2は支持0） |
|---|---|---|
| 0.5m | 13 / 14 | 13 / 13 |
| 1.0m | 0 / 14 | 0 / 13 |
| 1.5m | 0 / 2 | 0 / 1 |
| 2.0m | 0 / 2 | 0 / 1 |

h30のraw1m到達14はrecovery12＋停止2。厳格診断で失われた停止1件は上記初期holdの例。
recovery各4件のh15→h30弧長はleft-far約0.67〜0.68→1.31〜1.32m、
left-near/right-far約0.56→1.12〜1.13m。ただし12件ともsource未確認なので適格性はUNKNOWN。
停止17件のraw prefix既知15、raw1mはh15=0/h30=2、strict診断h30=1。
source-verified tierに至った分母0、29件すべてstrict verified coverageはUNKNOWNであり0/29の失敗率ではない。
全29件で前段raw弧長と一致し、共通0.1m grid prefix残差0。これは同一sourceの期待結果でモデル精度改善ではない。
各anchorの終端XY/弧長/時間/cut理由/IDとrun/episode/case別母数はsidecarとcomparison_summaryに保存。

### 7. Reference・runtime意図

選択recoveryのReference/interval/baseとphase sourceのhash関連は確認した。
Reference CSVのs/x/y/psi/kappa/vx/axは時刻付き実pose列ではない。frame/生成時刻/座標基準時刻の明示根拠、
anchorのroute ID/進行index対応、ego接続、後輪extrinsic、車体輪郭、clearanceは未確定。
phaseの近傍pose投影をroute intentそのものとしない。preflightの計画情報やmtimeを実行許可に使わない。
Referenceはteacher/debug-onlyの別sourceであり、推論時planned route intent配線の存在を示さない。
現行選択runtimeは縦横MPCではないという前段コード確認を引き継ぐ。今回controller実行・変更はしていない。

### 8. sidecarとテスト

execution_manifest、source_inventory_delta、selection/selected_anchors、all_anchor_status、anchor_evidence、
raw_read_plan/report/progress、raw_window_evidence、comparison_summary、geometry_converter_schema_proposal、
report_jaを新規directoryへ保存。学習Datasetではなく、stable IDとhashでjoinする根拠sidecar。
OBSERVED_ONLY29、GEOMETRY_VERIFIED0、PATH_SUPERVISION_REVIEWED0。negative continuation/safe endpointは生成なし。

実行版7ae8298のfocused **34 passed in 5.03s**、全回帰 **680 passed, 40 warnings in 53.75s**。
40 warningsはPyTorch Transformer nested tensor/norm_first警告。既存合成学習unit testは許可範囲として実行、
実データ学習・checkpoint利用・モデル推論・ROS/AWSIM実行はしていない。
source不変/dry-run決定性/partial manifest/MSG・IDL MCAP/zstd窓byte上限/各幾何・文脈条件を合成検証。
Windows Pythonはpytest/rosbags未導入のためWindows full reader suiteは未実行。
WindowsとWSLで同一float32 30点0.5m/s fixtureのcompare_horizons論理JSON SHAを比較し一致：
`8ca99772f0801a43c8da04deea96cfad0c7295e67cabe06b693d4cf6f92d68c0`。
この1 fixtureの一致を全実データreaderのクロスOS検証完了とは呼ばない。

### 9. 個別gate

- geometry-only converter：**BLOCKED_NO_SOURCE_VERIFIED_PREFIX**。schema設計は保存済みだが採用可能prefix未確定。
- 停止/発進教師設計：**BLOCKED_INTENT_PERMISSION_INCOMPLETE**。正commandと未来motionでは不足。
- controller/MPC oracle：**BLOCKED_ENVIRONMENT_VEHICLE_POLICY**。環境、車体、意図、入力契約の根拠不足。

今回の限定scope監査は、raw PARTIALとunknownを保存して完了。V4正式教師採用・M3・発進成功・安全性合格ではない。

### 10. 次の最小1タスク

**今回保存した小さなraw_window_evidenceだけで、同一pose stampの競合とdedup方針を監査する。**
微小差と実質的な差、同一bag時刻の順序の根拠、source publisher識別可能性を分け、
数値再現PASSを保ったまま一意性/時刻の根拠を定める。閾値緩和だけで合格を作らない。
新規raw展開、追加予算、学習、走行を伴わないこの1タスクを優先する。
