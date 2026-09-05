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

実行後、code/output identity、限定調査件数、budget実績、source再現、テスト結果を追記する。
