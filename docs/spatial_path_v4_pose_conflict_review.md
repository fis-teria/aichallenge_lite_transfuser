# 保存済み抽出物によるpose競合・prefix影響監査

## 目的・scope

旧監査のFAILとその適用範囲を、新しいsidecarで分類する。旧tierやreaderは変更しない。
開始Windows HEAD `ec255bce2d2d838698a6c964f880ba2d09a28303` はclean。
origin/branchは指定どおり。固定版428f30a、実行版7ae8298、前段2a25587はいずれもobject存在・HEAD祖先。
7ae8298から開始HEADの差はレポート追記とAstra依頼文だけで、コード差分なし。

今回の新規ファイルは `spatial_pose_conflict_v4.py`、CLI、同名test、本書のみ。
モデル/学習moduleや既存raw readerをimportしない。標準ライブラリだけで処理する。
Dataset、bag、sensor、checkpoint、JSON内source path/commandの追跡や実行はしない。
全pytestはoptimizerを実行する合成テストも含むため**実行しない**。過去680 passedは今回の結果ではない。

## 入力とidentity

allowlistは必須 `execution_manifest.json`、`raw_window_evidence.json`、`anchor_evidence.json`、
`raw_read_report.json` と、join用任意 `selection.json` の5ファイルのみ。
JSON内のsource pathは文字列であって追加読取先ではない。symlink leaf、既存出力、入出力包含を拒否。
指定された3つのSHAは固定期待値と照合し、異なればBLOCKED。report/selectionの新hashは
独立した過去identityの証明ではない。旧実行commit、Dataset identity（Datasetを読まずmanifest報告値を照合）、
設定、anchor/record件数、selectionのID集合を確認。入力SHAは終了時に再照合する。

既定解析上限：1ファイル16MiB、合計32MiB、10,000 records、64 anchors、30 steps/anchor、
4,096 pairs/group、全100,000 pairs、解析60秒。JSON読取前にbyte上限、解析前に件数を確認。
record上限では全recordをinventoryで追跡し、群解析はまとめて延期。anchor上限では残りIDをNOT_INSPECTEDで残す。
pair上限では測定済み最大を下界として保存し、全群最大はnull。値のNaN/Infinityはinvalidとして残し0へ補完しない。

## 再確認した旧実装の6点

1. mcap_converter_v2の `_deduplicate_sorted` とevidenceの `interpolate_records` は渡された列の同stamp最後を選ぶ。
   前者のAnyReader列と後者のchunk/parse列の順序同一性は、この抽出JSONから証明できない。
2. `observed_boundaries` はbag時刻sortの隣接XY差>1e-8m。yaw・非隣接群・invalid候補は十分に検査していない。
3. `evidence_for_anchor` はh30全体、run全抽出のbag/header差、run completeと最小0.1mをまとめてtier化。
4. `compare_horizons` は `strict_polyline` のboundary_eventsへ実イベントを渡していない。
5. 先頭欠損はraw null、strict arc0/reachesFalseという意味の差がある。
6. file-zstd早期停止はlog time単調性仮定であり、COMPLETEは独立した完全性証明ではない。

今回はこれらを既存moduleへ修正しない。legacy predicateの再現を新しい現象分類と分離する。

## 新sidecarの判定契約

record ID = input SHA＋run ID＋元JSON配列indexの論理hash。物理順序・publisher sequenceではない。
pose/velocityは同stampを全候補で集める。明示source_domain/clock_domain/clock_epochがそろう場合だけdomainを分ける。
不明ならcandidate bucket。frame/typeで群を分けて競合を隠さない。既存source_idはpublisher証拠ではない。

全pair最大、先頭対後続、群内配列隣接、旧bag-sort隣接predicateは別名で記録する。
yawはradの最短角差（math.remainder、±piを同値とする）を使い、未校正の許容角を導入しない。
同payloadでもdomain不明なら物理同一を主張しない。投影値の一致と全Odometry一致も別。
非ゼロ差はNONZERO_DIFFERENCE_UNCALIBRATED。material budgetは既定なし。
20µm再現許容値との比較は参考診断だけで、world pose noiseの閾値へ流用しない。
float64 hex/ULPは診断用、world座標をfloat32に丸めない。元future全値がないので相対float32量子化は再計算しない。

endpoint stamp/hashから元再現が依存した候補IDをjoinし、複数一致は曖昧さを保持。
anchor姿勢の依存は全futureへ、右側endpointはその前のtargetへも伝播する。
同stamp変数の全組合せを仮想選択した新XYは作らず、依存関係の保守的な影響だけを示す。
全targets、保存valid、既存strict prefix、h15/h30、windowは独立scope。
domain不明ならscopeの影響証明はUNKNOWNでも、「保存候補に数値差あり」は別booleanで記録する。
最初の不明/不適格依存点の前までをsource-consistent prefix候補とするが、新実長・XY・残差は生成しない。

旧支持値は報告値として保持し、先頭欠損はsupport/reaches=null、保持stepがありarc0なら既知0。
原点のみgridはNOT_COMPARABLE、正のgrid点数を保存。旧0残差を新しい比較成功としない。
0.5秒holdは縦速度だけの旧診断policyでありlateral/yaw静止、意図停止、安全endpointではない。
短い幾何の既知性と最小0.1mのpath長gateは分離。新しいtier昇格・教師採用は行わない。

## 実行・exit code

Windows commit → `tools/sync_to_wsl.ps1 -CheckOnly` → 通常sync。双方のGit状態と実行中processを先に確認する。
WSLでは以下をshared lock経由で実行する。

```bash
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_spatial_pose_conflict_v4.py
bash tools/with_wsl_training_lock.sh .venv/bin/python tools/audit_spatial_pose_conflicts_v4.py \
  --evidence-root /home/thistle/e2e_autonomous/runs/spatial_v4_evidence_run_v3_20260906 \
  --output /home/thistle/e2e_autonomous/runs/spatial_v4_pose_conflicts_20260905
```

0 = 宣言scope処理完了（UNKNOWN残存は許容）、2 = PARTIAL（解析上限/invalid record等）、
3 = BLOCKED（identity不一致/必須欠損/JSON構造不正/出力衝突等）。引数syntaxエラーはargparseの2。
出力衝突では書込なし。その他入力BLOCKEDは作成可能な新規outputへmanifestを保存する。

成果物：input_inventory、pose_stamp_groups（velocityも含む）、anchor_prefix_impact、summary、execution_manifest、report_ja。
旧実行commitと新再分類commitを別記し、code/input/config/policyと解析結果の論理identityを保存する。
logical identityは処理時間・作成時刻・環境固有pathを除く。UNKNOWNを含む一致は同一性の確認で、安全性ではない。

## 実測結果

### 1. 版・入力identity・完了scope

実行・テスト版は `befc97dd98434843245b888fb6d1d7110acc8a93`。
本結果の追記commitは文書のみ。Windows/WSLで同じ実装commitを使用し、pushしていない。
旧実行版7ae8298、報告版428f30a、依頼文追加版ec255bcとは分離した。

| allowlist入力 | bytes | SHA256 |
|---|---:|---|
| execution_manifest.json | 4,716 | `4261af73079fdaafa7b2478b94447ab904fbd9bfcd61b7ed8529bd0c449d340b` |
| raw_window_evidence.json | 3,453,639 | `97d2ccd5992aacf7c20b894f30afc03de283ded61dd4e15cda3ea2b28d853d75` |
| anchor_evidence.json | 1,193,073 | `98399b988b6c69fd7c47668f91d1388a62d46f3cb03d7a1e442ea8582db5c7df` |
| raw_read_report.json | 8,004 | `e3da319b45ba62dbbfdea16dc2efa051c05c96010b5ba50d98f8e9a3e732f401` |
| selection.json | 12,103 | `d2518ac717712ff2c8860b053114f6a478a2a1c8c757884dcee8d58ac3560295` |

最初の3つは指定期待値と一致。後2つは今回hash記録であり、独立した過去実行へのhash結合ではない。
旧manifestのcode、Dataset内部identity、29選択/8 groups/530追跡/72,697全件の報告値、暫定設定を確認した。
Dataset本体、旧ledger、raw pathへはアクセスしていない。
全6,600 JSON records、29 anchorsを処理し、invalid record/anchor、上限延期とも0。
最終状態は `COMPLETE_DECLARED_SCOPE` / exit0。元raw抽出がPARTIALだった事実はそのまま保持。

### 2. legacyの再現

旧h30のstatus/flagsは**29/29で一致**。normal17件のlegacy FAILとrecovery12件のUNKNOWNを再現。
これは旧述語の一致であり、旧FAILを危険判定として追認したり、誤りと確定したりするものではない。
source再現PASS17/UNKNOWN12は旧版の報告として保持し、今回の独立した数値再現PASSは生成していない。
元future全XYを読んでいないので、新canonical残差・新実長・新XY教師は未計算。

### 3. 同stamp群の分類

保存記録はpose857、velocity535、clock3,747、command919、gear535、AWSIM state7。
pose685 candidate bucketsとvelocity535 buckets、計1,220群。pose重複172群は**すべて2候補**。
velocityはこの抽出範囲で同stamp重複0。復帰3 runsはpayload0なので、重複数をnullにして未観測とした。

| scope | 重複群 | XY差>1e-8mのpair | 備考 |
|---|---:|---:|---|
| normal 20260902-131505 | 139 | 111 | 旧「111組」はこのrunの閾値超過pair数 |
| normal 20260902-132822 | 33 | 16 | こちらは旧111組の母数外 |
| 合計 | 172 | 127 | 全群2候補なので先頭対後続と全pairが一致 |

非ゼロ投影差171群、投影XY/yaw一致1群。全172群でpayload hashは候補間で異なる。
XY非ゼロ171群のうち44群は1e-8m以下で、旧predicateには引っ掛からない。
yaw差は170群。最大XY差0.1311721647999626m、最大最短yaw差0.015095404171680205rad。
参考として20µmを超えるXY差は73群。ただし20µmは保存再現の許容値であり、physical noiseや競合採用の閾値ではない。
既定material budgetは未設定なのでMATERIAL_DIFFERENCE_EVIDENCEDを付けた群は0。
171群すべてNONZERO_DIFFERENCE_UNCALIBRATEDとして残した。
frame/type競合、非有限候補は今回抽出範囲では未検出。微小差の発生原因は確定しない。
全172群のsource/clock epochと物理順序は不明で、ORDER_OR_EPOCH_AMBIGUOUSを別軸に持つ。

### 4. anchor・h15/h30・prefixへの依存影響

依存は保存されたtarget/source stamp/payload hashを使い、候補IDへjoinした。
ここで「差に依存」は保存stamp/hashを対応付けたときの数値差候補への依存であり、
物理domain同一性の証明や、安全性FAILを意味しない。

| scope | 保存valid targets | 数値差候補に依存するvalid targets | 既存strict保持targets | 数値差候補に依存する保持targets |
|---|---:|---:|---:|---:|
| h15 | 423 | 206 | 251 | 63 |
| h30 | 853 | 409 | 446 | 70 |

normal17 anchorsすべてのvalid target集合に数値差候補への依存を確認。
anchor姿勢のendpoint自体に差があるのは10件で、その影響は全futureへ伝播する。
既存strict prefixに数値差候補依存があるのは15 anchors。残りnormal2件は先頭欠損でprefix未成立。
recovery12件はendpoint payloadがなく、差の有無もnull。正常な無競合例に数えていない。
retained_stepsとelapsed time・保存validの整合は29件の両horizonで確認できた。

各anchorにはwindow全体のlegacy再計算、h15/h30の依存、strict prefixの独自時間窓・境界根拠・抽出完全性を別々に保存した。
現在の29件ではsource/epoch aliasが未解決のため、全horizonの独立依存適格性はUNKNOWN。
「競合なしであることを証明したsource-consistent prefix候補」は全件step0で、支持長はnull。
これは実幾何の長さ0という意味ではなく、証明できる範囲が未確定という意味。
h30後半だけの競合を無関係h15に伝播しない条件、右endpoint→直前target、anchor→全futureは合成テストで確認した。
実例全17件のh15にも依存差があるため、今回実例からh15救済の成功件数は主張しない。

### 5. 確定事項・仮説・未記録

確定できたのは「抽出JSONの同stamp候補に数値差があること」「旧predicateと元再現の依存関係」。
最後の配列候補を選ぶと元endpoint hashに一致するかを診断しているが、AnyReaderと今回readerの列生成順序同一性は未証明。
更新された推定値、丸め、再送、別publisher、epoch混在などの原因は仮説にとどまる。
payload hash差をcovariance差やpublisher差と断定する情報もない。
原quaternion、covariance、channel/schema、publish_time、sequence、offset、publisher、明示epochは未記録。
COMPLETEなindexed抽出でもこれらの完全性・対応関係を抽出物だけでは証明できずUNKNOWNを維持した。

### 6. UNKNOWN・既知0・比較母数

旧strict policyで報告された支持は、両horizonとも既知prefix13件、既知0が14件、先頭欠損UNKNOWN2件。
既知0は「旧縦速度hold/noise policyで採用されたpolylineの弧長が0」という報告であり、物理standstillの証明ではない。
recoveryの既知prefix12件は旧保存geometryの報告値として保持し、source適格性は別のUNKNOWN。
全29件中16件は共通gridが原点のみなのでNOT_COMPARABLE。残差0を新しい比較成功と扱わない。
残り13件も正のgrid点数を報告値から数えただけで、新しいXY照合はしていない。
旧FAIL、新たな投影差、domain UNKNOWN、NOT_INSPECTEDを単一のunknown_reasonsへ押し込んでいない。

### 7. 独立gate

- 非実行用geometry-only変換の採用：BLOCKED。保存済み支持の存在は確認できるが、独立したdomain/order/completenessが不足。
- 停止/発進教師：BLOCKED。明示意図・許可・Safety根拠不足。
- controller/MPC oracle：BLOCKED。環境・車体・入力policyの根拠不足。現行選択runtimeは縦横MPC実装済みではない。

短さだけで幾何そのものを消さず、0.1m gateは独立に表示。今回の再分類で旧tierを書換えたり、新採用を行ったりしていない。

### 8. テスト・実行結果・成果物

実行版befc97dで**52 passed in 0.78s**。内訳は新規JSON監査42件＋既存synchronization非学習回帰10件。
MCAP fixtureテスト、Dataset操作テスト、モデル/optimizer/学習テスト、full pytestは未実行。
新moduleのsubprocess import試験でもtorch/rosbags/training moduleがロードされないことを確認した。
CLIのBLOCKED exit3、PARTIAL exit2、immutable出力、原本前後hash一致、NaN/欠落/上限・決定性も試験した。

最終WSL出力：`/home/thistle/e2e_autonomous/runs/spatial_v4_pose_conflicts_final_20260905`
上のCLI例のoutputだけをこの新規directoryへ変更して実行した。解析約0.383秒、pair計算172/100,000、上限到達なし。
初回/改良途中の新規出力directoryも残し、いずれも上書きなし。
Windowsでも同一commit/入力/設定で標準PythonによるCLIを実行し、全論理結果が一致：
`ccd1f4e830d4726fcb8fbef45ac83bc60a9f99ec9ca4233dce6f4590f6f4ba44`。
Windows確認出力は `E:\workspace\e2e_lite_transfuser\tmp\spatial_v4_pose_conflicts_windows_final_20260905`（Git対象外）。

| WSL成果物 | SHA256 |
|---|---|
| execution_manifest.json | `2a305ef80f4a9ae979644ff77d75fc8010b7b4e01db26ac4bfcd02b680255e3f` |
| pose_stamp_groups.json | `c5734787fbd8a7bb14038c601398ac5ed098514e1cf9349d55a94cf26aa63f92` |
| anchor_prefix_impact.json | `12a3115b46fe4de310b6bc8cfc1de9e64a358890cde607592917890415c11e00` |
| summary.json | `da224114867d9d827212be5d179329564b759ba9dd3a8ba74d92c28427e1ef22` |

allowlist入力の全5hashはWindows/WSL各実行の前後とも不変。
新raw読取・reader実行・Dataset読取/生成・学習・推論・checkpoint・oracle・ROS/走行・制御変更・pushは0。

### 9. 次に必要な最小field（追加取得は未実行・別承認）

1. semantic stampが属する `clock_domain` と `clock_epoch`、その割当根拠となるreset/eventの参照。
2. source publisher/domain IDとchannel IDの対応。recording source_idだけではpublisher識別にならない。
3. channel→schema ID/definition hashの対応。型名一致だけでは同一定義を保証しない。
4. 同時刻候補の順序を検証するphysical record/chunk offset、sequence、publish_timeと各fieldの意味。
   欠けるfieldをhash辞書順で代替しない。
5. 必要な候補に限定した元quaternion・covariance等。投影値一致/微小差の発生箇所を調べる場合に限る。

これらは保存抽出物からは復元できない。原因確定やtier昇格には、必要fieldを限定した追加証拠の取得方針を
別タスクで承認する必要がある。今回、その新規raw取得を実行していない。
