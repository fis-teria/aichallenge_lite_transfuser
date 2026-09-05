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

実行後に、群の分布、旧111組との数え方の比較、依存影響、テスト結果と不足fieldを追記する。
