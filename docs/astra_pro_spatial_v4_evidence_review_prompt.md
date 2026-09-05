# Astra Pro：Spatial Path V4の教師適格性・pose重複監査レビュー

あなたは、自動運転の学習ベース経路計画、ROS bag時系列、教師データ設計、MPC連携に詳しい独立レビュー担当です。
以下のGitHubコードと監査レポートを読み、監査実装自体の妥当性をレビューし、次にローカルCodexへ渡す最小の実装依頼を作成してください。
レポートの結論を追認するのではなく、過剰なFAIL・根拠不足のPASS・UNKNOWNの誤解・実装不備を同じ厳しさで検査してください。

## 1. 対象版と資料へのアクセス

- Repository: https://github.com/fis-teria/aichallenge_lite_transfuser
- Branch: `codex/windows-wsl-training-sync`
- 監査レポート固定版: `428f30aca0418bc6362cbe8eb78b9aa6a07e1e4f`
- raw監査・全pytest実行版: `7ae8298b71aa7bdbacf1d1757798bf0de66bcfc4`
- 前段coverage実装版: `2a2558749706e7554362d3d49613d92fbe3030f6`
- 本プロンプトの追加commitと、監査の実行commitを混同しないでください。

最初に開くレポート：
https://github.com/fis-teria/aichallenge_lite_transfuser/blob/428f30aca0418bc6362cbe8eb78b9aa6a07e1e4f/docs/spatial_path_v4_teacher_eligibility.md

固定版のコード一覧：
https://github.com/fis-teria/aichallenge_lite_transfuser/tree/7ae8298b71aa7bdbacf1d1757798bf0de66bcfc4

優先して読むファイル：

1. `AGENTS.md`、`docs/spatial_path_v4_teacher_eligibility.md`
2. `src/aic_transfuser_lite/data/spatial_evidence_v4.py`
3. `src/aic_transfuser_lite/data/spatial_source_reader_v4.py`
4. `tools/audit_spatial_evidence_v4.py`、`tests/test_spatial_evidence_v4.py`
5. 上記から参照されるcanonical converter、`mcap_converter_v2.py`、schema、前段`spatial_coverage_v4.py`、DatasetViewV3の必要部分

実際に読めたcommit・ファイル・範囲を最初に明記してください。Webで読めなかった場合は、未読を明示して必要なファイルの添付を求めてください。別版を黙って代用したり、存在しないアクセス権を仮定しないでください。

## 2. 目的と制約

長期的な目標は、モデルが適切な経路を生成し、縦横制御をMPC等へ分離することです。
ただし現行選択runtimeは縦横MPCではなく、delay-aware waypoint tracker＋LongitudinalControllerV3です。MPC実装済みとして議論しないでください。
今回はV4正式設計を確定する前の、教師適格性と監査規則のレビューです。

- 観測geometry、数値的source再現、時刻/frame/境界検証、教師利用の適格性、走行許可、安全性を独立に扱う。
- UNKNOWN/NOT_INSPECTEDをfalse、失敗率、安全確認、negative continuationへ変換しない。
- 学習、推論、checkpoint利用、Dataset生成、制御変更、oracle replay、走行、raw予算拡大は今回の依頼ではない。
- h30やMPCだけで発進・完走・安全性が改善すると断定しない。
- ユーザーの旧称Special Path V4はSpatial Path V4と同じものを指す。

## 3. Dataset概要と今回の報告値

以下はローカル監査の報告値であり、あなた自身がrawを再計測した値ではありません。

- canonical: 26 runs、72,697 anchors。train45,190 / val13,641 / test13,866。splitはrun単位。
- 保存futureは30点・0.1秒間隔・3秒、現行loaderは先頭15点・1.5秒。
- `[H,8] = time_sec,x_m,y_m,yaw_rad,longitudinal_mps,lateral_mps,yaw_rate_rps,valid`。
- 座標は`base_link@t_obs`、後輪中心との一致は未確認。無効点の状態値はNaN、timeは保持。
- 今回は5 runs・8 groups・29 anchorsを選択。停止17件/5推定episodes＋復帰12件/3補足windows。
- val stopped-commanded530件全件のIDを追跡。残り513件は追加raw未調査。right-near val例なし。test幾何は未検査。
- 通常2 runsはraw読取完了。復帰3 runsはsource byte上限で対象payload未取得。成功例だけの全件確認ではない。
- 通常17 anchorsのvalid493点はsource再現PASS。最大位置残差約1.21e-7m。
- 一方、17件とも同一pose stampに異なるXYを検出し、boundary FAIL。
- 重複判定閾値は差>1e-8mで未校正。normal131505の抽出窓全体では差約1e-8〜0.131172mの111組。最大値を全17件の局所差と混同しない。
- 先頭future欠損2例はpose補間endpoint片側50msの条件を超過。後続validへ橋渡ししていない。
- 選択集合のraw prefix既知27、先頭欠損UNKNOWN2。1m到達h15=0/h30=14、共通strict診断では0/13。
- 1件の差は初期0.5秒の観測holdによる打切り。横offset-hold annotationとは別。
- GEOMETRY_VERIFIEDは0、29件はOBSERVED_ONLY。全29件で許可・Safety・停止意図はUNKNOWN。
- 全pytestは実行版で680 passed / 40 warnings。テスト合格は実データ教師適格性を保証しない。

raw最終試行はsource259,923,400 bytes、expanded590,154,294 bytes、6,600 decoded messages、temp0。
初期reader失敗試行では一部counterが保存されず、全試行の厳密合算に限界があります。最終試行の予算は以前の読取分を控除しており、自動拡大していません。この説明と実装の整合性も検査してください。

## 4. 実データはGitHubにありません

大容量Dataset・bag・重み・今回のevidence JSONはGitへ追加していません。
ローカル成果物rootは `/home/thistle/e2e_autonomous/runs/spatial_v4_evidence_run_v3_20260906` ですが、Webから読めるURLではありません。
レポートにはsource identity、出力hash、具体例があります。必要なら次の小さな成果物のうち必要なものだけ追加添付として求めてください：

- `execution_manifest.json`、`selection.json`、`raw_read_report.json`
- `anchor_evidence.json`、`comparison_summary.json`
- `raw_window_evidence.json`（今回取得した非sensor記録。未添付なら読んだことにしない）

rawを読めなくてもコードレビューと検証設計は進め、実データの原因確定とは分けてください。

## 5. 重点レビュー項目

### A. 重複poseとdedupの正当性

- canonical converterと今回readerは、同一stampの順序とlast-record選択について本当に同じ規則か。
- 同一bag time内の順序、chunk順、channel/source identity、header time、clock epochが保存・検証されているか。
- 再送、丸め誤差、同一stampの別推定値、別publisher、reset等を現在の根拠だけで区別できるか。原因は推測と明記する。
- 異なるXYだけでなくyaw/frameの競合、非隣接重複、入力順序依存、非有限値の抜けを確認する。
- 1e-8mによる一律FAILは過剰か。保存再現誤差の許容値と、実poseの一意性・noise校正の閾値は別物として設計する。
- 微小差と実質的な競合を分けるにあたり、数値精度・元source・順序証拠に基づく方針を提案する。合格件数を増やすためだけに閾値を緩めない。

### B. tierと境界のscope

- anchor前後のwindow全体FAILと、実際に採用する連続prefixのFAILを分離すべきか。
- h15/h30で共通の因果的規則を守り、h30後半の事象だけでh15まで不当に不適格化していないか。
- 「保存値を再現できた」と「一意な時刻・pose対応を独立に確認した」を混同していないか。
- 0.5秒holdでgeometryを切る規則は、停止/再発進の観測をどう扱うべきか。意図的停止や安全endpointとは独立に検討する。
- GEOMETRY_VERIFIEDの必要十分な根拠と、PATH_SUPERVISION_REVIEWEDに追加すべき根拠を明示する。
- UNKNOWNと既知の支持0、mask/censor、raw/strictの母数を集計で混同していないか。

### C. reader・evidenceの信頼性

- plain MCAP indexとfile-zstd forward streamの能力差、時間窓の完全性、早期停止の時刻単調性仮定。
- headerとbag時刻のmargin、pose/velocity補間、clock/reset/run/segment境界の根拠。
- 計数漏れ、例外経路、展開メモリ・時間上限、PARTIAL保存、dry-run identity、source不変性の限界。
- schema/型定義の差、decode失敗、未記録topic、source順序を暗黙に成功へ落としていないか。
- 合成テストでカバーできることと、実sourceで未確認のことを明確に分ける。

### D. 次段の個別gate

geometry-only converter、停止/発進教師設計、controller/MPC oracleを別々に判定してください。
「すべてBLOCKED」という現行結論が正当かもレビュー対象です。根拠が足りる部分だけを条件付きで進められるなら条件を示してください。
Referenceの実在を推論時route intent、環境整合、clearance、車体制約の証明にしないでください。

## 6. 出力形式

日本語で、次の順に回答してください。

1. 実際に確認できた版・資料、読めなかったもの。
2. 重要な指摘を優先度順に。各指摘へfile/関数/確認可能な行、根拠、影響、最小修正、検証テストを付ける。
3. 報告済み事実・コード確認事実・推測・未確認事項の整理。
4. 同一stamp競合の分類と、採用/保留/除外の判定表。閾値の根拠と未校正部分を明記。
5. tier、prefix/window、h15/h30比較、UNKNOWNの修正案。変更不要なら理由を示す。
6. geometry-only / 停止教師 / oracleの独立した判定。
7. 次に行う最小の1タスクを選び、そのままローカルCodexへ貼れる実装依頼プロンプト。

最後の実装依頼は、原則として保存済み`raw_window_evidence.json`等の小さな成果物のみを入力にし、原本不変・versioned sidecar・合成テスト・明示的完了条件を含めてください。
必要な情報が抽出物に存在しない場合、捏造せずその分岐をBLOCKEDとし、追加取得の必要性を別途提案してください。新規raw展開や予算拡大を既定手順へ入れないでください。
ローカル実装はWindows編集/commit→既定sync→WSL lock付き検証。既存変更破棄、強制reset、process停止、自動push、学習・走行は禁止と明記してください。
