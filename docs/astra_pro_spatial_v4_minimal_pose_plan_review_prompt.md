# Astra Pro：Spatial Path V4 最小追加pose証拠取得計画の独立レビュー

あなたはROS bag時系列、教師データprovenance、数値幾何、学習ベース経路計画に詳しい独立監査担当です。
以下の固定版コード・レポート・添付成果物を読み、最小追加証拠取得計画の妥当性をレビューしてください。
目的は「何を取得すればどのclaimを判定できるか」を確定し、次にローカルCodexへ渡す最小の依頼文を作ることです。
同じ保存JSONを再分類して全件UNKNOWNを繰り返すことや、閾値を緩めてPASSを増やすことは目的ではありません。

## 1. 対象版：古い公開版と混同しない

Repository: https://github.com/fis-teria/aichallenge_lite_transfuser
Branch: `codex/windows-wsl-training-sync`

- 計画生成・focusedテスト実行版: `17ead237de8f1a6b8e52fdc18bcf7c01e75e5403`
- 今回の計画結果レポート固定版: `69d21d376ac95dde881fa75d5773827fb68fd04e`
- 初期plan実装版: `1efb7a4b206f51d71d83fa37dcb022f243366a90`
- 前段pose競合監査実行版: `befc97dd98434843245b888fb6d1d7110acc8a93`
- 前段pose結果追記版: `abec800e031a07f2898612af5d8177651354fd84`
- 旧raw監査実行版: `7ae8298b71aa7bdbacf1d1757798bf0de66bcfc4`
- 旧rawレポート固定版: `428f30aca0418bc6362cbe8eb78b9aa6a07e1e4f`

本プロンプトの追加commitは上記より後です。branch HEAD、本プロンプト版、計画生成版、結果追記版を分離してください。
過去のWeb API422や公開HEAD `ec255bce…` を理由に、今回の指定版を未実装と断定しないでください。
まず実際に取得できたcommit・ファイル・範囲を明記。取得不能はアクセス未確認であり、object不在の証明ではありません。
指定版を読めなければ必要なファイルの添付を求め、古い版を黙って代用しないでください。

最初に読むレポート:
https://github.com/fis-teria/aichallenge_lite_transfuser/blob/69d21d376ac95dde881fa75d5773827fb68fd04e/docs/spatial_path_v4_minimal_pose_evidence_plan.md

計画生成コード:
https://github.com/fis-teria/aichallenge_lite_transfuser/blob/17ead237de8f1a6b8e52fdc18bcf7c01e75e5403/src/aic_transfuser_lite/data/spatial_pose_evidence_plan_v4.py

CLI:
https://github.com/fis-teria/aichallenge_lite_transfuser/blob/17ead237de8f1a6b8e52fdc18bcf7c01e75e5403/tools/plan_spatial_pose_evidence_v4.py

合成テスト:
https://github.com/fis-teria/aichallenge_lite_transfuser/blob/17ead237de8f1a6b8e52fdc18bcf7c01e75e5403/tests/test_spatial_pose_evidence_plan_v4.py

前段poseレポート:
https://github.com/fis-teria/aichallenge_lite_transfuser/blob/abec800e031a07f2898612af5d8177651354fd84/docs/spatial_path_v4_pose_conflict_review.md

必要な関連コードは上記固定版の `AGENTS.md`、`spatial_pose_conflict_v4.py`、`spatial_evidence_v4.py`、
`spatial_source_reader_v4.py`、`mcap_converter_v2.py` に限定して静的参照してください。

## 2. Webから見える証拠とローカル成果物の境界

Gitにはコード・テスト・レポートがあり、raw、Dataset、checkpoint、大きなJSON成果物はありません。
レポートにあるWSL/WindowsパスをWebから開けると仮定しないでください。

別添がある場合の計画成果物6件:

1. `execution_manifest.json`
2. `minimal_read_proposal.json` (約3.54MB、全seed/candidate/hash/endpoint/closure)
3. `claim_requirements.json`
4. `input_manifest.json`
5. `unresolved_and_unrecoverable.json`
6. `report_ja.md`

計画logical identity:
`394292c7c268715a5efc4cd408f0e9634835d5d4cf016729a01c9f4786dde71d`

成果物が未添付なら、コードとレポートに基づくレビューを進めつつ、実際のID/hash照合は未実施としてください。
不足資料を具体的に絞り、raw全体やDatasetのアップロードを要求しないでください。
計画成果物は入力9 JSONそのものではありません。入力内容まで独立検証したとは表現しないでください。
以下の実行結果はローカル報告であり、あなた自身が再実行した結果ではありません。

## 3. 報告された結果

- 固定9 JSON、11,768,262 bytes。期待hash・前後hash一致。
- Dataset identityはmanifest文字列を照合しただけでDataset本体未検証。
- 保存6,600 records / 29 anchors / pose685群 / 重複172群、全重複は2候補かつ異なるpayload hash。
- 非ゼロ投影差171群、XY/yaw一致1群。正のXY差<=1e-8mは44群、yaw非ゼロ170群。
- 保存最大XY差0.1311721647999626m、最大yaw差0.015095404171680205rad。物理誤差・安全閾値ではない。
- anchor姿勢への差分候補依存10件、strict prefixへの依存はh15/h30各15件。
- recovery12件はpayload未取得。無競合としていない。
- 新規33 + 非学習synchronization10 = 43 passed in 0.43s。全pytestやoptimizerは実行していない。
- Windows/WSLで同じlogical identity。既存tier変更、新教師生成なし。
- 計画生成時のraw/Dataset本体読取、学習、推論、制御変更、走行、pushは0。
- その後、ユーザーが結果共有のためGit pushを別途依頼。公開作業はraw取得承認ではない。

最大4seedの選択役割と結果 (全てnormal131505、normal132822も候補母集団に含む):

| 役割 | group ID先頭 (完全IDはレポート/JSON) | stamp ns | 保存XY差 m | 部分probe |
|---|---|---:|---:|---|
| anchor endpoint非ゼロ | 8fbd120c | 6189999861 | 0.00006032426614470164 | 6群/9候補/122clock、0.610秒 |
| strict prefix内大差 | 208cfcac | 256259994272 | 0.060719386103918874 | 6群/8候補/587clock、2.925秒、上限超過 |
| 旧1e-8近傍 | 353301a9 | 5449999878 | 0.000000009857472638787348 | 6群/7候補/128clock、0.635秒 |
| XY/yaw同値対照 | ffe514be | 939999978 | 0 | 1群/2候補/101clock、0.500秒、anchorなし |

4 partial probesのunionは19群/26候補。個別上限超過を合算上限内という理由で解除しない。
別のfull-prefix計画は5 anchors × h15/h30 = 10件。
6件は観測closure上限内、2件超過、2件は正の既存prefixなし。
後半anchorのh30は122群/146候補/705clock/約3.520秒。
全体最大XY差群は選択anchor群のstrict prefix非依存。prefix最大誤差とはしない。
全14 claimの独立domain partitionと原本候補scope完全性は未確定。

## 4. 重点レビュー項目

1. Claim A保存候補差、B指定policyの限定stream再現、C投影不変性/感度、D物理正確性、E教師採用/発進/Safety/MPCの分離は正しいか。
   Bに本当に必要な証拠と単に便利なfieldを区別。投影同値Cに順序を過剰要求していないか。
2. seed役割・重複統合・偏り・対照・最大差の依存範囲は適切か。
   anchor役割の先頭欠損step1が、適格prefixではなく診断と明示されているか。
3. anchor endpoint→全future、左右pose/velocity endpoint、clock/domain/source/schemaのclosureに欠落はないか。
   seed上限、個別closure上限、合算上限、物理chunk読取コストを混同していないか。
4. 局所窓のclock観測と独立domain割当、whole-run epoch alias不存在を混同していないか。
   未収録domainでも他の十分な証拠があり得るのに一律BLOCKEDにしていないか。
5. source completenessが未確定な場合、限定probeでも有用に検証できるpredicateは何か。
   全体再現の主張を撤回した部分probeと、必要依存を削った不正な全prefix主張を区別すること。
6. 1秒窓/256clock等の停止上限は診断目的に合理的か。大差群を取得すべきなら、最小代替案と別承認が必要な変更量を示すこと。
   単に予算を増やしたり、3probeだけで全prefix確認済みにしないこと。
7. source/schema/記録位置・順序を取得すれば何が変わり、何が残るか。
   historical AnyReader版・file/connection列挙・tie証拠と、現在版/physical-lastを同一視していないか。
8. schema検証、固定allowlist、symlink/reparse、入出力包含、前後hash、例外/上限/PARTIAL、logical identityとテストに具体的欠陥はないか。
   指摘は固定版ファイル・関数・行と反例に結び付け、重大度・影響・最小修正・非学習テストを示すこと。

一次仕様も確認してください:
https://mcap.dev/spec
https://ternaris.gitlab.io/rosbags/api/rosbags.highlevel.html

sequenceは0/recorder由来の場合があり、publish_timeはlog_timeと等しい場合がある。
channel IDはpublisher IDではない。chunkファイルoffsetと展開chunk内offsetを分離する。
これらは仕様上の話で、収録ファイルに意味のある値が存在する証明ではありません。
未収録/意味不足なら代替十分証拠を評価し、なければNOT_RECORDED/UNRESOLVABLE_FROM_THIS_SOURCEで終了する。

## 5. 実行権限と将来予算

この依頼はレビューと次タスクの文案作成だけです。
raw/metadata/index取得、Dataset操作、学習、推論、optimizer、oracle、ROS/実車走行、既存tier変更を実行・承認しないでください。
将来停止上限案はsource64MiB、expanded128MiB、5,000 messages、60秒、temporary disk0、single record16MiB、8chunks。
estimateはnull。JSONサイズからrawコストを作らず、旧残量・不確かな過去counterを流用しないでください。
readerの例外時計数、partial/error manifest、schema hash、早期停止仮定、source変化伝播、log/header範囲の分離は将来実装の前提です。

最終目標は経路生成と縦横制御の分離ですが、現行選択runtimeを縦横MPC実装済みとはしないでください。
geometry-only設計、実データ採用、停止教師、controller/MPC oracleは別gateのまま維持します。

## 6. 出力形式

日本語で次の順に回答してください。

1. 実際に読めた版・資料と未読範囲。ローカル報告と独立確認を分離。
2. 計画の妥当な点と問題点。重大度・根拠コード・反例・修正・必要テスト。
3. A〜Eごとの必要predicate、代替十分証拠、取得で解ける範囲、解けない条件の表。
4. 4seed/closureの維持・修正案。変更する場合は具体的理由と元IDへの結合。
5. 「計画修正だけ必要」「readerを非実行で整備可能」「限定取得案をユーザーへ承認申請可能」の判断と条件。
6. 次にローカルCodexへ貼れる、1目的の最小タスク依頼文。対象commit/変更ファイル/許可・禁止/合成テスト/成果物/完了条件を含める。

次タスクの文案を作っても取得を承認したことにはしないでください。
rawが必要なら、対象source・段階・window・claim・独立予算・中止条件を具体的に書き、
`raw_execution_authorized: false` / `approval_gate: PENDING_EXPLICIT_AUTHORIZATION` のままユーザー判断で終えてください。
