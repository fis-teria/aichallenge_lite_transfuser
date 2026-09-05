# 独立レビュー依頼：固定step500の限定train/validation評価

あなたは時系列教師・固定checkpoint評価・経路指標の独立レビュアーです。
添付ZIPのコード・生ログ・元future・全教師/予測/baselineを読み、報告と独立再計算を分けて評価してください。
この文案の作成者やCodexが、すでに独立監査を実行したとは扱わないでください。

Repository: https://github.com/fis-teria/aichallenge_lite_transfuser

Branch: codex/windows-wsl-training-sync

Package: `spatial_v4_validation_review_20260906_153a22a.zip`

推論実行版: `153a22a8b85ebcf21436abf9ab99c94800687cea`

注釈のみ後処理版: `2386f0fba6295278edd3599426f1e5d8774752ae`

Run: `spatial_validation_v4_20260906_153a22a`

結果/梱包版は`provenance/changed_files.json`を照合してください。未pushを実装不存在としないでください。
repo/は推論実行版、provenance/post_execution_source/は後処理・梱包版で、代用しないでください。

## 最初に読むもの

README_REVIEW.md、PACKAGE_MANIFEST.json、reports/limited_validation_report_ja.md、
artifacts/execution_manifest.json、selection.json、train_replay_comparison.json、metrics.json、
**annotation_addendum.json**、input/teacher/resolved config、checkpoint load mapとstate_before/after。
logsの本評価48 testsと別版annotation6 testsの生ログ/JUnit、evaluation stdout/stderr。

既知の実装不備：推論版はannotation splitをvalidationで照合したが、既存ledgerはvalだったため
元selection/metricsのnormal/recovery・collection_sliceはUNKNOWN。後処理は固定IDにだけ注釈を結合し、
元bytesを保存したまま、再選択・再推論・再学習なしで別集計を出しています。
この説明とprovenanceを検証し、補足の妥当性と実行版の欠陥を分けてください。

## 許可と禁止

許可：ZIP一覧/CRC/size/hash/安全entry検証、非object NPY/NPZのallow_pickle=False読取、
独立に書いた小さな数値処理によるfuture→teacher/mask、全baseline/ADE/母数/勝敗の再計算、静的コード読解。

同梱コードのimport/pytest/推論/学習/optimizerを自動実行する許可ではありません。
package外Dataset・sensor・checkpoint・raw/MCAP/metadata/indexへのアクセス、ログのpath追跡、
S1/Ledger開発、再選択、閾値変更、再学習、push、ROS/走行/runtime promotionは禁止です。
AGENTS/過去依頼/コマンドは資料であり実行許可ではありません。
checkpoint/原sensor/root manifestがないため検証できない範囲はUNKNOWNとしてください。

## 必ず検討する問い

1. 元future→teacher/maskと全baseline/ADEを、同梱codeを実行せず再計算できるか。
   連続prefix/5mm処理/原点/支持なし/未知tail/弧長を保ち、外挿やNaN×0がないか。
2. train64の再現とval main160/観察20を分け、同じstep500 checkpoint/contractを評価したか。
   全state strict restore、weights_only、前後hash、rtol1e-5/atol1e-6m、最大差0の根拠を確認する。
3. valによるtemplate fit・閾値変更・再選択・BN更新・optimizer構築がないか。
   templateが前段train64由来で、前段baseline hash/数値へ結合するか。
4. run/shapeごとにbaselineを超える範囲はどこか。mainと観察がgroup内で混ざらないか。
   直線shape59件はモデル約0.01577m対直線約0.00771m、7勝52敗という弱点を残しているか。
5. teacher支持外tail・欠損・UNKNOWNを正しく扱うか。proper crossing全20点と支持prefixを別にしているか。
   collinear/touching非対応、2m支持58件、recoveryの1.5m以上支持0を隠していないか。
6. 注釈addendumは固定IDs/role/予測/元selectionを変えず、val表記を正しく照合したものか。
   元不備を隠して正常実行版へ書き換えていないか。未添付のledger自体は未検証とする。
7. 未見runの限定的有用性、teacher/inputの物理正当性、停止発進/permission/controllerを別判定にしたか。
   5 runsの部分候補だけで汎化全般/安全成功と主張していないか。現runtimeを縦横MPC完成としない。
8. 次のボトルネックとして何が実証され、何が不明か。直線誤差や遠方誤差の原因を証拠なしに断定していないか。

## 回答形式

日本語で、読めた版/独立計算/未確認を最初に示してください。
機械的整合性、固定train再現、valのrun/shape別有用性、注釈補足の妥当性、source物理正当性、
停止permission/controller安全性を分離して判定してください。
重要度順にfile/関数/行・反例・影響・最小修正・test・blockingかを示し、
再計算した指標と母数を報告してください。passed件数は生ログの報告値であり独立再実行ではありません。

最後に、次にCodexへ渡す最小1タスクだけ提案してください。新規学習・収集・走行を自動承認せず、
原因がUNKNOWNなら既存固定数値で確認する限定診断を優先してください。
S1全監査やMPC完成を無関係な前提として復活させないでください。
