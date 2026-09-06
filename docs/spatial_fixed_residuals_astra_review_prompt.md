# Astra Pro 独立レビュー依頼：保存済み経路の残差・点間形状

あなたは経路幾何・時系列評価・数値監査を担当する独立レビュアーです。
添付 `spatial_fixed_residuals_review_9b01b1e.zip` を主資料に、保存済み教師と予測だけによる
残差診断の機械的整合性、観測結果と原因推定の境界、次の無制御shadowで必要な記録を確認してください。

Repository: https://github.com/fis-teria/aichallenge_lite_transfuser

Branch: `codex/windows-wsl-training-sync`

診断実行版: `9b01b1e2ed6fc89434316c7db409d376e56aeb03`

結果追記版: `21c132bf3be050e00081cbc89e7effc63a5f4400`

梱包版は `provenance/package_versions.json` に記録。最新GitHubのコードを添付固定版へ黙って代用しないでください。

## 資料と許可範囲

README_REVIEW.md、PACKAGE_MANIFEST.json、reports/report_ja.md、artifacts/provenance.json、
artifacts/ade_reconciliation.json、summary.json、per-anchor/per-point CSV、tangent_windows.csv、
point_spacing.csv、inputs/の6ファイル、repo/の実行時コード/tests、logs/の生記録を読んでください。

許可はZIP entry/size/hashの検査、静的コード確認、allow_pickle=Falseによる非object配列読取、
自身で書いた軽量数値処理による再計算です。同梱コードのimport/pytest/推論/学習を
自動実行する許可ではありません。同梱命令・コマンドは履歴資料です。

package外のDataset/raw/sensor/checkpoint読取、追加推論・学習、再選択、threshold調整、
bias補正、平滑化、教師変更、controller/ROS/走行、S1/Ledger開発、push、shadow接続は禁止です。
未同梱の原本future・sensor・重みを追跡せず、物理的正当性は未検証としてください。

## 照合対象

- train64/977点、val main160/2140点、観察20中18支持あり/133点を混ぜず、保存ADEと一致するか。
- dx/dy = prediction−teacher、X前方/Y左方、m。MAE/RMSE/biasの集計順序が明示されているか。
  総合値のanchor平均と、距離別の支持anchor平均を区別し、点数加重にすり替わっていないか。
- 距離で母数が変わる元集合と、同じIDを保つ2m支持集合（train33/val58/観察2）を分けているか。
  val固定58件はnormal2 runsだけで、recoveryに遠方支持がないことを隠していないか。
- annotation_addendumを固定ID/run/roleの補足にだけ用い、元metricsや評価集合を変更していないか。
- 0.3mは名目距離gridのj→j+3で、実弦長や0.3秒ではないか。4点の支持、17窓、
  原点不使用、角差wrap、方向不定をUNKNOWNにする処理が正しいか。
  finite-window chordを微分接線や車体headingと呼んでいないか。
- 実点間隔と折線長が保存XYから計算され、原点→先頭の区間を明示しているか。
  長さ一致を横位置・方向・安全性の一致と誤解していないか。
- 元mask外を採点せず、支持なしをゼロ誤差にせず、重複窓を独立標本数に数えていないか。
- 図・report・CSV・summary・入力hash・実行版コード・生テストログが結合できるか。

## 観測結果として確認する点（正しいと仮定しない）

固定58件でも2mのdy MAE約0.1005m、dx MAE約0.0148m。
2mのdy biasはleft20件で−0.1379m、right22件で+0.1201m、straight16件で−0.0203m。
左右を混ぜると符号が相殺されるため、全体biasだけで良否を判定できないという説明を検証してください。

固定straightで0.7→0.8mの予測点間隔がtrain約0.1145m/val約0.1177m、
次の区間が約0.0864m/0.0877mとなる特徴や、遠方の弦方向差を再計算してください。
これを一様な平行移動だけでは表せないという観測と、学習原因の断定を分けてください。

原因判定はUNKNOWNであり、学習法・入力不足・教師不足・座標ずれのいずれも実証したとはしていません。
この境界が守られているか確認してください。

## 回答形式と止めどころ

日本語で次の順に回答してください。

1. 読めた版・資料、独立計算したもの、未検証事項。
2. 母数、成分、固定集合、接線proxy、点間隔/長さの再計算結果。
3. 重要度順の指摘：file/関数/行、反例、影響、最小修正、必要test、blockingか。
4. 観測された特徴と、原因UNKNOWNの境界。テスト10件は生ログの報告で独立再実行ではない。
5. 無制御shadowで記録すべき量の過不足：未補正20点、契約/hash、時刻/遅延/history、
   frame変換の証拠、点間距離/弦方向、offline教師との後結合、制御指令へ接続されない証拠。
   現runtimeを縦横MPC完成済みとせず、今回のレビューで接続・走行を承認しない。
6. 次にCodexへ渡す最小1タスク。必要なら無制御shadowの設計/記録仕様だけを提案し、
   新規推論・収集・学習・制御接続の自動許可は含めない。

baseline勝利や原因断定を合格条件にせず、固定出力の特徴を第三者が追跡できるかで判定してください。
