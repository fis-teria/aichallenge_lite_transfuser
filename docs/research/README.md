# 文献調査管理データ

対象期間は2018-10-02〜2026-10-02。期間前の基礎は別枠です。

- `taxonomy.json`: 分類、対象期間、調査状態、研究系列の重複除去方針
- `coverage-matrix.{json,csv}`: 時代 × 領域 × 資料種別の全144セル。`coverage_complete` は全セル `false`
- `source-ledger.{json,csv}`: 一次資料の書誌、確認日、確認範囲、関連記事、研究系列ID
- `timeline.{json,csv}`: 書誌に基づく資料年表（分野全体の網羅的技術史ではない）
- `existing-article-audit.{json,csv}`: 既存17記事と今回の重複・更新方針
- `research-writing-queue.{json,csv}`: 次の調査・執筆と公開前確認条件
- `editorial-roadmap.md`: 人が読む編集計画

CSVはUTF-8 BOM付き。配列・辞書セルはJSON表記です。機械処理にはJSONを優先してください。空欄は未確認を表し、DOIがない、改訂がないと断定しません。

## source-ledger の最小契約

| フィールド | 意味 |
|---|---|
| source_id | 資料（版・URL）単位の一意ID |
| study_family_id | 同一研究系列。会議版・arXiv・雑誌版・公式コードを束ねる |
| title | 資料名 |
| authors_or_organization | 著者または公式組織名。省略時は明記 |
| year / publication_date | 公表年 / 確認できた公表日 |
| updated | 最終改訂日、版、または未確認 |
| url / doi | 確認した一次URL / DOI（未確認は空欄） |
| source_type | paper / preprint / survey / code / benchmark / technical |
| domain / era | 分類ID |
| checked_date / checked_scope | 確認日 / 本文・概要・公式ページ等の読解範囲 |
| related_articles | 既存・新規記事slugの配列 |
| review_status | 調査済み / 未調査 / 資料不足 |
| boundary_note | 対象期間境界、公表日と改訂日、採択状況の留保 |
| origin_ledger | 整理に使った領域別台帳名 |

`調査済み`は特定資料の明記された範囲を確認した状態です。本文未読で書誌のみ確認した資料は、その制限を必ず `checked_scope` に残します。詳細な技術主張の根拠として使う前に該当節を確認します。

## 公開境界

このフォルダはリポジトリ内の編集管理用です。公開サイト `docs/site` にCSV/JSONをコピーしたりリンクしたりしません。読者向けの確認済み内容は `docs/site_src/articles/research-map.html` にあります。作業仮説・候補検索のメモは公開記事へ自動転載しません。

## 統合

1. Windows正本の変更を確認し、無関係な変更を保全する
2. `docs/research` と `docs/site_src/articles/research-map.html` を配置する
3. 追加・更新する記事のレコードを既存 `docs/site_src/articles.json` にslug単位で統合する。既存の公開日・無関係な記事情報を保全する
4. 文献担当の3概説・個別記事と関連記事を統合する
5. `python tools/build_project_site.py`
6. `python tools/build_project_site.py --check`
7. `python tools/check_project_site.py`
8. UI・ナビゲーション変更時は `python tools/check_project_site_browser.py`

ここでの確認は文献と記事の確認です。実装再現・テスト・AWSIM走行の成功を意味しません。

## 次回のポータブルな更新手順

このフォルダのJSONを正本として、追加資料に一意の `source_id` と既存・新規の `study_family_id` を割り当て、確認日と確認節を追記します。`source_type` は確認したURLの版を表します。査読済み研究の著者arXiv版も、URLの資料種別としては `preprint` です。`original_source_type` とDOIを残すため、研究自体の査読状況は失われません。`year` は元の書誌年、`classification_year` は資料の初稿年を確認できた場合の分類年です。

追加後はCSV、年表、144セルの資料ID・件数をJSONから更新し、全資料IDの一意性、URL、研究系列の重複、記事slugの存在、資料数とセル件数の一致を検査します。確認済み資料の追加だけでは `coverage_complete` を `true` にしません。版・評価の不足は次の調査キューへ残します。公開の `research-map` 記事には確認範囲と関連する解説のみ反映します。

記事の安定参照は `research-map.html`、`e2e-field-overview.html`、`racing-overview-2018-2026.html`、`mpc-mppi-overview.html`。既存のTransFuser・MPC記事のslugを変更せず、関連記事として相互参照します。

原稿の受け渡し用manifestと作業補助スクリプトは公開記事や保守スクリプトではありません。初回反映の対象はこの `docs/research/`、記事編集元、記事メタデータと生成された公開HTMLです。

公表年を確認できない資料は `year: null`、`classification_year: null`、`era: undated` です。これらは144セルの時代別件数に含めず、台帳と年表の末尾に保全します。そのため全台帳の件数と時代別セルの件数合計は一致しない場合があります。差分は `undated` の件数と一致する必要があります。

## 初回サイト検査（2026-10-02）

- Windowsのサイト用worktreeで7本文（6新規・TransFuser原著1更新）を反映。23記事・27ページ。
- `python tools/build_project_site.py --check` と `python tools/check_project_site.py` が成功。965リンク・asset参照、29種類のcommit固定根拠を確認。
- `python -m unittest discover -s tests/site -p 'test_*.py'` は29件成功。
- `python tools/check_project_site_browser.py --channel msedge` が成功。5画面幅、全27ページのPC・スマホ表示、検索、キーボード、旧hash導線、JS無効時の閲覧を確認。
- 追加・更新7記事は320 / 390 / 768 / 1440 pxでも個別確認。表のスクロール、図・数式と長い書誌情報の折り返しを修正し、画像を目視確認した。
- 資料台帳55資料・45研究系列と108セルを照合。31セルは範囲限定の調査済み、77セルは未調査で、網羅済みセルは0。公表年不明の2資料を時代別集計から除外。
- 外部一次資料59 URLは56件HTTP 200。DOIの出版社転送先3件は403または202で自動取得に制限があり、出版社本文の全件取得成功とは扱わない。記事のarXiv・著者資料へのリンクは保持した。
- 学習・制御・ROSコードは変更していない。全pytest、学習、AWSIM・実車走行は今回未実施。既存の全pytest記録との区別は `docs/site/README.md` を参照。

上記は公開前のサイト検査であり、論文の追試や走行性能の検証ではない。公開commitのCIと実配信確認は公開操作の結果として別途報告する。

## 定期技術ニュース第1便（2026-10-02）

20論文と6補助資料を統合し、81資料・65研究系列へ更新しました。navigationを研究分類へ追加し、144セル中34セルは範囲限定の調査済み、110セルは未調査です。全セルの `coverage_complete` はfalse。公表年不明4資料は台帳と年表に残し、年代別集計から除外しました。初回の55資料・108セルの検査記録は上の履歴として保持します。

- `article-intents.{json,csv}`: ニュース由来20論文の記事目的、公開前条件、状態。3件は `published`、16件は `awaiting_fuller_review`、LOOP1件は `on_hold_primary_inconsistency`
- `story-batches.json`: 比較の軸でまとめた候補群。掲載順は固定優先順位ではない
- 文献の `review_status` と記事の `article_status` は別。書誌や限定節の確認だけで個別記事を公開済みにしない
- DreamStreamのREADMEは実行コード未提供のためtechnical。StreamRigのコード・評価protocol・設定は同一研究系列。TrafficSignBenchのコード・データは初公表年未確認のためundated
- 共有会話の本文・URL、原論文の丸ごと、重み・datasetをこの更新へ含めない

記事commit `0a6a0024d2cea4d003e1b641ba22e5b8b0c57855` の[CI](https://github.com/fis-teria/aichallenge_lite_transfuser/actions/runs/37006412478)と実配信を確認し、3件をpublishedへ更新しました。公開URL・確認結果は `news-publication-20261002.json` に記録しています。30ページと3資産の完全一致、実配信の4ページ×4画面幅と相互リンクを確認しました。生成・リンク・サイトテスト・ブラウザ検査は既存の `docs/site/README.md` の手順で実施します。学習・制御コードの変更、学習、走行、著者結果の独立再現はこの文書更新に含みません。

### 第1便の公開前検査

- Windowsの生成・一致検査とサイト検査が成功。30ページ、1236リンク・asset参照、31種類のcommit固定根拠を確認。
- サイト回帰テスト29件が成功。既存ブラウザ検査で5画面幅・全30ページのPC/スマホ・検索・キーボード・旧hash・JS無効時を確認。
- 3新記事と研究地図を320/390/768/1440pxで追加確認。表題・横スクロール・図の文字境界、長い書誌情報の折り返しを確認し、図を目視確認。
- 台帳・CSV・年表・範囲表のIDと件数を照合。既存55資料のレコードを保全し、未調査の過去年代と全セルcoverage_complete=falseを維持。
- 追加記事・資料の外部参照59 URLを確認。28件は通常HTTP 200、GitHub HTMLの一時的な503等があった31件は同一commitの公式rawまたは再取得でHTTP 200を確認。arXiv節リンクの参照先欠損なし。GitHub HTMLの全件取得成功とは扱わない。
- 学習・制御・ROSコード、設定、既存pytest対象に差分はない。全体pytestは今回再実行しておらず、学習・走行・著者結果の追試も行っていない。

## LMPCの追加調査（2026-10-03）

LMPC (2019) の個別記事を追加し、レーシング・MPC概説と研究地図へ接続しました。原著v4全7頁と図表、著者シミュレーション実装、論文指定BARCブランチを静的に確認しています。論文とコードの予測長・履歴選択・slackの違い、QP単独時間と制御全体の違いを区別しました。現行公開系統 `45c0b4d3f17ad7135b40e219be71aab39a88faa3` のモデル・配布設定・制御コード・検証記録に基づく導入候補であり、実装や追試は行っていません。

台帳は84資料・66研究系列、144セル中34セルは範囲限定の調査済み、110セルは未調査です。公式2実装の初公表日は未確認のためundatedとし、計6資料を年代集計から除外します。commitの日付を初公開日と読み替えません。全セルcoverage_complete=falseを維持します。ニュース由来20件のarticle-intentsは変更していません。

親側から受領した共有ニュースの再確認（2026-10-03 02:42 UTC）は、既存3便20 IDと同じ共有スナップショットでした。元会話のlive状態に新情報がないことは確認していません。確認範囲を `news-intake-check-20261003.json` に記録し、私的なURLや会話本文は保存していません。

生成・検査・公開確認は既存の `docs/site/README.md` の手順を使用します。本更新は文書のみで、学習・制御・ROSコードや設定の変更、全体pytest、学習・AWSIM・実車試験は含みません。公開状態は対象CIと実配信の確認後に別途記録します。

### LMPC記事の公開前検査

- 生成・一致検査とサイト検査が成功。27記事・31ページ、1309リンク・asset参照、31種類のcommit固定根拠を確認。
- サイト回帰テスト29件成功。既存ブラウザ検査で5画面幅、全31ページのPC・スマホ、検索・キーボード・旧hash・JS無効時を確認。
- 追加記事と研究地図・2概説を320/390/768/1440pxで確認。表5件のcaptionと横スクロール領域、概念図・数式・相互リンクを検査し、図と数式を目視確認。
- 新記事の外部一次資料20 URLはHTTP 200、arXiv節のanchorも確認。台帳・年表の既存81レコードと既存調査キュー28件を保全し、CSVとJSONの件数を照合。
- 文書以外の差分なし。全体pytestや学習・走行試験を今回の成功として扱わない。

LMPC記事の公開commit `9f143da8da3fdedfe40e268da1a28a75428a8ff5` の[CI](https://github.com/fis-teria/aichallenge_lite_transfuser/actions/runs/37092173124)と、変更した14配信HTMLのローカル生成物との完全一致を確認しました。新記事・研究地図・2概説の実配信を4画面幅で開き、相互リンクと表のキーボード横スクロールを確認。公開URL・hash・確認範囲は `lmpc-publication-20261003.json` に記録し、該当記事状態をpublishedにしました。変更のない配信ページ・資産の再取得は行っていません。

## LBCの追加調査（2026-10-04）

Learning by Cheating（CoRL 2019 / PMLR 2020）を1記事として追加しました。既存E2E-LBCの書誌・概要確認を、採択本文10頁の方法・評価と図表の精読へ更新し、公式補足3頁と固定版コードを同じ研究系列へ追加しています。資料数86、研究系列数66、144セル中34セルは範囲限定の調査済み、110セルは未調査です。コード初公表日不明の1件をundatedへ追加し、年代集計から除外する資料は7件です。全セルcoverage_complete=falseを保ちます。

全分岐のwaypoint監督、生徒の訪問状態とreplay、到着率と衝突判定、CARLA 0.9.5/0.9.6の条件差、原著と公開コードの制動閾値の単位差を整理しました。現在の公開系統038d84bの入力builder・未来教師・TimePath・配布設定・検証記録を確認し、command historyと高位の進路指示も区別しました。学習・制御・ROSコードや設定を変更せず、再学習・著者追試・AWSIM・実車試験は行っていません。

親側が2026-10-04 02:06 UTC頃に確認した共有ニュースは、従来の3便20項目と同じ閲覧範囲でした。元会話や分野の最新情報に変更がないことを意味しません。`news-intake-check-20261004.json`に確認範囲を保存し、私的URL・会話本文は含めません。ニュース由来の20件のarticle-intentsは変更していません。

生成・検査・公開はdocs/site/README.mdの既存手順を使用し、公開状態はCIと実配信確認後に別途記録します。全体pytestは今回の文書更新では再実行せず、サイト回帰・表示検査と区別します。

### LBC記事の公開前検査

- 生成・一致検査とサイト検査が成功。28記事・32ページ、1387リンク・asset参照、35種類のcommit固定根拠。
- サイト回帰テスト29件成功。既存ブラウザ検査で5画面幅、全32ページのPC・スマホ、検索・キーボード・旧hash・JS無効時を確認。
- 新記事・研究地図・E2E概説・DAggerを320/390/768/1440pxで確認。6表のcaption・横スクロール、図・数式を確認し、PC・スマホの図と数式を目視確認。相互リンクも確認。
- 新記事の一次資料22 URLはHTTP 200。既存83資料・年表レコードと既存30キュー項目を保全し、LBCだけ確認範囲を更新。CSVとJSONの件数を照合し、ニュース由来20件の状態を変更していない。
- 差分は調査管理とサイト本文・メタデータ・生成HTMLのみ。全体pytest、学習・走行、著者コードの独立再現は今回未実施。

LBC記事の公開commit `4d8215f737dfe3e1e0d07b9df9e20d763a4ca294` の[CI](https://github.com/fis-teria/aichallenge_lite_transfuser/actions/runs/37171095274)と、変更した15配信HTMLの生成物との完全一致を確認しました。新記事・研究地図・E2E概説・DAggerを実配信で4画面幅確認し、相互リンクと表のキーボード横スクロールも確認。公開URL・hash・確認範囲は `lbc-publication-20261004.json` に記録し、該当記事状態をpublishedにしました。変更のない配信ページ・資産は再取得していません。

## PSFの追加調査（2026-10-07）

既存C04を限定節の確認へ増補し、PSFの個別記事とMPC概説・保持経路・研究地図からの逆リンクを追加しました。初稿2018-12-13と確認版2021-05-17を区別し、定理4.6の条件付き確率保証と現行2ファイルの静的確認を記録しています。台帳は86資料・66研究系列で不変、144セル中34セルは範囲限定の調査済み、110セルは未調査。公表年不明7資料を年代集計から除外し、全セルcoverage_complete=falseを維持します。研究地図の概要メタデータを最新JSONの件数へ修正しました。

生成・一致・全リンク・29件のサイト回帰が成功。33ページ・1436リンク/asset・37種類のcommit固定根拠を確認しました。Windows Edgeで5画面幅のナビゲーション、全33ページのPC/mobile、追加・更新4記事の320/390/768/1440px、図・表・数式・逆リンク・JS無効時を確認しました。既存ブラウザ検査のリサイズ直後の判定は一度失敗し、描画2フレームの同期を加えた作業フォルダ内wrapperで検査本体を変更せず再確認して成功。サイトのCSS/JS・検査スクリプトは変更していません。検査記録は `psf-validation-20261007.json`。公開commitのCIと実配信は確認後に別途記録します。学習・制御・ROSコードや設定は変更せず、全体pytest・学習・AWSIM・実車走行・著者結果の独立再現は本更新に含めません。未移送Roach原稿は別件として保持し再作成していません。

### PSF記事の公開確認

記事commit `cba9fe6b0493c7845667e7ccd774d613ebc7677a` をHervararのWindows checkoutから既存fis-teria認証でmainへpushしました。[公開CI](https://github.com/fis-teria/aichallenge_lite_transfuser/actions/runs/37568803977)はsuccess。全33ページと3資産がHTTP 200でWindows生成物とバイト単位で一致しました。[PSF記事](https://fis-teria.github.io/aichallenge_lite_transfuser/articles/predictive-safety-filter-2018.html)と関連記事3本を実配信の320/390/768/1440pxで確認し、構成図・比較表の横スクロール・停止距離式・逆リンク・JS無効時の閲覧に問題はありません。公開記録は `psf-publication-20261007.json`。Git作者情報は移行先で未設定だったため、既存履歴の表記をcommit単位で使用し、グローバル設定・origin・認証は変更していません。新しい資格情報の発行、SSH/WSL/クラウドpush、API書込みは行っていません。

## BayesRaceの追加調査（2026-10-08）

既存R15の本文未確認を解消し、予測残差の学習をLMPCの終端学習・PSFの条件付き保証と比較する価値でBayesRaceを選定しました。前回の制御安全から車両モデルへ軸を移しています。採択本文12頁・付録・図表、arXiv v2、固定版の著者コードを静的確認し、初稿2020とPMLR書誌2021を区別。台帳88資料・66研究系列、年代集計80資料と公表年不明8資料です。分類補正により144セル中33セルが範囲限定の調査済み、111セルが未調査となり、全セルcoverage_complete=falseを保っています。

コードのGP特徴順・scaler、境界制約の既定無効、solve単独の計時、周回後の再学習、論文と基準モデルのヨーレート式および積分法の不一致を記録しました。コード・保存データ・依存の実行や著者結果の再現は未実施です。ニュース由来20件のarticle-intentsは保全。親側の共有ニュース確認は可視3便20項目で新規候補が見えた範囲はなく、元会話のlive更新状態は未確認です。確認範囲はnews-intake-check-20261008.jsonに記録しています。

生成・一致・全リンク検査と29件のサイト回帰が成功。30記事・34ページ、1514リンク/asset・37種類のcommit固定根拠を確認しました。既存ブラウザ検査で5画面幅のナビゲーション、全34ページのPC/mobile、旧hashとJS無効時を確認。新記事と関連記事4本を320/390/768/1440pxで確認し、図・4表・数式・キーボード横スクロール・相互リンクを検査、PC/スマホ画像を目視確認しました。一次資料22 URLとHTML節アンカーはHTTP 200で確認。検査記録はbayesrace-validation-20261008.json。描画2フレームを待つ作業フォルダ内wrapperを使用し、既存検査本体・サイトJS/CSSは変更していません。公開CI・実配信は確認後に別途記録します。学習・制御・ROSコードや設定を変更せず、全体pytest・学習・AWSIM・実車試験はこの文書更新に含めません。Roachの未移送原稿は別件として保持し、再作成していません。

### BayesRace記事の公開確認

記事commit `d99c7a2e523de6f3c15bf6e1eba9e37f252bfbe1` をHervararのWindows checkoutから既存fis-teria認証でmainへpushしました。[公開CI](https://github.com/fis-teria/aichallenge_lite_transfuser/actions/runs/37719572604)はsuccess。全34ページと3資産がHTTP 200で生成物とバイト単位で一致しました。[BayesRace記事](https://fis-teria.github.io/aichallenge_lite_transfuser/articles/bayesrace-2020.html)と関連記事4本を実配信の320/390/768/1440pxで確認し、図・4表・数式・キーボード横スクロール・逆リンク・JS無効時の閲覧が成功。最終題名のローカル表示も再確認しています。記録はbayesrace-publication-20261008.jsonで、該当調査キューをpublishedへ更新しました。公開上の未解決事項はありません。著者コード・保存データの再現、現行依存での起動、実車遅延・未見路面の検証は記事公開とは別の未実施事項です。origin・認証・グローバルGit設定は変更せず、新規資格情報・SSH/WSL/クラウドpush・API書込みは使用していません。

## World on Railsの追加調査（2026-10-09）

既存E2E-WORをarXiv v3の限定本文確認へ増補し、固定版の公式CARLAコードを同じworld-on-rails系列に1資料追加しました。31記事・89資料・66系列。公表年不明9資料は年代別144セルから除外し、年代集計80資料・調査済み33セル・未調査111セルと全coverage_complete=falseを保ちます。公式コードのcommit日時は初公表年へ代用しません。論文・コードの版差と現行TimePathへ必要な教師/出力の設計を個別記事、E2E概説・LBC・研究地図へ反映しています。

ニュース受入記録は親側の公開ページ確認に基づき、直近三版20項目と全七版48項目を区別しました。古い項目は新着にせず、10/8版が見えないことからlive元会話の更新なしとは結論しません。学習・制御・ROSコードや設定変更、著者コード実行、学習・走行・独立再現は未実施です。生成一致・35ページの1581リンク/asset・42種類のcommit固定根拠、29件のサイト回帰が成功しました。Windows Edgeで5画面幅のナビゲーション、全35ページのPC/mobile、更新4記事×320/390/768/1440pxの表示、構成図・数式・4表・横スクロール・逆リンク・JS無効時を確認。外部根拠21 URLはHTTP 200。既存検査本体・CSS/JSは変更していません。検証記録は `wor-validation-20261009.json`。対象commitのCI・実配信はpush後に別記録へ残します。
