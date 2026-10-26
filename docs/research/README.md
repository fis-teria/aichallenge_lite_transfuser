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

### World on Rails記事の公開確認

記事commit `1de8216b39cf922992506963edec23b8e7cee157` をHervararのWindows正式checkoutから既存fis-teria認証でmainへ通常pushしました。[対象CI](https://github.com/fis-teria/aichallenge_lite_transfuser/actions/runs/37895990742)はsuccess。全35ページと3資産がHTTP 200でWindows生成物とバイト単位で一致しました。[記事](https://fis-teria.github.io/aichallenge_lite_transfuser/articles/world-on-rails-2021.html)と関連記事3本を実配信の320/390/768/1440pxで確認し、構成図・数式・4表・横スクロール・逆リンク・JS無効時に問題はありません。公開記録は `wor-publication-20261009.json`。Git作者表記は既存履歴をcommit単位で使用し、グローバル設定・origin・認証・公開workflowは変更していません。

## MPPI-Genericの追加調査（2026-10-10）

既存C08をarXiv v4の限定本文確認、C09を公式解説の再確認へ増補しました。公式ライブラリと比較コードの固定版を同じmppi-generic系列へ2資料追加。32記事・91資料・66研究系列、公表年不明11資料を除く年代集計80資料です。144セル中調査済み33・未調査111、全coverage_complete=falseを維持します。新コードのcommit日時を初公表年へ代用していません。

候補数による掲載平均時間の比較、split/combined kernel、著者コードの計時範囲、ライブラリmainと比較コードgitlinkの版差、現行の静的参照空間MPPIとの境界を個別記事・MPC概説・RMPPI・研究地図へ反映しました。公式コードのビルド・実行、追試、学習・制御コードや設定の変更、走行・独立再現は未実施です。ニュース由来20件のarticle-intentsとRoachの別件原稿は保全しました。

共有チャットは公開共有ページのHTML内JSONをデータとして確認。全7版48項目、最新の掲載期間9/25–10/1で、取得スナップショットに10/8版は見えません。元会話のlive更新なしとは結論せず、古い項目を新着にしていません。範囲はnews-intake-check-20261010.jsonに保存。共有本文・URLは公開記事へ転載していません。

ユーザーの2026-10-10の指定により、記事の編集・生成・commit/push・配信確認はHervararのWindows正式checkoutで行います。記事用の絶対パスをAGENTSのサイト運用、docs/site/README、site-maintenance記事へ明記しました。定期更新設定は親側で扱います。生成・サイト回帰・ブラウザ検査と対象commitのCI・実配信は、それぞれ確認後に別記録へ残します。

正式生成・生成一致、36ページの1,655リンク/assetと45種類の固定commit根拠、29件のサイト回帰が成功しました。既存ブラウザ検査の5画面幅、全36ページのPC/mobile、新記事と更新関連記事の320/390/768/1440px、図・数式・3表・キーボード横スクロール・相互リンク・JS無効時を確認し、PC/mobileの図表を目視確認しました。一次資料16 URLはHTTP 200。CSVはBOMを除いた列名・元の列順とJSONの一致を確認し、対象外の既存行を保全。詳細はmppi-generic-validation-20261010.jsonに記録。対象commitのCI・実配信はpush後に別記録へ残します。

### MPPI-Generic記事の公開確認

記事commit `93c2cc1bc64b477ca9472290ba8fd3e054f054fc` をHervararのWindows正式checkoutから既存fis-teria認証でmainへ通常pushしました。[対象CI](https://github.com/fis-teria/aichallenge_lite_transfuser/actions/runs/38056505465)はsuccess。全36ページと3資産がHTTP 200でWindows生成物とバイト単位で一致しました。[記事](https://fis-teria.github.io/aichallenge_lite_transfuser/articles/mppi-generic-2024.html)と関連記事・運用記事を実配信の320/390/768/1440pxで確認し、構成図・数式・3表・キーボード横スクロール・逆リンク・JS無効時に問題はありません。公開記録は `mppi-generic-publication-20261010.json`、該当キューはpublishedです。公開上のブロッカーはありません。原著・コードの全体監査、依存・実験版の完全対応、著者結果の独立再現、制御導入・走行は別の未実施事項です。記事更新環境をHervararへ統一し、定期設定自体は親側の管理対象として変更していません。origin・認証・グローバルGit設定を変更せず、新資格情報・SSH/WSL/クラウドpush・API書込みは使用していません。

## On-Track SysIDの追加調査（2026-10-11）

MPPI-Genericの計算資源から物理パラメータ同定へ比較軸を移し、既存R06/R07のコード確認不足を補う1記事を追加しました。原著v1 §III–V・Fig.5–6・Tables I–IIとPDF、固定commit 7b2b712f4e5da2f10b1aa753ee538484fa9fe4a2の主要経路、公開mainの車体応答・操舵・配布設定を静的確認。雑誌の書誌2025と初稿2024を区別し、雑誌本文との完全な版比較は未実施です。

台帳91資料・66研究系列、年代集計80・公表年不明11は不変。144セル中33セルは限定範囲の調査済み、111セルは未調査、全coverage_complete=falseを維持。NN58対130、時刻同期・荷重正規化・フィット目的・LUTと計時の境界、RMSE最小のNLSがN.C.となる実機例を整理しました。コード実行・追試・導入、学習・制御・ROS変更、学習・AWSIM/実車走行は未実施。既存ニュース20件と未移送Roach原稿は別件として保全します。

共有ニュースは実際に可視7版48項目を取得。最新期間9/25–10/1と最新message日時10/2は前回と同じです。10/8版が見えないことからlive元会話の更新なしとは結論しません。公開CI・実配信は検証後に別記録へ残します。

### On-Track SysID記事の公開前検査

正式生成・一致検査と全リンク検査、サイト回帰29件が成功。33記事・37ページ、1736リンク/asset、49種類のcommit固定根拠を確認しました。既存Edge検査は5画面幅と全37ページのPC/mobile、検索・メニュー・キーボード・旧hash・JS無効時で成功。新記事と関連記事4本は320/390/768/1440pxで、図1・3表・数式・表の横スクロールと相互リンクを確認し、PC/mobileの図表を目視しました。外部一次資料18 URLはHTTP200、arXiv節リンクも一致。JSON/CSV、無関係89台帳レコード、記事範囲外5514ファイルを保全。CSS/JS・生成/検査ツールは変更していません。検査記録は `on-track-sysid-validation-20261011.json`。全体pytest・学習・走行・独立追試は未実施です。

### On-Track SysID記事の公開確認

記事commit `64e76282d3858906acd40767bd574277fd2d3b4a` をHervararのWindows正式checkoutから既存Git認証でmainへ通常pushしました。[公開CI](https://github.com/fis-teria/aichallenge_lite_transfuser/actions/runs/38107185295)のbuild/deployはsuccess。全37ページと3資産がHTTP200でWindows正式生成物とバイト単位で一致。[新記事](https://fis-teria.github.io/aichallenge_lite_transfuser/articles/on-track-sysid-2024.html)と関連記事4本を実配信の320/390/768/1440pxで確認し、図・3表・数式・キーボード横スクロール・相互リンク・JS無効時に問題はありません。公開記録は `on-track-sysid-publication-20261011.json`、キューをpublishedへ更新。SSH/WSL/クラウドpush、API書込み、remote/認証/グローバル設定変更は行っていません。

## DODGERの追加調査（2026-10-12）

On-Track SysIDの物理同定から、動的navigationの訓練参照と実行監視へ比較軸を移しました。v1/v2の関連本文・機構・評価条件・付録とPDF対応箇所、固定版著者コード02dd5a965bdaa2e9ab60da3f3255b9acf5d45febの選択経路、現行mainの監視・静的SLAM MPPI・限定設定を確認。v1/v2の取得HTML article本文は空白正規化後に一致しましたが、PDF/TeX/成果物の完全一致・証明の監査は未確認です。

既存v1を保全し、主期間後の10/5改訂v2と初公表年未確認のcodeを同系列へ追加。93資料・66系列、年代集計81（初稿年分類を保つ改訂追跡を含む）・undated12。144セル中調査済み33・未調査111、全coverage_complete=false。ニュース20件のうちDODGERだけを更新し、他19件・他90台帳レコードとRoach別件原稿を保全しました。

QP参照と実行入力の分岐、二枝critic、原著と公開設定の目標・評価数の差、実機h違反・推定余裕と認識誤りを整理。著者コード実行・学習・導入・AWSIM/実車試験、全体pytestは未実施。共有スナップショットは可視7版48項目で最新9/25–10/1、message10/2と前回同じ。live元会話の更新なしとは結論せず、会話本文・URLを転載しません。公開CI・実配信は検証後に別記録します。

### DODGER記事の公開前検査

正式生成・一致検査と全リンク検査、サイト回帰29件が成功。34記事・38ページ、1821リンク/asset、52種類のcommit固定根拠を確認しました。既存Edge検査は5画面幅と全38ページのPC/mobile、検索・メニュー・キーボード・旧hash・JS無効時で成功。新記事と関連記事4本は320/390/768/1440pxで、図1・3表・数式・表の横スクロールと相互リンクを確認し、PC/mobileの図表を目視しました。外部一次資料18 URLはHTTP200、arXiv節リンクも一致。JSON/CSV、無関係90台帳レコード、記事範囲外5516ファイルを保全。CSS/JS・生成/検査ツールは変更していません。検査記録は `dodger-validation-20261012.json`。全体pytest・学習・走行・独立追試は未実施です。

### DODGER記事の公開確認

記事commit `2927a0069d6c5ee1ab5e0c336bc2a9312d38a74c` をHervararのWindows正式checkoutから既存Git認証で通常pushしました。[CI](https://github.com/fis-teria/aichallenge_lite_transfuser/actions/runs/38198640216)のbuild/deployはsuccess。全38ページ+3資産はHTTP200、Windows公式生成物とバイト単位で一致。[新記事](https://fis-teria.github.io/aichallenge_lite_transfuser/articles/dodger-2026.html)と関連記事4本を実配信320/390/768/1440pxで確認し、図1・3表・式・横スクロール・相互リンク・JS無効時に問題なし。`dodger-publication-20261012.json`へ記録し、関連資料/intent/queueをpublishedへ更新。ニュース20件は公開4・追加精読15・LOOP保留1です。他19ニュースと無関係な台帳・batchを保全。SSH/WSL/クラウドpush・API書込み・remote/認証/グローバル設定変更、学習・走行は行っていません。

## FIVE-VLAの追加確認（2026-10-13）

DODGERの動的navigationからE2Eの時間表現へ移り、既存v1の本文評価の不足と現行の指令履歴・保持経路との比較価値で選定しました。§3–5・関連付録とPDFの機構/Table5/A.18/A.19、著者紹介頁、現行mainのTimePath・履歴・設定を静的確認。明示位置履歴のオフライン誤差改善と閉ループSR低下、RAMの二状態交代と計時範囲、非反応評価の責任判定/分母を区別しています。

台帳93資料・66系列、年代集計81・undated12、144セル中調査済み33・未調査111は不変、全coverage_complete=falseです。既存FIVEだけを増補し、他92資料/19ニュース項目を保全。原著・著者頁・検索の範囲ではFIVE固有実装/重みを確認できず、名称API検索は403 rate limitで未取得。再試行・別認証で迂回していません。公開実装なしとは断定しません。

共有ニュースは可視7版48項目、最新9/25–10/1・message10/2で前回と同じ。live元会話の更新なしとは結論せず、本文・URLは転載しません。学習/制御/ROS/設定変更、著者コード実行・追試・実装・学習・AWSIM/実車試験、全体pytestは未実施。Roachの未移送原稿は別件として保全。公開CI・配信は確認後に別記録します。

### FIVE-VLA記事の公開前検査

正式生成・一致検査と39ページ1897リンク/asset・58固定根拠、サイト回帰29件が成功。目標点の記録未来軌道由来を最後に補足し、公式再生成と一致確認後に表示検査を実施。既存Edge検査は5幅・全39ページPC/mobile、検索/メニュー/キーボード/旧hash/JS無効で成功。新記事と関連4本は320/390/768/1440px、図1・3表・式・横スクロールと相互リンクが成功し、画像を目視確認。外部一次11 URLはHTTP200、arXiv節アンカー一致。JSON/CSVと他92資料/19ニュース、記事範囲外5520ファイルを保全しました。記録は `five-vla-validation-20261013.json`。全体pytest・著者コード追試・学習・走行は未実施。

### FIVE-VLA記事の公開確認

記事commit `e7a6fef8809b3bbab974be7492a49f7d3e286254` を承認済みHervararの正式Windows checkoutから既存認証で通常push。[CI](https://github.com/fis-teria/aichallenge_lite_transfuser/actions/runs/38338852582)のbuild/deployはsuccess。全39ページと3資産はHTTP200で正式生成物と完全一致。[新記事](https://fis-teria.github.io/aichallenge_lite_transfuser/articles/five-vla-2026.html)と関連4本を実配信320/390/768/1440pxで確認し、図1・3表・式・横スクロール・相互リンク・JS無効で問題なし。公開記録は `five-vla-publication-20261013.json`。対象資料/intent/queueをpublishedへ更新。ニュース20件は公開5・追加精読14・LOOP保留1、他19ニュース/92資料/他batchを保全。FIVE固有実装と独立再現は未確認。学習/制御/設定変更・学習/走行、SSH/WSL/他host/クラウドpush、API/Web書込み、別認証・remote変更は行っていません。

## Predictive Splinerの追加確認（2026-10-14）

FIVE-VLAのE2E時間表現からレーシングの相手予測へ移り、公式コード未照合と評価選別を補う比較価値で選定。R08 v1を保全し、v2と固定commit codeを同系列へ追加しました。§III–V/Algorithms1–2/TablesII–IIIとv2 PDF対応箇所、選択8ファイル、現行静的占有MPPIを確認。GP/CCMAの分岐・平均伝播・不確実性field、成功5回までの分母と方式別最大速度、計時を区別しています。

台帳95資料・66系列、年代集計82・undated13、144セル中調査済み33・未調査111、全coverage_complete=false。初稿分類と改訂日を区別し、codeのcommit日は初公表日へ代用しません。既存他92資料、ニュース20intent/batch、Roachの未移送原稿を保全。共有スナップショットは実取得7版48項目、最新9/25–10/1・message10/2で前回可視範囲と同じ。live元会話は未確認、本文/URLは転載しません。完全な版/実験commit照合、著者コード実行・独立再現・導入・学習・AWSIM/実車試験、全体pytestは未実施。学習/制御/ROS/設定変更なし。公開CIと実配信は確認後に別記録します。

### Predictive Spliner記事の公開前検査

正式生成と一致、40ページ1972リンク/asset・59固定根拠、サイト回帰29件を確認。全サイトEdgeは5幅・全40ページPC/mobile、既存操作/JS無効で成功。対象5記事を4幅で確認し、図1・3表・式・横スクロール・相互リンク、PC/mobile画像を確認。研究地図本文とカードを最新95資料/66系列へ照合して正式再生成。外部一次10 URLはHTTP200、arXiv節アンカー一致。JSON/CSV・他92台帳/年表レコード、20ニュースintent/batchと記事範囲外5526ファイルを保全。記録はpredictive-spliner-validation-20261014.json。全体pytest・著者コード追試・学習・走行は未実施。

### Predictive Spliner記事の公開確認

記事commit `969b8b741a80220be28f256e058129385337df65` をHervararの正式Windows checkoutから既存認証で通常push。[CI](https://github.com/fis-teria/aichallenge_lite_transfuser/actions/runs/38504716732)のbuild/deployはsuccess。全40ページと3資産はHTTP200で正式生成物と完全一致。[新記事](https://fis-teria.github.io/aichallenge_lite_transfuser/articles/predictive-spliner-2024.html)と関連4本を実配信320/390/768/1440pxで確認し、図1・3表・式・横スクロール・相互リンク・JS無効で問題なし。公開記録はpredictive-spliner-publication-20261014.json。対象3資料/queueをpublishedへ更新し、他92資料・20ニュースintent/batch・Roach別件原稿を保全。完全な版/実験commit照合・独立再現・導入は未確認。学習/制御/設定変更・学習/走行、SSH/WSL/他host/クラウドpush、API/Web書込み、別認証・remote変更は行っていません。

## EvolvingNavの追加確認（2026-10-15）

Predictive Splinerの相手予測からnavigationの意味記憶へ比較軸を移し、未照合だった公式codeと可視性/証拠更新の比較価値で選定。AD-Memoはcode未確認、NavSafeはv2のイベント/付録確認が残る候補として保持し、固定優先は設けません。旧v2を保全し主期間後v4と固定codeを同系列追加。v4§3–5・関連A.4/A.6/A.7/B.6/D、32頁PDF対応箇所と12選択ファイル、現行静的占有/保持経路/限定設定/既存試験を確認。HTML§3のみv2/v4空白正規化後一致、全版/PDF一致ではありません。

台帳97資料66系列、年代83・undated14。144セル中調査済み33・未調査111、全coverage_complete=false。コードcommit日は初公表日へ代用せず、他94資料/19ニュースintentとRoach別件を保全。現在/到着信念、未検出の較正と被覆ラウンド、VLMutility制限、N1–N5/事前固定N4/実機M20全64試行と転移subsetを区別。全版・実験commit・runner/データ漏えい完全監査、著者コード実行・独立再現・導入・全pytest・学習・走行は未実施。学習/制御/ROS/設定・デザインは変更なし。

共有スナップショットは実取得7版48項目、最新9/25–10/1・message10/2で前回可視範囲と同じ。元liveは未確認、共有本文/URLを転載しません。公開CI/実配信は確認後に別記録します。

### EvolvingNav記事の公開前検査

正式生成と一致、41ページ2053リンク/asset・62固定根拠、サイト回帰29件を確認。公式Edgeは5幅・全41ページPC/mobile・既存操作/JS無効で成功。対象6記事を4幅、図1・3表・式・横スクロール・相互リンクで確認しPC/mobile画像を目視確認。外部一次13 URLはHTTP200、arXiv節アンカー一致。研究地図144セルをJSON照合し、従来navigation2026の表示9対台帳10を補正してv4追加後11資料へ更新。台帳97/66、年代83/undated14、JSON/CSVと他94資料/19ニュースintent、記事範囲外5527ファイルを保全。記録はevolvingnav-validation-20261015.json。全体pytest・著者コード追試・導入・学習・走行は未実施。

### EvolvingNav記事の公開確認

記事commit `fe68047010dadcebad254909d0367f7083657095` をHervarar正式Windows checkoutから既存認証で通常push。[CI](https://github.com/fis-teria/aichallenge_lite_transfuser/actions/runs/38673481283)のbuild/deployはsuccess。全41ページと3資産はHTTP200・正式生成物と完全一致。[新記事](https://fis-teria.github.io/aichallenge_lite_transfuser/articles/evolvingnav-2026.html)と関連5本を実配信320/390/768/1440pxで確認し、図1・3表・式・横スクロール・相互リンク・JS無効は問題なし。公開記録はevolvingnav-publication-20261015.json。対象3資料/intent/queue/batchの公開状態を更新。ニュース20件は公開6・追加精読13・LOOP保留1、他94資料/19intent/他batchを保全。全版/実験対応・コード実行・独立再現・導入は未実施。学習/制御/設定変更・学習/走行、SSH/WSL/他host/クラウドpush、API/Web書込み、別認証・remote変更は行っていません。

## NavSafe/MILEの独立再執筆（2026-10-26）

現地の公式一次資料から2記事を新規著述しました。旧稿の回収/コピーや未取得内容の推測再現ではありません。既存NavSafev1を保全してv2と公式code、既存MILEv2へ公式codeを追加。同じ研究系列を保ち100資料66系列、年代84/undated16、144セル33調査済111未調査/全complete=false。MILEのlatent/policy/再帰/想像と出力契約、NavSafeのイベント別成功/違反/分母/資源を確認。固定codeのcompletedはGOAL_REACHED/HOLD_SATISFIED両方を含むため、文書の狭い説明だけで停止成功を否定しません。

全版/recipe/checkpoint/実験対応、著者コード実行・独立追試・導入・全pytest・学習/走行は未実施。学習/制御/ROS/設定・デザインは変更なし。他95既存資料/19ニュースintentを保全。対象8記事のうち、ローカル本文のないRoach/SmoothMPPI/DynaBARN/VAD/CoVO-MPC/MAPとFormulaZero追補は保留しています。公開CI/配信は確認後に別記録します。

### NavSafe/MILE独立再執筆の公開前検査

正式生成・一致、43ページ2181リンク/asset・65固定根拠、サイト回帰29件を確認。公式Edge5幅・全43ページPC/mobile、対象7記事×4幅・図2/表6/数式/横スクロール/相互リンク/JS無効は成功、PC/mobile画像を目視確認。外部一次17URLはHTTP200、節アンカー一致。台帳100資料66系列/年代84/undated16、144セルの表示とJSON/CSVを照合。他95既存資料/19ニュースintent、記事範囲外5531ファイルを保全。旧稿を受け取ったものとして扱わず、公式資料から現地で新規著述した2記事だけの検査。保留6記事とFormulaZero追補は変更していません。記録はindependent-rewrite-validation-20261026.json。

### NavSafe/MILE独立再執筆の公開確認

記事commit `0e22feb4d1d5db2e46bb9e98b3d4da6410529fb0` をHervarar正式Windows checkoutから既存認証で通常push。[対象CI](https://github.com/fis-teria/aichallenge_lite_transfuser/actions/runs/40251970164)のbuild/deployはsuccess。全43ページ+3資産はHTTP200/正式生成物完全一致。[NavSafe](https://fis-teria.github.io/aichallenge_lite_transfuser/articles/navsafe-infinity-2026.html)・[MILE](https://fis-teria.github.io/aichallenge_lite_transfuser/articles/mile-2022.html)と関連5本を実配信4幅で検査し、図2/表6/数式/横スクロール/相互リンク/JS無効は成功。記録はindependent-rewrite-publication-20261026.json。両記事は公式一次資料から現地で新規に著述した独立再執筆版で、旧稿の回収/コピー/推測再現ではありません。台帳100資料66系列、年代84/undated16。ニュースは公開7/追加精読12/LOOP保留1、他95資料/19intent/他batchを保全。保留6記事とFormulaZero追補は変更なし。全版/実験対応・著者コード実行・独立再現・導入・全pytest・学習/走行は未実施。remote/認証・学習/制御/ROS/設定・デザイン変更、SSH/WSL/他host/クラウドpush、API/Web書込みは行っていません。

## 5記事とFormulaZero追補の独立再執筆（2026-10-26）

今回の公開依頼に基づきRoach、VAD、Smooth MPPI、CoVO-MPC、MAPの5記事と既存レーシング概説のFormulaZero追補を公式一次資料からHervarar上で新規著述。旧クラウド原稿の回収・コピー・推測復元ではなく、未取得旧稿は置換・削除せず保全。DynaBARNは著者大学公開原典のTLS証明書検証に失敗し、検証無効化や別経路迂回をせず保留。Libraryの正規転送障害も未解決です。

既存6資料を限定増補し、MAP v2の版日付を補正して保全、v4と5公式codeを同じ系列へ追加。最新JSONから106資料・66系列、年代85・undated21、144セル中33調査済/111未調査・全coverage_complete=false。対象以外94資料と全20ニュースintentを保全。コードcommit日を初公表年へ代用しません。記事は教師/学生、開/閉ループ、入力増分と経路係数、無限サンプルの最適化解析と制御保証、同定LUTと既存速度補償を分離。全証明/実験対応・著者コード実行・独立再現・現行導入・全pytest・学習/走行は未実施。学習/制御/ROS/設定・デザインは変更なし。正式サイト検証と公開CI/配信は確認後に記録します。

### 5記事・FormulaZero追補の公開前検査

正式生成・一致、48ページ2469リンク/asset・70固定根拠、サイト回帰29件を確認。公式Edge5幅・全48ページPC/mobile、対象14記事×4幅・図5/表15/数式/横スクロール/相互リンク/JS無効は成功。PC/mobileの図表と式を目視確認し、相互リンク検査で見つけたCoVOから既存SLAM MPPIへの戻りリンクを補正後に再検査。外部一次33URLはHTTP200。台帳106資料66系列/年代85/undated21と研究地図144セル/JSON/CSVを照合し、他94資料・全20ニュースintent・記事範囲外5533ファイルを保全。DynaBARNは原典TLS証明書検証失敗で保留、Library原稿回収障害は未解決のまま保全。記録はremaining-rewrite-validation-20261026.json。

### 5記事・FormulaZero追補の公開確認

記事commit `f6da62ef079b251a6a7cf972b08e9c5f41d7619a` をHervarar正式Windows checkoutから既存認証で通常push。[対象CI](https://github.com/fis-teria/aichallenge_lite_transfuser/actions/runs/40255218981)のbuild/deployはsuccess。全48ページ+3資産はHTTP200・正式生成物完全一致。対象14記事を実配信4幅で検査し、図5/表15/数式/横スクロール/相互リンク/JS無効は成功。記録はremaining-rewrite-publication-20261026.json。5記事と追補は公式一次資料から独立再執筆し、旧稿の回収/コピー/推測復元はしていません。対象8記事中NavSafe/MILEと合わせ7記事+追補が公開済み。DynaBARNは原典のTLS証明書検証失敗、Library原稿回収障害は未解決。証明書検証無効化・別経路転送・旧稿置換/削除なし。106資料66系列、年代85/undated21、他94資料・全20ニュースintentを保全。全証明/実験対応・著者コード実行・独立再現・導入・全pytest・学習/走行は未実施。学習/制御/ROS/設定・デザイン、remote/認証変更、SSH/WSL/他host/クラウドpush、API/Web書込みは行っていません。
