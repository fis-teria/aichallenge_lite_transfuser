# 文献調査管理データ

対象期間は2018-10-02〜2026-10-02。期間前の基礎は別枠です。

- `taxonomy.json`: 分類、対象期間、調査状態、研究系列の重複除去方針
- `coverage-matrix.{json,csv}`: 時代 × 領域 × 資料種別の全108セル。`coverage_complete` は全セル `false`
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

追加後はCSV、年表、108セルの資料ID・件数をJSONから更新し、全資料IDの一意性、URL、研究系列の重複、記事slugの存在、資料数とセル件数の一致を検査します。確認済み資料の追加だけでは `coverage_complete` を `true` にしません。版・評価の不足は次の調査キューへ残します。公開の `research-map` 記事には確認範囲と関連する解説のみ反映します。

記事の安定参照は `research-map.html`、`e2e-field-overview.html`、`racing-overview-2018-2026.html`、`mpc-mppi-overview.html`。既存のTransFuser・MPC記事のslugを変更せず、関連記事として相互参照します。

原稿の受け渡し用manifestと作業補助スクリプトは公開記事や保守スクリプトではありません。初回反映の対象はこの `docs/research/`、記事編集元、記事メタデータと生成された公開HTMLです。

公表年を確認できない資料は `year: null`、`classification_year: null`、`era: undated` です。これらは108セルの時代別件数に含めず、台帳と年表の末尾に保全します。そのため全台帳の件数と時代別セルの件数合計は一致しない場合があります。差分は `undated` の件数と一致する必要があります。

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
