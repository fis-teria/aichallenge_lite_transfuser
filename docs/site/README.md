# Project research site

公開URL: https://fis-teria.github.io/aichallenge_lite_transfuser/

トップは全体への入口です。構成・技術スタック等の継続資料、試験ごとの検証記録、
1本ずつの論文ノート、開発・運用記録を独立した記事に分けています。
サイドメニューと狭い画面の左ドロワーを維持し、旧URLの`#stack`等も新記事へ転送します。

## 編集元と公開物

| 場所 | 役割 |
| --- | --- |
| `docs/site_src/articles/<slug>.html` | 1記事の本文。ここを編集する |
| `docs/site_src/articles.json` | タイトル、分類、概要、公開日、更新日、状態、関連記事 |
| `docs/site_src/home.html` | トップの紹介と案内。長文記事は増やさない |
| `docs/site_src/layout.html` | 全ページ共通のヘッダー・サイドメニュー |
| `docs/site_src/assets/` | CSS・JavaScript・アイコンの編集元 |
| `docs/site/` | 生成された公開HTML。README以外は直接編集しない |

Python標準ライブラリで生成します。npmビルド・外部フォント・CDN・アクセス解析は不要。
本文と記事間リンクはJavaScriptなしでも読めます。検索、狭い画面のドロワー、旧hash転送はJSで補助します。

## 更新を作業の完了条件にする

モデル、データ、入力契約、制御、安全監視、依存、起動方法、評価結果が変わったら、
同じタスク内で関連記事を更新します。公開main未収録の内容は開発ブランチの進捗と明示します。

- 継続資料: 同じ記事本文と`updated`を更新し、現在の説明を保つ。
- 新しい試験・実測値: 条件と日付が分かる別の記事を追加。旧結果と失敗・制限は残す。
- 論文: 1本1記事。一次資料の要約とローカル実装への解釈を区別する。
- トップ: 読み始めの案内と最近の記事への入口。作業ログや長文記事を集約し続けない。

記事に変更内容、確認日、コード/モデル/設定、検証範囲、未確認事項、根拠を記載します。
根拠のGitHubリンクは可能な限り完全なcommit SHAへ固定し、`data-source`にリポジトリ内パスを付けます。
実装・単体テスト・限定AWSIM試験・一般化や完走保証を同一視しません。

## 新しい記事を作る

リポジトリ直下から実行します。PowerShellでは1行で実行できます。

```bash
python tools/new_site_article.py --slug example-run --title "新しい検証の記録" --summary "対象と目的を短く説明" --category report --status "検証中" --date 2026-09-27
```

分類は`guide` / `report` / `paper` / `development`。
論文には`--topic fusion`、`learning`、`control`のいずれかを追加。
必要なら`--nav stack`等でサイドメニューの所属を指定します。
作成後、本文の項目を実際の記録で埋め、`ARTICLE_DRAFT`マーカーを削除してください。
未完成記事・日付の逆転・slug重複・存在しない関連記事はビルド時に拒否します。
既存slugを指定した上書きも拒否します。

```bash
python tools/build_project_site.py
python tools/build_project_site.py --check
python tools/check_project_site.py
python -m unittest discover -s tests/site -p 'test_*.py'
```

記事一覧・最近の記事・関連記事・公開HTMLが生成されます。編集元と生成物を一緒にcommitします。
URLを維持するため、不要になった記事もいきなり削除せず、位置づけと後継記事を本文へ記載します。
公開済みHTMLが登録から消えた場合は自動削除せず、ビルドを停止します。

## 変更と記事の対応を検査する

commit後、対象差分の基準SHAを指定します。

```bash
python tools/check_site_update.py --base <変更前commit> --head HEAD
```

mainへのpushとmain向けPRでは同じ検査をCIで実行します。プロジェクトのソース、設定、
仕様、起動ツール、依存、テスト等が変わったとき、記事本文の更新が必要です。
`articles.json`の更新日だけの変更や、公開HTMLだけの変更では通りません。

読者向け説明に影響しない変更は、当該変更で`.github/site-update-note.json`を作成/更新します。
`reviewed_paths`には検査対象になった全パスを正確に列挙し、`reason`に具体的な理由を20文字以上で記載します。
過去の記録を変更せずに使い回したり、一部のパスだけ説明したりすることはできません。

```json
{
  "schema_version": 1,
  "reviewed_paths": ["tests/test_example.py"],
  "reason": "既存テストの変数名のみを変更し、公開仕様・実行手順・検証結果には変更がないため。"
}
```

CIは記事本文の変更有無とファイル範囲を検査します。記事が変更内容を正確に説明しているかは
作業者が確認します。文章・実験結果の自動執筆や、GitHubの保護ルールによる直接push禁止は設定していません。
mainの検査が失敗した場合はPagesへの新しい配信を行わず、前回の公開状態を維持します。

## 表示とブラウザ確認

`docs/site/index.html`を直接開くか、以下を実行します。

```bash
python -m http.server 8765 --bind 127.0.0.1 --directory docs/site
# http://127.0.0.1:8765/
```

UI・共通レイアウト・導線を変更したら、任意の検証用venvにPlaywrightを用意して確認します。

```bash
python -m pip install playwright
python -m playwright install chromium
python tools/check_project_site_browser.py --screenshots /path/to/local/site-checks
# 導入済みWindows Edgeの場合は追加ブラウザの代わりに:
python tools/check_project_site_browser.py --channel msedge
```

5画面幅、記事間移動、論文検索、ドロワー開閉、キーボード、旧hash URL、JS無効時を確認します。
スクリーンショットはローカル保存。学習/ROSロジックを変えた場合は既定のWSL同期・lockとpytestも実行します。

## 公開と履歴

GitHub Pagesの配信対象は`docs/site/`だけです。編集元・重み・生ログ・datasetを配信artifactに含めません。
mainへ反映すると`.github/workflows/project-site.yml`が生成物・リンク・記事対応を検査して配信します。
公開後はActionsの成功、HTTP応答、公開されたHTMLを確認します。pushはWindowsのローカルcheckoutから行います。

初版の実装資料は`e27e3caae00d721f22e750a06fe32e9eccbec193`、DINOv3開発記録は
`0c63f67e1cf58019c976715b08398ff1d6ee5c30`を確認した記事です。記事が更新されても、
過去の実験コードと結果を現在の実装の実績に読み替えません。

## 初版の確認記録（2026-09-27）

- `60417a48fcd69ec3b230f6514d01bf1dd8497b76`をWindowsでcommitし、専用のnative WSLコピーへ
  既定syncスクリプトで同期。lock下の`pytest -q`は**3324 passed / 4 skipped**（143.13秒）。
  4 skipはOSQP、schema validator関連2件、任意の公式package不足。
- 最初の実行はGit管理外の旧checkpoint 2ファイルがないため5件失敗した。
  既存WSLから専用コピーへ配置してSHA-256一致を確認後、全体を再実行して通過。
  重みをGitへ追加したり、テストをskipへ変更したりしていない。
- Edge headlessで5画面幅、file URLとHTTP配信、JS無効時の閲覧を確認。
  PC・スマホのスクリーンショットも目視確認。検索・目次・開閉とリソース読込は正常。
- ローカルasset / 内部リンク44件、commit固定の根拠13種類を検査。
  後続変更は公開workflowの権限設定とこの記録のみ。学習・ROSのコード差分はない。
- サイトの公開はAWSIM試験や走行性能の追加検証ではない。

### サイドメニューへの統一（2026-09-27）

幅900 px以下の横並び目次を廃止し、左から開く縦メニューへ変更。
Edge headlessの5画面幅で、メニューの縦配置、項目選択後の閉鎖、Escape・背景・閉じるボタン、
Tabの循環、フォーカス復帰、背景の操作抑止、画面幅変更時の解除を確認した。
静的リンク検査・JavaScript構文検査も通過。スマホの開閉状態を画像で目視確認した。
変更はサイトとブラウザ検証補助のみ。Pythonモデル・ROS・既存pytest対象の差分はなく、
全pytestは上記の通過記録を引き継ぎ、今回の変更にはブラウザsmokeを実行した。

### 記事分割と継続更新の仕組み（2026-09-27）

- `e4de53844b1ee3a88f94169a61ef15eb2d46fe50`で15記事・19ページへ分割。
  記事生成、追加、更新漏れ検査と29件の回帰テストを追加した。
- Windowsでcommitした同一SHAを既定syncスクリプトで専用native WSLへ同期し、
  `tools/with_wsl_training_lock.sh`下の`pytest -q`は**3353 passed / 4 skipped**（120.50秒）。
  skipの理由は初版と同じ。既存の検証用checkpointを使用し、追加学習・AWSIM試験は行っていない。
- 生成物一致検査、19ページのリンク/asset 482件、commit固定根拠13種類、
  実commit間のサイト更新対応検査を通過した。
- Edge headlessの5画面幅でサイドメニュー・検索・記事間移動・キーボード操作を確認。
  全19ページをPC/スマホ幅で確認し、横はみ出し、リソース欠損、JavaScriptエラーはなし。
  旧hash URLの転送とJS無効時の閲覧を確認し、スクリーンショットも目視確認した。
- この確認記録の追記はREADMEのみ。記事・生成ツール・モデル・ROSのコードに追加変更はない。

### DINOv3とResNet18の図解記事（2026-09-27）

- `dinov3-resnet-guide`を追加。5つの図で画像encoderの役割、CNN/ViTの構造、
  事前学習と運転用学習、Gram anchoring、ローカルadapterの接続を説明する。
  公式論文・公式実装を出典とし、期待する利点と未検証の走行効果を区別した。
- トップページ、技術スタック、DINOv3論文要約と開発記事から移動できるようにした。
  更新は記事本文・メタデータ・表示用CSS・生成HTMLのみ。学習とROSの実装変更はない。
- 上記のサイト生成・一致検査・リンク検査・29件のサイト回帰テストを実行し、通過。
  17記事・21ページ、内部リンク/asset 601件、commit固定の根拠24種類を確認した。
- Edge headlessの既存ブラウザ検査が通過。5画面幅、全21ページのPC/スマホ表示、
  検索、メニュー、キーボード操作、旧hash URL、JS無効時を確認した。
  新記事の5つの図とリンクも5画面幅で追加確認し、PC/スマホの図を目視確認した。
- 全pytestは上記`e4de538`の通過記録を引き継ぎ、今回の文書変更では再実行していない。
  公式学習済みDINOv3の比較学習・推論遅延・AWSIMでの効果は今回の確認対象外。
