# Project research site

プロジェクトの現状、構成、検証結果、技術スタック、参考論文の要約をまとめた静的HTMLサイト。
外部フォント、CDN、アクセス解析、npmビルドは不要。本文はJavaScriptなしでも読める。

- 公開URL: https://fis-teria.github.io/aichallenge_lite_transfuser/
- HTML: `docs/site/index.html`
- CSS / JS / アイコン: `docs/site/assets/`
- 配信対象: **`docs/site/`のみ**。リポジトリ全体や実験artifactをPagesへコピーしない。
- 公開workflow: `.github/workflows/project-site.yml`

## ローカル表示

`docs/site/index.html`をブラウザで直接開くか、リポジトリ直下で実行する。

```bash
python -m http.server 8765 --bind 127.0.0.1 --directory docs/site
# http://127.0.0.1:8765/
```

## 検証

```bash
python tools/check_project_site.py
```

HTMLのID、内部アンカー、ローカルasset、外部依存の不在、commit固定の根拠リンクを確認する。
全履歴のあるcheckoutで実行する。外部サイトの稼働や記述内容の正しさを自動保証するものではない。

UIを変更した場合は、学習環境とは別の任意venvにPlaywrightを入れてブラウザsmokeも実行する。

```bash
python -m pip install playwright
python -m playwright install chromium
python tools/check_project_site_browser.py --screenshots /path/to/local/site-checks
# Windowsで導入済みEdgeを使う場合、ブラウザ追加インストールの代わりに:
python tools/check_project_site_browser.py --channel msedge
# HTTP配信も確認する場合:
python tools/check_project_site_browser.py --url http://127.0.0.1:8765/ --channel msedge
```

1440 / 1024 / 768 / 390 / 320 pxで横はみ出し、論文の分野フィルタ・検索・0件表示、
キーボードでの開閉、目次移動、JavaScriptエラー、JS無効時の閲覧を確認する。
スクリーンショットはローカルに保存し、Gitへ追加しない。
Python/モデル/ROSコードを変更した場合は既定のWSL同期とlock下で`pytest -q`も行う。

## 更新方針

1. 公開mainの対象commitと根拠資料を確認する。別ブランチの機能は明記する。
2. HTMLの更新日・基準commit・根拠リンクを更新する。根拠は完全なcommit SHAへ固定する。
3. 数値には試験日、条件、コード、限界を添える。設定上限を実測値として扱わない。
4. 実装済み、単体テスト済み、限定AWSIM試験、未検証を分ける。
5. 論文は一次資料を確認し、要約と本プロジェクトへの解釈を分ける。
6. 上記のリンク検査とブラウザ確認を実行し、Windowsからcommit / pushする。

初版のコード基準は`e27e3caae00d721f22e750a06fe32e9eccbec193`。
DINOv3開発ブランチの記録は`0c63f67e1cf58019c976715b08398ff1d6ee5c30`。
既存資料を整理したサイトであり、新たなAWSIM試験や学習を実行した報告ではない。
実験の生ログ、動画、重みには別保管物がある。根拠資料の公開と全実験の完全再現は異なる。

## GitHub Pages

GitHub Settings → Pages → Build and deploymentを**GitHub Actions**へ設定する。
mainへのサイト関連変更のpush、またはActionsの`Project research site`手動実行で配信する。
workflowはリンク検査後に`docs/site/`だけをアップロードする。
配信先の権限は`github-pages`環境と`pages: write` / `id-token: write`に限定する。
公開後はActionsの成功と公開URLの内容を確認する。

公式workflow参考: https://github.com/actions/starter-workflows/blob/main/pages/static.yml
