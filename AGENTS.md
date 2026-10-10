# AGENTS.md

## Project Goal

自動運転AIチャレンジE2E部門向けに、Camera + 2D LiDAR + ego stateから
将来waypoint、目標速度、停止確率、行動モードを予測するTransFuser風モデルを構築する。
最終目標は、完走、障害物回避、回避不能時停止である。

## Architectural Constraints

- 推論入力とteacher/debug-only情報を厳密に分離する。
- 主出力はfuture waypoints、target speed、stop probabilityとする。
- 直接steering/accelerationはbaselineまたは補助Headに限定する。
- モデル外にSafety Supervisorを置き、センサtimeout、停止距離、NaN、異常出力を監視する。
- 変更は小さく保ち、1タスク1目的とする。
- 大規模リファクタは、既存テストが通る状態を維持して段階的に行う。
- ROS依存コードとPyTorch/数学ロジックを分離し、後者は通常のpytestで検証できるようにする。

## Coding Rules

- Python 3.10以上。
- 型ヒントを付ける。
- パス、topic、shape、単位を暗黙にしない。
- 角度はrad、速度はm/s、加速度はm/s^2、時間はsまたは明示したmsを使う。
- 入力tensor shapeをdocstringとassertで確認する。
- LiDARのNaN/inf/範囲外を前処理で除去する。
- データsplitはrun/scenario単位で行う。frameランダムsplitを既定にしない。
- エラーを握り潰さない。入力欠損は明示的に報告する。
- 新規ロジックにはunit testまたはsmoke testを追加する。

## Done Definition

### External review policy (2026-09-09 user revision)

- 外部GPTレビューは任意とし、実装・テスト・適用・AWSIM試験の必須条件にしない。
- ユーザーが当該タスクで明示依頼しない限り、レビュー送信・キュー登録・搬送agent起動・待機を自動実行しない。
- 過去資料の外部レビュー待ちやqueueの占有を、通常作業の停止条件として引き継がない。
- 過去のレビュー記録・他作業のclaim/lockは保全する。テスト、安全確認、実行許可、有限予算は従来どおり維持する。

各タスクは次を満たした時に完了とする。

1. 実行コマンドがREADMEまたは該当docsに記載されている。
2. 既存の`pytest -q`が通る。
3. 新規処理のshape、単位、例外条件がテストされている。
4. 変更点と未解決事項が明記されている。
5. 大きなデータ・重み・rosbagをGitへ追加していない。
6. ROSコードの場合、公式環境で未確認ならその旨を明記している。

## Windows / WSL Workflow

- Codexの編集元・Git正本は`E:\workspace\e2e_lite_transfuser`とする。
- 本学習・Linux/CUDA/ROS検証は`/home/thistle/e2e_autonomous/e2e_lite_transfuser`で行う。
- `/mnt/e`上では学習しない。`.venv`、datasets、runs、checkpoint、rosbag、ROS build出力はWSL側に保持する。
- Windows側で変更をコミットしてから`tools/sync_to_wsl.ps1`で同一コミットをWSLへ同期する。強制reset、cleanup、`rsync --delete`で同期しない。
- WSLおよびSSH接続先から`git push`しない。pushは必ずWindowsのローカルPC側checkoutから行う。
- 同期前にWindows/WSL両方のGit状態、実行中の学習プロセス、対象commit SHAを確認する。
- WSLで学習・テスト・ROS検証を実行するときは`tools/with_wsl_training_lock.sh`を通し、同期と同じworktree lockを保持する。
- 詳細は`docs/windows_codex_wsl_training_workflow.md`を参照する。

## Project site maintenance (2026-09-27 user policy)

- 2026-10-10ユーザー指定: 記事の編集・生成・commit/push・配信確認はHervararのWindows環境で行う。
  記事用正式checkoutは `C:\Users\euPHo\Documents\Codex\2026-10-06\task\migration-20261006\e2e`。
  記事以外の学習・制御・認証・実行環境の権限をこの指定から拡張しない。

- 変更・更新を行う各タスクには、公開サイトの関連記事の確認・更新を含める。
  モデル、入力/教師契約、データ処理、制御、安全監視、依存環境、起動手順、検証結果が
  変わった場合は、原則として同じ変更単位で記事本文とメタデータを更新する。
- トップページへ長文や作業ログを追記し続けない。1テーマ1記事を基本に分散する。
  構成・技術スタック等の継続資料は同じ記事を更新し、別条件の試験・新しい実測結果は
  独立した検証記事へ追加する。論文要約は1本1記事。過去の失敗・制限・根拠を消さない。
- 編集元は `docs/site_src/articles/*.html` と `docs/site_src/articles.json`。
  共通レイアウト・CSS・JSも `docs/site_src/` で編集する。`docs/site/` の生成HTMLを直接修正しない。
- 記事には変更点、確認日、実装/試験の条件、観測結果、未確認事項、根拠を記載する。
  設定値と実測値、単体テストとAWSIM、開発ブランチと公開mainを区別する。
  新しいコードの存在だけで既存の走行結果を最新版の実績へ読み替えない。
- `python tools/build_project_site.py` で一覧・関連記事・公開HTMLを再生成し、
  `python tools/build_project_site.py --check` と `python tools/check_project_site.py` を実行する。
  UI/ナビゲーション変更では `tools/check_project_site_browser.py` も実行する。
- CIはプロジェクト差分に対する記事本文の更新を確認する。読者向け説明に影響しない場合は
  `.github/site-update-note.json` を当該変更で更新し、理由と対象ファイルを正確に記録する。
  更新日だけの変更や生成HTMLだけの変更を、記事更新の代わりにしない。
- 記事の追加・公開・検査方法は `docs/site/README.md` を参照する。作業ブランチにサイトが
  まだない場合は公開mainを確認し、Windowsの分離checkoutで関連記事を更新する。
  無関係な未コミット変更を含めず、Gitのcommit/pushは当該依頼の許可範囲で行う。
  mainへ反映された更新はGitHub Pagesへ自動公開し、配信結果も確認する。

## Work Order

1. dataset audit
2. canonical dataset converter
3. LiDAR preprocessing
4. PyTorch Dataset
5. LiDAR-only baseline
6. Safety Supervisor
7. Camera-only baseline
8. Late fusion
9. Transformer fusion
10. ROS inference integration
11. closed-loop evaluation
12. BEV/temporal/multi-hypothesis extensions
