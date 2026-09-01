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
