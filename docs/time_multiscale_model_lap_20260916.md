# 多段階の復帰データで再学習したモデルのAWSIM試験

2026-09-16。実行先は `graneple@192.168.3.10`、解析・評価はnative WSL。
AWSIM本体、シーン、車両・センサファイルは変更せず、全ファイルを開始前後にhash照合する。
通常RVizにE2Eの生予測 `/visualization/time_path/raw_path` を表示する。

## 試験条件

- 12・20・40・60cmの復帰データで再学習したepoch3モデルを使用する。
- 元checkpointはWSL `runs/time_recovery_multiscale_20260916/training/best.pt`、SHA256は
  `685f8b8f9936ab7e272be12616982374fd8a8d61fbe4a304b25475dc2a206ba8`。
- 停止状態からE2Eだけで発進する通常走行1回。教師運転や外乱は追加しない。
- 固定目標5km/h、Pure Pursuit、`stopping_preview_extended_v1`、操舵応答補償、標準の停止領域監視を維持する。
- 合格は公式Judgeによる順序付き区間通過と1周完了、その後の停止確認。
- 上限は走行600秒（sim/wall各々）、外側710秒＋終了猶予10秒。既存監視・進捗停止でも終了する。
- 失敗を隠す無条件の再試行や、この走行記録の学習への自動追加は行わない。

## 実行前に見つかったメタデータ不具合

元checkpointは `TEACHER_RUNTIME_CONTRACT_MISMATCH` で実行側に拒否された。
多段階復帰の学習ツールが、元cacheにある `teacher_manifest.contract` を保存していなかった。
保存処理にcontract転記を追加し、今後のcheckpointがそのまま実行側へ読み込める回帰テストを追加した。
既存exportツールをこの既知形式にも対応させ、元cacheの全hash・split・入力設定を照合して別ファイルへexportする。
元checkpointは保持し、全モデルstateと既存検証入力の予測が変わらないことを確認する。
モデル重み・学習条件・runtimeの拒否条件は変更せず、再学習は不要。

## 実行コマンド

Windowsでcommitして公式同期を行い、native WSL repoで共有worktree lockを取得して実行する。

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  tools/export_time_recovery_runtime_checkpoint.py \
  --checkpoint ../runs/time_recovery_multiscale_20260916/training/best.pt \
  --checkpoint-sha256 685f8b8f9936ab7e272be12616982374fd8a8d61fbe4a304b25475dc2a206ba8 \
  --cache ../datasets/cache/time_recovery_multiscale_20260916 \
  --output ../runs/time_multiscale_model_lap_20260916/multiscale_runtime.pt

# .10: source、installed Python、checkpointを照合し、公式イメージ内ROS smoke成功後:
timeout --signal=TERM --kill-after=10s 710s python3 \
  <deployment>/<source>/tools/run_time_path_awsim_trial.py \
  --deployment <deployment> --run-id codex-time-multiscale-lap01 --display :0 \
  --config configs/control/time_path_multiscale_lap_20260916.json

# rawの転送後、native WSLで全hashを照合して評価:
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  tools/evaluate_time_awsim_trial.py --run <verified_raw_run> --output <evaluation>
```

走行結果と証拠は完了後に追記する。

## 配置前の確認結果

source `def95a93cb406688b2af737aa75f38c4701c6bb1` のnative WSL全pytestは **2,706 passed / 4 skipped**、102.39秒。
元cacheの全hashを照合し、215個のモデルstateは完全一致。既存validationから選んだ12入力についてCUDA・float32予測も完全一致した。
実行用checkpointは `runs/time_multiscale_model_lap_20260916/multiscale_runtime.pt`、SHA256 `1d36d36d02116a332489dab72e8d2c655b1daf24e34bb1ec5cd33347bb3b61a0`。
設定の差分は従来通常走行設定に対してこのcheckpoint SHAのみ。元の学習済み重みは変更していない。
