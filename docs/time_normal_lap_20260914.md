# 通常 E2E 完走試験（2026-09-14）

## 主な合格条件

ユーザーの最新指定に従い、通常の初期状態から E2E 予測軌道を Pure Pursuit で追従し、AWSIM の周回判定で 1 周を完了することを主な合格条件とする。目標速度は固定 5 km/h。通常の Autoware RViz に未加工の E2E 予測経路を表示する。教師制御による発進・走行介入は行わない。

横ずれ 5 cm・向き 2 度以内への復帰は補助評価であり、通常完走の必須条件ではない。先の復帰試験結果は `time_recovery_expanded_awsim_20260914.md` に保全する。周回完了後の停止確認、センサ・操舵・停止領域監視は従来のまま記録する。

## 発進停止の修正

`stopping_preview_extended_v1` はまず従来と同じ距離範囲 `[d, d+0.5] m` の原予測折れ線を探索する。`d=max(1, 0.4+0.5v+v²/2)`、`v` は実測 m/s。操舵可能点が見つからなかった場合のみ `[d, d+1.0] m` を再探索する。最低停止余裕・物理タイヤ角上限 0.3 rad・元の時刻対応・経路の終端を維持する。範囲外への経路外挿や平滑化は行わない。通常発進に限らず、同じ条件を満たす走行中の参照にも適用する。

保存された通常発進失敗記録の WSL 調査では、再学習モデルの候補点欠損 103 件すべてで、最大探索距離を 1.5 m から 2 m へ広げると候補点が見つかった。これは候補点探索の診断であり、実際の発進や完走を証明するものではない。最初の失敗入力を `tests/fixtures/time_path/expanded_startup_band.json` に原値と出典 SHA256 付きで保持した。

既存の `time_path_expanded_5kmh_20260914.json` から探索方針だけ変更した `configs/control/time_path_lap_candidate_20260914.json` を使う。再学習済み checkpoint SHA256 は `7ec445b6ab955e76d0d71efbd8f2f1dd3066ebe365516df38df8cf18adb56f44`。

## 検証・実行

Windows でコミット後、`tools/sync_to_wsl.ps1` で同期する。WSL ネイティブ checkout で:

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 .venv/bin/python -m pytest -q
```

実行先は `graneple@192.168.3.10` の新しい専用 deployment `~/e2e_autonomous/time_normal_lap_20260914`。ソースと重みのハッシュを検証し、隔離した ROS スモークで予測経路 → PP → 操舵応答補償 → 操舵変換 → 停止領域監視まで確認してから実車両のない AWSIM を起動する。

```bash
timeout --signal=TERM --kill-after=10s 710s python3 \
  <deployment>/source_<commit>/tools/run_time_path_awsim_trial.py \
  --deployment <deployment> --run-id codex-time-lap-candidate01 --display :1 \
  --config configs/control/time_path_lap_candidate_20260914.json
```

予算は通常初期状態からの 1 試行、走行 600 sim s / 600 wall s、外側 720 wall s。外乱付与なし。結果と停止原因を確認せずに反復しない。生ログを WSL へハッシュ検証付きで転送し、次で評価する:

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  tools/evaluate_time_awsim_trial.py --run <WSL raw run> --output <WSL evaluation>
```

実行結果は検証・AWSIM 試験の終了後に追記する。
