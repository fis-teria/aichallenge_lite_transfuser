# 復帰データ追加後の再学習（2026-09-15）

状態: 実行準備。学習・比較結果は完了後に追記する。

既存の expanded uniform モデルに未使用だった r50/r51 と、確認済みランダム外乱収集 r64/r65 を統合する。学習は Windows の確定コミットを同期した native WSL で実行する。

## 固定したデータと比較条件

| 区分 | 追加前 | 今回の追加 | 統合後 |
|---|---:|---:|---:|
| 学習 unique anchors | 37,678 | r50:92、r51:93、r64:273 | 38,136 |
| うち復帰 | 952 | 458 | 1,410 |
| モデル選択用 validation | 12,346 | 0 | 12,346 |
| 選択後の診断用 validation | r46/r47:186 | r65:243 | 429 |

1 anchor は現在の入力履歴と、0.1 s 刻み・3 s・30 点の将来 XY 教師。連続フレームは独立した走行数ではない。r65 は学習に使用しない別走行 1 本・3 イベントであり、広い条件への汎化や完走率を示す試験ではない。

- 初期重みは以前と同じ command-off epoch10。既学習モデルへの追加 epoch による比較の混同を避け、同じ初期重みから再学習する。
- モデル、損失、AdamW、seed 42、float32、batch 32、lr 3e-5、3 epochs を保持。
- 45,646 presentations/epoch（通常 36,726、復帰 8,920）、計 4,281 updates。通常 anchor の位置と順序を保持し、復帰枠を 1,410 unique anchors に均等配分（6 または 7 回）。
- best は以前と同じ 6 validation runs の 3 s XY run-macro で選択。r46/r47、r65 は選択後の診断専用。
- nominal test4 と r48/r49 は封印を維持。r62 診断走行、未収集 r52–r61/r63、旧 V4 の未監査教師を含めない。
- 初期 validation の厳密一致、以前の cache 160 files の同一性、学習依存コードと sampler の同一性を確認する。
- 新規 AWSIM 試験の予算は 0。本タスクは再学習と offline 比較まで。

機械可読の全パス・ハッシュ・件数は `configs/time_path_p1/recovery_random_update_20260915.json` に固定。
旧モデルは `runs/time_recovery_expanded_training_20260914/best.pt`（SHA256 `7ec445b6ab955e76d0d71efbd8f2f1dd3066ebe365516df38df8cf18adb56f44`）。新規 cache と重みは既存出力と別ディレクトリに保存する。

## 実行コマンド

Windows で変更を commit した後:

```powershell
.\tools\sync_to_wsl.ps1 -CheckOnly
.\tools\sync_to_wsl.ps1
```

WSL（`/home/thistle/e2e_autonomous/e2e_lite_transfuser`）:

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -u \
  tools/train_time_recovery_update.py prepare \
  --plan configs/time_path_p1/recovery_random_update_20260915.json --root ..
tools/with_wsl_training_lock.sh timeout --signal=TERM --kill-after=20s 7200s \
  env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 .venv/bin/python -u \
  tools/train_time_recovery_update.py train \
  --plan configs/time_path_p1/recovery_random_update_20260915.json --root ..
```

正常終了した同じ出力への再実行や再 prepare は行わない。中断時は plan/source 同一を確認して `train --resume`。原データ、旧 cache、旧重みを上書き・削除しない。

## 変更範囲と未確認事項

追加済み教師・センサ cache を再ハッシュし、split と shape を検査して新 cache に追記する処理と、その学習ドライバを追加。raw と準備済み教師の対応、イベント ID、全30点教師、入力参照の範囲を検査する。モデルや既存教師生成処理には変更を加えていない。

学習後は旧モデルと新モデルを同じ通常走行・旧復帰・追加復帰・ランダム復帰 validation に通し、時間ごとの XY 誤差と PP の操舵計算を比較する予定。結果判明前の性能向上は主張しない。AWSIM 完走・実際の復帰性能は今回の offline 数値からは確定できない。
