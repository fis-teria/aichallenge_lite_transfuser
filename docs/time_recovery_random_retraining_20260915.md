# 復帰データ追加後の再学習（2026-09-15）

状態: native WSL で再学習・重み再読込検証が正常完了。場面別の旧新モデル比較を実行する。

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
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -u \
  tools/compare_time_recovery_update.py \
  --plan configs/time_path_p1/recovery_random_update_20260915.json --root .. \
  --lookahead-policy stopping_preview_extended_v1
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -u \
  tools/plot_time_recovery_update.py \
  --plan configs/time_path_p1/recovery_random_update_20260915.json --root ..
```

正常終了した同じ出力への再実行や再 prepare は行わない。中断時は plan/source 同一を確認して `train --resume`。原データ、旧 cache、旧重みを上書き・削除しない。

## 変更範囲と未確認事項

追加済み教師・センサ cache を再ハッシュし、split と shape を検査して新 cache に追記する処理と、その学習ドライバを追加。raw と準備済み教師の対応、イベント ID、全30点教師、入力参照の範囲を検査する。モデルや既存教師生成処理には変更を加えていない。

学習後は旧モデルと新モデルを同じ通常走行・旧復帰・追加復帰・ランダム復帰 validation に通し、時間ごとの XY 誤差と PP の操舵計算を比較する予定。結果判明前の性能向上は主張しない。AWSIM 完走・実際の復帰性能は今回の offline 数値からは確定できない。

WSL 全体テスト（source `f560a9561a643d7dd6f3aae92a79b8d2d2388a08`）: 2,488 passed / 4 skipped / 65 warnings、93.70 s。追加の split・event・教師 shape・入力参照の回帰テスト24件を含む。スキップは既存の任意依存関係に由来する。

比較の PP は直近の学習方法比較と同じ `stopping_preview_extended_v1`、age 0 s を明示する。参照する既存車両 config は segment policy を既定値として含むため、比較 CLI で先読み方式を明示する。これは学習結果が出る前に固定した比較条件であり、学習・cache plan・選択基準には影響しない。
以前の r46/r47 の厳格な外向き逸脱6 anchors、新規 r65 の同6 anchors は、それぞれ既存収集監査の確定 ID を用いて補助集計する。全体の誤差で苦手場面の悪化が隠れないようにするためであり、checkpoint 選択には使わない。

## 完了済みの事前照合

- 学習 source: `8c3fd83145830f774473680a88c947b1e3c83899`、PyTorch `2.7.1+cu128`、WSL RTX 4080。
- 追加4走行について原データ、閉じた bag、因果監査、materialized 教師、prepared センサを再ハッシュ・照合し、全件 PASS。
- 新 cache identity: `cb52a01fae1a492b5d06f1473c6ed773410934fe39a019e91c33097db340c895`。identity.json ファイルの SHA256 は `7500d36e570d7091d745a5c7158379165001093bfbe9e63fd125812dfadc2cfa`（JSON 内容の identity hash とファイル bytes hash は別）。
- 既存160 cache files の bytes、以前の学習提示順の再現、通常36,726枠の位置と順序は一致。
- 新復帰1,410 unique anchors は、950件を6回、460件を7回提示し、8,920枠を保持。
- 元の初期重みによる同じ6 validation runs の評価結果は以前の JSON と厳密一致し、学習開始 gate を通過。

事前照合の cache identity と実行 log は `docs/evidence/time_recovery_random_retraining_20260915/`。重み・予測 tensor・学習 cache は native WSL に保持する。

## 学習完了

- 3 epochs、4,281 updates、136,938 presentations を完了。各 epoch の有効44,280・入力除外158・教師未支持1,208件は以前と同数。
- 学習時間は3,019.09 s（約50.3分）。best は epoch3。
- 初期重みの内部 hash と初期検証が一致し、best 重みを再読込した予測 tensor も厳密一致。`matched_budget_verification.json` は PASS。
- 新重み: `/home/thistle/e2e_autonomous/runs/time_recovery_random_update_20260915/best.pt`
- 新重み SHA256: `53e1962b97cfaa47acae3e2ad4687fac96c80672905406fdbe1514abd9563da2`

| Epoch | 前回6-run macro 3 s [cm] | 今回6-run macro 3 s [cm] |
|---|---:|---:|
| 1 | 5.4564 | 5.1652 |
| 2 | 5.1364 | 4.9209 |
| 3 (best) | 4.7960 | 4.8125 |

既存6走行の最終3 s誤差は+0.34%でほぼ同等。全支持点を重みとする ADE は1.6658 cm →1.6375 cm。これは旧選択用 validation の結果であり、新規復帰 validation での改善は以下の比較で別に確認する。
