# 復帰データ追加後の再学習（2026-09-15）

状態: **再学習・旧新モデル比較・図の確認まで完了**。新規復帰の位置予測は改善したが、厳格な外向き逸脱で PP 操舵誤差が悪化し、実走での復帰向上は未確認。新重みは比較候補として保存した。

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

旧モデルと新モデルを同じ通常走行・旧復帰・追加復帰・ランダム復帰 validation に通し、時間ごとの XY 誤差と PP の操舵計算を比較した。AWSIM 完走・実際の復帰性能は今回の offline 数値からは確定できない。

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

## 同じ入力による旧新モデル比較

比較 source は `72ca5ba0774c4c6ce7ecbe501d942f482150abb4`。best は両モデルとも epoch3。通常の選択用、既存追加復帰、新規ランダム復帰で推論バッチを分離し、以前と同じバッチ形状を維持した。両モデルの選択用予測はそれぞれ保存済み tensor と厳密一致。

3 s の位置誤差は、支持された anchor 内で各 run の平均を求め、run 間を等重みとする。変化率は `(新/旧-1)*100`、小さいほど良い。「外向き」行は上位行の部分集合であり、独立した追加データとして合算しない。

| 検証対象 | run数 | 3 s計算母数 / 総anchors | 旧 [cm] | 新 [cm] | 変化 |
|---|---:|---:|---:|---:|---:|
| 通常走行 | 4 | 11,780 / 12,237 | 5.435 | 5.428 | −0.14% |
| 初期の復帰 r22/r23 | 2 | 109 / 109 | 3.517 | 3.582 | +1.84% |
| 追加復帰 r46/r47 全体 | 2 | 186 / 186 | 3.475 | 3.441 | −0.98% |
| r46/r47 外向き subset | 2 | 6 / 6 | 4.724 | 5.083 | +7.60% |
| 新規ランダム復帰 r65 全体 | 1 | 243 / 243 | 4.683 | 4.068 | −13.14% |
| r65 外向き subset | 1 | 6 / 6 | 6.635 | 6.044 | −8.91% |

通常走行を維持しながら、新規復帰 run の位置予測は改善した。ただし旧外向き subset は悪化しており、全体に一様な改善ではない。

新規 r65 のイベント別 3 s 誤差は event1 が5.399→3.491 cm、event2 が3.476→3.472 cm、event3 が5.453→5.085 cm。全体の改善は特に event1 に寄っている。1走行内の3イベントと連続 frame は相関があり、243件の独立試行や異なるコースへの汎化として扱わない。

## PP の操舵計算

固定目標5 km/h、`stopping_preview_extended_v1`、age 0 s、同じ実測速度・車両モデルで計算した。指標は教師軌道から求めた物理タイヤ角と予測軌道から求めた角度の絶対差を run 等重みで平均した値。教師 PP が成立する母数を固定し、予測側が拒否された場合は0.6 radを加える。

| 対象 | 教師PP支持数 | 旧 [rad] | 新 [rad] | 変化 |
|---|---:|---:|---:|---:|
| 通常走行 | 8,226 | 0.016909 | 0.016968 | +0.35% |
| 初期の復帰 r22/r23 | 109 | 0.005353 | 0.005750 | +7.41% |
| r46/r47 全体 | 186 | 0.002811 | 0.002961 | +5.33% |
| r46/r47 外向き subset | 6 | 0.006463 | 0.007148 | +10.60% |
| r65 全体 | 243 | 0.003780 | 0.003496 | −7.53% |
| r65 外向き subset | 6 | 0.006897 | 0.008387 | +21.61% |

復帰群は両モデルとも全件 PP 計算が成立し、NaN 予測も0件。通常走行では、教師支持8,226件中の予測拒否は両モデルとも53件。教師支持に限定しない全 PP 試行の拒否は両モデルとも524 / 適用対象8,780件（速度域外3,412件・入力無効45件は別）。拒否して難しいサンプルを指標から外すことで改善した結果ではない。

**新規復帰の将来位置誤差が小さくなっても、外向き逸脱時の PP 操舵誤差は改善していない。** このため、現時点では実走への採用判断を保留する。今回の比較にはセンサ遅延後の状態変化、操舵応答、scan監視、AWSIM反復走行は含めていない。データ追加だけで復帰問題が解決したとは判断しない。

## 成果物と確認済み範囲

- [比較の全数値](evidence/time_recovery_random_retraining_20260915/summary.json)、[要点の数値](evidence/time_recovery_random_retraining_20260915/compact_summary.json)
- [時間別の位置誤差図](evidence/time_recovery_random_retraining_20260915/horizon_errors.png)、[PDF](evidence/time_recovery_random_retraining_20260915/horizon_errors.pdf)
- [復帰区間の軌道図](evidence/time_recovery_random_retraining_20260915/random_recovery_paths.png)、[PDF](evidence/time_recovery_random_retraining_20260915/random_recovery_paths.pdf)
- 軌道図は各イベントの監査済み外向き subset の先頭 anchor を固定選択した。良い例の選別はしていない。車体座標の前方・左方 [m] で、横軸と縦軸の縮尺は異なる。PNGを目視確認済み。
- 最終 source72ca5ba の WSL `pytest -q`: **2,488 passed / 4 skipped / 65 warnings、92.08 s**。比較・描画CLIも native WSLで正常終了。
- [搬送ハッシュ照合](evidence/time_recovery_random_retraining_20260915/transfer_verification.json): native WSLとWindowsで22ファイル・1,249,174 bytesが一致。旧新 checkpoint の SHA256も再照合済み。大きな重み・入力・予測 tensor はWSLに保持。
- nominal test4 と r48/r49 の bag/cache は未読、AWSIM新規試験0回。推論環境のモデル置換は行っていない。学習結果を見た後の追加 epoch、seed変更、選択基準変更は行っていない。
