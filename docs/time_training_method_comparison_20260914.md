# 既存データ固定：復帰状態の提示とモデル選定の比較

## 比較前に固定する条件

データ不足と学習方法を区別するため、既存の入力・教師・splitを固定して、提示バランスとモデル選定だけを比較する。新規教師生成、追加収集、loss変更、速度・安全監視の変更は行わない。モデル外の教師・評価情報を推論入力へ追加しない。

| 条件 | 復帰データの提示 | epoch の選定 |
|---|---|---|
| A | 従来の均等反復 | 従来の3秒XY誤差 |
| B | 従来の均等反復 | 教師軌道によるPP操舵との一致 |
| C | 外向き状態を多く提示 | 従来の3秒XY誤差 |
| D | 外向き状態を多く提示 | 教師軌道によるPP操舵との一致 |

同じ学習済みepoch群からA/BまたはC/Dを選ぶため、選定基準の効果を追加のoptimizer更新と混同しない。同じepochが選ばれれば、その組の差は0として報告する。

- unique train 37,678、うち復帰952を全て保持。通常36,726提示＋復帰8,920提示/epoch、3epochで136,938提示・4,281更新を固定。
- 外向き23アンカーは監査済みのtrain 8runだけから指定する。復帰8,920枠の25%（2,230提示）をこの状態へ割り当て、その中はrun等重みとする。残る6,690枠は他の復帰929アンカーへ均等に割り当てる。通常アンカーの提示位置・順序を変更しない。23アンカーは8回の復帰中の相関したフレームであり、反復で独立データは増えない。
- 起点・モデル・XY全30点のL1 loss・AdamW・cosine scheduler・seed42・batch32・lr3e-5・float32を前回と一致させる。3epochの有限比較で、学習回数や容量が十分かを証明する実験ではない。
- 選定は従来の通常4run＋旧復帰2runだけ。新復帰r46/r47は選定後の比較に使用する。これら2runは既に前回報告した検証データで、未見の最終testとは呼ばない。元のtest 4runとr48/r49の評価予約は未使用のまま保つ。
- 新選定は、同じ観測状態で教師30点と予測30点を現行PPへ通した物理タイヤ角の絶対差をrun等重みで平均する。教師が成立する固定母集団を使い、モデルの拒否は0.6 rad（タイヤ角の全範囲）として分母に残す。候補の拒否で平均を改善させない。同値なら3秒誤差、さらに同値なら早いepochを選ぶ。
- PPは固定目標5km/h・`stopping_preview_extended_v1`・既存車両モデル。選定のageは0 s。これは観測状態での幾何計算であり、遅延・操舵応答・scan監視・反復走行の成功を含めない。
- 従来学習の全epochの保存予測からA/Bを選定する。選ばれた重みが既存bestにあれば再利用し、異なるepochの重みが残っていなければ同一条件で3epochだけ再現する。最大で新規2arm、各学習コマンド7200秒。モデルやlossは変えず、全epochのcheckpointを保持する任意引数だけをrunnerに加える。default動作と保持ありの予測同一性を回帰テストする。

通常走行、旧復帰、新復帰、外向き6アンカー、左右別について、0.5/1/2/3秒のXY誤差、前後・横成分、PP成立数、教師操舵との差を同じ入力で比較する。オフラインの改善をAWSIM完走改善とは扱わない。今回の比較だけで実行モデルへ自動昇格しない。

## 実行

Windows正本でcommit後、`tools/sync_to_wsl.ps1`で同期する。下記はnative WSLのrepo rootで実行する。出力は新規ディレクトリ専用で、再開時のみ同じplanと`--resume`を使う。

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  .venv/bin/python -m pytest -q

tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  .venv/bin/python -u tools/compare_time_training_methods.py prepare \
  --plan configs/time_path_p1/method_comparison_20260914.json \
  --output ../runs/time_training_method_comparison_20260914

tools/with_wsl_training_lock.sh timeout --signal=TERM --kill-after=20s 7200s \
  env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  .venv/bin/python -u tools/compare_time_training_methods.py train --arm balanced \
  --plan configs/time_path_p1/method_comparison_20260914.json \
  --output ../runs/time_training_method_comparison_20260914

# prepare_result.json が uniform_retraining_needed=true の場合に限る。
# 上のtrainコマンドの --arm を uniform にして同一予算で再現する。

tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  .venv/bin/python -u tools/compare_time_training_methods.py compare \
  --plan configs/time_path_p1/method_comparison_20260914.json \
  --output ../runs/time_training_method_comparison_20260914
```

重み・cache・全予測配列はWSLに保持し、小さい指標・証跡・図だけをWindowsへ戻す。実測結果は完了後に追記する。
