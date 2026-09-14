# 既存復帰データの提示配分・学習指標の比較

事前計画。2026-09-15。結果は実行後に追記する。

既存復帰データの学習不足を、データ追加なしで切り分ける。4条件の2×2比較を行う。

|条件|復帰データの提示配分|学習指標|
|---|---|---|
|uniform_l1|現行の均等提示|現行XY L1。完了済みモデルを条件照合後に再利用|
|balanced_l1|外向き35地点を復帰提示の25%にする|現行XY L1|
|uniform_geometry|現行の均等提示|XY L1 + 復帰用操舵・遠方横位置誤差|
|balanced_geometry|外向き35地点を復帰提示の25%にする|XY L1 + 復帰用操舵・遠方横位置誤差|

既存cache SHA `cb52a01fae1a492b5d06f1473c6ed773410934fe39a019e91c33097db340c895` を使用する。
train 38,136地点（復帰1,410地点）、復帰15 run。各epoch 45,646提示、うち通常36,726・復帰8,920。
均等提示の外向き状態は223提示/epoch、比較条件は2,230提示/epoch。同じ35地点を繰り返すため、独立した観測が増えるわけではない。
外向き対象の11 runへ均等配分し、残りの復帰地点も全地点残す。通常走行の提示位置とデータの内容は不変。

初期重みは共通の通常走行学習済み `time_p1_20laps_20260913/command_off/best.pt`。
seed 42、float32、batch 32、AdamW lr 3e-5・weight decay 1e-4、gradient clip 1、cosine schedule。
3 epoch・4,281更新・136,938提示/条件。新規3条件は各7,200秒の外側上限を置き、逐次実行する。
単一seedであり、多seedでの改善の再現性までは検証しない。

追加損失は次の固定式。重みは比較結果を見る前に固定し、比較専用validationで調整しない。

`L = mean_supported_anchor( XY_L1 + recovery * (1.0 [m/rad] * abs(delta_PP_rad) + 0.5 * mean_abs_y_error_2_to_3s_m) )`

教師経路に実際のPure Pursuit選点を適用して、時刻と速度依存の応答長を固定する。
同じ時刻で予測経路・教師経路を線形補間し、rear axleへ約1 mm変換した後、
`atan(2 * response_length_m * y / (x*x + y*y))` の物理タイヤ角誤差を求める。
1 radの誤差を1 m相当として加え、遠方Yの0.5倍を追加する。これは初回の比較用設定で、最適値との主張はしない。
遠方は2.0〜3.0秒の11点。通常走行には追加損失を適用しない。
全支持anchor数で正規化し、未支持教師で薄めない。復帰用教師の欠損は明示エラー。
追加情報はモデルforward後のloss専用で、画像・LiDAR・ego入力には追加しない。

操舵損失は教師が選んだ時刻を固定する微分可能な近似であり、予測ごとのruntime選点そのものではない。
学習・診断ともfixed 5 km/h、`stopping_preview_extended_v1`、観測時点age=0。
停止領域監視・遅延・実操舵追従はこの追加損失に含まない。診断では実際のPP選点も別に通して比較する。

epoch選択は従来の6 validation runのrun等重み3秒XY誤差の最小を維持する。
復帰r46/r47/r65は選択後の診断専用。train全復帰1,410地点・外向き35地点、validation全復帰538地点・外向き12地点を比較する。
通常validation4 runの誤差も確認する。各runの結果を残し、相関したframeを独立試行とは数えない。
封印testのraw/cacheは読まない。AWSIM完走は別の検証であり、オフライン改善だけで合格とはしない。

既存runnerへ既定OFFの追加loss引数だけを設ける。default経路のASTが基準commitと等しいことを検査する。
過去の厳密なsource proofは緩めず、今回専用の差分検証を用いる。旧実験toolを再現する場合は旧commitを使用する。

Windowsでcommitし、`tools/sync_to_wsl.ps1 -CheckOnly`、`tools/sync_to_wsl.ps1`の`SYNC_OK`を確認してから実行する。
WSL native repo `/home/thistle/e2e_autonomous/e2e_lite_transfuser` で以下を実施する。

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 .venv/bin/python -m pytest -q
tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 .venv/bin/python tools/compare_time_recovery_objectives.py prepare --plan configs/time_path_p1/recovery_objective_comparison_20260915.json
# ARMを balanced_l1 / uniform_geometry / balanced_geometry として各1回実行
tools/with_wsl_training_lock.sh timeout --signal=TERM --kill-after=20s 7200s env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 .venv/bin/python tools/compare_time_recovery_objectives.py train --plan configs/time_path_p1/recovery_objective_comparison_20260915.json --arm "$ARM"
tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 .venv/bin/python tools/compare_time_recovery_objectives.py compare --plan configs/time_path_p1/recovery_objective_comparison_20260915.json
```

重み・全予測・実行logはWSLの`runs/time_recovery_objective_comparison_20260915`へ保存し、Gitには含めない。
