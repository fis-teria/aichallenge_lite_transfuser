# 10km/h・停止監視を5km/h相当に固定した比較試験

ユーザーの「停止範囲は5km/hのときと変えずにもう一度AWSIMテスト」に基づく、`graneple@192.168.3.10`での1回のシミュレータ比較試験。先行する10km/h試験は約44.95m、最初の右コーナーで停止領域監視が発動した。

## 比較条件

- 同一checkpoint `launch_balanced/epoch_03.pt`、SHA256 `1fe12ca066791d3eb8907c6921120e2cfbb38925c422d5be54940f4acbdd120a`。
- 目標10km/h、超過監視11km/h、PP先読み・操舵応答・経路・車両モデルを先行試験から維持。
- `stopping_distance_policy=awsim_cap_5kmh_diagnostic_v1`だけを比較用に追加。停止領域の距離・横移動余裕の計算速度を`min(実速度, 5/3.6)`m/sにする。
- 距離式`0.4 + 0.5v + v²/2`mは同じ。5km/h以上では最大2.0590m、横移動余裕0.0567m。車体寸法・左右20cmの余裕・操舵区間・LiDARの時刻合わせは同じ。
- 曲率区間・yaw・横速度の妥当性確認には実速度を使う。元の5km/h走行と車体姿勢が異なるので、地図上で同一形状になるという意味ではない。
- PP最低先読みは実速度で計算するため、10km/hでは約5.647mのまま。
- この領域は10km/hでの停止範囲を保証しない。通常設定は変更せず、明示的なdiagnostic設定・指定ホスト・1周の有限試験に限定する。
- 監視発動、完走、sim/wall各600秒のいずれかで終了。AWSIMファイルは変更せず、通常RVizの生予測経路とAWSIM画面を録画する。

## 実行

編集・commitはWindows、pytest・評価はnative WSLの共有lock内で行う。operatorは同名出力を上書きしないため、新しい試走には新しい出力先とrun IDが必要。

```powershell
python tmp/time_ten_site_recovery_plan_20260915/sync_native_transport.py sync
Get-Content -Raw tmp/time_launch_10kmh_stop5_20260917/prepare_native.py | wsl -d Ubuntu-22.04-Recovered -u thistle --exec bash -c 'cd /home/thistle/e2e_autonomous/e2e_lite_transfuser && tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 .venv/bin/python -'
Get-Content -Raw tmp/time_launch_10kmh_stop5_20260917/compare_previous_guard_native.py | wsl -d Ubuntu-22.04-Recovered -u thistle --exec bash -c 'cd /home/thistle/e2e_autonomous/e2e_lite_transfuser && tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -'
python tmp/time_launch_10kmh_stop5_20260917/manage.py package
python tmp/time_launch_10kmh_stop5_20260917/manage.py prepare
python tmp/time_launch_10kmh_stop5_20260917/manage.py start
python tmp/time_launch_10kmh_stop5_20260917/monitor.py
python tmp/time_launch_10kmh_stop5_20260917/finish_and_evaluate.py
```

準備中。合否はこの診断設定下での完走有無として記録し、通常停止領域を使った10km/h完走とは区別する。
