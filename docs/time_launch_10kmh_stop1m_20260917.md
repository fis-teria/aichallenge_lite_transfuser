# 目標10km/h・停止監視1mのAWSIM比較

ユーザーの「停止監視を1mにしましょう」に基づく1回のAWSIM試験。直前の5km/h相当監視（移動距離最大2.05895m）は202.37m・section 4で終了した。

## 条件

- 実行先`graneple@192.168.3.10`、評価はnative WSL。
- 同一checkpoint `launch_balanced/epoch_03.pt`、SHA256 `1fe12ca066791d3eb8907c6921120e2cfbb38925c422d5be54940f4acbdd120a`。
- 目標10km/h、速度超過監視11km/h、PP先読み、操舵応答、車体寸法・左右余裕は直前試験と同じ。
- config差分は`stopping_distance_policy=awsim_cap_1m_diagnostic_v1`だけ。直前の距離式に`min(距離, 1.0)`を適用する。停止状態では既存の0.4mを維持。
- 1mは車体を掃引する後軸の移動距離の上限であり、後軸から1mだけの点群を見る意味ではない。車体前端・幅・旋回時の掃引も判定に含む。
- 横移動余裕は直前の5km/h相当設定を維持する（最大0.0567m）。曲率区間・速度・yaw・横速度は実測値で検証する。
- PP先読みは実速度依存のまま（10km/hなら最低約5.647m）。学習済み生経路を変更しない。
- 1m領域は10km/hでの制動完了を保証しない。指定ホスト・1周・有限時間の診断設定で、通常設定は保持する。AWSIMのファイルは変更しない。
- 通常RVizの予測経路表示、AWSIM／RViz録画を実施する。

## コマンド

Windowsでcommit後、公式同期を実施する。各operatorは同名出力を上書きしない。

```powershell
python tmp/time_ten_site_recovery_plan_20260915/sync_native_transport.py sync
Get-Content -Raw tmp/time_launch_10kmh_stop1m_20260917/prepare_native.py | wsl -d Ubuntu-22.04-Recovered -u thistle --exec bash -c 'cd /home/thistle/e2e_autonomous/e2e_lite_transfuser && tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 .venv/bin/python -'
Get-Content -Raw tmp/time_launch_10kmh_stop1m_20260917/compare_previous_guard_native.py | wsl -d Ubuntu-22.04-Recovered -u thistle --exec bash -c 'cd /home/thistle/e2e_autonomous/e2e_lite_transfuser && tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -'
python tmp/time_launch_10kmh_stop1m_20260917/manage.py package
python tmp/time_launch_10kmh_stop1m_20260917/manage.py prepare
python tmp/time_launch_10kmh_stop1m_20260917/manage.py start
python tmp/time_launch_10kmh_stop1m_20260917/monitor.py
python tmp/time_launch_10kmh_stop1m_20260917/finish_and_evaluate.py
```

準備中。試走結果・動画・判定再現を追記する。
