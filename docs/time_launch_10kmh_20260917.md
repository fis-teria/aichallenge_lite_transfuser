# 目標10km/hのAWSIM試走

ユーザーの「次は目標速度10km/hでやってみようか」に基づく1回の探索試験。前回5km/hで完走した`launch_balanced/epoch_03.pt`を使い、`graneple@192.168.3.10`で実施する。学習は行わず、記録の評価はnative WSLで行う。

- checkpoint SHA256: `1fe12ca066791d3eb8907c6921120e2cfbb38925c422d5be54940f4acbdd120a`。
- 設定: `configs/control/time_path_launch_10kmh_20260917.json`。目標10/3.6 m/s、超過監視11/3.6 m/s。
- 前回設定から変更する4項目はspeed policy・目標速度上限・超過監視値・明示的な10km/h用車両モデルpolicy。
- `awsim_understeer_10kmh_trial_v1`は既存の低速車両応答式を速度方向に外挿する試験用設定。10km/hで再校正済みとは扱わない。従来の`awsim_understeer_v1`とideal policyの6km/h制限を維持する。
- PP・操舵応答補償・操舵角／速度制限・制動1m/s²・遅延0.5秒・固定停止余裕0.4m・横速度許容0.03m/s・車体幅余裕を維持する。10km/hの停止距離に応じた最低先読みは約5.647m。予測が短ければ制動する。
- 停止領域の積分は同じ式を使い、10km/h用policyだけ移動長と横移動幅の計算範囲を広げる。積分が仮定する旋回角域`max(abs(k))*travel < pi`を明示的に検証し、域外は拒否する。
- 既存の1周／監視停止／sim・wall各600秒／外側720秒で終了。通常RVizに生予測経路を表示する。
- `--record-video`でAWSIMと通常RVizのアプリwindowだけを各10fps、H.264で録画する。録画の開始を確認してから発進する。録画用Docker imageは別に用意し、AWSIMのファイルを変更しない。
- 正式採用や前回のオフライン不合格判定の書換えは行わない。

## 実行手順

Windowsでcommit後、公式同期scriptを通してWSLへ同期する。各operatorは出力の新規作成を要求するため、再実行する試走には別のrun ID・出力先を使う。

```powershell
python tmp/time_ten_site_recovery_plan_20260915/sync_native_transport.py sync
Get-Content -Raw tmp/time_launch_10kmh_20260917/prepare_native.py | wsl -d Ubuntu-22.04-Recovered -u thistle --exec bash -c 'cd /home/thistle/e2e_autonomous/e2e_lite_transfuser && tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 .venv/bin/python -'
python tmp/time_launch_10kmh_20260917/manage.py package
python tmp/time_launch_10kmh_20260917/manage.py prepare
python tmp/time_launch_10kmh_20260917/manage.py start
python tmp/time_launch_10kmh_20260917/monitor.py
python tmp/time_launch_10kmh_20260917/finish_and_evaluate.py
Get-Content -Raw tmp/time_launch_10kmh_20260917/inspect_native.py | wsl -d Ubuntu-22.04-Recovered -u thistle --exec bash -c 'cd /home/thistle/e2e_autonomous/e2e_lite_transfuser && tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -'
Get-Content -Raw tmp/time_launch_10kmh_20260917/route_native.py | wsl -d Ubuntu-22.04-Recovered -u thistle --exec bash -c 'cd /home/thistle/e2e_autonomous/e2e_lite_transfuser && tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -'
python tmp/time_launch_10kmh_20260917/pack_evidence.py
```

## 検証・結果

準備中。全pytest、前回5km/hログの新実装による制御再生、学習／runtime推論一致、隔離ROSでの10km/h合成経路・停止距離・超過監視を確認してから実走する。到達距離・周回判定・停止理由・動画を追記する。
