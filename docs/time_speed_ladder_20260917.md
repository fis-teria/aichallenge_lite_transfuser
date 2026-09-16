# 停止監視1mでの速度段階試験

ユーザーの「15km/hから5km/hずつ増やしてどこまで行けるか」に基づくAWSIM比較。直前の目標10km/hでは1周134.74秒、実測速度中央値9.57km/hで完走している。

## 固定する条件と進め方

- 実行先は`graneple@192.168.3.10`、学習・評価環境はnative WSL。
- 同一checkpoint `launch_balanced/epoch_03.pt`、SHA256 `1fe12ca066791d3eb8907c6921120e2cfbb38925c422d5be54940f4acbdd120a`。
- 停止監視は直前の移動距離最大1m。PP先読み・操舵応答・車体幅余裕・生予測軌道・各種データ鮮度監視は同じ。
- 15km/hから実施し、合格したときだけ20km/h、25km/h…と5km/hずつ増やす。各設定1周・sim/wall各600秒の有限試験とする。
- 速度超過監視は各目標+1km/h。車両応答式の速度域だけを明示的な試験policyで拡張し、5km/h・10km/h用の従来の速度域を保持する。
- 合格はこれまでのユーザー指定どおりJudgeの1周完了。初回未完走以降の速度は走らせない。
- 実測速度の中央値・最大値も別に記録する。中央値が目標の90%以上かは速度到達の補助指標であり、完走の合否条件には加えない。制動を含むactive期間の速度サンプルを使い、0.1m/s以下のみ除外する。
- 1m監視は実速度での制動距離を保証しない。低速で得た車両応答式の外挿であり、高速域で校正済みとは扱わない。モデルや通常監視設定の正式採用は変更しない。
- AWSIM本体を変更せず、通常RVizにE2E経路を表示し、AWSIM動画を記録する。

15km/h用に変更するconfig項目はspeed policy、目標速度、超過監視、車両モデルpolicyの4つ。停止領域が1mでも、PP最低先読みは実速度で計算するため、15km/hで11.1639mとなる。予測経路が届かなければ既存条件どおり制動し、経路を延長・補正して試験を通さない。

## 実行コマンド

Windowsでcommit後にWSLへ同期する。operatorは同名出力を上書きしない。後続速度へ進む場合は、それぞれ専用configとrun IDを作る。

```powershell
python tmp/time_ten_site_recovery_plan_20260915/sync_native_transport.py sync
Get-Content -Raw tmp/time_launch_15kmh_stop1m_20260917/prepare_native.py | wsl -d Ubuntu-22.04-Recovered -u thistle --exec bash -c 'cd /home/thistle/e2e_autonomous/e2e_lite_transfuser && tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 .venv/bin/python -'
python tmp/time_launch_15kmh_stop1m_20260917/manage.py package
python tmp/time_launch_15kmh_stop1m_20260917/manage.py prepare
python tmp/time_launch_15kmh_stop1m_20260917/manage.py start
python tmp/time_launch_15kmh_stop1m_20260917/monitor.py
python tmp/time_launch_15kmh_stop1m_20260917/finish_and_evaluate.py
```

準備中。実測値・周回判定・速度到達判定・停止理由を追記する。
