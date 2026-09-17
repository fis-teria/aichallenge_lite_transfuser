# 時間軌道とPure Pursuit先読みの整合

3秒先までの時間軌道に対し、操舵目標の距離にも停止距離の二乗項を要求していたため、高速になるほど予測範囲と制御要求が離れていた。操舵用の先読みと障害物監視の停止距離を分離する。モデル・教師・生の予測経路・AWSIM本体は変更しない。

## 実装

新しい`velocity_time_preview_v1`は、実車速v [m/s]に対し`max(1m, 0.4m + 1.5s × v)`をPPの最低先読み距離とする。1.5秒は今回比較する初期設定であり、最適値を同定したという意味ではない。元の年齢・姿勢補正済み経路上から、従来と同じ頂点優先・区間補間・操舵角制限で目標を選ぶ。探索幅0.5m、選択不能時の最大1m拡張も維持する。

終端による速度制限も同じ関数の逆算に統一する。`先読み(v) + 0.5m + 0.3s × v <= 補正後の予測終端距離`を満たす速度までとし、最低1mの分岐も計算する。車速を偽って先読みを短縮したり、予測を外挿したりしない。現在の実車速で必要な目標が取れなければ拒否する。

| 実車速 | 旧PP最低距離 | 新PP最低距離 | 新速度計画が求める予測距離 | 直進・等速3秒の距離 |
| --- | ---: | ---: | ---: | ---: |
| 10km/h | 5.65m | 4.57m | 5.90m | 8.33m |
| 15km/h | 11.16m | 6.65m | 8.40m | 12.50m |
| 20km/h | 18.61m | 8.73m | 10.90m | 16.67m |

カーブへの制動距離から決める速度制限、横加速度目安1m/s²、加速度指令−1〜+1m/s²、速度ゲイン4/sは維持する。障害物監視側の物理停止距離は従来の二乗式を保持する。今回のAWSIM条件は前回のユーザー指定を継承し、1m領域の侵入を記録のみとする。センサ鮮度・NaN・姿勢・運動・速度超過監視は継続する。

旧`curvature_preview_15kmh_v1`と固定速度条件の挙動は保持し、新しい`curvature_time_preview_15kmh_v1`と`curvature_time_preview_20kmh_v1`を明示的な設定で選ぶ。20km/hの車両応答係数は既存近似の外挿であり、高速域の実測校正ではない。指定ホスト・指定シーン・1周・有限時間のAWSIM試験に限定する。

## 検証・実行

解析解のある等速直線・左右円弧で先読み、速度上限、操舵制限を検証する。予測の経過時間0〜0.5秒、短い予測、NaN、速度超過、記録改ざん、独立した停止距離も検証する。全pytestと旧ログの再生はnative WSLの共有lock内で実行する。

```powershell
python tmp/time_ten_site_recovery_plan_20260915/sync_native_transport.py sync
Get-Content -Raw tmp/time_preview15_20260917/prepare_native.py | wsl -d Ubuntu-22.04-Recovered -u thistle --exec bash -c 'cd /home/thistle/e2e_autonomous/e2e_lite_transfuser && tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 .venv/bin/python -'
python tmp/time_preview15_20260917/manage.py package
python tmp/time_preview15_20260917/manage.py prepare
python tmp/time_preview15_20260917/manage.py start
python tmp/time_preview15_20260917/monitor.py
python tmp/time_preview15_20260917/finish_and_evaluate.py
```

まず前回と同じ15km/h上限で比較し、完走できれば20km/h上限でも試験する。停止領域の扱いは両者で同じとする。実測結果、実行source、動画、未解決事項は取得後に追記する。
