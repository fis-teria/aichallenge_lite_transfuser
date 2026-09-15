# AWSIM停止領域の余裕を小さくする限定診断（2026-09-15）

ユーザーの「一度ギリギリまで走らせてもよい」という追加指示に対し、通常条件の6走行を完了した後、現行Aで1走行だけ行う。
実行先は `graneple@192.168.3.10`、通常RVizにE2E生経路を表示し、目標5km/h、PP・操舵・推論重みは通常比較Aと同じ。

監視領域の側方余裕を片側20cmから5cmへ、停止距離への固定加算を40cmから10cmへ変更する。
車体幅1.30mに対する半幅は0.85mから0.70mになる。前後端、遅延0.5秒、制動1m/s²、実操舵・ヨーレートからの曲率範囲、横運動と離散化の余裕は維持する。
先読み点の選択・予測軌道の点列を変更しない。NaN/欠損scan、センサtimeout、overspeed、実操舵上限、単一発行元とAWSIM専用の起動条件も維持する。

`diagnostic_clearance_profile=awsim_near_limit_v1` は `diagnostic_only=true`、最大1試行、指定AWSIMホスト、既存車両応答モデル・support監視・one_lap設定がそろう場合だけ使用できる。
通常設定の既定値と計算結果は維持する。比較6本は変更前のsourceと標準余裕で実施し、診断結果を通常条件の合格数へ混ぜない。

1周または既存停止条件まで、走行最大600秒、外側最大720秒。前方LiDARに基づく領域判定であり、全車体の非接触を証明するものではない。
監視作動時は既存runnerが所有するシミュレータをfreezeするため、制動して実際に停止し切る挙動とは区別する。

## 検証と実行

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
```

source archive・checkpoint・installed Pythonの全hash照合、公式イメージ内ROS smokeを通した専用deploymentで実行する。

```bash
python3 <deployment>/source_<sha>/tools/run_time_path_awsim_trial.py \
  --deployment <deployment> --run-id codex-time-near-limit-01 --display :0 \
  --config configs/control/time_path_near_limit_lap_20260915.json

tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  tools/evaluate_time_awsim_trial.py --run <verified_raw> --output <evaluation>
```

保存scanの再評価では `scan_margin(..., clearance_profile='awsim_near_limit_v1')` を明示し、通常profileでも同じ保存状態を照合する。
教師が攻めた経路を通るという仮説は、正常教師の実走位置とE2E位置・予測経路を同じ地点で比較して判断する。

## 結果

準備中。実走とWSLでの再生後に記入する。
