# 全コーナーの箱・コーン教師収集

PC10 `graneple@192.168.3.10`、修正版MPPI V45 `lidar-motion-intent-r2`、上限5km/h。
曲率区間で検出した12コーナーの入口に箱6個・コーン6個を交互に同時配置する。
物理車両の4台制限とは別のnative objects枠（上限32個）を使用する。
AWSIMバイナリや既存の教師マージンを変更しない。

生成・検証コマンド:

```bash
python3 tools/prepare_corner_obstacle_collection.py \
  --awsim-repo /home/graneple/git/autononous_ai/aichallenge-racingkart \
  --reference integrations/mppi_v45/assets/multi_purpose_mpc_ros/env/final_ver3/in_corce_line.csv \
  --output /absolute/new/scenarios --prefix lidar-v45-pc10-corners-native-a01

python3 tools/collect_mppi_v45.py \
  --awsim-repo /home/graneple/git/autononous_ai/aichallenge-racingkart \
  --runtime /home/graneple/e2e_autonomous/mppi_v45_collection_fix_20260918/runtime \
  --scenario /absolute/new/scenarios/lidar-v45-pc10-corners-native-a01.yaml \
  --run-id lidar-v45-pc10-corners-native-a01 --speed-cap-kmh 5 \
  --wall-timeout-s 900 --run-budget-gib 1.25 --free-reserve-gib 2 --execute

bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
```

終了は監視基準線のunwrapped進捗 `340.547 + 60 = 400.547m`。
スタートグリッドより手前のcorner_01を次周に通過してから収録を終了する。
シミュレーション予算780s、全処理900s、記録1.25GiB、空き2GiBを保持する。

箱・コーンはV2X車両IDを持たず、物体位置の教師入力はLiDAR→V2Xで作る。
配置の地図確認には既存カートのフットプリントを代理使用するが、実際のprefab寸法を
検証したものではない。既存の0.30m車体間距離監査を緩めず、物体接触の公式結果、
各配置の通過、センサ同期、実測未来教師を別途記録する。
データはrun群全体でsplit未割当とし、既存学習へ自動登録しない。

実走・WSL検証結果は収録後に追記する。
