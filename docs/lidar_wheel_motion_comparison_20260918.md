# コーナー入口のLiDAR scan間移動と車輪オドメトリの比較

実行中の制御を変更しない保存データの診断。地図照合の成否と、scan間の相対移動を分離する。
LiDAR側は単位姿勢からのpoint-to-line ICP。地図、GNSS/IMU、記録済み自己位置、車輪の罰則項を使わない。
車輪を初期値にした再計算、逆方向の照合、幾何の弱方向も独立性・安定性の確認用に記録する。
LiDARを真値とは扱わず、条件の弱い区間も除外せず報告する。
`base_link -> lidar` の1.65 m前方の取付位置を考慮して両方を前時刻の車両座標へ変換する。
位置はm、角度はrad、時刻はシミュレーション秒。

## 再現

Windowsから通常手順で同一commitをnative WSLへ同期した後:

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q tests/test_lidar_wheel_motion_diagnostic.py
tools/with_wsl_training_lock.sh .venv/bin/python tools/diagnose_lidar_wheel_motion.py \
  --run /home/thistle/e2e_autonomous/runs/time_teacher_si26_20260911/laps03/5kmh_run01 \
  --records /home/thistle/e2e_autonomous/runs/lidar_map_runtime_20260918/replay05/records.jsonl \
  --start 35 --end 50 \
  --output /home/thistle/e2e_autonomous/runs/lidar_wheel_motion_20260918/compare01
```

出力先は新規directoryとし、raw scan配列はWSLのruns内に保持する。
約0.2秒間隔に間引かれた連続scanの同一stampで比較する。元センサの全フレーム間隔ではない。
既存replayの車輪積分結果を使い、GNSS/IMU/既存poseを追加で読まない。

## 判定の限界

`diagnostic_consistent` は、対応点率70%以上、100点以上、正規化情報行列の最小/最大固有値比0.001超、
順逆の往復差および初期値変更の差が0.03 m/0.01 rad未満という診断上の条件。
安全性や統計的信頼区間を意味しない。5 mの角度スケールで固有値を比較する。
完全な直線は前後位置を拘束せず、残差が小さくても移動量の真値にはならない。
動的物体、遮蔽、実機のscan内運動歪みの一般的な除去は今回の診断の対象外。

## 結果

実測結果は検証終了後に記載する。
