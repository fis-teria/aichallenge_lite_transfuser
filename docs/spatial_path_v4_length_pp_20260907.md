# 可変長参照41件：Pure Pursuit仮想追従

ユーザーの「Pure Pusruit追従まで」に対応。新方式の受理済み41件を各1回、保存曲線を変えず評価。
旧5件の結果と混ぜず、元41件の順序・ID・raw hash・prefix・初期状態・configを照合。
case全件のhashを開始時/終了時に確認して保存。再fit・再推論・学習・MPC solveは0。

既存 `spatial_pure_pursuit_v4.evaluate` とPP/速度P制御/RK4自転車plantは変更しない。
先読み0.5m、速度0.2m/s（上限0.3）、既定操舵・加減速・jerk制限、最大60virtual秒/件。
移動後静止1秒、最大cross-track<=0.1m、停止目標からの最終距離<=0.1m、制約違反0を事前基準。
停止目標は参照末端から0.1m手前、停止距離予約は前回と同じ。結果を見た調整・探索なし。
違反や未停止を成功に書き換えない。画面/実sensor/障害物のない仮想試験であり安全証明ではない。

Windows commit→既定CheckOnly/同期→WSL同SHAでworktree lock付き限定検証。
既定Datasetルートの存在判定のみ許可。Dataset内容/raw/sensor/checkpointは読まない。
AWSIM/ROS起動0、制御publish0、remote simulatorへの配置0、自動pushなし。

```bash
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_spatial_length_pp_v4.py tests/test_spatial_pure_pursuit_v4.py tests/test_spatial_sim_e2e_v4.py
bash tools/with_wsl_training_lock.sh .venv/bin/python tools/evaluate_spatial_length_pp_v4.py --input /home/thistle/e2e_autonomous/spatial_length_review_20260907/results --baseline /home/thistle/e2e_autonomous/spatial_blocker_fixes_20260907/replay_final/replay.json --configs /home/thistle/e2e_autonomous/spatial_blocker_fixes_20260907/inputs --output /home/thistle/e2e_autonomous/spatial_length_pp_20260907/results
```

新しいV4観測での経路更新、滑り/遅延、周辺監視、衝突/逸脱、実sim一周は対象外。
今回成功しても、それらのゲートや追加走行予算が承認されたことにはしない。
