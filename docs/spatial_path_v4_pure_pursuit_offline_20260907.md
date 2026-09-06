# V4 Pure Pursuit：受理済み5件のoffline追従試験

対象は固定replay_final.json SHA256 `8ca6e490f99cbc86ef57aa45246d4a1c3d4a890b2230d75aa7c4849591cbb927`。
既に幾何受理された5件だけを使う。拒否41件の再選択・再fit・制限緩和なし。
既存control_from_waypointsのPure Pursuitと速度P制御、既存RK4自転車plantを使用。
rear axle frameの固定参照・元初期状態を使い、実車状態の推定精度とは分離する。
先読み0.5m、速度0.2/上限0.3m/s、kp1/s、最大60virtual秒/件。
終端は元参照末端-0.1m。既存jerk付き停止距離+1制御周期進行+0.03mで停止にlatch。
steering rate0.8rad/s、加速0.6/制動1m/s²、jerk2m/s³を保持する。
停止状態を1秒連続確認。評価基準は移動後停止、終端目標距離<=0.1m、最大横誤差<=0.1m、制約違反0。
plantの速度・舵角saturationがあれば違反扱い。速度のゼロ下限は明示した仮想接触条件。
この値は診断条件であって安全保証ではない。結果を見て調整・再試行しない。

Windows commit→既定CheckOnly/同期→WSL lock付き限定testと5件試験。
Datasetルート存在判定のみ許可、内容/raw/sensor/checkpoint/学習はなし。
AWSIM/ROS起動・送信0、MPC solve0、新推論0、参照fit0。sim共通予算を消費/変更しない。
物理的な壁・摩擦・遅延・センサ更新・周辺監視は未試験。現在経路を走った後にV4が
新しい観測から経路更新できるかは検証しない。1周完走・障害物回避の証拠にはしない。

```bash
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_spatial_pure_pursuit_v4.py tests/test_spatial_sim_e2e_v4.py
bash tools/with_wsl_training_lock.sh .venv/bin/python tools/evaluate_spatial_pure_pursuit_v4.py --input /home/thistle/e2e_autonomous/spatial_blocker_fixes_20260907/replay_final/replay.json --configs /home/thistle/e2e_autonomous/spatial_blocker_fixes_20260907/inputs --output /home/thistle/e2e_autonomous/spatial_pp_offline_20260907/trials
```
