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

## 結果：固定5件すべてで仮想追従・停止成立

実行版 `5ff0cf8ae4cf0f1adecaed5c433280e0781657b9`。
開始Windows HEAD `d4cedc985d9a76625d73fc06c8fb25a7bfb445ce`、clean。
origin `https://github.com/fis-teria/aichallenge_lite_transfuser.git`、branch `codex/windows-wsl-training-sync`。
最終限定test **30 passed / 0.94秒**。全体pytestは学習・Datasetアクセスを避け未実行。
既定同期によるDatasetルート存在確認は実施。Dataset内容/raw/sensor/checkpoint読取は未実施。

対象IDはrun_af1c7f1_03 forward9、run_b7a3fae_02 forward7、
run_d41022c_01 forward5/20/35。元の停止中観測に由来する近い形状の5件であり、
独立した5コースや多様なカーブ5場面の検証ではない。

| 診断項目 | 結果 |
|---|---|
| 事前固定した追従・停止基準 | 5/5成立 |
| 各試験のvirtual時間 | 8.8秒（静止確認1秒以上を含む） |
| 最大cross-track | 約0.0164m（約1.7cm） |
| 停止目標からの最終位置差 | 約0.063m（約6.3cm手前） |
| 最大速度 | 約0.19992m/s |
| 最終速度 | 全件0m/s |
| 負加速度での停止操作 | 各7cycle |
| 最大操舵速度 / jerk | 0.8rad/s / 2m/s³ |
| 速度・舵角saturation / 記録された制約違反 | 0 / 0 |

cross-trackは**受理された参照曲線に対する誤差**であり、未補正20点や真の正解routeとの誤差ではない。
元20点→参照の偏差上限10cmを、この約1.7cmで置き換えない。
停止目標は参照末端より0.1m手前で、さらに約6.3cm手前で停止した。末端まで走破とはしない。
初期rear stateを固定し、経路を一度も再生成せず、独立RK4仮想plantの状態で追従した。

最初の試行は1件の仮想計算後、numpy boolのJSON直列化で保存前に失敗。
Python boolへ修正し、JSON回帰test追加後に同じ5件・同じ制御設定で再実行。
制御設定調整/探索はなし。計算実施数は初回1+最終5=6（合成unit testは別）。
初回ログと出力directoryを保全、旧試験結果の上書きなし。

結果はWindows `tmp/spatial_pp_offline_20260907/trials_final/summary.json` と `trial_0..4.json`、
生ログ/JUnit/同期記録は同rootの `evidence/` に保存。
summary SHA256 `a04555ce6df42c5726cffbcf90cc4f985936f56e9e22392344db9cac4c93c1ac`。
再現時の--outputは未使用名を指定すること。最終実行は上記commandの末尾を `trials_final` とした。

結論：**この5件の固定短経路をPure Pursuitで追うことは仮想試験上成立した**。
41件の参照拒否、走行後の新観測→V4→経路更新、滑り/遅延、周辺監視は未解決/未検証。
Pure Pursuitへの実runtime切替、AWSIM起動、実制御送信、追加学習、pushは行っていない。
実走行への承認・追加予算が得られたとは扱わない。
