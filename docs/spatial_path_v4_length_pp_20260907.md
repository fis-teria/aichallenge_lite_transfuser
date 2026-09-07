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

## 実行結果：41/41で固定参照の仮想追従・制動停止成立

実行・限定test版 `7165e5fc48b641b247d6abd22c78b8dc4ae71d83`。
開始HEAD `8b0125aeaa3efab1c47dbd89d5e0238026dbb751`、clean。
origin `https://github.com/fis-teria/aichallenge_lite_transfuser.git`、branch `codex/windows-wsl-training-sync`。
Windowsで今回の専用読込/評価CLIとtest/docのみ追加。既存PP/速度制御/仮想plantは変更なし。

限定pytest **38 passed / 1.27秒**。全体pytestは学習・Datasetアクセスを避け未実施。
受理済み41件は各1回、試行41、再実行0、設定探索0。合成unit testはこの41とは別。
既定同期によるDatasetルート存在確認を実施。Dataset内容/raw/sensor/checkpoint読取は未実施。

| 項目 | 全41件の結果 |
|---|---:|
| 事前定義した追従・停止基準 | **41/41成立** |
| 最大cross-track（各試験の最大値の範囲） | 1.459～1.570cm |
| 停止目標との最終位置差 | 5.891～7.129cm |
| 最大速度 | 0.199956～0.199992m/s |
| 各試験virtual時間 | 9.4～11.0秒 |
| 最大舵角 | 0.33974～0.42831rad |
| 最大操舵速度 | 0.8rad/s |
| 最大正加速度 / 減速度 | 約0.2 / 0.8m/s² |
| 最大jerk | 2m/s³（浮動小数点差4.4e-16を数値許容内と判定） |
| 最大横加速度 | 0.016794m/s²以下 |
| 制約違反のある試験 | 0 |
| 停止理由 | 全件ENDPOINT_BRAKE |
| 負加速度の停止操作 | 各7cycle |
| 最終速度 | 全件0m/s |

移動後に停止状態を1秒以上確認。初めから静止、host pause/KILLによる停止ではない。
ただし現実の車両ではなく、独立RK4自転車モデルと明示的な速度ゼロ下限での結果。
停止目標は参照末端-0.1mで、さらに約5.9～7.1cm手前に停止。参照末端まで到達とは言わない。

cross-trackは新たな**生成参照**への距離。元20点や正解routeに対する誤差ではない。
前段の元20点→生成参照の最大偏差7.16～7.49cmを今回の約1.57cmで置き換えない。
41件は停止中の似た観測に由来する短経路であり、41種類のコーナー、全周回、
学習汎化、実sensor更新下の追従成功を意味しない。

## 保存成果物

- Windows `tmp/spatial_length_pp_20260907/results/summary.json`。
- 同directory `trial_00.json`～`trial_40.json`：各時刻の前後状態、lookahead、要求/適用操作、停止理由。
- 同task root `evidence/validation.log`、`tests.xml`、CheckOnly/同期ログ。
- WSL `/home/thistle/e2e_autonomous/spatial_length_pp_20260907/results`。
- summary SHA256 `4d389db9c4388c7c390848aa8069ea647c2a860c3fefc8fdff89ed0e720296e8`。
- 入力summary hashと全41case hashをsummaryに記録し、実行前後不変を確認。

今回の仮想追従検証は完了。新推論/学習/refit/MPC/ROS/AWSIM/制御送信は全0。
旧5件の試験は別結果として保全し、今回再実行していない。
runtime既定設定や制御方式の切替、simulator hostへの配置、pushは行っていない。
次段階は更新経路との接続と監視を含む限定sim試験の準備だが、未解決の監視と追加走行枠は別承認。
