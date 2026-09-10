# 2 m V4：0.2 m参照のAWSIM shadow試験27

2026-09-10。実行版 `1ab0ad6c2837e3d545774f90724632830b320039`、
origin `https://github.com/fis-teria/aichallenge_lite_transfuser.git`、
branch `codex/windows-wsl-training-sync`。
報告保存時には別の20 m学習タスクのcommit `9bc0228` が追加されていた。
本試験結果は同commitの実行結果ではない。学習側の変更は保全した。

## 結論

0.2 m再サンプリング・rawとの差の上限0.03 mで実入力を試した。
一部は幾何検査を通過して現在状態検査まで進んだが、**有効Trajectoryは0件**。
V4の追従走行はまだ成立していない。実際の車両は既存基準参照PPで走行した。

次の問題は `control/path_control_bridge.py` の `ShadowBridge.tick` が、
`Limits.speed_cap_mps` を参照目標速度上限と現在車速の許容上限の両方へ使用すること。
今回のshadow設定は0.5 m/sだが、既存PPによる観測車速はこれを超える。
目標速度上限と観測状態の受入れ範囲を分離する必要がある。
速度検査を削除することや、実車用の制約が校正済みという意味ではない。

`STATE_OUT_OF_LIMITS` 222周期に対応する、probe内の直近受信vxは
1.6225574～4.5389724 m/sで、全222件が0.5 m/sを超えた。
これは同一stream上の直近受信値であり、node内部で採用したstateの完全な対応証拠ではない。
同reasonには操舵上限等も含まれるので、全件の唯一原因が速度とは断定しない。
次回は採用state値と拒否項目をstatusに残す。

## 実施と結果

- launchに `config_file` 引数を追加。既定設定、rawモデル出力、車両側接続は変更なし。
- 試験専用configだけ `shadow_spacing_m=0.2`。最大参照偏差0.03 mは維持。
- 新規学習・20 mモデル変更なし。固定V4の実推論・実sensor受信は実施。
- Windows commit後、既定CheckOnlyと同期を実施。
  Datasetルート存在確認を実施。Dataset内容の読取りは未実施。
- WSL lock下の限定tests：19 passed in 0.19 s。全pytestは今回NOT_RUN。
- 専用配布先のROS package build：1 package成功。
- AWSIM移動時間20.004999553 sim秒、host全工程64.09755秒。
- 最大車速4.5389761 m/s（約16.34 km/h）。
- scan695、SLAM pose688、外挿pose2461、PLAN record172。
- Trajectory747件すべて空。shadow PP指令3312件すべて負加速要求。
- 実制御topic送信元は `/simple_pure_pursuit_node` のみ。
- probe fault / host errorなし。AWSIM改変なし、V4実制御送信なし。
- 終了は `OWNED_SIMULATOR_FREEZE_KILL_NOT_BRAKING`。所有instanceのみ終了し、
  cleanup errorなし、終了後docker psは空。制動成功・無接触・完走の証明ではない。

statusは毎周期の集計であり、172経路の件数とは区別する。

|reason|周期数|
|---|---:|
|STATE_INVALID:NO_PLAN|134|
|REFERENCE_DEVIATION_EXCEEDED|191|
|STATE_INVALID:STATE_STALE_OR_UNALIGNED|1|
|CURVATURE_INFEASIBLE|199|
|STATE_OUT_OF_LIMITS|222|

補助 `per_plan_geometry.json` は元raw点の診断であり、再サンプリング後の拒否内訳ではない。
曲率と偏差の拒否も残るため、状態速度の分離だけで全経路が通るとはいえない。

## 再現コマンド・証拠

WSL正本同期後、lock付き限定検証：

```bash
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_shadow_resample_v4.py tests/test_v4_pp_connection.py tests/test_v4_pp_reference_adapter.py
```

専有simulator host `graneple@192.168.3.10` で使用した実行コマンド：

```bash
CARTOGRAPHER_BUILD_ROOT=/home/graneple/e2e_autonomous/cartographer_extrap_build_20260910 CARTOGRAPHER_TEST_PROJECT=codex-v4-resample-27 CARTOGRAPHER_V4_EXTRAP_COMPARE=1 V4_SLAM_SHADOW_ROOT=/home/graneple/e2e_autonomous/v4_pp_resample_27 python3 /home/graneple/e2e_autonomous/cartographer_v4_resample_27/run_moving.py
```

runnerは専用instanceの `make dev DEV_AUTO_START=false CONTROL_METHOD=pure_pursuit CAPTURE=false ROSBAG=false AWSIM_LAPS=1` と公式Start要求を使用。
上記は実行記録であり、無条件再実行の承認ではない。

- Windows証拠：`tmp/v4_pp_resample_27/evidence/` 内の
  `timing.jsonl`, `shadow.jsonl`, `probe_result.json`, `host_result.json`, `live_connection_summary.json`。
- Windows実行スクリプト：`tmp/v4_pp_resample_27/`。
- hostの完全ログ：`/home/graneple/e2e_autonomous/cartographer_v4_resample_27/`。
- V4配布先：`/home/graneple/e2e_autonomous/v4_pp_resample_27/`。

次の最小変更はshadow状態受入れ範囲と参照速度方針の分離、および拒否時stateの記録。
幾何閾値を無根拠に緩和せず、まず合成testsで減速参照と状態判定の責任を検証する。
実制御への昇格、V4による走行、停止性能は未検証。
