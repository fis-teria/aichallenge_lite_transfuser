# V4 → 既存PP shadow接続：AWSIM実入力試験

2026-09-10、実行commit `ed688acd3e4c929410f5ca6209434f97749cdbf6`。
branch `codex/windows-wsl-training-sync`、Windows正本を既定CheckOnly/同期後に配布。
同期はDatasetルート存在確認のみ。今回のAWSIM試験は実sensor・固定checkpointを使用。

## 結論

実入力でV4レコード168件を受信したが、**有効Trajectoryは0件**。
ROS接続は稼働し、専用PPは停止要求を返した。
V4による追従走行は成立していない。実際の車両制御は既存の基準参照PPのみ。

今回は設定を変えず試験した。0.5 m/sというshadow試験速度上限よりも先に、
経路幾何検査で全件が拒否された。速度制限だけを緩めても今回の拒否は解消しない。

## 実施環境・結果

- host: `graneple@192.168.3.10`、ROS Humble。
- 既存racingkart: `/home/graneple/git/autononous_ai/aichallenge-racingkart`。
- 専有project: `codex-v4-pp-live-26`。
- V4配置: `/home/graneple/e2e_autonomous/v4_pp_live_26`。
- 試験配置: `/home/graneple/e2e_autonomous/cartographer_v4_pp_live_26`。
- 走行: **20.004999553 sim秒**、最大4.53863 m/s（約16.34 km/h）。
- host全工程65.2808秒、probe faultなし、host errorなし。
- LiDAR受信708、SLAM pose697、現在外挿pose2470。
- V4 PLAN受信168、PP入力Trajectory833（全て空）。
- shadow PP指令3372（全て負加速停止要求、正加速0）。
- `/control/command/control_cmd` の送信元は `/simple_pure_pursuit_node` だけ。
- AWSIM改変なし、V4から車両への制御送信なし。
- 所有instanceをfreeze/終了、cleanup errorなし、終了後docker psは空。
  停止方式は `OWNED_SIMULATOR_FREEZE_KILL_NOT_BRAKING`。
  制動停止・無接触・完走の証明ではない。

## 拒否内訳：母数を分離

### レコード168件の幾何再計算

保存済みraw XYの隣接方向差、実点間距離、頂点曲率を使用し、
実装と同じ順序（foldback → curvature）で最初の幾何拒否を集計した。
新規推論・補正・平滑化・threshold変更はしていない。

|最初の幾何拒否|経路数|
|---|---:|
|PATH_FOLDBACK：隣接方向差 > 1.0 rad|47|
|CURVATURE_INFEASIBLE：頂点曲率からの操舵角 > 0.64 rad|121|
|合計|168|

曲率は方向差 / 隣接2区間の平均長、操舵角はatan(1.087×曲率)。
これは現在の折線近似・試験設定に対する不適合であり、原因をモデル学習に断定しない。
例として最初の経路は最大方向差1.77857 rad、最小間隔0.02042 m、
最大頂点操舵相当1.50611 radだった。

### ROS status 833回

毎周期の状態出力なので、経路数とは同じ母数ではない。

- PATH_FOLDBACK: 194回
- CURVATURE_INFEASIBLE: 499回
- 初期NO_PLAN: 135回
- 初期velocity欠落3回、pose欠落1回、stale/unaligned1回

移動中statusはfoldback29回、curvature499回。
有効参照が一度もないため、今回の実入力では有効参照からexpiryへの遷移は未検証。
その遷移の合成ROS試験は前報の結果として区別する。

## 再現・証拠

```bash
CARTOGRAPHER_BUILD_ROOT=/home/graneple/e2e_autonomous/cartographer_extrap_build_20260910 CARTOGRAPHER_TEST_PROJECT=codex-v4-pp-live-26 CARTOGRAPHER_V4_EXTRAP_COMPARE=1 V4_SLAM_SHADOW_ROOT=/home/graneple/e2e_autonomous/v4_pp_live_26 python3 /home/graneple/e2e_autonomous/cartographer_v4_pp_live_26/run_moving.py
```

再試行は既存ログを上書きせず新規専有出力先を使う。
内部で既存 `make dev CONTROL_METHOD=pure_pursuit DEV_AUTO_START=false` と
公式Start要求を使用。V4接続は `v4_pp_shadow_connection.launch.py`。
host180秒、走行20sim秒/25wall秒、heartbeat3秒の既存試験上限を維持。
今回のruntimeコード・車両設定に変更なし。新しい全pytestは実行していない。
直前実行版の結果は1797 passed/4 skipped（前報）。

ローカル保存: `tmp/v4_pp_live_26`（試験script、source.tar、準備script）。
同 `evidence/` に `timing.jsonl`、`shadow.jsonl`、`probe_result.json`、
`host_result.json`、`live_connection_summary.json`、`per_plan_geometry.json`。
SSHの試験配置下 `evidence/` に全生ログを保存。

## 次の切り分け

保存済み168件で、折り返し・過大曲率が現れる点番号と実点間隔を確認し、
未補正経路そのものの形状と現在の離散曲率検査の影響を分離する。
設定を緩めたり経路を平滑化したりする前に、この不適合を説明する必要がある。
V4実走行への自動切替は行わない。
