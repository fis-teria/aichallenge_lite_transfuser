# TimePath ROS 2 / make dev

既存の `aic_e2e_runtime` に、時間基準モデルの推論とPure Pursuitをまとめる
`time_path_awsim.launch.py` を追加。旧V4/tinyの `make dev` 分岐は維持する。
モデルの再学習・予測経路の平滑化は行わない。

## AWSIMを起動する

実行先は `graneple@192.168.3.10`。準備済みdeploymentには既定でTimePathを選ぶ
Makefileを配置済み:

```bash
ssh graneple@192.168.3.10
cd ~/e2e_autonomous/time_path_dev_20260917
make dev MAX_SPEED_KMH=20 CORNER_MAX_SPEED_KMH=10
# 例: 全体15 km/h、コーナー8 km/h、動画も保存
make dev MAX_SPEED_KMH=15 CORNER_MAX_SPEED_KMH=8 TIME_RECORD_VIDEO=1
```

deployment内の `source_f235112` ディレクトリから直接起動する場合は
`make dev DEV_CONTROLLER=time ...` と指定する。deploymentのMakefileはこの
検証済みsourceを選び、指定された速度変数をそのまま引き渡す。

`tools/time_dev_runner.py` が既存の有限試験runnerを呼ぶ。通常RVizに
`/visualization/time_path/raw_path` を表示し、公式Start、1周判定、制動停止、
所有コンテナの終了を行う。1周または異常で終了、駆動上限600秒、外側上限720秒。
AWSIM本体・scene・センサは変更しない。今回試験中のLiDAR近接監視は記録のみの
`log_only_awsim_v1` を引き継ぐ。センサ/経路の鮮度、異常値、競合制御器、
速度超過、停止確認などの検査は維持する。実車向け起動ではない。

|変数|既定値|意味|
|---|---:|---|
|`MAX_SPEED_KMH`|20|全体の目標速度上限、km/h|
|`CORNER_MAX_SPEED_KMH`|10|コーナー内の目標速度上限、km/h|
|`TIME_RECORD_VIDEO`|0|1でAWSIMと通常RVizの動画を保存|
|`TIME_NPCS`|0|AWSIM内蔵NPC車両の台数、0〜3。自車のE2Eは1台のまま|
|`TIME_RUN_ID`|UTC時刻から生成|任意指定は `codex-time-...`、既存runの再利用禁止|
|`TIME_DEPLOYMENT`|sourceの親|installと `command_off_best.pt` を持つ専用deployment|
|`DISPLAY`|環境値、なければ`:0`|実際のデスクトップdisplay|

有効範囲は `0 < CORNER_MAX_SPEED_KMH <= MAX_SPEED_KMH <= 20`。
実速度超過の停止閾値は全体上限+1 km/h。内部計算はm/s。
上限値は維持速度や実速度の瞬時上限を保証する値ではない。
曲率、予測経路の残り長さに応じて、これより遅い目標を選ぶ。

NPC対応sourceを配置したdeploymentでは、次の条件で2台のNPCを追加できる:

```bash
make dev MAX_SPEED_KMH=20 CORNER_MAX_SPEED_KMH=10 TIME_NPCS=2 TIME_RECORD_VIDEO=1
```

既存AWSIMの `--npcs` を使用し、本体・sceneを編集しない。
NPCありでは `--collisions on` を明示し、公式Start前にUnityログで
要求台数の生成、`racing-line` モード、車両同士の衝突判定を確認する。
NPC速度と出現位置は既存AWSIMの実装に従う。起動確認だけで回避成功とは判定しない。

コーナー判定は予測経路の1 m以上の区間から測った曲率とPP追従曲率に基づく。
絶対曲率0.05 /m以上（半径20 m以下）でコーナー上限を全面適用。
0.025〜0.05 /mは直線上限から連続的に移行する。
接近時は応答遅延0.5 s、余裕0.5 m、計画減速度0.7 m/s²から手前の速度を制限する。
急曲率の横加速度上限1 m/s²や短い予測経路の制限も同時適用する。
これは縦方向速度計画であり、障害物回避や無接触の保証ではない。

## ROS 2 packageのみ起動する

公式Autoware環境と本packageのinstallをsourceしたコンテナ内で:

```bash
ros2 launch aic_e2e_runtime time_path_awsim.launch.py \
  checkpoint:=/time/command_off_best.pt \
  output:=/time/unique-run run_id:=codex-time-manual-shadow \
  max_speed_kmh:=15.0 corner_max_speed_kmh:=8.0
ros2 param get /time_path_controller max_speed_kmh
ros2 param get /time_path_controller corner_max_speed_kmh
```

ROS launch単体はshadow出力が既定。AWSIM、Start、走行権限ファイルは生成しない。
走行は上記makeを使う。makeからは `authorize_awsim_only:=true` として起動し、
従来の実センサ・graph確認と公式Startが完了した後に外側runnerが走行権限を渡す。
両ノードは別process、片方の終了時にはlaunch全体を終了する。

速度は起動時に設定し、実効 `trial_config.json` をrun内へ保存する。
ROS parameterは検証済み設定と一致するread-only値として公開する。
`ros2 param set` による走行中の変更は拒否し、値を変えて次のrunを起動する。
有効設定のSHAをcontrollerとhostで照合し、事後replayでも両速度を使う。
旧JSONには `speed_parameters` を追加しない限り旧計算がそのまま適用される。

## ビルド・検証

Windowsでコミットし `tools/sync_to_wsl.ps1` で同期後、native WSLで:

```bash
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
```

公式ROS環境をsourceしたLinuxコンテナ内（sourceはrepo全体の配置が必要）:

```bash
cd /time/source_<commit>/ros2_ws
colcon --log-base /time/build_log build --packages-select aic_e2e_runtime \
  --build-base /time/build --install-base /time/install
source /time/install/setup.bash
ros2 launch aic_e2e_runtime time_path_awsim.launch.py --show-args
# network none / ROS_DOMAIN_ID=93の隔離コンテナで非走行確認
python3 /time/source_<commit>/tools/check_time_dev_launch.py \
  --checkpoint /time/command_off_best.pt --output /time/launch_smoke
```

checkpoint、dataset、rosbag、build成果物はGitに含めない。
標準設定は `configs/control/time_path_dev.json` に置き、setup.pyがpackage shareへ同じファイルを配置する。

## 2026-09-17 実施結果

実行sourceは `f2351125a1b0f17db70f211a8cf7413349aa96cd`。
checkpointは従来のepoch 3、SHA-256
`1fe12ca066791d3eb8907c6921120e2cfbb38925c422d5be54940f4acbdd120a`。
Windowsコミットをnative WSLへ同期し、共有lock下で検証した。

- 全体pytest: **2935 passed, 4 skipped** / 236.06秒。4件は既存の任意依存・環境不足。
- 旧20 km/h設定の走行replay: **2569件一致**。既存設定に速度パラメータを足さなければ従来挙動を維持。
- 公式環境colcon build成功、sourceとinstallの246ファイルが一致。launchと標準JSONを含む。
- 隔離ROS接続smoke成功。合成センサ/経路、曲率減速、経路欠損、clock停止、速度超過の制動を確認。
- 新ROS launch: 全体12 km/h・コーナー8 km/hを実parameter serviceで読み出し、
  実効JSONとの一致、read-only、両processのheartbeat、actual-control publisher=0を確認。
- deploymentルートの `make -n dev MAX_SPEED_KMH=12 CORNER_MAX_SPEED_KMH=8` で両値の引き渡しを確認。
- **実際のmake devで全体20 km/h・コーナー10 km/hを指定し、AWSIM公式判定で1周133.67秒。**
  run IDは `codex-time-dev-lap01`。停止確認、所有コンテナ終了、通常RVizの経路表示まで成功。

|この1試行の値|結果|
|---|---:|
|走行中の実速度中央値（追従指令時、0.1 m/s超）|9.572 km/h|
|最大実速度|12.805 km/h|
|最大追従目標速度|13.481 km/h|
|PP追従曲率が0.05 /m以上の指令|1617件、全件で目標10 km/h以下|
|近接監視で停止相当と判定された指令|0件（監視は記録のみ）|
|実効パラメータ込みの制御replay|2832件一致、最大差1.073e-12|

replay不能5件は既存の `PLAN_STALE`。走行中には後輪横速度検査5件、
予測の初期方向検査10件の一時制動もあり、検査を外して成功扱いにしていない。
LiDARの停止判定自体をreplayで再認証したものではない。
1周完走は確認したが、物理衝突センサによる無接触認証や複数周の安定性評価は未実施。
速度12/8はROS起動検証であり、AWSIM走行確認した速度設定は20/10のみ。

AWSIMの1089ファイルは前後のSHA一致。既存Git差分、RViz設定、過去コンテナを保全。
AWSIM/RViz動画は165.1/165.8秒で、転送SHAと全フレームdecodeを確認済み。
rawと動画はWSLの `/home/thistle/e2e_autonomous/runs/time_path_dev_20260917` に保存。
小さな検証結果は [evidence](evidence/time_path_dev_20260917/manifest.json)、
[走行集計](evidence/time_path_dev_20260917/checks/dev_analysis.json) に保存。
