# TimePath + local SLAM MPPI avoidance

TimePathの予測経路を通常追従し、進路上の障害物がある間は局所SLAM地図で
**reference-space MPPI**を実行する。初期版はAWSIM・静止障害物・5 km/h専用。
既存の`time_path_dev.json`とSLAM減速専用プロファイルの既定動作は変更しない。

## 構成

- Cartographer: LiDARと車速・操舵オドメトリによる位置合わせ。GNSS/IMU、事前地図、
  シミュレータ物体座標、V2Xを制御入力に追加しない。
- `slam_obstacle_node --mppi`: 観測と同じ時刻・座標で障害物と回避経路をまとめて配信。
  `/time_path/slam/obstacles`のJSONに`mppi`を追加。制御指令は発行しない。
- `time_trial_controller_node`: 最新モデル経路を従来どおり検査し、正常な場合に限り
  MPPIの経路へ切り替える。scan時刻の車輪オドメトリで現在座標へ移し、PPで追従。
  物理操舵角から既存の操舵応答補償・レート制限へ接続する。
- 独立LiDAR guard: `stop_v1`と`measured_speed_v1`を必須とし、実速度で停止領域を検査。
  既存の1 m制限・記録のみの監視をこのプロファイルに使わない。

通常追従 → 障害物検出 → MPPI回避 → 新鮮なクリア観測0.5秒以上かつ
基準経路への位置・向きの復帰 → 通常追従。回避中の左右の頻繁な切り替えを抑え、
同じ側で通れなければ停止する。停止中も新鮮な観測で再計画する。
地図や経路の欠落・鮮度切れ、計算時間150 ms超過、候補不成立は停止要求。
SLAM停止は0.35秒、元モデルの観測は0.5秒の既存鮮度契約を引き継ぐ。

## 計画の範囲と制限

ROS非依存実装は`src/aic_transfuser_lite/control/slam_mppi.py`。
256候補×3反復で横方向オフセットをサンプルし、コストの指数重みで更新する。
これは局所的なreference-space MPPIで、同梱V45教師C++ノードそのものの再利用ではない。
[MPPIの指数重み更新](https://acdslab.github.io/mppi-generic-website/docs/mppi.html)を
参照し、横方向の2係数に制限した探索を行う。

オフセットは現在の横ずれ・向きから滑らかに始まり、予測経路へ合流する。
経路は最大16 m、元モデルが予測している範囲に限定し、外挿しない。
基準経路からの横ずれ上限2.5 m、操舵角上限0.3 rad、回避速度上限5/3.6 m/s。
全候補と重み付き平均の候補に、曲率と車体寸法を含む占有地図検査を行う。
平均した経路が通れるとは仮定せず、検査済みの候補だけを採用する。
RVizの`/time_path/slam/path`には採用した回避経路を表示する。

未観測セル・地図外は通行不可。占有地図のTTLは既存の2秒。
空間の観測だけでは路面や法的な走行可能領域を判定できないため、
**2.5 mの範囲制限はコース内保証ではない**。走行可能領域の意味情報が必要な場面は別途検証する。
移動物予測は未実装で、NPC/背景車付きの起動をこの初期版では拒否する。
予測経路が短い・遮蔽が大きい・車体余裕が不足する場合は、回避できず停止する。
安全余裕を減らして回避成功に見せない。

## 起動

新ソースを専用deploymentへ配置し、公式ROS環境で`aic_e2e_runtime`を再ビルドする。
既存のCartographer overlayとcheckpointが必要。旧installを上書きせず、
`docs/time_path_make_dev.md`のビルド・所有プロセス管理手順に従う。
以下は**新版を配置・ビルドしたdeployment内**のコマンド。

```bash
make dev DEV_CONTROLLER=time TIME_SLAM_MPPI=1 MAX_SPEED_KMH=5 CORNER_MAX_SPEED_KMH=5

# 物理箱の固定シナリオを指定する有限試験。結果は成功と仮定しない。
python3 tools/run_time_path_awsim_trial.py \
  --deployment /home/graneple/e2e_autonomous/NEW_MPPI_DEPLOYMENT \
  --run-id codex-time-slam-mppi-box01 --display :0 \
  --config configs/control/time_path_slam_mppi.json --slam-obstacles \
  --static-obstacle-scenario configs/scenarios/slam_stop_box.yaml
```

通常入口へ戻す場合は`TIME_SLAM_MPPI=0`。異なる速度や交通条件を暗黙に採用しない。
この初期版は専用固定速度プロファイルを使い、TimePathの可変速度ROS launchを介さず
既存の有限試験runnerから同じ推論・制御ノードを起動する。

## 検証と未確認事項

```bash
# Windowsでcommitし、tools/sync_to_wsl.ps1で同一commitを同期後
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_slam_mppi.py
```

合成占有地図で箱の回避・基準経路への合流、通路閉塞と未観測領域の拒否、
座標変換、異常・古い入力、速度・起動設定を検証する。理想運動モデルの連続再計画試験は
AWSIMの操舵遅れ・タイヤ・SLAM誤差・未知の障害物運動を再現した試験ではない。

実行中は`slam_obstacles.jsonl`の`mppi`と`control.jsonl`の`details.slam_mppi`、
実際の指令・停止理由を照合する。回避/停止の指令理由を通常追従と区別して記録する。
従来のPP replayは地図とMPPIを再現しないため、この構成は`UNSUPPORTED_SLAM_MPPI`を返す。

現時点では公式ROS/AWSIMでの起動・衝突なし回避・周回は未確認。
部署先の旧source/installは自動的には更新されない。
