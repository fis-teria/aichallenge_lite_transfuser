# LiDAR + wheel odometryによる地図位置合わせ

## 現在の状態

**検証用の実装は追加済み。実データの全周追従には未達のため、既定起動には適用していない。**
2026-09-18、runtime commit `986cbdbd038196c8c9b1f3978a65342e750812f0`。
固定のLiDAR取付TFを維持し、地図と車輪オドメトリの間だけを照合で補正する。
GNSS/IMU、記録済み自己位置、既存TFをlocalizerの入力に使わない。

通常RVizの左右境界を使った診断から進めた実装である。
[先行する診断と地図の制限](lidar_map_alignment_20260918.md)も参照。
前回の「記録済み自己位置を使う固定補正」の結果は、今回のGNSS-free追従の証明ではない。

## データの流れ

```text
VelocityReportの前進速度 + SteeringReportの実操舵角
  -> 既存TimeControlOdometry -> /time_path/wheel_odometry
  -> LiDAR地図照合の予測
LaserScan + Lanelet2左右境界 + 手動初期位置
  -> 境界への照合 -> map -> time_wheel_odom の補正
```

- 車輪オドメトリ本体は連続のまま。補正値を車輪積分へ戻さない。
- `time_path_controller` は従来と同じ入力で制御し、既存局所姿勢をOdometryとして追加配信する。
  Odometry covarianceは未較正として大きな値を設定し、精密な絶対位置とは扱わない。
- E2Eモデル、waypoint追従、車両指令の入力は変更していない。
- 地図位置合わせノードは車両指令を配信しない。
  将来の地図ベース回避・停止をこの出力へ接続する処理は未実装。

専用TFは次の構成。既存EKFの `map -> base_link` と親を競合させない。

```text
map -> time_wheel_odom -> time_localized_base_link -> time_localized_lidar
```

最後の取付TFは前方1.65 m、高さ0.0377 m、回転0。
元の `base_link -> lidar` も変更しない。
推定は平面XY/yawのみ。表示のZには地図境界の標高中央値を使うため、車体の実測高さではない。

## 入出力と無効化

|方向|topic|契約|
|---|---|---|
|入力|`/clock`|シミュレーション時刻。停止・巻き戻りを検出|
|入力|`/sensing/lidar/scan`|`lidar` frame、有限レンジを抽出、0.3〜15 m|
|入力|`/time_path/wheel_odometry`|`time_wheel_odom` / `base_link`、速度・操舵由来|
|手動入力|`/time_path/localization/initialpose`|map上の初期XY/yaw。GNSS初期化やゼロ姿勢fallbackなし|
|出力|`/time_path/localization/scan`|照合有効時のみ。距離値は保持し専用LiDAR frameで表示|
|出力|`/time_path/localization/pose`|照合有効時のみの平面base推定|
|出力|`/time_path/localization/status`|`valid`、状態、対応点率、誤差、地図hash、補正値|

`valid=false` のとき補正値はnullで、新しいTF・整列点群・poseを配信しない。
TF履歴の古い値を最新の有効位置として流用しないこと。RVizは点群を0.5秒で消す。
状態名だけでなく **`valid` と時刻の鮮度を必ず確認する**。

直線の前後方向は退化し得る。強い2方向だけの場合は `DEGRADED` として示し、
弱い曲がり形状の情報は正則化して使う。境界線分の端点も制約に含める。
対応点探索の範囲と採用判定は別で、最終的に0.3 m以内の対応点が40点以上、
全有効抽出点の55%以上あることなどを要求する。地図にない物体も母数に含める。
近傍の再照合で補正が大きくなる場合は、初期化と同じ範囲内に限定し、
3スキャンの再確認前には出力を再開しない。1秒以上照合が成立しなければ再初期化を要求する。

これらは検証段階の自己位置品質判定であり、物理的な安全距離の認証ではない。
現在のE2E制御はこのlocalizerを入力にしないため、localizerのロスト自体では車両を停止させない。
地図回避を接続する際は、ロスト時の停止判断を別途Safety Supervisorへ結線する必要がある。

## 検証結果

native WSLで既存を含む `pytest -q` は **3030 passed, 4 skipped, 84 warnings**
（117.52秒）。localizerの単体試験10件も成功。
skipはOSQP、JSON schema validator、任意の公式package不足によるもの。
[検証記録とファイルhash](evidence/lidar_map_runtime_20260918/manifest.json)を保存した。

公式Humble環境でpackage build・installed source hash一致を確認。
既存TimePathのROS接続smokeとlaunch smokeは成功。
localizer単独の隔離ROS試験も成功し、次を確認した。

- LiDAR・車輪Odometry・時計・手動初期位置の4購読だけで動作。
- 合成の既知位置 `(3, 1) m` を復元し、専用TFと29スキャンを配信。
- scan欠落、不正frame、clock停止で出力を無効化。
- 手動再初期化から復帰。
- GNSS/IMUおよび継続的な外部自己位置の配信なし、車両指令publisher=0。

実データの再生は、元bagから時計・scan・速度・操舵の4topicだけを読み、
同じ車輪オドメトリを再構築した。初期位置は起動時に明示した固定の概略値。
これは初期位置指定なしの大域探索ではない。

|既存bag|結果|
|---|---|
|`laps03/5kmh_run01`|2029照合時刻中、有効198（9.76%）。最初の曲がり付近で補正量超過、以後再初期化待ち|
|`laps03/8kmh_run01`|冒頭の速度・操舵stamp結合が `STEERING_JOIN_TIMEOUT`。照合開始前に終了|

5kmh runの有効区間での平均対応点距離は約0.055 mだが、これは採用された対応点だけの値。
**全周精度として使ってはいけない**。元の全点に対する距離も各recordへ保存している。
ロストを成功に数えず、両runとも `PARTIAL_OR_FAILED` と記録した。
初期版の対応点選択・コーナー拘束・再確認処理も検証したが、全周追従には至っていない。
補正閾値を外して見かけだけ合わせる変更はしていない。

現時点で、コーナーの対応付け、道路境界と物理壁の違い、弱い前後方向の推定が未解決。
原因を地図誤差だけに断定しない。次段階は、scan間のLiDARオドメトリと実測壁面地図を
用意し、既存道路境界への位置合わせと局所的な壁面追従を分けて検証すること。
今回のデータでは常用できる品質に達していないため、既定の `make dev` は変更していない。

## 再現コマンド

Windowsのコミットを通常のsync手順でnative WSLへ同期後:

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  tools/replay_lidar_map_localization.py \
  --run /home/thistle/e2e_autonomous/runs/time_teacher_si26_20260911/laps03/5kmh_run01 \
  --map /home/thistle/e2e_autonomous/runs/lidar_map_alignment_20260918/lanelet2_map.osm \
  --initial-pose 89631 43128 2.12 \
  --output /home/thistle/e2e_autonomous/runs/lidar_map_runtime_20260918/new_replay
```

この初期値は当該保存データのスタート付近を指定する診断用で、別の開始位置へ流用しない。
既存outputは上書きしない。

ROSでは新版TimePath controllerが `/time_path/wheel_odometry` を配信している環境で:

```bash
ros2 launch aic_e2e_runtime lidar_map_localization.launch.py \
  map:=/aichallenge/workspace/install/aichallenge_submit_launch/share/aichallenge_submit_launch/map/lanelet2_map.osm
rviz2 -d /time/install/aic_e2e_runtime/share/aic_e2e_runtime/config/lidar_map_alignment.rviz
```

専用RVizの `2D Pose Estimate` で、停止中の概略位置・向きを設定する。
通常RVizの初期位置topicとは別なので、通常EKFへ誤送信しない。
ROSだけの試験は以下を `--network none` の隔離環境で実行した。

```bash
python3 tools/check_lidar_map_ros.py --output /time/new_map_smoke
```

検証用deploymentはホストの `/home/graneple/e2e_autonomous/time_lidar_map_20260918_r2`。
既定入口の転送先は従来の `time_no_gnss_20260918_r2` のまま。
初回deployment試験ではarchiveからtests fixtureが欠けて接続smokeが終了した。
fixtureを含む別deploymentを作り直して再試験し、古い失敗ログは保全した。
新localizerによるAWSIM実走・地図回避・停止制御への採用は未実施。
