# LiDAR + wheel odometryによる地図位置合わせ

## 現在の状態

**検証用の実装は追加済み。実データの全周追従には未達のため、既定起動には適用していない。**
後続評価では強補正の有効出力も記録済み自己位置から大きく逸脱した。
同参考基準に対し位置0.5 mかつ向き5°以内は0/706、有効出力の位置差中央値41.55 m。
これは真値評価ではないが、照合有効率34.8%を位置精度の改善と解釈してはいけない。
[参考基準との比較](lidar_map_reference_evaluation_20260918.md)を参照。
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

### シミュレーションで強く補正するモード

2026-09-18のユーザー指示に基づき、補正量による拒否を外した比較モードを追加。

|correction_mode|補正量上限|照合探索半径・反復数|1秒以上不成立の後|
|---|---|---|---|
|`bounded`|通常0.45 m / 0.10 rad、初期化0.90 m / 0.18 rad|0.8/1.2 m・12回|手動再初期化|
|`unlimited`|なし|既存と同じ。上限だけを外す比較用|手動再初期化|
|`simulation_aggressive`|なし|3.0 m・40回|車輪予測を維持して自動で再照合|

`simulation_aggressive` は解法の1反復の更新幅を0.5 m / 0.10 radへ拡大する。
これは総補正量の採用上限ではない。欠測後や照合失敗後は3回連続成立で有効出力へ戻す。
点群の対応率・残差・有限値の条件は共通で、成立しないscanを成功扱いしない。
補正量は必ず記録する。大きな補正を採用しても地図上の正解位置を保証するものではない。

再生コマンドに `--correction-mode simulation_aggressive`、ROS起動には
`correction_mode:=simulation_aggressive` を付ける。同じ入力で上限だけの影響を見る場合は `unlimited`。
既存起動との比較のため、引数を省略した場合は `bounded`。

同一の5kmh bag・地図・初期値での比較結果（2026-09-18）:

|モード|有効な照合時刻 / 全2029回|結果|
|---|---|---|
|従来 `bounded`|198 / 2029（9.76%）|45.789秒で補正拒否|
|`unlimited`|292 / 2029（14.39%）|元の拒否地点を通過。64秒付近で対応点不足|
|`simulation_aggressive`|706 / 2029（34.80%）|72秒付近まで連続、その後も自動再照合して断続的に復帰|

強補正では45.789秒に約1.35 mの補正を採用して照合を継続した。
採用した補正の最大は位置2.044 m、向き16.668°。
補正量超過による拒否は0回となり、残る不成立1307回は `INSUFFICIENT_SUPPORT`。
例として72.442秒では0.3 m以内に重なる点の割合が54.4%で、共通条件55%を下回った。
不成立後も車輪予測で探索を続け、最終415.097秒にも有効出力へ復帰している。
**有効率は判定を通過した割合で、正解位置に一致した割合ではない。全周安定追従は未達。**
保存点群を地図境界に強く寄せるため、外れた境界への対応付けが正しいかは引き続き未確認。
無効区間を跨ぐ線は実走軌跡とは扱わず、比較図では有効時刻を帯として表示する。

[比較図・有効区間](evidence/lidar_map_aggressive_20260918/comparison.png)と
[数値](evidence/lidar_map_aggressive_20260918/comparison.json)を保存。
演算本体のreplay commitは `97744de2af412b989df1c033cdcc1c0884b75bd4`、
ROS試験用commitは `b3b0263a16efef581cccfc022ca20914e67599d4`。
localizer単体14件成功、全体 `pytest -q` は **3046 passed, 4 skipped, 84 warnings**（133.13秒）。

公式ROS環境でbuild・248 Python sourceのinstalled hash一致・launch引数を確認。
従来モードと強補正モードの隔離ROS smokeが成功し、強補正では約2 mの初期位置ずれの補正と、
scan途絶後の手動seedなし自動復帰を確認した。初回配送はconfigs不足でbuild失敗したため、
不足ファイルを含む別directoryへ再配送し、初回ログを残した。
検証用配置は `/home/graneple/e2e_autonomous/time_lidar_aggressive_20260918_r2`。
通常の `make dev` の転送先は変更しておらず、このモードでのAWSIM実走は未実施。

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
