# LiDAR物体検出・V2X変換モジュール

`aic_lidar_v2x` は、LaserScan・時刻付きTF・既知の静的地図から物体候補を検出・追跡し、
MPPI V44と同じV2Xメッセージ型へ変換する独立ROS 2パッケージ。
既定は **shadow（検出の検証・収録用）**。
ユーザーの方針に従い、V44の既存衝突判定マージンをそのまま使う
`teacher_existing_margin` モードと `teacher.launch.py` を追加した。
教師モードはLiDARの検出位置を既存V2X入力と同じ他車近似へ渡す。
追加の楕円・物体寸法推定・中心補正は導入しない。

## 構成と入出力

1. `core.py`: ROS非依存。無効レンジの除去、ビーム時刻でのSE(2)補間、自己車体除去、
  地図壁の除去、距離クラスタリング、ID付き追跡、V2X形式への変換。
   壁除去前にも連結性を調べ、地図上の壁が短い残差へ分断されて車両候補になるのを抑制する。
2. `io.py`: ROS形式の地図YAML/画像、Reference CSV、平面TFの検証。
3. `node.py`: LaserScanとtf2を接続し、V2X・検出詳細・状態・通常RViz用MarkerArrayを配信。

| 入出力 | 内容 |
|---|---|
| 入力 `/sensing/lidar/scan` | `sensor_msgs/LaserScan`、range m、angle rad |
| 入力 `/tf`, `/tf_static` | 観測時刻の `map -> lidar` と `map -> base_link` |
| 入力 `map_yaml` | ROS trinary地図。unknown・地図外は物体候補にしない |
| 任意入力 `reference_csv` | `known_vehicle` モードの向き事前情報。教師専用 |
| 出力 `/collection/lidar_v2x/vehicle_positions` | `v2x_msgs/V2XVehiclePositionArray`、map座標 |
| 出力 `/collection/lidar_v2x/objects` | JSON。ID、観測サイズ、速度、中心表現、box fit誤差・曖昧さ |
| 出力 `/collection/lidar_v2x/status` | JSON。モード、入力有効性、欠損理由、教師入力状態 |
| 出力 `/collection/lidar_v2x/markers` | 通常RVizのMarkerArray表示用、観測した範囲を表示 |

native `/v2x/vehicle_positions` を入力に使わず、同トピックへも配信しない。
E2Eの入力・モデル・制御権限は変更しない。教師用の地図/Reference/検出結果を
学生モデルの入力へ混入させない。AWSIM本体・車両アセットの変更は不要。

## V44との契約確認

確認対象はSI26に保存された `mppi-sim-v44` のsourceと実際の`v2x_msgs`。

- `V2XVehiclePosition` はheader、vehicle_id、position、covarianceのみ。
  **この環境のcovarianceは標準偏差[m]であり分散[m²]ではない**。
  V44 `receiveVehiclePositions` もそのまま `sigma_x_m/sigma_y_m` として読む。
- V44はID文字列から履歴を持ち、位置差分から速度を推定する。
  このモジュールは `lidar_000001` 等を使い、同一プロセス内のresetでもIDを再使用しない。
- 大きさ・向き・物体分類はV2Xに格納できない。V44には固定の車体寸法を使う処理があり、
  recovery側にも別の障害物近似がある。今回は既存の他車近似と余白を共用する。
- 受信箇所はMPPI planner `input/vehicle_positions`、Reference選択
  `input/vehicle_positions`、recovery `input/vehicles`。
  `teacher.launch.py` ではこれらを専用トピックへ一括で切り替える。
  nativeと合成データを同じトピックから交互配信すると、recoveryのactive IDリストが置き換わる。

## 検出の2モード

- `surface`（既定・教師モード採用）: 観測点のXY外接矩形中心を出す。
  車両の中心位置とは同一ではないが、今回の方針ではこの位置を既存マージン付き判定へ渡す。
- `known_vehicle`（実験用・今回の保存走行では採用不可）: 寸法既知・コースに沿うNPCという収集条件を明示して使う。
  既定2.064m × 1.30m、ReferenceのXYから求めた向きで矩形を置き、
  センサから矩形への最初の交点と実測レンジを照合して中心を推定する。
  物体分類器ではないため、任意の箱や壁を車に見立てた正しさは保証しない。
  一面しか見えない場合の中心の曖昧さを別の値として記録する。
  寸法や曖昧さをcovarianceへ偽装して埋め込まない。

位置標準偏差0.15mの設定は継続する。V44の既存計算がこの値を扱い、別の余白を上乗せしない。
壁の除去余白0.25mは壁際の物体も除去し得る。未観測の奥行きや隠れた物体を復元する機能はない。
走査平面より低い物体など、2D LiDARが検出できない物体も対象外。

`motion_model=rolling` はLaserScanのtime_incrementに従い、最初と最後のビーム時刻のTFで補間。
`snapshot` は全ビームをheader時刻の同時観測として扱う。実センサ/シミュレータの仕様に合わせて選ぶ。
TFがない場合に最新TFや単位変換へ置き換えない。大きいroll/pitchは平面モデルの範囲外として拒否する。
tf2が「最新」と解釈するstamp=0も拒否する。
NaN等の不正レンジが20%を超えるscanは、物体ゼロとして配信せず入力異常にする。
観測欠落時に位置・時刻を捏造して配信せず、再同定用の履歴だけ0.5s残す。

## 実行

Windowsで当該変更をコミットし、`tools/sync_to_wsl.ps1`で同じcommitをWSLへ同期する。
既存の別タスクのindexがある場合は保全し、当該commitのcleanなWindows transport cloneから同期する。

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q

tools/with_wsl_training_lock.sh .venv/bin/python tools/replay_lidar_v2x.py \
  --bag /path/to/native-wsl/rosbag2_autoware \
  --map-yaml /path/to/occupancy_grid_map.yaml \
  --output /path/to/native-wsl/runs/lidar_v2x_surface

tools/with_wsl_training_lock.sh .venv/bin/python tools/replay_lidar_v2x.py \
  --bag /path/to/native-wsl/rosbag2_autoware \
  --map-yaml /path/to/occupancy_grid_map.yaml \
  --reference-csv /path/to/in_corce_line.csv \
  --object-model known_vehicle \
  --output /path/to/native-wsl/runs/lidar_v2x_vehicle
```

ROS 2 HumbleとV44と同じ`v2x_msgs`をsourceした環境で、独立パッケージとしてビルドする。
build/install/logはWSL native側（AWSIMホストへの適用時は専用実験領域）に置く。
WSLで実施する場合は、以下のビルド・ROS起動も `tools/with_wsl_training_lock.sh` 経由で実行する。

```bash
colcon build --base-paths /path/to/e2e_lite_transfuser/ros2_ws/src/aic_lidar_v2x \
  --build-base /path/to/native/build --install-base /path/to/native/install
source /path/to/native/install/local_setup.bash
ros2 launch aic_lidar_v2x shadow.launch.py \
  map_yaml:=/path/to/occupancy_grid_map.yaml
# 既知NPCモデルを比較する場合はreference_csvとobject_model:=known_vehicleを指定。
```

V44の教師overlayをsourceした状態で、既存の教師起動を次に置き換える。
AWSIMやE2Eの起動・制御権限は既存の収集ハーネスが担当する。

```bash
ros2 launch aic_lidar_v2x teacher.launch.py \
  map_yaml:=/path/to/occupancy_grid_map.yaml \
  domain_id:=1 speed_cap_mps:=2.7777777777777777 run_rviz:=false
```

既存の衝突判定パラメータは変更しない。
`obstacle_longitudinal_inflation_m=1.10`、`obstacle_lateral_inflation_m=1.15`、
`clearance_target_m=0.35`、車体の半幅0.65m・前方1.06m・後方1.10mを維持する。
これらは単純な「V2X点を半径1.15mに膨らませる」指定ではない。
V44は自車・他車の寸法を姿勢に応じて投影した範囲と設定値を比較し、不確かさも含めて判定する。
復帰制御は従来の相手車両半径設定（simulation configは0.65m）を使う。

`teacher_ready` は教師モードで入力が有効な状態を表す。回避成功の証明ではない。
LiDAR/TFなどの有効入力が0.75s途絶えた場合（初回入力は起動後15s待機）、
既存の `/control/mpc/stop_request` に通知する。V44の停止はラッチされ、同じrun内で自動解除しない。
これは入力欠損の扱いであり、新しい障害物停止領域は設けない。

隔離したROS環境での有限通信試験は、ビルド済みoverlayをsourceして次を実行する。
実行ホストのnative V2Xや車両に届かないネットワークで行う。

```bash
ROS_DOMAIN_ID=97 python3 /path/to/aic_lidar_v2x/test/smoke_ros.py
ROS_DOMAIN_ID=97 python3 /path/to/aic_lidar_v2x/test/smoke_ros.py --teacher-existing-margin
```

同じ隔離環境で `teacher.launch.py domain_id:=97 ...` を起動中に、接続先と実パラメータを検査する。
headlessな検証コンテナでは `QT_QPA_PLATFORM=offscreen` を指定する。

```bash
ROS_DOMAIN_ID=97 python3 /path/to/aic_lidar_v2x/test/check_teacher_graph.py
```

SI26の既存devイメージでは、既定のCycloneDDS設定が別マウントを参照していた。
検証は `docker --network none` 内でloopback専用のDDS設定を与えて実施した。
`ROS_LOCALHOST_ONLY=0` と以下の `CYCLONEDDS_URI` をこの隔離コンテナ内だけで使う。
ホストや既存のAWSIM/ROS環境の設定は変更しない。

```xml
<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" multicast="false"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>
```

RVizのFixed Frameを `map` にし、MarkerArrayに `/collection/lidar_v2x/markers` を指定する。
rosbagには元のCamera/LiDAR/TF/ego/教師軌跡/native V2Xと、上記4出力を収録する。
学習用future trajectoryは従来どおり観測した将来poseから作り、run/配置シナリオ単位でsplitする。

## 採用前に残る検証

単体試験と保存走行のreplayに加え、公式ROS環境での通信・欠損試験、native V2Xとの
比較、壁際/コーナー/遮蔽での見失いを確認する。
教師モードは既存shape近似・マージンと入力欠損時の既存停止要求を使用する。
`input_valid=true` は変換できた意味であり、走行可能・障害物不存在・教師収集許可を意味しない。
replayが成功しても、閉ループ回避走行や学習データ採用の証明にはしない。

## 保存走行での結果（2026-09-18）

対象は `static-v44-straight-b-01`、静止NPC 1台、LaserScan 1,642件。
rolling処理できたのは1,633件。開始時TF不足8件とbag末尾1件を除外した。
native V2Xは照合専用で、検出器・追跡器への入力には使っていない。

照合母集団は「センサから前方2–15m・方位±1.5rad内にnative V2X位置があり、
その位置の半径1.5m内にLiDAR点が4個以上ある」472 scan。
位置差2m以内を対応候補とするため、以下は**人手正解に対する検出再現率ではない**。

| モード | 対応候補 / 母集団 | native V2X位置との差・中央値 / P95 | 処理時間P95 |
|---|---:|---:|---:|
| surface・rolling | 461 / 472 | 0.555 / 0.788 m | 14.66 ms |
| surface・snapshot | 461 / 472 | 0.558 / 0.790 m | 13.77 ms |
| known_vehicle・rolling | 167 / 472 | 0.954 / 1.136 m | 17.12 ms |

処理時間はWSLでの検出・追跡・payload作成のCPU時間で、DDS通信・TF待ち時間は含まない。
このshadow実装時点の全体pytestは **3,017 passed / 4 skipped**、追加の数値テストは21件。
skipは既存のOSQP、jsonschema関連、公式Tiny packageの不足によるもの。
ROSパッケージのビルドと有限通信試験も成功し、型・標準偏差の単位・ID・TF欠損・
NaN入力・scan timeout・native V2X非配信を確認した。
使用commit、入力SHA、各比較条件、試験集計は [検証記録](lidar_v2x_validation.json) に保存した。

表面候補の検出はできたが、表面中心を車両中心として流用すると約0.55mのずれが残る。
矩形の既知寸法による中心補正はこのデータでは悪化したため採用しない。
走査時間モデルを切り替えても、この位置差はほぼ変わらなかった。
surfaceの対応候補には5個のtrack IDが使われ、静止NPCの見かけの追跡速度P95は0.687m/s。
この差と追跡の揺れは残る測定結果として保存する。今回のユーザー方針では
中心の再推定を待たず、既存マージンを使う教師モードを実装した。

壁処理の改善前後で、上記母集団における対応先のないtrack出力は234件→26件へ減った。
これは余計な候補の減少であり、全物体の正解がないため誤検出率とは呼ばない。
同じ1走行を改善にも使っているため、別の配置・コーナーでの独立評価が必要。

shadowでは既存V2Xの教師と比較でき、teacherではLiDAR V2Xを教師に渡せる。
視点ごとの位置差やID継続は、教師モードでの回避結果と合わせて評価する。
再学習には成功した教師走行を選別して使い、検出結果だけを正解経路にしない。

WSLのraw replay・全scan出力は `/home/thistle/e2e_autonomous/runs/lidar_v2x_20260918/`。
ROS通信試験はSI26の専用 `ai-work/raw/lidar_v2x_20260918/` で実施し、
AWSIMを起動せず、native V2X publisherを増やさず、運転指令を配信していない。

## 既存マージンを使う教師接続の検証（2026-09-18）

`teacher_existing_margin` を追加した。LiDARのsurface位置を既存のV2X型で渡し、
MPPI V44の衝突包絡・車体寸法・clearance設定をそのまま使用する。
V44本体の実行ファイル・共有ライブラリのSHA-256も保存済みV44と一致した。

SI26の公式devイメージを `--network none`・domain 97で起動し、次を確認した。

- MPPI planner、基準経路生成、復帰制御の3ノードが
  `/collection/lidar_v2x/vehicle_positions` を購読し、native V2Xの購読が残っていない。
- 同topicのpublisherは `lidar_v2x` の1個。V2XのRViz表示も同じ入力を購読する。
- 実ノードのマージン・車体寸法パラメータ6個は上記の既存値と一致する。
- 合成scan/TFの有限通信試験で104配信・102非空配信、IDは1個。
  TF欠損・NaN入力・scan timeoutを検出し、教師モードの入力欠損時に既存停止要求を発行した。

WSLでの回帰試験は **3,030 passed / 4 skipped**。
LiDAR V2X単体24件も成功した。4件のskip理由は上記と同じ。
検証commitは `986cbdbd038196c8c9b1f3978a65342e750812f0`、ROS試験のpackage treeは
`7f23cd8` と同一である。
並行作業によるcheckout変更を避けるため、同一commitを専用のnative WSL cloneへ同期し、
既存venv・検証用データを使用した。必要な旧checkpoint 2個はコピー後にSHA-256を照合した。

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser_lidar_v2x_margin
bash tools/with_wsl_training_lock.sh env PYTHONPATH="$PWD/src" \
  .venv/bin/python -m pytest -q
```

使用commit、ログSHA-256、ROS接続・実パラメータ・V44バイナリの記録は
[教師接続の検証記録](lidar_v2x_teacher_validation.json) に保存した。
従来の [shadow検証記録](lidar_v2x_validation.json) は履歴として保全している。

この接続試験の後、次節のAWSIM回避収集を実施した。
接続と設定維持だけでは、表面位置のずれを既存マージンで常に吸収できる証明にはならない。
AWSIM本体・センサ取付TFは変更していない。

## AWSIMでの教師収集（2026-09-18）

SI26で停止NPCを障害物として、直線・コーナー入口・出口の8配置を9回試行した。
教師はLiDAR V2Xを入力するMPPI V44、速度上限10 km/h、既存マージンを維持。
E2Eは推論のみのshadow。箱・コーン・歩行者・移動車両はこのバッチに含まない。

- 実走6回: 無接触かつ地図内で通過した2回を採用候補とし、4回を接触・はみ出し・未通過で除外。
- 起動不良3回: `PlayStart`から進まず、学習候補から除外。
- 前進回避候補446観測: 直線B中央203、コーナー入口B左243。独立した成功走行は2回。
- 全585ファイル、2,398,503,242 bytes（約2.40 GB）をnative WSLへ移し、全SHA-256を照合。
- rawはCamera 8,145観測（実走bagは7,588）、LiDAR 17,108 scan。失敗・後退もrawには保管。

将来教師は実測poseの30点×0.1 s。共有の時間履歴選択・教師生成を用い、
前1 s〜後3 sに後退が混ざる観測、履歴/未来不足、緊急停止、未通過・接触したrunを除く。
`stop_probability`は停止意図が不明なため未設定。既存の学習splitへの統合・再学習は未実施。
runと関連配置sectorを分離せず扱い、frameランダムsplitをしない。

実行中の`brain.follow_gap_m`は5.0 m。静止物・20 km/h以下の相手が基準経路を塞ぐとき、
基準経路へ投影した自車前端と推定相手後端の距離が5 m以内でAVOID候補を要求する。
進行中のOVERTAKE等の状態条件もあり、検出距離25 mとは別。今回この開始条件は変更していない。
接触や停滞は残っており、5 m条件だけが原因とは未確定。成功した出口の回避データは未取得。

保存先は`/home/thistle/e2e_autonomous/runs/lidar_v2x_obstacles_20260918/`。
`collected/<run_id>/`にraw・provenance・転送manifest、`audits/<run_id>/`に
`audit.json`・`anchors.jsonl`・`observed_teachers.npz`を置く。
`summary/training_candidates.jsonl`が採用候補の索引、`summary/collection_paths.png`が実走図。
再実行スクリプトと転送確認記録は`reproduction.zip`に保存し、同じくhash照合済み。

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser_lidar_v2x_margin
# 出力先は未作成のディレクトリを指定する。
bash tools/with_wsl_training_lock.sh env PYTHONPATH="$PWD/src" \
  .venv/bin/python tools/audit_lidar_v2x_obstacles.py \
  --collected /path/to/collected/run \
  --output /path/to/new_audit
```

監査commit `89622a6faa6f16400a6e5a7c06f6b9287476c5b3`のWSL全体試験は
**3,042 passed / 4 skipped**。実データでも入力tensor・教師軌道の組立を確認した。
起動前終了の3 bagはraw保存・索引化のみとし、実走6 bagを時間教師の監査へ通した。
既存monitorの結果書込後exit 139は残るため、最終状態とbag完了・decodeを別途確認する。
AWSIM 1,092ファイル・V44ソース926ファイルに変更がないことを照合し、専用コンテナは停止済み。
[各試行・保存先・検証記録](lidar_v2x_obstacle_collection_validation.json)に詳細を記載した。
これはMPPI教師の収集であり、E2Eの閉ループ回避性能を示す結果ではない。
