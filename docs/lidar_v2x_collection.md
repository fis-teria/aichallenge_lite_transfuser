# LiDAR物体検出・V2X変換モジュール

`aic_lidar_v2x` は、LaserScan・時刻付きTF・既知の静的地図から物体候補を検出・追跡し、
MPPI V44と同じV2Xメッセージ型へ変換する独立ROS 2パッケージ。
現段階の起動モードは **shadow（検出の検証・収録用）**。
`teacher_ready=false` を明示し、運転ノードの起動や入力の自動切替は行わない。

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
| 出力 `/collection/lidar_v2x/status` | JSON。入力有効性、欠損理由、`teacher_ready=false` |
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
  recovery側にも別の障害物近似がある。**任意サイズの障害物を安全に表現できる契約ではない**。
- 受信箇所はMPPI planner `input/vehicle_positions`、Reference選択
  `input/vehicle_positions`、recovery `input/vehicles`。
  将来教師入力に採用する場合、これらを専用トピックへ一括で切り替え、実グラフで確認する。
  nativeと合成データを同じトピックから交互配信すると、recoveryのactive IDリストが置き換わる。

## 検出の2モード

- `surface`（既定）: 観測点のXY外接矩形中心を出す。任意物体の候補を保持できるが、
  **車両の中心位置ではない**。V2X出力は互換性検証用。
- `known_vehicle`（実験用・今回の保存走行では採用不可）: 寸法既知・コースに沿うNPCという収集条件を明示して使う。
  既定2.064m × 1.30m、ReferenceのXYから求めた向きで矩形を置き、
  センサから矩形への最初の交点と実測レンジを照合して中心を推定する。
  物体分類器ではないため、任意の箱や壁を車に見立てた正しさは保証しない。
  一面しか見えない場合の中心の曖昧さを別の値として記録する。
  寸法や曖昧さをcovarianceへ偽装して埋め込まない。

位置標準偏差0.15mは未較正の設定値。V44へ実制御入力として採用する前に較正する。
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

隔離したROS環境での有限通信試験は、ビルド済みoverlayをsourceして次を実行する。
実行ホストのnative V2Xや車両に届かないネットワークで行う。

```bash
ROS_DOMAIN_ID=97 python3 /path/to/aic_lidar_v2x/test/smoke_ros.py
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
教師切替にはshape表現と検出欠損時の停止/収集除外の接続が必要。
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
最終ソースの全体pytestは **3,017 passed / 4 skipped**、追加の数値テストは21件。
skipは既存のOSQP、jsonschema関連、公式Tiny packageの不足によるもの。
ROSパッケージのビルドと有限通信試験も成功し、型・標準偏差の単位・ID・TF欠損・
NaN入力・scan timeout・native V2X非配信を確認した。
使用commit、入力SHA、各比較条件、試験集計は [検証記録](lidar_v2x_validation.json) に保存した。

表面候補の検出はできたが、表面中心を車両中心として流用すると約0.55mのずれが残る。
矩形の既知寸法による中心補正はこのデータでは悪化したため採用しない。
走査時間モデルを切り替えても、この位置差はほぼ変わらなかった。
surfaceの対応候補には5個のtrack IDが使われ、静止NPCの見かけの追跡速度P95は0.687m/s。
**中心の定義とID/速度の安定化を解決するまで、教師V44の走行入力へ切り替えない。**

壁処理の改善前後で、上記母集団における対応先のないtrack出力は234件→26件へ減った。
これは余計な候補の減少であり、全物体の正解がないため誤検出率とは呼ばない。
同じ1走行を改善にも使っているため、別の配置・コーナーでの独立評価が必要。

当面は既存V2XでMPPI教師を走らせ、このモジュールを観測専用として同時収録する。
次の判断材料は、NPCの実際の走査面・位置基準点とTF外部パラメータの整合、
視点ごとの中心ずれ、遮蔽時のID継続、任意形状をV44へ渡す方法。
再学習には成功した教師走行を選別して使い、検出結果だけを正解経路にしない。

WSLのraw replay・全scan出力は `/home/thistle/e2e_autonomous/runs/lidar_v2x_20260918/`。
ROS通信試験はSI26の専用 `ai-work/raw/lidar_v2x_20260918/` で実施し、
AWSIMを起動せず、native V2X publisherを増やさず、運転指令を配信していない。
