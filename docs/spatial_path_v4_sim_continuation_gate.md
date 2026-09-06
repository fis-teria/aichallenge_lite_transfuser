# V4 simulator継続確認：選択シーンの座標と残る実行境界

## 到達点

開始HEAD `d015a9c04e8aac1c2633e2518a5a3e6fec991bac`、branch
`codex/windows-wsl-training-sync`、開始時working tree clean。
origin `https://github.com/fis-teria/aichallenge_lite_transfuser.git`。
前回の実装版 `b3b90b91ee3e82d5b6d0406fd6546528d5db1d0b` を保護した。

今回は既存AWSIMのシーン・component metadataと既存C#の静的確認を追加した。
**閉ループ走行の完了ではない。新しいruntime実装、推論、制御送信、走行は未実施。**
前回の7判定を成功へ昇格しない。新たな試験instance起動も行っていない。

前回の「後輪基準が不明」は、シーンの静的座標に関して進展した。
現在の実行を止める優先境界は、選択instanceで利用できる
**リアルタイムの衝突監視経路が確認できないこと**。
加えて車体全体の現在空間coverage、時刻付きpose adapter、独立停止系、
live controlled wrapperは依然として実装・実検証が必要である。

## 読み取った対象と方法

SSH `graneple@192.168.3.10` の既存実行物:
`/home/graneple/git/autononous_ai/aichallenge-racingkart/aichallenge/simulator/AWSIM`。

UnityPy 1.23.0を今回専用の
`/home/graneple/e2e_autonomous/spatial_sim_e2e_20260906/static_deps`
へ追加した。既存Python/ROS/torch環境やsimulator本体は変更していない。
追加された依存は Pillow 12.3.0、archspec 0.2.6、astc-encoder-py 0.1.12、
attrs 26.1.0、brotli 1.2.0、etcpak 0.9.15、fsspec 2026.7.0、
lz4 4.4.5、pyfmodex 0.7.2、texture2ddecoder 1.0.6。
これは実行時依存のupgradeではなく、既存Unity asset読取り用の隔離追加である。

`level1`のGameObject/Transform/collider、既存managed assemblyのscript名、
meshのlocal AABBを読んだ。assetを保存・再構築するAPIは使用していない。
MonoBehaviourのカスタムtype treeは省略されていたため、初回の完全読取は
byte count errorを記録。後続抽出では標準headerをpartial読取りとし、
script参照と元component bytesを保存した。**全カスタムfieldの復元成功ではない。**
下記Transformの数値は標準type treeから得たもの。

| 入力 | SHA-256 |
|---|---|
| AWSIM_Data/level1 | `9ab2e1e8865c02885594e0bbdde372302530f047b5a2e89c455be6a18f3e090b` |
| AWSIM_Data/globalgamemanagers.assets | `9b19aa2e22e004049272dd9c9ee2a58bddef4c8f055c44540023a3de98d9cfbd` |
| AWSIM_Data/sharedassets1.assets | `932d21250cfe685c9599acf33cca2d00d17e4141676fec236b813a069d9df34c` |
| 抽出scene_metadata_v3.json（Windows保存bytes） | `7f75b999e1e9de9a6a5d39f466c5fa17596331799a89689b7b4f04db367481e4` |

## 選択車両GoKart1の静的座標

以下はUnity座標（x右、y上、z前）で、m単位。
GameObject `GoKart1` path_id=339、Transform=718。
vehicle root以下の対象親Transformには回転・scaleの変更がないことも確認した。

| 対象 | Transform path_id / 根拠 | root相対の位置・意味 |
|---|---|---|
| base_link | URDF 876 → base_link 796 | `(0, 0.05, -0.485)` |
| 後輪collider中心 | Colliders 847 → Wheels 772 → 756 / 568 | 左右`x=±0.635`、`y=0.082`、`z=-0.484` |
| 前輪collider中心 | 847 → 772 → 894 / 854 → 867 / 546 | 左右`x=±0.509`、`z=0.603` |
| lidar_link | base_link 796 → 840 | base相対`(0, 0, 1.65)` |
| gnss_link | base_link 796 → 620 | base相対`(0, 0, -0.26)` |
| imu_link | base_link 796 → 545 | base相対`(0, 0, 0.85)`、Unity quaternion `(0,-0.70710683,0,0.70710683)` |

したがって静的な後輪中心はbase_linkの前方約0.001m、上方約0.032m。
前後輪間隔は約1.087m。見た目のwheel meshではなくWheelColliderの中心を参照した。
これはlive時刻付き変換・接地後の全姿勢や、車体全体のfootprintの検証とは別である。
Vehicleのruntime tuningや物理steering/gripの挙動を、この表だけで実測済みとしない。

IMU orientationは `AWSIM.ImuSensor.FixedUpdate` がsensor transformのworld rotationを生成し、
`ImuRos2Publisher.Publish` がROS座標へ変換して送る実装である。
GNSSは `GnssSensor.FixedUpdate` がsensor位置をROS座標+MGRS offsetへ変換する。
したがってGNSS/IMUからのpose構成候補は存在するが、外部パラメータ・座標変換・
camera時刻への同期を実装検証する前に、そのまま後輪poseとして採用しない。

## 衝突と空間の残存境界

前回保存した実graph `graph_domains_191_0_03.txt` に
`/awsim/ground_truth/on_collision` とground-truth pose publisherはない。
今回のlevel1 component一覧にも `OnCollisionRos2Publisher`、`PoseSensor`、
`PoseRos2Publisher` のinstanceがなく、DLL内のクラス存在とscene接続は別だった。

代替候補も静的確認した:

- `JudgeSystem.PublishVehicleState`: Spawned/Grounded/Ready/Start/Finishの状態通知。
  collision状態ではない。前回volatile購読で見えなかったlatched状態を、
  単にtopicが存在しないとは扱わない。
- `LapCount.UpdateSimulatorStatus`: 7値は時間・lap・section・timeScale・boost。
  衝突fieldはない。
- `CollisionDetector` → `JudgeSystem.HitEnter` → `LapCount.OnHitEnter`:
  section lineとの交差処理。汎用的な衝突通知として使わない。
- `VehiclePenaltyInstaller` はruntimeで `VehiclePenaltyController` を追加する。
  ただしcontrollerの接触処理はRaceStartSignal成立やcontact分類の条件があり、
  通常の毎frame ROS collision publisherではない。
  `JudgeSystem.SaveResultForVehicle` のresult-details書出しは事後結果であり、
  独立watchdogが即時停止を判断するliveフィードバックにはならない。
- `OnCollisionRos2Publisher` の既存クラスはOnCollisionEnter時にtrueだけを送る。
  仮に接続しても、単にメッセージが来ないことをheartbeat健全・collision=falseとは認定できない。

LiDARは前回実測で約180度・750点、今回シーンでbase前方1.65mと確認した。
現在車体の横・後方はその単一scanの観測範囲外である。
車体全体のswept footprintを無条件freeにすることはできない。
新たに通過する未観測領域への拒否、または承認された追加の現在空間観測が必要。
この継続確認では、特定の走行軌跡のclearance成立/不成立までは計算していない。

## pause/time-scale候補

`/awsim/cmd`の `JudgeSystem.HandleVehicleCommand` はboostの処理であり、pause命令ではない。
`AutowareSimulation.Awake` には `--json_path` / TimeScale処理があるものの、
今回のserialized componentにはtrafficManager/egoTransform参照がない。
TimeScale設定直後のnull参照を利用するような起動は実施しない。
Docker pauseも今回未実行・未検証。既存emergency consumerの実装確認と、
solverと別processの監視・実停止確認は別途必要である。

## ソース所在・変更権限の境界

指定候補2checkoutのtracked Unity scene / ProjectSettings / 対象C#と、
既存simulator directoryの限定探索から、編集可能なUnity projectは見つからなかった。
ホスト全体の探索や、新しい大容量ソース/Unity/imageのdownloadは行っていない。
「どこにも存在しない」とは断定しない。

ユーザーの追加指示「AWSIMに変更を加えないでください」を適用する。
先に提示したAWSIM側の小規模計測追加案は取り下げる。
既存binary・DLL・Unity scene・asset・AWSIM設定ファイルを変更せず、
試験用コピーへの改変、コード注入、plugin追加、再構築による迂回も行わない。

継続する場合の変更対象はAWSIM外部のV4/MPC wrapper・入力adapter・監視・記録に限定し、
未改変AWSIMの既存インターフェースで成立する方法を検討する。
時刻付きpose、衝突監視、車体周辺の現在空間監視の根拠が不足する場合は、
その不足をUNKNOWN/未成立として残し、走行gateを緩和して進めない。
この追加指示は走行再開やSafety条件変更の承認ではない。
正解route/未来poseをV4/controllerへ供給する機能は含めない。

## 実施数・停止状態

今回追加: simulator起動0、固定checkpoint読取0、固定forward0、control/mode publish0、
MPC solve0、powered episode0、sensor snapshot0、unit test0。
前回の累積1005.284 simulator wall秒等は変更せず、予算をresetしていない。
静的asset解析時間を新たなsimulator稼働時間と混同しない。

Dataset内容・raw bag・checkpoint読取は未実施。
今回はWSL同期も実行していないため、既定同期によるDatasetルート存在確認の追加も0。
一方で、許可された既存simulatorのシーン・mesh/component資産は読取り/hash計算した。

終了確認で `docker ps` とAWSIM/RViz/ros2 launchのprocess照会は空。
既存停止container・simulator実行物・runは削除せず保持した。
学習、走行、pushは未実施。runtime実装/test成果と偽って報告しない。

## 小証拠と実行方法

Windowsの `tmp/spatial_sim_e2e_20260906/` に今回の
`inspect_scene.py`、`summarize_scene.py`、
`evidence/scene_metadata{,_v2,_v3}.json`、`selected_scene_bindings.txt`、
`continuation_static_hashes_and_shutdown.txt`、`continuation_unity_source_lookup.txt` を保存した。
最終抽出は2,186,892 bytes。全sensor保存やDataset作成ではない。

使用した静的抽出コマンド（履歴の記録であり、走行開始コマンドではない）:

```bash
PYTHONPATH=/home/graneple/e2e_autonomous/spatial_sim_e2e_20260906/static_deps \
  timeout 30s python3 /home/graneple/e2e_autonomous/spatial_sim_e2e_20260906/inspect_scene.py
```

```powershell
python tmp/spatial_sim_e2e_20260906/summarize_scene.py
```
