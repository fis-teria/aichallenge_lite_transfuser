# 全コーナーの箱・コーン教師収集

PC10 `graneple@192.168.3.10`、修正版MPPI V45 `lidar-motion-intent-r2`、上限5km/h。
曲率区間で検出した12コーナーの入口に箱6個・コーン6個を交互に同時配置する。
物理車両の4台制限とは別のnative objects枠（上限32個）を使用する。
AWSIMバイナリや既存の教師マージンを変更しない。

生成・検証コマンド:

```bash
python3 tools/prepare_corner_obstacle_collection.py \
  --awsim-repo /home/graneple/git/autononous_ai/aichallenge-racingkart \
  --reference integrations/mppi_v45/assets/multi_purpose_mpc_ros/env/final_ver3/in_corce_line.csv \
  --output /absolute/new/scenarios --prefix lidar-v45-pc10-corners-native-a01

python3 tools/collect_mppi_v45.py \
  --awsim-repo /home/graneple/git/autononous_ai/aichallenge-racingkart \
  --runtime /home/graneple/e2e_autonomous/mppi_v45_collection_fix_20260918/runtime \
  --scenario /absolute/new/scenarios/lidar-v45-pc10-corners-native-a01.yaml \
  --run-id lidar-v45-pc10-corners-native-a01 --speed-cap-kmh 5 \
  --wall-timeout-s 900 --run-budget-gib 3 --free-reserve-gib 2 --rviz --execute

bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
```

終了は監視基準線のunwrapped進捗 `340.547 + 60 = 400.547m`。
スタートグリッドより手前のcorner_01を次周に通過してから収録を終了する。
シミュレーション予算780s、全処理900s、最終試行の記録3GiB、空き2GiBを保持する。
初期試行の記録予算は1.25GiB。

箱・コーンはV2X車両IDを持たず、物体位置の教師入力はLiDAR→V2Xで作る。
配置の地図確認には既存カートのフットプリントを代理使用するが、実際のprefab寸法を
検証したものではない。既存の0.30m車体間距離監査を緩めず、物体接触の公式結果、
各配置の通過、センサ同期、実測未来教師を別途記録する。
データはrun群全体でsplit未割当とし、既存学習へ自動登録しない。

標準RVizは`--rviz`で起動する。教師採用軌道・候補軌道・壁地図・LaserScan・
LiDAR→V2X障害物マーカーをdomain 1で表示する。Xorg/Waylandの両デスクトップに対応。

WSLでは次で全物体の生成数・コーナー別の通過・時間教師を検査する。

```bash
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src \
  .venv/bin/python tools/audit_native_corner_collection.py \
  --collected /absolute/verified/collected/run-id --output /absolute/new/audit/run-id
```

`/awsim/state`が車両状態Spawned/Grounded/Readyだけを配信する環境では、
monitorの`/admin/awsim/state=start`観測を同時刻のego simulation stampへ対応させる。
レース開始の観測が無いrunは拒否する。wall timeをsimulation timeと混同しない。

WSL全体テスト: 3,136 passed / 4 skipped / 84 warnings、111.89s。
実装commit `6042aa96924644f275eb8d3440b99a276fac329f`。

## 実走の配置調整

初期配置は通路中央寄りで、corner_02のコーン手前で停止を保持した。
次の配置は各物体を基準線から概ね1.2m横へ寄せ、corner_02〜09を通過したが
corner_10の手前で停止した。新規候補が`execution_sweep / collision`で棄却され、
既存軌道の速度を0にするretimeが選ばれていた。停止だけから実物への接触とは断定しない。

3回目の走行配置は `configs/collection/corners_native_pc10_20260918.yaml`、
対応する位置・地図根拠は同名JSON。corner_10のコーンだけ、基準線の右1.2mから
左0.7mへ移す。箱6・コーン6、他の11配置、教師・5km/h・マージンは共通。
再実行時は既存runを保全し、YAMLのname・JSONのcasesとcollectorのrun-idに新しい同一IDを使う。

停止したrunも削除せずWSLへハッシュ照合して保存し、成功教師として自動採用しない。
Rvizの実ウィンドウ、採用軌道の描画、5 topicのpublish/subscribeを確認済み。

`a04`は入口で停止後、教師が自力で再開していた。直前の停止観測をもとに作業者が
中断をかけたため、停止継続による失敗として扱わない。終了直前は約1.19m/sで走行中。
同じseed・12配置・始点・終点を`a06`で再走し、一時停止だけを理由に中断しない。
`a05`は始点変更の準備だけで、既存AWSIMのグリッド外Ready制約を検出して未実行。

## 保存容量の確保

完了済み各runはWSL側と再度全rawのSHA256を照合してからPC10側コピーだけ整理した。
使用中のuvキャッシュは変更していない。

旧 `PC10:/home/graneple/e2e_autonomous/time_recovery_random_20260915` は
660ファイル / 1,729,292,639 bytesを全SHA256照合して次へ退避し、PC10側コピーを整理した。

`/home/thistle/e2e_autonomous/runs/pc10_storage_archive_20260918/time_recovery_random_20260915/raw/time_recovery_random_20260915/`

これは旧復帰データの保全移動で、今回の障害物収集件数へ加算しない。
退避後のPC10空きは5,457,813,504 bytesで、3GiB収録と2GiB空きの予算を確保した。

## 最終走行 a06

`lidar-v45-pc10-corners-native-a06` は人による途中中断なしで、694.831sに
終了条件 `reference_s_reached` を達成した。unwrapped進捗400.565m、
公式の完了ラップ数は1、実移動距離は約385.9m。780sの予算内に終了した。
入口およびcorner_11手前では長く待機した後、自力で走行を再開した。

公式結果のcrash / wall / overは0 / 0 / 0。しかし、地図による車体全体の判定
`no_off_track` が不合格だった。最大侵入深さは0.50m、記録時刻239.7702s。
全体の`scenario_verdict`は`failed`であり、終了地点への到達と良好な教師品質は
区別する。箱・コーンの実meshとの0.30m clearanceも未確認なので、成功教師として
自動採用せず、入力・実測未来教師・停止挙動を診断用に保全する。

最終runの423ファイル / 2,669,772,748 bytesは全SHA256一致を確認してWSLへ保存した。

WSL監査でも12 / 12地点の通過と、各地点から25m以上の後続観測を確認した。
最終runのカメラ観測は6,698、3秒先まで全30点が揃う実測未来軌道は6,628窓。
`xy_m`は`[6698, 30, 2]`、0.1s間隔・観測時base_link座標・単位mで、
有効mask部分の有限値と全点有効数の一致を検証した。
カメラとLiDARの時刻逆行は0、最大間隔はそれぞれ0.105s / 0.050s。
これは重複する時間窓の数で、独立した回避イベント数ではない。

| run末尾 | 終了条件達成 | 通過地点 / 12 | カメラ観測 | 全30点の未来窓 | 厳格な教師採用数 |
| --- | --- | ---: | ---: | ---: | ---: |
| a02 | いいえ・作業者中断 | 0 | 955 | 887 | 0 |
| a03 | いいえ・作業者中断 | 8 | 2,581 | 2,513 | 0 |
| a04 | いいえ・走行再開中に作業者中断 | 3 | 881 | 812 | 0 |
| a06 | はい | 12 | 6,698 | 6,628 | 0 |

今回の4実走を合計すると1,661ファイル / 4,719,636,357 bytes、カメラ11,115観測、
全30点が揃う未来軌道10,840窓をハッシュ検証して保存した。
後退区間と`/control/mpc/stop_request`受信は各runで0。
ただし通常の速度0指令による長い停止は含まれ、停止が無かったという意味ではない。
採用数0は入力や未来教師が全欠損という意味ではなく、完走・地図逸脱・物体距離を含む
既存のrun品質条件を満たしていないため。train/validation splitは未割当、再学習は未実施。

集計根拠はWSL `runs/mppi_v45_pc10_20260918/corner-native-collection-summary.json`、
各audit配下の`audit.json`、`corner_coverage.json`、`anchors.jsonl`、`observed_teachers.npz`。

```text
/home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918/collected/lidar-v45-pc10-corners-native-a06
/home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918/audit/lidar-v45-pc10-corners-native-a06
```

RViz実画面と5 topicのpublisher/subscriberを最終runでも確認した。
画像はローカル `tmp/corner_obstacles_20260918/lidar-v45-pc10-corners-native-a06-rviz.jpg`。
候補軌道Displayにはエラー表示が残るため、全候補の正常描画までは確認済みとしない。
採用軌道と壁地図の描画は確認済み。RVizは教師containerに付随し、収録終了時に閉じる。

今回の「RVizも出しておく」依頼に合わせ、収録終了後はPC10ホストの標準RVizを別起動した。
domain 1、同じ表示設定、simulation clock使用で、現在はlive topic受信待ち。
走行nodeやAWSIMは再起動していない。閉じる場合は通常どおりRVizウィンドウを閉じる。

```bash
source /opt/ros/humble/setup.bash
DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority ROS_DOMAIN_ID=1 \
  rviz2 -d /home/graneple/e2e_autonomous/mppi_v45_collection_fix_20260918/teacher_collection_standalone.rviz \
  --ros-args -p use_sim_time:=true
```

### 有限の追加収録キュー

Windows側の`tmp/native_collection_resume_20260918/continue_10kmh_batch.py`は、
実行中のa09が閉じるのを待ち、a10・a11を最大2回だけ追加する。
各runは10km/h上限、780s simulation / 900s wall / 3GiB記録 / 2GiB空き予約。
各終了後に、PC10からWSLへの全ファイルSHA256照合、時間教師と地点coverageの監査、
物理壁地図での再確認、raw再照合、PC10の検証済みコピーだけの整理を順に行う。
接触カウンタまたは物理壁重なりを検出した場合と処理エラー時は、保存して次の開始を保留する。
自動で学習データへ採用する処理は含めない。

```powershell
# Windows。二重起動はstateファイルで拒否する。
python tmp/native_collection_resume_20260918/continue_10kmh_batch.py --check-only
python tmp/native_collection_resume_20260918/continue_10kmh_batch.py
Get-Content tmp/native_collection_resume_20260918/10kmh-batch-state.json
```

現在状態・完了run・保存量・採否未確定は上記stateに逐次保存される。
キュー準備時のPC10読み取り確認ではAWSIM等の保護対象7ファイル、教師vendor323ファイル、
runtimeのhashが一致した。標準RVizだけを通信設定付きで再起動している。

終了後にAWSIM等の保護対象7ファイル、教師vendor source 323ファイル、
実行runtimeのhash一致を確認した。収集用containerは残存せず、PC10の空きは
2,793,189,376 bytes。最終runのPC10 rawコピーは保全している。

## 収集継続と判定の再確認

12地点を一度通過したことは、必要な教師データの収集完了ではない。
`a07`（seed 20260919、同じ12配置・5km/h）で追加収録を再開した。
収録前にa06のWSL raw全57ファイルを再度SHA256照合し、PC10側の同一コピーだけ
整理して空き5,457,133,568 bytesを確保した。WSL原本は保全している。

PC10の実AWSIM資産を読み取り専用でコピーし、WSLで物理壁地図を再生成した。
level1のSHA256は`9ab2e1e8865c02885594e0bbdde372302530f047b5a2e89c455be6a18f3e090b`。
生成PGMはMPPIの使用物と完全一致
（`f7aa23da81c626b8e7457628744f25600f4ff8afee9d2f37d06bc390889903a3`）。
これは旧監査地図の境界と実物理壁を区別する根拠であり、AWSIMの改変ではない。

a06の13,702サンプルを、記録されたbase_link姿勢と実MeshColliderのXY外形で
再確認すると物理地図との重なりは0件だった。旧地図での最大0.50mの判定は
corner_10の入口、進捗231.421mに対応する。
ただしcorner_08のコーンへの最小離隔は約0.182mで、0.30m条件を満たさない。
箱は標準形状が0.5m立方体と分かったが、生成後に動的物理へ移るため、
設定上の初期位置だけで実走中の離隔を認定しない。
既存の厳格maskはこの再確認だけで書き換えず、区間ごとの採否を別に検証する。

次の収録から、解決済みスタートグリッド位置・全物体station・必要な後続25mから
終了進捗を計算する。現配置では約373.125mとなり、以前の400.547mまで走って
2周目の別障害物に遭遇する必要はない。通過後25mの観測条件は維持する。
また、`/awsim/state`の遅れて届くStartより、監視側が確認したレースStartの
simulation stampを優先し、最初の回避区間を誤ってSTARTUPへ落とさない。
両変更の回帰確認は上記のWSL `pytest -q`で実施する。

再確認の出力はWSL `runs/mppi_v45_pc10_20260918/` の
`pc10_physical_wall_map/provenance.json`、`native-offtrack-a06.png`、
`lidar-v45-pc10-corners-native-a06-physical-inspection.json` と
同名prefixの`physical-samples.jsonl`。従来の監査結果・rawは保全している。

## 上限10km/hへの変更と追加収録

ユーザー指示により、a08以降は速度上限を10km/h（2.7777777778m/s）とした。
障害物に対する教師の減速・停止判断は残す。a08、a09のlive ROSパラメータで
`execution_profile.max_speed_mps`を確認し、既存の離隔・車体マージンも一致した。
教師binaryとAWSIM binaryは、この速度変更では更新していない。

変更コミット`71657aad691c6059dae82854557aea519d06c604`のWSL全テストは
3,146 passed / 4 skipped（115.55s）。ログはWSLの
`runs/mppi_v45_pc10_20260918/native_collection_71657aa_pytest.log`。

| run | 上限 | 終了条件到達 | 通過地点 / 12 | 通過後25mを確認 / 12 | カメラ観測 | 未来30点が揃う窓 |
|---|---:|---|---:|---:|---:|---:|
| a07 | 5km/h | いいえ・時間切れ | 12 | 12 | 7,506 | 7,432 |
| a08 | 10km/h | はい・148.906s | 12 | 11 | 1,499 | 1,430 |
| a09 | 10km/h | いいえ・時間切れ | 1 | 0 | 7,505 | 7,434 |

上記3runの公式crash / wall / overカウンタは0。a08の実測最高速度は
2.795m/s（約10.06km/h）、移動距離353.8m。
設定した進捗への到達とシナリオ全体のPASSは区別する。旧監査地図の逸脱判定が残り、
rawのシナリオ判定はfailed。採用maskを自動変更したわけではない。

a07は419ファイル / 2,899,640,589 bytes、a08は417ファイル / 706,359,054 bytesを
PC10からWSLへ転送し、全ファイルSHA256を照合した。さらにrawのSHA256を再検証してから、
PC10の同一rawコピーだけを削除した。WSLの原本・教師出力・provenanceは保持している。

a08ではcorner_08のコーンをteacher reference横位置-1.2mから-0.9mへ移動した。
a07記録姿勢での事前離隔は約0.472m、a08実走記録での離隔は約0.391m。
後者の全2,936姿勢では、実AWSIM由来の物理壁地図への車体重なりは0件だった。
箱は初期位置に置いた0.5m外形による参考計算でcorner_09約0.169m、corner_11約0.154m。
実際の箱の移動・姿勢を未検証のため、これを確定した実離隔として採用しない。
全てのカメラ窓を訓練可能と数えない。従来の厳格maskではa07、a08とも採用0で、
箱の位置検証およびnative物体に対応した区間ごとの採否確認が引き続き必要。

a08の最終GNSS進捗は373.097mで、最後のcorner_01通過後は約24.972mだった。
監視の終了条件は推定自己位置を使うため、a09では必要後続25mに終了余裕0.5mを加え、
終了進捗を373.625mとした。a08の過去判定・不足2.8cmは書き換えていない。
a09（seed 20260921）は同じ配置・10km/hで追加収録し、箱の手前で前進候補の
`execution_sweep`衝突棄却による停止を確認した。停止から再開せず780sで終了した。
426ファイル / 2,510,230,339 bytesの全SHA256照合とWSL監査を完了し、
物理壁重なり0、地点通過1/12、後続25m確認0/12、厳格採用0だった。
多数の未来窓が揃っていても、大部分は同じ停止状態であり、回避イベント数には換算しない。
原本をWSLへ保全し、検証済みPC10コピーの整理後、キューがa10を自動起動した。

標準ホストRVizは、最初の起動がFast DDS、教師側がloopbackのCycloneDDSであったため、
受信できていなかった。ホストRVizのみを同じCycloneDDS/domain 1へ再起動した。
a09で採用軌道・候補・壁地図・LiDAR物体marker・LaserScanの5 topicすべてに
`rviz`購読を確認した。候補Marker Displayのエラーは残るため、全候補の正常描画は未確認。

```bash
# 保存済みのa09シナリオを使った実行コマンド。別の実行には新しいrun IDを使う。
python3 /home/graneple/e2e_autonomous/mppi_v45_collection_fix_20260918/source/tools/collect_mppi_v45.py \
  --awsim-repo /home/graneple/git/autononous_ai/aichallenge-racingkart \
  --runtime /home/graneple/e2e_autonomous/mppi_v45_collection_fix_20260918/runtime \
  --scenario /home/graneple/e2e_autonomous/mppi_v45_collection_fix_20260918/scenarios/lidar-v45-pc10-corners-native-a09.yaml \
  --run-id lidar-v45-pc10-corners-native-a09 --speed-cap-kmh 10 \
  --wall-timeout-s 900 --run-budget-gib 3 --free-reserve-gib 2 --execute

# 収録とは独立した標準RViz。走行nodeは起動しない。
source /opt/ros/humble/setup.bash
DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority ROS_DOMAIN_ID=1 \
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  CYCLONEDDS_URI=file:///home/graneple/git/autononous_ai/aichallenge-racingkart/vehicle/cyclonedds.xml \
  rviz2 -d /home/graneple/e2e_autonomous/mppi_v45_collection_fix_20260918/teacher_collection_standalone.rviz \
  --ros-args -p use_sim_time:=true
```
