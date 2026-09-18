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

## a10・a11の完了と点群整合の診断

10km/hの有限キューはa10・a11まで完了し、現在の収録は終了している。
両runとも12/12地点の通過と通過後25mを確認した。
a10は413ファイル / 718,259,223 bytes、未来30点が揃う窓1465。
a11は414ファイル / 681,161,334 bytes、同1427。
全SHA256を照合してWSLへ保全し、PC10の検証済みrawコピーは整理済み。
両runの公式crash / wall / overは0、記録姿勢で計算した物理壁重なり0、厳格採用0。
物体実姿勢などの採否検証を通過したことにはしない。

ユーザーからの点群ずれの指摘を受け、a09とa10をWSLで再解析した。
a09の長時間停止中にyawの変化がIMUとEKFで約35°乖離し、
同じ点群をIMUの相対yawで再投影すると壁距離中央値が1.225mから0.052mへ改善した。
したがって、記録EKF姿勢に依存する車体離隔計算も真値保証として扱わない。
診断・修正対象・未確認範囲は[lidar_alignment_diagnosis_20260918.md](lidar_alignment_diagnosis_20260918.md)に記録した。
AWSIM本体・稼働ROS設定・既存教師maskの変更や追加学習は行っていない。

## 自己位置のずれが起きる前までの教師を残す

ユーザー指定により、同じrunの正常な前半を残し、最初の自己位置品質低下以降を除外する。
`audit_native_corner_collection.py`は元の監査を保全したうえで、`pose_prefix/`へ
この追加条件を適用した教師と候補indexを保存する。既存監査への適用は次で行う。

```bash
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  tools/filter_teacher_pose_prefix.py \
  --collected /absolute/verified/collected/run-id \
  --audit /absolute/original/audit/run-id \
  --output /absolute/new/pose_prefix/run-id
```

既定の判定は、EKF yawとIMU yawの相対差を開始後1〜3秒の基準差に対して比較し、
0.5秒の中央値フィルタ後、5°以上の差が1秒継続した最初の時刻を検出する。
フィルタの0.5秒と追加0.5秒をさかのぼって除外境界とする。
欠測・未検証の初期基準・末尾の未解消のずれも正常扱いしない。
閾値は`--max-heading-error-deg`で指定でき、使用した値は成果物へ記録する。
固定の取り付け座標差を除いた変化量の検査であり、初期の絶対姿勢や並進精度を保証しない。

教師は未来3秒・30点で、補間endpointの最大50msも境界より前にあることを要求する。
したがってcamera観測時刻は除外境界より3.05秒以上前でなければ候補にしない。
境界と一致するサンプルも除外し、途中で姿勢が戻っても同run内では採用を再開しない。
除外したXY・速度教師はNaN、maskはfalseとし、停止教師へ置き換えない。

`pose_prefix.json`は採否境界、入力・出力SHA256、件数、元run/epochを記録する。
`observed_teachers.npz`と`anchors.jsonl`は全anchorの追加mask、
`prefix_candidates.npz`と同名JSONLは正常な前半かつ入力・未来が揃う候補の抜粋。
候補には元の`source_label_index`を残す。元の接触・離隔の棄却を解除せず、
train/validationは元run/scenario単位を維持し、自動で学習へ追加しない。
この変更は教師採否だけに適用し、AWSIM・MPPI・E2E実行時の自己位置推定は変更しない。

### 保存済み9走行への適用結果

実装・全体テスト・適用commitは`8159415c8c469ffe09f444e185049e0e7203b812`。
WSL `pytest -q`は3,166 passed / 4 skipped / 84 warnings（114.47s）。
実データでは同じcapture stampでEKFが再更新される例があり、a09の最大差は0.0171°未満だった。
教師生成と同じ後着採用を使い、差と件数を記録する。大きな同時刻不一致は未検証として除外する。
またa03/a04/a06の旧監査の遅い車両Startに基準時刻を合わせないよう、
SHA256検証済みのmonitorとsamplesから実レースStartを再確認した。
旧`STARTUP`理由の訂正根拠を残し、接触・離隔による棄却を解除していない。

| run | yaw品質判定 | 除外境界 / 評価終端 [simulation s] | 入力・未来が揃った前半の候補数 |
|---|---|---:|---:|
| a02 | 初期基準が不安定・保留 | — | 0 |
| a03 | ずれ検出 | 82.60 | 636 |
| a04 | ずれ検出 | 62.70 | 447 |
| a06 | 持続する閾値超過なし | 703.37 | 6,178 |
| a07 | ずれ検出 | 603.75 | 5,296 |
| a08 | 持続する閾値超過なし | 157.49 | 1,287 |
| a09 | ずれ検出 | 98.45 | 763 |
| a10 | ずれ検出 | 112.00 | 911 |
| a11 | 初期基準が不安定・保留 | — | 0 |

計15,518候補。うち未来速度が一度でも0.1m/sを超えるもの9,159、
未来30点の速度がすべて有効かつ絶対値0.05m/s未満のもの5,781。
残る578はこの二分類に入らない低速・速度support不足などの窓。
いずれもcamera anchor数であり、独立した回避イベント数ではない。
a09の最後の候補観測はsimulation 95.334997869sで、3秒の未来と50ms補間余裕は
98.449997777sの境界より前に収まる。

採用先の最新出力はWSL
`/home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918/pose_prefix_v2/`。
`batch_summary.json`に全件の境界・件数、`verification.json`に検証結果を保存した。
原本・元監査のhash、候補教師と元教師の値一致、境界を越える全Headの無効化、
元run/epoch保持、既存の厳格棄却を昇格していないことを9走行すべてで確認した。
`pose_prefix_v1/`は旧Startを再確認する前の診断記録で、今後の選択には使用しない。

正常な前半の抽出は完了したが、既存の物体離隔等の厳格採否は全走行で保留のまま。
従って15,518を「学習投入済み」や「回避に成功した教師数」とは数えない。
元rosbagを削除せず、split割当・再学習は行っていない。

### ずれ前候補の接触・物体離隔の確認

保存済み9runの公式結果、候補のある7runの記録車体姿勢、35,312 LaserScan、
LiDAR→V2Xの表面検出をWSLで追加照合した。入力bag、samples、公式結果、
生成時YAML、配置metadataはexport manifestのSHA256と再照合した。
各camera候補について、過去1秒と未来3秒に補間support各50msを加えた窓を検査した。

公式crash / wall / overは9runすべて0。検査した記録EKF姿勢に車体のMeshColliderの
XY外形を置いた場合、実AWSIM由来の物理壁地図への重なりは0件。
この結果は記録姿勢に基づく確認であり、絶対姿勢の誤差や3次元の接触を保証しない。

| run・物体 | 最接近のsimulation時刻 [s] | 記録姿勢での投影外形間距離 [m] | その場面を含む候補窓数 |
|---|---:|---:|---:|
| a06 / corner_08 cone | 214.885 | 0.182 | 46 |
| a07 / corner_08 cone | 194.235 | 0.172 | 52 |
| a08 / corner_09 box | 89.135 | 0.169 | a08の箱2か所で計96 |
| a08 / corner_11 box | 118.145 | 0.154 | 同上 |
| a10 / corner_08 cone | 83.175 | 0.149 | 48 |
| a10 / corner_09 box | 88.895 | 0.143 | 49 |

コーンの0.30m未満に該当する候補146、箱の初期配置による参考計算に該当する候補145、
重複を除く計291窓に注意フラグを付けた。いずれもcamera anchor数で、291回の
接触・回避イベントを意味しない。元の教師・採用maskは変更していない。
4窓にはmonitor姿勢の履歴端の不足もあり、別の保留理由として保存した。

コーンの形状について、配備DLLの読み取り専用再確認では、fallbackは半径0.175m、
高さ0.7m、24分割の円錐MeshCollider（convex）だった。以前の診断メモの
「CapsuleCollider」という説明は訂正する。今回の計算はXY投影を覆う外接円を使う。
円錐の高さ方向と車体形状を含めた3次元最短距離ではなく、保守的な投影距離である。
別実装の24角形距離との比較も行い、外接円との差が理論上限約1.50mm以内と確認した。
箱は初期位置の0.5m立方体の投影による参考値。生成6固定更新後にRigidbodyがdynamicに
なり得るため、この値から実際の箱の移動・接触を断定しない。

記録されたLaserScanは750本、角度-1.566607〜1.570796radで、センサの取付位置は
base_linkから前方1.65m。車体側面と後方は直接観測範囲外になる。
実際の最接近位置を車体座標で図示すると、上表のコーンや箱は側方にあり、
前方scanに30cm未満の点が無いことでは側面の離隔を確認できなかった。
検査した35,312 scanの観測点と車体投影外形の最短距離は約0.353mだったが、
これを車体全周の離隔として用いない。

LiDAR→V2Xの記録は`object_model=surface`で、`surface_xy_m`と`observed_size_xy_m`。
native物体の固有ID、実中心、隠れた面を含む外形の正解ではない。native物体の全身poseや
全接触を記録したtopicは無く、native `/v2x/vehicle_positions`も0件。
7,735候補窓では箱の初期位置が車体付近6m内にあり、実位置の確認が必要と記録した。
6mは監査対象を検索する半径で、教師の回避開始距離や安全閾値の変更ではない。

現時点では、15,518候補は保全したまま厳格採用0を維持する。
291窓の注意フラグがない候補も、物理離隔を確認できた成功教師とは認定していない。
採用を進めるには、箱の実位置・姿勢を追跡し、側方通過中も自己位置の不確かさを
含めて0.30mを満たす区間を確認する必要がある。未観測範囲を離隔良好として補完しない。
AWSIM本体、教師制御、自己位置推定、学習splitはこの診断では変更していない。

最新成果物はWSL
`/home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918/native_prefix_clearance_v2/`。
全体`summary.json`、`verification.json`、各runの`input_sha256.json`、
`candidate_checks.jsonl`に、元run/epoch/source_label_indexとsimulation時刻を保持した。
図は`side_visibility.png`。`native_prefix_clearance_v1/`は形状説明の訂正前の診断記録。

Windows側の診断元・結果コピーは`tmp/lidar_alignment_20260918/`。
実行済み診断scriptは
`check_native_prefix_clearance.py`（SHA256 `a1c17752ea27e26cdaaf7544f9848848bcfaedcb2461074972295acd9edda483`）、
照合scriptは`verify_native_prefix_clearance.py`。
全15,518窓の元ID、epoch、source index、時間境界、原本mask/hash保持を照合してPASS。
距離計算の既知形状・非有限値・shape拒否、窓端包含のsmoke assertionもWSLでPASS。
製品コードの変更はなく、全体pytestは再実行していない。前節の3,166 passedは
同じ製品実装8159415での既存結果で、今回の離隔を保証するテストではない。

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser_lidar_v2x_margin
# 原本は保持する。再実行時はscriptの出力directoryを新しい名前に変更する。
bash tools/with_wsl_training_lock.sh .venv/bin/python \
  /home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918/check_native_prefix_clearance_v2.py
bash tools/with_wsl_training_lock.sh .venv/bin/python \
  /home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918/verify_native_prefix_clearance.py
```

### 軌道・速度教師の選別

`tools/curate_native_teacher_data.py`は、正常な前半の候補を対象に、
観測された未来軌道・速度の学習用selectionを別directoryへ保存する。
物理離隔を保証した回避成功データの判定と、通常の観測軌道の品質選別は分ける。
既存の`forward_avoidance_eligible`を昇格せず、停止意図・行動クラスの教師は付与しない。

既定条件は、未来30点のXY・速度が有効、未来速度が全点0.2m/sを超えること。
注意フラグの291窓は除外し、箱の初期位置から6m以内を含む窓は保留する。
静止コーンは投影外形距離0.30mに監査余裕0.30mを加え、0.60m以上を要求する。
この追加余裕は統計的な自己位置誤差の上限ではなく、品質選別の条件である。
コーンが6m以内にない区間では、検査窓全体で教師modeがFREE_RUNであることも要求する。

各anchorの過去1秒・未来3秒に50msの補間余裕を加え、全区間で以下を確認する。
教師の非常停止なし、perception ready、pose/scan/command/perceptionに150ms超の欠測なし。
記録EKF姿勢で点群を物理壁地図へ投影し、各scanの壁残差中央値0.15m以下、
0.25m内の壁inlier率70%以上を要求する。距離1〜12mの有効点を3本おきに取り、
80点以上、角度幅60°以上を必要とする。地図へ合うように姿勢や教師を修正しない。

選別済みindexは最大5Hz（200ms以上の間隔）に間引く。間引き前の適格indexも
`decisions.jsonl`に保持する。`selected_teachers.npz`は元の教師と同一値の抜粋で、
`selected_anchors.jsonl`に元run/epoch/source_label_index・入力履歴参照を残す。
同じ`all_corners_20260918`シナリオ群をtrain/validation/testへ分散させず、split未割当で保存する。

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser_lidar_v2x_margin
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  tools/curate_native_teacher_data.py \
  --root /home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918 \
  --output /home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918/curated_xy_speed_v1
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
```

#### 保存済みデータへの選別結果

9run・元camera anchor 30,654のうち、前段で残した15,518候補を選別した。
品質条件を満たした760窓から200ms以上の間隔で392窓を抽出。
通常走行87窓、静止コーン周辺の走行305窓。間引いた368窓も適格としてindexを保持した。
離隔の注意条件に当たる291窓は今回の選択から除外し、残り14,467窓は理由付き保留。
前段の15,136候補外anchorを含め、元rosbag・元教師・既存採用maskは保全している。

| run | 前段候補 | 適格・間引き前 | 選別済み | 通常走行 | コーン周辺 | 保留 | 離隔条件で除外 |
|---|---:|---:|---:|---:|---:|---:|---:|
| a02 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| a03 | 636 | 71 | 37 | 0 | 37 | 565 | 0 |
| a04 | 447 | 22 | 12 | 0 | 12 | 425 | 0 |
| a06 | 6,178 | 196 | 100 | 35 | 65 | 5,936 | 46 |
| a07 | 5,296 | 198 | 102 | 52 | 50 | 5,046 | 52 |
| a08 | 1,287 | 127 | 66 | 0 | 66 | 1,064 | 96 |
| a09 | 763 | 20 | 10 | 0 | 10 | 743 | 0 |
| a10 | 911 | 126 | 65 | 0 | 65 | 688 | 97 |
| a11 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

保留理由は重複し得る。静止コーンの文脈がないAVOID等9,351窓、箱の近接7,728窓、
停止・微速の意図未確認7,670窓、点群/壁地図の整合または観測support不足6,570窓など。
元々除外された291窓では追加の保留理由を集計していないため、前回の近接集計とは母集団が異なる。

用途は観測未来XY・速度の追加教師。コーン周辺305窓を305回の回避成功とは数えない。
物理離隔が全周で証明された成功イベント用の`forward_avoidance_eligible`はfalseのまま。
一方、今回の用途別選別では、正常な観測軌道として使う392窓に採用indexを付けた。
停止probability・行動classを推測して補完していない。学習splitへの統合と再学習は未実施。

選別処理の実行commitは`6b11042689e490d69ddc6dd988808729fe67cfa7`。
出力はWSL
`/home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918/curated_xy_speed_v1/`。
`selection_manifest.json`が全体目録、各runの`selected_anchors.jsonl`と
`selected_teachers.npz`が学習用の参照indexと教師。cameraやLiDAR原本は
目録の`source_bag`を参照し、重複コピーしていない。
任意のrunをフレーム単位でvalidationへ分割せず、シナリオ群の割当を決めてから
既存の学習corpus/cacheへ取り込む。

全392窓について、元source index・run/epoch・未来境界・値とmaskの一致・split group保持を
別scriptで照合してPASS。さらに各runの先頭・中間・末尾の計21anchorを元bagから読み直し、
カメラ`[1,4,3,224,384]`、LiDAR`[1,4,2,750]`、全履歴参照、
未来XY/速度`[30,2]`/`[30]`の再構築を確認した。
`selection_verification.json`と`selected_camera_samples.jpg`へ保存し、画像一覧も目視確認済み。
選別目録SHA256は`4b67c66a96201131e7be57214a18fb8a583632e127c47b44777ec4c9afcc7c4d`。

最終実装commit `67a2daa2465654f074ae86eccc7ab1d7e743eee2`では、
1秒履歴・3秒未来・50ms以上の補間supportと、既存0.30m条件を短縮できないようにした。
選別時の既定条件は同じ。WSLの全体`pytest -q`は3,203 passed / 4 skipped / 84 warnings、113.19s。
ログはrun rootの`curation_full_pytest.log`。新規テストは、未来側だけの不具合、
欠測窓の補間禁止、箱の近接、停止意図未確認、shape/非有限値、CLI実行、間引き、
既存時間・離隔条件の短縮拒否を含む。

Windowsの確認用コピーは`tmp/curated_teacher_20260918/`。
独立照合scriptは`tmp/lidar_alignment_20260918/verify_curated_xy_speed.py`、
SHA256 `f34beb8373bfabdcbbccf878db6c31bd17f8e629a141b5dce9ad9d9098573657`。
WSLでは同じscriptをrunのrootに保存して実行した。

```bash
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  /home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918/verify_curated_xy_speed.py
# 上記照合結果は上書きしない。再実行時は照合出力名を変更する。
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
