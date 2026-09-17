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

終了後にAWSIM等の保護対象7ファイル、教師vendor source 323ファイル、
実行runtimeのhash一致を確認した。収集用containerは残存せず、PC10の空きは
2,793,189,376 bytes。最終runのPC10 rawコピーは保全している。
