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
  --wall-timeout-s 900 --run-budget-gib 1.25 --free-reserve-gib 2 --rviz --execute

bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
```

終了は監視基準線のunwrapped進捗 `340.547 + 60 = 400.547m`。
スタートグリッドより手前のcorner_01を次周に通過してから収録を終了する。
シミュレーション予算780s、全処理900s、記録1.25GiB、空き2GiBを保持する。

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
