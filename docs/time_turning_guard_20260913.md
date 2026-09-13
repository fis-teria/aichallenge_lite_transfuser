# TimePath B0 旋回に合わせた前方監視

ユーザー依頼「曲がり角でも旋回できるようにしてほしいです」。
前回 [lap06/lap07記録](time_lap_20260913.md) の診断を踏まえた新しい修正・試験判断。
目標5km/h、Pure Pursuit、生30点のTimePath B0、通常RViz表示、実行先
`graneple@192.168.3.10` を継続。成功条件はまず以前のコーナー入口を通過し、
引き続き同じ周回judgeと停車確認で1周を目指すこと。衝突/監視拒否なら終了して診断する。

編集前 `76b3ae7c4417c132cd7ef66c053ea70d8e7ad1ed`、Windows clean。
ホストの既存差分、停止済みcontainerと39個の履歴Composeは保全。
前回と同じ直線監視の再試行は行わない。ソース・テスト・WSL評価の正本はWindows。

## 変更が必要な理由と範囲

最初の阻害要因は、旋回する指令に対して直線の停止矩形を使用していたこと。
しきい値緩和では曲がる時の障害物位置を表現できないため、監視の幾何を変更する。
所有層はAWSIM試験用の前方近接監視と、その制御ノードへの接続。
推論・教師・PPの目標点・重み・目標速度・操舵/加速度制限は変更しない。
旧監視は `straight_v1` として保存し、新規設定の `steering_sweep_v1` のみで有効化する。

- 実操舵、前回送信操舵、今回のrate limit適用後の送信操舵を含む曲率区間を使う。
  その区間内で操舵が変化する場合の位置・姿勢誤差上界で、旋回する車体矩形を膨張する。
- 停止移動距離は従来の `0.4 + v*0.5 + v^2/(2*1.0)` mを維持。
  車体余裕は従来の左右±0.85m、vehicle root前端+1.5m（rear axle前端約1.984m）、
  後輪から後ろ0.510m。後方寸法は既存 `spatial_sim_e2e_v4.yaml` を参照。
- 25mm以下の移動刻みと補間移動量の膨張でsample間を覆う。scanの角度間隔も膨張へ入れる。
- LiDARの元capture姿勢から現在rear axleへ変換。base_link→LiDAR+1.65m、
  base_link→rear axle約+0.001m。時刻・frame不一致や変換不足は拒否。
- 各rayと旋回矩形の区間を比較し、矩形の奥まで観測が届かない場合も拒否する。
  NaN/-inf/範囲外は拒否、AWSIMの+infはrange_maxまでのno-returnとして従来同様に扱う。
- 既存のclock/timeout/source/authority/geometry/停止距離/overspeed監視を維持。
  停止時は前回操舵保持、target0・acceleration=-1。重大faultではホストがAWSIMをfreeze終了。

範囲の限界: これは**前方LiDARの観測rayに対する旋回近接監視**であり、
現在の側面・後方など未観測領域をfreeと認定する全周衝突保証ではない。
従来の前方監視と同じAWSIM限定の検証範囲を保ち、`full_body_free_space_verified=false`
を明示する。停止応答0.5s・減速度1m/s²と、操舵が区間内に収まる仮定は
実車に対する校正値ではない。モデルの未来headingを捏造して監視には使わない。

## 検証計画・コマンド

直進/左右旋回、曲がる側の障害物、外側の壁、実操舵が追いついていない場合、
前回指令が残る場合、NaN/時刻/frame/速度上限、曲率が時間変化する軌跡の包絡をunit test。
新旧設定は明示的に分けて切り戻せる。WSL full pytest後、実行物をHumbleでbuildし、
影響した純Python moduleとROS nodeのsource/install hashを確認する。
ROS shadowで実モデルPathと合成制御の負例を通してから、新規run IDを1回消費する。

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q \
  tests/test_turning_scan_guard.py tests/test_time_trial_v1.py
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
```

初回試験専有root `/home/graneple/e2e_autonomous/time_turning_20260913`、
run ID `codex-time-turn-08`、走行600sim/600wall秒・全体720wall秒以内。
通常RVizで `/visualization/time_path/raw_path` を表示する。

```bash
timeout --signal=TERM --kill-after=10s 710s python3 SOURCE/tools/run_time_path_awsim_trial.py \
  --deployment /home/graneple/e2e_autonomous/time_turning_20260913 \
  --run-id codex-time-turn-08 --display :1 \
  --config configs/control/time_path_turning_5kmh_20260913.json </dev/null
```

現在: 実装・事前検証済み。`1c0cf319cd99c0dc0c40ce051bc64bfea5403fd4` の
native WSL限定テスト33passed (0.31s)。実行sourceは
`a2e05e2e81ee92a9f06bd8787ee7fccc6a24db26`、Windows/WSL同一commit。
全体2080passed/4skipped/63warnings (85.27s)。Humble build0.92s。
隔離ROS smokeは実モデルPath一致6、新しいsweep guardを通った合成正加速110、
stale11/clock7/overspeed12の制動、実車両topic publisher0、child exit0。
9個のsource/install module hash一致、checkpointは従来と同一。
source archive SHA256 `949770aca53a28f09497503d4c0b6827b2c9c65c7ae24a1938c48142fbacd60a`、
config SHA256 `d9033a4f15bc6b404cff0be2ab2c487cfd13537327448656b5db3914ae5c0085`。
正式SOURCEは専有root内 `source_a2e05e2`。次は上記turn08を1回実施する。
lap07には実操舵値が保存されていないため、新監視の完全な実測再計算とは称さない。
今回から実操舵と使用した監視区間・元scan姿勢を記録する。

## turn08: scanとposeの到着順序の不具合

turn08は `STOPPED_NO_LAP`、`PROGRESS_STALLED` で停車確認後終了、outer exit1、
wall83.901s、cleanup error0。通常RViz購読rviz2、既存Compose39個保全。
コーナー監視の合否を判断できる走行には至らなかった。
264回の `OBSERVATION_POSE_MISSING` は全てplanとPPの検証後のscan姿勢補間で発生。
最新scanがpose callbackより先に届くと、その時刻の後側poseがなく補間不能になる。
新規監視接続で導入した不具合で、制動/再発進を繰り返した。減速後に
`STEERING_INFEASIBLE` 97回も記録されたが、まずscanの時刻同期を修正する。

原因所有層: 新監視に渡すscanの選択。4件のbounded bufferから、最新pose履歴で
補間可能な一番新しいscanを選ぶ。元timestampを保持し、既存のcapture150ms/receipt300ms
上限を満たせない場合は拒否。clock resetでbufferを破棄し、未来poseの外挿は行わない。
非同期5msずれを再現するunit/ROS smokeを追加してから、別run ID `codex-time-turn-09`
で再検証する。turn08の結果は保全し、自動再試行ではなくこの診断に基づく新しい判断。

## turn09実行前確認

source `75ca90b068f06c3daf1c3fc95f9849b902784553` をWindows/WSL同期済み。
WSL full 2081passed/4skipped/63warnings (79.05s)。Humble build0.94s、
5ms先着scanを含むROS smokeでguard正加速111、stale11/clock7/overspeed12、
実モデルPath一致6、実車両topic publisher0、child exit0。9module source/install一致。
新しい専有rootは `/home/graneple/e2e_autonomous/time_turning_aligned_20260913`、
SOURCEはその中の `source_75ca90b`。旧installとturn08記録は元rootに保全。
archive SHA256 `04517c277524629a471e544f033f42e3046aa09a0e868ba58deb418d7680eb49`。
config/checkpointはturn08と同一hash。上記実行コマンドのdeployment/SOURCE/run IDを
このroot/source/`codex-time-turn-09` に置き換えて1回実施する。有限枠は同じ。

## turn09結果と操舵接続の診断

turn09は発進29.244999346sim秒後に `CONTROL_STOPPING_SWEEP_OCCUPIED`、
未完走。585回の連続追従でscan補間欠損は0回となり、非同期修正は有効。
wall72.981s、outer exit1、cleanup error0、停止実測前にホストがfreeze終了。
RViz購読rviz2、終了後active container0、既存Compose39個を保全。
実速度最大1.301809907m/s (4.6865km/h)。最初の拒否は実操舵-0.040193655rad、
候補入力-0.063965410rad、前回入力-0.070386117rad。
47ファイルをremote→Windows→native WSLへhash検証して保全した。
archive SHA256 `34e42442e5583123ff2f1387a4d72196729220d35c0e58341c0bed989c5a70a0`。

原因所有層を操舵指令の単位変換へ絞る。実行中に読み込まれたvehicle.yamlは
`gripSteerFactor: 0.6`。同一hashの実行DLLではAckermann rad→符号反転deg→
入力上限30deg→0.6倍→遅延/応答処理→車輪へ設定し、reportは実車輪角をradへ戻す。
現制御はPPの物理角をそのまま入力していた。WSLでturn09の320点を比較すると、
0.15s遅れの最小二乗倍率0.599981934、0.6倍の平均絶対誤差0.000886415rad、
倍率1では0.020714471rad。これは受信時刻の遅れも含む集計で、完全な動特性同定ではない。

必要な変更: PPの物理角を0.6で割ってAWSIM入力へ変換する専用profileを追加する。
既存入力上限±0.5rad、rate0.8rad/sを維持し、到達不能な物理角は明示拒否する。
監視へは実操舵と、入力を0.6倍した物理目標角を渡す。AWSIMの設定/物理/重みや
生の予測経路は変更しない。改善指標は要求物理角と実操舵の整合、および以前の
コーナー入口通過。asset hashが違えば開始前に拒否し、旧profileで切り戻せる。
左右の倍率・上限・rate・異常値のunit testと、非ゼロ操舵のROS shadowを通してから
新規turn10を1回実施する。turn09の同条件再実行や監視しきい値の緩和はしない。

記録評価器も、PP計算後の監視拒否をPP計算不一致と誤判定していたため修正する。
数学計算の再現と、その後の監視で拒否した件数を別に示す。既存の失敗した
`evaluation08/` は保全し、新しい出力先で再評価する。

turn10設定: `configs/control/time_path_calibrated_turning_5kmh_20260913.json`。
PPの許容物理角0.5radに対し、このAWSIMの入力上限0.5radで実現可能な物理角は0.3rad。
0.3rad超は `STEERING_ACTUATOR_INFEASIBLE` として制動する。監視やPPの要求角自体を
clampして隠さない。COMMAND_SENTのsteer_radは従来どおり送信入力で、detailsの
steer_radはPPの要求物理角。steering_actuatorに倍率・rate適用後入力・物理目標を記録する。

```bash
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q \
  tests/test_awsim_steering.py tests/test_time_trial_replay.py tests/test_turning_scan_guard.py tests/test_time_trial_v1.py
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python tools/evaluate_time_awsim_trial.py \
  --run /home/thistle/e2e_autonomous/runs/time_turning_20260913/codex-time-turn-09 \
  --output /home/thistle/e2e_autonomous/runs/time_turning_20260913/evaluation09
```
