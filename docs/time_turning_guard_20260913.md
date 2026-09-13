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

## turn10実行前確認

source `ee708287b6419056af8328a1a086dff99e8e1132`、Windows/WSL同一commit。
WSL限定47passed (0.38s)、全体2094passed/4skipped/63warnings (80.20s)。
Humble build0.91s。10module source/install一致。ROS smokeで実モデルPath一致6、
左右の非ゼロ操舵変換28、到達不能角の制動10、stale11/clock7/overspeed12、child exit0。
最初のhash検証補助コマンドは存在しないvisualizer filenameを指定して失敗したが、
実際のPath配信元time_path_node.pyを含む正しい10moduleで再確認して一致した。
archive SHA256 `e0416a392f64044a42702fbdf65871f3c25ccd5e65830c8331e6d970e1f8fb8b`、
config SHA256 `f3bb6675487acd841b6575eda050b24822d7f1b08ea8820b9afc3752a2f52f20`。

専有root `/home/graneple/e2e_autonomous/time_turning_calibrated_20260913`、
その中の `source_ee70828` がSOURCE。車両設定とDLLは実行前hash照合が必須。

```bash
timeout --signal=TERM --kill-after=10s 710s python3 SOURCE/tools/run_time_path_awsim_trial.py \
  --deployment /home/graneple/e2e_autonomous/time_turning_calibrated_20260913 \
  --run-id codex-time-turn-10 --display :1 \
  --config configs/control/time_path_calibrated_turning_5kmh_20260913.json </dev/null
```

## turn10結果と目標点の到達可能性

turn10は発進せず `STOPPED_NO_LAP / PROGRESS_STALLED`、停止実測確認あり。
wall49.321s、outer exit1、cleanup error0、active container0、既存Compose39個保全。
103回の `STEERING_ACTUATOR_INFEASIBLE` が原因。要求物理角の絶対値は
0.313707〜0.316088rad、15回はAWSIM自体の30deg×0.6限界も超えている。
入力上限をAWSIM限界へ広げても全件は解消しない。48ファイルのhashをnative WSLまで検証。
archive `c2c3f18ec4be8a90316e2629469025aec7865493b70e69d01601bf47cdcd9bb2`。
archive内のlatest log symlinkは内包された対象を確認し、WSLでは通常fileとして検証した。

WSLで同じ未変更の残存経路を調べると、103件全てに1.397〜1.463m先の到達可能点がある。
先頭例は[1.369716,-0.277234]m、要求角-0.299336rad。新しい原因所有層は
PPの固定1m目標点選択。物理上限を考慮せず近い点を選んでから拒否している。

次の変更は `feasible_1_to_1p5m_v1` の明示profileで、残存経路の元の点を時間順に調べ、
1.0〜1.5m内かつ要求物理角±0.3rad内の最初の点を選ぶ。範囲外への探索延長、
点の生成/変形、経路の差し替え、角度clampでの救済はしない。該当点がなければ
`STEERING_FEASIBLE_LOOKAHEAD_MISSING` で制動する。全点の既存geometry検証と
選んだ指令に対する旋回監視、時刻/authority/停止距離/overspeedは維持する。
これはPPの目標点選択だけの変更で、他の旧profileは保全して切り戻し可能。
改善指標は未変更の経路から物理上限内の指令を生成できることと、その実走結果。
保存したturn10の回帰例、範囲外/全点到達不能、左右旋回のテスト後にturn11を1回実施する。

## turn11実行前確認

source `d2d7d1e001fd40f73c6396258bb694177c45b25c`、Windows/WSL一致。
WSL限定51passed (0.47s)、全体2098passed/4skipped/63warnings (80.50s)。
Humble build0.90s、10module一致。ROS smoke実モデルPath6、左右28、
到達不能な曲線の制動10、stale11/clock7/overspeed12、child exit0。
新規設定は `configs/control/time_path_feasible_turning_5kmh_20260913.json`。
archive SHA256 `1a9e7cbd9b5e8fce41d47559168cd53a178f09b28f47ffed4f767b26b94f53e9`、
config SHA256 `9c55ee9ce44184de71cb3a1ba754efa5e46f864c5a62a48f7feb1171d2c34370`。
専有root `/home/graneple/e2e_autonomous/time_turning_feasible_20260913`、SOURCEは
その中の `source_d2d7d1e`。前述timeoutコマンドのroot/configをこの値へ、run IDを
`codex-time-turn-11` へ変更して1回実行する。有限枠/実行先/通常RVizは同一。

## turn11結果と監視の幾何境界

turn11は発進30.699999314sim秒後に `CONTROL_STOPPING_SWEEP_OCCUPIED`、未完走。
608回追従、目標点を1.2m超へ選んだのは7回。実操舵は-0.088705rad、要求物理角
-0.097906rad、補正した入力-0.163176rad。0.15s遅れを合わせた実操舵誤差の平均絶対値
0.001148radで、操舵変換の不一致は改善した。停止地点は前回より先へ進みyawも
1.832radまで旋回したが、コーナー通過には至らなかった。
wall74.802s、cleanup error0、ホストfreeze前の停車実測なし。通常RViz購読あり。
終了後active0、既存Compose39個を保全。47ファイルhash検証、archive
`523f0333730448a99ede33ad5ab5f88048aa1250ae944a38dc7401124885c62d`。

保存scanのWSL再計算では、後輪原点[3.942211,0.438657]mの観測点が
膨張した停止範囲に入り、ray余裕-0.004566m。実操舵を固定する計算では+0.020362m、
要求角へ即時到達する計算では+0.083596mだが、どちらも実際の変動操舵の保証には使えない。
元の等方膨張0.055337mは姿勢不確かさを全方向へ広げるため過大な部分がある。
単に矩形の回転区間へ置換する検算でも-0.000475mなので、それだけの変更は採用しない。

次の原因所有層は、変動曲率で到達可能な車体範囲の上界計算。
`steering_support_v2` では64方向の支持半平面で全停止範囲を包む。距離sでのheadingは
[k_min*s,k_max*s]内。各方向の位置射影をそのheading区間で最大化して積分し、
車体4頂点も全heading区間で最大化する。積分は5mm以下で、曲率上限Kに対し
各区間K*ds²/2の誤差上界を加える（必要なmidpoint上界K*ds²/4以上）。sample間の
移動/回転、sensor offset、64面の外接誤差、scan角度間隔の余裕も加える。
これにより曲率が区間内で時間変化する場合も含み、固定操舵への置換はしない。
全停止範囲を凸包へ広げるので内側の空間では過剰拒否が残り得る。

既存車体寸法・余裕・停止距離・操舵区間を保ち、旧v1は保存する。
scan/frame/NaN/authority/時計の拒否とホスト終了を維持。改善指標は保存した偽陽性候補で
余分な幾何膨張を減らし、独立積分した変動操舵軌跡を全て包むこと、その後の実走結果。
実装版のsnapshot回帰、左右/内外障害物/観測不足、最大全曲率区間での包絡テストを通す。
新規turn12を1回実施する判断は、その検証とHumble smokeが通った後に行う。
