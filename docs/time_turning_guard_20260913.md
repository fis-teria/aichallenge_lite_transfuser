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

## turn12実行前確認

source `8fa561fef0f5f5f8844a582805b4d4bc6e58f5b3`、Windows/WSL一致。
WSL限定69passed (1.63s)、全体2116passed/4skipped/63warnings (87.23s)。
6曲率区間×4変動方式×3001距離点の全車体4頂点を独立積分し、64半平面内を確認。
保存turn11拒否scanの実装版回帰でも、実操舵と指令の差を残して通過する。
Humble build0.93s、11module一致。ROS smoke実モデルPath6、support guard111、
左右29、到達不能曲線の制動10、stale11/clock7/overspeed12、child exit0。
archive SHA256 `d22b7d5f2663c4a6535ac766b2342d7a9d4ce22496ff8e530f3050f6be410f55`、
config SHA256 `ee10307dd73f30d0169d1517e4de78281cc210d0c5f40b2b5e81fab9cd6e01e1`。
専有root `/home/graneple/e2e_autonomous/time_turning_support_20260913`、SOURCEは
その中の `source_8fa561f`。設定 `configs/control/time_path_support_turning_5kmh_20260913.json`、
run ID `codex-time-turn-12` として、同じ710s+10s外側timeoutで1回実行する。
正常目標5km/h、生のTimePath、通常RViz、scene/vehicle/DLL/重みは同一。

## turn12結果と速度に対するPP preview不足

turn12は発進31.939999286sim秒後に `CONTROL_STOPPING_SWEEP_OCCUPIED`、未完走。
639回追従、最大速度1.298153m/s。停止地点yaw1.688462radで、turn11よりさらに約1.75m先。
wall75.447s、cleanup0、active0、既存Compose39個保全、ホストfreeze前の停車実測なし。
48ファイルhash検証済み、archive `7bceccfa403f4ba71e66943eaecbf9c2c4ae4bc0c31bc9ee85a5536a0823f549`。
実装版support guardはturn11保存scanで+0.034383m、WSL計算median3.595/p954.789ms。
これは保存scanの証拠でありturn12通過の保証ではなかった。

turn12の拒否時は実操舵-0.101294rad、PP要求-0.091356radでハンドルを戻す方向。
実操舵固定のsupport再計算は余裕+0.046956m、要求角固定では拒否となる。
同じ未変更の残存経路は1.8m点で-0.116605rad、2m点で-0.118627radを要求しており、
近い点だけを見ることが、曲がりが強くなる先の情報を使うのを遅らせている。
監視の追加緩和ではなく、PP目標点選択が今回の原因所有層。

`stopping_preview_v1` を追加し、最低preview距離を
max(1m, 0.4+v*0.5+v²/2)へ速度連動させる。探索幅は従来と同じ+0.5m、
経路の元の点を時間順に選び、実現可能物理角±0.3radを守る。速度・余裕・停止距離や
監視方式はturn12と同一。未変更の経路に適切な点がなければ既存の拒否を維持する。
改善指標は記録された拒否瞬間で、実操舵/前回入力を残して新指令の監視が通ること、
その後のコーナー実走。turn12 regressionと走行速度でのROS fixture後にturn13を1回実施する。

## turn13実行前確認

source `50b83a765294ec8346bcbb04da1203771b8a768f`、Windows/WSL一致。
WSL限定75passed (1.62s)、全体2122passed/4skipped/63warnings (87.74s)。
turn12回帰で実操舵/前回入力/rate limitを残した新PP指令が同一support guardを通過。
0〜6km/hのpreview距離/上限/元経路不変を検証した。
Humble build0.92s、11module一致。ROS smokeの左右旋回は実測速度fixture1.2m/sで実施し、
preview1.72m以上の適用28件、実モデルPath6、support guard111、到達不能曲線の制動10、
stale11/clock7/overspeed12、child exit0。
archive SHA256 `c38c42b3a036ac18fca76d5c968f668ff009cd392964f43f0951c3caf014a92a`、
config SHA256 `f3aad671d0a403e90d1cfa1190fddff2f376d686736770355606337f58269142`。
専有root `/home/graneple/e2e_autonomous/time_turning_preview_20260913`、SOURCEは
その中の `source_50b83a7`。設定 `configs/control/time_path_preview_turning_5kmh_20260913.json`、
run ID `codex-time-turn-13` として、同じ710s+10s外側timeoutで1回実行する。

## turn13結果と固定5km/hでの操舵応答改善

turn13は最初のコーナーを通過し、公式section 0→1→2へ進んだ。
発進79.394998226sim秒後、走行99.717404mで後続コーナーの
`CONTROL_STOPPING_SWEEP_OCCUPIED` により終了、未完走。追従1587回、
最大実測1.308060m/s（4.7090km/h）。通常RVizに未変更のモデル経路を表示した。
wall123.057s、cleanup error0、active container0、既存Compose39個を保全。
ホストfreeze前の停車実測はない。47ファイルのhash検証とWSL評価を完了し、
制御再生1589件一致、最大誤差4.44e-16。archive SHA256
`c97f721287e574dd934012b5dce7a4d8bfd41d6e18ea7ffc150f8fa90ab97cf5`。
証拠は `docs/evidence/time_turning_20260913/turn13/`。

拒否時の実操舵+0.138781radに対してPP要求は+0.145663rad、前回要求は
+0.143785rad。実操舵固定でも拒否、要求角へ即時到達する仮定なら+0.025472m。
同じ観測で目標点を1〜3mへ変えても、実操舵を含む監視は拒否した。
低速化の提案に対して、ユーザーは「固定5km/hを維持して操舵応答の改善を進める」
と明示した。正常目標速度を変更せず、次の原因所有層を操舵の動的応答とする。

同一hashの実行DLLとvehicle.yamlでは、入力gain0.6に加えて遅延0.07s、
一次応答0.02sがある。既存補正はgainのみ。次は物理PP要求の変化率に
0.09sの先行補償を与え、既存の入力角±0.5rad・入力変化率0.8rad/s内で指令する。
変化率はシミュレーション時計で求め、フィルタ・上限・初期化を明示する。
到達不能な元のPP要求は引き続き拒否し、補償だけを残りの角度余裕で制限する。
操舵reportのstamp/受信時刻も記録し、report遅れを物理遅れへ混同しない。
監視には引き続き実操舵、前回と今回の物理指令を渡す。監視、車体寸法、
停止距離、モデル経路、重み、AWSIM物理設定は同一。独立した遅延/一次応答モデルの
回帰、WSL全体テスト、Humble smokeが通った後、新規turn14を有限枠で1回実施する。

検証/実行コマンド（Windows commit後に `tools/sync_to_wsl.ps1` で同期）:
```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q tests/test_awsim_steering_response.py tests/test_awsim_steering.py tests/test_time_trial_replay.py tests/test_time_trial_v1.py tests/test_curvature_support_v2.py tests/test_turning_scan_guard.py
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
# .10専有deploymentでHumble buildと隔離ROS smoke後、run ID未使用を確認して実行:
timeout --signal=TERM --kill-after=10s 710s python3 SOURCE/tools/run_time_path_awsim_trial.py --deployment DEPLOYMENT --run-id codex-time-turn-14 --display :1 --config configs/control/time_path_response_turning_5kmh_20260913.json
```

## turn14実行前確認

source `c2e264f507a72b8e99b2dd859467c3045c2ab733`、Windows/WSL一致。
WSL限定97passed (1.66s)、全体2144passed/4skipped/63warnings (80.66s)。
独立した遅延/一次応答plantのramp/sineに対し平均誤差が旧補正の45%未満、
最大誤差60%未満であることを検証。角度/rate上限、元要求の到達不能、時刻reset、
保存state連続性の再生検査を通した。Humble build0.91s、12module一致。
隔離ROS smokeは実モデルPath6、support guard110、左右28、先行補償10件、
到達不能の制動10、stale11/clock7/overspeed12、child exit0 (18.722s)。
archive SHA256 `3883f46d421fb8c681813d1c031ebc16b0f2c4b3a62114d4dbc1a93c31609fc7`、
config SHA256 `2e6f424034ff441001f5e42a545f9ccb45d13234e9296a1393a5ad6aa900a21c`。
専有root `/home/graneple/e2e_autonomous/time_turning_response_20260913`、SOURCEは
その中の `source_c2e264f`。上記run ID、710s+10s外側timeoutで1回実行する。
開始前active0、既存Compose39個。正常目標5km/h、通常RVizを維持する。

## turn14結果と残る旋回量の問題

turn14は最初のコーナーを通過、公式section 0→1→2へ進んだが、後続角で
`CONTROL_STOPPING_SWEEP_OCCUPIED`。発進79.654998220sim秒、記録移動99.420366m、
追従1588回で未完走。正常指令は全て5/3.6m/s、実測最大1.308408m/s（4.7103km/h）。
通常RVizの実購読・画面記録あり。wall123.187s、外側124.140s、cleanup error0、
終了後active0、既存Compose39個とremote Gitの既存493行を保全。
ホストfreeze前の停車実測はない。48ファイル、59,535,360bytesをhash検証。
archive SHA256 `048252c031fcdb1df3634bc0cf4bf1d10fa2c379803bc1e9778010a44103f218`。
WSL評価の制御再生1594件、actuator/response再生1589件一致、最大誤差4.44e-16。
計算とstate連続性の再生であり、全scan判断の再生とは区別する。

先行補償は1576指令で有効、補償量は-0.014884〜+0.033716rad。
操舵report取得からのsim経過時間は中央値15ms、p95 30ms、最大35ms。
実行DLLのMultiDomainROS2ManagerはUnity Updateで受信を処理する。
発行済み指令のzero-order holdに固定物理遅延70ms・一次応答20msを適用した
保存時系列の当てはめでは、追加35〜40msの遅れを含めると平均絶対誤差約0.00026rad。
これは当該走行の事後推定であり、追加遅延をruntime定数へ採用する根拠とはしない。
独立plant回帰に追加40msの輸送遅れを加えた感度確認も追加した。

拒否時は実操舵+0.138832rad、PP要求+0.144389rad、補償後+0.146836rad。
このturn14の保存scanは、要求へ即時到達する仮定でも拒否する。
全区間を+0.16rad固定と置いた仮定ではray余裕+0.009665m、+0.17radで+0.048179m。
これらは仮定した幾何計算であり、その場で実行可能な操舵や実車体の空き距離ではない。
元の経路から距離1m以上の任意の点を選んでもPP要求は最大+0.150262rad。
したがって今回の残りは、単なる指令遅れや目標点の入れ替えだけでは説明できない。
先行補償による実走改善や後続コーナー通過は確認できていない。

さらに推定姿勢の0.5s差分yaw rateを `v*tan(実操舵)/1.087` と照合すると、
turn13/14の最小二乗比は0.90990/0.91144、操舵変化が小さい区間でも
0.90938/0.91324だった。ただしEKF推定姿勢は独立したground truthではなく、
この比をそのまま操舵gainやwheelbaseへ入れていない。
停止範囲の数学的包絡は指定した曲率区間に対するもので、PhysX車体の横滑りや
推定姿勢誤差まで保証したものではない。制御側だけ倍率を上げると監視側の
運動仮定との整合が崩れるので、次の原因所有層は車体yaw応答とPP/監視の共通運動モデル。

次の実装前に必要な切り分け:
1. 同時刻の速度・舵角・IMU yaw rate・推定姿勢のyaw rateを照合し、
   車体の旋回不足と姿勢推定の遅れを分離する（現control記録はIMU yaw rateを持たない）。
2. その結果からPPの要求曲率→物理操舵と停止包絡に同じ運動契約を適用する。
   未変更のモデル経路そのものの曲がりが不足する場合は別にモデル誤差として扱う。
3. 左右・定常旋回・過渡応答・到達不能・監視を検証してから新規run IDを発行する。
   固定5km/hの指定は継続し、未検証倍率によるturn15は実施していない。

小さい証拠一式は `docs/evidence/time_turning_20260913/turn14/`。
raw scan/全指令/全予測はWSLの `runs/time_turning_20260913/codex-time-turn-14`。
評価の再実行は同じWSL lock内で、既存出力を上書きしない新しいoutputを指定する:
```bash
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python tools/evaluate_time_awsim_trial.py --run /home/thistle/e2e_autonomous/runs/time_turning_20260913/codex-time-turn-14 --output /home/thistle/e2e_autonomous/runs/time_turning_20260913/evaluation14_recheck
```

最終検証commit `9e00781bb524302a2ff3e151fa864a2a2aa02ca4` は、turn14の実行source
`c2e264f5` とsrc/ROS/tools/configsが同一。追加40ms感度回帰を含めWSL限定99passed
(1.76s)、全体2146passed/4skipped/63warnings (87.27s)。既存optional依存4skipは継続。
実走コーナー課題は未解決として記録し、シミュレーションは終了済み。
