# TimePath 復帰教師の収集方法と現地確認

## 判断

2026-09-13追記：ユーザーがAWSIM実行環境での収集開始を指示。
この文書を今回の実行・進捗の参照元とする。最初の完了条件は、左右0.20mの各1周を
固定目標5km/h、自車1台で記録し、正常停止、復帰の実測、原本転送とSHA-256を確認すること。
失敗試行は保全して原因を分類し、同じ物理的失敗を条件変更なしで繰り返さない。

実行準備では、既存Pure Pursuitの名目commandを専用収集topicへ出し、
現在の `check_turning_scan` / `select_aligned_scan` / `scan_pose_in_rear` と
`awsim_understeer_v1` による監視を通して、単一の最終command発行元からAWSIMへ渡す。
cameraを含むfreshness、元stamp、発行元、速度、操舵・加速度の上限を記録・検証する。
モデルの学習・推論経路、共通監視の計算式や閾値、既存実験の設定は変更しない。
公式controllerは固定5km/h、復帰用CSV以外の追従parameterを途中で調整しない。

変更の必要性：現行収集には新しい復帰区間の注釈と、現在の監視へ名目commandを接続する
入口がない。目的は教師の復帰区間を実測で採れるようにすることであり、走行合格条件の緩和ではない。
生成・注釈とcollection用の新規entrypointに変更を限定し、既存入力・出力契約を保つ。
新規純粋関数はWSLの回帰テスト、ROS接続は停止状態、最後に各1周のAWSIMで確認する。
切戻しは当該runの停止と専用生成物の不使用で行い、既存source/install/Composeを上書きしない。

実行先は `/home/graneple/e2e_autonomous/time_recovery_collection_20260913` の新規出力。
現在のsource/build一致を保証するため、公式PPとtrajectory generatorの必要packageのみ
専用build/installへ再構築する。既存dirty checkoutはread-onlyの入力とする。
過去の1周bag実容量を踏まえ、初回の容量上限は各3GiB、2試行合計6GiBとする。
開始前に10GiB以上の空きを要求し、30分sim / 31分wall / 33分外側上限で監督する。
現時点の最初の未成立条件は「現行の実行物に対応した収集経路・監視接続・区間記録」。

実行先 `graneple@192.168.3.10` で収集する方法は見えている。
優先案は、既存Pure Pursuitに専用の参照経路を追わせ、走行中に小さな横ずれを
作ってから元の適切なラインへ戻す方式。カメラ・LiDARも実際のずれた姿勢で取得する。
以下の現地確認は収集前の記録。収集実行の結果は末尾へ追記する。

先行根拠: `time_teacher_clearance_comparison_20260913.md`。
通常走行上のモデル誤差は小さいが、turn16では教師ラインから約85cm左へずれて
停止判定に至った。制御がずれの発生初期へ与えた影響は未確定なので、収集時も
command、実舵角、速度、yaw rateを保持し、復帰能力と追従を別々に確認する。

## 2026-09-13 現地で確認したもの

- 実行先はGPU利用可能、RTX4060 Laptop、driver595.91.07。
  active container 0、停止済み等の既存Compose 39件を保全。
- 初回のroot filesystem空きは約8.9GiB（使用率97%）。同日の容量整理で、ユーザーが許可した
  キャッシュとWSL移行済みの旧復帰教師2コピーを削除し、空き22.17GiB（使用率91%）を確認。
  全コピーのSHA-256照合と削除後の原本確認は `pc10_storage_cleanup_candidates_20260913.md` に記録。
  新しい収集前にも空きを測定し、大量の連続収集を無制限に置ける前提にしない。
- 現在の基準CSVは
  `/home/graneple/git/autononous_ai/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/simple_trajectory_generator/data/raceline_awsim_15km.csv`。
  SHA256 `c563a757a243a2521a68acacd565492e1ea158db7e7d68327682583d24fc79e7`。
  座標はmap、8列のpose/quaternion/speed形式。ファイル名の15kmは今回の速度指定ではない。
- `simple_trajectory_generator.cpp` は7列以上のReference形式と8列pose形式を読み、
  `csv_path`の変更callbackも持つ。読込み失敗時は新経路を受理しない。
  今回はソース確認であり、実行中binaryでの切替検証はまだ行っていない。
- Pure Pursuitは外部目標速度に加え、同じtrajectoryの速度profileも上限として使う。
  `external_target_vel`だけを変更して固定5km/hになるとは扱わない。
- AWSIMの`vehicle.yaml`には`start.positions`の座標/yaw設定例があり、
  `--start-random`の記載もある。旧収集ではseed指定による物理開始位置の変化を
  記録で確認済み。ただし今回の場所・yawを指定した開始は未実行。

参照実装:
`tools/generate_recovery_reference_v3.py`、
`src/aic_transfuser_lite/data/recovery_reference_v3.py`、
`tools/record_dataset_v3.py`、`docs/v3_recovery_data_reference.md`。
旧収集実績は `docs/v3_teacher_collection_pilot_20260904.md`、
時間教師の収集/終了処理は `docs/time_teacher_si26_speed_laps_20260911.md`。
旧MPC・0.75m/sの設定を、今回のPure Pursuit・目標5km/hへそのまま実行しない。

## 優先する収集の流れ

1. `.10`のAWSIM、車体設定、使用センサ設定、基準経路、教師controllerを固定する。
   source/install/loaded configを照合する。既存dirty checkoutは上書きしない。
2. 基準CSVのコピーから、直線またはコーナー出口の1区間だけを滑らかにずらした
   収集用経路を作る。通常の基準ラインは横ずれ評価用に固定する。
   点間の連続性、曲率、現行車体の停止時の通過領域と路端余裕を事前確認する。
3. 同じ教師controllerが「通常→横へ寄せる→維持→元へ戻る→通常」と走る。
   初回は固定された1本のCSVとして起動前に読み込ませる。走行中に突然経路を
   切り替えたり、位置推定だけをずらしたりする必要はない。
4. 最初のpilotは左+0.20m・右−0.20mの各1周。正常目標は5/3.6m/sを維持する。
   外部速度・trajectory速度profile・最終command・実速度を別々に記録して照合する。
   過去の「上限5km/h教師」の実速度が約3.18km/hだった点を繰り返し見落とさない。
5. 復帰後も通常走行を記録し、1周完了後に少なくとも4秒の将来軌道を確保して
   正常制動する。30分は上限であり、単に3分経過したことを正常終了条件にしない。
   fault/接触/入力異常時はその時点で中止し、正常1周とは別に保存する。
6. bagを閉じてからWSLへ転送し、hashとセンサ同期、元stamp、操舵、実速度、
   実測復帰量、停止確認を監査する。開いているSQLite bagには検査をかけない。
7. pilot成立後、左右0.40m、場所、向きずれの組合せを増やす。
   3度/6度などの向きずれ帯は実測で分類する。位置と角度を独立に制御する
   追加ケースには物理初期姿勢設定を使えるか別途検証する。
   最大85cmを最初から全箇所へ与えず、復帰可能な通過領域を確かめて範囲を広げる。

目標は独立した復帰事例を増やすこと。生フレーム数や同じ周回の繰返しだけで
十分性を判定しない。pilot各1周は収集方式の検証であり、学習に十分な量ではない。

## 教師へ混ぜる区間の扱い

| 区間 | 保存 | TimePathの正解として採用 |
|---|---|---|
| 横ずれを作る | センサ・状態・実行commandを保存 | しない |
| 横ずれを維持する | 同上 | しない |
| 元の通常ラインへ復帰する | 同上、実測復帰量も記録 | 品質を確認して採用 |
| 復帰後の通常走行 | 同上 | 品質を確認して採用 |
| collision、reset、収集終了の制動 | 理由・開始時刻も保存 | 復帰教師にしない |

重要な追加実装はこの区間管理。旧generatorはholdも`training_eligible=True`なので、
旧eligibilityを今回の時間教師へ流用しない。TimePathの現行corpusはrun末尾の
単一intervention時刻を主に扱っており、複数の区間を安全に採用/除外する処理が必要。
最初の復帰用対応では、既存cruisingの結果を変えず、区間注釈と将来3秒のmaskを
別の採用条件として加える。phase境界とセンサ/制御の元capture時刻を保持する。

## 今回の実行コマンド

Windowsでcommit後、`tools/sync_to_wsl.ps1 -CheckOnly` と `tools/sync_to_wsl.ps1` を実行する。
WSLの正本checkoutで以下を実施する（入力・出力はWSL native filesystem）。

```bash
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python tools/generate_time_recovery_collection.py \
  --inputs /home/thistle/e2e_autonomous/runs/time_recovery_collection_20260913/inputs \
  --output /home/thistle/e2e_autonomous/runs/time_recovery_collection_20260913/references
```

`.10`の専用sourceへ同一commitを展開し、WSL生成のreferencesをSHA照合して配置する。
source/build/installの証拠は専用rootの`input_source_sha256.json`、`cpp_build_result.json`、
`deployed_commit.txt`に保存。Docker実行は以下のhost entrypointが監督する。

```bash
cd /home/graneple/e2e_autonomous/time_recovery_collection_20260913/source
PYTHONPATH=src python3 tools/run_time_recovery_awsim.py \
  --run-id codex-time-recovery-left020-r01 --side left
PYTHONPATH=src python3 tools/run_time_recovery_awsim.py \
  --run-id codex-time-recovery-right020-r01 --side right
```

`--preflight-only`は開始要求を送らず、停止状態の接続・経路表示だけで終了する。
走行runも同じ停止状態の検証を通過してから開始する。
生成されたphase境界は基準courseの連続した距離で定義し、V3の丸めたpoint-index区間は流用しない。
`recovery_teacher_mask`はアンカーから将来各点まで適格区間が連続する場合だけTrue。
実データの同期・将来pose maskとのANDが別途必要であり、生bagを既存のcruising教師へ直接投入しない。
現時点では学習コーパスへ追加する処理、学習再開は実施しない。

横ずれ生成/維持の画像は復帰時の**過去入力履歴**として使える。
学習anchorの除外と履歴データの物理削除を混同しない。
復帰開始を予定区間名だけで数えず、実測した横ずれとheadingの変化も確認する。
初回の採用候補は、正しい側の横ずれが実際に生じ、3秒以内に十分減少し、
その後も安定して通常ラインへ戻るもの。途中で監視が止めた事例を成功へ数えない。

教師XYは従来どおり元観測から0.1〜3.0秒後の実測poseで作る。
予定CSVや地図をfuture XYの代用品にしない。reference、phase、横ずれ評価値は
teacher/debug-onlyで、モデルの推論入力へ入れない。
splitは収集run/episode群単位で先に決め、同じ復帰動作の連続frameを分散しない。
既存の最終テスト周回も引き続き触らない。

## 記録と実行の条件

カメラ、CameraInfo、LiDAR、IMU、GNSS、元stamp付きpose/odometry、
VelocityReportの前後/横速度とheading_rate、実SteeringReport、教師nominalと
最終command、clock、TF、AWSIM状態、介入/phaseイベントを記録する。
衝突・監視判定はその実行物で取得可能なauthoritativeな信号/ログを固定する。
存在未確認のcollision topicに件数0があっても「無衝突」と解釈しない。
通常RVizでは元の基準ライン・収集用経路・実測軌跡を見られるようにする。

1台のみ、no NPC、単一の最終command発行元、現行監視、時刻/センサfreshness、
操舵上限を維持。復帰用経路以外のcontroller/actuator/safety設定を同時調整しない。
教師が目標5km/h条件で復帰できない場合は、データを増やす前にその理由を切り分ける。
pilot単位で停止確認し、次へ進む。正常制動の実測確認前にfreezeだけで成功扱いしない。

空き容量は毎回事前確認し、pilotは各3GiB、合計6GiBを上限とする。
実際の記録量を見て更新し、足りなければ新しいrunを開始しない。
WSL転送後も既存データやremote bagを勝手に削除しない。
大量収集への拡張は保存先容量/転送方式を決めてから行う。

## 現在地と次の実装

手段の候補・現地ソース・過去実績・必要な収集内容を確認済み。
次に必要なのは、現在の時間教師向けのphase採用処理、コピーした復帰用CSV、
現行監視を維持した収集runnerへの接続、および固定5km/hでのpilotである。
ここまでを確認せず旧runnerで大量収集は開始しない。

現地事前確認の後、collection専用entrypointとphase maskを追加した。既存モデル・共通監視は変更していない。
専用C++ buildは成功。WSLの全pytestと参照生成、停止状態のROS接続、pilotの順で確認中。
AWSIM自身のlap上限は2に設定するが、収集停止条件は最初の検証済み1周+4秒とする。
1周でAWSIM自体が終了して将来poseが欠けるのを避け、collectorの30分上限も維持する。

WSL full pytest: 2,209 passed / 4 skipped / 63 warnings（95.38s、598488e）。
左右0.20mの参照生成と新規entrypoint 4本のPython構文検証が成功。
左右とも基準s=233.233～257.233mの同一区間を選択。復帰区間はs=247.233～257.233m。
left020-r01は公式start未要求・走行前に終了。sidecarのPYTHONPATH上書きによりros2cli
metadataを見つけられなかった。ROS環境の既存PYTHONPATHへsource/srcを追加する形へ修正。

left020-r02: 正常RVizと単一最終publisher、PPパラメータの読取は成立。
start後も新collectorのnominal許容値0.5radが公式PPの0.64rad上限と不一致で、発進しなかった。
WSLで閉じたbagのSQLite検査ok、PP最大値0.639999986rad・実速度0、初期横ずれ+0.664mを確認。
公式PPの名目入力は0.64radまで受理し、最終入力は従来どおり0.5rad/0.8rad/s・加速度±1m/s²で制限する。
停止監視に渡すのは制限後の入力×実装済み0.6応答gain。共有監視とphysicsは変更しない。
名目入力の拒否でarm/stop要求の処理まで飛ばしていた順序も修正する。

left020-r03: 発進後10.455sim秒でSTOPPING_SWEEP_OCCUPIED。0.03m/s未満を3sim秒確認して終了、bag正常close。
指定0.20m区間まで到達していない。基準CSVは121点で最後が先頭の閉路点だったが、
生成器が内部の周期表現120点をそのままCSVへ出力し、閉路の約3mを落としていたことを確認。
公式PPはnearest以降の配列だけを探索し、末尾に達すると末尾点へfallbackするため、
開始姿勢の少し後方にある誤った末尾点へ操舵し続けた。新規exportの不具合であり、モデル学習の問題ではない。
exportに閉路区間と先頭12m以上の連続するprefixを付ける。物理座標・速度・PP tuning・監視は変更しない。
基準sによるphaseと評価の周期表現は120点のままとし、export形状の回帰テストを追加する。

e11ad65のWSL full pytestは2,210 passed / 4 skipped / 63 warnings（80.80s）。
left020-r04は修正した閉路を走行。初期横ずれ+0.664mから約−0.011mへ10.2秒で復帰した。
その直後、pose stamp=31.775sに対してcollectorのclock=31.735sとなり、未来20msの上限で停止。
pose自体の欠測ではなくclock受信キューの遅れが疑われる（旧depth10で200Hzなら最大約50ms）。
公式PPのclock QoSと同じBEST_EFFORT/depth1へ変更し、最新clockを受け取る。
元stampとfuture/stale閾値は維持。正常停止の3秒確認、bag closeは成立。指定20cm区間はまだ未到達。

left020-r05: 時計QoS修正後18.705秒走行し、STALE_cameraで正常制動・停止を確認。
同時刻のbagではカメラが約105ms間隔で継続しており、collectorへの配送欠け/遅延が疑われる。
カメラpublisherはBEST_EFFORT/depth1（bagのoffered QoSで確認）。RELIABLE受信への変更は不適合なので行わない。
新collectorが全sensorへ150msを当てたのは、既存TimePathのカメラ契約と不一致だった。
`time_path_node.py` のcamera anchor/receipt admissionは500msであるため、cameraだけ既存500msへそろえる。
pose/velocity/steering/scan/nominalは引き続きcapture150ms、receipt300ms、未来20ms。
この違いを回帰テストで固定し、各入力の元stamp・receiptもcontrol.jsonlへ追加する。
bagの欠損/履歴/将来poseの品質検査は別に行い、watchdog通過だけで教師として採用しない。

6681552でWSL full pytestは2,211 passed / 4 skipped / 63 warnings（98.99s）。
閉じたr04/r05の全転送manifest・SQLite・実pose投影・sensor header・phase maskの監査もWSLで成功。
両runとも指定復帰区間未到達として扱い、正常pilotや学習採用データの件数へ含めない。

監査の再実行例（rawは書き換えず、結果はrunsへ出力）:

```bash
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  tools/audit_time_recovery_collection.py \
  --run /home/thistle/e2e_autonomous/raw/time_recovery_collection_20260913/codex-time-recovery-left020-r06 \
  --types /home/thistle/e2e_autonomous/runs/time_recovery_collection_20260913/types \
  --output /home/thistle/e2e_autonomous/runs/time_recovery_collection_20260913/left020_r06_audit.json
```

Windows→WSLの`git fetch /mnt/e`がPlan9ドライブI/O待ちになった際は、当該所有fetchだけを終了し、
Windowsで作った同一commitの差分Git bundleをWSL nativeへ転送してobjectを先にfetchした。
その後、通常の`sync_to_wsl.ps1 -CheckOnly`/`sync_to_wsl.ps1`でclean/SHA/protected-path検証を通して同期した。
強制resetやignored datasetの置換は行っていない。

left020-r06: 約117.1秒、基準s=107.6mまで走行しSTALE_nominalで停止。正常制動と3秒停止確認、bag closeが成立。
failureのclock=139.200sに対して新着nominal=139.225s（25ms未来）、直前nominal=139.195sはまだ5ms古いだけで適格。
最新着1件で置き換える方式ではDDSのtopic間到着順を処理できなかったため、有界historyから現在clockに適格な最新sampleを選ぶ。
元stamp/receiptを保持し、未来20ms、各roleのcapture/receipt期限、source/skew検査は変更しない。
適格sampleがなければ従来どおり停止。poseも選択したstampの実測poseを使用し、新着の未来poseへすり替えない。
この実測ケースとclock追従・期限切れを回帰テストに追加した。

d79c0f2のWSL full pytestは2,212 passed / 4 skipped / 63 warnings（90.44s）。
left020-r07は約178.5秒、s=160.34mまで走行しSTATE_FRAME_OR_CAPTURE_SKEWで停止、3秒停止確認・bag close成立。
直前にcallbackが約140ms遅延し、最新clock=199.790s、pose=199.715sに対してvelocity/steering=199.640sだった。
motion側のDDS depth5が古いreportを順に配送する構成を、clock同様のBEST_EFFORT/depth1へそろえる。
受信後の有界historyは維持し、pose/velocity/steeringを独立に選ぶ代わりに、
各capture/receipt期限と元の50ms skewを同時に満たす最新の実測3組を選ぶ。
適格な組がなければ停止する。r07の記録済み前pose=199.630sは160ms古いため救済に使わない回帰テストも追加。
未受信の補間値を作らず、時計・stale/skew閾値・車両/制御/監視パラメータは変更しない。

717c409のWSL full pytestは2,214 passed / 4 skipped / 63 warnings（120.88s）。
left020-r08は発進1.49秒後にFRESH_ALIGNED_SCAN_MISSING。新着scanはcurrent poseより約1.6ms未来で使えず、
前scan=22.0366sの補間に必要な中間poseもdepth1受信では抜けた。正常制動、3秒停止確認・bag close成立。
poseは前後の実測補間点が必要なので従来sensor-data depth5へ戻す。velocity/steering/nominalはdepth1、
motionの同時刻帯選択は維持する。欠けたposeを外挿で埋めず、scan選択・各50ms補間端点期限を保持する。
新着scanが未来の場合と、中間poseを欠いた場合/保持した場合のscan選択を回帰テストで確認する。

3372f07のWSL full pytestは2,215 passed / 4 skipped / 63 warnings（86.74s）。
left020-r09はs=98.70mでFRESH_ALIGNED_SCAN_MISSING、制動・3秒停止確認・bag close成立。
callback遅延後に最新poseとvelocityが60msずれ、同時刻帯にそろえられる前のmotionは既に約150ms古く、
対応scanが150msを超えていた。受信QoSのみでは制御計算/graph/disk IOによるcallback待ちを解消できない。
センサ受信Node/executorを独立したthreadへ分け、短いlock内で取得した不変snapshotだけを制御へ渡す。
受信中のclock reset/pose異常はsnapshot世代とfaultで検出し、publish直前に同じ採用sampleの期限を再検査する。
実際に監視へ使ったscanを再検査対象とする。起動前は100回連続READY（約5秒）を必要とする。

収集bagの因果再現検査で別の保存設定の不具合を確認した。新runnerの`--use-sim-time`ではbag receiptが
同じsim clock値にまとまり、r07は既存epoch detectorで7,434区間に分断された。これは入力の受信順を
独立した時計で再現する既存TimePath契約に適合しない。r01〜r09は診断用として保全し、学習採用しない。
新runは通常のrosbag system-time receiptを保持し、sensorの元sim headerと`/clock`を別に記録する。
既存epoch判定やcausal selectorを緩めず、新bagで1 epochと実入力/将来30点の再現を検証する。

35b0713のWSL full pytestは2,215 passed / 4 skipped / 63 warnings（117.70s）。
preflight-r10は無走行で100回連続READY、通常RViz、正常bag close成立。49filesのhash検証・SQLite検査がPASS。
修正bagは既存readerで1 epoch、fallback 0。既存学習/実推論と同じreceipt+50ms freezeで、実画像・LiDARから
入力[1,4,3,224,384]/[1,4,2,750]と将来XY[30,2]を再現し、全30点が実観測と一致した。
これは停止中の接続確認であり復帰教師の採用例ではない。比較用freeze0ではcurrent velocityが未到着で除外された。

left020-r11は発進約5.89秒でSTALE_scan、停止確認・bag close成立。
採用scanはsnapshot時点では約72ms古いが、計算約59msとsim時計の進行によりpublish直前に150msを超えた。
期限再検査は維持する。時刻関連のsnapshot失敗に限り、新たに受信した実測snapshotで最大1回再計算する。
初回80ms以内のみ再計算を許し、snapshot取得からpublishまで100msを超えたgo指令は拒否する。
同じscanを再利用して期限を変える処理ではない。再計算でも不適格なら制動・fault latch。
障害物、速度、source、clock reset、車体/scan alignmentの異常は再計算対象にしない。
