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
bash tools/with_wsl_training_lock.sh .venv/bin/python tools/generate_time_recovery_collection.py \
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
