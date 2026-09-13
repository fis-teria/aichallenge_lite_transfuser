# TimePath 車体応答・制御・監視の整合

## 現在の目的と前提

ユーザーの「底の整合性をとりましょうか」に基づき、Pure Pursuitが要求する曲率、
物理舵角、AWSIM車体の旋回、停止監視の運動モデルを照合して共通化する。
正常目標5km/h、TimePath B0 epoch10の未変更30点、通常RViz、実行先
`graneple@192.168.3.10`を維持。Windows正本で編集/commit、native WSLのlock内で
学習系テスト/評価を行う。AWSIM物理設定や重みを変更せず、以前のrunを保全する。
先行作業は `time_turning_guard_20260913.md`、本作業の最新実走はturn16。

開始HEAD `16383f2faaea216667914e8e7e5ef605a2c917b1`、Windows clean。
.10はactive container0、既存Compose39個。remote HEAD `bb60929a67d620f2af9bb757417435f65d63064e`。
外部レビュー・push・削除・実車操作は本作業に含めない。

## 先行診断と最初の不一致

turn14は約79.65s/99.42m、後続角でSTOPPING_SWEEP_OCCUPIED、未完走。
推定姿勢yaw rateと `v*tan(実操舵)/1.087` の比が約0.91だが、
元のcontrol記録はIMU/VelocityReport heading rateを持たない。
実車体応答と姿勢推定の誤差を混同しないため、先に同時刻の独立した信号を収集する。

静的シーンの物理WheelCollider配置をWSLで再確認した。
GoKart1 root transform718に対して前輪546/867のz=0.6029999852180481m、
後輪568/756のz=-0.48399999737739563m、差=1.0869999825954437m。
全親変換を合成してscale/rotationを確認し、表示用wheel meshとは分離した。
従来の1.087mは静的寸法と一致する。base_link796のz=-0.48500001430511475mも一致。
寸法を未確認の経験倍率に置換する変更は行わない。

## turn15: 応答を直接照合するための記録

原因所有層はまず観測記録。既存controllerのcallbackで、取得stamp・受信monotonic・
epoch・frameを保った速度(longitudinal/lateral/heading rate)、物理舵角、姿勢、
IMU角速度を独立したJSONLへ記録する。IMUは/sensing/imu/imu_rawの受動購読で、
モデル入力や制御条件に追加しない。送信元のROS graphも記録し、欠損・非有限値は
評価で明示的に除外/拒否する。新設定の記録flagで有効化し、既存設定は保存する。

この変更で改善する指標は、同時刻の車体角速度/舵角/推定姿勢を照合できること。
単なる同一走行の再試行ではなく、不足していた因果切り分けの信号を取得する。
WSLテストと隔離Humble smokeで元stamp、単位、4種類の信号の記録を確認してから、
未使用ID `codex-time-turn-15` を既存one_lap枠（600sim/600wall秒、外側710+10秒）で
1回実行する。制御/監視の数式・限界値はturn14と同一。既存fault/ホストfreezeで終了。
試験後は原記録をhash検証してnative WSLへ送り、取得時刻を合わせて評価する。

その証拠からのみPPと停止監視の共通運動契約を変更し、左右・定常/過渡・
到達不能・安全側境界をテストする。実走結果と数学検証は区別して記録する。

## turn15実行前確認

source `1a32ebf85aadcd121d486b32805c6c63647f77b3`、Windows/WSL一致。
WSL関連100passed/2.10s、全体2147passed/4skipped/63warnings/88.18s。
Humble build0.93s、12module source/install一致。隔離ROSは4役割945観測、
実モデルPath6、support110、左右29、先行補償10、到達不能制動10、
stale11/clock7/overspeed12、child exit0。source archive SHA256
`6fba2d734894345b6add2dcdfda8a04fa71b2fe0e220dc0aa2d46a968afbe48b`、
config SHA256 `f6450189a689261a258a7dc32718c5d93b3fa3eb3d369f0fb236ec6f53c5983d`。
専有deployment `/home/graneple/e2e_autonomous/time_vehicle_consistency_20260913`、
その中の `source_1a32ebf` を使用する。記録flagのみ有効化した設定:
`configs/control/time_path_motion_audit_5kmh_20260913.json`。
```bash
timeout --signal=TERM --kill-after=10s 710s python3 source_1a32ebf/tools/run_time_path_awsim_trial.py --deployment /home/graneple/e2e_autonomous/time_vehicle_consistency_20260913 --run-id codex-time-turn-15 --display :1 --config configs/control/time_path_motion_audit_5kmh_20260913.json
```

## turn15の結果と原因

約78.845sim秒、最初の角を通過し公式section0→1→2、後続角で
`CONTROL_STOPPING_SWEEP_OCCUPIED`。未完走。wall123.06秒、cleanup0、active0、
既存Compose39個とremote Git状態493行を保全。49ファイル63,027,200bytesのarchive
SHA256 `0317321437a7432ba2f72a32c04472be29a82e613535346171c242b0bda35b00`。
native WSL評価は1578制御再現、最大誤差8.89e-16、scan admissionは別評価。

原stampの0.5秒積分窓を照合したところ、IMU/理想bicycleの最小二乗比0.92033、
VelocityReportは0.92059、EKF poseは0.91187。IMUとVelocityReportのMAEは
0.000128rad/s。したがって旋回量不足はEKFだけの誤差ではない。
79回の送信元記録でIMU/速度/舵角は/awsim_d1、姿勢は/ekf_localizer。
poseの4つの重複stamp矛盾とVelocityReportのEuler wrap角速度異常は原記録を保持して
診断から明示除外。初回診断は重複stampで失敗し、除外区間を明示するv2で評価した。
診断script/output: native `runs/time_turning_20260913/analyze_vehicle_motion_v2.py`、
`motion15_v2/summary.json`。4070窓は重複する窓であり独立サンプル数ではない。

操舵変化が0.005rad未満の0.5秒窓178個では、
`yaw_rate = v*tan(tire)/(1.087 + K*v^2)` のK=0.0455s²/mが前後両区間で一致。
ただし観測速度は約1.20〜1.306m/sであり、多速度の同定・汎用校正ではない。
全旋回窓の見かけKは約-0.0114〜0.1617で、過渡状態を定常Kだけで説明できない。

速度reportはframe_id=base_linkでも実装はVehicle.LocalVelocity=
InverseTransformDirection(Rigidbody.velocity)。後輪地点のGetPointVelocityではない。
Vehicle.AwakeはCoM transform891を使い、root z=-0.30899998546m、後輪との差
0.17500001192m。YAMLのCoM上書きは無効。平面剛体速度を後輪へ移すと
`v_rear_y = v_report_y - 0.17500001192*yaw_rate`、前後速度は同一になる。
この補正後の横速度は今回のv>0.8区間で[-0.02240,0.00578]m/s。
Unityの[GetPointVelocity公式仕様](https://docs.unity3d.com/2022.3/Documentation/ScriptReference/Rigidbody.GetPointVelocity.html)も地点速度の角速度成分を明記している。

## 共通運動契約の実装方針（turn16前）

原因所有層は、物理舵角と車体曲率を相互変換する純粋数学モジュール。
旧ideal_bicycle_v1を既定として保持し、新しい有限AWSIM専用policyで
名目K=0.045s²/mをPPの逆変換とguardの順変換で共有する。
静的wheelbase1.087mと有効応答長L+Kv²は別々に記録する。
未変更のraw30点からの目標選択・到達可能判定にも同じ有効長を使う。

監視は実測/今回/前回の物理舵角を保ち、K∈[0,0.2]s²/m、制動中の速度
0〜現在速度の全域を囲む曲率区間と実測yaw/vを合併する。
K範囲は今回の記録を含む実験上の仮定で、将来のPhysX応答の保証ではない。
低速v<0.2ではyaw/vを求めず既存の全曲率範囲を使う。
非有限・不可能なyaw、後輪横速度0.03m/s超過は明示制動。
残る横移動は0.03*(0.5+v)メートルを支持多角形へ追加し、制動中もこの上限が
成立するという条件付きで囲む。既存の車体幅、停止距離、scan marginは縮めない。
新policyには既存のasset hash一致、support_v2、固定5km/hを必須にする。

左右/零速/非有限/逆変換/制動中の速度変化/横移動の包含をunit testし、
ROSで正常信号とyaw異常制動を確認後、未使用ID turn16で1回だけ走行する。
結果から名目応答と実応答の差・guard区間・通常RViz表示を確認する。

## turn16実行前確認

source `1b0985eef16a34b86d3aee67bc3dfdb2141dd4b8`、Windows/WSL一致。
WSL関連140passed/1.90s、全体2187passed/4skipped/63warnings/80.46s。
旧turn15の制御再評価も1578件PASS（最大8.89e-16、旧policyを維持）。
Humble build0.92s、13module source/install一致。隔離ROSは実モデルPath6、
左右28、新共通モデル28、yaw異常制動16、横速度異常制動8、既存の到達不能/
stale/clock/overspeed制動PASS、車両command publisher0、child exit0。
source archive SHA256 `6c7bebef8fe8bbdb800935cb97ad35ae9944e55976b90dd8aea954b81adb3b11`、
config SHA256 `d1b0a0e24836af77924dd6f9119124cb02681e9d850a75d2690da22101a9d54c`。
専有deployment `/home/graneple/e2e_autonomous/time_vehicle_model_20260913`。
新しい走行はこの変更の影響を確認する1回のみ。車両YAML/DLL/scene/重み、速度目標、
raw30点は以前と同一。既存の監視範囲へ応答の不確かさと横移動を追加するため、
監視が以前より早く止める可能性も結果として記録する。

```bash
# native WSL, shared worktree lock
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
# .10, prepared own deployment; normal RViz raw E2E path enabled by host harness
timeout --signal=TERM --kill-after=10s 710s python3 source_1b0985e/tools/run_time_path_awsim_trial.py --deployment /home/graneple/e2e_autonomous/time_vehicle_model_20260913 --run-id codex-time-turn-16 --display :1 --config configs/control/time_path_vehicle_model_5kmh_20260913.json
```

## turn16結果：共通契約を確認、未完走

| 観測 | turn15（理想車体モデル） | turn16（共通応答モデル） |
|---|---:|---:|
| 最初のscan拒否までのsim秒 | 78.845 | 97.650 |
| 記録姿勢による走行距離 | 99.389m | 123.225m |
| 正常追従command数 | 1575 | 1953 |
| 公式section | 0→1→2 | 0→1→2 |
| 完走 | なし | なし |

単発試行の比較で、再現性や完走率の改善を示す反復評価ではない。
目標は5km/hを維持、turn16の計測速度上限は1.30521m/s（4.699km/h）、
末尾3秒の中央値は4.567km/h。実速度が常時5km/hだったとは扱わない。

turn16は1954件のPP/操舵応答/actuator/
停止用車体モデルをnative WSLで再現しPASS、最大誤差6.902e-12（許容1e-9）。
そのうち1953件は追従、1件はpost-PP scan拒否である。
各scanの全admission再生はしていない。最初の拒否scanは別に原値で再生し一致した。
scan観測は改変せず、normal RViz上のmagentaのTime model raw predictionを
80s時点とfreeze後の画像で確認。可視化用地図・既存緑経路はモデル入力ではない。

停止時の実舵角=-0.003453rad、PP要求=-0.005455rad、今回入力の物理換算
-0.008519radで、操舵限界に達した旋回ではなく、角を抜けた後のほぼ直進区間。
後輪から前方3.704m/左0.932mのLiDAR検出に対して、曲率と横移動を囲む
新監視のray余裕=-0.004499m（scan rayに沿う差であり実車体の侵入距離ではない）。
旧理想監視は同じscanで+0.120441mだが、横移動を省略するため新設定へ戻す根拠にしない。
計測後輪横速度=0.008211m/s、制動中の横移動上限=0.053577m。
単純な監視緩和を防ぐfixture `tests/fixtures/time_path/turn16_side_margin_rejection.json`
と回帰テストを追加した。実車体の接触を確認した結果ではない。

実応答の独立照合では、turn16の低操舵変化窓279個でKの最良値0.0465s²/m。
事前設定0.045に近いが、同じコース・狭い速度域の重複窓であり汎用同定ではない。
新しいfit値へ再調整したり、guardのK上限/横移動上限を縮めたりはしていない。
97回のsource graphは所定4役割と一致。poseの3重複stampは曖昧窓として除外。

hostは`FAILED / CONTROL_STOPPING_SWEEP_OCCUPIED`、wall141.68秒、outer142.59秒。
このfaultでホストがfreezeして終了しており、freeze前の実測停止確認は未成立。
cleanupエラー0、active container0、既存Compose39個とremote Git状態を保全。
49ファイル77,813,760bytesのarchive SHA256
`74a59ed07685f6730b26be1ff65bc083cdc37c4b2c449a353528161609ad605d`を
全ファイルhashとともに検証してnative WSLへ保存した。

証拠は `docs/evidence/time_vehicle_consistency_20260913/`。
`turn16_evaluation.json`、`turn16_motion_summary.json`、`turn16_stopping_margin.json`、
`turn16_normal_rviz_80s.png`、`turn16_normal_rviz_frozen.png`と各manifestを参照。
raw記録/NPZ/checkpointはGitに追加していない。

```bash
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python tools/evaluate_time_awsim_trial.py --run /home/thistle/e2e_autonomous/runs/time_turning_20260913/codex-time-turn-16 --output /home/thistle/e2e_autonomous/runs/time_turning_20260913/evaluation16
bash tools/with_wsl_training_lock.sh .venv/bin/python docs/evidence/time_vehicle_consistency_20260913/analyze_vehicle_motion_v2.py --run /home/thistle/e2e_autonomous/runs/time_turning_20260913/codex-time-turn-16 --output /home/thistle/e2e_autonomous/runs/time_turning_20260913/motion16
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python docs/evidence/time_vehicle_consistency_20260913/diagnose_stopping_margin.py --run /home/thistle/e2e_autonomous/runs/time_turning_20260913/codex-time-turn-16 --output /home/thistle/e2e_autonomous/runs/time_turning_20260913/stopping16
```

出力ディレクトリは再実行時に未使用名を指定する。既存証拠を上書きしない。
底の整合（同じ車体応答をPP/guard/replayで使用）は実装・検証済み。
残る課題は、コーナー出口から直進時の路端側の余裕、横応答の過渡/速度別の同定、
停止時のfreeze前速度確認と完走評価。新profileの将来応答上限は実験上の仮定であり、
全周囲の自由空間や実車安全の認証ではない。
