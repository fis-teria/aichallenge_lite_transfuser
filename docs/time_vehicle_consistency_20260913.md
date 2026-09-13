# TimePath 車体応答・制御・監視の整合

## 現在の目的と前提

ユーザーの「底の整合性をとりましょうか」に基づき、Pure Pursuitが要求する曲率、
物理舵角、AWSIM車体の旋回、停止監視の運動モデルを照合して共通化する。
正常目標5km/h、TimePath B0 epoch10の未変更30点、通常RViz、実行先
`graneple@192.168.3.10`を維持。Windows正本で編集/commit、native WSLのlock内で
学習系テスト/評価を行う。AWSIM物理設定や重みを変更せず、以前のrunを保全する。
先行作業は `time_turning_guard_20260913.md`、最新実走はturn14。

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
