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
