# V4相対TFの生成元確認

2026-09-09。既存TFを許可入力由来の場合だけ再利用する方針に基づく静的確認。
AWSIM/ROSを起動せず、SSH先のコードを読み取り専用で確認した。

## 確認対象

Remote: `graneple@192.168.3.10`
Repository: `/home/graneple/git/autononous_ai/aichallenge-racingkart`
HEAD: `4af395eee10f928c7fc7225760adfa04c4c07ff4`。
既知のdirty checkoutの現行ファイルを読んだ。HEADだけを実ファイル版とは扱わない。
過去run18のV4設定は `/localization/kinematic_state` を購読していた。

## 確認した経路

`aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/reference.launch.xml`

- 107行付近: `/vehicle/status/velocity_status` を vehicle_velocity_converterへ入力。
  出力は `/sensing/vehicle_velocity_converter/twist_with_covariance`。
- 138行付近: gyro_odometerは上記twist **と** `/sensing/imu/imu_data` を入力。
- 145行付近: imu_gnss_poserを起動。heading_reference.csvも設定される。
- 150行付近: ekf_localizerへimu_gnss_poserのposeとgyro_odometerのtwistを入力。
  `output_odom_name=kinematic_state`、`tf_rate=30.0`。

`aichallenge/workspace/src/aichallenge_submit/gyro_odometer/src/gyro_odometer_core.cpp`
84–85行付近: twistのlinear.xは車両速度平均、angularはgyro平均。
IMUが届かない場合の待機・timeout処理もある。

`aichallenge_submit_launch/config/vehicle_velocity_converter.param.yaml`
は `frame_id=base_link` を設定するが、twistのframe指定は移動TFの生成を意味しない。
robot_state_publisherの車体・センサ座標系も自車移動量の根拠にはならない。

## 判断と未確認

このEKF経由のpose/TFを差分にしてもGNSS/IMU依存は消えない。
確認したsubmit/systemソース範囲では、独立したwheel-only odom→base_link生成器を
特定できなかった。全実行環境に存在しないという証明ではない。
今回ライブTF tree/authorityは未取得で、TFの正確な親子名や実際の送信元は未確認。
`odom→base_link` が存在すると仮定して接続はしない。

また `/vehicle/status/velocity_status` のheading_rateが許可入力だけから生成されるかは
今回未確定。topic名がWheel Odometry相当でも全fieldの出典を保証しない。

## 次の実装に必要な分離

1. V4元frameでの推論と、グローバルposeを使う診断・追従接続を分ける。
2. Wheel Odometry・実操舵角・Gearの生成元と意味、車体基準点、必要な車両値を確認する。
3. 独立した局所odometryが必要なら、上記入力だけから作る方式を設計する。
   現在のmap→base_linkと同じchildを二重publishしない。既存TF treeと隔離する。
4. 相対変換は同じepoch内の観測時刻と現在時刻の両poseから求める。
   pose欠損時に恒等変換を捏造せず、元frameの経路出力と変換不可を区別する。

今回runtime変更・publisher追加・同期・実推論・走行・pushは行っていない。
