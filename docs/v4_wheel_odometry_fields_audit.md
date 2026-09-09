# Wheel Odometry相当入力のfield・生成元調査

2026-09-09。読み取り専用調査。ROS/AWSIM起動、実センサ収集、制御、runtime変更なし。

## 結論

現AWSIMの `/vehicle/status/velocity_status` には前後速度・横速度・yaw rateがある。
送信コードはGNSS/IMUセンサ出力を参照せず、シミュレータ車体状態から生成する。
左右輪エンコーダーのpulse/RPMを配信する形式ではない。
したがって「左右輪速度からyaw rateを新しく計算しなければならない」とは限らない。
このVelocityReportを入力にする独立した局所pose積分が技術的な第一候補になる。
物理車両のエンコーダーだけによるodometryと同等の誤差特性を保証しない。

## 現物との対応

対象host `graneple@192.168.3.10`、repository
`/home/graneple/git/autononous_ai/aichallenge-racingkart`。
既存dirtyを保全。対象ファイル:
`aichallenge/simulator/AWSIM/AWSIM_Data/Managed/Assembly-CSharp.dll`。
SHA256はWindows保存済み `tmp/Assembly-CSharp-awsim-d1.dll` と一致:
`859e5560dbffd7d0833cd1fe5eb1f36b87fa8d22f6e45eb0e1203d04a1ea6d13`。
保存済み逆コンパイル結果に加え、同DLLから下記2型を今回再表示して生成式を確認した。

```powershell
tmp/ilspycmd/ilspycmd.exe -t AWSIM.VehicleReportRos2Publisher tmp/Assembly-CSharp-awsim-d1.dll
tmp/ilspycmd/ilspycmd.exe -t AWSIM.Vehicle tmp/Assembly-CSharp-awsim-d1.dll
```

## fieldの生成

`AWSIM.VehicleReportRos2Publisher.FixedUpdate`:

| field | 生成元 |
|---|---|
| longitudinal_velocity | ROS座標へ変換したVehicle.LocalVelocity.x + 設定bias |
| lateral_velocity | 同LocalVelocityのROS y |
| heading_rate | -Vehicle.AngularVelocityをROS座標へ変換したz |
| steering_tire_angle | Vehicle.SteerAngleを符号反転・deg→rad変換 + 設定Gaussian noise |
| gear report | Vehicle.AutomaticShiftInputの変換 |
| stamp | SimulatorROS2Nodeの現在ROS時刻 |

Vehicle.LocalVelocityはRigidbody.velocityを車体ローカルへ変換したもの。
Vehicle.AngularVelocityは車体rotationのEuler角と前回値の差をdeltaTimeで割りradへ変換。
この生成経路にImuSensor/imu topic/GNSS/EKFはない。
ただしEuler角の折返し・計算順序・車体基準点に起因する特性は別途検証が必要。

PublishHzのコード既定は30。FixedUpdateのtimer到達時に0へ戻す実装。
ApplyTuningで変更可能なため実scene値の確定ではない。
前回ログの補間bracket 35msは整合するが、全topic受信レート測定は今回NOT_RUN。
steering noise/biasも設定可能なので、コード既定値を実適用値とは断定しない。

## 公式仕様との照合

2026年公式インターフェースはVelocityReportの3field、base_link frame、
SteeringReportとGearReportを記載する。
https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/specifications/interface.html

E2E部門はWheel OdometryとSteer Angleを使用可能センサとして記載する。
https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/competition/ai-class.html

公式に供給されるVelocityReportを積分する候補と、内部pose/TFを直接取得する案は区別する。
シミュレータ内部の物理量からセンサmessageを生成すること自体を、独自に禁止と判定しない。
一方、この調査は審査者による全field利用の承認や実車適合認証ではない。

## 車速＋操舵方式を選ぶ場合

現repositoryの `racing_kart_description/config/vehicle_info.param.yaml` は
wheel_base=1.087m、wheel_tread=1.12m、wheel_radius=0.24mを宣言する。
これは当該車両の設定ファイル値であり、今回simulator車体の実寸/基準点を校正した値ではない。
車速だけを使う場合は旋回が決まらないので、実操舵角と根拠付きwheelbaseが必要。
今回はVelocityReportにyaw rateがあるため、この方式を無条件に追加する理由はない。

## 次の実装方針

- GNSS/IMU/EKFから切り離したVelocityReport→局所pose積分coreを用意する。
- source stampのdtを使い、epoch/reset、重複、stale、異常yaw rateを検査する。
- 初期原点は局所座標。map位置へ結び付けず、長時間の絶対位置精度を主張しない。
- TFを提供する場合は既存map→base_linkとchild競合しない専用treeにする。
- V4元frame経路の生成は外部poseを必須にしない。過去経路の変換とは分離する。
- V4内部egoのheading_rateは既にこのVelocityReportから取得している。
  入力の意味を変える場合は学習時との整合も別途確認する。

追加左右輪topicは今回確認した公式interface/車体report publisherには見つからなかった。
全実行環境に別publisherが存在しないというライブgraph証明ではない。
