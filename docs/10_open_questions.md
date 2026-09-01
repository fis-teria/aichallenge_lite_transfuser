# 10. 未確定事項・次に確認する情報

## Dataset

- 公開データのURL、ライセンス、配布者の利用条件
- rosbag2/mcap/独自形式のどれか
- topic名、message type、収録周期
- CameraとLiDARのstampが同一clockか
- control commandと実操舵statusの両方があるか
- future poseまたはMPC trajectoryがあるか
- 障害物、追い越し、停止、衝突、offtrack annotationの有無
- データ量、run数、driver数、scenario数

## Vehicle / Simulation

- wheelbase
- steering limit/rate
- acceleration/braking limit
- actuation delay
- vehicle footprint
- LiDAR mounting position、angle_min、angle_increment、forward direction
- Camera intrinsics/extrinsics
- 公式評価の障害物配置と採点条件

## Training Environment

- GPU型番とVRAM
- 学習可能時間
- Docker/ROS 2/PyTorch/CUDA版
- ONNX/TensorRT利用可否

## Design Decisions to Freeze

実装開始前に最低限決める。

1. Canonical index列と座標系
2. future waypointのhorizonと点数
3. label shift候補
4. ego stateの利用可能次元
5. stop label定義
6. model output contract
7. Safety Supervisorの責任境界
8. closed-loop regression scenario set
