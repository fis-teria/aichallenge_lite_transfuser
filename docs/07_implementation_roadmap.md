# 07. 実装ロードマップ

## M0: 公式環境とデータの棚卸し

成果物：

- topic/type/rate一覧
- 公開datasetのライセンス記録
- `ros2 bag info`または同等情報
- センサsampleの可視化

完了条件：

- 推論入力とteacher-only情報が区別されている。
- 走行run数、時間、scenario、失敗区間が把握できている。

## M1: Canonical dataset

成果物：

- rosbag extractor
- index.csv
- metadata.yaml
- dataset audit report
- sample video

完了条件：

- 同じraw dataから同じindexを再生成できる。
- 欠損、sync delta、分布が数値化されている。

## M2: LiDAR-only baseline

成果物：

- preprocess
- Dataset/DataLoader
- model
- train/eval
- checkpoint

完了条件：

- syntheticとreal mini datasetで学習が通る。
- output shape、NaN、save/loadがテストされている。

## M3: Safety Supervisor

成果物：

- stopping distance logic
- timeout logic
- command clamp/rate limit
- unit tests
- ROS node骨格

完了条件：

- front obstacle、timeout、NaNの各試験で安全側に倒れる。

## M4: Camera-only / Late fusion

成果物：

- Camera encoder
- late fusion baseline
- ablation report

完了条件：

- Cameraまたはfusion追加の効果がclosed-loopで比較できる。

## M5: AIC-TransFuserLite v0

成果物：

- Transformer fusion
- waypoint/speed/stop/mode Heads
- training config
- offline report

完了条件：

- 全Headが学習できる。
- late fusionとの差を説明できる。

## M6: ROS 2 closed-loop

成果物：

- inference node
- controller node
- safety node
- launch
- latency logger

完了条件：

- 10Hz以上で連続推論する。
- sensor staleとdeadline missを検知する。

## M7: 障害物fine-tuning

成果物：

- MPC/追加手動データ
- stop/avoid/recovery dataset
- fine-tuned checkpoint

完了条件：

- 固定回避・閉塞scenarioでbaselineを改善する。

## M8: 勝負版

候補：

- BEV occupancy branch
- temporal fusion
- K candidate trajectories
- confidence calibration
- ONNX/TensorRT最適化
- scenario hard mining

導入条件：v0のfailure modeが対象機能で改善可能と確認できること。
