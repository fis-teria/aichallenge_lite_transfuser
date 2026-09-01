# Validation Report

検証環境で以下を実行しました。

## Static/Syntax

```bash
python -m compileall -q src tools tests ros2_ws/src/aic_e2e_runtime/aic_e2e_runtime
```

結果: 成功。

## Unit tests

```bash
PYTHONPATH=src pytest -q
```

結果: **10 passed**。

対象:

- LiDAR invalid値処理・正規化
- 簡易BEV生成
- TransFuser-lite forward shape
- LiDAR-only forward shape
- 停止距離
- 前方障害物によるbrake override
- sensor timeout
- clear path時のnominal command
- waypoint controllerの左右操舵・速度制御

## Model smoke test

```bash
PYTHONPATH=src python tools/smoke_test.py --config configs/transfuser_lite_v0.yaml
```

確認した出力:

- waypoints: `[2, 6, 2]`
- target_speed: `[2, 1]`
- stop_logit: `[2, 1]`
- mode_logits: `[2, 6]`
- direct_control: `[2, 2]`

## Synthetic one-epoch training

合成16サンプルを生成し、LiDAR-only modelを1 epoch学習しました。
checkpointとhistoryが正常に保存されることを確認しました。

## 未検証

- 公式ROS 2 / Autoware環境でのbuildとtopic接続
- 公開手動走行datasetとの互換性
- CUDA/GPU mixed precision
- AWSIM closed-loop走行
- 公式車両の制御限界・wheelbase・actuation delay
- 安全パラメータの妥当性

ROS 2配下は意図的に接続骨格としており、公式環境のmessage type、field、QoSを確認してから実装する必要があります。

## MCAP canonical converter

`tools/convert_mcap_dataset.py`を追加し、同梱24 runのfile-level zstd圧縮
rosbag2 MCAPを実際に走査した。

検証用にwheelbaseを仮の1.0 mとして全runを変換した結果：

- 入力run: 24
- 採用run: 3 (`vehicle_x1_mpc_1`〜`3`)
- canonical sample: 1,575
- split: train 601 / validation 471 / test 503（各1 run）
- 欠損画像: 0
- 欠損LiDAR: 0
- 保存LiDAR shape: `[1080]`
- 最大Camera-LiDAR同期差: 24.69 ms
- 最大Camera-ego proxy同期差: 58.71 ms

残り21 runはAckermann commandのspeedが常に0で、実測odometry/poseも存在しないため、
誤ったzero-waypoint教師を作らずrunごと除外した。これはconverterの
`metadata.yaml`に記録される。

実Camera/LiDARから各16 sampleを使ったLiDAR-only 1 epoch学習も実施し、
checkpointとhistoryの保存を確認した。

```text
train_loss: 6.790070533752441
val_loss:   5.899618625640869
```

追加後のunit test結果は`18 passed`。公式wheelbaseは未確定であり、本学習用datasetは
実値を確認して再変換する必要がある。また、採用3 runには停止例がないため、
stop headや障害物回避の教師としては不足している。

## LateFusion baseline

Camera、LiDAR、egoを個別にencode・global poolingし、feature vectorを結合する
LateFusion baselineを実装した。出力契約はTransFuserと同じである。

- waypoints: `[B, 6, 2]`
- target speed: `[B, 1]`
- stop logit: `[B, 1]`
- mode logits: `[B, 6]`
- direct control: `[B, 2]`
- parameters: 11,376,150

実Camera/LiDARのtrain/validation各16 sampleで1 epoch学習し、checkpoint保存を確認した。

```text
train_loss: 6.784394264221191
val_loss:   5.718108654022217
checkpoint: 45,594,036 bytes
```

LateFusion追加後のunit test結果は`20 passed`。
