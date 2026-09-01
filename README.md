# AIC-TransFuserLite 設計・実装スターターバンドル v0.1

自動運転AIチャレンジ E2E 部門を想定した、**Camera + 2D LiDAR + Ego State** の
TransFuser風マルチモーダルE2Eモデルの設計資料と実装骨格です。

目標は次の3点です。

1. コースを安定して完走する。
2. 障害物がある場合は安全な軌道へ回避する。
3. 安全な回避軌道が成立しない場合は停止する。

本バンドルは完成品ではなく、公開手動走行データの監査から、baseline、融合モデル、
ROS 2統合、安全監視、closed-loop評価へ進むための**設計基準・コード骨格・テスト雛形**です。
ROS 2メッセージ型・トピック名・車両制御範囲は、使用する公式環境の版で再確認してください。

## 推奨アーキテクチャ

```text
Camera RGB ─ Camera Encoder ─ image tokens ┐
                                            │
2D LiDAR ─ Polar/BEV Encoder ─ lidar tokens ├─ Fusion Transformer
                                            │       ├─ future waypoints
Ego state ─ State Encoder ─ state token ────┘       ├─ target speed
                                                    ├─ stop probability
                                                    ├─ behavior mode
                                                    └─ collision risk (optional)

future waypoints + target speed
        ↓
Waypoint Controller
        ↓
Safety Supervisor
        ↓
Ackermann control command
```

## 重要な設計原則

- モデルの主出力は直接操舵ではなく、**将来waypoint・目標速度・停止確率**とする。
- 直接制御Headはbaselineまたは補助損失として残す。
- 推論入力と、教師生成・解析だけに使うprivileged情報を分離する。
- 学習モデルの外側に独立したSafety Supervisorを置く。
- frame単位のランダム分割ではなく、run/scenario単位でtrain/validation/testを分ける。
- いきなりTransformerを作らず、LiDAR-only、Camera-only、late fusionを経て性能差を確認する。
- オフラインlossではなく、closed-loopの完走・衝突・停止・介入回数を主評価にする。

## ディレクトリ

```text
.
├── AGENTS.md                         # Codex向けプロジェクト指示
├── configs/                          # 学習・モデル・安全設定
├── docs/                             # 要求、設計、学習、評価、ロードマップ
├── schemas/                          # データ形式とindex.csv仕様
├── src/aic_transfuser_lite/          # PyTorchと制御ロジックの骨格
├── tools/                            # audit、可視化、demo、smoke test
├── tests/                            # 前処理・shape・安全・制御テスト
└── ros2_ws/src/aic_e2e_runtime/      # ROS 2統合骨格
```

## 最初の確認

Python 3.10以上を想定しています。

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
pip install -e .
pytest -q
python tools/smoke_test.py --config configs/transfuser_lite_v0.yaml
```

合成データで学習パイプラインを確認する場合：

```bash
python tools/build_demo_dataset.py --output /tmp/aic_demo --samples 64
python -m aic_transfuser_lite.training.train \
  --config configs/transfuser_lite_v0.yaml \
  --train-index /tmp/aic_demo/index.csv \
  --val-index /tmp/aic_demo/index.csv \
  --output /tmp/aic_run \
  --epochs 1
```

## MCAP実データの変換と学習

`sample_train_data_20260717/rawdata` のfile-level zstd圧縮rosbag2 MCAPは、
ROS 2をインストールせずにcanonical datasetへ変換できる。
`wheelbase-m`はwaypoint教師生成に使うため、公式車両の実値を明示すること。

```bash
python tools/convert_mcap_dataset.py \
  --input-root sample_train_data_20260717/rawdata \
  --output datasets/processed/aic_real_dataset \
  --config configs/transfuser_lite_v0.yaml \
  --wheelbase-m 1.087

python tools/dataset_audit.py \
  --index datasets/processed/aic_real_dataset/index.csv \
  --output datasets/processed/aic_real_dataset/audit

python -m aic_transfuser_lite.training.train \
  --config configs/lidar_only_v0.yaml \
  --train-index datasets/processed/aic_real_dataset/train_index.csv \
  --val-index datasets/processed/aic_real_dataset/val_index.csv \
  --output runs/lidar_only_v0 \
  --epochs 1
```

短い変換smoke testでは`--input-root`を
`sample_train_data_20260717/rawdata/vehicle_x1_mpc_1`へ変更し、
`--max-samples-per-run 16`を追加できる。converterはCameraを基準に
LiDARと現在制御指令を同期し、実データの750点LaserScanを設定値の1080点へ
nearest resamplingする。splitはframe単位ではなくrun単位で生成される。

このデータには実測odometry/poseがない。したがって、ego stateは観測時刻以前の
最新Ackermann制御指令、future waypointは将来の制御指令をkinematic bicycleで
積分したteacher-only近似である。`stop_flag`も将来の指令速度によるproxyであり、
意図的停止と開始前・終了後の停止を区別できない。これらの由来とdrop件数は出力先の
`metadata.yaml`へ記録される。公式wheelbase、実測state topic、停止annotationが
得られた場合は再変換すること。

さらに、24 run中21 runはAckermann commandの`speed`が常に0で、
`acceleration`だけが変化する。加速度指令だけから実速度や軌跡の真値は復元できないため、
converterは既定でこれらを除外し、非ゼロ速度指令を持つMPC 3 runだけを変換する。
除外は`metadata.yaml`の`unusable_commanded_speed_run`に記録される。
まずはmode lossが0のLiDAR-only baselineで検証する。TransFuserを学習する場合も、
このproxy datasetでは`loss_weights.mode: 0.0`の専用configを使用すること。

Camera + LiDAR + egoのLateFusion baselineも同じindexで学習できる。
現在の`late_fusion_v0.yaml`はproxy behavior labelに合わせてmode lossを0としている。

```bash
python -m aic_transfuser_lite.training.train \
  --config configs/late_fusion_v0.yaml \
  --train-index datasets/processed/aic_real_dataset/train_index.csv \
  --val-index datasets/processed/aic_real_dataset/val_index.csv \
  --output runs/late_fusion_v0 \
  --epochs 100
```

## 実装順

1. 公開データのライセンスとtopicを確認する。
2. `dataset_audit.py`で欠損・同期・分布を監査する。
3. 10分程度のmini datasetを作る。
4. LiDAR-only baselineを学習・closed-loop評価する。
5. Safety Supervisorを単独で動かす。
6. Camera-onlyとlate fusionを比較する。
7. AIC-TransFuserLite v0を実装・学習する。
8. BEV LiDAR、temporal fusion、複数候補軌道へ拡張する。
9. 固定scenario setで回帰試験する。

詳細は `docs/` を参照してください。

## 注意

- 公開されているデータでも、学習・競技利用・派生重み・再配布の許諾があるとは限りません。
- 手動走行データだけでは、回避不能停止や復帰ケースが不足する可能性があります。
- モデル出力が妥当に見えても、センサ遅延や制御周期の不一致でclosed-loop挙動は崩れます。
- ROS 2統合骨格は、公式環境の実際のメッセージ定義に合わせて調整してください。
