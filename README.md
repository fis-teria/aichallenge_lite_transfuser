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

## Codex編集環境とWSL学習環境

Windowsの`E:\workspace\e2e_lite_transfuser`をCodexの編集元・Git正本、WSLの`/home/thistle/e2e_autonomous/e2e_lite_transfuser`を本学習・Linux/CUDA/ROS検証環境として運用する。

コミット単位の安全な同期手順、除外する学習資産、検証コマンドは[`docs/windows_codex_wsl_training_workflow.md`](docs/windows_codex_wsl_training_workflow.md)を参照する。

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

## V3 canonical data CLI

V3のデータ基盤は、versioned topic profile、metadata-only bag inventory、
clock epoch、同期、atomic canonical storage、audit、leakage-safe split、V1互換viewを
`aic-e2e`から操作する。V1のconfig、Dataset v2、model、checkpoint、runtimeは変更しない。

```bash
python -m aic_transfuser_lite.cli bag scan \
  --input-root /path/to/bags --output /tmp/bag_inventory.json

python -m aic_transfuser_lite.cli bag validate \
  --input-root /path/to/bags \
  --config configs/data/topic_profile_v3.yaml \
  --output /tmp/bag_validation.json

python -m aic_transfuser_lite.cli dataset build \
  --input-root /path/to/bags \
  --config configs/data/dataset_v3.yaml \
  --topic-profile configs/data/topic_profile_v3.yaml \
  --dataset-id dataset_v3_001 \
  --output /path/to/dataset_v3 \
  --dry-run

python -m aic_transfuser_lite.cli dataset audit \
  --dataset-root /path/to/dataset_v3 --output /tmp/dataset_v3_audit

python -m aic_transfuser_lite.cli dataset split \
  --runs-json /path/to/split_runs.json \
  --dataset-manifest-sha256 <sha256> \
  --config configs/data/split_v3.yaml \
  --output /path/to/split_manifest.json

python -m aic_transfuser_lite.cli view build \
  --dataset-root /path/to/dataset_v3 \
  --config configs/data/view_v1_compat.yaml \
  --output /path/to/view_manifest.json
```

`dataset build`の出力先は新規pathに限定し、`--resume`は完成済みで同じ
`dataset_id`の出力だけを再利用する。`--dry-run`はDatasetを書かない。
V3 converterはpose、velocity、LiDAR、controlのtimestamp indexをrunごとに一度だけ作り、
各sampleと未来30点の同期では二分探索を再利用する。timestampが重複または逆順のstreamは
index作成時に明示的に拒否する。
raw bag、Dataset、checkpoint、run artifactはGitへ追加しないこと。

## V3自動校正データ収集

AWSIMのsteering、drive、brake励起を、dry-run優先かつplan SHA-256でarm
されるcollectorで1 runずつ記録できる。競合するnominal/final command
publisherが存在すれば拒否し、実行時の指令は独立したSafety Supervisorを
通す。

```bash
PYTHONPATH=src python3 tools/collect_calibration_v3.py \
  --plan configs/calibration/excitation_steering_low_speed_v1.yaml \
  --topic-profile configs/data/topic_profile_v3.yaml \
  --output-root /absolute/native/linux/path/calibration_bags/v3 \
  --run-id steering_r01 \
  --scenario-id awsim_calibration_pad
```

上記はdry-runであり、ROS processや出力directoryを作らない。Granepleで
排他的な制御経路と停止状態を確認した後だけ`--execute`を付ける。必要な
environment、run反復、変換、fit、未検証境界は
[`docs/v3_calibration_capture.md`](docs/v3_calibration_capture.md)を参照。

## V3 full-control学習（V3-018到達点）

V3のfull-controlモデルは、4-frame Camera/LiDAR履歴、10-step ego/command履歴から、
15点の軌道・速度profileと現在の`[steering rad, speed m/s, acceleration m/s^2]`を
同時に学習する。教師はnominal commandを優先し、欠ける場合だけ
`final_fallback` provenance付きでfinal commandを使用する。
Dataset V3のMCAP readerも同じ契約を使用し、V1凍結readerとは分離される。
そのため`/nominal_control_cmd`が無いrunでも、topic profileの必須sensorが揃い、
`/control/command/control_cmd`の型が一致すればfull-control label能力を保持する。
学習batchはDataset全体をRAM/GPUへ一括展開せず、選択batchのassetだけを逐次読み込み、
optimizer stepの直前にdeviceへ転送する。`--dry-run`も同じlazy batchを全件走査して
class weightを検証する。
未来trajectoryのvalid stepが0件のanchorは、空mask lossを作らないよう学習対象から除外する。

Full Controlは状況判断を`aic_behavior_v1`の補助Headとしても学習する。

```text
0 FORWARD_NORMAL   1 FORWARD_FOLLOW   2 FORWARD_AVOID
3 FORWARD_RETURN   4 RECOVERY

side: 0 NONE   1 LEFT   2 RIGHT
```

教師は`autoware.log`の`[speed.diag]`をMCAPの`/awsim/state`でwall timeから
simulation timeへ変換したbehavior viewから読む。`Ready -> Start -> Finish`の外、
診断間隔が500 msを超える箇所、ラベル遷移を挟む区間はmaskされる。旧形式の
不完全な診断行を`none`として補完しない。Dataset V3を作成した後、学習前に
次を実行する。複数runは`--run-source`を繰り返す。

```bash
.venv/bin/aic-e2e behavior build \
  --dataset-root /home/thistle/e2e_autonomous/datasets/aic_dataset_v3 \
  --run-source RUN_ID /path/to/autoware.log /path/to/rosbag2_autoware \
  --output /home/thistle/e2e_autonomous/datasets/behavior_view_v1
```

behavior viewはDataset manifest、log、bag storage、ラベルCSVのSHA-256と、
runごとのwall-to-sim offsetを記録する。`STOP`は意図的停止と開始前・スタック時の
停止を現ログから区別できないため、このontologyには含めない。

学習はLinux/CUDA環境でworkspace lockを保持して実行する。`--dry-run`はDataset、
split、shape、full-control label能力を検証するが、run directoryやcheckpointを作らない。

```bash
tools/with_wsl_training_lock.sh .venv/bin/aic-e2e train \
  --config configs/models/full_control_lite_v3.yaml \
  --dataset-root /home/thistle/e2e_autonomous/datasets/aic_dataset_v3 \
  --split-manifest /home/thistle/e2e_autonomous/datasets/aic_dataset_v3/split_manifest.json \
  --view-config configs/data/view_temporal_v3.yaml \
  --behavior-view /home/thistle/e2e_autonomous/datasets/behavior_view_v1 \
  --output /home/thistle/e2e_autonomous/runs/full_control_lite_v3 \
  --dry-run

tools/with_wsl_training_lock.sh .venv/bin/aic-e2e train \
  --config configs/models/full_control_lite_v3.yaml \
  --dataset-root /home/thistle/e2e_autonomous/datasets/aic_dataset_v3 \
  --split-manifest /home/thistle/e2e_autonomous/datasets/aic_dataset_v3/split_manifest.json \
  --view-config configs/data/view_temporal_v3.yaml \
  --behavior-view /home/thistle/e2e_autonomous/datasets/behavior_view_v1 \
  --output /home/thistle/e2e_autonomous/runs/full_control_lite_v3
```

中断後は同じcommandへ`--resume`を追加する。Dataset manifest、split、view、model
contractのhashがcheckpointと異なる場合はresumeを拒否する。actual steeringをmodel入力に
含めるconfigでは、その値が欠損したanchorを学習対象にしない。headを無効化したまま
current-control lossを非zeroにした場合も開始前に失敗する。
behavior class weightはtraining splitの有効ラベル数から算出し、通常学習で
いずれかのclassまたはsideが0件なら開始前に失敗する。behavior viewのhashは
checkpoint identityへ含まれるため、ラベルを変更したcheckpointへの`--resume`も拒否する。
学習完了時の`runtime_artifact.json`にはbehavior capability、モデル構築引数、
checkpoint SHA-256が記録され、そのままV3 runtimeのartifact manifestとして使える。
`lidar_points`はDataset V3のnative geometryと一致必須で、既定値は現行AWSIM記録の
750 beamsである。別geometryを黙ってresampleせず、configを明示的に変更する。

ROS 2 trajectory-only profileはcontrol publisherを生成しない。AWSIM／ROS 2の実行試験は
公式環境または指定された`graneple@192.168.3.10`で実行した結果だけを成功として扱う。
behavior capabilityを持つV3 artifactでは、制御authorityを変えずに
`behavior_mode`、`behavior_label`（JSON）、`behavior_confidence`、`behavior_side`を
診断出力する。confidence閾値未満は`UNKNOWN`（mode/sideは`-1`）になる。
confidenceは制御判断には使わず、必要に応じて検証データでtemperatureを校正する。
通常走行・追従・復旧ではsideを必ず`NONE`にし、回避・復帰で左右confidenceが
閾値未満または`NONE`の場合はsideだけを`UNKNOWN`にする。
