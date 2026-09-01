# 03. モデル設計

## 1. モデル名

`AICTransFuserLiteV0`

Camera、2D LiDAR、ego stateをtoken化し、Transformer Encoderで融合する。
本家TransFuserの考え方を参考にしつつ、2D LaserScan向けに軽量化する。

## 2. v0 Tensor仕様

| 入力 | shape | 備考 |
|---|---|---|
| image | `[B, 3, 180, 320]` | RGB、0〜1正規化後に標準化 |
| lidar | `[B, 1080]` | mから0〜1へ正規化 |
| ego | `[B, 5]` | velocity等をscale |

| 中間 | shape |
|---|---|
| image tokens | `[B, 64, 128]` |
| lidar tokens | `[B, 64, 128]` |
| ego token | `[B, 1, 128]` |
| fused tokens | `[B, 129, 128]` |

| 出力 | shape |
|---|---|
| waypoints | `[B, 6, 2]` |
| target_speed | `[B, 1]` |
| stop_logit | `[B, 1]` |
| mode_logits | `[B, 6]` |
| direct_control | `[B, 2]`、任意 |

## 3. Camera Encoder

初期実装：ResNet18 backbone。

```text
RGB image
  ↓ ResNet18 convolutional backbone
feature map
  ↓ 1x1 projection to dim=128
  ↓ adaptive pooling to 8x8
64 image tokens
```

- 初回smoke testはpretrained=falseで外部downloadを避ける。
- 実学習ではImageNet pretrainedの有無をablationする。
- 上空・空・不要領域が大きい場合は固定cropを検討する。
- cropは可視化し、障害物やコース境界を欠落させない。

## 4. LiDAR Encoder

### v0: Polar 1D

```text
LaserScan [1080]
  ↓ Conv1d blocks
  ↓ adaptive pooling to 64 bins
64 lidar tokens
```

長所：軽量、実装容易、LaserScanの角度順序を保持。
短所：camera tokenとの明示的な幾何対応が弱い。

### v1: BEV occupancy

LaserScanをxyへ変換し、vehicle-centric BEVへrasterizeする。

推奨channel：

- occupied
- free-space
- unknownまたはvalidity

BEV範囲・解像度はLiDAR範囲とコース幅から決める。
例：前方30m、左右10m、0.2m/cell。

### v2: Dual branch

Polar 1DとBEVを併用し、近距離角度分解能とfree-space幾何を両立する。

## 5. Ego Encoder

候補5次元：

- longitudinal velocity
- lateral velocity
- heading/yaw rate
- steering tire angle
- gear numeric value

MLPで128次元の1 tokenへ変換する。
各値は物理範囲でscaleし、NaNを許容しない。

## 6. Fusion

v0は全tokenをconcatし、modality embeddingとlearned positional embeddingを加える。

```text
[image tokens | lidar tokens | ego token]
       + modality embeddings
       + positional embeddings
              ↓
TransformerEncoder(depth=3, heads=4, dim=128)
              ↓
ego token / pooled token
```

v1以降の候補：

- image→LiDAR / LiDAR→imageのcross attention
- multi-scale image token
- camera rayまたはangle positional encoding
- LiDAR angle positional encoding
- query token方式のwaypoint decoder

## 7. Head

### Waypoint Head

6点×2次元を予測する。
初期実装はMLP。拡張時はquery tokensまたはGRU decoderを比較する。

### Target Speed Head

非負制約を持たせる。実装ではraw出力にsoftplusを適用する。
上限はcontroller/safety側でもclipする。

### Stop Head

logitを出し、学習時にweighted BCEまたはFocal Lossを使う。
推論時はsigmoidしてthreshold判定する。
stop Head単独を停止保証としない。

### Mode Head

補助タスク。annotation品質が低い場合はloss weightを0にする。

### Risk/Confidence Head（v1）

- predicted trajectory collision risk
- aleatoric uncertainty
- sensor quality confidence

Safety Supervisorの減速判断に使えるが、独立LiDAR停止より優先しない。

## 8. 複数候補軌道（推奨拡張）

回避と停止を明確化するため、v2でK本の候補軌道を予測する案を推奨する。

```text
K=3: left / center / right candidate
各candidate: waypoints, target_speed, feasibility_score, collision_risk
```

Safety Supervisorまたはtrajectory selectorが、LiDAR occupancyと車両footprintで候補を検証する。
全候補が危険なら停止する。

これは「モデルが1本だけ危険な軌道を出した時に、回避可能性を判断できない」問題を減らす。
ただしv0の学習・評価が安定してから導入する。

## 9. Temporal拡張

過去4 frame程度を使う候補：

- 各時刻のfused tokenをGRUへ入れる
- temporal Transformer
- feature-level exponential smoothing

目的：

- センサ瞬断への耐性
- 他車や障害物の相対運動推定
- 操舵ジッタ低減

計算量と同期複雑度が増えるため、single-frame baselineとの差を測ってから採用する。
