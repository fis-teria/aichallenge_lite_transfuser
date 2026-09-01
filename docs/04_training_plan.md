# 04. 学習計画

## 1. 段階的学習

| Stage | モデル | 目的 | Exit条件 |
|---:|---|---|---|
| 0 | データ監査のみ | 欠損・同期・偏りを把握 | audit reportレビュー完了 |
| 1 | Direct control baseline | pipeline疎通 | 1 epoch学習、有限loss |
| 2 | LiDAR-only | 停止・障害物反応 | offlineと短距離closed-loop確認 |
| 3 | Camera-only | コース追従 | 通常走行baseline成立 |
| 4 | Late fusion | センサ併用の純効果 | 単眼/単LiDARより改善 |
| 5 | Transformer fusion | TransFuser風本命 | closed-loop回帰基準を満たす |
| 6 | BEV/temporal | 回避・安定性強化 | ablationで有意改善 |
| 7 | MPC/DAgger fine-tune | 分布ずれ対策 | 失敗scenario改善 |

## 2. Loss

初期式：

```text
L =
  1.00 * L_waypoint
+ 0.30 * L_speed
+ 1.00 * L_stop
+ 0.20 * L_mode
+ 0.20 * L_direct_control
+ 0.05 * L_smoothness
```

### Waypoint loss

Smooth L1を使い、遠いhorizonへ重みを付けるか比較する。

```text
L_waypoint = Σ_i w_i * SmoothL1(pred_i, target_i)
```

遠い点ほど不確実なため、単純増加ではなく均一・近距離重視・遠距離重視をablationする。

### Stop loss

停止データは少ない可能性が高い。

- `pos_weight = negative_count / positive_count`を上限付きで使う
- stop sampleをoversamplingする
- Focal Lossと比較する
- precisionよりrecallを安全側の主指標にする

### Smoothness

waypointの二階差分を罰する。

```text
L_smooth = mean(|p[i+1] - 2p[i] + p[i-1]|)
```

過度に強くすると必要な急回避を抑えるため、低いweightから始める。

## 3. Optimizer初期値

```yaml
optimizer: AdamW
learning_rate: 3e-4
weight_decay: 1e-2
batch_size: 64
mixed_precision: true
epochs: 30
scheduler: cosine or plateau
```

Camera backboneをpretrainedにする場合：

- backbone lrを0.1倍
- 最初の数epochはbackbone freeze
- その後unfreeze

を比較する。

## 4. Sampling

データの大半が直進になる場合、uniform frame samplingは使わない。

bucket例：

- straight
- left curve
- right curve
- low speed
- stop
- avoid left
- avoid right
- recovery

DataLoaderでweighted samplingまたはscenario batchを使う。

## 5. Human control label shift

手動操作には反応遅れがある。

候補：

- 0ms
- +100ms
- +200ms
- +300ms

オフラインcontrol errorだけで決めず、closed-loopの位相遅れ・蛇行・カーブ進入を比較する。
future waypointラベルがある場合、直接制御より優先する。

## 6. Augmentation

### Camera

安全な範囲：

- brightness/contrast
- mild gamma
- small blur/noise
- limited crop/resize jitter

注意：左右反転は、操舵・waypoint・LiDAR・modeを同時に反転できる場合だけ使う。

### LiDAR

- small Gaussian range noise
- random beam dropout
- limited sector dropout
- range scale jitter

障害物を消し過ぎたり生成したりしない。Safetyに関わるaugmentationはscenarioレビューする。

## 7. Fine-tuning

公開手動データでpretrainした後、以下を追加する。

- MPCオーバーテイク成功データ
- 回避不能停止データ
- 回避後復帰データ
- baselineが失敗した初期状態からのexpertデータ
- sensor delay/dropoutを含むhard cases

単純に混ぜるだけでなく、dataset sourceをmetadataに持ち、source別性能を報告する。

## 8. 実験管理

各runに保存するもの：

- git commit
- config snapshot
- dataset manifest hash
- train/val split ID
- seed
- loss curve
- best checkpoint
- offline metrics
- closed-loop metrics
- 代表動画
- failure notes
