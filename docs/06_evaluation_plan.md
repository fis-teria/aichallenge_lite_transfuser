# 06. 評価計画

## 1. 原則

- オフライン指標はデバッグ用。
- 採用判断はclosed-loop指標を中心にする。
- 同じscenario set・seed・環境設定で比較する。
- 平均だけでなく失敗回数、最悪値、分散を報告する。
- safety interventionを成功扱いで隠さず、依存度として記録する。

## 2. Offline metrics

| 指標 | 用途 |
|---|---|
| waypoint ADE | 全horizonの平均誤差 |
| waypoint FDE | 最終点誤差 |
| target speed MAE | 速度方針の誤差 |
| stop precision/recall/F1 | 停止判断 |
| stop false negative | 安全上の最重要件数 |
| mode accuracy/confusion | 行動分類の妥当性 |
| direct control MAE | baseline比較 |
| predicted path collision rate | LiDAR占有との整合 |
| calibration/ECE | confidence利用時 |

## 3. Closed-loop metrics

| 指標 | 優先度 |
|---|---:|
| collision count/rate | 最高 |
| stop false negative | 最高 |
| lap completion rate | 高 |
| offtrack count/time | 高 |
| stuck/timeout count | 高 |
| safety intervention count/time | 高 |
| average/median lap time | 中 |
| average speed | 中 |
| steering rate RMS | 中 |
| minimum obstacle clearance | 高 |
| inference latency p50/p95/max | 高 |

## 4. Scenario matrix

### 通常走行

- 障害物なし
- 低速・中速
- 初期横ずれ
- カーブ入口/出口
- 複数seed

### 回避可能

- 直線中央障害物
- 左のみ通過可能
- 右のみ通過可能
- カーブ内側/外側障害物
- 障害物距離を段階変更
- 回避後復帰

### 回避不能

- 完全閉塞
- 左右margin不足
- 高速・近距離で回避不能
- 回避するとコース外
- 複数障害物

### 動的・他車

- 低速車追従
- 追い越し可能
- 追い越し中に経路が狭くなる
- 他車の予期しない減速

### Fault injection

- Camera dropout
- LiDAR dropout
- Ego state stale
- 100/200/300ms artificial delay
- NaN/inf model output
- inference deadline miss

## 5. 比較実験

最低限のablation：

1. LiDAR-only
2. Camera-only
3. Late fusion
4. Transformer fusion
5. Transformer + BEV
6. Transformer + temporal
7. stop Headあり/なし
8. Safety Supervisorあり/なし（危険scenarioではsimulation限定）
9. manual only data vs manual+MPC
10. direct control vs waypoint controller

## 6. 回帰ゲート案

本番採用前の最低条件例：

- 固定安全scenarioでcollision 0
- 完全閉塞scenarioでstop false negative 0
- sensor timeout試験で暴走0
- model output NaN試験で安全停止
- 通常走行完走率が現行baselineを下回らない
- p95 latencyがcontrol deadline内
- intervention率が前版から悪化していない

閾値は、評価scenario数と公式採点条件に合わせて確定する。

## 7. レポート単位

各experimentについて次を残す。

```yaml
experiment_id: exp_0001
git_commit: ...
model_config: ...
dataset_manifest: ...
train_split: ...
validation_split: ...
closed_loop_scenarios: ...
metrics: ...
latency: ...
artifacts:
  - checkpoint
  - config
  - logs
  - representative_video
failures:
  - scenario
  - timestamp
  - symptom
  - suspected_cause
```
