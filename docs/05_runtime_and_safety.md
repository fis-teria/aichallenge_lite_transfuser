# 05. Runtime・制御・安全設計

## 1. 実行パイプライン

```text
latest synchronized observation
  ↓
model inference
  ↓
waypoints + target speed + stop probability
  ↓
waypoint controller
  ↓ nominal steering/acceleration
  ↓
Safety Supervisor
  ↓ safe command
```

## 2. Waypoint Controller

初期実装はPure Pursuit相当と速度P制御を使う。

```text
curvature = 2 * y_target / lookahead_distance^2
steering = atan(wheelbase * curvature)
acceleration = kp * (target_speed - current_speed)
```

実車両モデル、操舵定義、最大角、制御周期に合わせて調整する。

必須制限：

- steering clamp
- acceleration clamp
- steering rate limit
- acceleration jerk/rate limit
- target speed clamp

## 3. Safety Supervisorの優先順位

優先順位は上ほど強い。

1. Emergency/manual stop
2. Sensor timeoutまたはtimestamp異常
3. NaN/inf/shape異常
4. 前方停止距離違反
5. 予測軌道とLiDAR占有の衝突
6. model stop probability
7. low confidenceによる減速
8. nominal command

## 4. 停止距離

初期式：

```text
d_stop = v * latency + v^2 / (2 * |a_brake|) + margin
```

- `latency`: センサ、推論、通信、actuation遅延の上限
- `a_brake`: 実環境で期待できる保守的な制動減速度
- `margin`: センサ誤差、車体前端、余裕

パラメータを勘で固定せず、速度別停止試験から同定する。

## 5. 前方距離

v0ではLaserScanの前方角度範囲内の最小距離を使う。

```text
front_sector = ±15 deg
front_distance = min(valid ranges in sector)
```

欠点：車幅、カーブ、斜め障害物を正確に扱えない。

v1ではvehicle footprintをwaypointに沿ってsweepし、BEV occupancyとの衝突を確認する。

## 6. 回避不能判定

v0：

- model stop probabilityが高い
- nominal waypoint corridorが占有される
- front distance < stopping distance

v2推奨：

- K本のcandidate trajectoryを生成
- 各candidateをfootprint + marginでcollision check
- 少なくとも1本安全なら最良candidateを選択
- 全candidateが危険なら停止

## 7. Timeout

初期候補：

| Stream | timeout |
|---|---:|
| Camera | 300ms |
| LiDAR | 200ms |
| Ego state | 200ms |
| Model output | 200ms |

timeout時に前回commandを無期限保持しない。
推奨動作は、短時間なら減速、継続なら停止。

## 8. Safety state machine

```text
NORMAL
  ├─ low confidence → DEGRADED
  ├─ hazard/timeout → BRAKING
  └─ emergency → STOPPED

DEGRADED
  ├─ recovered for N cycles → NORMAL
  └─ hazard persists → BRAKING

BRAKING
  ├─ speed below threshold → STOPPED
  └─ hazard cleared + policy allows → DEGRADED

STOPPED
  └─ explicit release/restart condition → DEGRADED
```

不要なstop/restart振動を避けるため、thresholdにhysteresisを持たせる。

## 9. ログ

毎cycle記録：

- input stampsとage
- model inference ms
- predicted waypoints
- target speed
- stop probability
- mode
- front distance
- stopping distance
- nominal command
- final command
- safety intervention reason
- node health

## 10. 未確定パラメータ

次は公式車両・sim環境で同定する。

- wheelbase
- steering tire angle limit
- steering rate limit
- acceleration/braking limits
- actual actuator latency
- LaserScan角度原点と正方向
- vehicle footprint
- LiDAR mounting transform
