# 00. 要求仕様

## 1. 目的

Camera、2D LiDAR、車両状態を使うTransFuser風E2Eモデルにより、競技環境で以下を実現する。

- R-FUNC-001: 規定コースを安定して完走する。
- R-FUNC-002: 走行可能領域内に安全な回避余地がある障害物を回避する。
- R-FUNC-003: 安全な回避が成立しない場合、衝突前に停止する。
- R-FUNC-004: 回避後に走行ラインへ安定して復帰する。
- R-FUNC-005: センサ欠損・推論異常時に暴走しない。

## 2. 機能分解

| 要求 | 主担当 | 補助担当 | 検証方法 |
|---|---|---|---|
| 完走 | Camera/LiDAR融合モデル | waypoint controller | 複数seed・複数runの完走率 |
| 回避 | LiDAR表現、fusion、trajectory head | safety corridor checker | 左右回避scenario |
| 回避不能停止 | stop/risk head | Safety Supervisor | 完全閉塞scenario |
| 復帰 | future waypoint head | controller平滑化 | 回避後の横偏差・復帰時間 |
| 異常時停止 | Safety Supervisor | command mux | timeout/NaN fault injection |

## 3. 非機能要求

- R-NF-001: 推論周期は最低10Hzを初期目標とする。
- R-NF-002: センサ受信から制御出力までのp95遅延を計測する。
- R-NF-003: 学習・評価条件をconfigで再現できること。
- R-NF-004: 全runについてデータ出所、scenario、split、モデル、重み、結果を追跡できること。
- R-NF-005: Safety Supervisorは学習モデルと独立してunit test可能であること。
- R-NF-006: 直接制御値にNaN/infまたは範囲外があれば安全側へ倒すこと。

## 4. 入出力境界

### モデル入力

- RGB camera image
- 2D LiDAR LaserScan
- longitudinal velocity
- lateral velocity（利用可能な場合）
- heading/yaw rate（利用可能な場合）
- steering tire angle
- gear status

### teacher/debug-only候補

- future pose
- MPC planned trajectory
- map pose
- GT obstacle state
- collision/offtrack flag
- scenario ID

teacher/debug-only情報は、推論時入力へ混入させない。

### モデル出力

必須：

- future waypoints in ego/base frame
- target speed
- stop probability
- behavior mode

任意：

- direct steering/acceleration auxiliary output
- collision risk
- uncertainty/confidence
- multiple trajectory hypotheses

## 5. 初期受入基準案

以下は開発用の暫定値であり、公式コースと車両特性に合わせて更新する。

| Gate | 暫定基準 |
|---|---|
| データ整合 | 有効サンプル率95%以上、重大なtimestamp逆転0件 |
| 推論疎通 | 合成入力で全Headが有限値を返す |
| 安全停止 | 固定閉塞テストでstop false negative 0件 |
| 通常走行 | baselineで短区間を連続走行できる |
| 完走 | 固定seed群で完走率を記録し、回帰低下を検知できる |
| レイテンシ | p50/p95/maxを記録し、制御周期を超えた場合に検知する |
