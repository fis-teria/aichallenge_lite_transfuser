# 09. リスク登録簿

| ID | リスク | 影響 | 検知 | 対応 | 優先度 |
|---|---|---|---|---|---:|
| R01 | 公開データの利用許諾不足 | 利用停止・提出不可 | license review | 許諾確認、出所記録 | 最高 |
| R02 | Camera/LiDAR/Control同期ずれ | 回避遅れ・蛇行 | delta histogram | tolerance、label shift | 最高 |
| R03 | 通常走行偏重 | 障害物で突進 | scenario count | oversampling、追加収集 | 最高 |
| R04 | 停止ラベル誤定義 | 止まりすぎ/止まらない | confusion review | intentional stop区別 | 最高 |
| R05 | 人間の操作揺れ | 蛇行 | steering spectrum | waypoint教師、平滑化 | 高 |
| R06 | LiDAR invalid値 | 誤停止/非停止 | range audit | sanitize、valid mask | 高 |
| R07 | Camera crop誤り | 境界/障害物欠落 | overlay video | crop review | 高 |
| R08 | offline過適合 | simで走らない | closed-loop test | scenario split、DAgger | 最高 |
| R09 | 推論遅延 | 衝突 | p95/max latency | 軽量化、deadline stop | 最高 |
| R10 | safety過介入 | 完走不能・低速 | intervention rate | threshold調整、hysteresis | 高 |
| R11 | safety不足 | 衝突 | stop FN | conservative margin | 最高 |
| R12 | Controller tuning不良 | ふらつき | rate/RMS | wheelbase、gain同定 | 高 |
| R13 | Transformer効果なし | 工数浪費 | ablation | late fusion gate | 中 |
| R14 | BEV幾何誤り | 左右反転・障害物位置ずれ | synthetic geometry test | frame convention固定 | 最高 |
| R15 | train/test leakage | 過大評価 | manifest audit | run/scenario split | 高 |
| R16 | ROS message版差異 | build/runtime失敗 | official env compile | adapter層 | 高 |
| R17 | 重み・config不一致 | 再現不能 | model metadata | schema/version check | 高 |
