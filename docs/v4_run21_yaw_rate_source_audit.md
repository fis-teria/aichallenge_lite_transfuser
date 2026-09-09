# Run21 異常角速度の生成箇所監査（保存物のみ）

調査HEAD: `0c93c662d147b6820c863110f9cf9b9e7bbdece3`。
目的はクランプ前の生成経路の確認。実装・AWSIM・参照CSVを変更していない。

## 結論と確度

1. **確定**: 約±1256rad/sは局所Odometryの積分結果ではなく、受信VelocityReport.heading_rateそのもの。nodeは変換せずcore.updateへ渡し、coreの警告は入力wzをそのまま記録する。
2. **コード上の欠陥を確認**: 試験時とhashが一致する保存DLLのVehicle.ComputeVehicleStateには、Euler角の成分差を折り返し処理なしで時間微分する式がある。角度境界で擬似的な約360度の差が生成される。
3. **有力だが動的確証は不足**: 実測3件の大きさはこの欠陥と整合する。しかし各瞬間のcurrent/last Unity quaternionとTime.deltaTimeが保存されておらず、3件それぞれの数値を原姿勢から完全再現したわけではない。teleport等の別の姿勢不連続もこの資料だけで完全には除外できない。

## 出典固定

保存DLL: `tmp/Assembly-CSharp-awsim-d1.dll`

SHA256: `859e5560dbffd7d0833cd1fe5eb1f36b87fa8d22f6e45eb0e1203d04a1ea6d13`

今回Get-FileHashで再確認し、run21の`evidence/v4_pp_shadow_21/provenance/prelaunch-manifest.json`記録と一致。以下のC#は保存逆コンパイル資料であり、公式公開ソースや現在SSH先の再確認ではない。

## 値の経路

| 段階 | ファイル・箇所 | 確認した処理 |
|---|---|---|
| 角速度生成 | `tmp/awsim_decompiled/AWSIM/Vehicle.cs:416` | `(rotation.eulerAngles - lastRotation.eulerAngles) / Time.deltaTime * (PI/180)` |
| 更新元 | 同 `FixedUpdate` 319/332行 | ComputeVehicleStateの後、lastRotationを現在rotationに更新 |
| 座標変換 | `tmp/awsim_decompiled/AWSIM/ROS2Utility.cs:41` | Unity `(x,y,z)` → ROS `(z,-x,y)` |
| ROS値生成 | `tmp/awsim_decompiled/AWSIM/VehicleReportRos2Publisher.cs:172` | `UnityToRosPosition(-vehicle.AngularVelocity)` のzをHeading_rateへ代入（175行） |
| ROS受信 | `ros2_ws/src/aic_e2e_runtime/aic_e2e_runtime/local_odometry_node_v4.py:56` | `/vehicle/status/velocity_status`のheading_rateをそのままcore.updateへ渡す |
| 積分・警告 | `src/aic_transfuser_lite/runtime/local_odometry_v4.py` | 前後twist平均を積分し、入力wzをyaw_rate_rpsとして記録。補正・クランプなし |

この角速度生成式に参照CSV・psi・曲率計算は登場しない。参照線が操舵を介して走行に影響することまでは否定しないが、共有されたゼロ長区間のatan2問題が、この巨大角速度の直接生成式ではない。参照線全体の正常性を本監査で証明したものでもない。

## 保存ログとの対応

`tmp/v4_pp_shadow_21/evidence/v4_pp_shadow_21/d1/autoware.log`:

| sim秒 | 入力heading_rate [rad/s] | 行 |
|---|---:|---:|
| 52.639998823 | +1256.332397461 | 983 |
| 60.094998656 | +1255.904296875 | 1163 |
| 72.974998368 | −1255.806640625 | 1590 |

積分後yawをラップするatan2(sin,cos)は既にあるが、入力角速度が壊れているため、それでは正常な移動量に戻らない。保存poseでは当該更新で約179度のyawジャンプがある。

## 数式の合成確認（実測ではない）

人工例としてUnity yawが359.95度から0.05度へ5msで変化した場合:

- 本来の短い差: +0.10度 → +0.349066rad/s (Unity)
- 現式の差: −359.90度 → −1256.288rad/s (Unity)
- publisherの符号・軸変換後: heading_rate +1256.288rad/s (ROS)

実測の巨大値と同程度になる。5msはこの例の仮定であり、保存ログで各発生瞬間の物理dtを直接確認した値とは扱わない。局所Odometryの受信更新間隔約35msと混同しない。

PowerShellによる合成計算と静的照合のみ実施。ROS/Unity実行テスト、追加収集、モデル推論、実装テストはNOT_RUN。既存の未追跡解析レポートは保全。自動pushなし。

## 残る確認と修正判断

生成式の欠陥と、受信時点で異常であることは確認できた。イベントごとの完全な確証には、物理更新時の前後姿勢とdt、生成AngularVelocity、対応するpublish値が必要。現保存物には不足しているため新規取得はしていない。

AWSIM不変更という条件は維持する。下流の修正を行う場合も、巨大値の一律クランプや、35msを使った機械的な2π補正を採用してはならない。まず生成dtと受信周期の違い、および正常な角速度を復元できる根拠を確定する必要がある。本タスクでは修正しない。
