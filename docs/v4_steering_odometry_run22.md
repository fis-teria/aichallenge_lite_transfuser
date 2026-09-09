# 車速・操舵Odometry有効化: AWSIM試験22

今回のユーザー依頼により、新方式を有効にして既存Pure Pursuit走行＋V4 shadowを1試行する。
外側300wall秒・駆動240sim秒以内、公式1周Finishで終了。前run21の使用量を保持して引き継ぐ。
V4制御ではなく、既存PPのみが制御する。AWSIM本体・scene・既存dirty checkoutは変更しない。

## 有効化根拠

2026-09-10、SSH先の現level1を既存隔離UnityPyで読取。
SHA256 `9ab2e1e8865c02885594e0bbdde372302530f047b5a2e89c455be6a18f3e090b`。
GoKart1のWheelCollider 1227/1232の後輪中心はroot座標で横x=±0.635、前後z=−0.484m。
前輪1229/1231は平均横x=0、z=0.603m。rootまでの中間Transformは回転単位・scale1、横並進0。
したがって後輪中心線に対するVehicle rootの横offsetは0m、前後輪間隔は約1.087m。
Vehicle.LocalVelocityはrootのInverseTransformDirectionで得られ、publisherはこれをbase_linkとして報告する。
root=後輪中心とは主張しない（rootの縦offset約0.484m）。測定vyを保持するため縦offsetに伴う横速度を捨てない。
静的設計寸法でありタイヤ滑りや動的校正まで証明しない。

`tmp/v4_pp_shadow_22/geometry.json`と読取scriptに証拠を保存。
同runのlive.template.jsonにwheelbase_m=1.087、reference_left_offset_m=0、証拠IDを設定。
他の既定configへ無条件に適用しない。ROS合成smokeも車速＋操舵入力へ更新する。

## 実行手順

Windows commit → 既定CheckOnly/WSL同期 → lock付き限定テスト。
Windows git archiveをSSH先専有`/home/graneple/e2e_autonomous/v4_pp_shadow_22`へ配布。
Humbleのnetwork noneでbuild/合成ROS確認の後に、同runのrun.pyで通常make devを実行する。

```bash
timeout --signal=TERM --kill-after=15s 300s python3 /home/graneple/e2e_autonomous/v4_pp_shadow_22/run.py
# 内部: make dev CONTROL_METHOD=pure_pursuit CAPTURE=false ROSBAG=false AWSIM_LAPS=1 RUN_ID=v4_pp_shadow_22
```

試験結果・実行版・停止確認は後記。自動pushなし。
