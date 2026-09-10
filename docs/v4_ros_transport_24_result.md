# V4 ROS搬送・AWSIM短試験（2026-09-10）

## 結論と未完了範囲

既存Pure Pursuitが走行するAWSIM上で、V4の同一forward出力を診断ROS topicへ
搬送し、156/156件のraw座標・時刻・ID・姿勢根拠一致を確認した。
**V4経路→既存PP→車両の接続はまだ未完了**。今回の走行をV4追従と呼ばない。

## 版・変更

- repo: fis-teria/aichallenge_lite_transfuser
- branch: codex/windows-wsl-training-sync
- 実行commit: `adda37d11fa0857ebb71f58941ec17546c4d7f74`
- SSH host: graneple@192.168.3.10、ROS Humble
- racingkart: `/home/graneple/git/autononous_ai/aichallenge-racingkart`
  （実在するディレクトリ名はautononous_ai）
- racingkart HEAD: `4af395eee10f928c7fc7225760adfa04c4c07ff4` + 既存dirty。
  dirty checkoutを上書きせず専用ディレクトリへ配布。
- 固定checkpoint: SHA256 `0316692543a901d9d6b96718c5651d6739367aa851f83fb3a133c126ccc8919f`、strict load。

`publisherless_shadow_v4.py` のPLANへ既存Planのclock/epoch/frame/reference point/
expiryとpose根拠を追記した。`slam_shadow_io_v4.py` のオプション
`slam_shadow.plan_transport=true` で `/shadow/v4/plan_record`（std_msgs/String JSON）へ
同じレコードを送る。RViz Pathから経路を復元せず、再推論しない。
出力種別は `DIAGNOSTIC_NOT_CONTROL`。実車command publisherの追加はない。

## 試験結果

- 走行時間: 20.014999553 sim秒、host全工程72.4824秒。
- 最大速度: 4.53856376 m/s（16.3388 km/h）。既存PPの速度設定を使用。
- PLAN生成156、ROS受信156、照合一致156。
- 全156件でexpiry=source stamp、accepted=false。
  live車両Limits未設定による意図した無効状態。搬送で延命していない。
- command publisher: `/simple_pure_pursuit_node` の1つ。
- probe fault: null、host observer_completed=true、cleanup errorなし。
- 停止方式: **OWNED_SIMULATOR_FREEZE_KILL_NOT_BRAKING**。
  制動停止成功とは扱わない。終了後docker psは空。
- AWSIM改変なし。RViz起動あり、画面ピクセル検証・録画なし。
- 完走、無接触、V4経路の追従性能: NOT_EVALUATED。

初回ビルド後の合成ROS確認はDDS設定mount不足で失敗した。
既存cyclonedds.xmlをread-only mountした再試行でcolconとROS合成確認成功。
モデル設定確認はCONFIG_OK_NO_INFERENCE。限定pytestは38 passed。
同一commitの全pytestは1792 passed、4 skipped、51 warnings、92.33秒。
skipは既存OSQP/JSON Schema validator/公式Tiny package未導入による。

## 実行・保存

Windows commit → 既定CheckOnly → 既定同期 → WSL lock付き検証を実施。
同期によるDatasetルート存在確認あり。Dataset内部読取りなし。
AWSIM試験では許可された実sensorと固定checkpointを使用した。

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_publisherless_shadow_v4.py tests/test_slam_shadow_geometry_v4.py tests/test_v4_shadow_package.py tests/test_v4_pp_reference_adapter.py
```

SSH実行コマンド（再試行は新しい専有出力先を使うこと）:

```bash
CARTOGRAPHER_BUILD_ROOT=/home/graneple/e2e_autonomous/cartographer_extrap_build_20260910 CARTOGRAPHER_TEST_PROJECT=codex-v4-transport-24 CARTOGRAPHER_V4_EXTRAP_COMPARE=1 V4_SLAM_SHADOW_ROOT=/home/graneple/e2e_autonomous/v4_ros_transport_24 python3 /home/graneple/e2e_autonomous/cartographer_v4_transport_24/run_moving.py
```

内部で `make dev DEV_AUTO_START=false CONTROL_METHOD=pure_pursuit` と
公式 `make awsim-request-start` を使用。DDS loopback、物理device不在、
現在instanceのmountと所有者、単一command送信元を確認。
上限: host180秒、走行20sim秒または25wall秒、観測器heartbeat3秒。

ローカル保存: `tmp/v4_ros_transport_24/`。実行script、source.tar、
`evidence/{shadow.jsonl,timing.jsonl,probe_result.json,host_result.json,transport_summary.json}`。
SSH全生ログ: `/home/graneple/e2e_autonomous/cartographer_v4_transport_24/evidence/`。

## 次の接続実装で残るもの

最初の未解決事項はlive車両Limitsと現時刻状態の接続。
観測時刻SLAM poseとは別に、現在SLAM外挿pose・実速度・操舵から状態を構成する。
Vehicle root/後輪中心の違いを明示して、既存PPに専用Trajectoryを渡す必要がある。
既存PPはTrajectory受信ベースの鮮度判定と長いlookahead既定値を持つため、
単純remapだけでV4短経路に適合したことにはならない。
expiry時の指令無効化と制動停止を確認後、V4による短走行を実施する。
この報告はその接続作業やV4走行の完了報告ではない。
