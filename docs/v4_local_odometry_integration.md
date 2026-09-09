# VelocityReport局所OdometryとV4 shadow

## 契約

`/vehicle/status/velocity_status` のbase_link座標のvx,vy[m/s],wz[rad/s]だけを使う。
stamp差[s]と端点平均twistをSE(2)積分し、最初の有効標本を局所原点とする。
GNSS、IMU、EKF、地図、制御command、車両寸法は積分入力にしない。
実車の車輪エンコーダー精度を保証しない。根拠は `v4_wheel_odometry_fields_audit.md`。

出力は `/v4/local_odometry` (Odometry)、TF `v4_odom → v4_base_link`。
v4_base_linkは入力base_linkと同じ物理基準点・軸向きの専用別名で、既存TFへ連結しない。
既存base_linkの親を変更せず、グローバルmap位置を取得しない。
Odometryのstampを現在時刻で再stampしない。共分散は未推定の大きな対角sentinelであり校正値ではない。

実装profile: max_gap_s=.25、max_speed_mps=10、max_yaw_rate_rps=2。
速度10m/sは既存試験監視の上限と整合。yaw上限は今回の低速sim試験用入力検査値で、
車両能力を実測した値ではない。値を超えた場合に適当なクリップをせず停止する。
同時刻同値は再publishしない。同時刻競合、非有限、stamp逆転、gap超過はfaultを保持。
明示clock巻戻りで局所epochを更新し原点reset。V4 session側は既存どおりclock resetで終了する。
欠落中の外挿・古いposeの再publishはしない。利用側はTF/poseの時刻と期限を必ず検査する。

## V4接続

通常make devの直接node起動では、JSONの `start_local_odometry: true` により
別名ROS nodeを同じ親processのexecutorへ追加する。モデルworkerとは分離される。
独立起動する場合のみ `local_odometry:=true` launchを使用し、JSONの起動flagはfalseにする。
両方を同時に有効化して二重起動しない。
V4 JSONには以下を設定する。

```json
{
  "start_local_odometry": true,
  "pose_frame": "v4_odom",
  "pose_child_frame": "v4_base_link",
  "pose_evidence": "VelocityReport-only SE2 integration; local origin; v4_base_link aliases physical input base_link axes, no global localization"
}
```

`topics.odometry=/v4/local_odometry`、`expected_nodes.odometry=/v4_local_odometry`。
その他の既存session/model/source/command契約は維持する。
この版のshadow joinは局所poseをまだ必須とする。GNSS/IMU由来のpose依存は外れるが、
完全なpose不要のraw推論分離まで完了したとは扱わない。
PPの既存Reference走行は別経路であり、今回のPP走行をE2E部門適合走行とは呼ばない。

## 再現・検証

Windows commit → 既定CheckOnly/sync → WSL共有lockで実行。

```bash
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_local_odometry_v4.py tests/test_shadow_observation_join_v4.py tests/test_v4_shadow_package.py
```

実ROS起動例（選択済みsimulator用configが必要）:

```bash
ros2 launch aic_e2e_runtime v4_shadow.launch.py config_file:=/v4/live.json
```

試験では前回同様の専有make dev wrapper、PP単一制御、V4非制御、1周・300wall秒上限を使用する。
AWSIM本体と既存dirty checkoutは変更しない。停止/故障を成功扱いせずログに残す。
