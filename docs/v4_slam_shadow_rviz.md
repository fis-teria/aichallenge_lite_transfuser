# V4 SLAM観測姿勢接続とRViz表示

slam_shadowを明示したconfigでのみ有効。既定shadowは不変。
入力 /cartographer_v4/tracked_pose (PoseStamped) -> /v4/slam_odometry (Odometry)。
元SLAM stampを保持し、LiDAR原点からVehicle rootへ前方1.1649999618530273mを逆変換。
frame=cartographer_v4_local、child=v4_base_link。実base_link TFは公開しない。
原点の対応根拠は現level1 hash 9ab2e1e8865c02885594e0bbdde372302530f047b5a2e89c455be6a18f3e090b。
V4の既存joinが画像観測時刻のposeを補間する。現在外挿poseを過去画像へ混ぜない。
現在時刻の外挿出力はCartographer側に残るがV4入力には採用しない。

slam_shadow config: lidar_forward_m=1.1649999618530273、geometry_evidence必須、rviz_path=true。
start_local_odometry=false、pose_frame=cartographer_v4_local、pose_child_frame=v4_base_link、
topics.odometry=/v4/slam_odometry、expected_nodes.odometry=/v4_slam_pose_adapter。

RViz: Fixed Frame=cartographer_v4_local、Path display topic=/shadow/v4/path。
未補正20点を、その同じforwardの観測時刻poseで固定局所frameへ変換して表示。
再推論/平滑化/制御経路への転送なし。各点stampは観測時刻で到達時刻ではない。
観測age500ms超は表示せず、sim500ms/受信後wall1秒で消去する。
Path表示は追従可否/安全性を意味しない。GNSS/IMU/mapへの固定TFは生成しない。

```powershell
.\tools\sync_to_wsl.ps1 -CheckOnly
.\tools\sync_to_wsl.ps1
```
WSL: `tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_slam_shadow_geometry_v4.py tests/test_v4_shadow_package.py tests/test_publisherless_shadow_v4.py`

ROS/AWSIM実行結果は別追記。実車command publisher/engageなし。
