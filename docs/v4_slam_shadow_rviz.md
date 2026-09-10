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

## 2026-09-10 実行結果

Windows実行commit 4f8d63daaef241b55dcb18cc0360e46c89749c50。
WSL CheckOnly/CHECK_OK・通常同期SYNC_OK。既定Datasetルート存在確認を実施。
Dataset内容/rawへのアクセスなし。固定checkpointは許可されたshadow推論で既存loaderが読取。
lock限定tests25 passed、同版全pytest 1779 passed/4 skipped/51 warnings、85.63秒。
SSH専有 /home/graneple/e2e_autonomous/v4_slam_shadow_23 にWindows archiveを配布し、
Humbleで1package build成功（1.30秒）。既存AWSIM/dirty checkout変更なし。

Cartographer runner fork版 f72e05c、binary source c793ee0/core dadc454aを使用。
```bash
V4_SLAM_SHADOW_ROOT=/home/graneple/e2e_autonomous/v4_slam_shadow_23 CARTOGRAPHER_BUILD_ROOT=/home/graneple/e2e_autonomous/cartographer_extrap_build_20260910 CARTOGRAPHER_TEST_PROJECT=codex-v4-slam-shadow-23 CARTOGRAPHER_V4_EXTRAP_COMPARE=1 python3 /home/graneple/e2e_autonomous/cartographer_v4_shadow_20260910_01/run_moving.py
```
同probe container内でV4と専用RVizを所有し、既存make devのPPのみが制御。
make側V4_SHADOW_ENABLED=falseは二重起動防止で、別所有processのV4が実推論する。
GNSS/IMUの評価購読flagはOFF。Cartographer入力はscan＋車速/操舵Odometry。

- 移動20.005sim秒、最高4.53844m/s。host wall69.843秒。
- V4 forward/PLAN/display publish各172、うち走行中観測132。
- cuda:0、NVIDIA GeForce RTX 4060 Laptop GPU、torch2.3.1+cu121。
- 独立ROS購読で20点Path172件、消去Path14件を受信。
- 保存PLANを同じ観測poseで変換した値と実受信Pathの最大差3.55e-15m。
- RVizのOpenGL4.5起動ログあり。画面pixel/screenshot確認はNOT_RUN。
- 車両制御は唯一の/simple_pure_pursuit_node。V4のacceptedは追従採用を意味しない。

**終了時の不具合を保全**: probeが20秒観測完了しfault=nullで結果保存した後、
子process終了待ち中にhostがOBSERVER_STALEと判定して停止した。
host_result.jsonのerrorは書き換えず、正常終了扱いにしない。
fork側6bd186aでFINALIZING通知を追加し、hostが直ちに所有sim停止へ進むよう修正。
この終了修正は実走行再試験NOT_RUN。監視自体を無効化・timeout延長していない。
所有AWSIM freeze/KILL、Autoware/RViz/V4/Cartographer終了、関連process/container残存なし。
ブレーキ停止、一周完走、V4制御、精度合格は主張しない。

生ログとsummary: Windows tmp/v4_slam_shadow_23_evidence、SSH同run/evidence。
`C:\Python310\python.exe tmp/summarize_slam_shadow.py`で受信20点を照合。
RViz設定の配布版はconfig/v4_slam_path.rviz。実行時のtmp版と同内容。
既存8chartフォルダと未追跡reportはユーザー許可で
tmp/preserved_before_slam_shadow_20260910へ退避（17files、manifest元path/size/SHA256、全一致）。
削除なし。復元可能。WindowsからもSSHからもpush未実施。
