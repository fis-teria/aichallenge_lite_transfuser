# 復帰収集時の外乱地点マーカー

標準 RViz の `rviz_default_plugins/MarkerArray` で、各走行の外乱地点を表示する。
topic は `/recovery_teacher/disturbance_markers`、座標は `map`、位置の単位は m。
左への操舵外乱は黄色、右はピンク。円・左右矢印・`S00 LEFT` などの地点ラベルを残す。
マーカーはその走行中保持し、次の独立走行では初期化する。

予定地点ではなく、`/recovery_teacher/phase` の記録から、実際に非ゼロの
外乱付き操舵指令を publish した最初の観測位置を使う。publish されなかった指令、
ゼロの立ち上がり、上限処理で外乱が無効になった指令、通過を見送った地点は表示しない。
これは指令送信位置であり、車輪が応答した時刻や復帰成功を示すものではない。

表示は独立した `time_recovery_paths_node.py` が担当する。制御やモデル入力へ戻さない。
Reliable / Transient Local / depth 1 で全マーカーを 0.5 s ごとに再配信するため、
RViz が後から接続しても既存の地点を受信できる。専用走行の RViz 設定に表示を追加し、
元環境の設定は変更しない。rosbag にも topic を記録する。

各 run の `disturbance_markers.json` に地点 ID、左右、実測位置、指令 publish 時刻、
その時の外乱量 rad を保存する。`path_heartbeat.json` に表示数と購読ノード名を残す。

Windows で commit 後、既定の同期・lock を用い、WSL の native checkout で検証する。

```bash
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_recovery_disturbance_markers.py
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
```

公式 ROS 環境での確認コマンド:

```bash
ros2 topic info /recovery_teacher/disturbance_markers --verbose
ros2 topic echo /recovery_teacher/disturbance_markers --once --qos-durability transient_local
```

実走行での表示・収集結果は `time_site_recovery_collection_20260915.md` に記録する。
