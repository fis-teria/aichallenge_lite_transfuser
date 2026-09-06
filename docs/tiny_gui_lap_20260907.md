# Tiny GUI 1周試験（追加承認1回）

ユーザーの「そしたら、つぎはコースを１周してみましょうか」と上限提示後の「大丈夫です」に基づく。
開始HEAD a40abef37047dab7b4184d250552456fe8105c65、clean。
origin https://github.com/fis-teria/aichallenge_lite_transfuser.git、branch codex/windows-wsl-training-sync。
新しい TINY_GUI_LAP_20260907 は1attemptのみ。旧失敗・消費を保全する。
累積 powered 6→7、sim 300→320秒、shared forward 6000→7100。
1回は制動込み240sim秒、Tiny5200forward、runtime600wall秒、host予約710wall秒。
wall3600秒、log512MiB、snapshot16、MPC6000は据置。09:50 JST駆動終了、10:00 JST期限。
停止予約6sim秒を除く駆動上限234秒。最初の順序付きjudge lapで停止要求。
近接地点、推論回数だけで完走としない。未完走でも自動再試行しない。

公式モデル・前処理・操舵・速度2.0/上限2.4m/s・AWSIMを変更しない。
RVizの既知の `//.rviz2` 保存失敗に対し、専有evidence/rviz_configだけを `/.rviz2` にRW mount。
runtimeのGLX vendorをnvidiaに指定、simulator環境や物理device mountは変更しない。
RVizの終了時heap corruption原因はUNKNOWNであり、この変更で解決済みとはしない。
GUI gateはPID/可視性に加えtiny_scan.rviz設定名付きwindowを要求し、裸のrviz2 dialogを成功にしない。
参考: [RViz source](https://raw.githubusercontent.com/ros2/rviz/humble/rviz_common/src/rviz_common/visualization_frame.cpp)、
[NVIDIA GLX PRIME](https://download.nvidia.com/XFree86/Linux-x86_64/435.17/README/primerenderoffload.html)。

## 検証・実行手順

Windows commit→未変更 tools/sync_to_wsl.ps1 -CheckOnly→通常同期。
既定Datasetルート存在確認のみ許可。Dataset内容/raw/学習sensor/V4checkpointは読まない。
WSLで同commitをlock付き限定pytest。全体pytestは許可外データアクセス防止のため実行しない。

```bash
TINY_OFFICIAL_PACKAGE=/home/thistle/e2e_autonomous/tiny_lidar_lap_20260907/official_package bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_tiny_lidar_sim.py tests/test_tiny_gui_integration.py
```

SSH先既存dirty checkoutは編集せず、固定source archiveと専有build/installを使う。
追加ROS packageのみ既存固定image内でcolcon build。原本AWSIMはRO mount、隔離とwatchdogを維持。

```bash
make dev CONTROL_METHOD=tiny_lidar_net_guarded TINY_GUI_LAP=1 \
 SIM_REPO=/home/graneple/git/autononous_ai/aichallenge-racingkart \
 TINY_PACKAGE=/home/graneple/e2e_autonomous/tiny_lidar_lap_20260907/official/tiny_lidar_net_controller \
 TINY_INSTALL=EXACT_INSTALL DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority \
 TINY_OUTPUT=UNIQUE_OUTPUT TINY_COMMIT=EXACT_COMMIT \
 TINY_BUDGET=/home/graneple/e2e_autonomous/spatial_run_continuation_20260907/budget.json
```

実行後にjudge、Tiny送信/scan対応、実速度と制動、介入、GUI終了、全消費と原本保全を報告。
これは実行前の仕様であり実測成功ではない。学習・実車・自動pushは許可しない。
