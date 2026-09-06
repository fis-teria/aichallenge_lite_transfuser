# Tiny GUI: 再帰make修正後の追加短試験

ユーザーの「もう一個追加して、試験を実施してください」により短試験1回を追加。
開始Windows HEAD `d672a82c46c4af355076cd0adab5699d812230c1`、clean。
origin `https://github.com/fis-teria/aichallenge_lite_transfuser.git`、branch `codex/windows-wsl-training-sync`。
新profile `TINY_GUI_RETRY2_20260907`、承認は`configs/control/tiny_gui_retry2_authorization_20260907.json`。
旧失敗/消費を変更せず、累積powered上限5→6のみ追加。新profileは1attempt限り。

駆動最大8sim秒・制動込み20sim秒・Tiny600forward・runtime120wall秒・host予約230wall秒。
共通wall3600/forward6000/powered300sim秒/log512MiB/snapshot16/MPC6000は不変。
09:50 JST駆動終了、10:00 JST期限は2026-09-07のまま。lapは許可しない。
公式モデル・入力前処理・速度2.0/上限2.4m/s・操舵・停止条件・AWSIMは変更しない。
再帰makeへのinclude伝達は前回修正af80183を使用する。

## 検証と実行

Windows commit後、未変更scriptで`pwsh -NoProfile -File tools/sync_to_wsl.ps1 -CheckOnly`と通常同期を行う。
既定Dataset rootの存在確認のみ許可。内容/raw/学習sensor/V4checkpointは読まない。
WSLの同commitで以下の限定検証（学習・forwardなし）：

```bash
TINY_OFFICIAL_PACKAGE=/home/thistle/e2e_autonomous/tiny_lidar_lap_20260907/official_package bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_tiny_lidar_sim.py tests/test_tiny_gui_integration.py --junitxml=/home/thistle/e2e_autonomous/tiny_gui_retry2_20260907/tests.xml
```

専有source archiveと別build/installをSSH先へ用意し、追加packageだけcolcon build。
既存dirty `/home/graneple/git/autononous_ai/aichallenge-racingkart` は上書きしない。
実行はこの引数を固定commit・専有install/outputで一度のみ：

```bash
make dev CONTROL_METHOD=tiny_lidar_net_guarded TINY_GUI_RETRY=2 \
 SIM_REPO=/home/graneple/git/autononous_ai/aichallenge-racingkart \
 TINY_PACKAGE=/home/graneple/e2e_autonomous/tiny_lidar_lap_20260907/official/tiny_lidar_net_controller \
 TINY_INSTALL=EXACT_INSTALL DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority \
 TINY_OUTPUT=UNIQUE_OUTPUT TINY_COMMIT=EXACT_COMMIT \
 TINY_BUDGET=/home/graneple/e2e_autonomous/spatial_run_continuation_20260907/budget.json
```

GUI所有/隔離確認、Ready/新scan、独立watchdog、期限/停止条件を維持。
この文書はさらに次の再試行・台帳減額・実車接続を承認しない。自動pushなし。
