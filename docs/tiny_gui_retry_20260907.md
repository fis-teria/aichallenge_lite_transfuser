# Tiny GUI hostname修正後の追加短試験

ユーザーの「もう一回やりましょう」を、直前の質問どおり短試験1回の追加許可として記録する。
開始HEAD `0e15a675b433e74f2efadff0d7635af5a7d55800`、clean。
Windows正本、branch `codex/windows-wsl-training-sync`、既存originは変更しない。

新profile `TINY_GUI_RETRY_20260907`、承認file
`configs/control/tiny_gui_retry_authorization_20260907.json`。
旧profile・旧失敗attempt・保守計上600forward/1駆動/20sim秒を保存し、累積powered上限4→5のみ追加。
新profileも1attempt限定。前回GUI承認からの遷移だけを許可する。
駆動8sim秒、制動込み20sim秒、Tiny600forward、runtime120wall秒、host予約230wall秒。
共通wall3600、forward6000、powered300sim秒、log512MiB、snapshot16、MPC6000は不変。
2026-09-07 09:50 JST駆動終了/10:00期限も不変。lapは許可しない。

変更は承認profileの追加・選択・記録のみ。公式Tinyの重み/前処理、速度2.0/上限2.4m/s、
操舵、watchdog、Ready条件、GUI照合、停止処理、AWSIM実行物・設定を変更しない。
既存dirty remote checkoutは上書きせず、新commitの専有archiveと単独ROS buildを使う。

## 実施コマンド

Windows commit後（同期scriptは未変更。固定Dataset root存在判定のみ）：

```powershell
pwsh -NoProfile -File tools/sync_to_wsl.ps1 -CheckOnly
pwsh -NoProfile -File tools/sync_to_wsl.ps1
```

WSL同commitで：

```bash
TINY_OFFICIAL_PACKAGE=/home/thistle/e2e_autonomous/tiny_lidar_lap_20260907/official_package bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_tiny_lidar_sim.py tests/test_tiny_gui_integration.py --junitxml=/home/thistle/e2e_autonomous/tiny_gui_retry_20260907/tests.xml
```

専有source内から以下を、正確なcommit/install/outputを指定して一回だけ実行する：

```bash
make dev CONTROL_METHOD=tiny_lidar_net_guarded TINY_GUI_RETRY=1 \
 SIM_REPO=/home/graneple/git/autononous_ai/aichallenge-racingkart \
 TINY_PACKAGE=/home/graneple/e2e_autonomous/tiny_lidar_lap_20260907/official/tiny_lidar_net_controller \
 TINY_INSTALL=EXACT_NEW_INSTALL DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority \
 TINY_OUTPUT=UNIQUE_NEW_OUTPUT TINY_COMMIT=EXACT_COMMIT \
 TINY_BUDGET=/home/graneple/e2e_autonomous/spatial_run_continuation_20260907/budget.json
```

履歴commandは追加試行許可ではない。実行結果は生ログと別の結果文書へ保存する。
自動push、Dataset内容/raw/学習sensor/V4checkpoint読取、学習はしない。

## 起動失敗後の限定修正

実行版3bddc66は両container作成後、既存recipeの再帰`$(MAKE)`が追加`-f`を
継承せず、source checkoutの別Makefileを読んで`autoware-command-mode-run`不在で終了した。
新methodだけ`MAKE`に同じincludeの`-f`を明示する修正を追加し、
合成Makefileで子・孫makeとcommand variableの継承を検証する。
この修正後にROS再起動・追加駆動はしていない。予算を再追加するものではない。
