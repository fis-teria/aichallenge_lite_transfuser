# 新ROSパッケージとCONTROL_METHOD（有限GUI短試験）

追加パッケージ: `aic_tiny_sim_test`。追加選択肢: `CONTROL_METHOD=tiny_lidar_net_guarded`。
既存`tiny_lidar_net`と他controllerの選択・公式Tinyモデルは変更しない。
公式推論coreを既存の検証済みworkerから利用し、独立supervisorが唯一の制御送信元になる。
公式ROSノードをそのまま起動したという主張ではない。V4/MPC/正解経路は使用しない。

## 許可と固定値

ユーザーが追加短試験を明示許可し、新package/control method追加を指示。
根拠は`configs/control/tiny_gui_authorization_20260907.json`。適用時刻はhostの実時刻、承認者名null。
GUI専用profileのみ累計駆動上限3→4。旧used/全attempt/不明消費は保持。
今回のattemptは1回限り（未駆動で失敗してもこのprofileで自動再試行しない）。
正駆動8sim秒、制動込み20sim秒、Tiny600forward、runtime120wall秒、起動終了込み230wall秒。
共通forward6000、駆動300sim秒、wall3600秒、log512MiB、snapshot16、MPC6000は不変。
09:50JST駆動終了・10:00期限は2026-09-07のまま。追加の一周試験は許可されていない。
速度目標2.0/上限2.4m/s、操舵pi/6rad、周期0.05sと制動式は従来どおり。

## 起動と分離

`integrations/tiny_gui/Makefile`がSSH先の既存Makefileをincludeする。
既存`dev`のsimulator/autoware起動recipeを使うが、新methodに限り事前記録を明示file allowlistへ分岐する。
source/build/install全体やPilotNet重みを読む旧fingerprintをこの新methodでは実行しない。
全体fingerprint相当の証明とは扱わない。旧methodのrecipeは変更しない。
通常の無指定make devを実行した結果とは区別する。

既存dirty checkoutを編集せず、Windows commit由来の追加package/includeを専用領域へ配布。
新methodのautoware serviceは新packageのlaunchへ接続し、通常のAutoware走行stackは起動しない。
RVizは実scanの可視化だけ（fixed frame=lidar）。経路生成/MPC/teacherは含めない。
GUIは実デスクトップDISPLAY/Xauthorityを明示し、Xvfbを使用しない。
両ウィンドウのPIDを所有containerのnamespace PIDへ照合し、IsViewableを確認後に駆動を許可する。
AWSIMは既存実行物と既存`run_simulator.bash dev`、双方read-only。
専用composeはnetwork=none/共有namespace、非privileged、cap_drop ALL、実機deviceなし。
音声は従来どおりOFF。カメラ/LiDAR、衝突、recovery等は前試験の設定を維持。
変更する画面条件はXvfb→デスクトップおよび画面サイズ640x360→960x540。

## 監視修正と終了

`host_armed()`はJSON読取後にmonotonic時刻を取得する。
750ms閾値は変えず、失敗時には読んだstamp/age/token/armed/判定理由を記録する。
前回の実原因を競合と断定した修正ではなく、静的反例への対処。
モデル停止・scan失効・非有限出力・競合publisher・clock異常・速度超過は停止側へ。
新規速度sampleで|v|≤0.03m/sを0.5sim秒確認した停止と、host freeze/KILLを区別。
freeze後にunpauseしない。対象project以外のprocess/containerは停止しない。

## ビルド・検証・試験手順

Windows commit→既定CheckOnly/sync→同SHA WSL lock付き限定pytest。
同期script変更なし。既定Dataset root存在確認のみで、Dataset/raw/学習sensor/V4 checkpointは未読取。
限定test対象は`tests/test_tiny_lidar_sim.py tests/test_tiny_gui_integration.py`。

ROSビルドは追加packageだけ。既存workspaceを上書きしないbuild/install/log先を明示する。
以下の変数は実行時の専有絶対パスとし、`ros2_ws/src/aic_tiny_sim_test`だけをbase-pathにする。

```bash
colcon --log-base "$TINY_BUILD_LOG" build \
  --base-paths "$TINY_SOURCE/ros2_ws/src/aic_tiny_sim_test" \
  --packages-select aic_tiny_sim_test \
  --build-base "$TINY_BUILD" --install-base "$TINY_INSTALL"
```

Windows同SHAを配布したsource directoryで：

```bash
make dev CONTROL_METHOD=tiny_lidar_net_guarded \
  SIM_REPO=/home/graneple/git/autononous_ai/aichallenge-racingkart \
  TINY_PACKAGE=/home/graneple/e2e_autonomous/tiny_lidar_lap_20260907/official/tiny_lidar_net_controller \
  TINY_INSTALL=EXACT_DEDICATED_INSTALL DISPLAY=:1 \
  XAUTHORITY=/run/user/1000/gdm/Xauthority \
  TINY_OUTPUT=UNIQUE_OWNED_OUTPUT TINY_COMMIT=EXACT_COMMIT \
  TINY_BUDGET=/home/graneple/e2e_autonomous/spatial_run_continuation_20260907/budget.json
```

host runnerが旧予算→追加許可→予約→精算を同じ台帳で記録する。
実GUI短試験を開始する前に既存ROSビルドの限定確認を行い、失敗時は駆動せず保存する。
この文書のコマンドだけで追加の試行枠・期限延長は許可されない。
未実施のビルド/GUI表示/制動を合成test成功から推定しない。実結果は別成果物に記録する。
