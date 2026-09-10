# V4-20 AWSIM observation trial

ユーザーのAWSIM試験依頼に基づく、20 sim秒の実入力shadow試験。
遠方validation MAEが16 epochより小さい12 epochを固定する。
重みSHA256: `07f2a04b9da0e2673036a1f0de6e13d841f1c4f84aa12e7a33ff61c070286c33`。

`v4_shadow_node`の明示`long_model`設定だけで46点loader/sessionを選択する。
設定は`checkpoint`絶対pathと上記`sha256`。既定2m loaderは従来どおり。
46点のraw XY[m]を同じ観測時刻SLAM姿勢で表示。固定2m PP adapterへは接続しない。
既存PPのみが車両を操縦し、V4-20は受信・推論・診断出力のみ。
停止は所有AWSIMのfreeze/killで、制動成功やV4閉ループ走行ではない。

Windows commit・CheckOnly・通常同期後、WSLで：

```bash
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
```

独立配布先のHumble packageをbuildし、既存run27 runnerを専用runへ複製する。
checkpoint mountを12 epochへ変更し、shadow PP connection起動を省く。
host wall上限180秒、probe移動上限20秒、shadow wall150秒を維持する。
結果と正確な実行コマンドは試験後に追記する。

## 実行結果

実行commit `cfd5bb5d1aabfae7adc120c7f9819891b3ff50be`。
Windows commit → 既定CheckOnly → 通常sync成功。WSL lock下で
1833 passed / 4 skipped / 52 warnings、71.81秒。
skipは従来のOSQP/JSON Schema/任意公式package依存。
WSL実checkpoint loader + 保存input-only tensorの46点forwardも成功。
AWSIMホストHumble package buildは1 package成功、checkpoint SHA一致。

run28は補助wheel odometryのimport配置不足で走行開始前に失敗。
`/probe/core`を今回の配布source `/v4/src`へ修正し、必要Luaを追加した。
失敗ログ・cleanup raceのerrorを保全し、新規run29で再実行。
AWSIM binary/設定、既存host checkoutは変更していない。

```bash
CARTOGRAPHER_BUILD_ROOT=/home/graneple/e2e_autonomous/cartographer_extrap_build_20260910 \
CARTOGRAPHER_TEST_PROJECT=codex-v4-20-shadow-29 CARTOGRAPHER_V4_EXTRAP_COMPARE=1 \
V4_SLAM_SHADOW_ROOT=/home/graneple/e2e_autonomous/v4_20_shadow_28 \
python3 /home/graneple/e2e_autonomous/cartographer_v4_20_shadow_29/run_moving.py
```

run29は既存make devのPPを唯一の制御送信元として公式Start要求を実施。
V4-20は実camera/LiDAR/ego/既存PP過去指令を受信。教師・future poseは入力しない。
SLAM姿勢は同観測時刻の外部表示変換に使用。

|項目|観測結果|
|---|---:|
|移動観測|20.004999552 sim秒|
|host全工程|61.6628秒|
|最高速度（既存PP）|4.538576 m/s|
|forward / 46点PLAN / ROS PLAN受信|166 / 166 / 166|
|走行中観測時刻のPLAN|123|
|非空ROS Path受信|165（全件46点）|
|保存raw+観測poseと受信Pathの最大座標差|0 m|
|forward時間 中央値 / p95 / 最大|30.780 / 60.045 / 417.235 ms|
|host error / probe fault / cleanup error|なし / なし / なし|

最大時間は初回forward。GPUはRTX 4060 Laptop、torch2.3.1+cu121。
時間は入力freeze/転送/モデル/CPU出力copyを含むworker計測で、end-to-end制御遅延ではない。
166件すべてfinite。1件は表示publishなし（既存表示の鮮度判定あり）。
表示165件すべてを独立ROS受信で照合した。RViz画面pixel確認は未実施。
入力join拒否はCAMERA_GRID_TOLERANCE 52、SUPERSEDED_BY_READY 32、JOIN_DEADLINE 4。
終了末尾にはTRANSPORT_FAULT:TRANSPORT_CLOSEDを記録し、削除していない。
forward REJECTEDは0件だが、入力候補全件採用という意味ではない。
graph検査はsingle publisher確認で、個々のmessage送信元検証ではない。

V4実制御送信0。終了は所有simulator freeze/killであり、ブレーキ停止ではない。
終了後docker psは空、AWSIM/V4/Cartographer/RVizプロセス残存なし。
経路図には折り返し・遠方のばらつきがある。異なる車両時刻の経路なので
この図だけから精度や時間的不安定性は断定できない。
20mは出力gridの教師距離であり、予測endpoint距離を20mへ拘束するものではない。
正解経路との精度比較、46点PP接続、V4自身の追従、完走・無接触・停止性能は未検証。

## 評価の場所と証拠

評価はWSL native filesystemで、共有lock下に実施。

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  /home/thistle/e2e_autonomous/runs/v4_20_awsim_shadow_20260910/evaluate.py \
  /home/thistle/e2e_autonomous/runs/v4_20_awsim_shadow_20260910/evidence \
  /home/thistle/e2e_autonomous/runs/v4_20_awsim_shadow_20260910/evaluation02
```

新規output必須。同名再実行は拒否する。
WSLには完全evidence、summary、raw_paths.pngを保持。
Windowsの`tmp/v4_20_shadow_28/`にはrunner/Lua/config/evaluate.py/summary/図を保存。
host完全ログは`/home/graneple/e2e_autonomous/cartographer_v4_20_shadow_29/evidence`。
初回失敗は同`cartographer_v4_20_shadow_28/evidence`。
重みはWSLからhostへ直接転送し、Windows/Gitに追加していない。pushなし。
