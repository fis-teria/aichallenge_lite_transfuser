# V4-20 predicted-path AWSIM tracking trial

ユーザーのV4-20追従走行依頼に基づく独立したSIM_ONLY実行方針。
12 epoch固定重みは前回と同一SHA。既定shadow/旧制御gateは変更しない。
専用`CONTROL_METHOD=v4_20_external`で既存基準経路controllerを起動しない。
唯一の`/v4_20_controller`がV4-20経路をPPで追従し、送信した実過去指令を
推論側がfinal_fallback履歴として受信する。速度予測モデルではない。

46点raw全体を保存し、制御には連続した近傍3m prefixと1m lookaheadを使用。
観測時SLAM root座標から現在SLAM rear axle座標へ変換する。
目標0.25m/s、過速度0.45m/sで停止、操舵上限0.5rad/速度0.8rad/s。
幾何異常・必要操舵超過・期限切れは負加速度要求。
独立ROS親controllerのwall timerがモデルprocessを待たず監視する。
LiDAR前方停止回廊、実速度、操舵、現在pose、clock、単一送信元を毎周期検査。
これは全周囲・接触判定の証明ではなく低速sim試験の近接監視。

公式Startの完了後にhostが専用instanceへ有限権限fileを作成する。
駆動10sim秒、その後制動、14sim秒で終了。host wall180秒、heartbeat監視あり。
物理deviceなし、DDS loopback、所有container/command consumerを確認する。
host予備停止は所有AWSIM freeze/kill。移動後速度<0.03m/sが1秒継続した場合のみ
ブレーキ停止観測を別記録。既存他者process・host checkout・AWSIMは変更しない。

Windows commit → sync CheckOnly → sync後、WSL共有lockで:

```bash
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
```

ROS環境で専有sourceをbuildし、`tmp/v4_20_tracking_30/run_moving.py`を実行。
runnerは前回29を複製し、controller起動・単一送信元・有限駆動/停止監視を追加。
評価はWSLで実施し、全attemptの結果・コマンドを追記する。

run32では駆動区間にPREFIX_FOLDBACK 169周期、PLAN_STALE 31周期で正加速0。
全近傍3mを必須とする方針から、最初の折り返し/不連続の手前で打ち切った
連続prefixを採用する方針へ変更。点の移動・平滑化・閾値緩和はしない。
4点以上、rearから1m以上のlookahead、停止余裕0.2m+0.5秒遅延+v²/(2×1m/s²)
が満たされないprefixは拒否する。元46点とcutoff理由・採用点数を記録する。

run33では65更新経路から180回の追従指令を送信し、速度積分0.3758m、
最高0.05538m/sの微動後に停止した。probeの走行判定>0.1m/sを満たさずNO_MOTION。
幾何拒否は解消し、駆動中PLAN_STALE 21周期で停止要求。
次のrun34は目標0.25m/sを維持し、速度Pゲイン2→4、加速要求上限0.5→1m/s²。
試験速度が低い原因を物理抵抗へ断定せず、応答を比較する。TTL/障害物/過速度監視は維持。

## 最終結果：低速・短区間の更新経路追従と停止

run34、実行commit `05ab767fbde8a23093a60b784a2991c82fddfdc3`。
V4-20 epoch12の予測を使用し、既存基準経路PPは起動していない。
車両control topicの唯一の送信元は `/v4_20_controller`。
新規観測→V4推論→連続prefix→PP操舵/速度P→実送信→速度観測を接続した。
全65更新経路・176追従指令について、保存rawとその時点の観測pose/現在pose/車速から
WSLで再計算し、lookahead・採用点数・加速要求が一致した。
操舵要求には別途0.8rad/sのrate limitを適用している。

|項目|run34観測|
|---|---:|
|駆動区間の設定|10 sim秒、続いて制動|
|駆動許可後の観測時間|13.209999704 sim秒|
|移動距離（独立速度購読の台形積分）|1.254568555 m|
|最高速度（vx/vyのノルム）|0.162124251 m/s|
|モデルforward/PLAN全工程|221|
|追従で使った更新PLAN|65|
|追従指令/うち正加速|176 / 176|
|採用prefix|全176周期で先頭13点（教師grid 0.1～1.3m）|
|現在rootからのprefix残長下界|1.1577～1.2554 m|
|操舵要求|−0.04894～−0.00979 rad|
|駆動中のPLAN_STALE停止要求|25周期|
|独立速度で停止1秒継続を確認した時刻|駆動許可から11.284999747 sim秒|
|host wall全工程|69.6488秒|
|host error / probe fault / cleanup error|なし / なし / なし|

**滑らかな連続走行ではない。** 経路期限切れで途中5回ほど停止・再発進している。
0.25m/sの目標にも到達せず、定常に近い区間は約0.15m/s。
今回成立したのは低速・約1.25mの更新経路追従と移動後の制動停止であり、
20m全出力の採用、一般的な追従精度、一周完走、障害物回避、無接触は未検証。
元46点は保持。折り返し以降を削ってモデルの精度が改善したとは扱わない。
次の課題はPLAN_STALEの周期的発生と速度応答の切り分け、遠方経路品質の改善。

速度<0.03m/sが1sim秒継続したことを、controller側と独立probeの両方で確認。
その後にhostが所有AWSIMをfreeze/killした。freezeを制動成功の根拠にはしない。
終了後docker psは空。AWSIM/V4/controller/Cartographer/RVizの残存processなし。
AWSIM binary/assetや既存host checkoutを編集せず、専用配布先だけを使用。pushなし。

## 実行・検証・証拠

Windows正本でcommit→既定CheckOnly→通常sync。最終WSL全pytestは
1836 passed、4 skipped、52 warnings、82.48秒。
skipは既存のOSQP、JSON Schema、任意公式package依存。
Humble package buildは1 package成功。試験source tarはWindows/host両方で
SHA256 `646ce0b7186fc74bbcdfc052c346dec67f6635d0545de0a8059a90297ced92a9` 一致。
重みSHA256は `07f2a04b9da0e2673036a1f0de6e13d841f1c4f84aa12e7a33ff61c070286c33`。

実行コマンド（既存evidenceへ再実行しない。新規専有runが必要）：

```bash
CARTOGRAPHER_BUILD_ROOT=/home/graneple/e2e_autonomous/cartographer_extrap_build_20260910 \
CARTOGRAPHER_TEST_PROJECT=codex-v4-20-tracking-34 CARTOGRAPHER_V4_EXTRAP_COMPARE=1 \
V4_SLAM_SHADOW_ROOT=/home/graneple/e2e_autonomous/v4_20_tracking_34 \
python3 /home/graneple/e2e_autonomous/cartographer_v4_20_tracking_34/run_moving.py
```

hostの全ログは同runner directoryの`evidence/`。
WSLの`/home/thistle/e2e_autonomous/runs/v4_20_tracking_20260910/run34/`へ転送し、
共有lock下で評価した：

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  /home/thistle/e2e_autonomous/runs/v4_20_tracking_20260910/evaluate_tracking_34.py \
  /home/thistle/e2e_autonomous/runs/v4_20_tracking_20260910/run34 \
  /home/thistle/e2e_autonomous/runs/v4_20_tracking_20260910/evaluation34
```

評価outputは新規path必須。再計算には実行commitのcoreを使う。
summary内に主要5ログのSHA256、重みロードidentity、再計算数を保存。
Windowsの`tmp/v4_20_tracking_30/summary34.json`、`tracking34.png`、
runner/prepare/Lua/evaluate scriptも保持する。重みはGitに追加していない。

## 全attemptの保全

- run30：controller起動時にPYTHONPATHを上書きしrclpy import失敗。Start前。
  環境を引き継ぐようrunnerを修正。cleanup後に遅れて生成された所有simulatorを
  run31のNOT_QUIESCENTで検出し、owner labelを確認してpause/killした。
  元host_resultのcleanup=[]を後から成功証拠へ読み替えない。
  起動中make/Dockerの終了競合はrunnerの残課題。元ログを保全。
- run31：NOT_QUIESCENTでpreflight拒否、sim起動・駆動なし。
- run32：起動・推論成功、近傍foldbackにより正加速0、NO_MOTION。
- run33：連続prefix採用、約0.376mの微動。最高0.0554m/sでprobe判定NO_MOTION。
- run34：上記の低速追従・制動停止。全工程エラーなし。

run32/33の完全evidenceもWSL nativeの同run番号へ保存。
run30/31を含むhost側出力は削除していない。既定make devへの自動昇格は行わない。
