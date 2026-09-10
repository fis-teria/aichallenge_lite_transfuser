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
