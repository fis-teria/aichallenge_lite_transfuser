# VelocityReport局所OdometryとV4 shadow

## 契約

`/vehicle/status/velocity_status` のbase_link座標のvx,vy[m/s],wz[rad/s]だけを使う。
stamp差[s]と端点平均twistをSE(2)積分し、最初の有効標本を局所原点とする。
GNSS、IMU、EKF、地図、制御command、車両寸法は積分入力にしない。
実車の車輪エンコーダー精度を保証しない。根拠は `v4_wheel_odometry_fields_audit.md`。

出力は `/v4/local_odometry` (Odometry)、TF `v4_odom → v4_base_link`。
v4_base_linkは入力base_linkと同じ物理基準点・軸向きの専用別名で、既存TFへ連結しない。
既存base_linkの親を変更せず、グローバルmap位置を取得しない。
Odometryのstampを現在時刻で再stampしない。共分散は未推定の大きな対角sentinelであり校正値ではない。

実装profile: max_gap_s=.25、max_speed_mps=10、max_yaw_rate_rps=2。
速度10m/sは既存試験監視の上限と整合。yaw上限は今回の低速sim試験用入力検査値で、
車両能力を実測した値ではない。
2026-09-09ユーザー指示で、速度・旋回速度の超過は停止条件から外した。
互換のためmax_speed_mps/max_yaw_rate_rpsという名前は維持するが、警告閾値としてのみ使う。
超過した有効入力もクリップせず積分し、各採用標本についてROS warningログへ
`LOCAL_ODOMETRY_PROFILE_WARNING` のJSONを出力する。
stamp_ns、epoch、vx_mps、vy_mps、yaw_rate_rps、超過項目、閾値、
`action=INTEGRATED_UNCLIPPED`を記録する。通常起動では既存autoware.log/ROSログへ保存される。
NaN/Inf、時刻逆転・競合・欠損、計算結果の非有限は従来同様に無効化する。
旧run20の停止結果と下記の旧残課題は履歴として保持する。
この変更は異常な入力値によるpose誤差を補正するものではない。
同時刻同値は再publishしない。同時刻競合、非有限、stamp逆転、gap超過はfaultを保持。
明示clock巻戻りで局所epochを更新し原点reset。V4 session側は既存どおりclock resetで終了する。
欠落中の外挿・古いposeの再publishはしない。利用側はTF/poseの時刻と期限を必ず検査する。

## V4接続

通常make devの直接node起動では、JSONの `start_local_odometry: true` により
別名ROS nodeを同じ親processのexecutorへ追加する。モデルworkerとは分離される。
独立起動する場合のみ `local_odometry:=true` launchを使用し、JSONの起動flagはfalseにする。
両方を同時に有効化して二重起動しない。
V4 JSONには以下を設定する。

```json
{
  "start_local_odometry": true,
  "pose_frame": "v4_odom",
  "pose_child_frame": "v4_base_link",
  "pose_evidence": "VelocityReport-only SE2 integration; local origin; v4_base_link aliases physical input base_link axes, no global localization"
}
```

`topics.odometry=/v4/local_odometry`、`expected_nodes.odometry=/v4_local_odometry`。
その他の既存session/model/source/command契約は維持する。
この版のshadow joinは局所poseをまだ必須とする。GNSS/IMU由来のpose依存は外れるが、
完全なpose不要のraw推論分離まで完了したとは扱わない。
PPの既存Reference走行は別経路であり、今回のPP走行をE2E部門適合走行とは呼ばない。

## 再現・検証

Windows commit → 既定CheckOnly/sync → WSL共有lockで実行。

```bash
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_local_odometry_v4.py tests/test_shadow_observation_join_v4.py tests/test_v4_shadow_package.py
```

実ROS起動例（選択済みsimulator用configが必要）:

```bash
ros2 launch aic_e2e_runtime v4_shadow.launch.py config_file:=/v4/live.json
```

試験では前回同様の専有make dev wrapper、PP単一制御、V4非制御、1周・300wall秒上限を使用する。
AWSIM本体と既存dirty checkoutは変更しない。停止/故障を成功扱いせずログに残す。

## 2026-09-09 実装・試験結果

最終実行commit `103ce346909ec378868821b877fec43ae8dc1d27`。
Windows commit、既定CheckOnly/同期、WSL共有lock付き検証を実施。
既定同期によるDatasetルート存在確認は実施。Dataset内容は未読。
固定checkpoint読取・実推論はAWSIM試験内で実施した。

### テスト

- 初版の関連tests: 37 passed。
- 最終版全体: **1757 passed, 4 skipped, 51 warnings / 69.38s**。
- skipはOSQP、Draft2020 validator、jsonschema、任意公式Tiny packageの不足。
- JUnit: WSL `runs/v4_local_odometry_103ce34/junit.xml`、
  Windows `tmp/v4_pp_shadow_20/junit.xml`。
- SSH先の固定Humble image、network noneでbuild成功（1 package / 1.28s）。
- 同隔離containerで `tools/check_local_odometry_ros_v4.py` を実行。
  synthetic VelocityReport11件からOdometry11件、x=.685795615m、y=.121254574m。
  合成円弧の解析解と一致。重複再publishなし、親のROS node名remapを継承しない。
  最初はDDS設定mount不足でROS初期化失敗。既存lo設定をread-only mountして再実行した。
  これはAWSIM走行でも実センサ精度検証でもない。

### run19: 起動経路の不一致

`a7e5281` を専有run19へ配布。通常make devはV4 nodeを直接起動し、
追加専用launchは使われなかった。そのためlocal Odometry publisherがなく、
V4が `PUBLISHER_COUNT:odometry` で終了。V4経路0件。
異常を読み取り確認した後、今回所有run.pyへSIGTERMを送り、所有AWSIMを停止。
raw result `OUTER_SIGNAL_15`、後続INPUT_STALEを保全。wall57.0012秒。
この失敗を受け、JSONで別名local nodeを同じ親executorへ登録する方式へ修正した。
startup例外もSESSION_ENDに記録する処理を追加した。

### run20: 局所Odometryと走行中V4接続は成立、途中停止

source.tar SHA256:
`753e1d776ae37a5cb399048f837babee9cb0ea6a0b3f43b5a68cbe6b35e330b4`。
Windows/remoteで一致を確認。remote既存dirty checkout・AWSIM・PP設定は未変更。

```bash
timeout --signal=TERM --kill-after=15s 300s python3 /home/graneple/e2e_autonomous/v4_pp_shadow_20/run.py
# wrapper内
make dev CONTROL_METHOD=pure_pursuit CAPTURE=false ROSBAG=false AWSIM_LAPS=1 \
  RUN_ID=v4_pp_shadow_20 OUTPUT_HOST_ROOT=/home/graneple/e2e_autonomous/v4_pp_shadow_20/evidence
```

- `/v4/local_odometry`: publisherは `/v4_local_odometry` 1件、V4購読を実graphで確認。
  RELIABLE/VOLATILE/depth10。既存EKF poseではなく、このlocal poseがJOIN_READYへ入った。
- 局所Odometry **972件**、source時刻.035〜34.020sim秒。
  frame v4_odom / child v4_base_link。TFは同じposeで送信するコード経路。
  今回TF message自体の全件記録・tf2 lookup試験は未実施。
- V4 FORWARD_STARTED **256件**、PLAN **256件**。
  source .205〜34.015sim秒。V4は非制御shadow、PPが既存Referenceで走行。
- camera受信→join平均 **28.53ms**、推論呼出し平均 **34.58ms**。
  実効PLAN更新 **7.62Hz**、最大wall間隔 **.628秒**。
  run17は110.51ms / 3.68Hzだが、今回は一周ではなく短区間・異なるpose源。
  同一条件での改善率や完走品質を断定しない。
- JOIN_REJECTED: CAMERA_GRID_TOLERANCE68、JOIN_DEADLINE8、SUPERSEDED_BY_READY3。
  既存100ms grid/40ms許容差は維持している。
- Start/Ready/Start stateを記録したが、**Finishなし・完走未達**。
  最大車速4.5386m/s。約34sim秒で `LOCAL_ODOMETRY_FAULT VELOCITY_OUT_OF_PROFILE`。
  coreの水平速度10m/sまたはyaw rate2rad/sの検査で停止。
  元のvx/vy/wzはfaultログに保存していないため、超過fieldと値はUNKNOWN。
  前後速度の記録だけでは横速度・yawの超過を断定できない。
- local pose停止後、observerがINPUT_STALEを検出し所有AWSIMをfreeze/KILL。
  wall64.3923秒、raw result FAILED/OBSERVER_EXIT、残存ownedなし。
  終了時のKeyboardInterrupt/二重rcl_shutdown例外もログに残る。一次停止原因と区別する。

run19/20とも独立した停止後確認で稼働container、当該project container、ROS/AWSIM processなし。
旧budget/used・失敗ログは保持し、run20へ履歴を引き継いだ。各runの保守予約forward10000と
実forward0/256は区別する。実ログはWindows `tmp/v4_pp_shadow_19/`、`tmp/v4_pp_shadow_20/`、
remote `/home/graneple/e2e_autonomous/` 配下同名runに保存。自動pushなし。

### 残課題

1. 超過した速度field・値・stampを記録して入力異常かprofile設定かを切り分ける。
   現状は任意の閾値引上げ・クリップ・Euler wrap補正を行っていない。
2. 短区間での相対変換誤差・累積driftを評価する。今回の停止はdrift過大の証拠ではなく、
   これだけを根拠にLiDAR SLAMが必要とは判断しない。
3. 走行制御は既存PP/EKF経路であり、E2E部門適合走行・V4制御での完走ではない。
   V4本体へのGNSS/IMU pose依存を外したことと、試験全体のセンサ依存を混同しない。

### 速度profileを警告へ変更した版の検証

ユーザー指示により `add974c55a81793ce4ff01e168acf463f55634c0` で超過停止を削除。
非有限・時刻・送信元等の検査、既存host/PP側の監視は変更していない。
WSL同期の固定Dataset root存在確認を実施、実Dataset内容は未読。

```bash
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q --junitxml=runs/local_odom_warning_add974c/junit.xml
```

**1760 passed, 4 skipped, 51 warnings / 71.23s**。
前後/横速度・正負yaw超過で停止せず解析解どおり積分すること、記録field、
閾値等号、reset、NaN/Inf・時刻異常の拒否を確認した。
JUnitは上記WSL path。SSH simulator hostへの再配布・実ROS警告ログ検証・AWSIM再試験は未実施。
超過値を採用するため、入力スパイク等で局所poseがずれる可能性は残る。
