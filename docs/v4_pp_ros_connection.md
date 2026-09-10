# V4 → 既存Pure PursuitのROS接続

## 完了範囲

V4の同一forwardレコード → 共通参照Adapter → 現在状態の検査 →
`autoware_auto_planning_msgs/Trajectory` → **既存simple_pure_pursuit実行物** →
shadow専用Ackermann指令を接続した。

今回の検証は、SSH先の公式ROS Humble環境・既存PP実行物を用いた合成ROS試験。
モデル推論、AWSIM起動、実センサ収集、車両command送信は行っていない。
V4によるAWSIM実走行・実制動停止はNOT_RUN。

## 実装とtopic

|役割|実装・topic|
|---|---|
|純粋接続ロジック|`control/v4_pp_connection.py`|
|ROS状態組み立て・参照出力|`v4_pp_connection_node.py`|
|V4入力|`/shadow/v4/plan_record`、std_msgs/String（既存PLANをJSON搬送）|
|現在pose|`/cartographer_v4/extrapolated_pose`、PoseStamped|
|車速・操舵|`/vehicle/status/velocity_status`、`/vehicle/status/steering_status`|
|PP専用参照|`/shadow/v4/pp/trajectory`、Trajectory|
|PP専用状態|`/shadow/v4/pp/odometry`、Odometry|
|既存PP出力|`/shadow/v4/pp/output/control_cmd`|
|状態と元plan ID/期限|`/shadow/v4/pp/status`|

起動: `launch/v4_pp_shadow_connection.launch.py`。
既存PPを専用namespaceで起動し、override、MPC horizon、外部目標速度、recoveryを無効化。
基準routeの購読は接続しない。絶対名のdebug出力もshadowへremap。
車両commandへのremap引数、engage、制御権限取得を追加していない。

## 座標・時刻・速度

- V4の20点はVehicle root基準の元XYを保持する。観測時刻poseで固定局所座標へ変換。
- 現在SLAMのLiDAR poseからforward offset 1.1649999618530273 mを引いてVehicle rootへ。
- 既存PPコード `computeLateralCommand()` はposeからwheelbase/2を引く。
  したがってPPへ渡すposeを軸間中央へ変換する。
  wheelbase=1.087 m、rear-in-root=-0.484 mなら、rootより前方0.0595 m。
  base_link、Vehicle root、後輪中心を同一視しない。
- 現在pose/車速/操舵のstamp差≤0.05秒、age≤0.15秒、future≤0.02秒。
  時刻は明示的な `/clock`。ROSノード自身はwall timerで監視し、sim pauseでも失効可能。
- 原本レコードのaccepted=false/expiry=source stampを改変しない。
  本接続は**別のshadow試験方針**としてsource stamp+0.5秒を設定する。
  原本のRUN許可を延命したことにはならず、実制御へ転用不可。
- 速度はモデル予測ではなく明示試験方針。曲率と残長を使って上限を下げ、終端速度0。
  configの `fixture_only=true` は影響のないshadow計算専用。
  車体寸法以外の数値は試験方針であり、現AWSIMの校正済み実効能力ではない。
- 状態不足・異常、入力源違い、失効、時計巻戻りでは空Trajectory。
  原本の再stampやlast-validの復活はしない。

標準Trajectoryにはplan ID/epoch/expiry欄がないためstatusを併記し、
生成側が毎周期検査する。Adapter停止時は既存PPのwall-time受信timeoutで停止要求。
別ノードによる再配信や悪意ある送信元への耐性まで証明してはいない。

## 検証結果（2026-09-10）

最終ROS実行commit: `772f2f752d3afeb213fc7559b61ee24e112144b7`。
Windows正本branch: `codex/windows-wsl-training-sync`。
PP正本: SSH先racingkart HEAD `4af395eee10f928c7fc7225760adfa04c4c07ff4`
+ 既存dirty checkoutのinstall。既存コード・AWSIMは変更しない。

`docker --network none` で公式環境と既存installをread-only mountした合成試験:

- 実PP command受信131件、追従時の正加速71件。
- 参照停止から1秒以降の負加速停止要求42件、空Trajectory42件。
- 非空Trajectory83件、PPへ渡した軸間中央x=0.0595 m。
- `/control/command/control_cmd` publisher数0。
- AdapterとPPの終了はともにclean。終了後docker psは空。

最終コード版の全pytest: 1797 passed、4 skipped、51 warnings、87.68秒。
限定テストは17 passed。4 skipは既存の未導入依存による。
結果追記commitでは文書とpackage依存宣言のみを追加し、実行ロジックは変更していない。

初回と2回目に終了時例外があり、最終版では修正した。
ROS context終了後のpublish抑止、およびテスト側のSIGINT二重配送を修正。
モデル入力bootstrap経由の不要なtorch importも除去した。
試行ログはSSH専用ディレクトリに保持しており、成功ログだけへの置換はしていない。

## 再現

Windows commit後、既定 `tools/sync_to_wsl.ps1 -CheckOnly` → 通常同期。
Datasetルート存在確認は実施、Dataset内容・raw・sensor・checkpoint読取りは未実施。

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_v4_pp_connection.py tests/test_v4_pp_reference_adapter.py
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
```

公式ROS環境でracingkart installと本package installをsourceして:

```bash
# 本コマンドはネットワーク隔離済みcontainerでのみ合成試験として実行する。
python3 /v4/tools/check_v4_pp_ros_connection.py
# 実入力によるshadow接続時（別途実センサ稼働が必要）:
ros2 launch aic_e2e_runtime v4_pp_shadow_connection.launch.py
```

SSH試験配置: `/home/graneple/e2e_autonomous/v4_pp_connection_25`。
最終ログ: `connection-test-final.log`。
Windowsコピー: `tmp/v4_pp_connection_25_evidence/connection-test-final.log`。

次の段階はAWSIM実入力でのこの専用PP shadow接続と、実車両制約・停止能力の確認。
現在configは低速shadow試験用であり、既存PP走行中の高車速入力は拒否し得る。
shadow指令の存在を、実車両の追従・停止成功や障害物非衝突と呼ばない。
