# TimePathランタイムの保持経路による復帰

現在のSLAM MPPI構成に`retained_time_path_recovery_v1`を追加する。
対象は単一車両・静止障害物の有限AWSIM試験。TimePathの一時的な短縮から、
直前の有効な空間経路を使って低速復帰する。新しい点・時刻は捏造しない。
GNSS/IMU、既知コース、teacher制御を復帰入力にしない。

## 遷移と上限

- NORMAL: 元のE2E/MPPI制御。残り3 m以上・横ずれ0.3 m以下・向き差0.4 rad以下の
  予測をSLAM座標で保持する。実際のモデル観測時刻も保持し、更新時刻で若返らせない。
- RECOVER: 予測の残りが2.5 m未満で0.3秒継続、実測2 m/s以下で検討。
  保持経路は5秒以内、残り2 m以上、横ずれ0.75 m以下、向き差0.6 rad以下。
  毎回最新のSLAM地図で車体・未知領域・操舵を検査してから最大3 km/hで前進。
  独立LiDAR停止と通常のセンサ/指令監視は継続する。
- HAND BACK: 3.5 m以上・横ずれ0.3 m以下・向き差0.3 rad以下の最新予測が
  0.5秒続いたら通常追従へ戻す。
- STOP: 1試行6秒または2.5 m、保持経路10秒、残り1.6 m等の上限で停止。
  1ラン最大2試行。使い切った試行は時計・入力欠落でリセットしない。
  初回の保持経路がない場合は元の通常制御へ委ね、架空の復帰経路を作らない。

入力欠落、NaN、出所・時刻・座標の不一致、大きな形状異常は復帰で隠さない。
停止時の微小な予測揺れについてのみ、有限30点かつ全点0.5 m以内で
`TIME_PATH_BACKTRACK` / `TIME_PATH_UNRESOLVED_EXCURSION`の場合に保持経路を検討する。
この例外では新しい予測点を追従せず、明示的なRECOVERパケットだけが制御を取得できる。
通常追従や通常MPPIへ同じ例外を広げない。

現TimePath出力に意味的な停止理由はないため、短縮だけで停止意図を完全には識別できない。
本機能は静止箱AWSIM用の明示的設定に限定する。停止要求・権限解除は既存controllerの
上位判定が優先される。信号・歩行者・意図的停止を含む一般環境の復帰保証ではない。
車体が大きく逸脱した場合や、未観測の先へ延長が必要な場合は復帰できない。

## 起動と検証

```bash
python3 source/tools/run_time_path_awsim_trial.py \
  --deployment /home/graneple/e2e_autonomous/time_slam_mppi_20260919 \
  --run-id codex-time-slam-mppi-recovery01 --display :1 \
  --config configs/control/time_path_slam_mppi_20_15_15_recovery.json --slam-obstacles \
  --static-obstacle-scenario configs/scenarios/slam_mppi_single_box.yaml --record-video

# 同じ専用deploymentのsource内で、20/15構成は復帰付き設定を選択する。
make dev DEV_CONTROLLER=time MAX_SPEED_KMH=20 CORNER_MAX_SPEED_KMH=15 TIME_SLAM_MPPI=1 TIME_RECORD_VIDEO=1

# Windowsの分離checkoutでcommit・既定sync後、native WSLで検証。
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
```

復帰なし比較用は`time_path_slam_mppi_20_15_15.json`、従来5 km/h設定も維持。
`slam_obstacles.jsonl`の`mppi.recovery`に状態・試行回数・元観測時刻・走行距離、
`control.jsonl`に`SLAM_MPPI_RECOVER:RETAINED_PATH_TRACKING`を記録する。
復帰指令の発行と、実際の前進・通常追従への復帰・周回完了は別々に判定する。

## 2026-09-19 検証結果

最新コード`0e21e9a322817b4099d84c282fbcc51ce0f665e6`はnative WSL全体
3321 passed / 4 skipped（158.48秒）。公式ROSのsource/install 258ファイル一致を確認。
初版のAWSIM2試験は復帰指令0件で停止。箱あり試験はMPPI候補不成立、箱なし試験は
保持条件が4 m以上に限定されており、復帰開始前に保持経路を使い切った。
最新版は3 m台の有効な最新経路も保持し、同じ記録入力の選択器再現では残り2.286 mで
復帰状態へ入る。占有地図・車両応答の再現ではなく、実際の復帰完了の証拠ではない。
最新版のAWSIM試験は共有ホストの別試験使用で起動前に停止しており、復帰実走は未確認。
詳細は`artifacts/slam_mppi_awsim_20260919/recovery_report.md`。

## Git公開とROS2パッケージ

`codex/slam-mppi-recovery`ブランチには、ROS2パッケージに加えて、ビルドに必要な
ルートの`src/`・`configs/`・`schemas/`と起動ツールを含む検証用ソースを公開する。
リポジトリ全体をcheckoutして使用する。`aic_e2e_runtime`単体のコピーではビルドできない。
ROS2のinstallには5 km/h、20/15/15 km/h、復帰付き20/15/15 km/hの3設定を同梱する。
モデル重み、データセット、rosbag、RViz動画はこの変更に追加しない。
試験ログ・動画の`artifacts/`リンクはローカル保存先であり、Git公開物には含まれない。
起動と依存環境の構築は[SLAM MPPI手順](slam_mppi_avoidance.md)を参照する。
