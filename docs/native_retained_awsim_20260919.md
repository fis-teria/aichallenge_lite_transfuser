# 旧出力保持モデルのAWSIM回避試験（2026-09-19）

**選定候補 `retained_step0512` はコーン・箱とも接触し、非接触回避は不合格。両試験とも未完走。**
オフライン保持基準の通過と前方教師ADEの11.2%改善は、今回の実走回避成功にはつながらなかった。
運用モデルは変更せず、専用deploymentでユーザー指定のAWSIM試験を実施した。

| 条件（各1試行） | 回避・通過 | 終了 | 記録最高速度 |
|---|---|---|---:|
| 単独コーン、上限10km/h | 正面接近後に接触、通過できず | `FAILED / CONTROL_OVERSPEED_OR_REVERSE` | 9.629km/h |
| 単独箱、上限10km/h | 正面接近後に接触、通過できず | `STOPPED_NO_LAP / PROGRESS_STALLED` | 9.631km/h |

動画で障害物が車両前部に達し、速度低下と`WALL`表示が出ることを確認した。
速度急変だけを接触判定には用いていない。両方とも周回未完了で公式結果JSONは生成されておらず、
公式ペナルティ件数を0とは扱わない。接触時刻は動画とログの完全同期によるground truthではない。

## 直前の予測とボトルネック

最初の制御異常の約0.10～0.14秒前に発行された通常追従指令と、その指令の元となった生予測を照合した。
XYは**観測時点のbase_link座標、単位m**。下表の横ずれは生予測の前方Xに対するYを線形補間した値で、
障害物との距離・車体クリアランス・実際の横移動量ではない。

| 観測 | コーン | 箱 |
|---|---:|---:|
| 最初の制御異常（走行許可から） | 9.255秒 | 9.090秒 |
| 比較指令の異常までの時間 | 0.105秒 | 0.135秒 |
| 生予測：前方1mのY | -0.0154m | -0.0230m |
| 生予測：前方2mのY | -0.0129m | -0.0336m |
| 生予測：前方3mのY | -0.0265m | -0.0716m |
| 生予測：3秒先終端(X,Y) | (7.684, -0.325)m | (7.479, -0.548)m |
| 指令時の車速 | 9.583km/h | 9.580km/h |
| 目標速度 | 10km/h | 10km/h |
| 加速度指令 | +0.463m/s² | +0.467m/s² |
| 操舵入力指令 | -0.0218rad | -0.0478rad |

遠方の予測点には曲がりがあるが、近距離1～3mでは横への変位が小さく、映像でも障害物を正面に捉えたまま接近した。
今回の観測では、**近距離に必要な回避幅を早めに作る軌道が出ていない**ことが主要な未解決点。
終端Yだけを大きくしても、手前の衝突を避ける軌道になるとは限らない。
ただし、この試験だけでは学習データ・モデル・制御追従の寄与を定量的に分離したことにはならない。

コーンの異常は、車速観測に **-0.097596m/s** が記録され、既存の下限-0.03m/sを下回ったことと整合する。
接触直前までの最大速度は約9.63km/hで、単に速度上限を超えていたという意味ではない。
異常後はブレーキ指令を出して試験を終了したが、コーン側は通常の停止確認フラグが成立する前に終了しており、
その点を停止成功とは扱わない。動画では速度0表示、記録末尾では速度0m/sを確認し、所有コンテナは終了済み。

箱は車速が9.580→4.793km/hへ0.05秒で低下した時点で`MOTION_REAR_LATERAL_INVALID`が発生し、
その後に向き・経路の異常が続いた。走行許可後15.900秒に進行停滞によるブレーキへ移り、停止確認が成立した。
接触後の停止であり、障害物手前での予測停止・回避不能時停止に成功したとは扱わない。

## 試験条件・実行の検証

- 実行先: `graneple@192.168.3.10`、RTX 4060 Laptop、display `:1`。
- 専用deployment: `/home/graneple/e2e_autonomous/time_native_retained_awsim_20260919`。
- checkpoint: `retained_step0512.pt`、SHA256 `e7afdab5d05873de0dbed454e1086b32f4d4f1b67f884a9417a349d7db726ad2`。
- 実行ソース: Windows commit `d12bdba09a1be6d0a91a28e18c9127853dfbefa0`。学習ソースは`8a92537`、間の変更は結果文書のみ。
- 設定は`time_path_dev.json`から候補SHAとcheckpoint epoch=1のみを変えた専用生成物。起動時に最大/コーナー10km/hを指定。
- シナリオ: `time_avoidance_single_cone.yaml` / `time_avoidance_single_box.yaml`。既存の同一位置・通常自車開始位置、衝突有効。
- E2E＋既存Pure Pursuit。NPC/背景車0、SLAM/MPPI回避補正なし。障害物配置はモデル/制御入力に与えていない。
- 近接監視は前回比較と同じ`log_only_awsim_v1`。障害物手前での自動停止を検証する設定ではない。
  センサ鮮度、異常出力・車両状態、停滞、有限時間、停止・cleanupは既存実装を維持した。
- 各試験はtimeout710秒＋kill猶予10秒。実行ラッパー実測はコーン59.94秒、箱65.39秒で終了。
- Windows上のcommitからソースarchiveを作成し、checkpointと共に転送SHAを照合。公式ROS環境で専用installをビルド。
  ソースとinstallのPython258ファイル一致、モデル接続smokeとlaunch smokeが成功した。
- 両走行のROSグラフで制御指令publisherが`time_path_controller`単独、制御購読がclock/scan/plan/steering/velocityだけであることを確認。
  GNSS/IMU/EKFを制御入力に追加していない。通常RVizの生予測経路購読も確認。
- 同一コードのnative WSL全体テストは前段で3,296 passed / 4 skipped。今回の変更は試験用生成物と結果文書であり、
  制御・学習実装の追加変更はない。今回別途実行した公式ROSのビルド・smokeとAWSIM実走は上記の結果。
- 各試験前後のAWSIM資産・既存保護対象・リモートrepo差分のhashが一致。cleanupエラー0、終了後の実行中Dockerコンテナ0。

## 証跡・再実行

WSLの全証跡: `/home/thistle/e2e_autonomous/runs/time_native_retained_awsim_20260919/`。
コーン80ファイル/49,277,943 bytes、箱81ファイル/52,733,994 bytesを転送後SHA256検証。
AWSIM/RViz動画4本は全フレームのデコードに成功。AWSIMはコーン25.8秒/258フレーム、箱33.5秒/335フレーム。
動画冒頭は試験の初期化を含むため、動画秒と走行許可後秒は一致しない。

- [コーン動画](../tmp/native_retained_awsim_20260919/media/cone_awsim.mp4)、[RViz](../tmp/native_retained_awsim_20260919/media/cone_rviz.mp4)
- [箱動画](../tmp/native_retained_awsim_20260919/media/box_awsim.mp4)、[RViz](../tmp/native_retained_awsim_20260919/media/box_rviz.mp4)
- [コーン接触画像](evidence/native_retained_awsim_20260919/cone/contact_detail.jpg)、[箱接触画像](evidence/native_retained_awsim_20260919/box/contact_detail.jpg)
- [生予測の集計](evidence/native_retained_awsim_20260919/encounter_analysis.json)、[準備・実行スクリプトと証跡](evidence/native_retained_awsim_20260919/)

実行したコマンド（既存出力は排他的作成で保護。再試験時は新しいdeployment/run IDを指定する）:

```bash
# PC10: 専用source・checkpointの照合、build、smoke
python3 /home/graneple/e2e_autonomous/time_native_retained_awsim_20260919/prepare_remote.py
python3 /home/graneple/e2e_autonomous/time_native_retained_awsim_20260919/run_trial.py cone
python3 /home/graneple/e2e_autonomous/time_native_retained_awsim_20260919/run_trial.py box
# 各runner内はtimeout710秒、ros-launch、max/corner10km/h、record-video。

# WSL: 転送後、native checkoutのlock下で解析
cd /home/thistle/e2e_autonomous/native_bn_validation_20260919
tools/with_wsl_training_lock.sh .venv/bin/python \
  /home/thistle/e2e_autonomous/runs/time_native_retained_awsim_20260919/analyze_encounter.py
```

次の切り分けは、接触前の観測と教師の回避開始位置を照合し、前方1～3mの軌道と車体幅を含む通過余裕を評価すること。
その上で旧出力保持を残し、近距離の回避教師への適合を改善する。今回の2条件各1試行から未学習配置の成功率や、
旧モデルに対する実走改善率は算出しない。追加学習・追加AWSIM試行はこの試験には含めていない。
