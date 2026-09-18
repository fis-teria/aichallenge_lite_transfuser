# 新モデルepoch 3・10km/hのAWSIM障害物シナリオ試験

**コーンは接触後に停止して未完走。箱は接触を伴って1周完走した。両シナリオとも非接触回避は不合格。**

| 条件（各1試行） | 完走 | 接触の証拠 | 終了理由 | 記録最高速度 |
|---|---|---|---|---:|
| コーン1個 | なし | 車両左前部へコーンが接近する動画、WALL表示、速度急落 | `PROGRESS_STALLED` | 9.632km/h |
| 箱1個 | 1周、142.7687秒 | 車両左側への接近動画、WALL表示、公式wallペナルティ1件 | `JUDGE_FIRST_LAP` | 9.997km/h |

箱はcrash 0 / wall 1 / over 0、wallイベント長5.150秒。コーンは未完走のため公式結果JSONが生成されず、ペナルティ数を0とは扱わない。箱の完走を障害物回避成功とは扱わない。

## 条件とモデル

- 実行先: `graneple@192.168.3.10`。通常のRVizで`/visualization/time_path/raw_path`を表示。
- 学習: `time_native_obstacle10_replay_20260918/training/epoch_03.pt`。旧障害物338窓＋10km/h教師104窓を追加した3epoch目。
- checkpoint SHA256: `9626c63922b152373fc590022ea458dc1f576de4ed3064e497f6e17c4951e4ca`。
- 実行ソース: Windows commit `b978e939d0c82a13265bd41d18b47af420b85636`。
- シナリオ: `time_avoidance_single_cone.yaml` / `time_avoidance_single_box.yaml`。同じ既存配置、通常の自車開始位置。
- 最大速度10km/h、コーナー上限10km/h、車速・曲率による既存減速は有効。固定速度を強制しない。
- E2E＋既存Pure Pursuit。SLAM/MPPI回避補正なし、NPC/背景車なし。
- 近接監視は従来の`log_only_awsim_v1`。センサ鮮度、異常出力、車両状態、進行停滞、有限時間の監視は維持。
- オフライン保持判定は`KEEP_EXISTING_MODEL`。今回はユーザーが明示した新モデルの専用試験で、標準モデルへの昇格ではない。

ROS接続smoke、launch smokeをPC10コンテナで通過。走行中のROSグラフ監査で両試験とも、制御指令publisherが`time_path_controller`単独であること、制御購読がclock/scan/plan/steering/velocityのみであること、通常RVizがE2E経路を購読していることを確認した。制御オドメトリは車速・操舵からの局所推定で、GNSS/IMU/EKFを制御入力に追加していない。

## 接触付近の確認

| 観測 | コーン | 箱 |
|---|---:|---:|
| 最大速度急落（km/h） | 9.554 → 5.000 | 9.551 → 5.000 |
| 上記の制御ログ間隔 | 0.060秒 | 0.055秒 |
| 急落時刻（制御許可から） | 10.050秒 | 9.750秒 |
| 約0.1秒前の目標速度 | 10km/h | 10km/h |
| 同時点の加速指令 | +0.495m/s² | +0.498m/s² |
| 同時点の3秒予測終端（自車座標、m） | (7.947, -0.098) | (7.937, +0.118) |

速度急落のみを接触検知として使用していない。動画25～27秒付近に障害物接近とWALL表示があり、箱では公式wall判定も一致する。動画時刻とログmonotonic由来の時刻は録画開始遅延を含むため、完全に同一の接触timestampとは主張しない。

両方とも接触直前はほぼ直進の予測で、減速指令による停止ではなかった。コーンでは速度急落の後に`MOTION_YAW_RATE_INVALID`（許可後10.20秒）、`TIME_PATH_INITIAL_DIRECTION`（10.35秒）、`STEERING_FEASIBLE_LOOKAHEAD_MISSING`（11.20秒）が現れ、その後16.25秒から進行停滞による停止指令になった。これらの事後監視を最初の速度急落の原因とは扱わない。

これは予測が必要な回避幅を確保できていない可能性を示すが、1条件1試行で学習方法とデータ不足の寄与までは確定できない。旧モデルとの同速度A/Bは今回実施していない。選別時に判明した「前方6m以内・横±1m以内の新規採用0窓」という不足も残る。

## 保存・検証

WSL: `/home/thistle/e2e_autonomous/runs/time_native10_awsim_20260919/`

- `cone/`, `box/`: 試験ログ、設定、通常RVizとAWSIMの動画、シナリオ起動証跡。
- `cone_analysis.json`, `box_analysis.json`: 制御ログと予測の集計。
- `cone_media/`, `box_media/`, `*_detail_media/`: 全フレームデコード確認と接触付近の画像。
- コーン82ファイル/53,153,825 bytes、箱88ファイル/242,976,251 bytesをSHA256照合して転送。
- コーンAWSIM動画333 frames/33.3秒、箱1716 frames/171.6秒。RViz動画も全フレーム正常デコード。
- 各試験前後でAWSIM資産・既存の保護対象ファイルのハッシュ一致。試験所有コンテナの終了処理にエラーなし。他タスクのコンテナは操作しない。
- 初回の配布に不足したschemas/テストfixtureを同一commitから補い、ビルド・smoke成功後に実走した。実走中のソース変更なし。

Windows動画（Git対象外）:

- [コーン](../tmp/native_obstacle10_awsim_20260919/evidence/cone_media/awsim.mp4)
- [箱](../tmp/native_obstacle10_awsim_20260919/evidence/box_media/awsim.mp4)

[コーン接触付近](evidence/native_obstacle10_awsim_20260919/cone/contact_detail.jpg)、
[箱接触付近](evidence/native_obstacle10_awsim_20260919/box/contact_detail.jpg)、
[箱公式結果](evidence/native_obstacle10_awsim_20260919/box/official_result.json)。

## 実行コマンド

実行したコマンド。既存結果は排他的作成で保護されているため、再試験では新しいdeployment/run IDを用いる。

```bash
# PC10: 専用配布・ハッシュ照合・smoke後
python3 /home/graneple/e2e_autonomous/time_native10_awsim_20260919/run_trial.py cone
python3 /home/graneple/e2e_autonomous/time_native10_awsim_20260919/run_trial.py box
# 各コマンド内はtimeout 710s + kill grace 10s、最大/コーナー10km/h、record-video、ros-launch。

# WSL: native checkoutのworktree lock下で結果集計
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser_lidar_v2x_margin
tools/with_wsl_training_lock.sh .venv/bin/python \
  /home/thistle/e2e_autonomous/runs/time_native10_awsim_20260919/summarize_run.py cone
tools/with_wsl_training_lock.sh .venv/bin/python \
  /home/thistle/e2e_autonomous/runs/time_native10_awsim_20260919/summarize_run.py box
```
