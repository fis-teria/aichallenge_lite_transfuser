# 基本地図とAWSIMの停止中照合試験

開始版6958d83、Windows clean。ユーザーは10:57以降に旧期限更新を承認。
準備/合成検証後、最初のmap-check予約で一度だけcutoff=その時刻+30分を設定。
駆動4回残、各回制動込み10秒、累積powered11/sim320/wall3600/forward7100は変更しない。
旧台帳/失敗/承認recordは保存。再起動ではdeadlineを更新しない。

今回はまず**停止中**のGNSS/IMU/LiDARを地図に照合する。地図診断は駆動許可ではない。
Unity scene保存metadataのEnvironment1298はMGRS grid54SUE、offsetは
raw struct offset32/36/40のfloat。ROS2Utility.UnityToRosPositionは(z,-x,y)、
GnssSensorはその値へmgrsOffsetPositionを加える。これを静的根拠として参照。
既存map loader configもMGRS。UTM54Nから[300000,3900000]を引く固定square仮説を
実観測で診断する。座標一致や実装成功を監視の完全性へ昇格しない。

新規 `map_observation_v4.py` は地図同座標へ変換した有効LiDAR hitを集計。
正確な占有セルhitと0.2m近傍hitを別々に保存（地図膨張/threshold変更はしない）。
poseはGNSS/IMUからbase位置を計算した既存値。scanは50ms以内の最寄りheader、
deskew未実施。最大10比較に限定。どの返り値もruntime_permission=false。
欠損/地図外/不一致をfreeにしない。既存監視/制御送信経路を変えない。
AWSIM本体/scene/元起動script/元dirty checkoutを変更しない。

## コマンド

```bash
# Windows commit→既定CheckOnly/同期後、WSL lock
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_map_observation_v4.py tests/test_static_course_map_v4.py tests/test_spatial_live_pp_v4.py tests/test_spatial_run_continuation_v4.py
# 専有host、新規source archive / 新規output
make dev V4_PURE_PURSUIT=1 V4_PHASE=stationary V4_FORWARD_LIMIT=40 V4_WALL_SECONDS=60 \
 V4_MAP_CHECK=/home/graneple/git/autononous_ai/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/env/official/occupancy_grid_map.yaml \
 SIM_REPO=/home/graneple/git/autononous_ai/aichallenge-racingkart \
 V4_CHECKPOINT=/home/graneple/e2e_autonomous/spatial_sim_e2e_20260906/fixed_final.pt \
 XVFB_ROOT=/home/graneple/e2e_autonomous/spatial_sim_e2e_20260906/xvfb-root \
 V4_BINDING=/home/graneple/e2e_autonomous/spatial_run_continuation_20260907/binding.json \
 V4_BUDGET=/home/graneple/e2e_autonomous/spatial_run_continuation_20260907/budget.json \
 V4_OUTPUT=NEW_ABSOLUTE_OUTPUT V4_COMMIT=EXACT_COMMIT
```

map-checkはstationary専用。一致/安全根拠が不足したままrunに切替えない。
Dataset rootの既定同期存在確認のみ許可。Dataset内容/rawは使わない。
今回の固定checkpoint/新sim観測は承認範囲。学習、push、他者process停止なし。
