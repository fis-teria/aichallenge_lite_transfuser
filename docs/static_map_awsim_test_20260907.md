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

## AWSIM実行結果

実行版 `bfd72633c434bc3feabb1f9440c6fd5aa01564be`、限定合成47 passed / 6.20s。
実hostの新規root `/home/graneple/e2e_autonomous/static_map_awsim_20260907/`、
source `source_bfd7263`、attempt `stationary_bfd7263_01`。
2026-09-07 11:06:03 JSTに新期限を11:36:03 JSTへ一度だけ設定し、旧期限と累積量を保存。
`make dev V4_MAP_CHECK=...`で既存AWSIMを起動、wall27.280600秒で終了。
source tar SHA256 `06d170dc5177244c4fb5adff68ee5bdb03c0884dafb459a81533f0216e1d1fe3`、
Windows→host転送前後一致。

| 項目 | 実測 |
|---|---|
| map比較 | 10組、全件pose/scan header同時刻（0.05～0.50sim秒） |
| base map XY | 約[89631.019,43128.029]m |
| yaw | 約2.12651rad |
| base位置の地図分類 | 全件FREE（車体全体の空き保証ではない） |
| 有効反射点 | 679点×9、680点×1 |
| official占有セルhit | 各328点 |
| official空きセルhit | 351～352点 |
| 0.2m近傍まで含む占有hit | 328～329点、48.24～48.45% |
| final_ver3で同じ保存点を再照合 | 初回331/679点が占有、348点が空き |
| 新規V4 forward | 40、MPC solve0、PP送信0 |
| HOLD/STOP制御publish | 156回、正加速度0回 |
| 最大観測速度 | 0.0001560971m/s |
| 駆動回数 | 0、初めから静止 |

比較地図final_ver3は既存configが指す版。再照合でmap/pose/hitをfitしたり、
採用地図を変更したりしていない。上記割合は**占有セル一致率という診断値**であり、
安全性・測位精度・物体認識精度ではない。

保存図で反射点列がコース壁の方向に沿う一方、特に片側で境界とのずれを確認。
MGRS square変換仮説の大域的な向き/場所は整合するが、壁形状・センサ外部パラメータ・
地図の生成元/時点・反射対象の寄与は未分離。原因を地図だけに断定しない。
地図のFREEへ実反射点が多数入る状態なので、空きセルを実際の通過可能域として
監視へ昇格できない。誤差に合わせて余裕を縮小したり、未知域をfreeにしたりしていない。
`geometry_binding_verified=false`、runtime_permission=falseを維持。

V4処理では35参照受理、5初期state制限拒否、21pose join期限切れ、14件が後結合以降へ到達。
200ms期限と周辺監視の未成立も残る。これを隠して走行成功とはしない。

host/runtime errorなし、host watchdog armed、cleanup errorなし。
所有simをfreeze→KILL、runtime終了、unpauseなし。停止確認は初めから静止の確認であり
移動後ブレーキ成功ではない。AWSIM主要asset/元起動scriptの前後hash一致。
元dirty porcelain hashは前後とも
`0b9af671098d0d60fd59457c6aba36e75c88b17ff6a04c1045622ea5617eb9a3`。
終了確認docker psは空。他者process停止・AWSIM変更なし。

## 保存と再現

Windows `tmp/static_map_awsim_20260907/` に実行/validationログ、worker/supervisorログ・summary、
host.log/summary、instance inspect、解決config、承認履歴、budget_afterを保存。
`results_comparison/summary.json` と `map_scan_overlay.png` は保存ログのみから生成。
後処理版 `d44d81d0f24201085a6ca2df6736926649ec43d4`（その前の95b684fで主地図のみ生成も保全）。

```bash
bash tools/with_wsl_training_lock.sh .venv/bin/python tools/summarize_static_map_awsim_v4.py \
 --input /home/thistle/e2e_autonomous/static_map_awsim_20260907 \
 --map-yaml /home/thistle/e2e_autonomous/static_course_map_20260907/official/occupancy_grid_map.yaml \
 --comparison-map-yaml /home/thistle/e2e_autonomous/static_course_map_20260907/final_ver3/occupancy_grid_map.yaml \
 --output NEW_ABSOLUTE_RESULT_DIRECTORY
```

supervisor.jsonl SHA256 `b07508375f4a987e464d4df441a310c6c61c0b493c5b05427eb0c5866a6d8bd8`。
results_comparison/summary.json SHA256 `2da77e3c5d18d952880dfb73857a184d863a5898acfcccf63fbe7fc41f443317`。
図は診断図であり、AWSIM画面録画ではない。今回のmake devは既存Xvfb版。

累積使用：wall1592.946576/3600s、V4 forward204、Tiny5711、共通5915/7100、
MPC1、snapshot11、powered7/11、sim269.989995/320s、log95085805bytes、active=null。
追加駆動4回は未消費、今回の新規駆動予約もなし。期限は自動延長しない。
既定同期によるDataset固定root存在確認を実施、Dataset内容/既存raw読取なし。
新規推論/新sim sensor読取は上記承認内で実施し、学習なし。

## 未完了

AWSIMの停止中地図照合試験は完了。走行中V4更新→PP追従は未完了。
次は固定map生成元と現scene、LiDAR外部パラメータ、MGRS配置を個別に突合し、
壁ずれの出所を確定する必要がある。次いで現在車体/停止までの掃引領域と
地図未掲載物体を扱う監視、pose join期限内処理を整える。未成立のまま駆動4回を反復しない。
