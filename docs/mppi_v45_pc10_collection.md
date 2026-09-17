# MPPI V45 / LiDAR V2X の移管とPC10収集

ユーザー依頼: MPPI V45とLiDAR→V2Xをaichallenge_lite_transfuserで管理し、
`graneple@192.168.3.10`へ反映して教師データを収集する。

現在: 移管、PC10専用overlay、3配置の有限収録、WSLへのSHA256検証付き転送・教師監査を完了。
成功は直線の停止カート1件。コーナー入口のカートと箱は停止・後退反復により未通過。
次の未達条件: この2条件を教師MPPIが通過できること。大量収集や再学習は開始していない。

## 範囲と方針

- 編集/Git正本: Windowsの本リポジトリ。V45の必要な6 ROS packageとReference資産を取り込む。
- LiDAR変換は既存 `ros2_ws/src/aic_lidar_v2x` を継続使用し、重複実装を作らない。
- SI26の元ソース・アーカイブは保全。PC10の既存制御系を上書きせず専用overlayを使用。
- AWSIM本体、操舵・衝突余白・watchdogを変更しない。V45の方策そのものも変更しない。
- LiDAR→V2Xは教師3ノードだけへ接続。native V2Xは評価用。E2Eの入力へ混ぜない。
- Camera/LiDAR/TF/ego/指令/教師軌道/認識状態を記録し、時間基準教師は実測future poseから生成。
- まず停止カート/箱/コーンの少数配置を個別試行。停止や後退反復もrawに保存し、成功と数えない。
- PC10の容量に合わせ1runずつWSLへ転送・SHA256照合。元の無関係なデータは削除しない。
- run/scenario単位のsplitを保ち、学習への統合・再学習はこの収集段階で自動実施しない。

## 検証

1. V45ソースの採用版一致、ビルド、既存C++回帰テスト。
2. ROS graphでLiDAR専用topicへの接続、速度[m/s]、既存マージン、実行identityを確認。
3. 有限時間・容量でAWSIM収録し、終了・公式結果・物体検出・欠測・教師支持区間を監査。
4. Windows sourceをcommitして公式手順でWSLへ同期し、共有lock下でpytestとデータ監査。

権限は依頼された移管・PC10反映・シミュレータ収集に限定。
Git pushは行わず、実車・他環境への展開・既存データ削除は行わない。
専用overlayをsourceしなければ元の実行構成に戻せる。

## 採用版

SI26: `ai-work/runs/2026-09-18_mppi-v45-early-avoidance/artifacts/mppi-sim-v45-submit-20260918.tar.gz`

SHA256: `ad2c932f903c50b20a7dda9ee57bd8a95403a1603c63df80a21a52756de1fde3`

V45は、選択された経路干渉相手が5km/h以下なら既存選択範囲内で5mより手前から
AVOID候補を探索する変更。旧V44のfollowing gapや衝突マージンは維持。
前回のV45走行には後退反復による未通過があり、採用版であることは回避成功の保証ではない。

## PC10の専用配置

`/home/graneple/e2e_autonomous/mppi_v45_collection_20260918/{source,runtime}`。
公式環境の既存installはread-onlyで参照し、専用runtimeを重ねる。
旧system launchがMPPIの制御選択・速度を転送しないため、V45専用launchが
同梱submitへ直接渡す。起動と収集コマンドは `integrations/mppi_v45/README.md`。

初回pilotは5km/h、各シミュレータ360s / wall480s、run容量1GiB、空き2GiBを下限。
箱だけは残容量に合わせrun容量0.5GiB。外側timeoutは540s、終了猶予45s。
停止・後退反復・衝突は記録したうえで不成功として扱い、収集成功と区別する。
PC10用の既存較正とReferenceに合わせシナリオを再生成し、SI26のs座標を流用しない。
`straight_b_s0_center` の実配置probeはPlayStartを確認し正常終了。
PC10の別SLAM試験の実行中は待機し、そのコンテナ・出力には操作しない。
移管時に不足していたrecoveryのPython依存と設定も採用アーカイブから同梱済み。
採用320ファイルはGit archive経由でも原本SHA256と一致する。

## 2026-09-18 収録・監査結果

| run suffix (`lidar-v45-pc10-`) | 配置 | 通過 | Camera観測数 | 前進回避教師の候補数 | 後退区間数 |
|---|---|---|---:|---:|---:|
| cart-b01 | 直線・停止カート | 成功、障害物の先25mへ到達 | 727 | 218 | 0 |
| cart-entry-b01 | コーナー入口・停止カート | 未通過、反復を中断 | 1,423 | 0 | 7 |
| box-b01 | 直線・箱 | 未通過、反復を中断 | 1,375 | 0 | 7 |

- 3runとも公式crash/wall/over countは0、LiDAR入力watchdogの停止要求は0。
  カート入口と箱の中断は、前進・後退を反復して障害物を通過できないための収集監督判断。
- 成功runは約81.9m走行、近似車体間距離の最小0.588m、地図上の車体逸脱0m。
  これは特定配置の通過試験であり、1周完走や全配置の回避性能を示さない。
- 218候補は1イベントから切り出した重複のある時間窓。
  入力画像 `[1,4,3,224,384]`、LiDAR `[1,4,2,750]`、実測future XY `[30,2]` m、dt=0.1s。
  入力欠損、未来不足、後退前後、停止要求、異常終了を除外。
- 箱はAWSIMのspawn logで1個を確認し、記録画像にも存在する。
  配置座標から1m以内のLiDAR表面候補を379件観測したが、物体同一性や形状の確定ではない。
  車両V2Xには現れないため、箱との最小距離は既存の車両監査では未計測。
- run/scenario単位の情報を維持。sector_04/sector_06の収集候補のみで、新しい検証イベントは0。
  学習splitへの登録、既存データとの自動混合、学習実行は行っていない。

## 保存先と再現

WSL: `/home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918/`

- `collected/<run-id>/`: raw、教師identity、シナリオ、ソースprovenance、転送検証書。
- `audit/<run-id>/`: `audit.json`、`anchors.jsonl`、`observed_teachers.npz`。
- 3runの転送対象は計1,221ファイル / 1,144,547,707 bytes。全ファイルSHA256一致。
- PC10の元rawも保全。今回の専用compose containerは全件終了し、残存0件。
- AWSIM本体を含む保護対象7ファイルは実行前後で同一。PC10終了時の空きは約2.53GiB。

監査はWSL native checkoutの共有lock内で実施する。

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser_lidar_v2x_margin
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  tools/audit_lidar_v2x_obstacles.py \
  --collected /home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918/collected/lidar-v45-pc10-cart-b01 \
  --output /home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918/audit_repeat/lidar-v45-pc10-cart-b01
```

出力先は新規ディレクトリを指定し、既存監査を上書きしない。
Windowsの証跡控え: `tmp/pc10_mppi_v45_collection_20260918/collected-summary.json`。

## 検証記録

- PC10 C++: 383 tests / 0 errors / 0 failures / 0 skipped。
- WSL pytest: 3,103 passed / 4 skipped（既存の任意依存・公式package未提供）。
  実行コードcommit `32e12454de877bbb3f234e4e95eeb9b41bf29555`。
- 非走行ROS graph: domain97、通信隔離で通過。実収録domain1でもカート・箱の2runで再確認。
  教師3 consumerのみLiDAR専用topicを参照し、native V2X購読なし。既存6マージンと5km/h上限一致。
- 中断時の診断結果は `exit_code=130` / `not_judged` として残し、教師候補0を確認。
