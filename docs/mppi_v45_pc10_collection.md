# MPPI V45 / LiDAR V2X の移管とPC10収集

ユーザー依頼: MPPI V45とLiDAR→V2Xをaichallenge_lite_transfuserで管理し、
`graneple@192.168.3.10`へ反映して教師データを収集する。

現在: 採用V45アーカイブのSHA256確認済み。PC10専用overlayをビルド済み。
C++回帰試験は383 tests / 0 errors / 0 failures / 0 skipped。
最初の未達条件: 非走行のROS接続・速度上限検証と収録pilot。

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
停止・後退反復・衝突は記録したうえで不成功として扱い、収集成功と区別する。
PC10用の既存較正とReferenceに合わせシナリオを再生成し、SI26のs座標を流用しない。
