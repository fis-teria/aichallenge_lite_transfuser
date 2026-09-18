# TimePath + local SLAM MPPI avoidance

TimePathの予測経路を通常追従し、進路上の障害物がある間は局所SLAM地図で
**reference-space MPPI**を実行する。初期版はAWSIM・静止障害物・5 km/h専用。
既存の`time_path_dev.json`とSLAM減速専用プロファイルの既定動作は変更しない。

## 構成

- Cartographer: LiDARと車速・操舵オドメトリによる位置合わせ。GNSS/IMU、事前地図、
  シミュレータ物体座標、V2Xを制御入力に追加しない。
- `slam_obstacle_node --mppi`: 観測と同じ時刻・座標で障害物と回避経路をまとめて配信。
  `/time_path/slam/obstacles`のJSONに`mppi`を追加。制御指令は発行しない。
- `time_trial_controller_node`: 最新モデル経路を従来どおり検査し、正常な場合に限り
  MPPIの経路へ切り替える。MPPI中は通常追従の先読み不足を代替できるが、
  元予測の鮮度・識別・幾何異常は拒否する。scan時刻の車輪オドメトリで現在座標へ移し、PPで追従。
  物理操舵角から既存の操舵応答補償・レート制限へ接続する。
- 独立LiDAR guard: `stop_v1`と`measured_speed_v1`を必須とし、実速度で停止領域を検査。
  既存の1 m制限・記録のみの監視をこのプロファイルに使わない。

通常追従 → 障害物検出 → MPPI回避 → 新鮮なクリア観測0.5秒以上かつ
基準経路への位置・向きの復帰 → 通常追従。回避中の左右の頻繁な切り替えを抑え、
同じ側で通れなければ停止する。停止中も新鮮な観測で再計画する。
地図や経路の欠落・鮮度切れ、計算時間150 ms超過、候補不成立は停止要求。
SLAM停止は0.35秒、元モデルの観測は0.5秒の既存鮮度契約を引き継ぐ。

## 計画の範囲と制限

ROS非依存実装は`src/aic_transfuser_lite/control/slam_mppi.py`。
256候補×3反復で横方向オフセットをサンプルし、コストの指数重みで更新する。
これは局所的なreference-space MPPIで、同梱V45教師C++ノードそのものの再利用ではない。
[MPPIの指数重み更新](https://acdslab.github.io/mppi-generic-website/docs/mppi.html)を
参照し、横方向の回避形状2係数・終端オフセット・終端勾配の計4係数を探索する。

この形状群で候補が成立しない場合は、現在の車体位置・向きから曲率を積分する
`curvature_rollout`へ切り替える。4点の曲率（1/m）を補間し、同じ操舵由来の曲率上限・
車体余裕・未知領域拒否・参照から2.5 m以内の制約で、追加256候補×3反復を評価する。
TimePathの細かな曲率を軌道へ直接コピーせず、追従コストに使用する。
重み付き平均の軌道も再検査する。成立した軌道は追加の開始待機なしで次の制御周期から採用。
一度この方式へ移った回避中は同方式で再計画し、通常追従への復帰時に状態をリセットする。
計画は毎回最新地図で評価し、過去の成功軌道を無検査で再利用しない。
計画時間150 msの上限と独立LiDAR停止ガードは維持する。
この積分は幾何学的な候補生成であり、AWSIMの操舵応答を完全に再現するモデルではない。

オフセットは現在の横ずれ・向きから滑らかに始まり、回避途中の終端を許容する。
元経路への合流はコストで促し、予測終端での合流を強制しない。
経路は最大16 m、元モデルが予測している範囲に限定し、外挿しない。
基準経路からの横ずれ上限2.5 m、操舵角上限0.3 rad、回避速度上限5/3.6 m/s。
全候補と重み付き平均の候補に、曲率と車体寸法を含む占有地図検査を行う。
平均した経路が通れるとは仮定せず、検査済みの候補だけを採用する。
RVizの`/time_path/slam/path`には採用した回避経路を表示する。

未観測セル・地図外は通行不可。静止障害物専用MPPIでは占有地図のTTLを10秒とする。
前方LiDARで観測済みの後方通過領域が、低速・減速中に消えて車体開始姿勢を拒否するのを防ぐ。
最新rayによる上書き・障害物検出は継続し、既定の減速専用モードは従来の2秒を維持。
空間の観測だけでは路面や法的な走行可能領域を判定できないため、
**2.5 mの範囲制限はコース内保証ではない**。走行可能領域の意味情報が必要な場面は別途検証する。
移動物予測は未実装で、NPC/背景車付きの起動をこの初期版では拒否する。
予測経路が短い・遮蔽が大きい・車体余裕が不足する場合は、回避できず停止する。
安全余裕を減らして回避成功に見せない。

停止で予測が縮む場合、回避開始時の参照経路をSLAM座標で保持する。
保持経路の終端との位置・向きの重なりを確認した新予測へ全体を更新し、モデル未予測の先へ外挿しない。
回避で車両の向きが変わった場合は、現在位置から0.5 m以内で始まり、進行方向との差が0.7 rad以内、
長さ3 m以上の新しい予測も採用する。古い終端を通らないことだけでは更新を拒否しない。
異なる予測の継ぎ足しは曲率の段差を作るため行わない。
延長または終端の再確認が10秒なければ失効。最新のモデルパケットとSLAM観測は必須で、
保持した参照から候補を毎回生成し、最新地図で全車体を再検査する。
保持経路の残りが1.5 m未満なら停止し、それ以上でも実速度に対する停止距離を
満たさなければ制御側で停止する。始点の前後誤差は0.3 m以内だけ実車位置へ滑らかにつなぐ。
新旧参照への位置・向きの整合、最新予測長3 m以上、クリア0.5秒を満たして通常追従へ戻る。
開始時に進路を塞いだ観測点も保持し、車体後方0.8 mより後ろへ通過するまで回避を維持する。
車両が横を向いて予測線から箱が外れただけでは解除しない。
回避追従点は1〜2.5 mの範囲から操舵上限を満たす点を選び、単一の近い点だけで可否を決めない。
候補の幾何条件・占有条件の通過数を`candidate_diagnostics`に記録する。
候補不成立・参照不足の初回には、正確な占有地図と計画入力を小さなNPZとして保存する。

## 起動

新ソースを専用deploymentへ配置し、公式ROS環境で`aic_e2e_runtime`を再ビルドする。
既存のCartographer overlayとcheckpointが必要。旧installを上書きせず、
`docs/time_path_make_dev.md`のビルド・所有プロセス管理手順に従う。
以下は**新版を配置・ビルドしたdeployment内**のコマンド。

```bash
make dev DEV_CONTROLLER=time TIME_SLAM_MPPI=1 MAX_SPEED_KMH=5 CORNER_MAX_SPEED_KMH=5

# 物理箱の固定シナリオを指定する有限試験。結果は成功と仮定しない。
python3 tools/run_time_path_awsim_trial.py \
  --deployment /home/graneple/e2e_autonomous/NEW_MPPI_DEPLOYMENT \
  --run-id codex-time-slam-mppi-box01 --display :0 \
  --config configs/control/time_path_slam_mppi.json --slam-obstacles \
  --static-obstacle-scenario configs/scenarios/slam_mppi_single_box.yaml --record-video
```

通常入口へ戻す場合は`TIME_SLAM_MPPI=0`。異なる速度や交通条件を暗黙に採用しない。
この初期版は専用固定速度プロファイルを使い、TimePathの可変速度ROS launchを介さず
既存の有限試験runnerから同じ推論・制御ノードを起動する。

## 検証と未確認事項

```bash
# Windowsでcommitし、tools/sync_to_wsl.ps1で同一commitを同期後
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_slam_mppi.py
```

合成占有地図で箱の回避・基準経路への合流、通路閉塞と未観測領域の拒否、
座標変換、異常・古い入力、速度・起動設定を検証する。理想運動モデルの連続再計画試験は
AWSIMの操舵遅れ・タイヤ・SLAM誤差・未知の障害物運動を再現した試験ではない。

実行中は`slam_obstacles.jsonl`の`mppi`と`control.jsonl`の`details.slam_mppi`、
実際の指令・停止理由を照合する。回避/停止の指令理由を通常追従と区別して記録する。
従来のPP replayは地図とMPPIを再現しないため、この構成は`UNSUPPORTED_SLAM_MPPI`を返す。

公式ROS/AWSIMでの起動と単体箱への接近・停止は下記試験で確認。
回避・周回は未達。配置先の旧source/installは自動的には更新されない。

### 2026-09-18 検証結果

- 今回の変更だけをWindowsの分離checkoutでcommitし、既定syncスクリプトで
  native WSLの専用cloneへ同期した。元の作業中のステージ済み文書整理は保全。
- 検証コードcommit: `3f8a435a43703efb0faf843699595ed3e4328721`。
  `tools/with_wsl_training_lock.sh`下の全体pytestは **3288 passed / 4 skipped**、131.19秒。
  スキップは既存のOSQP、JSON Schema validator、任意の公式パッケージ不足によるもの。
- Windowsの関連テストは **57 passed**。ROS入口を含む変更Pythonファイルの構文も確認。
- 箱の回避から合流までの連続再計画は理想運動モデル上の試験。
  AWSIMへの配置・ROS起動・実際の回避走行成功は、この結果には含めない。

### AWSIM単体箱シナリオと録画（2026-09-18）

- 専用deployment: `/home/graneple/e2e_autonomous/time_slam_mppi_20260918`。
  公式ROS環境でビルドし、source/installのPython 257ファイル一致と
  Cartographer接続smokeを確認。シナリオ追加commitは分離Windows checkoutの`14de88c`。
- 上記runnerを`--display :1 --run-id codex-time-slam-mppi-box01`で実行。
  箱1個、NPCなし、5 km/h上限、公式Start要求あり。最大実測速度4.67 km/h。
- 結果は`STOPPED_NO_LAP`、終了要求`PROGRESS_STALLED`。
  MPPI回避指令は0件。`MPPI_NO_FEASIBLE_PATH`による停止9指令、
  `MPPI_REFERENCE_TOO_SHORT`による停止12指令、その後
  `STEERING_FEASIBLE_LOOKAHEAD_MISSING`が101指令続いた。
  箱手前で停止した映像はあるが、回避成功・周回成功とは扱わない。
- 最初の候補不成立時、現在座標へ変換したTimePath参照終端は前方約3.63 m。
  減速後は約2.59 mへ短くなり、停止後は追従先を選べなくなった。
  短い予測範囲と回避・停止後の再発進の関係は未解決。
  候補不成立の内訳（曲率・未知領域・占有）はこのログだけでは確定しない。
- `artifacts/slam_mppi_awsim_20260918/box01/`へ映像と判定ログを保存。
  RVizは35.0秒、AWSIMは34.5秒、H.264/10 fps/音声なし。
  両動画の全フレームデコード成功、リモートとローカルのSHA-256一致、
  RVizのSLAM地図・点群・経路表示とAWSIMの箱への接近・停止を静止画で確認。
  所有する試験コンテナは終了し、cleanup errorsは0。

### 2026-09-19 部分回避・停止時の参照保持

- 検証コードcommit: `39d566cc968441b6257f90fa07643f321dc1efe2`
  （Windows分離checkoutでcommit、native WSLへ同期）。
  全体pytest: **3297 passed / 4 skipped**、127.02秒。関連42テストも通過。
- 終端の位置と向きを自由にし、最新観測で保持参照から毎回再計画する。
  部分終端、左右非対称通路、短い/ゼロ予測、保持の失効、占有地図の変更、
  経路更新時の曲率、制御への受け渡し、停止距離をテストした。
- 実地の追加原因: 2秒TTLでは通過済み後方セルが失効し、開始姿勢の車体判定円から
  未知領域まで1.0 mしかなく、必要な1.383 mを満たさなかった。
  静止障害物モードだけTTLを10秒に変更し、最新試験では同部の余裕が2.4 mへ改善。
- 予測を継ぎ足すと約1 cmの接続誤差でも曲率が増大したため、重なりを確認後に
  新予測全体へ更新する方式とした。操舵上限0.3 radと車体余裕は維持。
- 専用deployment: `/home/graneple/e2e_autonomous/time_slam_mppi_20260919`。
  最終run: `codex-time-slam-mppi-partial05`。公式ROS再ビルド後、source/installの
  Python 257ファイルを照合。箱1個・NPCなし・5 km/h上限・RViz/AWSIM録画あり。
- 結果: **回避指令21回、箱の通過・合流は未達**。最大速度4.67 km/h。
  回避経路11観測、その後`MPPI_NO_FEASIBLE_PATH`で停止し、
  `PROGRESS_STALLED` / `STOPPED_NO_LAP`で終了。参照不足・始点契約による停止は0。
- 残課題: 箱へ接近した後の状態で、操舵/形状と占有地図の条件を同時に満たす候補がない。
  正確な参照と占有地図を保存し、再評価した。未知領域を自由と仮定する診断や
  4096候補×3反復の診断でも不成立。ただし全ての物理的な回避経路が存在しない証明ではない。
  短い予測範囲で回避を継続できる計画・車両応答との整合は未解決。
- 証拠: `artifacts/slam_mppi_awsim_20260919/partial05/`。
  RViz 35.9秒、AWSIM 35.3秒。全フレームデコード・転送SHA-256照合済み。
  所有試験コンテナ終了、cleanup errorsは0。既定走行への昇格・回避成功宣言はしない。
