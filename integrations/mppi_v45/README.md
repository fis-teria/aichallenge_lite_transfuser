# MPPI SIM V45 teacher

SI26の採用ソースから必要な6 ROS packageと`final_ver3` Reference/地図資産を取り込んだ。
元ソースは320ファイル、5,288,149 bytes（6 package、地図・設定資産、recovery用Python依存8ファイル）。
収集用修正版 `lidar-motion-intent-r2` は、LiDAR位置履歴の速度推定、採用済み回避の継続、計画停止時の後退抑制を追加した。
元アーカイブと現在の各ファイルのSHA256は`source_manifest.json`。変更ファイルには元の`upstream_sha256`も保存する（追加ファイルはnull）。
衝突判定、車体マージン、センサtimeoutは継続使用する。
元データはSI26に保全し、生成物・学習重み・bag・ビルド出力は本ディレクトリに含めない。

LiDAR変換の正本は `ros2_ws/src/aic_lidar_v2x`。
PC10の旧system launchはMPPI用の速度・プロファイル引数を転送しないので、
`teacher_v45.launch.py` が同梱submit launchへ直接渡す。
AWSIMアダプタ・初期姿勢・Start/Finish処理は公式環境のものを継続使用する。

## ビルド

Linuxの専用deployment sourceから実行する。既存installへの上書きを拒否する。
ソースはread-only、C++ buildは5GiBの一時メモリ領域、install/logは指定runtimeに保存。
実行には公式Autoware workspaceと、既存Dockerイメージが必要。

```bash
python3 tools/build_mppi_v45_overlay.py \
  --awsim-repo /home/graneple/git/autononous_ai/aichallenge-racingkart \
  --runtime /home/graneple/e2e_autonomous/mppi_v45_collection_20260918/runtime
```

`source`の6packageとLiDARモジュールをビルドし、MPPI既存C++回帰試験を実行する。
`runtime-identity.json` はこのPCで再ビルドした実行ファイルのハッシュを記録する。
別ホストでビルドしたバイナリと同じハッシュになるとは仮定しない。

起動にはコンテナ内の公式underlayとこのruntimeの`install/local_setup.bash`を順にsourceする。
Reference資産は実行コンテナのsource/install両方の`multi_purpose_mpc_ros/env/final_ver3`へread-onlyで接続する。
`support`にはrecoveryが依存する採用版のPythonモジュール7個と空の`__init__.py`をそのまま同梱し、
専用教師プロセスのPYTHONPATHへ追加する。Recovery用`config.yaml`も採用版をread-onlyで参照する。PC10 underlayの古いPython依存は置換しない。

```bash
export PYTHONPATH="/source/integrations/mppi_v45/support:${PYTHONPATH:-}"
ros2 launch aic_lidar_v2x teacher_v45.launch.py \
 map_yaml:=/source/integrations/mppi_v45/assets/multi_purpose_mpc_ros/env/final_ver3/occupancy_grid_map.yaml \
 domain_id:=1 speed_cap_mps:=1.3888888888888888 run_rviz:=false
```

収集launchだけが `brain.collection_motion_enabled`、`brain.collection_avoidance_continuation`、
`collection_planned_stop_gate` を有効にする。通常ノードの既定値はfalse。
位置推定は過去0.6sのベクトル傾きの中央値を使い、最新観測時刻へ位置を合わせる。
0.2m/s未満は静止扱い、初期0.2s未満または観測3点未満は速度未確定として0m/s。3km/h移動は回帰試験対象。
既存の新鮮度条件内で、同じ前方障害物への採用済みAVOIDを速度20km/h以下なら継続する。
対象喪失時や別IDへの切り替わりに無期限の保持はしない。候補軌道の衝突検証は従来どおり行う。
基準経路へ戻る軌道の実行中でも、新鮮な障害物が回避条件を満たす場合は回避探索を再開する。
戻り軌道の保持がAVOID探索を無期限に禁止する状態を防ぎ、採用前の衝突検証を維持する。
後退開始の4sタイマーは新鮮なCMA前進速度要求で判定する。計画停止・指令欠落時はタイマーを解除する。
開始済みの復帰・ギア切り替え・停止要求の処理は既存の制御に従う。

WSLでのPython回帰試験:

```bash
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
```

C++と実ノードの回帰試験は上記公式Dockerビルド内で実施する。
保存入力の改善は走行成功とは別に評価し、再収集結果をPC10運用記録へ残す。

これはAWSIM専用。LiDAR由来V2XをReference生成・MPPI・recoveryの3ノードへ渡す。
native V2Xや配置座標をE2Eの入力には追加しない。詳細と最新の収集結果は
[収集教師の修正・実走記録](../../docs/mppi_v45_collection_motion_fix_20260918.md)を参照。

## 有限収集

PC10のScenario Toolで検証したシナリオを指定する。既存runの上書きは拒否する。
以下はcomposeのpreviewのみ。確認後に `--execute` を付けると1回だけ収録する。

```bash
python3 tools/collect_mppi_v45.py \
  --awsim-repo /home/graneple/git/autononous_ai/aichallenge-racingkart \
  --runtime /home/graneple/e2e_autonomous/mppi_v45_collection_20260918/runtime \
  --scenario /absolute/path/to/scenario.yaml \
  --run-id lidar-v45-pc10-example-001 \
  --speed-cap-kmh 5 --wall-timeout-s 480 --run-budget-gib 1 --free-reserve-gib 2
```

Camera/LiDAR/TF/ego/指令に加え、LiDAR認識状態・教師軌道・停止要求を記録する。
bagは256MiB単位で分割しlossless zstd圧縮する。容量・時間超過はScenario Toolへ
SIGINTを渡し通常終了を要求する。運用時は外側にも有限timeoutを設定する。
一度に1runを実行し、終了後にWSLへSHA256照合付きで移して教師候補を監査する。
rawの存在や正常終了だけで、回避成功・学習投入可能とは判定しない。
手動・予算による中断時も、終了済みrawから `exit_code=130` の結果を残す。
中断・monitor異常終了・欠測・停止要求・後退前後の区間を成功教師に昇格させない。
箱などnative物体は車両V2Xへ現れず、既存監査の車体間距離を測れない。
そのrawは診断用として保全し、物体との距離が別途検証されるまで自動採用しない。
