# MPPI SIM V45 teacher

SI26の採用ソースから必要な6 ROS packageと`final_ver3` Reference/地図資産を取り込んだ。
311ファイル、5,155,755 bytes。元アーカイブと各ファイルのSHA256は`source_manifest.json`。
ソースのアルゴリズム・既存安全判定・パラメータは変更していない。
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
Reference資産は実行コンテナの`multi_purpose_mpc_ros/env/final_ver3`へread-onlyで接続する。

```bash
ros2 launch aic_lidar_v2x teacher_v45.launch.py \
  map_yaml:=/source/integrations/mppi_v45/assets/multi_purpose_mpc_ros/env/final_ver3/occupancy_grid_map.yaml \
  domain_id:=1 speed_cap_mps:=1.3888888888888888 run_rviz:=false
```

これはAWSIM専用。LiDAR由来V2XをReference生成・MPPI・recoveryの3ノードへ渡す。
native V2Xや配置座標をE2Eの入力には追加しない。詳細と収集結果は
[PC10運用記録](../../docs/mppi_v45_pc10_collection.md)を参照。

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
