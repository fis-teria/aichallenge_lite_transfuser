# AWSIMに合わせたMPPIの対壁判定

`reference_space_mppi.launch.xml` は通常設定に加えて
`config/awsim_wall_map/footprint.param.yaml` を読み込む。`brain.wall_footprint_xy_m`
はbase_link基準の凸多角形で、現在位置、参照経路、予測状態間の掃引に共通で使う。
空配列を指定した既存設定では、従来の `brain.footprint_*` から矩形を構成する。
対車両の近似に使う既存の `brain.footprint_*` と、対壁の実Collider投影は別の設定である。

現在の33頂点は、配布AWSIMの `GoKart1/Colliders/Collider` の物理MeshColliderを
base_linkへ変換したXY凸包。前端約1.615 m、後端約-0.379 m、左右約±0.768 m。
ペナルティ判定用Triggerではなく物理Colliderから生成している。
3D接触を完全再現するものではなく、ロール・ピッチを除いた2D投影である。

壁地図はMPPIパッケージ内の `config/awsim_wall_map/occupancy_grid_map.yaml` を使う。
`multi_purpose_mpc_ros/env/final_ver3` の共有地図はそのままにし、Unityの
`citycircuit_marge_inner_10` MeshColliderから壁面と路面を抽出して占有を再構成する。
地図原点・解像度は共有地図と同じ。旧地図は路面高の標本選択にのみ使い、
旧地図の占有・不明セルは出力へ継承しない。物理的な路面を塞いでいた旧境界を除き、
実際の壁面と重なるセルを占有にする。一律の座標補正や壁の膨張は行わない。

コースの標高は一定ではないため、単一の世界座標z断面は使わない。
既存地図の壁から1 m以上離れた路面内の、ほぼ水平な面（法線の鉛直成分比>0.98）を
0.5 m区画で最低点優先に抽出する。近傍3点の中央値から局所路面高を求め、
路面上0.15～0.75 mの急斜面（法線の鉛直成分比<0.5）を切り出す。
路面標本から4 m以上離れた面は補正に使わない。これは現配布コースに対する
オフライン生成条件であり、別コース・別AWSIMへの無検証流用はしない。
投影面と接する全セルを占有にするため、解像度0.1 m相当の離散化は残る。

自由領域は、同じ局所路面高から±0.15 m以内のほぼ水平な面を投影し、壁セルを除いた後、
路面標本に4近傍で接続する成分から作る。壁セルを最優先とし、観測した接続路面以外は不明とする。
不明画素値はYAMLのfree/occupied閾値の中間に対応させる（現在147）。
これは物理形状に基づく走行空間であり、競技規則上の走行許可領域を別途表現するものではない。

## 再生成

AWSIM資産は読み取り専用の入力である。出力先はMPPI側に置く。
生成時だけPythonのnumpy、scipy、Pillow、PyYAML、UnityPyを使う。
走行時に追加Python依存やAWSIMファイルへのアクセスは発生しない。

リポジトリルートで、依存を導入したPython環境から実行する。

```bash
python3 aichallenge/workspace/src/aichallenge_submit/reference_space_mppi_planner/scripts/generate_awsim_wall_geometry.py \
  --awsim-level aichallenge/simulator/AWSIM/AWSIM_Data/level1 \
  --mgrs-offset 89637.703125 43503.5 35.400001525878906 \
  --base-map-yaml aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/env/final_ver3/occupancy_grid_map.yaml \
  --output-dir aichallenge/workspace/src/aichallenge_submit/reference_space_mppi_planner/config/awsim_wall_map
```

MGRS offsetはこの配布版のEnvironment設定に対応する値。
`provenance.json` に元資産・元地図・出力のSHA-256、形状、抽出条件を残す。
生成後は全コースの自由・占有・不明領域、記録された接触姿勢、接触前の通常走行位置を確認する。
`test_generate_wall_geometry.py` とC++の `wall_geometry` テストが幾何と資産の検証を担当する。

この変更は対壁の幾何表現を直すもので、参照更新と将来制御の予測差や、接触後の復帰方法は
別の課題として残る。修正後のH2Hによる完走性・走行中の過剰停止は別途確認が必要。
